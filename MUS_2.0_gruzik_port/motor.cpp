#include "motor.h"
#include "robot_config.h"

void LowPassFilter_Init(LowPassFilter_t *LPF, float alpha)
{
    LPF->alpha = alpha;
    LPF->output = 0.0f;
}

float LowPassFilter_Update(LowPassFilter_t *LPF, float input)
{
    LPF->output = LPF->alpha * input + (1.0f - LPF->alpha) * LPF->output;
    return LPF->output;
}

void Motor_Init(motor_t *motor, volatile int32_t* enc_count, float Kp, float Ki)
{
    memset(motor, 0, sizeof(*motor));
    motor->encoder_count = enc_count;
    motor->kp = Kp;
    motor->ki = Ki;
    motor->EncoderDirection = 1;
    motor->PWM_to_MS_Scaler_value = 1.0f;
    
    if (motor->encoder_count) {
        motor->EncoderValue = *(motor->encoder_count);
        motor->PreviousEncoderValue = motor->EncoderValue;
    }
}

void Motor_SetEncoderDirection(motor_t *motor, int8_t direction)
{
    motor->EncoderDirection = direction;
}

static float clampf(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

// Maksymalna sensowna delta enkodera na jeden obieg petli, odpowiadajaca limitowi 3000 RPM
// z zapasem 2x na czas obiegu > 1 ms (14 zliczen/obieg zamiast 7). Liczone w czasie kompilacji:
// 2 * 3000 * 140 * 0.001 / 60 = 14 zliczen/obieg.
static constexpr int32_t MOTOR_MAX_ENCODER_DELTA =
    (int32_t)(2.0f * 3000.0f * ROBOT_ENCODER_COUNTS_PER_WHEEL_REV * ROBOT_CONTROL_DT_S / 60.0f);

void Motor_CalculateSpeed(motor_t *motor)
{
    if (!motor->encoder_count) return;

    motor->EncoderValue = *(motor->encoder_count);
    
    int32_t delta = motor->EncoderValue - motor->PreviousEncoderValue;
    motor->PreviousEncoderValue = motor->EncoderValue;
    
    // Apply direction
    delta *= motor->EncoderDirection;

    // Ochrona przed skokowym bledem odczytu enkodera: przycinamy delte do +/- max
    // (nie zerujemy i nie podmieniamy na poprzednia - zachowuje kierunek i realny ruch,
    // odrzuca tylko fizycznie niemozliwy nadmiar). Ta sama delta idzie do RPM i dystansu.
    if (delta > MOTOR_MAX_ENCODER_DELTA || delta < -MOTOR_MAX_ENCODER_DELTA) {
        delta = (delta > 0) ? MOTOR_MAX_ENCODER_DELTA : -MOTOR_MAX_ENCODER_DELTA;
        motor->glitch_count++;
        // Log ograniczony do max. 1 wpisu na sekunde, zeby nie blokowac petli 1 kHz.
        uint32_t now_ms = millis();
        if (now_ms - motor->glitch_last_print_ms >= 1000u) {
            motor->glitch_last_print_ms = now_ms;
            Serial.printf("[MOTOR] encoder glitch clamped, total=%u\n", (unsigned)motor->glitch_count);
        }
    }

    // Convert to RPM and MetersPerSecond
    // N20 AB Hall encoder: 7 PPR (motor shaft) x 2 A-edges (CHANGE ISR, A-only) x 10 (gear ratio) = 140 CPR (wheel shaft)
    // Matches ROBOT_ENCODER_COUNTS_PER_WHEEL_REV in robot_config.h
    // RPM = (delta / ENCODER_CPR) * (60.0f / dt_s)
    motor->RPM = ((float)delta / ROBOT_ENCODER_COUNTS_PER_WHEEL_REV) * (60.0f / ROBOT_CONTROL_DT_S);

    LowPassFilter_Update(&motor->EncoderRpmFilter, motor->RPM);
    motor->PreviousRPM = motor->EncoderRpmFilter.output;

    motor->MetersPerSecond = (motor->EncoderRpmFilter.output / 60.0f) * (ROBOT_WHEEL_DIAMETER_M * PI);

    LowPassFilter_Update(&motor->MetersPerSecondLPF, motor->MetersPerSecond);
    motor->PreviousMetersSecond = motor->MetersPerSecondLPF.output;
    // True unfiltered distance traveled for Odometry (crucial for correct mapping shape!)
    motor->DistanceInMeasurement = ((float)delta / ROBOT_ENCODER_COUNTS_PER_WHEEL_REV) * ROBOT_WHEEL_CIRCUMFERENCE_M;
    motor->LpfDistanceInMeasurement = motor->MetersPerSecondLPF.output * ROBOT_CONTROL_DT_S;
}

void PI_Loop(motor_t *motor)
{
    motor->current_speed = motor->MetersPerSecondLPF.output;

    motor->error = motor->set_speed - motor->current_speed;
    motor->Error_sum = clampf(motor->Error_sum + motor->error, -10.0f, 10.0f);
    motor->speed = motor->set_speed + (motor->error * motor->kp) + (motor->Error_sum * motor->ki);
}
