#include "line_follower.h"
#include "robot_config.h"

static float clampf(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

static uint16_t readSensorMux(uint8_t sensorIndex) {
    digitalWrite(PIN_MUX_S0, sensorIndex & 0x01);
    digitalWrite(PIN_MUX_S1, (sensorIndex >> 1) & 0x01);
    digitalWrite(PIN_MUX_S2, (sensorIndex >> 2) & 0x01);
    digitalWrite(PIN_MUX_S3, (sensorIndex >> 3) & 0x01);
    delayMicroseconds(5);
    return analogRead(PIN_MUX_SIG);
}

void readSensorsCalibrated(LineFollower_t *LF) {
    for (uint8_t i = 0; i < 14; i++) {
        uint16_t raw = readSensorMux(i);
        if (LF->sensorCalibrated) {
            if (LF->SensorMaxValues[i] > LF->SensorMinValues[i]) {
                int32_t value = (int32_t)(raw - LF->SensorMinValues[i]) * 1000 /
                                (LF->SensorMaxValues[i] - LF->SensorMinValues[i]);
                if (value < 0) value = 0;
                if (value > 1000) value = 1000;
                LF->SensorRawValues[i] = (uint16_t)value;
            } else {
                LF->SensorRawValues[i] = 0;
            }
        } else {
            LF->SensorRawValues[i] = raw;
        }
    }
}

static int SensorRead(LineFollower_t *LF)
{
    readSensorsCalibrated(LF);
    
    int position_sum = 0;
    int active = 0;

    for (uint8_t i = 0u; i < 14u; i++)
    {
        uint8_t is_active = (LF->SensorRawValues[i] > LF->treshold) ? 1u : 0u;
        LF->SensorValues[i] = is_active;

        if (is_active)
        {
            position_sum += LF->SensorRawValues[i] * i * 1000;
            active += LF->SensorRawValues[i];
        }
    }

    if (LF->SensorValues[0] != 0u && millis() > (LF->LastEndTimer + 1u))
    {
        LF->LastEndTimer = millis();
        LF->Last_end = 0;
    }

    if (LF->SensorValues[13] != 0u && millis() > (LF->LastEndTimer + 1u))
    {
        LF->LastEndTimer = millis();
        LF->Last_end = 1;
    }

    LF->actives = active;
    if (active == 0)
    {
        LF->Last_idle++;
        LF->SensorPosition = (LF->Last_end == 1) ? 13000 : 0;
    }
    else
    {
        LF->Last_idle = 0;
        LF->SensorPosition = position_sum / active;
    }

    return LF->SensorPosition;
}

int LineFollower_UpdateSensors(LineFollower_t *LF)
{
    return SensorRead(LF);
}

void hardware_motor_write(float right_pwm, float left_pwm)
{
    // Map from ROBOT_PWM_MAX to ESP32 8-bit analogWrite (0-255)
    int left_out = (int)(fabs(left_pwm) * 255.0f / ROBOT_PWM_MAX);
    int right_out = (int)(fabs(right_pwm) * 255.0f / ROBOT_PWM_MAX);
    
    if (left_out > 255) left_out = 255;
    if (right_out > 255) right_out = 255;

    // DRV8835 in IN/IN Mode (Fast Decay). 
    // Left motor is mechanically mirrored from right motor.
    
    if (left_pwm >= 0.0f) {
        // Forward (Left): AIN1=0, AIN2=PWM
        analogWrite(PIN_AIN1, 0);
        analogWrite(PIN_AIN2, left_out);
    } else {
        // Reverse (Left): AIN1=PWM, AIN2=0
        analogWrite(PIN_AIN1, left_out);
        analogWrite(PIN_AIN2, 0);
    }

    if (right_pwm >= 0.0f) {
        // Forward (Right): BIN1=PWM, BIN2=0
        analogWrite(PIN_BIN1, right_out);
        analogWrite(PIN_BIN2, 0);
    } else {
        // Reverse (Right): BIN1=0, BIN2=PWM
        analogWrite(PIN_BIN1, 0);
        analogWrite(PIN_BIN2, right_out);
    }
}

void motor_control(LineFollower_t *LF, float pos_right, float pos_left)
{
    float speed_scale = (LF->Speed_level > 0.05f) ? LF->Speed_level : 1.0f;
    LF->TargetMotorLeft = clampf(pos_left * speed_scale, -LF->Max_speed_L, LF->Max_speed_L);
    LF->TargetMotorRight = clampf(pos_right * speed_scale, -LF->Max_speed_R, LF->Max_speed_R);
}

static void sharp_turn(LineFollower_t *LF)
{
    if (LF->Last_idle < 25)
    {
        if (LF->Last_end == 1)
        {
            motor_control(LF, LF->Sharp_bend_speed_right, LF->Sharp_bend_speed_left);
        }
        else
        {
            motor_control(LF, LF->Sharp_bend_speed_left, LF->Sharp_bend_speed_right);
        }
    }
    else
    {
        if (LF->Last_end == 1)
        {
            motor_control(LF, LF->Bend_speed_right, LF->Bend_speed_left);
        }
        else
        {
            motor_control(LF, LF->Bend_speed_left, LF->Bend_speed_right);
        }
    }
}

static uint8_t EdgeSensorsActive(const LineFollower_t *LF)
{
    for (uint8_t i = 0u; i < ROBOT_GAP_BRIDGE_EDGE_GUARD_COUNT; i++)
    {
        if (LF->SensorValues[i] != 0u) return 1u;
        if (LF->SensorValues[13u - i] != 0u) return 1u;
    }
    return 0u;
}

static uint8_t GapBridgeCandidate(const LineFollower_t *LF, float error)
{
    if (LF->actives <= 0) return 0u;
    if (fabs(error) > ROBOT_GAP_BRIDGE_CENTER_ERROR) return 0u;
    return (EdgeSensorsActive(LF) == 0u) ? 1u : 0u;
}

static uint8_t TryGapBridge(LineFollower_t *LF)
{
    uint32_t now = millis();

    if (LF->GapBridgeActive != 0u)
    {
        if ((now - LF->GapBridgeStartMs) <= ROBOT_GAP_BRIDGE_TIMEOUT_MS)
        {
            motor_control(LF, LF->LastLineMotorRight, LF->LastLineMotorLeft);
            return 1u;
        }

        LF->GapBridgeActive = 0u;
        LF->GapBridgeArmed = 0u;
        return 0u;
    }

    if (LF->GapBridgeArmed != 0u)
    {
        LF->GapBridgeActive = 1u;
        LF->GapBridgeStartMs = now;
        motor_control(LF, LF->LastLineMotorRight, LF->LastLineMotorLeft);
        return 1u;
    }

    return 0u;
}

static void forward_brake(LineFollower_t *LF, float pos_right, float pos_left)
{
    if (LF->actives == 0)
    {
        if (TryGapBridge(LF) == 0u)
        {
            sharp_turn(LF);
        }
    }
    else
    {
        LF->GapBridgeActive = 0u;
        motor_control(LF, pos_right, pos_left);
    }
}

static void past_errors(LineFollower_t *LF, int error)
{
    for (int i = 9; i > 0; i--)
    {
        LF->Errors[i] = LF->Errors[i - 1];
    }
    LF->Errors[0] = error;
}

void PID_control(LineFollower_t *LF)
{
    uint16_t position = (uint16_t)LineFollower_UpdateSensors(LF);

    if (LF->actives == 0)
    {
        forward_brake(LF, 0.0f, 0.0f);
        return;
    }

    float error = ((float)position - 6500.0f) / 1000.0f;

    LF->P = error;
    LF->Error_P = error;
    LF->D = error - LF->Last_error;
    LF->Error_D = LF->D;
    LF->Last_error = error;
    past_errors(LF, (int)error);

    float motor_speed = (LF->P * LF->Kp * LF->Kp_Multiplier) + (LF->D * LF->Kd * LF->Kd_Multiplier);
    float left_speed = LF->Base_speed_L + motor_speed;
    float right_speed = LF->Base_speed_R - motor_speed;

    left_speed = clampf(left_speed, -LF->Max_speed_L, LF->Max_speed_L);
    right_speed = clampf(right_speed, -LF->Max_speed_R, LF->Max_speed_R);

    LF->LastLineMotorRight = right_speed;
    LF->LastLineMotorLeft = left_speed;
    LF->GapBridgeArmed = GapBridgeCandidate(LF, error);

    forward_brake(LF, right_speed, left_speed);
}

float GetAverageSpeed(LineFollower_t *LF)
{
    return LF->AverageSpeed;
}
