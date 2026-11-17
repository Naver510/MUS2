#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

typedef struct {
    float alpha;
    float output;
} LowPassFilter_t;

void LowPassFilter_Init(LowPassFilter_t *LPF, float alpha);
float LowPassFilter_Update(LowPassFilter_t *LPF, float input);

typedef struct
{
    volatile int32_t *encoder_count;
    int32_t EncoderValue;
    int32_t PreviousEncoderValue;
    int8_t EncoderDirection;

    float RPM;
    float PreviousRPM;

    // Liczba tikow, w ktorych delta enkodera przekroczyla fizyczny limit (3000 RPM)
    // i zostala przycieta (glitch odczytu). Do diagnostyki w logu.
    volatile uint32_t glitch_count;
    uint32_t glitch_last_print_ms;

    float MetersPerSecond;
    float PreviousMetersSecond;
    float DistanceInMeasurement;
    float LpfDistanceInMeasurement;

    float set_speed;
    float current_speed;
    float speed;
    float error;
    float Error_sum;

    float kp;
    float ki;

    float PWM_to_MS_Scaler_value;

    LowPassFilter_t EncoderRpmFilter;
    LowPassFilter_t MetersPerSecondLPF;

} motor_t;

void Motor_Init(motor_t *motor, volatile int32_t* enc_count, float Kp, float Ki);
void Motor_SetEncoderDirection(motor_t *motor, int8_t direction);
void Motor_CalculateSpeed(motor_t *motor);
void PI_Loop(motor_t *motor);

#endif // MOTOR_H
