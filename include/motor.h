#pragma once

#include <cstdint>
#include "low_pass_filter.h"

constexpr float RPM_TO_MH = 0.20833333333f;
constexpr float IMPULSES_PER_ROTATION = 4096.0f; // AS5600 resolution per shaft turn
constexpr float GEAR_RATIO = 10.0f;
constexpr float MEASUREMENT_PERIOD = 0.001f; // 1 kHz loop
constexpr float WHEEL_CIRCUMFERENCE = 0.0706858347f;

struct motor_t {
    uint16_t encoderRaw = 0;
    uint16_t encoderPrev = 0;
    float RPM = 0.0f;
    float PreviousRPM = 0.0f;
    float NumberOfRotations = 0.0f;
    float MetersPerHour = 0.0f;
    float KilometersPerHour = 0.0f;
    float MetersPerSecond = 0.0f;
    float DistanceInMeasurement = 0.0f;
    float LpfDistanceInMeasurement = 0.0f;
    float DistanceTraveled = 0.0f;
    float set_speed = 0.0f;
    float current_speed = 0.0f;
    float speed = 0.0f;
    float error = 0.0f;
    float Error_sum = 0.0f;
    float ki = 0.0f;
    float kp = 0.0f;
    LowPassFilter_t EncoderRpmFilter;
    LowPassFilter_t MetersPerSecondLPF;
};

void Motor_Init(motor_t* motor, float Kp, float Ki);
void Motor_CalculateSpeed(motor_t* motor, int32_t impulses);
void PI_Loop(motor_t* motor);
