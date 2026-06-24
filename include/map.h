#pragma once

#include <cstdint>
#include "motor.h"

constexpr float MAIN_PCB_LENGTH = 0.155f;
constexpr float MAIN_TO_SENSOR_LENGTH = 0.160f;

struct Map_t {
    uint8_t Mapping = 0;
    float Kk = 0.0f;
    float Ti = 0.0f;
    float Ri = 0.0f;
    float Xri = 0.0f, PreviousXri = 0.0f;
    float Yri = 0.0f, PreviousYri = 0.0f;
    float Ori = 0.0f, PreviousOri = 0.0f, OriIMU = 0.0f;
    float AlhphaOri = 1.0f;
    float Qri[3] = {0.0f, 0.0f, 0.0f};
    float Pci[2] = {0.0f, 0.0f};
    float PreviousPci[2] = {0.0f, 0.0f};
    float Si = 0.0f;
    float Ai = 0.0f, PreviousAi = 0.0f;
    float Ki = 0.0f;
    float SetX = 0.0f;
    float SetY = 0.0f;
    float SetRotation = 0.0f;
    float SetSpeed = 0.0f;
    float p = 0.13f;
    float i = 0.0f;
    float d = 13.0f;
    float ErrorSum = 0.0f;
    float LastError = 0.0f;
    float mi = 0.0f;
};

struct MapSample {
    char buffer[128];
    int len = 0;
};

using MotorControlFn = void (*)(double, double);
using LoadNextWaypointFn = bool (*)(Map_t& map);

bool MapUpdateV2(Map_t& map, motor_t& MotorLeft, motor_t& MotorRight, float errorP, float errorD, MapSample* sampleOut);
void DriveOnMap(Map_t& map, motor_t& MotorLeft, motor_t& MotorRight, LoadNextWaypointFn loadNext, MotorControlFn motorControl);
