#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <SPI.h>

typedef struct {
    int cs_pin;
    float Gyro_Z_RawDps;
    float Gyro_Z_BiasDps;
    float Gyro_Z;
    uint8_t initialized;

    // Non-blocking calibration state
    uint16_t cal_samples_target;  // total samples to collect (0 = not calibrating)
    uint16_t cal_samples_done;    // samples collected so far
    float    cal_sum;             // running sum of raw DPS readings
    uint8_t  calibrating;         // 1 while calibration is in progress
} IMU_t;

// Call once in setup() — uses delay() only before FreeRTOS tasks start, which is acceptable
uint8_t IMU_Init(IMU_t *imu, int cs_pin);

// Call every control-loop tick. Returns 1 if a new sample was stored, 0 if IMU not ready.
uint8_t IMU_Read_GyroZ(IMU_t *imu);

// Start a non-blocking calibration: call IMU_CalibrateTick() each control-loop tick.
// 'samples' readings, collected at the natural control-loop rate (no delay needed).
void IMU_StartCalibration(IMU_t *imu, uint16_t samples);

// Call every control-loop tick while imu->calibrating == 1.
// Returns 1 when calibration is complete, 0 while still collecting.
uint8_t IMU_CalibrateTick(IMU_t *imu);

// Online bias correction: call when robot is known to be stationary.
void IMU_UpdateGyroZBias(IMU_t *imu, float learn_rate);

#endif // IMU_H
