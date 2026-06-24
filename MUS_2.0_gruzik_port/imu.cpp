#include "imu.h"
#include "robot_config.h"

// BMI160 Registers (all offsets from datasheet Table 25)
#define BMI160_REG_CHIPID      0x00
#define BMI160_REG_PMU_STATUS  0x03
// Gyro data registers (BMI160 Table 25):
//   0x0C/0x0D = GYR_X,  0x0E/0x0F = GYR_Y,  0x10/0x11 = GYR_Z   ← gyro block
//   0x12/0x13 = ACC_X,  0x14/0x15 = ACC_Y,  0x16/0x17 = ACC_Z   ← accel block
// GYR_Z_L = 0x10  (NOT 0x16 — that is ACC_Z_L!)
#define BMI160_REG_DATA_GYRO_Z 0x10
#define BMI160_REG_CMD         0x7E

#define BMI160_CMD_GYRO_NORMAL 0x15   // set gyro to normal (active) power mode

// ---------------------------------------------------------------------------
// Internal SPI helpers
// ---------------------------------------------------------------------------

static void writeReg(int cs_pin, uint8_t reg, uint8_t val) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); // BMI160: CPOL=0, CPHA=0
    digitalWrite(cs_pin, LOW);
    SPI.transfer(reg & 0x7F);   // MSB=0 → write
    SPI.transfer(val);
    digitalWrite(cs_pin, HIGH);
    SPI.endTransaction();
}

static uint8_t readReg(int cs_pin, uint8_t reg) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); // BMI160: CPOL=0, CPHA=0
    digitalWrite(cs_pin, LOW);
    SPI.transfer(reg | 0x80);   // MSB=1 → read
    uint8_t val = SPI.transfer(0x00);
    digitalWrite(cs_pin, HIGH);
    SPI.endTransaction();
    return val;
}

// ---------------------------------------------------------------------------
// IMU_Init — called once in setup() before FreeRTOS tasks are started.
// delay() here is acceptable: tasks haven't launched yet.
// ---------------------------------------------------------------------------
uint8_t IMU_Init(IMU_t *imu, int cs_pin)
{
    memset(imu, 0, sizeof(*imu));
    imu->cs_pin = cs_pin;

    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);

    // One dummy read forces BMI160 into SPI mode (required after power-on)
    readReg(cs_pin, 0x7F);
    delay(2);   // ≤2 ms — BMI160 datasheet §2.2: SPI mode active after first CS assertion

    uint8_t chip_id = readReg(cs_pin, BMI160_REG_CHIPID);
    if (chip_id != 0xD1) {
        Serial.printf("[IMU] BMI160 chip-ID wrong: 0x%02X (expected 0xD1)\n", chip_id);
        return 0;
    }

    // Start gyro in normal (full-power) mode
    writeReg(cs_pin, BMI160_REG_CMD, BMI160_CMD_GYRO_NORMAL);
    delay(100); // BMI160 datasheet: gyro startup ≤80 ms. 100 ms guard. OK in setup().

    // Default gyro range = ±2000 dps → 16.4 LSB/dps (no extra config needed)

    imu->initialized = 1;
    Serial.println("[IMU] BMI160 init OK (SPI, ±2000 dps range)");
    return 1;
}

// ---------------------------------------------------------------------------
// IMU_Read_GyroZ — reads one Z-gyro sample, fully non-blocking (SPI burst).
// Call every control-loop tick (1 kHz).
// ---------------------------------------------------------------------------
uint8_t IMU_Read_GyroZ(IMU_t *imu)
{
    if (!imu->initialized) return 0;

    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(imu->cs_pin, LOW);
    SPI.transfer(BMI160_REG_DATA_GYRO_Z | 0x80);  // read burst from 0x16
    uint8_t zl = SPI.transfer(0x00);               // GYRO_Z_L
    uint8_t zh = SPI.transfer(0x00);               // GYRO_Z_H
    digitalWrite(imu->cs_pin, HIGH);
    SPI.endTransaction();

    int16_t gz = (int16_t)((zh << 8) | zl);

    // ±2000 dps → sensitivity = 16.4 LSB/dps
    imu->Gyro_Z_RawDps = (float)gz / 16.4f;
    imu->Gyro_Z_RawDps *= ROBOT_IMU_Z_DIRECTION;

    // Subtract calibrated bias
    imu->Gyro_Z = imu->Gyro_Z_RawDps - imu->Gyro_Z_BiasDps;
    return 1;
}

// ---------------------------------------------------------------------------
// Non-blocking calibration — replaces the old blocking IMU_CalibrateGyroZ().
// Call IMU_StartCalibration() once, then IMU_CalibrateTick() each control tick.
// Robot must be stationary during calibration.
// ---------------------------------------------------------------------------
void IMU_StartCalibration(IMU_t *imu, uint16_t samples)
{
    if (!imu->initialized) return;
    imu->cal_samples_target = samples;
    imu->cal_samples_done   = 0;
    imu->cal_sum            = 0.0f;
    imu->calibrating        = 1;
    Serial.printf("[IMU] Gyro-Z calibration started (%u samples at 1 kHz)\n", samples);
}

// Returns 1 when done (bias updated), 0 while still collecting.
uint8_t IMU_CalibrateTick(IMU_t *imu)
{
    if (!imu->calibrating) return 1;

    // IMU_Read_GyroZ already called this tick in ControlLoopTask — use its result directly
    imu->cal_sum += imu->Gyro_Z_RawDps;
    imu->cal_samples_done++;

    if (imu->cal_samples_done >= imu->cal_samples_target) {
        imu->Gyro_Z_BiasDps = imu->cal_sum / (float)imu->cal_samples_target;
        imu->calibrating = 0;
        Serial.printf("[IMU] Calibration done. Gyro-Z bias = %.3f dps\n", imu->Gyro_Z_BiasDps);
        return 1;
    }
    return 0;
}

// ---------------------------------------------------------------------------
// Online bias correction — call when robot is stationary (optional).
// ---------------------------------------------------------------------------
void IMU_UpdateGyroZBias(IMU_t *imu, float learn_rate)
{
    if (!imu->initialized) return;
    imu->Gyro_Z_BiasDps =
        (1.0f - learn_rate) * imu->Gyro_Z_BiasDps +
        learn_rate * imu->Gyro_Z_RawDps;
}
