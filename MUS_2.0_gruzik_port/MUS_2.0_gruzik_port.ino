#include <Arduino.h>
#include <WiFi.h>
#include <AsyncUDP.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include "web_server.h"

#include "robot_config.h"
#include "motor.h"
#include "imu.h"
#include "encoder.h"    // 2× SW-ISR quadrature decoder
#include "odometry.h"
#include "line_follower.h"
#include "map.h"
#include "nvs_params.h"

// Globals
LineFollower_t GRUZIK;
Map_t GruzikMap;
motor_t Motor_R;
motor_t Motor_L;
Odometry_t Odometry;
IMU_t IMU;

// enc1_count / enc2_count are now read-only snapshots updated each control tick
// from the hardware PCNT counters via Encoder1_GetCount() / Encoder2_GetCount().
// Kept as volatile int32_t so Motor_Init() pointer interface is unchanged.
static volatile int32_t enc1_count = 0;
static volatile int32_t enc2_count = 0;

// Note: SD card shares the main SPI bus (SPI2) with the IMU.
// Using a separate SPIClass(HSPI) would reconfigure the GPIO matrix to SPI3,
// silently disconnecting the IMU from the bus — do NOT add spiSD(HSPI).

// SD Card logging double buffering
#define LOG_BUFFER_SIZE 32768
char logBufferA[LOG_BUFFER_SIZE];
char logBufferB[LOG_BUFFER_SIZE];
volatile char *activeBuffer = logBufferA;
volatile char *writeBuffer = logBufferB;
volatile size_t activeBufferPos = 0;
volatile size_t writeBufferLen = 0;
SemaphoreHandle_t bufferSwapMutex;
volatile bool bufferReadyToWrite = false;
volatile bool telemetryActive = false;
volatile bool flushRequested = false;

// SPI bus arbitration — IMU (Core 1, 1 kHz) and SD (Core 0) share SPI2.
// A FreeRTOS binary semaphore is required instead of a plain volatile flag:
// ESP32-S3 Xtensa LX7 cores each have private data caches; a volatile write
// on Core 0 may not be seen by Core 1 until a cache-flush barrier occurs.
// FreeRTOS xSemaphoreGive/Take include portMEMORY_BARRIER() and therefore
// guarantee cross-core visibility without manual barrier instructions.
//
// Protocol:
//   IMU read  → xSemaphoreTake(spiSem, 0)  // non-blocking, skip if SD busy
//   SD ops    → xSemaphoreTake(spiSem, portMAX_DELAY) held for entire op
SemaphoreHandle_t spiSem = NULL;

// Helpers
static void parseMessageValue(const String &message, float &outValue) {
    int spaceIndex = message.indexOf(' ');
    if (spaceIndex != -1) {
        String valStr = message.substring(spaceIndex + 1);
        valStr.trim();
        outValue = valStr.toFloat();
    }
}

void sdWriteTask(void *parameter) {
    while (true) {
        if (bufferReadyToWrite || flushRequested) {
            xSemaphoreTake(bufferSwapMutex, portMAX_DELAY);
            size_t len = writeBufferLen;
            bool doWrite = (len > 0);
            bufferReadyToWrite = false;
            flushRequested = false;
            xSemaphoreGive(bufferSwapMutex);

            if (doWrite) {
                // Acquire SPI bus exclusively for the entire file operation.
                // ControlLoopTask will skip IMU reads (use stale data) until released.
                xSemaphoreTake(spiSem, portMAX_DELAY);
                File logFile = SD.open("/GRUZIK.txt", FILE_APPEND);
                if (logFile) {
                    logFile.write((const uint8_t *)writeBuffer, len);
                    logFile.close();
                    Serial.println("[SD] write OK");
                } else {
                    Serial.println("[SD] ERROR: open /GRUZIK.txt failed");
                }
                xSemaphoreGive(spiSem);

                xSemaphoreTake(bufferSwapMutex, portMAX_DELAY);
                writeBufferLen = 0;
                xSemaphoreGive(bufferSwapMutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void flushLogBuffer() {
    xSemaphoreTake(bufferSwapMutex, portMAX_DELAY);
    if (activeBufferPos > 0) {
        volatile char *temp = activeBuffer;
        activeBuffer = writeBuffer;
        writeBuffer = temp;
        writeBufferLen = activeBufferPos;
        activeBufferPos = 0;
        flushRequested = true;
    }
    xSemaphoreGive(bufferSwapMutex);
}

void telemetryLog(const char* data) {
    if (!telemetryActive) return;
    size_t len = strlen(data);
    xSemaphoreTake(bufferSwapMutex, portMAX_DELAY);
    if (activeBufferPos + len < LOG_BUFFER_SIZE) {
        memcpy((void *)(activeBuffer + activeBufferPos), data, len);
        activeBufferPos += len;
    } else {
        volatile char *temp = activeBuffer;
        activeBuffer = writeBuffer;
        writeBuffer = temp;
        writeBufferLen = activeBufferPos;
        activeBufferPos = 0;
        bufferReadyToWrite = true;
        memcpy((void *)(activeBuffer + activeBufferPos), data, len);
        activeBufferPos += len;
    }
    xSemaphoreGive(bufferSwapMutex);
}

void ControlLoopTask(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1 ms → 1 kHz

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // --- Snap PCNT hardware counters into the variables Motor_* reads ---
        enc1_count = Encoder1_GetCount();
        enc2_count = Encoder2_GetCount();

        // --- IMU: read gyro-Z (non-blocking SPI burst) ---
        // Non-blocking semaphore: if SD is using SPI bus, skip and use last value.
        // xSemaphoreTake with timeout=0 never blocks, preserving 1 kHz loop timing.
        if (xSemaphoreTake(spiSem, 0) == pdTRUE) {
            IMU_Read_GyroZ(&IMU);
            xSemaphoreGive(spiSem);
        }

        // --- Non-blocking calibration: runs for first N ticks, then done ---
        if (IMU.calibrating) {
            IMU_CalibrateTick(&IMU);
            // While calibrating keep motors stopped; skip state machine below
            Motor_L.set_speed = 0.0f;
            Motor_R.set_speed = 0.0f;
            PI_Loop(&Motor_L);
            PI_Loop(&Motor_R);
            hardware_motor_write(0.0f, 0.0f);
            continue;
        }

        // --- Speed calculation (per-wheel PI has its own LPF) ---
        Motor_CalculateSpeed(&Motor_L);
        Motor_CalculateSpeed(&Motor_R);

        // --- Odometry: fused encoder + gyro-Z ---
        Odometry_Update(&Odometry,
                        Motor_L.DistanceInMeasurement,
                        Motor_R.DistanceInMeasurement,
                        IMU.Gyro_Z,
                        ROBOT_CONTROL_DT_S);

        // --- Online gyro bias correction — active when robot is stationary ---
        if (fabsf(Motor_L.DistanceInMeasurement) < ROBOT_ENCODER_STATIONARY_M &&
            fabsf(Motor_R.DistanceInMeasurement) < ROBOT_ENCODER_STATIONARY_M &&
            fabsf(IMU.Gyro_Z) < ROBOT_GYRO_STATIONARY_DPS)
        {
            IMU_UpdateGyroZBias(&IMU, ROBOT_GYRO_BIAS_LEARN_RATE);
        }

        // --- Main state machine ---
        if (GRUZIK.PowerMode == Start) {
            if (GRUZIK.state == PidFollowing) {
                PID_control(&GRUZIK);
            } else if (GRUZIK.state == Mapping || GRUZIK.state == SpeedUnMapping) {
                PID_control(&GRUZIK);
                float avgSpeed = 0.5f * (Motor_L.MetersPerSecond + Motor_R.MetersPerSecond);
                if (MapUpdate(&GruzikMap, &Odometry.pose, avgSpeed, GRUZIK.P, GRUZIK.D) == 0) {
                    GRUZIK.state = PidFollowing;
                    telemetryActive = false;
                    flushLogBuffer();
                }
            } else if (GRUZIK.state == UnMapping) {
                if (DriveOnMap(&GruzikMap, &Odometry.pose) == 0) {
                    GRUZIK.state = PidFollowing;
                }
            }

            // Per-wheel PI speed loops (Motor_L and Motor_R are independent)
            Motor_L.set_speed = GRUZIK.TargetMotorLeft;
            Motor_R.set_speed = GRUZIK.TargetMotorRight;
            PI_Loop(&Motor_L);
            PI_Loop(&Motor_R);
            hardware_motor_write(Motor_R.speed * Motor_R.PWM_to_MS_Scaler_value,
                                 Motor_L.speed * Motor_L.PWM_to_MS_Scaler_value);

        } else if (GRUZIK.PowerMode == ForceForward) {
            motor_control(&GRUZIK, 1.0f, 1.0f);
            Motor_L.set_speed = GRUZIK.TargetMotorLeft;
            Motor_R.set_speed = GRUZIK.TargetMotorRight;
            PI_Loop(&Motor_L);
            PI_Loop(&Motor_R);
            hardware_motor_write(Motor_R.speed * Motor_R.PWM_to_MS_Scaler_value,
                                 Motor_L.speed * Motor_L.PWM_to_MS_Scaler_value);

        } else {
            // Stop / Initialization: zero targets, keep PI alive to bleed I-term
            Motor_L.set_speed = 0.0f;
            Motor_R.set_speed = 0.0f;
            PI_Loop(&Motor_L);
            PI_Loop(&Motor_R);
            hardware_motor_write(0.0f, 0.0f);
        }
    }
}

void WebTelemetryTask(void *parameter) {
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // 20 Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();
    unsigned long lastDebugMillis = 0;

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        WebServer_BroadcastTelemetry();

        unsigned long currentMillis = millis();
        if (currentMillis - lastDebugMillis >= 500) {
            lastDebugMillis = currentMillis;
            // Diagnostics:
            //   GPIO raw  : stuck at 1? → encoder not powered or wrong pin
            //   ISR count : stays 0?    → ISR never called (GPIO/hardware issue)
            //   ISR count ↑ but enc=0?  → direction cancellation in ISR logic
            Serial.printf(
                "ENC cnt=%d/%d  isr#=%u/%u  GPIO6=%d,7=%d,8=%d,9=%d | "
                "IMU_RawZ=%.2f bias=%.3f cal=%d\n",
                (int)Encoder1_GetCount(), (int)Encoder2_GetCount(),
                isr1_count, isr2_count,
                digitalRead(PIN_ENC1_A), digitalRead(PIN_ENC1_B),
                digitalRead(PIN_ENC2_A), digitalRead(PIN_ENC2_B),
                IMU.Gyro_Z_RawDps, IMU.Gyro_Z_BiasDps, (int)IMU.calibrating
            );
        }
    }
}

void setup() {
    Serial.begin(115200);

    // --- Output pins ---
    pinMode(PIN_MUX_S0, OUTPUT);
    pinMode(PIN_MUX_S1, OUTPUT);
    pinMode(PIN_MUX_S2, OUTPUT);
    pinMode(PIN_MUX_S3, OUTPUT);
    pinMode(PIN_MUX_SIG, INPUT);

    pinMode(PIN_AIN1, OUTPUT);
    pinMode(PIN_AIN2, OUTPUT);
    pinMode(PIN_BIN1, OUTPUT);
    pinMode(PIN_BIN2, OUTPUT);
    pinMode(PIN_MODE, OUTPUT);
    digitalWrite(PIN_MODE, LOW); // IN/IN Mode for DRV8835

    // --- SPI bus (shared by IMU and SD card) ---
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);
    pinMode(PIN_SPI_MISO, INPUT_PULLUP); // Prevent MISO floating when IMU releases bus

    // --- IMU: BMI160 on SPI ---
    // IMU_Init uses delay() — acceptable here (setup(), before any task is started)
    if (!IMU_Init(&IMU, PIN_IMU_CS)) {
        Serial.println("[IMU] Init failed — gyro will be unavailable");
    }
    // Gyro-Z bias calibration is non-blocking: accumulates IMU_CAL_SAMPLES
    // readings in ControlLoopTask at 1 kHz.  Robot must be stationary at boot.
    IMU_StartCalibration(&IMU, 500); // 500 ms at 1 kHz, no delay()

    // --- Encoders: 2× SW-ISR (A-only CHANGE) on GPIO6 (ENC1) and GPIO8 (ENC2) ---
    // B pins are read inside the A ISR for direction only. CPR=140 (7PPR×2×10).
    // 4× (A+B ISR) causes cancellation at high RPM due to FreeRTOS ISR latency.
    Encoders_Init();
    Encoders_Reset();

    // --- Motor structs: pass pointers to the SW-ISR snapshot variables ---
    Motor_Init(&Motor_L, &enc1_count, 0.05f, 0.01f);
    Motor_Init(&Motor_R, &enc2_count, 0.05f, 0.01f);
    Motor_SetEncoderDirection(&Motor_L, ROBOT_LEFT_ENCODER_DIRECTION);
    Motor_SetEncoderDirection(&Motor_R, ROBOT_RIGHT_ENCODER_DIRECTION);
    Motor_L.PWM_to_MS_Scaler_value = 114.4f; // 286 PWM / 2.5 m/s
    Motor_R.PWM_to_MS_Scaler_value = 114.4f;

    LowPassFilter_Init(&Motor_L.EncoderRpmFilter, ENCODER_SPEED_FILTER_ALPHA);
    LowPassFilter_Init(&Motor_R.EncoderRpmFilter, ENCODER_SPEED_FILTER_ALPHA);
    LowPassFilter_Init(&Motor_L.MetersPerSecondLPF, ENCODER_SPEED_FILTER_ALPHA);
    LowPassFilter_Init(&Motor_R.MetersPerSecondLPF, ENCODER_SPEED_FILTER_ALPHA);

    // --- Odometry ---
    Odometry_Init(&Odometry, ROBOT_ODOMETRY_GYRO_WEIGHT);

    // --- SD Card: shares SPI2 bus with the IMU (same SPI object, different CS) ---
    // spiSD(HSPI) must NOT be used here: HSPI.begin() would reroute GPIO11-13
    // to SPI3 in the GPIO matrix, permanently breaking IMU reads on SPI2.
    //
    // SD init sequence requirements (SD spec §6.4.1):
    //   1. CS must be HIGH before first clock pulse (74+ dummy clocks with CS=H)
    //   2. Identification mode: max 400 kHz
    //   3. A short settling delay after IMU SPI activity helps the bus stabilize
    pinMode(PIN_SD_CS, OUTPUT);
    digitalWrite(PIN_SD_CS, HIGH);   // ensure CS is deasserted before SD.begin()
    delay(10);                        // let bus settle after IMU SPI activity
    if (!SD.begin(PIN_SD_CS, SPI)) {
        Serial.println("[SD] init failed");
    }
    // NOTE: SdMapFile is NOT opened here intentionally.
    // Log writes go through the double-buffer sdWriteTask (FILE_APPEND).
    // SdMapFile is opened read-only in Map_BeginPlayback() for playback only.
    // Opening it here would hold a file-handle slot permanently and cause
    // undefined behaviour when SD.remove() is called during "mapping".

    // --- Load tuning params from NVS ---
    Load_Params(&GRUZIK, &GruzikMap);
    Load_Cal(&GRUZIK);

    // --- Semaphore for SPI bus arbitration (IMU vs SD card) ---
    spiSem = xSemaphoreCreateBinary();
    xSemaphoreGive(spiSem);  // initially available

    // --- Mutex for double-buffer SD logging ---
    bufferSwapMutex = xSemaphoreCreateMutex();

    // --- WebServer ---
    WebServer_Init();

    // --- FreeRTOS Tasks ---
    xTaskCreatePinnedToCore(sdWriteTask,      "SDWrite",     4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(ControlLoopTask,  "ControlLoop", 8192, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(WebTelemetryTask, "WebTelemetry",4096, NULL, 2, NULL, 0);
}

void loop() {
    vTaskDelete(NULL); // Arduino loop() not used — all work is in FreeRTOS tasks
}
