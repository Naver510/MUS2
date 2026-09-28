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

// Note: IMU now uses a dedicated hardware SPI bus (HSPI).
// The SD card remains on the primary SPI bus (FSPI).
SPIClass spiIMU(HSPI);

// Telemetry log: single buffer in PSRAM, flushed to SD only when robot stops.
// During active driving only the IMU uses SPI; the SD card stays completely
// idle, eliminating the bus-contention errors that corrupt the SD state machine.
// After stop, IMU reads are gated off and SD has exclusive SPI access.
#define TELEMETRY_RAM_SIZE (512u * 1024u)  // 512 KB — ~5 min of mapping at 25 rec/s
char *telemetryBuffer = NULL;
size_t telemetryBufferCapacity = 0;
volatile size_t telemetryBufferPos = 0;
volatile bool telemetryActive = false;
volatile bool sdFlushRequested = false;
volatile uint32_t droppedPoints = 0; // diagnostic counter for dropped records

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

// NOTE: sdReinit() (SD.end + SD.begin) was removed.
// SD.end() calls spi_bus_remove_device() which permanently tears down the
// ESP-IDF SPI device handle; subsequent SD.begin() fails consistently after
// any IMU activity on the shared bus.  The card is mounted once at boot and
// stays mounted.  Flush uses SD.open() directly — it works because IMU reads
// are gated off (PowerMode == Stop) before sdWriteTask runs the flush.

void sdWriteTask(void *parameter) {
    while (true) {
        if (sdFlushRequested && telemetryBuffer != NULL) {
            // At this point telemetryActive is already false — ControlLoopTask
            // is no longer writing to the buffer.  PowerMode is Stop (or
            // mapping auto-closed) so IMU reads are gated off (see ControlLoopTask
            // gate: `if (IMU.calibrating || GRUZIK.PowerMode == Start)`).  The
            // SPI bus belongs exclusively to this task — no reinit needed.
            size_t len = telemetryBufferPos;
            if (len > 0) {
                xSemaphoreTake(spiSem, portMAX_DELAY);
                File logFile = SD.open(ROBOT_MAP_OUTPUT_FILE, FILE_APPEND);
                if (logFile) {
                    // Write in 4 KB chunks to avoid watchdog timeout
                    const size_t CHUNK = 4096;
                    size_t written = 0;
                    while (written < len) {
                        size_t n = (len - written < CHUNK) ? (len - written) : CHUNK;
                        logFile.write((const uint8_t *)(telemetryBuffer + written), n);
                        written += n;
                    }
                    logFile.close();
                    Serial.printf("[SD] Flushed %u bytes OK (dropped %u)\n",
                                  (unsigned)len, (unsigned)droppedPoints);
                    telemetryBufferPos = 0;  // only reset on success
                } else {
                    Serial.println("[SD] ERROR: open " ROBOT_MAP_OUTPUT_FILE " failed");
                    // Buffer NOT reset — data preserved for next attempt
                }
                xSemaphoreGive(spiSem);
            } else {
                Serial.println("[SD] Flush requested but buffer empty");
            }
            sdFlushRequested = false;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Signal sdWriteTask to flush the PSRAM buffer to SD.
// Safe to call from any context — just sets two volatile flags.
void requestTelemetryFlush() {
    telemetryActive  = false;
    sdFlushRequested = true;
}

// Append a telemetry record to the PSRAM buffer.
// Called exclusively from ControlLoopTask (Core 1) — no mutex needed.
void telemetryLog(const char* data) {
    if (!telemetryActive || telemetryBuffer == NULL) return;
    size_t len = strlen(data);
    size_t pos = telemetryBufferPos;
    if (pos + len < telemetryBufferCapacity) {
        memcpy(telemetryBuffer + pos, data, len);
        telemetryBufferPos = pos + len;
    } else {
        droppedPoints++;
    }
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
        // Only access SPI for IMU when actively driving or during calibration.
        if (IMU.calibrating || GRUZIK.PowerMode == Start) {
            // IMU is on its own dedicated SPI bus — no semaphore needed!
            IMU_Read_GyroZ(&IMU);
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
                    requestTelemetryFlush();
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

    // --- Main SPI bus (for SD card only now) ---
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);
    
    // --- IMU SPI bus (HSPI) ---
    spiIMU.begin(PIN_IMU_SCK, PIN_IMU_MISO, PIN_IMU_MOSI, -1);
    pinMode(PIN_IMU_MISO, INPUT_PULLUP); // Prevent MISO floating when IMU releases bus

    // --- SD Card: MUST be initialized FIRST on clean SPI bus ---
    // The old code used a dedicated HSPI (SPI3) bus for SD with no prior traffic.
    // In this shared-bus design (both IMU and SD on SPI2), IMU_Init's SPI
    // transactions leave the bus in a state that prevents SD.begin()'s CMD0
    // from getting a response.  Initializing SD first on a pristine bus works.
    // spiSD(HSPI) cannot be used: HSPI.begin() would reroute GPIO11-13
    // to SPI3 in the GPIO matrix, breaking IMU reads on SPI2.
    pinMode(PIN_SD_CS, OUTPUT);
    digitalWrite(PIN_SD_CS, HIGH);   // ensure CS is deasserted before SD.begin()
    delay(10);
    if (!SD.begin(PIN_SD_CS, SPI)) {
        Serial.println("[SD] *** INIT FAILED *** -- check card/wiring");
    } else {
        Serial.printf("[SD] init OK  type=%d  size=%lluMB\n",
                      (int)SD.cardType(),
                      (unsigned long long)(SD.cardSize() / (1024ULL * 1024ULL)));
    }

    // --- IMU: BMI160 on SPI (after SD init to avoid bus pollution) ---
    // IMU_Init uses delay() — acceptable here (setup(), before any task is started)
    if (!IMU_Init(&IMU, PIN_IMU_CS)) {
        Serial.println("[IMU] Init failed -- gyro will be unavailable");
    }

    // --- Encoders: 2x SW-ISR (A-only CHANGE) on GPIO6 (ENC1) and GPIO8 (ENC2) ---
    // B pins are read inside the A ISR for direction only. CPR=140 (7PPRx2x10).
    // 4x (A+B ISR) causes cancellation at high RPM due to FreeRTOS ISR latency.
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

    // NOTE: SdMapFile is NOT opened here intentionally.
    // SdMapFile is opened read-only in Map_BeginPlayback() for playback only.
    // Opening it here would hold a file-handle slot permanently and cause
    // undefined behaviour when SD.remove() is called during "mapping".

    // --- Gyro-Z bias calibration (non-blocking, in ControlLoopTask) ---
    // Runs at 1 kHz for 500 ms.  During calibration the IMU reads happen on
    // Core 1 but nobody accesses SD (no mapping/flush requested yet), so there
    // is no SPI bus contention.  Robot must be stationary at boot.
    IMU_StartCalibration(&IMU, 500);

    // --- Load tuning params from NVS ---
    Load_Params(&GRUZIK, &GruzikMap);
    Load_Cal(&GRUZIK);

    // --- Semaphore for SPI bus arbitration (IMU vs SD card) ---
    spiSem = xSemaphoreCreateBinary();
    xSemaphoreGive(spiSem);  // initially available

    // --- Telemetry buffer in PSRAM (ESP32-S3 N16R2 has 2 MB) ---
    // Data is accumulated in RAM during the run and flushed to SD only on stop.
    telemetryBuffer = (char *)ps_malloc(TELEMETRY_RAM_SIZE);
    if (telemetryBuffer) {
        telemetryBufferCapacity = TELEMETRY_RAM_SIZE;
        Serial.printf("[TEL] PSRAM buffer: %u KB\n", TELEMETRY_RAM_SIZE / 1024);
    } else {
        // Fallback: try regular heap with a smaller buffer
        telemetryBufferCapacity = 32768;
        telemetryBuffer = (char *)malloc(telemetryBufferCapacity);
        if (telemetryBuffer) {
            Serial.printf("[TEL] Heap fallback: %u KB\n",
                          (unsigned)(telemetryBufferCapacity / 1024));
        } else {
            Serial.println("[TEL] ERROR: no memory for telemetry buffer!");
        }
    }

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
