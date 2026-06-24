#include <AS5600.h>
#include <AsyncUDP.h>
#include <DFRobot_BMI160.h>
#include <SD.h>
#include <SPI.h>
#include <WiFi.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>


// I2C Bus 1: AS5600_1 (encoder1)
#define I2C1_SDA 6
#define I2C1_SCL 7

// I2C Bus 2: AS5600_2 (encoder2) + BMI160
#define I2C2_SDA 8
#define I2C2_SCL 9

// CD74HC4067 Multiplexer pins for 14-sensor line array
#define MUX_SIG_PIN 1 // Analog input from multiplexer
#define MUX_S0 2
#define MUX_S1 3
#define MUX_S2 4
#define MUX_S3 5

// Custom 14-sensor line array configuration
#define SENSOR_COUNT 14
#define SENSOR_TIMEOUT 2500 // microseconds for RC decay (if using RC sensors)

uint16_t sensorValues[SENSOR_COUNT];
uint16_t sensorMin[SENSOR_COUNT]; // Calibration min values
uint16_t sensorMax[SENSOR_COUNT]; // Calibration max values
bool sensorCalibrated = false;

// Forward declarations
void muxSelectChannel(uint8_t channel);
uint16_t readSensor(uint8_t sensorIndex);
void readSensors();
void readSensorsCalibrated();
void calibrateSensors();
void initCalibration();
uint16_t readLinePosition(bool whiteLine);
void initSensorArray();
void telemetriaCleanBuffered();
void flushLogBuffer();
void calibration();
void request_sensorsRaw();
void request_params();
bool parseMessageValue(const String &message, float &outValue);
void robot_control();
void PID(int error);
void speedcontrol(int mota, int motb);
void forward_brake(int posa, int posb);
void left_brake(int posa, int posb);
void right_brake(int posa, int posb);
void motor_stop();

// Use the framework's RGB_BUILTIN and override brightness
#undef RGB_BRIGHTNESS
#define RGB_BRIGHTNESS 255
// WS2812B LED on GPIO10
#define RGB_LED_PIN 10
#define RGB_BRIGHTNESS 255

#define PIN_MISO 13 // DAT0
#define PIN_MOSI 11 // CMD
#define PIN_SCLK 12 // CLK
#define PIN_CS 20

TwoWire I2C_2 = TwoWire(1);

DFRobot_BMI160 bmi160; // Will use I2C_2
const int8_t bmi160_i2c_addr = 0x69;

AS5600 encoder1;         // Uses default Wire (I2C1)
AS5600 encoder2(&I2C_2); // Uses I2C_2

SPIClass spiSD(HSPI);

int lastError = 0;
boolean onoff = 0;
int val, cnt = 0, v[3];

// Line detection threshold (0-1000 calibrated range)
// Higher value = sensor sees more of the line
// Typically 200-400 works well after calibration
float threshold = 500;

float maxspeeda = 150;
float maxspeedb = 150;
float basespeeda = 100;
float basespeedb = 100;
float TurnSpeed = 100;

float minspeeda = -100;
float minspeedb = -100;

float Kp = 0.0;
float Ki = 0.0;
float Kd = 0.0;
uint8_t multiP = 1;
uint8_t multiI = 1;
uint8_t multiD = 1;
uint8_t Kpfinal;
uint8_t Kifinal;
uint8_t Kdfinal;
int P;
int I;
int D;
float Pvalue;
float Ivalue;
float Dvalue;

#define MODE_PIN 21
#define AIN1_PIN 18
#define AIN2_PIN 17
#define BIN1_PIN 16
#define BIN2_PIN 15

const char *ssid = "LFMUS";
const char *password = "rabarbar";

AsyncUDP udp;

// --- Double Buffering for Logging ---
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

// --- SD Write Task ---
void sdWriteTask(void *parameter) {
  while (true) {
    if (bufferReadyToWrite || flushRequested) {
      xSemaphoreTake(bufferSwapMutex, portMAX_DELAY);
      size_t len = writeBufferLen;
      if (len > 0) {
        File logFile = SD.open("/log.txt", FILE_APPEND);
        if (logFile) {
          logFile.write((const uint8_t *)writeBuffer, len);
          logFile.close();
        } else {
          Serial.println("Error opening log.txt");
        }
        writeBufferLen = 0;
      }
      bufferReadyToWrite = false;
      flushRequested = false;
      xSemaphoreGive(bufferSwapMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// --- Telemetry Task: 400Hz ---
void telemetryTask(void *parameter) {
  const TickType_t xFrequency = pdMS_TO_TICKS(2); // 2ms = 500Hz
  TickType_t xLastWakeTime = xTaskGetTickCount();
  int skip = 0;
  while (true) {
    if (telemetryActive) {
      if (skip < 4) {
        telemetriaCleanBuffered();
      }
      skip = (skip + 1) % 5;
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void telemetriaCleanBuffered() {
  char line[128];
  int len = 0;
  int raw1 = encoder1.readAngle();
  int raw2 = encoder2.readAngle();
  len = snprintf(line, sizeof(line), "%lu %d %d\n", millis(), raw1, raw2);

  xSemaphoreTake(bufferSwapMutex, portMAX_DELAY);
  if (activeBufferPos + len < LOG_BUFFER_SIZE) {
    memcpy((void *)(activeBuffer + activeBufferPos), line, len);
    activeBufferPos += len;
  } else {
    // Buffer full, swap
    char *temp = (char *)activeBuffer;
    activeBuffer = writeBuffer;
    writeBuffer = temp;
    writeBufferLen = activeBufferPos;
    activeBufferPos = 0;
    bufferReadyToWrite = true;
    // Write new line to new buffer
    memcpy((void *)(activeBuffer + activeBufferPos), line, len);
    activeBufferPos += len;
  }
  xSemaphoreGive(bufferSwapMutex);
}

void flushLogBuffer() {
  xSemaphoreTake(bufferSwapMutex, portMAX_DELAY);
  if (activeBufferPos > 0) {
    // Swap and mark for write
    char *temp = (char *)activeBuffer;
    activeBuffer = writeBuffer;
    writeBuffer = temp;
    writeBufferLen = activeBufferPos;
    activeBufferPos = 0;
    bufferReadyToWrite = true;
    flushRequested = true;
  }
  xSemaphoreGive(bufferSwapMutex);
}

// --- Custom 14-Sensor Line Array Functions ---

// Select multiplexer channel (0-15)
void muxSelectChannel(uint8_t channel) {
  digitalWrite(MUX_S0, channel & 0x01);
  digitalWrite(MUX_S1, (channel >> 1) & 0x01);
  digitalWrite(MUX_S2, (channel >> 2) & 0x01);
  digitalWrite(MUX_S3, (channel >> 3) & 0x01);
}

// Read single sensor through multiplexer
uint16_t readSensor(uint8_t sensorIndex) {
  muxSelectChannel(sensorIndex);
  delayMicroseconds(5); // Allow mux to settle
  return analogRead(MUX_SIG_PIN);
}

// Read all 14 sensors
void readSensors() {
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    sensorValues[i] = readSensor(i);
  }
}

// Read sensors with calibration applied (returns 0-1000 range)
void readSensorsCalibrated() {
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    uint16_t raw = readSensor(i);

    if (sensorCalibrated) {
      // Map raw value to 0-1000 based on calibration
      if (sensorMax[i] > sensorMin[i]) {
        int32_t value = (int32_t)(raw - sensorMin[i]) * 1000 /
                        (sensorMax[i] - sensorMin[i]);
        if (value < 0)
          value = 0;
        if (value > 1000)
          value = 1000;
        sensorValues[i] = (uint16_t)value;
      } else {
        sensorValues[i] = 0;
      }
    } else {
      sensorValues[i] = raw;
    }
  }
}

// Calibrate sensors - call this while moving robot over line
void calibrateSensors() {
  uint16_t rawValues[SENSOR_COUNT];

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    rawValues[i] = readSensor(i);

    // Update min/max
    if (rawValues[i] < sensorMin[i]) {
      sensorMin[i] = rawValues[i];
    }
    if (rawValues[i] > sensorMax[i]) {
      sensorMax[i] = rawValues[i];
    }
  }
}

// Initialize calibration values
void initCalibration() {
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    sensorMin[i] = 4095; // Max ADC value for ESP32 (12-bit)
    sensorMax[i] = 0;
  }
  sensorCalibrated = false;
}

// Calculate line position (0 to (SENSOR_COUNT-1)*1000)
// Returns weighted average position of line
// For 14 sensors: range is 0 to 13000, center is 6500
// Mimics QTR library behavior: returns 0 or max when line is lost
uint16_t readLinePosition(bool whiteLine = false) {
  readSensorsCalibrated();

  uint32_t avg = 0; // Weighted sum
  uint32_t sum = 0; // Sum of sensor values
  static uint16_t lastPosition = (SENSOR_COUNT - 1) * 500; // Default to center

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    uint16_t value = sensorValues[i];

    // Invert if detecting white line on black background
    if (whiteLine) {
      value = 1000 - value;
    }

    // Only include sensors above threshold in weighted average
    if (value > threshold) {
      avg += (uint32_t)value * i * 1000;
      sum += value;
    }
  }

  if (sum == 0) {
    // Line lost - return extreme value based on last position
    // This causes PID to output max correction
    if (lastPosition < (SENSOR_COUNT - 1) * 500) {
      return 0; // Line was on left, return 0 to make PID turn left hard
    } else {
      return (SENSOR_COUNT - 1) *
             1000; // Line was on right, return max to turn right hard
    }
  }

  lastPosition = avg / sum;
  return lastPosition;
}

// Initialize multiplexer pins
void initSensorArray() {
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  pinMode(MUX_SIG_PIN, INPUT);

  // Set ADC resolution to 12 bits (0-4095)
  analogReadResolution(12);

  // Initialize calibration
  initCalibration();

  Serial.println("14-sensor line array initialized");
}

bool parseMessageValue(const String &message, float &outValue) {
  String valuePart = message.substring(2);
  valuePart.trim();

  while (valuePart.length() > 0 &&
         (valuePart[0] == ':' || valuePart[0] == '=' || valuePart[0] == ',')) {
    valuePart.remove(0, 1);
    valuePart.trim();
  }

  if (valuePart.length() == 0) {
    return false;
  }

  char *endPtr = nullptr;
  outValue = strtof(valuePart.c_str(), &endPtr);
  return endPtr != valuePart.c_str();
}

void setup() {
  Serial.begin(115200);

  bufferSwapMutex = xSemaphoreCreateMutex();

  // Create the SD write task
  xTaskCreatePinnedToCore(sdWriteTask, "SDWriteTask", 4096, NULL, 1, NULL, 1);

  // Create the telemetry task
  xTaskCreatePinnedToCore(telemetryTask, "TelemetryTask", 4096, NULL, 1, NULL,
                          1);

  Wire.begin(I2C1_SDA, I2C1_SCL);
  encoder1.begin();
  encoder1.setDirection(AS5600_CLOCK_WISE);

  I2C_2.begin(I2C2_SDA, I2C2_SCL);
  encoder2.begin();
  encoder2.setDirection(AS5600_COUNTERCLOCK_WISE);

  if (encoder1.isConnected()) {
    Serial.println("AS5600 #1 detected");
  } else {
    Serial.println("No communication with AS5600 #1!");
    while (true)
      ;
  }

  if (encoder2.isConnected()) {
    Serial.println("AS5600 #2 detected");
  } else {
    Serial.println("No communication with AS5600 #2!");
    while (true)
      ;
  }

  // Initialize BMI160 on I2C_2 bus
  if (bmi160.softReset() != BMI160_OK) {
    Serial.println("BMI160 reset failed");
    while (true)
      ;
  }

  if (bmi160.I2cInit(bmi160_i2c_addr) != BMI160_OK) {
    Serial.println("BMI160 initialization failed");
    while (true)
      ;
  }
  Serial.println("BMI160 initialized");

  // Initialize motor pins
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  pinMode(MODE_PIN, OUTPUT);
  digitalWrite(MODE_PIN, HIGH);

  // Initialize 14-sensor line array with multiplexer
  initSensorArray();

  delay(1000);
  Serial.println("Initializing microSD card...");

  spiSD.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS);

  if (!SD.begin(PIN_CS, spiSD)) {
    Serial.println("Failed to initialize SD card!");
    while (true)
      ;
  }
  Serial.println("SD card ready.");

  WiFi.softAP(ssid, password);

  if (udp.listen(1234)) {
    Serial.print("UDP Listening on IP: ");
    Serial.println(WiFi.localIP());
  }

  udp.onPacket([](AsyncUDPPacket packet) {
    char *tmpStr = (char *)malloc(packet.length() + 1);
    memcpy(tmpStr, packet.data(), packet.length());
    tmpStr[packet.length()] = '\0';
    String message = String(tmpStr);
    message.trim();
    String firstTwoLetters = message.substring(0, 2);
    firstTwoLetters.toUpperCase();
    free(tmpStr);
    Serial.println(message);
    if (message == "Cal") {
      calibration();
    }
    if (message == "Reset") {
      ESP.restart();
    }
    if (message == "Start") {
      neopixelWrite(RGB_LED_PIN, RGB_BRIGHTNESS, 0, RGB_BRIGHTNESS);
      onoff = 1;
      telemetryActive = true;
      Serial.println("Start command received");
    }
    if (message == "Stop") {
      neopixelWrite(RGB_LED_PIN, 255, 159, 0);
      onoff = 0;
      telemetryActive = false;
      motor_stop();
      flushLogBuffer(); // Flush remaining logs
      Serial.println("Stop command received");
    }
    if (message == "Sensors") {
      request_sensorsRaw();
    }
    if (message == "Params") {
      request_params();
    }
    if (firstTwoLetters == "KP") {
      float numericalValue;
      if (parseMessageValue(message, numericalValue)) {
        Kp = numericalValue;
        Serial.print("Kp set to: ");
        Serial.println(Kp);
      }
    }
    if (firstTwoLetters == "KI") {
      float numericalValue;
      if (parseMessageValue(message, numericalValue)) {
        Ki = numericalValue;
        Serial.print("Ki set to: ");
        Serial.println(Ki);
      }
    }
    if (firstTwoLetters == "KD") {
      float numericalValue;
      if (parseMessageValue(message, numericalValue)) {
        Kd = numericalValue;
        Serial.print("Kd set to: ");
        Serial.println(Kd);
      }
    }
    if (firstTwoLetters == "MA") {
      float numericalValue;
      if (parseMessageValue(message, numericalValue)) {
        maxspeedb = numericalValue;
        maxspeeda = numericalValue;
        Serial.print("MaxSpeed set to: ");
        Serial.println(maxspeedb);
      }
    }
    if (firstTwoLetters == "BA") {
      float numericalValue;
      if (parseMessageValue(message, numericalValue)) {
        basespeedb = numericalValue;
        basespeeda = numericalValue;
        if (maxspeeda < basespeeda) {
          maxspeeda = basespeeda;
        }
        if (maxspeedb < basespeedb) {
          maxspeedb = basespeedb;
        }
        Serial.print("BaseSpeed set to: ");
        Serial.println(basespeedb);
      }
    }
    if (firstTwoLetters == "TU") {
      float numericalValue;
      if (parseMessageValue(message, numericalValue)) {
        TurnSpeed = numericalValue;
        Serial.print("TurnSpeed set to: ");
        Serial.println(TurnSpeed);
      }
    }
    if (firstTwoLetters == "TH") {
      float numericalValue;
      if (parseMessageValue(message, numericalValue)) {
        threshold = numericalValue;
        Serial.print("Threshold set to: ");
        Serial.println(threshold);
      }
    }
  });

  delay(500);
}

void loop() {
  if (onoff == 1) {
    robot_control();
  }
  if (onoff == 0) {
    motor_stop();
    // Debug: encoder readings
    Serial.printf("ENC1: %d  ENC2: %d\n", encoder1.readAngle(),
                  encoder2.readAngle());
    delay(200);
  }
}

void robot_control() {
  static uint8_t lineLostStreak = 0;
  const uint8_t lineLostDebounceCycles = 4;

  // Read line position (0 to 13000 for 14 sensors, center = 6500)
  uint16_t position = readLinePosition(false); // false = black line on white
  int error = 6500 - position;                 // Center position for 14 sensors

  // Count sensors seeing the line and calculate average position
  int cnt = 0;
  float sum = 0;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (sensorValues[i] >= threshold) {
      cnt++;
      sum = sum + i;
    }
  }

  // LINE LOST: No sensors see the line - spin in place to find it
  if (cnt == 0) {
    lineLostStreak++;

    // Ignore short dropouts/noise to avoid oscillation and shaking
    if (lineLostStreak < lineLostDebounceCycles) {
      speedcontrol(basespeeda, basespeedb);
      return;
    }

    // Use position to determine which way to spin
    // position 0 = line was on left, position 13000 = line was on right
    if (position < 6500) {
      // Line was on left - spin left in place
      left_brake(TurnSpeed, TurnSpeed);
    } else {
      // Line was on right - spin right in place
      right_brake(TurnSpeed, TurnSpeed);
    }
    return;
  }

  lineLostStreak = 0;

  // Line detected: use PID for smooth forward tracking
  PID(error);
}

void PID(int error) {
  const float minTrackSpeed = 0.0f;

  int P = error;
  I += error;
  int D = error - lastError;
  lastError = error;
  Pvalue = (Kp / pow(10, multiP)) * P;
  Ivalue = (Ki / pow(10, multiI)) * I;
  Dvalue = (Kd / pow(10, multiD)) * D;

  float motorspeed = Pvalue + Ivalue + Dvalue;

  // Correction direction flipped: + for A, - for B
  float motorspeeda = basespeeda - motorspeed;
  float motorspeedb = basespeedb + motorspeed;

  if (motorspeeda > maxspeeda) {
    motorspeeda = maxspeeda;
  }
  if (motorspeedb > maxspeedb) {
    motorspeedb = maxspeedb;
  }
  if (motorspeeda < minTrackSpeed) {
    motorspeeda = minTrackSpeed;
  }
  if (motorspeedb < minTrackSpeed) {
    motorspeedb = minTrackSpeed;
  }

  speedcontrol(motorspeeda, motorspeedb);
}

void speedcontrol(int mota, int motb) {
  if (mota >= 0 && motb >= 0) {
    forward_brake(mota, motb);
  }
  if (mota < 0 && motb >= 0) {
    mota = 0 - mota;
    right_brake(mota, motb);
  }
  if (mota >= 0 && motb < 0) {
    motb = 0 - motb;
    left_brake(mota, motb);
  }
}

void forward_brake(int posa, int posb) {
  digitalWrite(AIN1_PIN, HIGH);
  digitalWrite(BIN1_PIN, HIGH);
  analogWrite(AIN2_PIN, 255 - posa);
  analogWrite(BIN2_PIN, 255 - posb);
}

void left_brake(int posa, int posb) {
  digitalWrite(AIN1_PIN, LOW);
  digitalWrite(BIN1_PIN, HIGH);
  analogWrite(AIN2_PIN, 255 - posa);
  analogWrite(BIN2_PIN, 255 - posb);
}

void right_brake(int posa, int posb) {
  digitalWrite(AIN1_PIN, HIGH);
  digitalWrite(BIN1_PIN, LOW);
  analogWrite(AIN2_PIN, 255 - posa);
  analogWrite(BIN2_PIN, 255 - posb);
}

void motor_stop() {
  // Set direction pins LOW and PWM to 0 to completely stop motors
  // Must use analogWrite(0) for PWM pins since they're configured for LEDC
  digitalWrite(AIN1_PIN, LOW);
  digitalWrite(BIN1_PIN, LOW);
  analogWrite(AIN2_PIN, 0);
  analogWrite(BIN2_PIN, 0);
}

void calibration() {
  neopixelWrite(RGB_LED_PIN, RGB_BRIGHTNESS, 0, 0); // Red during calibration

  // Reset calibration values
  initCalibration();

  // Calibrate for ~2 seconds (400 iterations * 5ms)
  // Move robot over the line during this time
  for (uint16_t i = 0; i < 400; i++) {
    calibrateSensors();
    delay(10);
  }

  sensorCalibrated = true;

  // Set threshold to 500 (50% of calibrated range) like the original
  threshold = 500;

  neopixelWrite(RGB_LED_PIN, 255, 159, 0); // Orange when done

  // Print calibration results
  Serial.println("Calibration complete:");
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    Serial.printf("Sensor %d: min=%d, max=%d\n", i, sensorMin[i], sensorMax[i]);
  }
  Serial.printf("Threshold automatically set to: %.0f\n", threshold);
}

void request_sensorsRaw() {
  readSensorsCalibrated(); // Update sensor values
  String sensors = "Sensor ";
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    sensors += String(sensorValues[i]) + " ";
  }
  udp.broadcast(sensors.c_str());
}

void request_params() {
  String params =
      "Kp: " + String(Kp) + " Ki: " + String(Ki) + " Kd: " + String(Kd) +
      " Max: " + String(maxspeedb) + " Base: " + String(basespeedb) +
      " Turn: " + String(TurnSpeed) + " Lost_th: " + String(threshold);
  udp.broadcast(params.c_str());
}

/*
log.txt sample:
65993 224 1568
...
*/
