#include "web_server.h"
#include "web_ui.h"
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <SD.h>
#include <WiFi.h>

#include "encoder.h"
#include "imu.h"
#include "line_follower.h"
#include "map.h"
#include "motor.h"
#include "nvs_params.h"
#include "odometry.h"
#include "robot_config.h"

extern LineFollower_t GRUZIK;
extern motor_t Motor_L;
extern motor_t Motor_R;
extern IMU_t IMU;
extern Map_t GruzikMap;
extern Odometry_t Odometry;

extern volatile bool telemetryActive;
extern SemaphoreHandle_t spiSem;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
             AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("WebSocket client #%u connected\n", client->id());
    WebServer_BroadcastParams();
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->final && info->index == 0 && info->len == len &&
        info->opcode == WS_TEXT) {
      data[len] = 0;
      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, (char *)data);
      if (error)
        return;

      const char *msgType = doc["type"];
      if (strcmp(msgType, "mode") == 0) {
        const char *action = doc["action"];
        if (strcmp(action, "start") == 0) {
          GRUZIK.PowerMode = Start;
          GRUZIK.state = PidFollowing;
        } else if (strcmp(action, "stop") == 0) {
          GRUZIK.PowerMode = Stop;
          telemetryActive = false;
          extern void flushLogBuffer();
          flushLogBuffer();
          xSemaphoreTake(spiSem, portMAX_DELAY);
          if (SdMapFile) SdMapFile.close();
          xSemaphoreGive(spiSem);
        } else if (strcmp(action, "force_fwd") == 0)
          GRUZIK.PowerMode = ForceForward;
        else if (strcmp(action, "mapping") == 0) {
          GRUZIK.state = Mapping;
          GRUZIK.PowerMode = Start;
          Map_Reset(&GruzikMap);
          GruzikMap.Mapping = 1;
          telemetryActive = true;
          // Close any leftover read-handle from a previous playback before
          // removing the file — SD.remove() on an open handle is undefined.
          xSemaphoreTake(spiSem, portMAX_DELAY);
          if (SdMapFile) SdMapFile.close();
          SD.remove(ROBOT_MAP_OUTPUT_FILE);
          xSemaphoreGive(spiSem);
        } else if (strcmp(action, "unmapping") == 0) {
          GRUZIK.state = UnMapping;
          GRUZIK.PowerMode = Start;
          xSemaphoreTake(spiSem, portMAX_DELAY);
          Map_BeginPlayback(&GruzikMap);
          xSemaphoreGive(spiSem);
        } else if (strcmp(action, "speedunmapping") == 0) {
          GRUZIK.state = SpeedUnMapping;
          GRUZIK.PowerMode = Start;
          xSemaphoreTake(spiSem, portMAX_DELAY);
          Map_BeginPlayback(&GruzikMap);
          xSemaphoreGive(spiSem);
        }
      } else if (strcmp(msgType, "debug_cmd") == 0) {
        const char *action = doc["action"];
        if (strcmp(action, "recal_imu") == 0) {
          // Restart non-blocking gyro-Z bias calibration (500 ticks @ 1 kHz)
          IMU_StartCalibration(&IMU, 500);
        }
      } else if (strcmp(msgType, "cmd") == 0) {
        const char *action = doc["action"];
        if (strcmp(action, "cal") == 0) {
          // Start basic calibration logic
          GRUZIK.sensorCalibrated = false;
          for (int i = 0; i < 14; i++) {
            GRUZIK.SensorMinValues[i] =
                1023; // Will be properly implemented via analogRead loop in
                      // control task if needed
            GRUZIK.SensorMaxValues[i] = 0;
          }
        } else if (strcmp(action, "params") == 0) {
          Save_Params(&GRUZIK, &GruzikMap);
        }
      } else if (strcmp(msgType, "set_param") == 0) {
        const char *param = doc["param"];
        float value = doc["value"];
        if (strcmp(param, "baseSpeed") == 0) {
          GRUZIK.Base_speed_L = value;
          GRUZIK.Base_speed_R = value;
        } else if (strcmp(param, "kp") == 0) {
          GRUZIK.Kp = value;
        } else if (strcmp(param, "kd") == 0) {
          GRUZIK.Kd = value;
        } else if (strcmp(param, "kpMult") == 0) {
          GRUZIK.Kp_Multiplier = value;
        } else if (strcmp(param, "kdMult") == 0) {
          GRUZIK.Kd_Multiplier = value;
        }
      }
    }
  }
}

void WebServer_Init() {
  // Start SoftAP
  WiFi.softAP("LFMUS", "rabarbar");
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  ws.onEvent(onEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", web_ui_html);
  });

  server.on("/map_raw", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SD, "/GRUZIK.txt", "text/plain");
  });

  server.on("/map_opt", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (SD.exists(ROBOT_MAP_INPUT_FILE)) {
      request->send(SD, ROBOT_MAP_INPUT_FILE, "text/plain");
    } else {
      // Fallback to raw map to prevent 404 if no offline optimization happened
      request->send(SD, ROBOT_MAP_OUTPUT_FILE, "text/plain");
    }
  });

  server.begin();
}

void WebServer_BroadcastTelemetry() {
  if (ws.count() == 0)
    return;

  JsonDocument doc;
  doc["type"] = "telemetry";

  // --- Motor / Encoder ---
  doc["speedL"] = Motor_L.MetersPerSecond;
  doc["speedR"] = Motor_R.MetersPerSecond;
  doc["encL"] = Encoder1_GetCount(); // full 32-bit PCNT count (left)
  doc["encR"] = Encoder2_GetCount(); // full 32-bit PCNT count (right)
  // Delta since last Motor_CalculateSpeed tick (counts per 1 ms)
  doc["encLDelta"] =
      (int32_t)(Motor_L.EncoderValue - Motor_L.PreviousEncoderValue);
  doc["encRDelta"] =
      (int32_t)(Motor_R.EncoderValue - Motor_R.PreviousEncoderValue);

  // --- IMU ---
  doc["gyroZ"] = IMU.Gyro_Z;            // bias-corrected dps
  doc["gyroRaw"] = IMU.Gyro_Z_RawDps;   // raw dps (before bias subtraction)
  doc["gyroBias"] = IMU.Gyro_Z_BiasDps; // calibrated offset
  doc["imuInit"] = (int)IMU.initialized;
  doc["imuCal"] = (int)IMU.calibrating;
  doc["calProgress"] =
      IMU.calibrating
          ? (int)(100 * IMU.cal_samples_done / IMU.cal_samples_target)
          : 100;

  // --- Odometry ---
  doc["odomX"] = Odometry.pose.x_m;
  doc["odomY"] = Odometry.pose.y_m;
  doc["odomYaw"] = Odometry.pose.yaw_rad * (180.0f / M_PI); // degrees

  // --- Line sensor ---
  doc["lineErr"] = GRUZIK.Last_error;

  // --- Robot state ---
  if (GRUZIK.PowerMode == Stop || GRUZIK.PowerMode == Initialization) {
    doc["state"] = "Stopped";
  } else if (GRUZIK.PowerMode == ForceForward) {
    doc["state"] = "Force Forward";
  } else {
    switch (GRUZIK.state) {
    case PidFollowing:
      doc["state"] = "Following Line";
      break;
    case Mapping:
      doc["state"] = "Mapping";
      break;
    case UnMapping:
      doc["state"] = "Driving on Map";
      break;
    case SpeedUnMapping:
      doc["state"] = "Fast Playback";
      break;
    default:
      doc["state"] = "Unknown";
    }
  }

  String output;
  serializeJson(doc, output);
  ws.textAll(output);
}

void WebServer_BroadcastParams() {
  if (ws.count() == 0)
    return;

  JsonDocument doc;
  doc["type"] = "params";
  doc["baseSpeed"] = GRUZIK.Base_speed_L;
  doc["kp"] = GRUZIK.Kp;
  doc["kd"] = GRUZIK.Kd;
  doc["kpMult"] = GRUZIK.Kp_Multiplier;
  doc["kdMult"] = GRUZIK.Kd_Multiplier;

  String output;
  serializeJson(doc, output);
  ws.textAll(output);
}
