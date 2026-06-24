#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

// ESP32-S3 Pin Definitions
#define PIN_MUX_SIG      1
#define PIN_MUX_S0       2
#define PIN_MUX_S1       3
#define PIN_MUX_S2       4
#define PIN_MUX_S3       5

#define PIN_I2C1_SDA     6
#define PIN_I2C1_SCL     7
#define PIN_I2C2_SDA     8
#define PIN_I2C2_SCL     9

#define PIN_RGB_LED      10

#define PIN_SPI_MOSI     11
#define PIN_SPI_SCK      12
#define PIN_SPI_MISO     13
#define PIN_IMU_CS       14

#define PIN_ENC1_A       6
#define PIN_ENC1_B       7
#define PIN_ENC2_A       8
#define PIN_ENC2_B       9

#define PIN_BIN2         15
#define PIN_BIN1         16
#define PIN_AIN2         17
#define PIN_AIN1         18

#define PIN_SD_CS        20
#define PIN_MODE         21

// Hardware Constants
#define ROBOT_WHEEL_BASE_M                 0.135f
#define ROBOT_SENSOR_OFFSET_M              0.125f
#define ROBOT_WHEEL_DIAMETER_M             0.026f
#define ROBOT_WHEEL_CIRCUMFERENCE_M        0.08168141f

// 2× quadrature (A-only CHANGE ISR): 7 PPR × 2 edges × 10 gear ratio = 140 CPR
// For 4× (A+B ISR), this would be 280 — but 4× has ISR cancellation at high RPM.
#define ROBOT_ENCODER_COUNTS_PER_WHEEL_REV 140.0f
#define ENCODER_SPEED_FILTER_ALPHA         0.35f

#define ROBOT_CONTROL_DT_S                  0.001f
#define ROBOT_PWM_MAX                       286.0f

#define ROBOT_GAP_BRIDGE_TIMEOUT_MS         4000u
#define ROBOT_GAP_BRIDGE_CENTER_ERROR       2000.0f
#define ROBOT_GAP_BRIDGE_EDGE_GUARD_COUNT   2u

#define ROBOT_LEFT_ENCODER_DIRECTION        1
#define ROBOT_RIGHT_ENCODER_DIRECTION       -1
#define ROBOT_ENCODER_YAW_DIRECTION         1.0f
#define ROBOT_IMU_Z_DIRECTION               1.0f

#define ROBOT_ODOMETRY_GYRO_WEIGHT          0.96f
#define ROBOT_GYRO_STATIONARY_DPS           2.0f
#define ROBOT_ENCODER_STATIONARY_M          0.00002f
#define ROBOT_GYRO_BIAS_LEARN_RATE          0.0005f

#define ROBOT_MAP_RECORD_EVERY_TICKS        40u
#define ROBOT_MAP_TARGET_RADIUS_M           0.075f
#define ROBOT_MAP_ADVANCE_LATERAL_M         0.120f
#define ROBOT_MAP_CLOSE_RADIUS_M            0.030f
#define ROBOT_MAP_CLOSE_MIN_DISTANCE_M      0.450f
#define ROBOT_MAP_CLOSE_MIN_POINTS          8u
#define ROBOT_PLAYBACK_LINE_WEIGHT          0.18f
#define ROBOT_PLAYBACK_LINE_CORR_LIMIT_MS   0.3f
#define ROBOT_PLAYBACK_ODOM_CORR_LIMIT_MS   1.0f
#define ROBOT_PLAYBACK_CORR_LIMIT_MS        1.2f
#define ROBOT_PLAYBACK_LOOKAHEAD_BASE_M     0.120f
#define ROBOT_PLAYBACK_LOOKAHEAD_SPEED_M    0.140f
#define ROBOT_PLAYBACK_CROSSTRACK_LIMIT_M   0.180f
#define ROBOT_PLAYBACK_D_FILTER_ALPHA       0.28f
#define ROBOT_PLAYBACK_TURN_SPEED_REDUCTION 0.35f
#define ROBOT_PLAYBACK_LOST_LINE_SPEED_MUL  0.85f

#define ROBOT_MAP_OUTPUT_FILE               "/GRUZIK.txt"
#define ROBOT_MAP_INPUT_FILE                "/map.txt"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// DEG_TO_RAD and RAD_TO_DEG are defined by Arduino.h

#endif // ROBOT_CONFIG_H
