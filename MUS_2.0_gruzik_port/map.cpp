#include "map.h"
#include "line_follower.h"
#include "robot_config.h"

extern LineFollower_t GRUZIK;
extern SemaphoreHandle_t spiSem;
File SdMapFile;

static float clampf(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

static void UpdateLineError(LineFollower_t *LF)
{
    int position = LineFollower_UpdateSensors(LF);
    float error = ((float)position - 6500.0f) / 1000.0f;

    LF->P = error;
    LF->Error_P = error;
    LF->D = error - LF->Last_error;
    LF->Error_D = LF->D;
    LF->Last_error = error;
}

static float LineConfidence(const LineFollower_t *LF)
{
    if (LF->actives == 0) return 0.0f;

    float center_error = fabs(6500.0f - (float)LF->SensorPosition);
    float confidence = 1.0f - clampf((center_error - 1800.0f) / 3600.0f, 0.0f, 1.0f);

    if (LF->actives > 7)
    {
        confidence *= 0.45f;
    }

    return confidence;
}

static uint8_t ReadMapLine_Unlocked(Map_t *map)
{
    char line[96];

    while (1)
    {
        uint8_t i = 0u;
        int sample = 0;

        while (i < (uint8_t)(sizeof(line) - 1u))
        {
            sample = SdMapFile.read();

            if (sample < 0) {
                // If we reach EOF, close the file
                SdMapFile.close();
                break;
            }

            if (sample == '\r') continue;
            if (sample == '\n') break;

            line[i++] = (char)sample;
        }
        line[i] = '\0';

        if (i == 0u && sample < 0) return 0u;

        char *token = strtok(line, " \t");
        if (token == NULL) continue;
        map->SetX = strtof(token, NULL);

        token = strtok(NULL, " \t");
        if (token == NULL) continue;
        map->SetY = strtof(token, NULL);

        token = strtok(NULL, " \t");
        if (token != NULL)
        {
            map->SetSpeed = clampf(strtof(token, NULL), -GRUZIK.Max_speed_L, GRUZIK.Max_speed_L);
        }
        else
        {
            map->SetSpeed = map->DefaultPlaybackSpeed;
        }

    }
}

static uint8_t ReadMapLine(Map_t *map)
{
    xSemaphoreTake(spiSem, portMAX_DELAY);
    uint8_t result = ReadMapLine_Unlocked(map);
    xSemaphoreGive(spiSem);
    return result;
}

static uint8_t AdvanceMapTarget(Map_t *map)
{
    map->PreviousSetX = map->SetX;
    map->PreviousSetY = map->SetY;
    return ReadMapLine(map);
}

static uint8_t ShouldAdvanceTarget(const Map_t *map, const Pose2D_t *pose, float distance_to_target)
{
    if (distance_to_target < ROBOT_MAP_TARGET_RADIUS_M) return 1u;
    if (map->TargetValid == 0u) return 0u;

    float segment_x = map->SetX - map->PreviousSetX;
    float segment_y = map->SetY - map->PreviousSetY;
    float segment_len_sq = (segment_x * segment_x) + (segment_y * segment_y);
    if (segment_len_sq < 0.000001f) return 0u;

    float pose_x = pose->x_m - map->PreviousSetX;
    float pose_y = pose->y_m - map->PreviousSetY;
    float projection = ((pose_x * segment_x) + (pose_y * segment_y)) / segment_len_sq;
    float lateral_error = fabs((pose_x * segment_y) - (pose_y * segment_x)) / sqrt(segment_len_sq);

    if ((projection > 1.02f) && (lateral_error < ROBOT_MAP_ADVANCE_LATERAL_M)) return 1u;
    if (projection > 1.30f) return 1u;

    return 0u;
}

extern void telemetryLog(const char* data);

static void WriteMapRecord(float x_ri, float y_ri, float x_pc, float y_pc, float speed_mps, float error_p, float error_d)
{
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\n",
             x_ri, y_ri, x_pc, y_pc, speed_mps, error_p, error_d);
    telemetryLog(buffer);
}

void Map_Reset(Map_t *map)
{
    float p = map->p;
    float i = map->i;
    float d = map->d;
    float default_speed = map->DefaultPlaybackSpeed;

    memset(map, 0, sizeof(*map));
    map->p = p;
    map->i = i;
    map->d = d;
    map->DefaultPlaybackSpeed = (default_speed > 0.0f) ? default_speed : 1.0f;
}

uint8_t Map_BeginPlayback(Map_t *map)
{
    map->PlaybackActive = 0u;
    map->ErrorSum = 0.0f;
    map->LastError = 0.0f;
    map->FilteredErrorDerivative = 0.0f;
    map->PreviousSetX = 0.0f;
    map->PreviousSetY = 0.0f;
    map->TargetValid = 0u;

    if (SdMapFile) SdMapFile.close();
    SdMapFile = SD.open(ROBOT_MAP_INPUT_FILE, FILE_READ);
    if (!SdMapFile) {
        SdMapFile = SD.open(ROBOT_MAP_OUTPUT_FILE, FILE_READ);
    }
    if (!SdMapFile) return 0u;

    if (ReadMapLine_Unlocked(map) == 0u) {
        SdMapFile.close();
        return 0u;
    }

    map->TargetValid = 1u;
    map->PlaybackActive = 1u;
    return 1u;
}

uint8_t MapUpdate(Map_t *map, const Pose2D_t *pose, float speed_mps, float error_p, float error_d)
{
    if (map->Mapping == 0u) return 1u;

    map->PreviousXri = map->Xri;
    map->PreviousYri = map->Yri;
    map->PreviousOri = map->Ori;
    map->PreviousPci[0] = map->Pci[0];
    map->PreviousPci[1] = map->Pci[1];

    map->Xri = pose->x_m;
    map->Yri = pose->y_m;
    map->Ori = pose->yaw_rad;
    map->Pci[0] = map->Xri + (ROBOT_SENSOR_OFFSET_M * cos(map->Ori));
    map->Pci[1] = map->Yri + (ROBOT_SENSOR_OFFSET_M * sin(map->Ori));

    if (map->StartValid == 0u)
    {
        map->StartValid = 1u;
        map->StartXri = map->Xri;
        map->StartYri = map->Yri;
        map->StartOri = map->Ori;
        map->StartPci[0] = map->Pci[0];
        map->StartPci[1] = map->Pci[1];
        map->PreviousXri = map->Xri;
        map->PreviousYri = map->Yri;
        map->PreviousOri = map->Ori;
        map->PreviousPci[0] = map->Pci[0];
        map->PreviousPci[1] = map->Pci[1];
        map->Ai = map->Ori;
        map->PreviousAi = map->Ori;
        WriteMapRecord(map->Xri, map->Yri, map->Pci[0], map->Pci[1], speed_mps, error_p, error_d);
        map->RecordedPoints++;
        return 1u;
    }

    float dx = map->Pci[0] - map->PreviousPci[0];
    float dy = map->Pci[1] - map->PreviousPci[1];
    map->Ti = sqrt((dx * dx) + (dy * dy));
    map->Si += map->Ti;

    if (map->Ti > 0.000001f)
    {
        map->PreviousAi = map->Ai;
        map->Ai = atan2(dy, dx);
        map->Ki = Odometry_NormalizeAngle(map->Ai - map->PreviousAi) / map->Ti;
    }

    if (map->loopNumber >= ROBOT_MAP_RECORD_EVERY_TICKS)
    {
        map->loopNumber = 0u;
        WriteMapRecord(map->Xri, map->Yri, map->Pci[0], map->Pci[1], speed_mps, error_p, error_d);
        map->RecordedPoints++;
    }

    map->loopNumber++;
    float close_dx = map->Xri - map->StartXri;
    float close_dy = map->Yri - map->StartYri;
    float close_distance = sqrt((close_dx * close_dx) + (close_dy * close_dy));
    
    if ((map->Si >= ROBOT_MAP_CLOSE_MIN_DISTANCE_M) &&
        (map->RecordedPoints >= ROBOT_MAP_CLOSE_MIN_POINTS) &&
        (close_distance <= ROBOT_MAP_CLOSE_RADIUS_M))
    {
        WriteMapRecord(map->StartXri, map->StartYri, map->StartPci[0], map->StartPci[1], speed_mps, error_p, error_d);
        map->RecordedPoints++;
        map->AutoClosed = 1u;
        map->Mapping = 0u;
        return 0u;
    }

    return 1u;
}

uint8_t DriveOnMap(Map_t *map, const Pose2D_t *pose)
{
    if (map->PlaybackActive == 0u) return 0u;

    map->Xri = pose->x_m;
    map->Yri = pose->y_m;
    map->Ori = pose->yaw_rad;

    float dx = map->SetX - map->Xri;
    float dy = map->SetY - map->Yri;
    float distance_to_target = sqrt((dx * dx) + (dy * dy));

    while (ShouldAdvanceTarget(map, pose, distance_to_target) != 0u)
    {
        if (AdvanceMapTarget(map) == 0u)
        {
            map->PlaybackActive = 0u;
            return 0u;
        }

        dx = map->SetX - map->Xri;
        dy = map->SetY - map->Yri;
        distance_to_target = sqrt((dx * dx) + (dy * dy));
    }

    float segment_x = map->SetX - map->PreviousSetX;
    float segment_y = map->SetY - map->PreviousSetY;
    float segment_len = sqrt((segment_x * segment_x) + (segment_y * segment_y));
    float error;

    if (segment_len > 0.000001f)
    {
        map->SetRotation = atan2(segment_y, segment_x);
        float pose_x = map->Xri - map->PreviousSetX;
        float pose_y = map->Yri - map->PreviousSetY;
        float cross_track = ((pose_x * segment_y) - (pose_y * segment_x)) / segment_len;
        cross_track = clampf(cross_track, -ROBOT_PLAYBACK_CROSSTRACK_LIMIT_M, ROBOT_PLAYBACK_CROSSTRACK_LIMIT_M);
        float speed_ratio = clampf(fabs(map->SetSpeed) / 2.5f, 0.0f, 1.0f);
        float lookahead = ROBOT_PLAYBACK_LOOKAHEAD_BASE_M + (ROBOT_PLAYBACK_LOOKAHEAD_SPEED_M * speed_ratio);
        error = Odometry_NormalizeAngle((map->SetRotation - map->Ori) + atan2(cross_track, lookahead));
    }
    else
    {
        map->SetRotation = atan2(dy, dx);
        error = Odometry_NormalizeAngle(map->SetRotation - map->Ori);
    }

    map->ErrorSum = clampf(map->ErrorSum + error, -3.0f, 3.0f);

    float error_derivative = Odometry_NormalizeAngle(error - map->LastError);
    map->LastError = error;
    map->FilteredErrorDerivative += ROBOT_PLAYBACK_D_FILTER_ALPHA * (error_derivative - map->FilteredErrorDerivative);

    float odom_correction = (map->p * error) + (map->i * map->ErrorSum) + (map->d * map->FilteredErrorDerivative);
    float odom_limit = clampf(fabs(map->SetSpeed) * 0.85f, 0.4f, ROBOT_PLAYBACK_ODOM_CORR_LIMIT_MS);
    odom_correction = clampf(odom_correction, -odom_limit, odom_limit);

    UpdateLineError(&GRUZIK);
    float line_correction = (GRUZIK.P * GRUZIK.Kp * GRUZIK.Kp_Multiplier) + (GRUZIK.D * GRUZIK.Kd * GRUZIK.Kd_Multiplier);
    line_correction = clampf(line_correction, -ROBOT_PLAYBACK_LINE_CORR_LIMIT_MS, ROBOT_PLAYBACK_LINE_CORR_LIMIT_MS);

    float motor_correction = -odom_correction;
    float base_speed = map->SetSpeed;
    float turn_speed_scale = 1.0f - (clampf(fabs(error), 0.0f, 1.0f) * ROBOT_PLAYBACK_TURN_SPEED_REDUCTION);
    base_speed *= turn_speed_scale;

    float line_confidence = LineConfidence(&GRUZIK);
    if (line_confidence > 0.0f)
    {
        motor_correction += line_correction * ROBOT_PLAYBACK_LINE_WEIGHT * line_confidence;
    }

    if (GRUZIK.actives == 0)
    {
        base_speed *= ROBOT_PLAYBACK_LOST_LINE_SPEED_MUL;
    }

    motor_correction = clampf(motor_correction, -ROBOT_PLAYBACK_CORR_LIMIT_MS, ROBOT_PLAYBACK_CORR_LIMIT_MS);

    float left_speed = clampf(base_speed + motor_correction, -GRUZIK.Max_speed_L, GRUZIK.Max_speed_L);
    float right_speed = clampf(base_speed - motor_correction, -GRUZIK.Max_speed_R, GRUZIK.Max_speed_R);

    motor_control(&GRUZIK, right_speed, left_speed);
    return 1u;
}
