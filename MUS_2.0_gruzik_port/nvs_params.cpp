#include "nvs_params.h"
#include <Preferences.h>

static Preferences preferences;

void Load_Params(LineFollower_t *LF, Map_t *map)
{
    preferences.begin("robot_params", true); // read-only

    LF->Kp = preferences.getFloat("ms_kp", 0.25f);
    LF->Kd = preferences.getFloat("ms_kd", 1.2f);
    LF->Kp_Multiplier = preferences.getFloat("ms_kp_mult", 1.0f);
    LF->Kd_Multiplier = preferences.getFloat("ms_kd_mult", 1.0f);
    
    LF->Base_speed_L = preferences.getFloat("ms_base_L", 1.0f);
    LF->Base_speed_R = preferences.getFloat("ms_base_R", 1.0f);
    LF->Max_speed_L = preferences.getFloat("ms_max_L", 2.5f);
    LF->Max_speed_R = preferences.getFloat("ms_max_R", 2.5f);

    LF->Sharp_bend_speed_left = preferences.getFloat("ms_sharp_L", -1.5f);
    LF->Sharp_bend_speed_right = preferences.getFloat("ms_sharp_R", 1.5f);
    LF->Bend_speed_left = preferences.getFloat("ms_bend_L", -0.5f);
    LF->Bend_speed_right = preferences.getFloat("ms_bend_R", 0.5f);
    
    LF->treshold = preferences.getUShort("thresh", 150);

    map->p = preferences.getFloat("map_p", 0.8f);
    map->i = preferences.getFloat("map_i", 0.0f);
    map->d = preferences.getFloat("map_d", 0.2f);
    map->DefaultPlaybackSpeed = preferences.getFloat("map_speed", 80.0f);

    preferences.end();
}

void Save_Params(const LineFollower_t *LF, const Map_t *map)
{
    preferences.begin("robot_params", false); // read-write

    preferences.putFloat("ms_kp", LF->Kp);
    preferences.putFloat("ms_kd", LF->Kd);
    preferences.putFloat("ms_kp_mult", LF->Kp_Multiplier);
    preferences.putFloat("ms_kd_mult", LF->Kd_Multiplier);
    
    preferences.putFloat("ms_base_L", LF->Base_speed_L);
    preferences.putFloat("ms_base_R", LF->Base_speed_R);
    preferences.putFloat("ms_max_L", LF->Max_speed_L);
    preferences.putFloat("ms_max_R", LF->Max_speed_R);

    preferences.putFloat("ms_sharp_L", LF->Sharp_bend_speed_left);
    preferences.putFloat("ms_sharp_R", LF->Sharp_bend_speed_right);
    preferences.putFloat("ms_bend_L", LF->Bend_speed_left);
    preferences.putFloat("ms_bend_R", LF->Bend_speed_right);

    preferences.putUShort("thresh", LF->treshold);

    preferences.putFloat("map_p", map->p);
    preferences.putFloat("map_i", map->i);
    preferences.putFloat("map_d", map->d);
    preferences.putFloat("map_speed", map->DefaultPlaybackSpeed);

    preferences.end();
}

void Load_Cal(LineFollower_t *LF)
{
    preferences.begin("robot_cal", true); // read-only

    size_t minLen = preferences.getBytes("sens_min", NULL, 0);
    if (minLen == sizeof(LF->SensorMinValues)) {
        preferences.getBytes("sens_min", LF->SensorMinValues, sizeof(LF->SensorMinValues));
    } else {
        for(int i=0; i<14; i++) LF->SensorMinValues[i] = 0;
    }

    size_t maxLen = preferences.getBytes("sens_max", NULL, 0);
    if (maxLen == sizeof(LF->SensorMaxValues)) {
        preferences.getBytes("sens_max", LF->SensorMaxValues, sizeof(LF->SensorMaxValues));
        LF->sensorCalibrated = true;
    } else {
        for(int i=0; i<14; i++) LF->SensorMaxValues[i] = 1023; // default 10-bit analog
        LF->sensorCalibrated = false;
    }

    preferences.end();
}

void Save_Cal(const LineFollower_t *LF)
{
    preferences.begin("robot_cal", false); // read-write

    preferences.putBytes("sens_min", LF->SensorMinValues, sizeof(LF->SensorMinValues));
    preferences.putBytes("sens_max", LF->SensorMaxValues, sizeof(LF->SensorMaxValues));

    preferences.end();
}
