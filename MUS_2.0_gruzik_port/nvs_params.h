#ifndef NVS_PARAMS_H
#define NVS_PARAMS_H

#include <Arduino.h>
#include "line_follower.h"
#include "map.h"

void Load_Params(LineFollower_t *LF, Map_t *map);
void Save_Params(const LineFollower_t *LF, const Map_t *map);

void Load_Cal(LineFollower_t *LF);
void Save_Cal(const LineFollower_t *LF);

#endif // NVS_PARAMS_H
