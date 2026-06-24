#ifndef MAP_H
#define MAP_H

#include <Arduino.h>
#include <SD.h>
#include "odometry.h"

#define MAP_SIZE 3000

typedef struct
{
    uint8_t Mapping;
    uint8_t PlaybackActive;
    float Map[MAP_SIZE][2];

    float Kk;
    float WayPoint[2][5];
    float WayPointError;
    float WayPointSpeedCompensation;

    float Ti;
    float Ri;
    float Xri;
    float PreviousXri;
    float Yri;
    float PreviousYri;
    float Ori;
    float PreviousOri;
    float OriIMU;
    float AlhphaOri;
    float Qri[3];
    float Pci[2];
    float PreviousPci[2];
    float Si;
    float Ai;
    float PreviousAi;
    float Ki;

    float StartXri;
    float StartYri;
    float StartOri;
    float StartPci[2];
    uint16_t RecordedPoints;

    float SetX;
    float SetY;
    float PreviousSetX;
    float PreviousSetY;
    float SetRotation;
    float SetSpeed;
    float DefaultPlaybackSpeed;

    float p;
    float i;
    float d;
    float ErrorSum;
    float LastError;
    float FilteredErrorDerivative;
    float mi;

    uint8_t loopNumber;
    uint8_t TargetValid;
    uint8_t StartValid;
    uint8_t AutoClosed;
} Map_t;

extern File SdMapFile;

void Map_Reset(Map_t *map);
uint8_t Map_BeginPlayback(Map_t *map);
uint8_t MapUpdate(Map_t *map, const Pose2D_t *pose, float speed_mps, float error_p, float error_d);
uint8_t DriveOnMap(Map_t *map, const Pose2D_t *pose);

#endif // MAP_H
