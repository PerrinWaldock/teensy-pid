#ifndef PID_PARAMS
#define PID_PARAMS

#include <Arduino.h>
#include "utils.h"

typedef struct
{
    float ki;
    float kp;
    float kd;
    uint32_t loopRate;
    Extrema setPointLimit;
    Extrema outputLimit;
} PidParams;

void savePID(PidParams params);

#endif