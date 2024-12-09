#ifndef PID_PARAMS
#define PID_PARAMS

#include <Arduino.h>
#include "utils.hpp"

typedef struct
{
    float ki;
    float kp;
    float kd;
    uint32_t loopPeriod_us;
    Extrema setPointLimit;
    Extrema outputLimit;
} PidParams;

#endif