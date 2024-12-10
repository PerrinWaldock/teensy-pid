#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>

#define bound(x, a, b) (x < a ? a : (x > b ? b : x))
#define sprint(x) Serial.print(x)

const int US_TO_S = 1000000;

// TODO define volts to int and ints to volt here to be used in commandregistrar

typedef struct
{
    uint16_t min;
    uint16_t max;
} Extrema;

typedef struct
{
    uint16_t* array;
    size_t length;
} U16Array;

uint16_t outputVolts2int(float x);
float int2outputVolts(uint16_t x);
uint16_t inputVolts2int(float x);
float int2inputVolts(uint16_t x);

#endif