#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>

#define bound(x, a, b) x < a ? a : x > b ? b : x
#define sprint(x) Serial.print(x)

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


template<typename T>
T stringToType(char* str);

template<typename T>
char* typeToString(T var);

#endif