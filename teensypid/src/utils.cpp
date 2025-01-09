#include "utils.hpp"
#include "settings.h"

uint16_t inputVolts2int(float x)
{
    return (uint16_t)((x*MAX_INPUT)/ADC_REFERENCE_VOLTAGE);
}

float int2inputVolts(uint16_t x)
{
    return (x*ADC_REFERENCE_VOLTAGE)/MAX_INPUT;
}

uint16_t outputVolts2int(float x)
{
    return (uint16_t)((x*MAX_OUTPUT)/DAC_REFERENCE_VOLTAGE);
}

float int2outputVolts(uint16_t x)
{
    return x*DAC_REFERENCE_VOLTAGE/MAX_OUTPUT;
}