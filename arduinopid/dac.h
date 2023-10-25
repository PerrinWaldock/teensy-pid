#include <stdint.h>

#include "pins.h"

#define DAC_VREF 5

#ifndef DAC_BITS
    #define DAC_BITS 16
    const uint16_t DAC_MAX = (int32_t)(1 << DAC_BITS) - 1;
#endif
#define DAC_BITS2VOLTS(x) (float)x*DAC_VREF/DAC_MAX
#define DAC_VOLTS2BITS(x) (uint16_t)(x*DAC_MAX/DAC_VREF)

void initDAC();
void writeDAC(uint16_t val);
void writeDACVolts(float volt);
