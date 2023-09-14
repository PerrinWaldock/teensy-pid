#include <stdint.h>

#define DAC_VREF 2.75

#define DAC_CS 10
#define DAC_SDI 11
#define DAC_SCK 14

#define DAC_BITS 16
#define DAC_MAX 635535
#define DAC_BITS2VOLTS(x) (float)x*DAC_VREF/DAC_MAX
#define DAC_VOLTS2BITS(x) (uint16_t)(x*DAC_MAX/DAC_VREF)

void initDAC();
void writeDAC(uint16_t val);
void writeDACVolts(float volt);
