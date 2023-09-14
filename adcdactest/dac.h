#include <stdint.h>

#define DAC_VREF 2.75

#define DAC_CS 10
#define DAC_SDI 11
#define DAC_SCK 14

#define DAC_BITS2VOLTS(x) (float)x*DAC_VREF/635536
#define DAC_VOLTS2BITS(x) (uint16_t)(x*635536/DAC_VREF)

void initDAC();
void writeDAC(uint16_t val);
void writeDACVolts(float volt);
