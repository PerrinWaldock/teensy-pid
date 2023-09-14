#include <stdint.h>

#define ADC_VREF 5

#define ADC_CNVST 28
#define ADC_SDO 1
#define ADC_SCK 27

#define ADC_BITS2VOLTS(x) (float)x*ADC_VREF/635536
#define ADC_VOLTS2BITS(x) (uint16_t)(x*635536/ADC_VREF)


uint16_t readADC();
float readADCVolts();
void calibrateADC(); //TODO see page 39 of datasheet
