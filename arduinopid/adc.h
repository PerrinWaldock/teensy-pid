#include <stdint.h>

#define ADC_VREF 5

#define ADC_CNVST 30 //28
#define ADC_SDO 31 //1
#define ADC_SCK 32 //27

#ifndef ADC_BITS
    #define ADC_BITS 16
    const uint16_t ADC_MAX = (int32_t)(1 << ADC_BITS) - 1;
#endif
#define ADC_BITS2VOLTS(x) (float)x*ADC_VREF/ADC_MAX
#define ADC_VOLTS2BITS(x) (uint16_t)(x*ADC_MAX/ADC_VREF)

void initADC();
void calibrateADC(); //TODO see page 39 of datasheet
uint16_t readADC();
uint16_t readADCMultiple(uint8_t power);
float readADCVolts();
