#include <stdint.h>

#include "pins.h"

void initADC();
void calibrateADC(); //TODO see page 39 of datasheet
uint16_t readADC();
uint16_t readADCMultiple(uint8_t power);
float readADCVolts();
