#include <Arduino.h>

#ifndef PRINTSTATE_H
#define PRINTSTATE_H
typedef struct {
	uint16_t printPeriod_ms;
	bool printOutput;
} PrintState;
#endif