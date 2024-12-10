#ifndef DATAMANGER_H
#define DATAMANGER_H

#include <CircularBuffer.hpp> //prepackaged Circular Buffer

typedef enum{
	LOG_OFF, //don't log output
	LOG_SINGLE, //log until full
	LOG_CONTINUOUS	//continue logging, overwriting old data
} LogState;

#if TEENSY==TEENSY32
	const uint32_t TOTAL_LOG_SPACE = 1000;
#elif TEENSY==TEENSY40
	const uint32_t TOTAL_LOG_SPACE = 100000;
#endif
const uint32_t INPUT_LOG_SIZE = TOTAL_LOG_SPACE/2/sizeof(uint16_t);

typedef struct {
	LogState state;
	CircularBuffer<uint16_t, INPUT_LOG_SIZE> input;
	CircularBuffer<uint16_t, INPUT_LOG_SIZE> output;
} DataLog;

DataLog GetNewDataLog();

#endif