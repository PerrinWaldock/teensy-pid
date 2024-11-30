#include "datalogger.h"

DataLog GetNewDataLog()
{
	return DataLog{LOG_OFF, CircularBuffer<uint16_t, INPUT_LOG_SIZE>(), CircularBuffer<uint16_t, INPUT_LOG_SIZE>()};
}