#include "setpointmanager.h"
#include "pins.h"

SetpointManager::SetpointManager()
{
	#if INPUT_MODE == DIGITAL_INPUT
		pinMode(PIN_REFERENCE0, INPUT);
		pinMode(PIN_REFERENCE1, INPUT);
	#elif INPUT_MODE == ANALOG_INPUT
		pinMode(PIN_REFERENCE, INPUT);
		analogReadAveraging(ANALOG_READ_AVERAGES);
		analogReadResolution(ANALOG_REFERENCE_RESOLUTION);
	#endif

	for (uint8_t i = 0; i < NUM_SETPOINTS; i++)
	{
		setPoints[i] = DEFAULT_SETPOINT;
	}
}

uint16_t SetpointManager::getSetPoint()
{
	#if INPUT_MODE == DIGITAL_INPUT
		return setPoints[getSetPointIndex()];
	#elif INPUT_MODE == ANALOG_INPUT
		setPoints[getSetPointIndex()] = analogRead(PIN_REFERENCE) << (ADC_BITS - ANALOG_REFERENCE_RESOLUTION);
		return setPoints[getSetPointIndex()];
	#elif INPUT_MODE == SOFTWARE_INPUT
		return *setPoints;
	#else
		return DEFAULT_SETPOINT;
	#endif 
}

uint16_t SetpointManager::getSetPoint(uint8_t index)
{
	if (index < NUM_SETPOINTS)
	{
		return setPoints[index];
	}
	else
	{
		return 0;
	}
}

void SetpointManager::setSetPoint(uint8_t index, uint16_t value)
{
	if (index < NUM_SETPOINTS)
	{
		setPoints[index] = value;
	}
}

uint8_t SetpointManager::getSetPointIndex()
{
	#if INPUT_MODE == DIGITAL_INPUT
		return ((digitalRead(PIN_REFERENCE1) << 1) | digitalRead(PIN_REFERENCE0));
	#else
		return 0;
	#endif
}

uint16_t* SetpointManager::getSetPoints()
{
	return setPoints;
}