#include "main.h"
#include <EEPROM.h>     //for permanently saving settings
#include <CircularBuffer.hpp> //prepackaged Circular Buffer
#include <IntervalTimer.h>
#include <Arduino.h>

#include "settings.h"
#include "fpid.h"
#include "commands.h"
#include "dac.h"
#include "adc.h"
#include "serialmanager.h"
#include "pins.h"

/*
Things left to do
- purge macros and constants
- set up timing of feedback loop and command parsing
- create readAdc function and writeDac function
- look into async ADC reads
	- https://github.com/hideakitai/TsyDMASPI
- create some sort of printPidState function
*/


#if RECORD_INPUT
	typedef enum{
		LOG_OFF, //don't log output
		LOG_SINGLE, //log until full
		LOG_CONTINUOUS	//continue logging, overwriting old data
	} LogState;
	const uint32_t INPUT_LOG_SIZE = 150000/2;
	CircularBuffer<uint16_t, INPUT_LOG_SIZE> inputLog;
	CircularBuffer<uint16_t, INPUT_LOG_SIZE> outputLog;
	LogState logState = LOG_OFF;
#endif

#if INPUT_MODE == ANALOG_INPUT
	#define readAnalogReference() analogRead(PIN_REFERENCE) << (ADC_BITS - ANALOG_REFERENCE_RESOLUTION);
#endif

// TODO move to its own file
uint16_t getSetPoint();
uint16_t getFeedback();

//IntervalTimer pidTimer; //used for timing the pid feedback loop
//IntervalTimer readTimer; //used for timing the serial read code


uint16_t setPoints[NUM_SETPOINTS];
uint8_t readAveragesPower = 0;
bool printOutput = false;

Extrema setpointLimits = {	inputVolts2int(DEFAULT_MIN_SETPOINT_VOLTS), 
							inputVolts2int(DEFAULT_MAX_SETPOINT_VOLTS)};
Extrema outputLimits = {	outputVolts2int(DEFAULT_MIN_OUTPUT_VOLTS), 
							outputVolts2int(DEFAULT_MAX_OUTPUT_VOLTS)};
PidParams params = {DEFAULT_KI, 
					DEFAULT_KP, 
					DEFAULT_KD, 
					DEFAULT_SAMPLE_PERIOD_US, 
					setpointLimits, 
					outputLimits};

FPid pidController = FPid(params, getSetPoint, getFeedback, writeDAC);
EepromManager eepromManager = EepromManager(pidController, setPoints, readAveragesPower);
CommandParser commandParser = CommandParser(pidController, eepromManager, setPoints, readAveragesPower, printOutput);

void setup()
{
	//sets pin states
	#if INPUT_MODE == DIGITAL_INPUT
		pinMode(PIN_REFERENCE0, INPUT);
		pinMode(PIN_REFERENCE1, INPUT);
	#elif INPUT_MODE == ANALOG_INPUT
		pinMode(PIN_REFERENCE, INPUT);
		analogReadAveraging(ANALOG_READ_AVERAGES);
		analogReadResolution(ANALOG_REFERENCE_RESOLUTION);
	#endif
    initADC();
    initDAC();

	for (uint8_t i = 0; i < NUM_SETPOINTS; i++)
	{
		setPoints[i] = DEFAULT_SETPOINT;
	}
	
	//sets up serial
	#ifdef SERIAL_BAUD

		Serial.begin(SERIAL_BAUD);
		while (pidController.getError()) 
		{
            if(Serial)
            {
		        Serial.println("There is a configuration error!");
            }
            delay(1000);
		}

		Serial.println("PID Begin");
	#endif

	#if SAVE_DATA
		eepromManager.load();
	#endif
}

void loop()
{
	static elapsedMillis timeSinceLastSerialComm;

	pidController.iterate();

	#ifdef SERIAL_BAUD
		if (timeSinceLastSerialComm >= READ_PERIOD_MS)
		{
			if (isLineAvailable())
			{
				char* line = readLine();
				commandParser.parse(line);
			}
		}
	#endif
}

uint16_t getSetPoint()
{
	#if INPUT_MODE == DIGITAL_INPUT
		uint8_t index = (digitalRead(PIN_REFERENCE1) << 1) | digitalRead(PIN_REFERENCE0);
		return setPoints[index];
	#elif INPUT_MODE == ANALOG_INPUT
		return analogRead(PIN_REFERENCE) << (ADC_BITS - ANALOG_REFERENCE_RESOLUTION);
	#elif INPUT_MODE == SOFTWARE_INPUT
		return *setPoints;
	#else
		return DEFAULT_SETPOINT;
	#endif 
}

uint16_t getFeedback()
{
	return readADCMultiple(readAveragesPower);
}