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
#include "setpointmanager.h"

/*
TODO
	convert managers, etc to pointers
	clean up header files, make sure stuff is only defined in one place
	clean build errors
		fastpid
		type comparison warnings
	consider making different files classes
	implement input/output logging with circularbuffer
	look into async ADC reads
		Bhttps://github.com/hideakitai/TsyDMASPI
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
void printStats(FPid& pidController);


uint8_t readAveragesPower = 0;
bool printOutput = false;

// TODO convert each of these to pointers because they need to be initialized in the setup function
SetpointManager setpointManager = SetpointManager();
// TODO make these const
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
EepromManager eepromManager = EepromManager(pidController, setpointManager.getSetPoints(), readAveragesPower);
CommandParser commandParser = CommandParser(pidController, eepromManager, setpointManager, readAveragesPower, printOutput);

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
	static elapsedMillis timeSinceLastPrint;

	pidController.iterate();

	#ifdef SERIAL_BAUD
		if (timeSinceLastSerialComm >= SERIAL_CHECK_PERIOD_MS)
		{
			timeSinceLastSerialComm = 0;
			if (isLineAvailable())
			{
				char* line = readLine();
				commandParser.parse(line);
			}
		}

		if (timeSinceLastPrint >= PRINT_PERIOD_MS && printOutput)
		{
			timeSinceLastPrint = 0;
			printStats(pidController);
		}
	#endif
}

uint16_t getSetPoint()
{
	return setpointManager.getSetPoint();
}

uint16_t getFeedback()
{
	return readADCMultiple(readAveragesPower);
}

void printStats(FPid& pidController)
{
	char line[200];
	PidState state = pidController.getPidState();
	snprintf(line, 200, "t: %i, sp: %i fb: %i op: %i ff: %i", state.iterationTime, state.setPoint, state.feedBack, state.output, pidController.getFeedForwardValue());
	Serial.println(line);
}