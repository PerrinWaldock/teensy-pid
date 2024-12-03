#include "main.h"
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
#include "datalogger.h"

/*
TODO
	clean up header files, make sure stuff is only defined in one place
	consider making different files classes
		commandregistrar
			gets passed some sort of serial object (inherits from print)
			convert print statements to using printf
		also pass serial object to all other classes
	look into async ADC reads
		Bhttps://github.com/hideakitai/TsyDMASPI
*/

// TODO move to its own file
uint16_t getSetPoint();
uint16_t getFeedback();
void setOutput(uint16_t);
void printStats(FPid& pidController);

DataLog dataLog = GetNewDataLog();
uint8_t readAveragesPower = 0;
bool printOutput = false;

SetpointManager* setpointManager;
FPid* pidController;
EepromManager* eepromManager;
CommandParser* commandParser;

void setup()
{
    initADC();
    initDAC();
	
	//sets up serial
	#ifdef SERIAL_BAUD
		Serial.begin(SERIAL_BAUD);
		Serial.println("PID Begin");
	#endif

	setpointManager = new SetpointManager();

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
	pidController = new FPid(params, getSetPoint, getFeedback, writeDAC);
	eepromManager = new EepromManager(*pidController, setpointManager->getSetPoints(), readAveragesPower);
	commandParser = new CommandParser(*pidController, *eepromManager, *setpointManager, readAveragesPower, printOutput);

	#ifdef SERIAL_BAUD
		while (pidController->getError()) 
		{
            if(Serial)
            {
		        Serial.println("There is a configuration error!");
            }
            delay(1000);
		}
	#endif

	#if SAVE_DATA
		eepromManager->load();
	#endif
}

void loop()
{
	static elapsedMillis timeSinceLastSerialComm;
	static elapsedMillis timeSinceLastPrint;

	pidController->iterate();

	#ifdef SERIAL_BAUD
		if (timeSinceLastSerialComm >= SERIAL_CHECK_PERIOD_MS)
		{
			timeSinceLastSerialComm = 0;
			if (isLineAvailable())
			{
				char* line = readLine();
				commandParser->parse(line);
			}
		}

		if (timeSinceLastPrint >= PRINT_PERIOD_MS && printOutput)
		{
			timeSinceLastPrint = 0;
			printStats(*pidController);
		}
	#endif
}

inline uint16_t getSetPoint()
{
	return setpointManager->getSetPoint();
}

inline uint16_t getFeedback()
{
	uint16_t retval = readADCMultiple(readAveragesPower);
	#if RECORD_INPUT
		if ((datalog.state == LOG_SINGLE && datalog.input.available()) || datalog.state == LOG_CONTINUOUS)
		{
			dataLog.input.unshift(retval);
		}
	#endif
	return retval;
}

inline void setOutput(uint16_t out)
{
	#if RECORD_INPUT
		if ((datalog.state == LOG_SINGLE && datalog.output.available()) || datalog.state == LOG_CONTINUOUS)
		{
			dataLog.output.unshift(out);
		}
	#endif
	writeDAC(out);
}

void printStats(FPid& pidController)
{
	char line[200];
	PidState state = pidController.getPidState();
	snprintf(line, 200, "t: %i, sp: %i fb: %i op: %i ff: %i", state.iterationTime, state.setPoint, state.feedBack, state.output, pidController.getFeedForwardValue());
	Serial.println(line);
}