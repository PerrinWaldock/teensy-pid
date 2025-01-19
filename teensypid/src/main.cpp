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
#include "printstate.h"

/*
TODO
	clean up and update the README
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

DataLog* dlog = getDataLog();
uint8_t readAveragesPower = DEFAULT_READ_AVERAGES_POWER;
PrintState printState = {
	DEFAULT_PRINT_PERIOD_MS,
	false
};

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
						DEFAULT_LOOP_RATE, 
						setpointLimits, 
						outputLimits};
	pidController = new FPid(params, getSetPoint, getFeedback, setOutput);

	eepromManager = new EepromManager(*pidController, setpointManager->getSetPoints(), readAveragesPower);

	CommandParserObjects commandParserObjects = {pidController, eepromManager, setpointManager, &readAveragesPower, &printState, dlog, &Serial};
	commandParser = new CommandParser(commandParserObjects);

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

	#if SAVE_DATA && LOAD_ON_STARTUP
		eepromManager->load();
	#else
		pidController->updateFeedForward();
	#endif

	pidController->setPidActive(true);
}

void loop()
{
	static elapsedMillis timeSinceLastSerialComm;
	static elapsedMillis timeSinceLastPrint;

	pidController->iterate();
	#if OUTPUT_SETTLE_DELAY_US > 0
		elapsedMicros outputTime;
	#endif

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

		if (timeSinceLastPrint >= printState.printPeriod_ms && printState.printOutput)
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
		if ((dlog->state == LOG_SINGLE && dlog->input.count() < INPUT_LOG_SIZE) || dlog->state == LOG_CONTINUOUS)
		{
			dlog->input.push_front(retval);
		}
	#endif
	return retval;
}

inline void setOutput(uint16_t out)
{
	static uint16_t lastOut = 0;
	#if RECORD_INPUT
		if ((dlog->state == LOG_SINGLE && dlog->output.count() < INPUT_LOG_SIZE) || dlog->state == LOG_CONTINUOUS)
		{
			dlog->output.push_front(out);
		}
	#endif

	if (out != 0 || out != lastOut)
	{
		writeDAC(out);

		int32_t change = out - lastOut;
		if (change < 0)
		{
			change *= -1;
		}

		#if OUTPUT_SETTLE
			delayMicroseconds((change >> SLEW_RATE_POWER_PER_US));
		#endif
	}
}

void printStats(FPid& pidController)
{
	PidState state = pidController.getPidState();
	if (!state.active || state.railed)
	{
		state.feedBack = getFeedback();
	}
	Serial.printf("t: %i, sp: %i fb: %i pd: %i op: %i ff: %i\n\r", state.iterationTime, state.setPoint, state.feedBack, state.pid, state.output, pidController.getFeedForwardValue());
}