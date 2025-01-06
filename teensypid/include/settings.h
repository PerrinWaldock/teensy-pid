#ifndef SETTINGS_H
#define SETTINGS_H

#include <Arduino.h>

#if defined(__MK66FX1M0__) //3.6
    #define TEENSY TEENSY36
#elif defined(__MK64FX512__) //3.5
    #define TEENSY TEENSY35
#elif defined(__MK20DX256__) //3.2
    #define TEENSY TEENSY32
#elif defined(__IMXRT1062__) //4.x
    #define TEENSY TEENSY40
#endif

typedef enum {
	SOFTWARE_INPUT,
	DIGITAL_INPUT,
	ANALOG_INPUT
} Input_Mode;

//////////////////////////////// Values the user can change

///////////// Defaults

//sets the PID loop frequency (too low and the code won't work properly)
const uint32_t DEFAULT_SAMPLE_PERIOD_US = 20;
const float DEFAULT_KP = .005;
const float DEFAULT_KI = 1000;
const float DEFAULT_KD = 0; // TODO change

//sets communication parameters
const uint32_t SERIAL_BAUD = 1000000; //comment this out to disable serial communication
const uint16_t SERIAL_CHECK_PERIOD_MS = 10;
const uint16_t DEFAULT_PRINT_PERIOD_MS = 500;      //how quickly it prints

#define PRECISE_LOOP_TIMING false

///////////// Settings

#define SERIAL_BAUD 1000000

const Input_Mode INPUT_MODE = DIGITAL_INPUT; // select type of setpoint input

#if INPUT_MODE == DIGITAL_INPUT
    const int NUM_SETPOINTS = 4; // number of selectable input states
#endif

#define FEED_FORWARD true

#if !FEED_FORWARD
	#define CLEAR_INTEGRAL_WHEN_RAILED true
#endif

#define NEGATIVE_OUTPUT_SLOPE false // set to true if a larger output value causes a smaller feedback value

#define RECORD_FEEDBACK_ALL_ITERATIONS false
#define SAVE_DATA true  //saves calibration data and pid constants
#define LOAD_ON_STARTUP true

#define DEFAULT_SETPOINT .5
#define DEFAULT_READ_AVERAGES_POWER 1 //TODO separate feedback and setpoint read averages
#if INPUT_MODE == ANALOG_INPUT
	#define ANALOG_READ_AVERAGES 1 //
	#define ANALOG_REFERENCE_RESOLUTION 12
#endif
#define OUTPUT_SETTLE_DELAY_US 8

// if true, microcontroller times how long it takes to perform a feedback loop
#define TIME_FEEDBACK_LOOP true // TODO implement

//logs input values
#define RECORD_INPUT false


//////////////////////// Don't change these values unless the hardware changes

#define ADC_BITS 16
#define DAC_BITS 16
const float ADC_REFERENCE_VOLTAGE = 5.0;
const float DAC_REFERENCE_VOLTAGE = 5.0;

const int32_t MAX_OUTPUT = (1 << DAC_BITS) - 1; //maximum output value for the DAC (integer)
const int32_t MAX_INPUT = (1 << ADC_BITS) - 1; //maximum output value for the DAC (integer)
const int32_t HALF_MAX_INPUT = 1 << (ADC_BITS - 1);
const int32_t HALF_MAX_OUTPUT = 1 << (DAC_BITS - 1);

const uint32_t DEFAULT_SAMPLE_RATE_HZ = 1000000/DEFAULT_SAMPLE_PERIOD_US;

const float DEFAULT_MIN_SETPOINT_VOLTS = 0.05;
const float DEFAULT_MAX_SETPOINT_VOLTS = ADC_REFERENCE_VOLTAGE - DEFAULT_MIN_SETPOINT_VOLTS;

const float DEFAULT_MIN_OUTPUT_VOLTS = 0;
const float DEFAULT_MAX_OUTPUT_VOLTS = DAC_REFERENCE_VOLTAGE;

#if FEED_FORWARD
    const uint32_t FEED_FORWARD_ARRAY_BITS = DAC_BITS; //want to have a value for each and every setpoint
    const uint32_t FF_CALIBRATION_ARRAY_BITS = DAC_BITS - 8;
    const uint16_t CALIBRATION_AVERAGES = 256;
	const uint16_t CALIBRATION_SETTLE_DELAY_US = 10;
	const uint32_t CALIBRATION_PRE_SETTLE_DELAY_US = 10000;
#endif

#endif