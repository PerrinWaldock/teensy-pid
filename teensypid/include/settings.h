#ifndef SETTINGS_H
#define SETTINGS_H

#include <Arduino.h>

#define TEENSY40 40
#define TEENSY36 36
#define TEENSY35 35
#define TEENSY32 32

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

//default PID constants
const float DEFAULT_KP = .005;
const float DEFAULT_KI = 1000;
const float DEFAULT_KD = 0;

//loop timing
const uint32_t DEFAULT_LOOP_RATE = 10;
#define PRECISE_LOOP_TIMING false
#define OUTPUT_SETTLE true

#define SERIAL_BAUD 1000000 //comment this out to disable serial communication

//periods
const uint16_t SERIAL_CHECK_PERIOD_MS = 10; //check and handle serial input
const uint16_t DEFAULT_PRINT_PERIOD_MS = 500; //print pid state

///////////// Settings

const Input_Mode INPUT_MODE = DIGITAL_INPUT; // select type of setpoint input

#define FEED_FORWARD true

#if !FEED_FORWARD
	#define CLEAR_INTEGRAL_WHEN_RAILED true
#endif

#define NEGATIVE_OUTPUT_SLOPE false // set to true if a larger output value causes a smaller feedback value

#define SAVE_DATA true  //saves calibration data and pid constants
#define LOAD_ON_STARTUP true

#define DEFAULT_SETPOINT .5
#define DEFAULT_READ_AVERAGES_POWER 1
#if INPUT_MODE == ANALOG_INPUT
	#define ANALOG_READ_AVERAGES 1 //sets how many reads are used when measuring the setpoint
	#define ANALOG_REFERENCE_RESOLUTION 8
#endif

#if FEED_FORWARD
    const uint16_t CALIBRATION_AVERAGES = 256;
	const uint16_t CALIBRATION_SETTLE_DELAY_US = 100;
	const uint32_t CALIBRATION_PRE_SETTLE_DELAY_US = 10000;
#endif

// records feedback even when setpoint out of range
#define RECORD_FEEDBACK_ALL_ITERATIONS true
// if true, microcontroller times how long it takes to perform a feedback loop
#define TIME_FEEDBACK_LOOP true 
//logs input values
#define RECORD_INPUT true
#if RECORD_INPUT
	#define RECORD_FEEDBACK true 
	#define RECORD_OUTPUT true
	#define RECORD_SETPOINT true
	#define RECORD_TIME true
#endif


//////////////////////// Don't change these values unless the hardware changes

#define ADC_BITS 16
#define DAC_BITS 16
const float ADC_REFERENCE_VOLTAGE = 5.0;
const float DAC_REFERENCE_VOLTAGE = 5.0;

const int32_t MAX_OUTPUT = (1 << DAC_BITS) - 1; //maximum output value for the DAC (integer)
const int32_t MAX_INPUT = (1 << ADC_BITS) - 1; //maximum output value for the DAC (integer)
const int32_t HALF_MAX_INPUT = 1 << (ADC_BITS - 1);
const int32_t HALF_MAX_OUTPUT = 1 << (DAC_BITS - 1);

const uint32_t DEFAULT_SAMPLE_RATE_HZ = 1000000/DEFAULT_LOOP_RATE;

const float DEFAULT_MIN_SETPOINT_VOLTS = 0.05;
const float DEFAULT_MAX_SETPOINT_VOLTS = ADC_REFERENCE_VOLTAGE - DEFAULT_MIN_SETPOINT_VOLTS;

const float DEFAULT_MIN_OUTPUT_VOLTS = 0;
const float DEFAULT_MAX_OUTPUT_VOLTS = DAC_REFERENCE_VOLTAGE;

#if INPUT_MODE == DIGITAL_INPUT
    const int NUM_SETPOINTS = 4; // number of selectable input states
#else
	const int NUM_SETPOINTS = 1;
#endif

#if FEED_FORWARD
    const uint32_t FEED_FORWARD_ARRAY_BITS = DAC_BITS; //want to have a value for each and every setpoint
    const uint32_t FF_CALIBRATION_ARRAY_BITS = 8;
#endif

const uint16_t SLEW_RATE_POWER_PER_US = 14;


#define FEEDBACK_TOKEN "f"
#define SETPOINT_TOKEN "s"
#define OUTPUT_TOKEN "o"
#define TIME_TOKEN "t"

#endif