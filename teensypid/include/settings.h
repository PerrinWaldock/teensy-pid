#ifndef SETTINGS_H
#define SETTINGS_H

#include <Arduino.h>

typedef enum{
	SOFTWARE_INPUT,
	DIGITAL_INPUT,
	ANALOG_INPUT
} Input_Mode;

//////////////////////////////// Values the user can change

///////////// Defaults

//sets the PID loop frequency (too low and the code won't work properly)
#define DEFAULT_SAMPLE_PERIOD_US 1000   //20  
#define DEFAULT_KP
#define DEFAULT_KI
#define DEFAULT_KD

///////////// Settings

const Input_Mode INPUT_MODE = DIGITAL_INPUT; // select type of setpoint input

#if INPUT_MODE == DIGITAL_INPUT
    #define INPUT_STATES 4 // number of selectable input states
#endif

#define FEED_FORWARD true

#define NEGATIVE_OUTPUT_SLOPE false // set to true if a larger output value causes a smaller feedback value

#define SAVE_DATA true  //saves calibration data and pid constants

#define DEFAULT_SETPOINT .5
#define READ_AVERAGES_POWER 3 //TODO separate feedback and setpoint read averages
#if INPUT_MODE == ANALOG_INPUT
	#define ANALOG_READ_AVERAGES 1 //
	#define ANALOG_REFERENCE_RESOLUTION 12
#endif
#define OUTPUT_SETTLE_DELAY_US 8

//sets communication parameters
#define SERIAL_BAUD 1000000 //comment this out to disable serial communication
#define INSTRING_LENGTH 80
#define READ_PERIOD_MS 100      //how quickly it prints

// if true, microcontroller times how long it takes to perform a feedback loop
#define TIME_FEEDBACK_LOOP true

//logs input values
#define RECORD_INPUT false


//////////////////////// Don't change these values unless the hardware changes

#define ADC_BITS 16
#define DAC_BITS 16

const uint16_t MAX_OUTPUT = (1 << DAC_BITS) - 1; //maximum output value for the DAC (integer)
const uint16_t MAX_INPUT = (1 << ADC_BITS) - 1; //maximum output value for the DAC (integer)
const int32_t HALF_MAX_INPUT = 1 << (ADC_BITS - 1);
const int32_t HALF_MAX_OUTPUT = 1 << (DAC_BITS - 1);

const uint32_t DEFAULT_SAMPLE_RATE_HZ = 1000000/DEFAULT_SAMPLE_PERIOD_US;

#if FEED_FORWARD
    const uint32_t FEED_FORWARD_ARRAY_BITS = DAC_BITS; //want to have a value for each and every setpoint
    const uint32_t FF_CALIBRATION_ARRAY_BITS = DAC_BITS - 4;
    #define CALIBRATION_AVERAGES 256
	#define CALIBRATION_SETTLE_DELAY_US 10
	#define CALIBRATION_PRE_SETTLE_DELAY_US 10000
#endif

#endif