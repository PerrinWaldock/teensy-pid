//for the Teensy 3.6

#include "adc.h"
#include "dac.h"

//defines the input and output pins 
#if defined(__MK66FX1M0__) //3.6
    #define TEENSY36
#elif defined(__MK64FX512__) //3.5
    #define TEENSY35
#elif defined(__MK20DX256__) //3.2
    #define TEENSY32
#elif defined(__IMXRT1062__) //4.x
    #define TEENSY41
#endif

#if defined(TEENSY41)
    #define PIN_REFERENCE0 2
    #define PIN_REFERENCE1 3
    #define PIN_REFERENCE A10 //TODO currently not connected
#endif

#define DIGITAL_INPUT true //if true, then it will go to setpoint if PIN_REFERENCE is high, and setpointlow if PIN_REFERENCE is low. ANALOG_INPUT being true supercedes this
#if DIGITAL_INPUT
    #define INPUT_STATES 4 //TODO do this more cleverly with an array
#endif

#define ANALOG_INPUT false //if true, it overrides the setpoint value with the value from PIN_REFERENCE TODO update PCB to support this
#define FEEDFORWARD false

#define SIGNED_OUTPUT FEEDFORWARD   //want unsigned output if no feedforward, and signed output if there is
#define DEFAULT_SAMPLE_PERIOD_US 1000   //20  //sets the PID loop frequency (too low and the code won't work properly)
#define NEGATIVE_OUTPUT false //false for positive control
#define LIMITED_SETPOINT true //pid is only active within limited setpoint range 

#define SAVE_DATA false  //saves calibration data and pid constants


const uint16_t MAX_OUTPUT = (1 << DAC_BITS) - 1; //maximum output value for the DAC (integer)
const uint16_t MAX_INPUT = (1 << ADC_BITS) - 1; //maximum output value for the DAC (integer)
const uint32_t DEFAULT_SAMPLE_RATE_HZ = 1000000/DEFAULT_SAMPLE_PERIOD_US;

#define DEFAULT_SETPOINT .5
#define READDAVERAGESPOWER 3 //TODO fiddle with this

#define KP_MIN 0.00390625
#define KP_MAX 255.0

//conversion macros
#define int2volts(x, b, ref) ref*x/((1 << b) - 1)
#define volts2int(x, b, ref) ((1 << b) - 1)*x/ref
#define flipoutput(x) MAX_OUTPUT - x
#define bound(x, a, b) x < a ? a : x > b ? b : x

//sets communication parameters
#define SERIAL_BAUD 9600 //comment this out to turn off serial control
#define INSTRING_LENGTH 80
#define READ_PERIOD_MS 100      //how quickly it prints


//if defined, then the microcontroller times how long it takes to do a loop
//set to false for speed
#define TIME_FEEDBACK_LOOP true