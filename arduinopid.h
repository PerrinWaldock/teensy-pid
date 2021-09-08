//for the Teensy 3.6


//defines the input and output pins 
#define PIN_INPUT     A0
#define PIN_OUTPUT    A21
#define PIN_REFERENCE A14

#define DIGITAL_INPUT false //if true, then it will go to setpoint if PIN_REFERENCE is high, and setpointlow if PIN_REFERENCE is low
#define ANALOG_INPUT false //if true, it overrides the setpoint value with the value from PIN_REFERENCE
#define FEEDFORWARD true


//sets the max PWM and ADC resolutions for the Teensy 3.2
#define REFERENCE_VOLTAGE 3.3 //can change to 1.2 volts if we want
#define PWM_BITS 15
#define ADC_BITS 12
#define SIGNED_OUTPUT false   //output only goes from 0 to +
#define SAMPLE_PERIOD_US 20    //sets the PID loop frequency (too low and the code won't work properly)
#define NEGATIVE_OUTPUT false //comment out for positive control
#define LIMITED_SETPOINT true //pid is only active within limited setpoint range 


const uint16_t MAX_OUTPUT = (1 << PWM_BITS) - 1; //maximum output value for the DAC (integer)
const uint16_t MAX_INPUT = (1 << ADC_BITS) - 1; //maximum output value for the DAC (integer)
const uint16_t SAMPLE_RATE_HZ = 1000000/SAMPLE_PERIOD_US;

#define DEFAULT_SETPOINT 1.1

//conversion macros
#define int2volts(x, b) REFERENCE_VOLTAGE*x/((1 << b) - 1)
#define volts2int(x, b) ((1 << b) - 1)*x/REFERENCE_VOLTAGE
#define flipoutput(x) MAX_OUTPUT - x
#define bound(x, a, b) x < a ? a : x > b ? b : x

//sets communication parameters
#define SERIAL_BAUD 9600 //comment this out to turn off serial control
#define INSTRING_LENGTH 80
#define READ_PERIOD_MS 100      //how quickly it prints


//if defined, then the microcontroller times how long it takes to do a loop
//set to false for speed
#define TIME_FEEDBACK_LOOP true
