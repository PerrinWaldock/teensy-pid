//for the Teensy 3.6


//defines the input and output pins 
#if defined(__MK66FX1M0__) //3.6
    #define TEENSY36
#elif defined(__MK64FX512__) //3.5
    #define TEENSY35
#elif defined(__MK20DX256__) //3.2
    #define TEENSY32
#endif

#if defined(TEENSY36) || defined(TEENSY35)
#define PIN_INPUT     A0
#define PIN_OUTPUT    A21
#define PIN_REFERENCE A14

#elif defined(TEENSY32)
//teensy 3.2
#define PIN_INPUT     A9
#define PIN_OUTPUT    A14
#define PIN_REFERENCE 0
#endif

#define DIGITAL_INPUT true //if true, then it will go to setpoint if PIN_REFERENCE is high, and setpointlow if PIN_REFERENCE is low
#define ANALOG_INPUT false //if true, it overrides the setpoint value with the value from PIN_REFERENCE
#define FEEDFORWARD true


//sets the max PWM and ADC resolutions for the Teensy 3.2
#if defined(TEENSY36) || defined(TEENSY35)
    #define REFERENCE_VOLTAGE 3.3 //can change to 1.2 volts if we want
    #define DAC_BITS 12
    #define ADC_BITS 13
#elif defined(TEENSY32)
    #define REFERENCE_VOLTAGE 3.3 //can change to 1.2 volts if we want
    #define DAC_BITS 12
    #define ADC_BITS 12
#endif

#define SIGNED_OUTPUT FEEDFORWARD   //want unsigned output if no feedforward, and signed output if there is
#define DEFAULT_SAMPLE_PERIOD_US 20    //sets the PID loop frequency (too low and the code won't work properly)
#define NEGATIVE_OUTPUT true //false for positive control
#define LIMITED_SETPOINT true //pid is only active within limited setpoint range 

#define SAVE_DATA true  //saves calibration data and pid constants


const uint16_t MAX_OUTPUT = (1 << DAC_BITS) - 1; //maximum output value for the DAC (integer)
const uint16_t MAX_INPUT = (1 << ADC_BITS) - 1; //maximum output value for the DAC (integer)
const uint32_t DEFAULT_SAMPLE_RATE_HZ = 1000000/DEFAULT_SAMPLE_PERIOD_US;

#define DEFAULT_SETPOINT .5

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
