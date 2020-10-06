/*
 * This is PID feedback control code intended to be used with the Teensy 3.2: 
 * https://www.pjrc.com/teensy/teensy31.html
 * Its pins are here:
 * https://www.pjrc.com/teensy/pinout.html
 * And its schematic is here:
 * https://www.pjrc.com/teensy/schematic.html
 * 
 * Most of the PID feedback control code is from here: https://github.com/mike-matera/FastPID/blob/master/src/FastPID.cpp
 * 
 * look here for code to speed up ADC: https://github.com/pedvide/ADC/blob/master/examples/analogContinuousRead/analogContinuousRead.ino
 */


#include <FastPID.h>	//prepackaged arduino feedback control library

//defines the input and output pins 
#define PIN_INPUT     A0
#define PIN_OUTPUT    A21//A14

//sets the max PWM and ADC resolutions for the Teensy 3.2
#define REFERENCE_VOLTAGE 3.3	//can change to 1.2 volts if we want
#define PWM_BITS 15
#define ADC_BITS 12
#define SIGNED_OUTPUT false		//output only goes from 0 to +
#define SAMPLE_PERIOD_US 200//50		//sets the PID loop frequency (too low and the code doesn't work)
#define NEGATIVE_OUTPUT //comment out for positive control
const uint16_t MAX_OUTPUT = (1 << PWM_BITS) - 1;

const uint16_t SAMPLE_RATE_HZ = 1000000/SAMPLE_PERIOD_US;

#define READ_AVERAGES 64			//analog inputs get averaged over this many times

//conversion macros
#define int2volts(x, b) REFERENCE_VOLTAGE*x/(1 << (b - 1))
#define volts2int(x, b) (1 << (b - 1))*x/REFERENCE_VOLTAGE

//sets communication parameters
#define SERIAL_BAUD 9600
#define INSTRING_LENGTH 80

//if defined, then the microcontroller times how long it takes to do a loop
//undefine for speed
#define TIME_FEEDBACK_LOOP

//pid feedback control parameters
float kp=5, ki=100, kd=0;
uint16_t setpoint = volts2int(0.7, ADC_BITS); //default setpoint

//things that only need to be defined if feedback control is active
#ifdef SERIAL_BAUD
	#define READ_PERIOD_MS 100			//how quickly it prints
	char instring[INSTRING_LENGTH];		//input string
	uint8_t inind = 0;
	bool serialActive = false;			//serial active flag
	bool printout = false;				//if true, print feedback stats
	volatile bool readSerial = true;	//read serial flag
#endif

volatile bool pidcalc = true; //flag saying to perform feedback control calc
bool pidactive = true;			//flag saying "pid control is active"

IntervalTimer pidTimer;	//used for timing the pid feedback loop
IntervalTimer readTimer; //used for timing the serial read code

FastPID myPID(kp, ki, kd, SAMPLE_RATE_HZ, PWM_BITS, SIGNED_OUTPUT); //feedback control object


void updateparams(char* string);	//updates parameters given string
uint32_t averagedread();			//reads and averages data
void setpidcalc();					//sets pid calc flag (ISR)
void setreadserial();				//sets read serial flag (ISR)

void setup()
{
	//sets pin states
	pinMode(PIN_INPUT, INPUT);
	pinMode(PIN_OUTPUT, OUTPUT);
	analogReadResolution(ADC_BITS);
	analogWriteResolution(PWM_BITS); 

	//sets reference voltage
	if (REFERENCE_VOLTAGE == 1.2)
	{
		//could also use EXTERNAL and use an external reference voltage
		analogReference(INTERNAL);	//sets reference voltage to 1.2
	}
	else
	{
		analogReference(DEFAULT); //3.3v
	}

//sets up serial
#ifdef SERIAL_BAUD
	Serial.begin(SERIAL_BAUD);
	serialActive = true;
	if (myPID.err()) {
		Serial.println("There is a configuration error!");
		for (;;) {}
	}
	else
	{
		Serial.println("PID Begin");
	}
	readTimer.begin(setreadserial, READ_PERIOD_MS*1000);
#endif

	pidTimer.begin(setpidcalc, SAMPLE_PERIOD_US);
}

//feedback control variables 
uint32_t feedback = 0;
uint16_t out = 0;
#ifdef TIME_FEEDBACK_LOOP
uint32_t ts = 0;
uint32_t tss = 0;
#endif

//main loop
void loop()
{	
	//run feedback control loop
	if(pidcalc)
	{
#ifdef TIME_FEEDBACK_LOOP
	ts = micros();
#endif
	if(pidactive)
	{
		feedback = analogRead(PIN_INPUT);
#ifdef NEGATIVE_OUTPUT
		out = MAX_OUTPUT - myPID.step(setpoint, feedback);
#else
		out = myPID.step(setpoint, feedback);
#endif
	}
	analogWrite(PIN_OUTPUT, out);
	pidcalc = false;
#ifdef TIME_FEEDBACK_LOOP
	tss = micros();
#endif
	}
#ifdef SERIAL_BAUD
	//if Serial connection exists and it's time to check for serial
	if(Serial && readSerial)
	{
		//sets up serial connection if necessary
		if(!serialActive)
		{
			Serial.begin(SERIAL_BAUD);
			serialActive = true;
		}
		//reads data from user and possibly sets parameters using it
		while (Serial.available() > 0)
		{
			instring[inind] = Serial.read();
			if(inind >= INSTRING_LENGTH)
			{
				Serial.println("Incoming String Too Long");
				inind = 0;
			}
			else if (instring[inind] == '\n')
			{
				instring[inind] = '\0';
				updateparams(instring);
				inind = 0;
			}
			else
			{
				inind++;
			}
		}
		//prints parameters
		if(printout)
		{
			Serial.print("s:");
			Serial.print(setpoint);
			Serial.print("\t f:");
			Serial.print(feedback);
			Serial.print("\t o:");
#ifdef TIME_FEEDBACK_LOOP
			Serial.print(out);
			Serial.print("\t t:");
			Serial.println(tss - ts);
#else
			Serial.println(out);
#endif
		}
		readSerial = false;
	}
	else if(serialActive) //if no serial object but it thinks the connection is active make sure to end the connection
	{
		Serial.end();
		serialActive = false;
	}
#endif
}

//updates pid feedback parameters given a string from the user
void updateparams(char* string)
{
	char* peql = strchr(string, '=');	//find address of equals sign in string
	if (peql)	//if equal sign exists
	{
		*peql = '\0';	//replace with end string character
		
		if(!strcmp(string, "kp"))
		{
			kp = atof(&(string[3]));
			Serial.print("kp=");
			Serial.println(kp);
		}
		else if(!strcmp(string, "ki"))
		{
			ki = atof(&(string[3]));
			Serial.print("ki=");
			Serial.println(ki);
		}
		else if(!strcmp(string, "kd"))
		{
			kd = atof(&(string[3]));
			Serial.print("kd=");
			Serial.println(kd);
		}
		else if(!strcmp(string, "sp"))
		{
			setpoint = atof(&(string[3]));
			Serial.print("setpoint=");
			Serial.println(setpoint);
		}
		else if(!strcmp(string, "sv"))
		{
			setpoint = volts2int(atof(&(string[3])), ADC_BITS);
			Serial.print("setpoint=");
			Serial.print(int2volts(setpoint, ADC_BITS));
			Serial.print("V, ");
			Serial.println(setpoint);
		}
		else if(!strcmp(string, "pa"))
		{
			pidactive = atoi(&(string[3]));
			Serial.print("pidactive=");
			Serial.println(pidactive);
		}
		else if(!strcmp(string, "ov"))
		{
			pidactive = false;
			out = volts2int(atof(&(string[3])), PWM_BITS);
			Serial.print("pidactive=");
			Serial.println(pidactive);
			Serial.print("output=");
			Serial.print(int2volts(out, PWM_BITS));
			Serial.print("V, ");
			Serial.println(out);
		}
#ifdef SERIAL_BAUD
		else if(!strcmp(string, "po"))
		{
			printout = atoi(&(string[3]));
			Serial.print("printout=");
			Serial.println(printout);
		}
#endif
		else
		{
			Serial.println("Invalid Command");
		}

		myPID.setCoefficients(kp, ki, kd, SAMPLE_RATE_HZ);
		
	}
	else
	{
		peql = strchr(string, '?');	//find address of '?' string
		if (peql)	//if equal sign exists
		{
			*peql = '\0';	//replace with end string character
			if(!strcmp(string, "kp"))
			{
				Serial.print("kp=");
				Serial.println(kp);
			}
			else if(!strcmp(string, "ki"))
			{
				Serial.print("ki=");
				Serial.println(ki);
			}
			else if(!strcmp(string, "kd"))
			{
				Serial.print("kd=");
				Serial.println(kd);
			}
			else if(!strcmp(string, "sp"))
			{
				Serial.print("setpoint=");
				Serial.print(int2volts(setpoint, ADC_BITS));
				Serial.print("V, ");
				Serial.println(setpoint);
			}
			else if(!strcmp(string, "ov"))
			{
				Serial.print("output=");
				Serial.print(int2volts(out, PWM_BITS));
				Serial.print("V, ");
				Serial.println(out);
			}
			else if(!strcmp(string, "pa"))
			{
				Serial.print("pidactive=");
				Serial.println(pidactive);	
			}
			else
			{
				Serial.println("Invalid Command");
			}
		}
	}
}

//conducts an averaged read
uint32_t averagedread()
{
	uint32_t total = 0;
	for (uint8_t i = 0; i < READ_AVERAGES; i++)
	{
		total += analogRead(PIN_INPUT);
	}

	return total/READ_AVERAGES;
}

//ISRs that set flags
void setpidcalc()
{
	pidcalc = true;
}

#ifdef SERIAL_BAUD
void setreadserial()
{
	readSerial = true;
}
#endif
