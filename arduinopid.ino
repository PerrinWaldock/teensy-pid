#include <string.h>

#define OUTPUT_PIN 5	//PWM output on pin #5 because it has fastest PWM cycle
#define INPUT_PIN A0	//PWM input on analog 0 because it is an analog pin

#define MAX_OUTPUT_VOLTAGE 5 	//maximum board output voltage
#define PWM_BITS 8				//bit resolution of PWM
#define REFERENCE_VOLTAGE 1.1	//ADC peak reference voltage
#define ADC_BITS 10				//bit resolution of ADC

#define BITSHIFT_BITS 6			//shift values left by 6 bits to approximate floating point numbers
#define OUTPUT_BITSHIFT 2		//divide output by 2^OUTPUT_BITSHIFT

#define MAX_ITERM 1000			//maximum value of the integral term (to stop integral wind-up)
#define READ_AVERAGES 8			//analog inputs get read 8 times

#define SERIAL_BAUD 9600
#define INSTRING_LENGTH 80


//note: could be better to use this: https://playground.arduino.cc/Code/PIDLibraryAdaptiveTuningsExample/

uint16_t minPWM;	//minimum output
uint16_t maxPWM;	//maximum output
uint16_t setpoint;	//desired setpoint

//PID constants(divide by 2^BITSHIFT_BITS to get actual number)
uint16_t kp = 64;
uint16_t ki = 1;
uint16_t kd = 0;

//integral term
uint32_t iterm = 0;
//last measured value
uint16_t lasterror = 0;

#ifdef SERIAL_BAUD
char instring[INSTRING_LENGTH];
uint8_t inind = 0;
#endif

//utility functions
uint8_t volts2pwm(float voltage);
uint16_t volts2bsint(float voltage);
uint16_t output2pwm(uint32_t output);
uint16_t averagedread();
void updateparams(char* string);


void setup() {
	// put your setup code here, to run once:

#ifdef SERIAL_BAUD
	Serial.begin(SERIAL_BAUD);
	Serial.println("Serial Active");
#endif

	if (REFERENCE_VOLTAGE == 1.1)
	{
		//could also use EXTERNAL and use an external reference voltage
		analogReference(INTERNAL);	//sets reference voltage to 1.1
	}
	
	minPWM = volts2pwm(0.5);		//don't output less than .5 volts
	maxPWM = volts2pwm(1.3);		//don't output more than 1.3 volts
	setpoint = volts2bsint(.05);	//want to read .05 volts on the ADC

#ifdef SERIAL_BAUD
	Serial.println("PID Active");
#endif

}

void loop() {

#ifdef SERIAL_BAUD
	while (Serial.available() > 0)
	{
		instring[inind] = Serial.read();
		//Serial.print(instring[inind], HEX);
		if(inind >= INSTRING_LENGTH)
		{
			Serial.println("Incoming String Too Long");
			inind = 0;
		}
		else if (instring[inind] == '\n')
		{
			instring[inind] = "\0";
			updateparams(instring);
			inind = 0;
		}
		else
		{
			inind++;
		}
	}
#endif
	
	// put your main code here, to run repeatedly:
	uint16_t measure = averagedread();		//measure from ADC
	int16_t error = setpoint - measure;
	//PID calculations
	int32_t dterm = kd*(error - lasterror);
	int32_t pterm = kp*error;
	iterm += ki*error;
	uint32_t output = dterm + pterm + iterm;
	
	uint16_t pwm = output2pwm(output);
	analogWrite(OUTPUT_PIN, pwm);

	lasterror = error;
}

uint8_t volts2pwm(float voltage)
{
	return voltage*(pow(2,PWM_BITS) - 1)/MAX_OUTPUT_VOLTAGE;
}

uint16_t volts2bsint(float voltage)
{
	return (uint16_t)(voltage*(pow(2,ADC_BITS) - 1)/REFERENCE_VOLTAGE) << BITSHIFT_BITS;
}

uint16_t averagedread()
{
	uint32_t total = 0;
	for (uint8_t i = 0; i < READ_AVERAGES; i++)
	{
		total += analogRead(INPUT_PIN);
	}

	return (total << BITSHIFT_BITS)/READ_AVERAGES;
}

uint16_t output2pwm(uint32_t output)
{
	uint16_t pwm = output >> (BITSHIFT_BITS + OUTPUT_BITSHIFT);
	if (pwm > maxPWM)
	{
		pwm = maxPWM;
	}
	else if (pwm < minPWM)
	{
		pwm = minPWM;
	}

	return pwm;
		
}

void updateparams(char* string)
{
	char* peql = strchr(string, '=');	//find address of equals sign in string
	if (peql)	//if equal sign exists
	{
		*peql = '\0';	//replace with end string character
		
		if(!strcmp(string, "kp"))
		{
			kp = atoi(&(string[3]));
			Serial.print("kp=");
			Serial.println(kp);
		}
		else if(!strcmp(string, "ki"))
		{
			ki = atoi(&(string[3]));
			Serial.print("ki=");
			Serial.println(ki);
		}
		else if(!strcmp(string, "kd"))
		{
			kd = atoi(&(string[3]));
			Serial.print("kd=");
			Serial.println(kd);
		}
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
		}
	}
}
