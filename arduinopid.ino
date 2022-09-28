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
 * 
 * If using digital control code, use pyserial to adjust setpoint: https://anaconda.org/anaconda/pyserial
 */

/*
 * TODO write python script that generates the parameter setting code
 * pass it a pointer to a struct of pointers to all relevant values for max speed
 * 
 * TODO allow for smaller feedforward array, have same output for all inputs of same MSBs
 * 
 * TODO allow for separate digital and analog inputs
 */

#include "arduinopid.h"
#include <FastPID.h>  //prepackaged arduino feedback control library

//pid feedback control parameters

#if FEEDFORWARD
float kp=.1, ki=2000, kd=0; //good for feedforward
#else
float kp=0.1, ki=45000, kd=0; //good for non-feedforward
#endif
/*float kp=1;
float ki=kp*SAMPLE_RATE_HZ;
float kd=0;*/
//this is also the setpoint if digital input is low
uint16_t setpoint = volts2int(DEFAULT_SETPOINT, ADC_BITS); //default setpoint (voltage it reads)
#if DIGITAL_INPUT
uint16_t setpointlow = 0;	//default low digital setpoint
#endif
uint8_t readaverages = 4;	//number of times to average the input (hardware), default 4

#if LIMITED_SETPOINT
uint16_t minsetpoint = volts2int(0.1,ADC_BITS);
uint16_t maxsetpoint = volts2int(3,ADC_BITS);
uint16_t minoutput = 0;
uint16_t maxoutput = MAX_OUTPUT;
#endif


//things that only need to be defined if feedback control is active
#ifdef SERIAL_BAUD
	char instring[INSTRING_LENGTH];   //input string
	uint8_t inind = 0;
	bool serialActive = false;      //serial active flag
	bool printout = false;        //if true, print feedback stats
	volatile bool readSerial = true;  //read serial flag
#endif

volatile bool pidcalc = true; //flag saying to perform feedback control calc
bool pidactive = true;      //flag saying "pid control is active"

#if FEEDFORWARD
    #define FEEDFORWARD_ARRAY_LENGTH 4096//65536 //MAXINPUT+1
    #define FF_CALIB_ARRAY_LENGTH 256
    #define ffCalibOutput(x) (int)((x*((1 << DAC_BITS) - 1))/(FF_CALIB_ARRAY_LENGTH - 1))
    uint16_t feedforward[FEEDFORWARD_ARRAY_LENGTH];
    uint16_t ffCalib[FF_CALIB_ARRAY_LENGTH];
#endif

IntervalTimer pidTimer; //used for timing the pid feedback loop
IntervalTimer readTimer; //used for timing the serial read code

FastPID myPID(kp, ki, kd, SAMPLE_RATE_HZ, DAC_BITS, SIGNED_OUTPUT);; //feedback control object

//private functions
#ifdef SERIAL_BAUD
void updateparams(char* string);  //updates parameters given string
#endif
void setpidcalc();          //sets pid calc flag (ISR)
void setreadserial();       //sets read serial flag (ISR)

#if FEEDFORWARD
void getFeedforwardReadings(uint16_t* readings);
void calibrateFeedforward(uint16_t* readings);
#endif

void setup()
{
	//sets pin states
	pinMode(PIN_INPUT, INPUT);
	pinMode(PIN_OUTPUT, OUTPUT);
#if DIGITAL_INPUT
	pinMode(PIN_REFERENCE, INPUT_PULLUP);
#endif
#if ANALOG_INPUT
	pinMode(PIN_REFERENCE, INPUT);
#endif

	//sets averaging, resolution, etc
	analogReadAveraging(readaverages);
	analogReadResolution(ADC_BITS);
	analogWriteResolution(DAC_BITS); 
	
	//sets reference voltage
	if (REFERENCE_VOLTAGE == 1.2)
	{
		//could also use EXTERNAL and use an external reference voltage
		analogReference(INTERNAL);  //sets reference voltage to 1.2
	}
	else
	{
		analogReference(DEFAULT); //3.3v
	}
	
	//sets up serial
	#ifdef SERIAL_BAUD

		Serial.begin(SERIAL_BAUD);
		serialActive = true;
		while(myPID.err()) 
		{
            if(Serial)
            {
		        Serial.println("There is a configuration error!");
            }
            delay(1000);
		}

		Serial.println("PID Begin");
		readTimer.begin(setreadserial, READ_PERIOD_MS*1000);
	#endif

	//begins pid feedback loop timer
	pidTimer.begin(setpidcalc, SAMPLE_PERIOD_US);

}

//feedback control variables 
uint32_t feedback = 0;
uint16_t out = 0;
uint16_t sp;
#if TIME_FEEDBACK_LOOP
	uint32_t ts = 0;
	uint32_t tss = 0;
#endif

//main loop
void loop()
{ 
	//run feedback control loop
	if(pidcalc)
	{
		#if TIME_FEEDBACK_LOOP
			ts = micros();
		#endif
        feedback = analogRead(PIN_INPUT); //gets feedback
		if(pidactive) //only run if pid is active
		{
			#if ANALOG_INPUT
				sp = analogRead(PIN_REFERENCE);	
                setpoint = sp;		
			#endif
			#if DIGITAL_INPUT
				if (digitalReadFast(PIN_REFERENCE) == LOW)
				{
					sp = setpointlow;			
				}
				else
				{
					sp = setpoint;			
				}
			#endif
			#if !DIGITAL_INPUT && !ANALOG_INPUT
				sp = setpoint;
			#endif
			#if LIMITED_SETPOINT //TODO look into using "clear" here
				if(sp < minsetpoint)
				{
					out = minoutput;
				}
				else if(sp > maxsetpoint)
				{
					out = maxoutput;
				}
				else
				{
					out = myPID.step(sp, feedback); //only runs the pid calculation if you are inside the specified setpoint range	
					out = bound(out, min(minoutput, maxoutput), max(minoutput, maxoutput));
				}
			#else
				out = myPID.step(sp, feedback); //no endpoints to worry about
			#endif
			

			#if NEGATIVE_OUTPUT
				out = flipoutput(out);
			#endif

            #if FEEDFORWARD
                out += feedforward[sp];
            #endif
		}
		analogWrite(PIN_OUTPUT, out); //still write output
		pidcalc = false;
		#if TIME_FEEDBACK_LOOP
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
				Serial.print(sp); //todo should this be setpoint?
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

#ifdef SERIAL_BAUD
//updates pid feedback parameters given a string from the user
//note: this could be much more elegantly implemented, but it's done this way for speed
void updateparams(char* string)
{
	char* peql = strchr(string, '='); //find address of equals sign in string
	if (peql) //if equal sign exists
	{
        bool updatePID = false;
		*peql = '\0'; //replace with end string character
	
		if(!strcmp(string, "kp"))
		{
			kp = atof(&(string[3]));
			Serial.print("kp=");
			Serial.println(kp,4);
            updatePID = true;
		}
		else if(!strcmp(string, "ki"))
		{
			ki = atof(&(string[3]));
			Serial.print("ki=");
			Serial.println(ki,6);
            updatePID = true;
			//_sum = 0; //reset integral
		}
		else if(!strcmp(string, "kd"))
		{
			kd = atof(&(string[3]));
			Serial.print("kd=");
			Serial.println(kd,4);
            updatePID = true;
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
			Serial.print(int2volts(setpoint, ADC_BITS),4);
			Serial.print("V, ");
			Serial.println(setpoint);
		}
#if DIGITAL_INPUT
		else if(!strcmp(string, "lp"))
		{
			setpointlow = atof(&(string[3]));
			Serial.print("low setpoint=");
			Serial.println(setpointlow);
		}
		else if(!strcmp(string, "lv"))
		{
			setpointlow = volts2int(atof(&(string[3])), ADC_BITS);
			Serial.print("low setpoint=");
			Serial.print(int2volts(setpointlow, ADC_BITS),4);
			Serial.print("V, ");
			Serial.println(setpointlow);
		}
#endif
#if LIMITED_SETPOINT
		else if(!strcmp(string, "lo"))
		{
			#if NEGATIVE_OUTPUT
				minoutput = flipoutput(volts2int(atof(&(string[3])), DAC_BITS));
			#else
				minoutput = volts2int(atof(&(string[3])), DAC_BITS);
			#endif
			Serial.print("Output Limits: ");
			Serial.print(int2volts(minoutput, ADC_BITS), 4);
			Serial.print("V, ");
			Serial.print(int2volts(maxoutput, ADC_BITS), 4);
			Serial.print("V");
		}
		else if(!strcmp(string, "ho"))
		{
			#if NEGATIVE_OUTPUT
				maxoutput = flipoutput(volts2int(atof(&(string[3])), DAC_BITS));
			#else
				maxoutput = volts2int(atof(&(string[3])), DAC_BITS);
			#endif			
			Serial.print("Output Limits: ");
			Serial.print(int2volts(minoutput, ADC_BITS), 4);
			Serial.print("V, ");
			Serial.print(int2volts(maxoutput, ADC_BITS), 4);
			Serial.print("V");
		}
		else if(!strcmp(string, "ls"))
		{
			minsetpoint = volts2int(atof(&(string[3])), DAC_BITS);
			Serial.print("Setpoint Limits: ");
			Serial.print(int2volts(minsetpoint, ADC_BITS), 4);
			Serial.print("V, ");
			Serial.print(int2volts(maxsetpoint, ADC_BITS), 4);
			Serial.print("V");
		}
		else if(!strcmp(string, "hs"))
		{
			minsetpoint = volts2int(atof(&(string[3])), DAC_BITS);
			Serial.print("Setpoint Limits: ");
			Serial.print(int2volts(minsetpoint, ADC_BITS), 4);
			Serial.print("V, ");
			Serial.print(int2volts(maxsetpoint, ADC_BITS), 4);
			Serial.print("V");
		}
#endif
		else if(!strcmp(string, "pa"))
		{
			pidactive = atoi(&(string[3]));
			Serial.print("pidactive=");
			Serial.println(pidactive);
		}
		else if(!strcmp(string, "ra"))
		{
			readaverages = atoi(&(string[3]));
			Serial.print("read averages=");
			Serial.println(readaverages);
			analogReadAveraging(readaverages);
		}
		else if(!strcmp(string, "ov"))
		{
			pidactive = false;
			out = volts2int(atof(&(string[3])), DAC_BITS);
            if (out > MAX_OUTPUT)
            {
                Serial.print("Output out of range!");
                out = MAX_OUTPUT;
            }
            if (out < 0)
            {
                Serial.print("Output out of range!");
                out = 0;
            }
			Serial.print("pidactive=");
			Serial.println(pidactive);
			Serial.print("output=");
			Serial.print(int2volts(out, DAC_BITS), 4);
			Serial.print("V, ");
			Serial.println(out);
		}
		else if(!strcmp(string, "po"))
		{
			printout = atoi(&(string[3]));
			Serial.print("printout=");
			Serial.println(printout);
		}
        else if (!strcmp(string, "cl"))
        {
            Serial.print("cl=");
            if(atoi(&(string[3])))
            {
                myPID.clear();     
            }
            Serial.println(&(string[3]));
        }
		else
		{
			Serial.println("Invalid Command");
		}

        if (updatePID)
        {
            bool success = myPID.configure(kp, ki, kd, SAMPLE_RATE_HZ, DAC_BITS, SIGNED_OUTPUT);
    
            if(!success)
            {
                Serial.println("Error setting coefficients");
            }        
        }
		
	}
	else
	{
		peql = strchr(string, '?'); //find address of '?' string
		if (peql) //if equal sign exists
		{
			*peql = '\0'; //replace with end string character
			if(!strcmp(string, "kp"))
			{
				Serial.print("kp=");
				Serial.println(kp, 4);
			}
			else if(!strcmp(string, "ki"))
			{
				Serial.print("ki=");
				Serial.println(ki,6);
			}
			else if(!strcmp(string, "kd"))
			{
				Serial.print("kd=");
				Serial.println(kd,6);
			}
			else if(!strcmp(string, "sp") || !strcmp(string, "sv"))
			{
				Serial.print("setpoint=");
				Serial.print(int2volts(setpoint, ADC_BITS));
				Serial.print("V, ");
				Serial.println(setpoint);
			}
			#if DIGITAL_INPUT
				else if(!strcmp(string, "lp") || !strcmp(string, "lv"))
				{
					Serial.print("low setpoint=");
					Serial.print(int2volts(setpointlow, ADC_BITS));
					Serial.print("V, ");
					Serial.println(setpointlow);
				}
			#endif
			#if LIMITED_SETPOINT
				else if(!strcmp(string, "lo") || !strcmp(string, "ho"))
				{
					Serial.print("Output Limits: ");
					Serial.print(int2volts(minoutput, ADC_BITS));
					Serial.print("V, ");
					Serial.print(int2volts(maxoutput, ADC_BITS));
					Serial.print("V");
				}
				else if(!strcmp(string, "ls") || !strcmp(string, "hs"))
				{
					Serial.print("Setpoint Limits: ");
					Serial.print(int2volts(minsetpoint, ADC_BITS));
					Serial.print("V, ");
					Serial.print(int2volts(maxsetpoint, ADC_BITS));
					Serial.print("V");
				}
			#endif
			else if(!strcmp(string, "ov"))
			{
				Serial.print("output=");
				Serial.print(int2volts(out, DAC_BITS));
				Serial.print("V, ");
				Serial.println(out);
			}
			else if(!strcmp(string, "ra"))
			{
				Serial.print("read averages=");
				Serial.println(readaverages);
			}
			else if(!strcmp(string, "pa"))
			{
				Serial.print("pidactive=");
				Serial.println(pidactive);  
			}
#if FEEDFORWARD
           else if(!strcmp(string, "cf")) //calibrate feedforward
            {
                Serial.println("calibrating feedforward");
                getFeedforwardReadings(ffCalib);
                calibrateFeedforward(ffCalib);

                /*for (uint32_t i = 0; i < FF_CALIB_ARRAY_LENGTH; i++)
                {
                    Serial.print(ffCalibOutput(i));
                    Serial.print(" ");
                    Serial.println(ffCalib[i]);
                    delay(10);
                }
                for (uint32_t i = 0; i < FEEDFORWARD_ARRAY_LENGTH; i += 16)
                {
                    Serial.print(i);
                    Serial.print(" ");
                    Serial.println(feedforward[i]);
                    delay(10);
                }*/
                Serial.println("done");  
            }
#endif
			else
			{
		    	Serial.println("Invalid Command");
			}
		}
		else //if no command is valid
		{
			Serial.println("<var>=<value> to set value and <var>? to read value");
			Serial.println("Valid variables to replace <var>:");
			Serial.println("kp, ki, or kd for the PID constants (floats)");
			Serial.println("sp for the integer value of the default setpoint (int)");
			Serial.println("sv for the voltage (in V) of the default setpoint (float)");
#if DIGITAL_INPUT
			Serial.println("If the input pin is pulled low, then the setpoint switches to an alternate setpoint");
			Serial.println("lp for the integer value of the alternate setpoint (int)");
			Serial.println("lv for the voltage (in V) of the alternate setpoint (float)");
#endif
#if LIMITED_OUTPUT
			Serial.println("If the setpoint value goes outside of the limits, pid control is deactivated and the output jumps to a predetermined setpoint");
			Serial.println("lo for the output corresponding to the low setpoint (in V) (float)");
			Serial.println("ho for the output corresponding to the high setpoint (in V) (float)");
			Serial.println("ls for the low setpoint limit (in V) (float)");
			Serial.println("hs for the high setpoint  limit(in V) (float)");
#endif
			Serial.println("ov for the voltage (in V) of the output (float). Setting it disables PID control.");
			Serial.println("ra for the number of read averages (int)");
			Serial.println("pa to enable PID control (1 to enable, 0 to latch output)");
			Serial.println("po to print the setpoint, feedback, output, and calculation time (1 to enable, 0 to disable)");
#if FEEDFORWARD
            Serial.println("cf to calibrate feedforward");
#endif
            Serial.println("cl to clear integral term, setpoint, etc");
			Serial.println("");
		}
	}
}
#endif

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

#if FEEDFORWARD
void getFeedforwardReadings(uint16_t* readings)
{
    for(uint16_t i = 0; i < FF_CALIB_ARRAY_LENGTH; i++)
    {
        uint16_t outputValue = ffCalibOutput(i);
        analogWrite(PIN_OUTPUT, outputValue);
        readings[i] = analogRead(PIN_INPUT);
    }
}


void calibrateFeedforward(uint16_t* readings)
{//desired is the desired feedback. feedforward should be an array of outputs to give the desired value
    uint16_t maxValue = 0;
    uint16_t maxIndex = 0;
    uint16_t minValue = MAX_INPUT;
    uint16_t minIndex = 0;
    for(uint16_t i = 0; i < FF_CALIB_ARRAY_LENGTH; i++)
    {
        if (readings[i] > maxValue)
        {
            maxIndex = i;
            maxValue = readings[maxIndex];
        }
        if (readings[i] < minValue)
        {
            minIndex = i;
            minValue = readings[minIndex];
        }
    }
    
    for(uint32_t desired = 0; desired < FEEDFORWARD_ARRAY_LENGTH; desired++) 
    {
#if LIMITED_SETPOINT
        if (desired < minsetpoint)
        {
            feedforward[desired] = minoutput;
        }
        else if (desired > maxsetpoint)
        {
            feedforward[desired] = maxoutput;
        }
#else
        if (false)
        {
            
        }
#endif
        else //if within range
        {
            
            
            bool valueFound = false;
            if(desired > maxValue) //if desired value is out of range, linearly interpolate
            {
                feedforward[desired] = ffCalibOutput(maxIndex);
                valueFound = true;
            }
            else if(desired < minValue)
            {
                feedforward[desired] = ffCalibOutput(minIndex);
                valueFound = true;
            }
            else //search through array
            {
                for(uint16_t j = 0; j < FF_CALIB_ARRAY_LENGTH-1 && !valueFound; j++)
                {
                    uint16_t jp = j + 1;
                    if(desired == readings[j] && desired == readings[jp])
                    {
                        feedforward[desired] = ffCalibOutput(j);
                        valueFound = true;
                    }
                    else if(desired >= readings[j] && desired <= readings[jp]) //by the IVT, there must be some set of subsequent rising points with the desired value between
                    {
                        feedforward[desired] = ffCalibOutput(j) + (desired - readings[j])*(ffCalibOutput(jp) - ffCalibOutput(j))/(readings[jp] - readings[j]); //linear interpolate
                        
                        valueFound = true;
                    }
                    if (valueFound)
                        break;
                }
            }
            if (!valueFound) //must be too low
            {
                Serial.println("Can't find");
                Serial.println(desired);
                //feedforward[desired] = ffCalibOutput(0) + (desired - ffCalib[0])*(ffCalibOutput(FF_CALIB_ARRAY_LENGTH-1) - ffCalibOutput(0))/(ffCalib[FF_CALIB_ARRAY_LENGTH-1] - ffCalib[0]); //linear interpolate beyond ends
            }
            //max/min bounding
            if(feedforward[desired] > maxoutput)
            {
                feedforward[desired] = maxoutput;
            }
            else if (feedforward[desired] < minoutput)
            {
                feedforward[desired] = minoutput;
            }
        }
    }
}
#endif
