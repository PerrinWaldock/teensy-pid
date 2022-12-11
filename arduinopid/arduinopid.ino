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
 * 
 * TODO allow for separate digital and analog inputs so the setpoint can be programmed remotely
 * 
 * TODO print warning and sets flag if output has railed (is set equal to max output) when setpoint has not railed
 * 
 * TODO allow setting calculation period on-the-fly?
 * 
 * TODO create python interface code that uses serial to
 * -reads extrema
 * -sets setpoint
 * -sets setpoint as a fraction of extrema (probably after a calibration)
 * -checks if output has railed
 */

#include "arduinopid.h"
#include <EEPROM.h>     //for permanently saving settings
#include <FastPID.h>  //prepackaged arduino feedback control library

//pid feedback control parameters

#if FEEDFORWARD
float kp=.005, ki=1000, kd=0; //good starting point for feedforward
#else
float kp=0.1, ki=45000, kd=0; //good starting point for non-feedforward
#endif
/*float kp=1;
float ki=kp*SAMPLE_RATE_HZ;
float kd=0;*/
//this is also the setpoint if digital input is low
uint32_t loopRateHz = DEFAULT_SAMPLE_RATE_HZ;
uint32_t loopPeriodUs = DEFAULT_SAMPLE_PERIOD_US;
uint16_t setpoint = volts2int(DEFAULT_SETPOINT, ADC_BITS); //default setpoint (voltage it reads)
#if DIGITAL_INPUT
uint16_t setpointlow = 0;	//default low digital setpoint
#endif
uint8_t readaverages = 4;	//number of times to average the input (hardware), default 4

#if LIMITED_SETPOINT
uint16_t minsetpoint = volts2int(0.05,ADC_BITS);
uint16_t maxsetpoint = volts2int(3.25,ADC_BITS);
uint16_t minoutput = 0;
uint16_t maxoutput = MAX_OUTPUT;

//TODO setOutputRange if not feedforward
#endif


//things that only need to be defined if feedback control is active
#ifdef SERIAL_BAUD
	char instring[INSTRING_LENGTH];   //input string
    char outstring[255];
	uint8_t inind = 0;
	bool serialActive = false;      //serial active flag
	bool printout = false;        //if true, print feedback stats
	volatile bool readSerial = true;  //read serial flag
#endif

volatile bool pidcalc = true; //flag saying to perform feedback control calc
volatile bool pidactive = true;      //flag saying "pid control is active"

#if FEEDFORWARD
    #define FEEDFORWARD_ARRAY_LENGTH 4096//65536 //MAXINPUT+1
    #define FF_CALIB_ARRAY_LENGTH 256
    #define ffCalibOutput(x) (int)((x*((1 << DAC_BITS) - 1))/(FF_CALIB_ARRAY_LENGTH - 1))
    #define CALIBRATION_AVERAGES 64
    uint16_t feedforward[FEEDFORWARD_ARRAY_LENGTH];
    uint16_t ffCalib[FF_CALIB_ARRAY_LENGTH];
#endif

IntervalTimer pidTimer; //used for timing the pid feedback loop
IntervalTimer readTimer; //used for timing the serial read code

FastPID myPID(kp, ki, kd, loopRateHz, DAC_BITS, SIGNED_OUTPUT);; //feedback control object

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

#if SAVE_DATA //TODO also save read averages
    const uint16_t KI_EEPROM_ADDRESS=0, KP_EEPROM_ADDRESS=4, KD_EEPROM_ADDRESS=8;
    const uint16_t LR_EEPROM_ADDRESS=20;
    void savePID(float ki, float kp, float kd, uint32_t looprate);
    void loadPID(float& ki, float& kp, float& kd, uint32_t& looprate);
    #if LIMITED_SETPOINT
    const uint16_t MINSP_EEPROM_ADDRESS = 12, MAXSP_EEPROM_ADDRESS = 14, MINOP_EEPROM_ADDRESS = 16, MAXOP_EEPROM_ADDRESS = 18;
    void saveLimits(uint16_t minsetpoint, uint16_t maxsetpoint, uint16_t minoutput, uint16_t maxoutput);
    void loadLimits(uint16_t& minsetpoint, uint16_t& maxsetpoint, uint16_t& minoutput, uint16_t& maxoutput);
    #endif
    #if FEEDFORWARD
    const uint16_t FFDATA_EEPROM_ADDRESS = 512;
    void saveFFdata(uint16_t* readings);
    void loadFFdata(uint16_t* readings);
    #endif

void saveEverything()
{
    savePID(ki, kp, kd, loopRateHz);
    #if LIMITED_SETPOINT
    //    saveLimits(minsetpoint, maxsetpoint, minoutput, maxoutput);
    #endif
    #if FEEDFORWARD
    saveFFdata(ffCalib);
    #endif
}

void loadEverything()
{
    loadPID(ki, kp, kd, loopRateHz);
    #if LIMITED_SETPOINT
    //    loadLimits(minsetpoint, maxsetpoint, minoutput, maxoutput);
    #endif
    #if FEEDFORWARD
    loadFFdata(ffCalib);
    #endif
}
    
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
	pidTimer.begin(setpidcalc, loopPeriodUs);

#if SAVE_DATA
    loadEverything();
#endif

#if FEEDFORWARD && !SAVE_DATA
    getFeedforwardReadings(ffCalib);
    calibrateFeedforward(ffCalib);   
#endif

}

//feedback control variables 
uint16_t feedback = 0;
int32_t out = 0;
uint16_t sp = 0;
#if TIME_FEEDBACK_LOOP
	uint32_t ts = 0;
	uint32_t tss = 0;
#endif

//main loop
void loop()
{ 
    bool skipcalc = false; //set to true if the PID calc should be skipped (if pid setpoint is out of limits)
	//run feedback control loop
	if(pidcalc)
	{
       pidcalc = false; //resets flag
		#if TIME_FEEDBACK_LOOP
			ts = micros();
		#endif
		if(pidactive) //only run if pid is active TODO add logic that skips the wait for digital control and restarts the PID clock
		{
            #if FEEDFORWARD
            bool onlyff = false;
            #endif
			#if ANALOG_INPUT
				sp = analogRead(PIN_REFERENCE);	
                setpoint = sp;		
			#endif
			#if DIGITAL_INPUT
				if (digitalReadFast(PIN_REFERENCE) == LOW)
				{
                    #if FEEDFORWARD //skip the calculation on first step after changing state
                    if(sp != setpointlow) //if the setpoint is not the desired setpoint
                    {
                        onlyff = true;
                        sp = setpointlow;
                    }
                    #else
                        sp = setpointlow;
                    #endif	
				}
				else
				{
                    #if FEEDFORWARD //skip the calculation on first step after changing state
                    if(sp != setpoint) //if the setpoint is not the desired setpoint
                    {
                        onlyff = true;
                        sp = setpoint;
                    }
                    #else
                        sp = setpoint;
                    #endif			
				}
			#endif
			#if !DIGITAL_INPUT && !ANALOG_INPUT
				sp = setpoint;
			#endif


            #if LIMITED_SETPOINT //TODO look into using "clear" here
                if(sp < minsetpoint)
                {
                    #if NEGATIVE_OUTPUT
                    out = maxoutput;
                    #else
                    out = minoutput;
                    #endif
                    skipcalc = true;
                    
                }
                else if(sp > maxsetpoint)
                {
                    #if NEGATIVE_OUTPUT
                    out = minoutput;
                    #else
                    out = maxoutput;
                    #endif
                    skipcalc = true;
                }
            #endif

            if (skipcalc)
            {
                pidTimer.begin(setpidcalc, loopPeriodUs);        
                pidcalc = true; //increase the response time by polling digital input more frequently
            }
            else
            {
                #if FEEDFORWARD
                    out = feedforward[sp];
                    if(!onlyff)
                    {
                        feedback = analogRead(PIN_INPUT); //only collects feedback
                    #if NEGATIVE_OUTPUT
                        out -= myPID.step(sp, feedback);
                    #else
                        out += myPID.step(sp, feedback);
                    #endif
                    }
                #else
                    feedback = analogRead(PIN_INPUT); //only collects feedback
                    out = myPID.step(sp, feedback);              
                    #if NEGATIVE_OUTPUT
                        out = flipoutput(out);
                    #endif
                #endif
    
                #if LIMITED_SETPOINT
    				out = bound(out, min(minoutput, maxoutput), max(minoutput, maxoutput));
    			#endif
            }
          
		}
		analogWrite(PIN_OUTPUT, out); //still write output
        if(!skipcalc)
        {
            delayMicroseconds(1); //let analog output settle
        }
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
                if(skipcalc)
                {
                    feedback = analogRead(PIN_INPUT); //currently don't collect input if skipping the calculation
                }
				Serial.print("s:");
				Serial.print(sp);
				Serial.print("\t f:");
				Serial.print(feedback);
                #if FEEDFORWARD
                Serial.print("\t ff:");
                Serial.print(feedforward[sp]);
                #endif
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
        else if(!strcmp(string, "lf"))
        {
            loopRateHz = atoi(&(string[3]));
            loopPeriodUs = 1000000/loopRateHz;
            Serial.print("Loop Frequency=");
            Serial.print(loopRateHz);
            Serial.println("Hz");
            updatePID=true;
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
			maxsetpoint = volts2int(atof(&(string[3])), DAC_BITS);
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
#if FEEDFORWARD
       else if(!strcmp(string, "cf")) //calibrate feedforward
        {
            if(atoi(&(string[3])))
            {
                Serial.println("cf=1");
                Serial.println("calibrating feedforward");
                getFeedforwardReadings(ffCalib);
                calibrateFeedforward(ffCalib);
                Serial.println("done");
            }
            else
            {
                Serial.println("cf=0");
            }
        }
#endif
#if SAVE_DATA
        else if (!strcmp(string, "ew"))
        {
            bool write2eeprom = atoi(&(string[3]));
            Serial.print("ee=");
            Serial.println(write2eeprom);
            if(write2eeprom)
            {
                Serial.println("Writing...");
                saveEverything();
                Serial.println("done");
            }
            else
            {
                Serial.println("Reading...");
                loadEverything();
                Serial.println("done");
            }
        }
#endif
		else
		{
			Serial.println("Invalid Command");
		}

        if (updatePID)
        {
            bool success = myPID.configure(kp, ki, kd, loopRateHz, DAC_BITS, SIGNED_OUTPUT);
            myPID.clear();
    
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
      else if(!strcmp(string, "lf"))
      {
          Serial.print("Loop Frequency=");
          Serial.print(loopRateHz);
          Serial.println("Hz");
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
            else if(!strcmp(string, "cd")) //feedforward calibration data
            {
                Serial.println("Feedforward Calibration Data:");
                Serial.println("Output, Input");
                for (uint32_t i = 0; i < FF_CALIB_ARRAY_LENGTH; i++)
                {
                    Serial.print(ffCalibOutput(i));
                    Serial.print(", ");
                    Serial.println(ffCalib[i]);
                    delay(10);
                }
            }
            else if(!strcmp(string, "cf")) //feedforward array
            {
                Serial.println("Feedforward Array:");
                Serial.println("Desired, Output");
                for (uint32_t i = 0; i < FEEDFORWARD_ARRAY_LENGTH; i += 16)
                {
                    Serial.print(i);
                    Serial.print(", ");
                    Serial.println(feedforward[i]);
                    delay(10);
                }
            }
            else if(!strcmp(string, "ce"))
            {
                uint16_t maxIndex = 0;
                uint16_t minIndex = 0;
                calibrationExtrema(ffCalib, maxIndex, minIndex);
                uint16_t maxValue = ffCalib[maxIndex];
                uint16_t minValue = ffCalib[minIndex];
                Serial.print("min: ");
                Serial.print(minValue);
                Serial.print(" (");
                Serial.print(int2volts(minValue, ADC_BITS), 4);
                Serial.print(" V)\t");
                Serial.print("max: ");
                Serial.print(maxValue);
                Serial.print(" (");
                Serial.print(int2volts(maxValue, ADC_BITS), 4);
                Serial.println(" V) ");
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
            Serial.println("lf for the loop frequency (in Hz) of the feedback loop (int)");
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
            Serial.println("cf to calibrate feedforward (cd to see calibration data, ce to see calibration extrema)");
#endif
#if SAVE_DATA
            Serial.println("ew to interact with eeprom (1 to write settings, 0 to read settings)");
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
void getFeedforwardReadings(uint16_t* readings) //TODO should ramp up and down several times then average out
{
    analogReadAveraging(CALIBRATION_AVERAGES);
    for(uint16_t i = 0; i < FF_CALIB_ARRAY_LENGTH; i++)
    {
        uint16_t outputValue = ffCalibOutput(i);
        analogWrite(PIN_OUTPUT, outputValue);
        delayMicroseconds(2); //wait for output to stabilize
        readings[i] = analogRead(PIN_INPUT);
    }

    for(uint16_t i = 0; i < FF_CALIB_ARRAY_LENGTH; i++)
    {
        uint16_t outputValue = ffCalibOutput(i);
        analogWrite(PIN_OUTPUT, outputValue);
        delayMicroseconds(2); //wait for output to stabilize
        readings[i] = (analogRead(PIN_INPUT) + readings[i])/2;
    }
    analogReadAveraging(readaverages);

}

void calibrationExtrema(uint16_t* readings, uint16_t& maxIndex, uint16_t& minIndex)
{
    maxIndex = 0;
    minIndex = 0;
    for(uint16_t i = 0; i < FF_CALIB_ARRAY_LENGTH; i++)
    {
        if (readings[i] > readings[maxIndex])
        {
            maxIndex = i;
        }
        if (readings[i] < readings[minIndex])
        {
            minIndex = i;
        }
    }
}


void calibrateFeedforward(uint16_t* readings)
{//desired is the desired feedback. feedforward should be an array of outputs to give the desired value
    uint16_t maxIndex = 0;
    uint16_t minIndex = 0;
    calibrationExtrema(readings, maxIndex, minIndex);
    uint16_t maxValue = readings[maxIndex];
    uint16_t minValue = readings[minIndex];

    for(uint32_t desired = 0; desired < FEEDFORWARD_ARRAY_LENGTH; desired++)  //iterates through calibration array
    {
//not convinced this helps - I'll trust in the feedforward array
/*
#if LIMITED_SETPOINT
    #if NEGATIVE_OUTPUT
        if (desired < minsetpoint)
        {
            feedforward[desired] = minoutput;
        }
        else if (desired > maxsetpoint)
        {
            feedforward[desired] = maxoutput;
        }
    #else
        if (desired < minsetpoint)
        {
            feedforward[desired] = maxoutput;
        }
        else if (desired > maxsetpoint)
        {
            feedforward[desired] = minoutput;
        }

    #endif
#else
        if (false) //magic to make the predfine chain make sense
        {
            
        }
#endif
        else //if within range
        {
*/ 
            
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
            else //search through calibration array for desired to be between two adjacent points
            {
                for(uint16_t j = 0; j < FF_CALIB_ARRAY_LENGTH-1 && !valueFound; j++)
                {
                    uint16_t jp = j + 1;
                    if(desired == readings[j])
                    {
                        feedforward[desired] = ffCalibOutput(j);
                        valueFound = true;
                    }
                    //readings[j] must be at least one smaller than desired and readings[jp] must be equal or larger so never will divide by zero
                    else if((desired > readings[j]) && (desired <= readings[jp])) //by the IVT, there must be some set of subsequent rising points with the desired value between
                    {
                        feedforward[desired] = ffCalibOutput(j) + ((int32_t)((desired - readings[j])*(ffCalibOutput(jp) - ffCalibOutput(j))))/(readings[jp] - readings[j]); //linear interpolate
                        valueFound = true;
                    }
                    else if (desired < readings[j] && desired >= readings[jp])
                    {
                        feedforward[desired] = ffCalibOutput(jp) + ((int32_t)(readings[j] - desired)*(ffCalibOutput(jp) - ffCalibOutput(j)))/(readings[j] - readings[jp]); //linear interpolate
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

            #if LIMITED_SETPOINT
                minsetpoint = 1.5*minValue + 1;
                maxsetpoint = .98*maxValue - 1;
                feedforward[desired] = bound(feedforward[desired], min(minoutput, maxoutput), max(minoutput, maxoutput));
            #endif
        //}
    }
}
#endif

//TODO start passing stuff by reference and use the new-fangled C++ tricks like that

#if SAVE_DATA
    void savePID(float ki, float kp, float kd, uint32_t lr)
    {
        EEPROM.put(KI_EEPROM_ADDRESS, ki);
        EEPROM.put(KP_EEPROM_ADDRESS, kp);
        EEPROM.put(KD_EEPROM_ADDRESS, kd);
        EEPROM.put(LR_EEPROM_ADDRESS, lr);
    
    }
    void loadPID(float& ki, float& kp, float& kd, uint32_t& lr)
    {
        EEPROM.get(KI_EEPROM_ADDRESS, ki);
        EEPROM.get(KP_EEPROM_ADDRESS, kp);
        EEPROM.get(KD_EEPROM_ADDRESS, kd);
        EEPROM.get(LR_EEPROM_ADDRESS, lr);
    }
    #if LIMITED_SETPOINT
    void saveLimits(uint16_t minsetpoint, uint16_t maxsetpoint, uint16_t minoutput, uint16_t maxoutput)
    {
        EEPROM.put(MINSP_EEPROM_ADDRESS, minsetpoint);
        EEPROM.put(MAXSP_EEPROM_ADDRESS, maxsetpoint);
        EEPROM.put(MINOP_EEPROM_ADDRESS, minoutput);
        EEPROM.put(MAXOP_EEPROM_ADDRESS, maxoutput);
    }
    void loadLimits(uint16_t& minsetpoint, uint16_t& maxsetpoint, uint16_t& minoutput, uint16_t& maxoutput)
    {
        EEPROM.get(MINSP_EEPROM_ADDRESS, minsetpoint);
        EEPROM.get(MAXSP_EEPROM_ADDRESS, maxsetpoint);
        EEPROM.get(MINOP_EEPROM_ADDRESS, minoutput);
        EEPROM.get(MAXOP_EEPROM_ADDRESS, maxoutput);
    
    }
    #endif
    #if FEEDFORWARD
    void saveFFdata(uint16_t* readings)
    {
        for(uint16_t j = 0; j < FF_CALIB_ARRAY_LENGTH; j++)
        {
            EEPROM.put(FFDATA_EEPROM_ADDRESS+(j*sizeof(*readings)), readings[j]);
        }
    }
    void loadFFdata(uint16_t* readings)
    {
        for(uint16_t j = 0; j < FF_CALIB_ARRAY_LENGTH; j++)
        {
            EEPROM.get(FFDATA_EEPROM_ADDRESS+(j*sizeof(*readings)), readings[j]);
        }
        calibrateFeedforward(readings);
    }
    #endif
#endif
