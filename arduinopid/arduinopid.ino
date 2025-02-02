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
 * TODO print warning and sets flag if output has railed (is set equal to max output) when setpoint has not railed
 * 
 */

 /*
  * TODO
  - -add optional buffer that stores output values to be printed at a later date (ro=1, ro=0, ro?)
  * -clean up code, split into multiple files
  * -look through previous todos
  * -(MAYBE) use one digital input to turn feedback control on/off, and use the other as a setpoint
  */

#include "arduinopid.h"
#include <EEPROM.h>     //for permanently saving settings
#include "CircularBuffer.h" //prepackaged Circular Buffer

//pid feedback control parameters

#if FEEDFORWARD
float kp=.005, ki=1000, kd=0; //good starting point for feedforward
#else
float kp=0.01, ki=100, kd=0; //good starting point for non-feedforward
#endif
/*float kp=1;
float ki=kp*SAMPLE_RATE_HZ;
float kd=0;*/
//this is also the setpoint if digital input is low
uint32_t loopRateHz = DEFAULT_SAMPLE_RATE_HZ;
#define loopPeriodUs (uint32_t)(1000000/loopRateHz)
uint16_t setpoint = ADC_VOLTS2BITS(DEFAULT_SETPOINT); //default setpoint (voltage it reads)

#if INPUT_MODE == DIGITAL_INPUT
	uint16_t setpoints[INPUT_STATES];	//default low digital setpoint
	uint8_t statenumber = 0;
#endif
uint8_t readaveragespower = READDAVERAGESPOWER;	//number of times to average the input (hardware)


#if LIMITED_SETPOINT
	uint16_t minsetpoint = ADC_VOLTS2BITS(0.05);
	uint16_t maxsetpoint = ADC_VOLTS2BITS(.95*ADC_VREF);
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
	uint8_t printout = 0;        //if true, print feedback stats
	volatile bool readSerial = true;  //read serial flag
#endif

volatile bool pidcalc = true; //flag saying to perform feedback control calc
volatile bool pidactive = true;      //flag saying "pid control is active"

#if FEEDFORWARD
    const uint32_t FEEDFORWARD_ARRAY_LENGTH = (MAX_INPUT+1); //want to have a value for each and every setpoint
    const uint32_t FF_CALIB_ARRAY_LENGTH = (FEEDFORWARD_ARRAY_LENGTH/16);//1024//length of array used to generate the abovearray
    #define ffCalibOutput(x) (int)((x*((1 << DAC_BITS) - 1))/(FF_CALIB_ARRAY_LENGTH - 1))
    #define CALIBRATION_AVERAGES_POWER 8
	#define CALIBRATION_SETTLE_DELAY_US 10
	#define CALIBRATION_PRE_SETTLE_DELAY_US 10000
    uint16_t feedforward[FEEDFORWARD_ARRAY_LENGTH];
    uint16_t ffCalib[FF_CALIB_ARRAY_LENGTH];
#endif

#if RECORD_INPUT
	typedef enum{
		LOG_OFF, //don't log output
		LOG_SINGLE, //log until full
		LOG_CONTINUOUS	//continue logging, overwriting old data
	}LogState;
	const uint32_t INPUT_LOG_SIZE = 150000/2;
	CircularBuffer<uint16_t, INPUT_LOG_SIZE> inputLog;
	CircularBuffer<uint16_t, INPUT_LOG_SIZE> outputLog;
	LogState logstate = LOG_OFF;
#endif

#if INPUT_MODE == ANALOG_INPUT
	#define readAnalogReference() analogRead(PIN_REFERENCE) << (ADC_BITS - ANALOGREFERENCERESOLUTION);
#endif

IntervalTimer pidTimer; //used for timing the pid feedback loop
IntervalTimer readTimer; //used for timing the serial read code

FastPID myPID(kp, ki, kd, loopRateHz, DAC_BITS, SIGNED_OUTPUT); //feedback control object

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
        saveLimits(minsetpoint, maxsetpoint, minoutput, maxoutput);
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
#if INPUT_MODE == DIGITAL_INPUT
	pinMode(PIN_REFERENCE0, INPUT);
	pinMode(PIN_REFERENCE1, INPUT);
#elif INPUT_MODE == ANALOG_INPUT
	pinMode(PIN_REFERENCE, INPUT);
	analogReadAveraging(ANALOGREADAVERAGES);
	analogReadResolution(ANALOGREFERENCERESOLUTION);
#endif
    initADC();
    initDAC();
	
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

	myPID.clear(); 
	myPID.configure(kp, ki, kd, loopRateHz, DAC_BITS, SIGNED_OUTPUT);
	myPID.clear(); 


	Serial.println(PARAM_MULT);
}

//feedback control variables 
int32_t feedback = 0;
int32_t out = 0;
int32_t sp = 0;

uint64_t loopTime;
//main loop
void loop()
{ 
	#if RECORD_INPUT
		bool loggedin = false;
		int32_t prevout = out;
	#endif
    bool skipcalc = false; //set to true if the PID calc should be skipped (if pid setpoint is out of limits)
	//run feedback control loop
    #if TIME_FEEDBACK_LOOP
        elapsedMicros timeSinceLoopStart;
		//uint64_t tstart = micros();
		//uint64_t tend = 0;
    #endif
	if(pidcalc)
	{
		pidcalc = false; //resets flag
		#if FEEDFORWARD
			bool onlyff = false;
		#endif
		if(pidactive) //only run if pid is active
		{
			#if INPUT_MODE == ANALOG_INPUT
				sp = readAnalogReference();	
                setpoint = sp;		
			#elif INPUT_MODE == DIGITAL_INPUT
				statenumber = digitalReadFast(PIN_REFERENCE1) << 1 | digitalReadFast(PIN_REFERENCE0);
				int32_t oldsp = sp;
				sp = setpoints[statenumber];
				setpoint = sp;
				bool changedstate = sp != oldsp;


                #if FEEDFORWARD //skip feedback calculation on first step after changing state
                    if(changedstate)
                    {
                        onlyff = true;
                    }
				#endif
			#else
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

					#if ! FEEDFORWARD && CLEAR_INTEGRAL_WHEN_RAILED
						myPID.clear(); //reduces overshoot
					#endif
                    
                }
                else if(sp > maxsetpoint)
                {
                    #if NEGATIVE_OUTPUT
                    out = minoutput;
                    #else
                    out = maxoutput;
                    #endif
                    skipcalc = true;

					#if ! FEEDFORWARD
						myPID.clear(); //reduces overshoot
					#endif
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
                        feedback = readADCMultiple(readaveragespower); //only collects feedback
						#if RECORD_INPUT
							if ((logstate == LOG_SINGLE && !inputLog.isFull()) || logstate == LOG_CONTINUOUS) {
								loggedin=true;
							}
						#endif
						#if NEGATIVE_OUTPUT
							out -= myPID.step(sp - HALF_MAX_INPUT, feedback - HALF_MAX_INPUT);
						#else
							out += myPID.step(sp - HALF_MAX_INPUT, feedback - HALF_MAX_INPUT);
						#endif
                    }
                #else
                    feedback = readADCMultiple(readaveragespower); //only collects feedback
						#if RECORD_INPUT
							if ((logstate == LOG_SINGLE && !inputLog.isFull()) || logstate == LOG_CONTINUOUS) {
								loggedin=true;
							}
						#endif
                    out = myPID.step(sp - HALF_MAX_INPUT, feedback - HALF_MAX_INPUT);     
					#if DAC_BITS == 16
						out = out << 1; //maximum output is maximum of int16_t
					#endif         
                    #if NEGATIVE_OUTPUT
                        out = flipoutput(out);
                    #endif
                #endif
    
                #if LIMITED_SETPOINT
    				out = bound(out, min(minoutput, maxoutput), max(minoutput, maxoutput));
    			#endif
            }
          
		}
		writeDAC(out);
		#if RECORD_INPUT
		if (loggedin) {
			inputLog.push((uint16_t)feedback);
			outputLog.push((uint16_t)prevout);
		}
		#endif
		if(onlyff){
			delayMicroseconds(OUTPUT_SETTLE_DELAY_US);
		}
		#if TIME_FEEDBACK_LOOP
			loopTime = timeSinceLoopStart;
			//loopTime = micros() - tstart;
			//tend =  micros();
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
				else if (instring[inind] == '\n' || instring[inind] == '\r')
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
			uint16_t adcValue;
			uint16_t setpointvalue = sp;
			if(printout) {
                if(skipcalc || !pidactive) {
                    adcValue = readADCMultiple(readaveragespower); //currently don't collect input if skipping the calculation
					#if INPUT_MODE == ANALOG_INPUT
						setpointvalue = readAnalogReference();	
					#endif
                }
				else {
					adcValue = feedback;
				}
			}
			//prints parameters
			if(printout==1)
			{
                //if(skipcalc || !pidcalc) //TODO check to make sure this is the correct condition
				Serial.print("s:");
				Serial.print(setpointvalue);
				Serial.print("\t f:");
				Serial.print(adcValue);
                #if FEEDFORWARD
                    Serial.print("\t ff:");
                    Serial.print(feedforward[setpointvalue]);
                #endif
				Serial.print("\t o:");
				#ifdef TIME_FEEDBACK_LOOP
					Serial.print(out);
					Serial.print("\t t:");
					//Serial.printf("\t %i\t", tend - tstart);
					Serial.println(loopTime);
				#else
					Serial.println(out);
				#endif
			}
			else if(printout==2)
			{
				Serial.printf("s:%0.5fV\t f:%0.5fV", ADC_BITS2VOLTS(setpointvalue), ADC_BITS2VOLTS(adcValue));
                #if FEEDFORWARD
                    Serial.printf("\t ff:%0.5fV", DAC_BITS2VOLTS(feedforward[setpointvalue]));
                #endif
				Serial.printf("\t o:%0.5fV", DAC_BITS2VOLTS(out));
				#ifdef TIME_FEEDBACK_LOOP
					Serial.printf("\t t:%ius", loopTime);
				#endif
				Serial.printf("\n");
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
            float tempk = atof(&(string[3]));
            if ((tempk < KP_MIN ||tempk > KP_MAX) && tempk != 0) {
                Serial.printf("kp=%f out of range (%f, %f)\n", tempk, KP_MIN, KP_MAX);
            }
            else {
                kp = tempk;
                Serial.print("kp=");
                Serial.println(kp,4);
                updatePID = true;
            }
		}
		else if(!strcmp(string, "ki"))
		{
            float tempk = atof(&(string[3]));
            if ((tempk < KP_MIN*loopRateHz || tempk > KP_MAX*loopRateHz) && tempk != 0) {
                Serial.printf("ki=%f out of range (%f, %f)\n", tempk, (float)KP_MIN*loopRateHz, (float)KP_MAX*loopRateHz);
            }
            else {
				ki = tempk;
                Serial.print("ki=");
                Serial.println(ki,6);
                updatePID = true;
            }
			//_sum = 0; //reset integral
		}
		else if(!strcmp(string, "kd"))
		{
            float tempk = atof(&(string[3]));
            if ((tempk < KP_MIN/loopRateHz || tempk > KP_MAX/loopRateHz) && tempk != 0) {
                Serial.printf("kd=%f out of range (%f, %f)\n", tempk, (float)KP_MIN/loopRateHz, (float)KP_MAX/loopRateHz);
            }
            else {
				kd = tempk;
                Serial.print("kd=");
                Serial.println(kd,8);
                updatePID = true;
            }
		}
#if INPUT_MODE == DIGITAL_INPUT
		else if(string[0] == 's' && string[1] == 'p')
		{	
			uint8_t statenum = atoi(&(string[2])); //default is zero
			char* valstr;
			if (string[2] == '=') {
				valstr = &(string[3]);
			}
			else {
				valstr = &(string[4]);
			}

			if (statenum < INPUT_STATES)
			{
				setpoints[statenum] = atoi(valstr);
				Serial.print("setpoint");
				Serial.print(statenum);
				Serial.print("=");
				Serial.println(setpoints[statenum]);
			}
			else
			{
				Serial.print("input state does not exist");
			}
		}
		else if(string[0] == 's' && string[1] == 'v')
		{	
			uint8_t statenum;
			const char* valstr = peql+1;
			if (peql - string == 2) {
				statenum = 0;
			}
			else {
				statenum = atoi(peql-1);
			}

			if (statenum < INPUT_STATES)
			{
				setpoints[statenum] = ADC_VOLTS2BITS(atof(valstr));
				Serial.print("setpoint");
				Serial.print(statenum);
				Serial.print("=");
				Serial.print(ADC_BITS2VOLTS(setpoints[statenum]),4);
				Serial.print("V, ");
				Serial.println(setpoints[statenum]);
			}
			else
			{
				Serial.print("input state does not exist");
			}
		}
		else if(!strcmp(string, "sn"))
		{	
			uint8_t statenum = atoi(&(string[3]));
			if (statenum < INPUT_STATES)
			{
				statenumber = statenum;
				Serial.print("using sp");
				Serial.print(statenumber);
				Serial.print("=");
				Serial.println(setpoints[statenum]);
			}
			else
			{
				Serial.print("input state does not exist");
			}
		}
#else
		else if(!strcmp(string, "sp"))
		{
			setpoint = atof(&(string[3]));
			Serial.print("setpoint=");
			Serial.println(setpoint);
		}
		else if(!strcmp(string, "sv"))
		{
			setpoint = ADC_VOLTS2BITS(atof(&(string[3])));
			Serial.print("setpoint=");
			Serial.print(ADC_BITS2VOLTS(setpoint),4);
			Serial.print("V, ");
			Serial.println(setpoint);
		}
#endif
        else if(!strcmp(string, "lf"))
        {
            loopRateHz = atoi(&(string[3]));
            Serial.print("Loop Frequency=");
            Serial.print(loopRateHz);
            Serial.println("Hz");
			pidTimer.begin(setpidcalc, loopPeriodUs);
            updatePID=true;
        }
#if LIMITED_SETPOINT
		else if(!strcmp(string, "lo"))
		{
			#if NEGATIVE_OUTPUT
				minoutput = flipoutput(DAC_VOLTS2BITS(atof(&(string[3]))));
			#else
				minoutput = DAC_VOLTS2BITS(atof(&(string[3])));
			#endif
			Serial.print("Output Limits: ");
			Serial.print(DAC_BITS2VOLTS(minoutput), 4);
			Serial.print("V, ");
			Serial.print(DAC_BITS2VOLTS(maxoutput), 4);
			Serial.print("V\n");
		}
		else if(!strcmp(string, "ho"))
		{
			#if NEGATIVE_OUTPUT
				maxoutput = flipoutput(DAC_VOLTS2BITS(atof(&(string[3]))));
			#else
				maxoutput = DAC_VOLTS2BITS(atof(&(string[3])));
			#endif			
			Serial.print("Output Limits: ");
			Serial.print(DAC_BITS2VOLTS(minoutput), 4);
			Serial.print("V, ");
			Serial.print(DAC_BITS2VOLTS(maxoutput), 4);
			Serial.print("V\n");
		}
		else if(!strcmp(string, "ls"))
		{
			minsetpoint = DAC_VOLTS2BITS(atof(&(string[3])));
			Serial.print("Setpoint Limits: ");
			Serial.print(ADC_BITS2VOLTS(minsetpoint), 4);
			Serial.print("V, ");
			Serial.print(ADC_BITS2VOLTS(maxsetpoint), 4);
			Serial.print("V\n");
		}
		else if(!strcmp(string, "hs"))
		{
			maxsetpoint = DAC_VOLTS2BITS(atof(&(string[3])));
			Serial.print("Setpoint Limits: ");
			Serial.print(ADC_BITS2VOLTS(minsetpoint), 4);
			Serial.print("V, ");
			Serial.print(ADC_BITS2VOLTS(maxsetpoint), 4);
			Serial.print("V\n");
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
			readaveragespower = atoi(&(string[3]));
			Serial.print("read averages=");
			Serial.println(1<<readaveragespower);
#if INPUT_MODE == ANALOG_INPUT
			analogReadAveraging(ANALOGREADAVERAGES);
#endif
		}
		else if(!strcmp(string, "ov"))
		{
			pidactive = false;
			out = DAC_VOLTS2BITS(atof(&(string[3])));
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
            Serial.println(&string[3]);
            Serial.println(atof(&(string[3])));
            Serial.println(DAC_VOLTS2BITS(atof(&(string[3]))));
			Serial.print("pidactive=");
			Serial.println(pidactive);
			Serial.print("output=");
			Serial.print(DAC_BITS2VOLTS(out), 4);
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
				myPID.configure(kp, ki, kd, loopRateHz, DAC_BITS, SIGNED_OUTPUT);  
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
#if RECORD_INPUT
		else if (!strcmp(string, "li")) {
			logstate = (LogState)atoi(&(string[3]));
			Serial.printf("li=%i\n", logstate);
		}
#endif
		else
		{
			Serial.println("Invalid Command");
		}

        if (updatePID)
        {
            myPID.clear();
            bool success = myPID.configure(kp, ki, kd, loopRateHz, DAC_BITS, SIGNED_OUTPUT);
    
            if(!success)
            {
                Serial.println("Error setting coefficients");
            }      
			else {
				Serial.printf("f=%i, kp=%f, ki=%f kd=%f\n", loopRateHz, kp, ki, kd);
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
			else if(!strcmp(string, "ks"))
			{
				Serial.printf("f=%i, kp=%f, ki=%f kd=%f\n", loopRateHz, kp, ki, kd);
			}
			else if(!strcmp(string, "sp") || !strcmp(string, "sv"))
			{
#if INPUT_MODE == DIGITAL_INPUT
				for(uint8_t i = 0; i < INPUT_STATES; i++)
				{
					if (i == statenumber)
					{
						Serial.print("-> ");
					}
					Serial.print("setpoint");
					Serial.print(i);
					Serial.print("=");
					Serial.print(ADC_BITS2VOLTS(setpoints[i]));
					Serial.print("V, ");
					Serial.println(setpoints[i]);
				}
#else
				Serial.print("setpoint=");
				Serial.print(ADC_BITS2VOLTS(setpoint));
				Serial.print("V, ");
				Serial.println(setpoint);
#endif
			}

      else if(!strcmp(string, "lf"))
      {
          Serial.print("Loop Frequency=");
          Serial.print(loopRateHz);
          Serial.println("Hz");
      }
			#if LIMITED_SETPOINT
				else if(!strcmp(string, "lo") || !strcmp(string, "ho"))
				{
					Serial.print("Output Limits: ");
					Serial.print(DAC_BITS2VOLTS(minoutput));
					Serial.print("V, ");
					Serial.print(DAC_BITS2VOLTS(maxoutput));
					Serial.print("V\n");
				}
				else if(!strcmp(string, "ls") || !strcmp(string, "hs"))
				{
					Serial.print("Setpoint Limits: ");
					Serial.print(ADC_BITS2VOLTS(minsetpoint), 4);
					Serial.print("V, ");
					Serial.print(ADC_BITS2VOLTS(maxsetpoint), 4);
					Serial.print("V\n");
				}
			#endif
			else if(!strcmp(string, "ov"))
			{
				Serial.print("output=");
				Serial.print(DAC_BITS2VOLTS(out),4);
				Serial.print("V, ");
				Serial.println(out);
			}
			else if(!strcmp(string, "ra"))
			{
				Serial.print("read averages=");
				Serial.println(1<<readaveragespower);
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
                Serial.print(ADC_BITS2VOLTS(minValue), 4);
                Serial.print(" V)\t");
                Serial.print("max: ");
                Serial.print(maxValue);
                Serial.print(" (");
                Serial.print(ADC_BITS2VOLTS(maxValue), 4);
                Serial.println(" V) ");
            }
#endif
#if RECORD_INPUT
			else if(!strcmp(string, "li")) {
				Serial.printf("Input Values (%i %i recorded, state %i, oldest first):\n", inputLog.size(), outputLog.size(), logstate);
				logstate = LOG_OFF;
				while(inputLog.size() > 0) {
					Serial.printf("%i, %i\n", inputLog.shift(), outputLog.shift());
				}
				inputLog.clear();
				outputLog.clear();
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
			Serial.println("kp, ki, or kd for the PID constants (floats) (ks? to view all of them)");
			Serial.println("sp for the integer value of the default setpoint (int)");
			Serial.println("sv for the voltage (in V) of the default setpoint (float)");
#if INPUT_MODE == DIGITAL_INPUT //TODO
			Serial.println("Use sv<int> or sp<int> to set the set voltage");
#endif
            Serial.println("lf for the loop frequency (in Hz) of the feedback loop (int)");
#if LIMITED_SETPOINT
			Serial.println("If the setpoint value goes outside of the limits, pid control is deactivated and the output jumps to a predetermined setpoint");
			Serial.println("lo for the lower output limit (in V) (float)");
			Serial.println("ho for the upper output limit (in V) (float)");
			Serial.println("ls for the low setpoint limit (in V) (float)");
			Serial.println("hs for the high setpoint  limit(in V) (float)");
#endif
			Serial.println("ov for the voltage (in V) of the output (float). Setting it disables PID control.");
			Serial.println("ra for the number of read averages (averages = 2^x where x is the integer input)");
			Serial.println("pa to enable PID control (1 to enable, 0 to latch output)");
			Serial.println("po to print the setpoint, feedback, feedforward value (if enabled), output, and calculation time (2 to print floats, 1 to print integers, 0 to disable)");
#if FEEDFORWARD
            Serial.println("cf to calibrate feedforward (cd to see calibration data, ce to see calibration extrema)");
#endif
#if SAVE_DATA
            Serial.println("ew to interact with eeprom (1 to write settings, 0 to read settings)");
#endif
#if RECORD_INPUT
			Serial.printf("li to log input -- %i to turn off, %i to log until full, %i to log continuously, li? to print and clear the log\n", LOG_OFF, LOG_SINGLE, LOG_CONTINUOUS);
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
	writeDAC(0);
	delayMicroseconds(CALIBRATION_PRE_SETTLE_DELAY_US);
    for(uint16_t i = 0; i < FF_CALIB_ARRAY_LENGTH; i++)
    {
        uint16_t outputValue = ffCalibOutput(i);
        writeDAC(outputValue);
        delayMicroseconds(CALIBRATION_SETTLE_DELAY_US); //wait for output to stabilize
        readings[i] = readADCMultiple(CALIBRATION_AVERAGES_POWER);
    }

    //repeat and average readings (this time from opposite direction)
	delayMicroseconds(CALIBRATION_PRE_SETTLE_DELAY_US);
    for(int32_t i = FF_CALIB_ARRAY_LENGTH; i >= 0; i--)
    {
        uint16_t outputValue = ffCalibOutput(i);
        writeDAC(outputValue);
        delayMicroseconds(CALIBRATION_SETTLE_DELAY_US); //wait for output to stabilize
        readings[i] = (readADCMultiple(CALIBRATION_AVERAGES_POWER) + readings[i])/2;
    }
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
                minsetpoint = 2*minValue + 1;
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
