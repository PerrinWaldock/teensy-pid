//#include <SPI.h>  // include the SPI library:

#define CNVST 28
#define SDO 1
#define SCK 27

#define VREF 5

#define CALIBRATE_CLOCKS 1024

#define dw(p, s) digitalWrite(p, s)
#define dr(p) digitalReadFast(p)

/*
 * NOTE: only one SPI controller can be used at a time. However, it might work to create a bitbanged interface... 
 * https://ww1.microchip.com/downloads/en/DeviceDoc/MCP33131-MCP33121-MCP33111-Family-Data-Sheet-DS20006122A.pdf
 * switch to DigitalWriteFast https://forum.pjrc.com/threads/57185-Teensy-4-0-Bitbang-FAST?p=212236&viewfull=1#post212236 
 *  * may need to write duplicate DigitalWriteFast to get the right timing (10ns clock cycle
*/

/*
ADC Read Procedure
	rising edge of conversion start
		ignores state of CNVST after rising edge
	make output available at SDO by falling edge of CNVST
barebones
	start acquisition (300ns)
	start conversion (700ns)
	wait 10ns
	collect output
	wait 10ns
continuous
	can start outputting data during input acquisition -- just make sure to wait at least 10ns before starting data conversion
	
*/

uint16_t readADC();

void calibrateADC(); //TODO see page 39 of datasheet


//SPI.begin();

void setup() {
	pinMode(SDO, INPUT_PULLUP);
	pinMode(SCK, OUTPUT);
	pinMode(CNVST, OUTPUT);
	digitalWrite(CNVST, LOW);
	Serial.begin(115200);
	Serial.println("starting...");
	
	delay(100);
	Serial.println("calibrating...");
	calibrateADC();
}


void loop() {
	while(true) {
		Serial.println(readADC());
		delay(1);
	}
}

//TODO create a continuous read function that is meant to be called repeatedly
uint16_t readADC(){
	uint16_t retval = 0;
	
	//initiate acquisition
	dw(CNVST, LOW); //ensure rising edge
	delayNanoseconds(300); //TODO need to test quality of this function
	
	//initiate conversion
	dw(CNVST, HIGH); //ensure rising edge 
	delayNanoseconds(700); //TODO need to test quality of this function
	
	//read out data
	dw(SCK, LOW);
	dw(CNVST, LOW);
	delayNanoseconds(10);
	for(int8_t i = 15; i >= 0; i--){
		dw(SCK, HIGH);
		retval |= dr(SDO) << i;
		dw(SCK, LOW);
		//TODO may need to adjust timing here
	}
	
	return retval;
}

void calibrateADC(){
	//initiate acquisition
	dw(CNVST, LOW); //ensure rising edge
	delayNanoseconds(300); //TODO need to test quality of this function
	
	//initiate conversion
	dw(CNVST, HIGH); //ensure rising edge 
	delayNanoseconds(700); //TODO need to test quality of this function
	
	//send calibrate command
	dw(SCK, LOW);
	dw(CNVST, LOW);
	delayNanoseconds(10);
	for(int16_t i = 0; i < CALIBRATE_CLOCKS; i++){
		dw(SCK, HIGH);
		delayNanoseconds(50); //wait to settle
		dw(SCK, LOW);
		delayNanoseconds(50); //wait to settle
		//TODO may need to adjust timing here
	}
	
	Serial.println("waiting for calibration to finish...");
	
	elapsedMillis calibtime; //todo change calibtime to 650
	while(dr(SDO) == LOW && calibtime < 2000); //wait for calibration to finish
	if(dr(SDO) == LOW){
		Serial.println("Calibration Failed!");
	}
	else{
		Serial.println("Calibration Succeeded!");
	}
	
	dw(CNVST, HIGH);
}
