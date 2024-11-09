#include "adc.h"
#include <Arduino.h>
#include <digitalWriteFast.h>

#define CALIBRATE_CLOCKS 1024

#define adcdigwr(p, s) digitalWriteFast(p, s)
#define adcdigrd(p) digitalReadFast(p)

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
	make output available at ADC_SDO by falling edge of CNVST
barebones
	start acquisition (300ns)
	start conversion (700ns)
	wait 10ns
	collect output
	wait 10ns
continuous
	can start outputting data during input acquisition -- just make sure to wait at least 10ns before starting data conversion
	
*/

#define ACQUISITION_TIME_NS 300  //290 is minimum, 300 is typical
#define CONVERSION_TIME_NS 700//700 is typical, 710 is max
#define MULTIPLE_ACQUISITION_TIME_NS 0
#define WAIT_TIME_NS 10
#define CLOCK_DELAY_NS 1

void initADC() {
	pinMode(ADC_SDO, INPUT_PULLUP);
	pinMode(ADC_SCK, OUTPUT);
	pinMode(ADC_CNVST, OUTPUT);
	digitalWrite(ADC_CNVST, LOW);
	Serial.println("starting...");
	
	delay(100); //ensures that voltage rails have settled
	Serial.println("calibrating...");
	calibrateADC();
}

uint16_t readADCnoAcq(){
	uint16_t retval = 0;
	
	//initiate conversion
	adcdigwr(ADC_CNVST, HIGH); //ensure rising edge 
	delayNanoseconds(CONVERSION_TIME_NS); //TODO need to test quality of this function
	
	//read out data
	adcdigwr(ADC_SCK, LOW);
	adcdigwr(ADC_CNVST, LOW);
	delayNanoseconds(WAIT_TIME_NS);
	for(int8_t i = 15; i >= 0; i--){
		adcdigwr(ADC_SCK, HIGH);
		#if CLOCK_DELAY_NS
			delayNanoseconds(CLOCK_DELAY_NS);
		#endif
		retval |= adcdigrd(ADC_SDO) << i;
		adcdigwr(ADC_SCK, LOW);
		#if CLOCK_DELAY_NS
			delayNanoseconds(CLOCK_DELAY_NS);
		#endif
		//TODO may need to adjust timing here
	}
	
	return retval;
}

//TODO create a continuous read function that is meant to be called repeatedly
uint16_t readADC(){
	
	//initiate acquisition
	adcdigwr(ADC_CNVST, LOW); //ensure rising edge
	delayNanoseconds(ACQUISITION_TIME_NS);

	return readADCnoAcq();
}



uint16_t readADCMultiple(uint8_t power){
	//assume ADC_CNVST is already low and have waited sufficient time
	if(power > 15)
		power = 15;
	
	uint32_t num = 0x1 << power;
	uint32_t sum = 0;
	for(uint16_t i = 0; i < num; i++) {
		sum += readADCnoAcq();
		#if MULTIPLE_ACQUISITION_TIME_NS
			delayNanoseconds(MULTIPLE_ACQUISITION_TIME_NS);
		#endif
	}
	
	return sum >> power;
}

float readADCVolts(){
	return readADC()*ADC_REFERENCE_VOLTAGE/65535.0;
}

void calibrateADC(){
	//initiate acquisition
	adcdigwr(ADC_CNVST, LOW); //ensure rising edge
	delayNanoseconds(ACQUISITION_TIME_NS); //TODO need to test quality of this function
	
	//initiate conversion
	adcdigwr(ADC_CNVST, HIGH); //ensure rising edge 
	delayNanoseconds(CONVERSION_TIME_NS); //TODO need to test quality of this function
	
	//send calibrate command
	adcdigwr(ADC_SCK, LOW);
	adcdigwr(ADC_CNVST, LOW);
	delayNanoseconds(10);
	for(int16_t i = 0; i < CALIBRATE_CLOCKS; i++){
		adcdigwr(ADC_SCK, HIGH);
		#if CLOCK_DELAY_NS
			delayNanoseconds(CLOCK_DELAY_NS);
		#endif
		adcdigwr(ADC_SCK, LOW);
		#if CLOCK_DELAY_NS
			delayNanoseconds(CLOCK_DELAY_NS);
		#endif
	}
	
	Serial.println("waiting for calibration to finish...");
	
	elapsedMillis calibtime; //todo change calibtime to 650
	while(adcdigrd(ADC_SDO) == LOW && calibtime < 2000); //wait for calibration to finish
	if(adcdigrd(ADC_SDO) == LOW){
		Serial.println("Calibration Failed!");
	}
	else{
		Serial.println("Calibration Succeeded!");
	}
	
	adcdigwr(ADC_CNVST, HIGH);
}
