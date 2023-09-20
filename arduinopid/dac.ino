#include "dac.h"

#define dacdigwr(p, s) digitalWrite(p, s) //fast write may not work well
#define dacdigrd(p) digitalReadFast(p)

#define WRITE_DELAY_NS 0

/*
 *https://www.analog.com/media/en/technical-documentation/data-sheets/MAX5541.pdf
*/

/*
 * data shifted in on rising clock edge, latches output on DAC_CS high
 * need ~50ns between all pin change states
*/



//SPI.begin();

void initDAC() {
	pinMode(DAC_SDI, OUTPUT);
	pinMode(DAC_SCK, OUTPUT);
	pinMode(DAC_CS, OUTPUT);
	digitalWrite(DAC_SDI, LOW);
	digitalWrite(DAC_SCK, LOW);
	digitalWrite(DAC_CS, LOW);
}

//TODO create a continuous read function that is meant to be called repeatedly
void writeDAC(uint16_t val){	
	//initiate acquisition
	dacdigwr(DAC_CS, LOW); //enables the chip
	
	//read out data
	dacdigwr(DAC_SCK, LOW);
	delayNanoseconds(WRITE_DELAY_NS);
	for(int8_t i = 15; i >= 0; i--){
		dacdigwr(DAC_SDI, (val >> i) & 0x0001);
		dacdigwr(DAC_SCK, HIGH);
		#if WRITE_DELAY_NS
			delayNanoseconds(WRITE_DELAY_NS);
		#endif
		dacdigwr(DAC_SCK, LOW);
		#if WRITE_DELAY_NS
			delayNanoseconds(WRITE_DELAY_NS);
		#endif
	}
	
	dacdigwr(DAC_CS, HIGH);
}


void writeDACVolts(float volt)
{
	//Serial.print(volt);
	//Serial.print(" ");
	//Serial.println(volt*65535/VREF);
	if (volt > DAC_VREF)
		volt = DAC_VREF;
	else if (volt < 0)
		volt = 0;
	writeDAC(volt*65535/DAC_VREF);
}
