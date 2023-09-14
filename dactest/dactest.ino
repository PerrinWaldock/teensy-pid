//#include <SPI.h>  // include the SPI library:

#define CS 10
#define SDO 11
#define SCK 14

#define dw(p, s) digitalWrite(p, s)
#define dr(p) digitalReadFast(p)

#define VREF 2.75

/*
 *https://www.analog.com/media/en/technical-documentation/data-sheets/MAX5541.pdf
*/

/*
 * data shifted in on rising clock edge, latches output on CS high
 * need ~50ns between all pin change states
*/

void writeDAC(uint16_t val);
void writeDACVolts(float volt);


//SPI.begin();

void setup() {
	pinMode(SDO, OUTPUT);
	pinMode(SCK, OUTPUT);
	pinMode(CS, OUTPUT);
	digitalWrite(SDO, LOW);
	digitalWrite(SCK, LOW);
	digitalWrite(CS, LOW);
	Serial.begin(115200);
	Serial.println("starting...");
}


void loop() {
	const int MAXVALUE = 30000;
	/*
	for (int i = 0; i < MAXVALUE; i++){
		writeDAC(i);
		delayMicroseconds(100);
	}*/
	//writeDAC(65335);
	//writeDAC(10000);
	writeDACVolts(2);
	//Serial.println("test");
	delayMicroseconds(1000);
}

//TODO create a continuous read function that is meant to be called repeatedly
void writeDAC(uint16_t val){
	
	Serial.println(val);
	
	//initiate acquisition
	dw(CS, LOW); //enables the chip
	
	//read out data
	dw(SCK, LOW);
	delayNanoseconds(50);
	for(int8_t i = 15; i >= 0; i--){
		dw(SDO, (val >> i) & 0x0001);
		dw(SCK, HIGH);
		delayNanoseconds(50);
		dw(SCK, LOW);
		delayNanoseconds(50);
		//TODO may need to adjust timing here
	}
	
	
	dw(CS, HIGH);
	
}


void writeDACVolts(float volt)
{
	//Serial.print(volt);
	//Serial.print(" ");
	//Serial.println(volt*65535/VREF);
	if (volt > VREF)
		volt = VREF;
	else if (volt < 0)
		volt = 0;
	writeDAC(volt*65535/VREF);
}
