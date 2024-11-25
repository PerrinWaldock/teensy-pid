#include <Arduino.h>
#include "settings.h"

bool isLineAvailable();
char* readLine();

#define checkSerial() {	if(!Serial) \
	{ \
		Serial.begin(SERIAL_BAUD); \
	}}

#define write(s) {checkSerial(); Serial.print(s);}
#define writeLine(s) {checkSerial(); Serial.println(s);}

