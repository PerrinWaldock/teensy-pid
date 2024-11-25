#include <Arduino.h>
#include "serialmanager.h"
#include "settings.h"

// todo turn into a class, pass anything that implements the serial parent class

#define INPUT_BUFFER_LENGTH 256

uint16_t lastLinePosition = 0;
uint16_t bufferPosition = 0;
char inputBuffer[INPUT_BUFFER_LENGTH];
char outputBuffer[INPUT_BUFFER_LENGTH];
bool lineAvailable = false;

#define isEndLineChar(c) (c == '\r' || c == '\n')

bool isLineAvailable()
{
	if(!Serial)
	{
		Serial.begin(SERIAL_BAUD);
	}

	while (!lineAvailable && Serial.available() > 0)
	{
		if (bufferPosition >= INPUT_BUFFER_LENGTH)
		{
			bufferPosition = 0;
		}

		char c = Serial.read();
		if (isEndLineChar(c))
		{
			inputBuffer[bufferPosition] = '\0';
			lineAvailable = true;
		}
		else
		{
			inputBuffer[bufferPosition] = c;
		}
		bufferPosition++;
	}

	return lineAvailable;
}


char* readLine()
{
	while (!isLineAvailable());

	uint16_t charactersToCopy = (bufferPosition + INPUT_BUFFER_LENGTH - lastLinePosition)%INPUT_BUFFER_LENGTH;
	for (uint16_t i = 0; i < charactersToCopy; i++)
	{
		outputBuffer[i] = inputBuffer[(i + lastLinePosition)%INPUT_BUFFER_LENGTH];
	}
	
	lastLinePosition = (lastLinePosition + charactersToCopy)%INPUT_BUFFER_LENGTH;

	return outputBuffer;
}