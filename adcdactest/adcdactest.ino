/*
 *mostly stable up to 720 MHz overclock with the v1.0 board, timings may need to be adjusted
*/

#include "adc.h"
#include "dac.h"

#define LED_PIN 13
#define DI0 2
#define DI1 3
#define ANALOG_PIN A10 //TODO not actually connected

#define USE_INTERNAL_ADC false

//https://registry.platformio.org/libraries/ftrias/TeensyThreads may be helpful for simultaneous read and write
//TODO compare to inbuilt ADC (using one of the digital input pins) -- need to grab BNC splitter

void readTest();
void writeTest();
void linearityTest();

double mean(double[], unsigned int);
double stdev(double[], unsigned int);

void setup(){
	Serial.begin(115200);
	Serial.println("starting...");
	initADC();
	initDAC();

    #if USE_INTERNAL_ADC
        pinMode(ANALOG_PIN, INPUT);
        analogReadResolution(12);
        analogReadAveraging(1);
    #endif
	
}

void loop(){
	if(Serial && Serial.available()) {
		if (Serial.read() == 'g'){
			Serial.println("\nLinearity Test:");
			linearityTest();
			Serial.println("\nRead Test:");
			readTest();
			Serial.println("\nMultiple Read Test:");
			readMultipleTest();
			Serial.println("\nWrite Test:");
			writeTest();
		//delay(10000);
		}
		else{
			Serial.println("\nEnter 'g' to run test\n");
		}
	}
}

void readTest(){
	const uint32_t NUM_POINTS = 10000;
	uint16_t values[NUM_POINTS];
	double voltvalues[NUM_POINTS];
	writeDACVolts(1);
    #if USE_INTERNAL_ADC
        analogReadAveraging(1);
    #endif
	elapsedMicros sincestart;
	for(uint16_t i=0; i < NUM_POINTS; i++) {
        #if USE_INTERNAL_ADC
            values[i] = analogRead(ANALOG_PIN);
        #else
    		values[i] = readADC();
        #endif
	}
	float readtime = (float)sincestart/NUM_POINTS;
	for(uint16_t i=0; i < NUM_POINTS; i++) {
        #if USE_INTERNAL_ADC
	    	voltvalues[i] = 3.3*values[i]/4095;
        #else
            voltvalues[i] = ADC_BITS2VOLTS(values[i]);
        #endif
	}
	double value = mean(voltvalues, NUM_POINTS);
	double dev = stdev(voltvalues, NUM_POINTS);
	Serial.printf("Read Time: %f us \t Value: %f V \t Deviation: %f V\n", readtime, value, dev);
}

void readMultipleTest(){
	const uint8_t pointspower = 4;
	const uint16_t points = 0x1 << pointspower;
	const uint32_t NUM_POINTS = 1000;
	uint16_t values[NUM_POINTS];
	double voltvalues[NUM_POINTS];
	writeDACVolts(1);
    #if USE_INTERNAL_ADC
        analogReadAveraging(points);
    #endif
	elapsedMicros sincestart;
	for(uint16_t i=0; i < NUM_POINTS; i++) {
        #if USE_INTERNAL_ADC
            values[i] = analogRead(ANALOG_PIN);
        #else
    		values[i] = readADCMultiple(pointspower);
        #endif
	}
	float readtime = (float)sincestart/NUM_POINTS;
	for(uint16_t i=0; i < NUM_POINTS; i++) {
        #if USE_INTERNAL_ADC
	    	voltvalues[i] = 3.3*values[i]/4095;
        #else
            voltvalues[i] = ADC_BITS2VOLTS(values[i]);
        #endif
	}
	double value = mean(voltvalues, NUM_POINTS);
	double dev = stdev(voltvalues, NUM_POINTS);
	Serial.printf("Points: %i \t Read Time: %f us (%f us/pt) \t Value: %f V \t Deviation: %f V\n", points, readtime, readtime/points, value, dev);
}

void writeTest(){
	const uint32_t NUM_POINTS = 10000;
	elapsedMicros sincestart;
	for(uint16_t i=0; i < NUM_POINTS; i++) {
		writeDACVolts((float)i/NUM_POINTS);
	}
	float writetime = (float)sincestart/NUM_POINTS;
	float value = readADCVolts();
	Serial.printf("Write Time: %f us\tValue: %f V\n", writetime, value);
}

void linearityTest(){
	const uint32_t NUM_POINTS = 100;
	double values[NUM_POINTS];
	double measurements[NUM_POINTS];
	double ratio[NUM_POINTS];
	for(uint32_t i=0; i < NUM_POINTS; i++){
		values[i] = (float)(i+1)*DAC_VREF/NUM_POINTS; //don't write zero
		writeDACVolts(values[i]);
		delay(10); //let it settle
        #if USE_INTERNAL_ADC
            measurements[i] = analogRead(ANALOG_PIN)*3.3/4095;
        #else
    		measurements[i] = readADCVolts();
        #endif
		if (values[i] == 0 && measurements[i] == 0)
			ratio[i] = 1;
		else if (values[i] == 0 && measurements[i] != 0)
			ratio[i] = 0;
		else
			ratio[i] = measurements[i]/values[i];
		Serial.printf("wrote %f \t read %f \t ratio %f\n", values[i], measurements[i], ratio[i]);
	}
	Serial.printf("ratio mean: %f \t ratio stdev: %f\n", mean(ratio, NUM_POINTS), stdev(ratio, NUM_POINTS));
}


double mean(double arr[], unsigned int arr_len){
	double total = 0;
	for(uint32_t i = 0; i < arr_len; i++) {
		total = total + arr[i];
	}
	return total/arr_len;
}

double stdev(double arr[], unsigned int arr_len){
	
	double avg = mean(arr, arr_len);
	double total = 0;
	
	for(uint32_t i = 0; i < arr_len; i++) {
		total += (arr[i] - avg)*(arr[i] - avg);	
	}
	return sqrt(total/(arr_len-1));	
}
