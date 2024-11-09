#ifndef PINS
#define PINS

typedef enum{
	REV_V0,
	REV_V2,
	REV_V3
} Board_Rev;

#define BOARD_REV REV_V3

#if (BOARD_REV == REV_V3)
    #define PIN_REFERENCE0 A0
    #define PIN_REFERENCE1 A1
    #define PIN_REFERENCE A2

	#define DAC_CS 21
	#define DAC_SDI 20
	#define DAC_SCK 19

	#define ADC_CNVST 2
	#define ADC_SDO 1
	#define ADC_SCK 0
#elif (BOARD_REV == REV_V2)
    #define PIN_REFERENCE0 24//2
    #define PIN_REFERENCE1 25//3
    #define PIN_REFERENCE A11 //same pin as digital input 0

	#define DAC_CS 41//10
	#define DAC_SDI 40//11
	#define DAC_SCK 39//14

	#define ADC_CNVST 30 //28
	#define ADC_SDO 31 //1
	#define ADC_SCK 32 //27
#endif


#endif