//hardware specific port for ARMduino to msp432 family

#include "armduino_msp432.h"						//we use aruidno GPIO port for msp432
#include "armduino.h"								//use ARMduino specific definitions

//global definitions
//struct used to map a pin to GPIO+mask
typedef struct {
	volatile GPIO_TypeDef *gpio;					//gpio for a pin
	uint16_t mask;									//pin mask - only 16-bit formated is supported in this port
} PIN2GPIO;

//global variables
//define your chips here
//declare pins based on chip packaging
//ALL PINS ARE MAPPED, WHETHER THEY EXIST OR NOT
//*********************************************************
//SO MAKE SURE THAT THE PINS YOU PICKED ACTUALLY EXIST FOR YOUR CHIP/PACKAGE
//*********************************************************
//Pin  0..15 -> GPIOA
//Pin 16..31 -> GPIOB
//Pin 32..47 -> GPIOC
//Pin 48..63 -> GPIOD
//Pin 64..79 -> GPIOE
//Pin 80..95 -> GPIOJ
const PIN2GPIO GPIO_PinDef[]={
		{GPIOA, 1<< 0},						//ARMduino Pin  0 = PA0
		{GPIOA, 1<< 1},						//ARMduino Pin  1 = PA1
		{GPIOA, 1<< 2}, 					//ARMduino Pin  2 = PA2
		{GPIOA, 1<< 3},						//ARMduino Pin  3 = PA3
		{GPIOA, 1<< 4},						//ARMduino Pin  4 = PA4
		{GPIOA, 1<< 5},						//ARMduino Pin  5 = PA5
		{GPIOA, 1<< 6},						//ARMduino Pin  6 = PA6
		{GPIOA, 1<< 7},						//ARMduino Pin  7 = PA7
		{GPIOA, 1<< 8},						//ARMduino Pin  8 = PA8
		{GPIOA, 1<< 9},						//ARMduino Pin  9 = PA9
		{GPIOA, 1<<10},						//ARMduino Pin 10 = PA10
		{GPIOA, 1<<11},						//ARMduino Pin 11 = PA11
		{GPIOA, 1<<12},						//ARMduino Pin 12 = PA12
		{GPIOA, 1<<13},						//ARMduino Pin 13 = PA13
		{GPIOA, 1<<14},						//ARMduino Pin 14 = PA14
		{GPIOA, 1<<15},						//ARMduino Pin 15 = PA15

		{GPIOB, 1<< 0},						//ARMduino Pin 16 = PB0
		{GPIOB, 1<< 1},						//ARMduino Pin 17 = PB1
		{GPIOB, 1<< 2},						//ARMduino Pin 18 = PB2
		{GPIOB, 1<< 3},						//ARMduino Pin 19 = PB3
		{GPIOB, 1<< 4},						//ARMduino Pin 20 = PB4
		{GPIOB, 1<< 5},						//ARMduino Pin 21 = PB5
		{GPIOB, 1<< 6},						//ARMduino Pin 22 = PB6
		{GPIOB, 1<< 7},						//ARMduino Pin 23 = PB7
		{GPIOB, 1<< 8},						//ARMduino Pin 24 = PB9
		{GPIOB, 1<< 9},						//ARMduino Pin 25 = PB10
		{GPIOB, 1<<10},						//ARMduino Pin 26 = PB11
		{GPIOB, 1<<11},						//ARMduino Pin 27 = PB12
		{GPIOB, 1<<12},						//ARMduino Pin 28 = PB13
		{GPIOB, 1<<13},						//ARMduino Pin 29 = PB14
		{GPIOB, 1<<14},						//ARMduino Pin 30 = PB15
		{GPIOB, 1<<15},						//ARMduino Pin 31 = PB16

		{GPIOC, 1<< 0},						//ARMduino Pin 32 = PC0
		{GPIOC, 1<< 1},						//ARMduino Pin 33 = PC1
		{GPIOC, 1<< 2},						//ARMduino Pin 34 = PC2
		{GPIOC, 1<< 3},						//ARMduino Pin 35 = PC3
		{GPIOC, 1<< 4},						//ARMduino Pin 36 = PC4
		{GPIOC, 1<< 5},						//ARMduino Pin 37 = PC5
		{GPIOC, 1<< 6},						//ARMduino Pin 38 = PC6
		{GPIOC, 1<< 7},						//ARMduino Pin 39 = PC7
		{GPIOC, 1<< 8},						//ARMduino Pin 40 = PC8
		{GPIOC, 1<< 9},						//ARMduino Pin 41 = PC9
		{GPIOC, 1<<10},						//ARMduino Pin 42 = PC10
		{GPIOC, 1<<11},						//ARMduino Pin 43 = PC11
		{GPIOC, 1<<12},						//ARMduino Pin 44 = PC12
		{GPIOC, 1<<13},						//ARMduino Pin 45 = PC13
		{GPIOC, 1<<14},						//ARMduino Pin 46 = PC14
		{GPIOC, 1<<15},						//ARMduino Pin 47 = PC15

		{GPIOD, 1<< 0},						//ARMduino Pin 48 = PD0
		{GPIOD, 1<< 1},						//ARMduino Pin 49 = PD1
		{GPIOD, 1<< 2},						//ARMduino Pin 50 = PD2
		{GPIOD, 1<< 3},						//ARMduino Pin 51 = PD3
		{GPIOD, 1<< 4},						//ARMduino Pin 52 = PD4
		{GPIOD, 1<< 5},						//ARMduino Pin 53 = PD5
		{GPIOD, 1<< 6},						//ARMduino Pin 54 = PD6
		{GPIOD, 1<< 7},						//ARMduino Pin 55 = PD7
		{GPIOD, 1<< 8},						//ARMduino Pin 56 = PD8
		{GPIOD, 1<< 9},						//ARMduino Pin 57 = PD9
		{GPIOD, 1<<10},						//ARMduino Pin 58 = PD10
		{GPIOD, 1<<11},						//ARMduino Pin 59 = PD11
		{GPIOD, 1<<12},						//ARMduino Pin 60 = PD12
		{GPIOD, 1<<13},						//ARMduino Pin 61 = PD13
		{GPIOD, 1<<14},						//ARMduino Pin 62 = PD14
		{GPIOD, 1<<15},						//ARMduino Pin 63 = PD15

		{GPIOE, 1<< 0},						//ARMduino Pin 64 = PE0
		{GPIOE, 1<< 1},						//ARMduino Pin 65 = PE1
		{GPIOE, 1<< 2},						//ARMduino Pin 66 = PE2
		{GPIOE, 1<< 3},						//ARMduino Pin 67 = PE3
		{GPIOE, 1<< 4},						//ARMduino Pin 68 = PE4
		{GPIOE, 1<< 5},						//ARMduino Pin 69 = PE5
		{GPIOE, 1<< 6},						//ARMduino Pin 71 = PE6
		{GPIOE, 1<< 7},						//ARMduino Pin 72 = PE7
		{GPIOE, 1<< 8},						//ARMduino Pin 73 = PE8
		{GPIOE, 1<< 9},						//ARMduino Pin 74 = PE9
		{GPIOE, 1<<10},						//ARMduino Pin 75 = PE10
		{GPIOE, 1<<11},						//ARMduino Pin 76 = PE11
		{GPIOE, 1<<12},						//ARMduino Pin 77 = PE12
		{GPIOE, 1<<13},						//ARMduino Pin 78 = PE13
		{GPIOE, 1<<14},						//ARMduino Pin 79 = PE14
		{GPIOE, 1<<15},						//ARMduino Pin 80 = PE15

		{GPIOJ, 1<< 0},						//ARMduino Pin 81 = PF0
		{GPIOJ, 1<< 1},						//ARMduino Pin 82 = PF1
		{GPIOJ, 1<< 2},						//ARMduino Pin 83 = PF2
		{GPIOJ, 1<< 3},						//ARMduino Pin 84 = PF3
		{GPIOJ, 1<< 4},						//ARMduino Pin 85 = PF4
		{GPIOJ, 1<< 5},						//ARMduino Pin 86 = PF5
		{GPIOJ, 1<< 6},						//ARMduino Pin 87 = PF6
		{GPIOJ, 1<< 7},						//ARMduino Pin 88 = PF7
		{GPIOJ, 1<< 8},						//ARMduino Pin 89 = PF8
		{GPIOJ, 1<< 9},						//ARMduino Pin 90 = PF9
		{GPIOJ, 1<<10},						//ARMduino Pin 91 = PF10
		{GPIOJ, 1<<11},						//ARMduino Pin 92 = PF11
		{GPIOJ, 1<<12},						//ARMduino Pin 93 = PF12
		{GPIOJ, 1<<13},						//ARMduino Pin 94 = PF13
		{GPIOJ, 1<<14},						//ARMduino Pin 95 = PF14
		{GPIOJ, 1<<15},						//ARMduino Pin 96 = PF15
};

//Arduino Functions: GPIO
//set a pin mode to INPUT or OUTPUT
//no error checking on PIN
inline void pinMode(PIN_TypeDef pin, uint8_t mode) {
	if (mode==INPUT) GIO_IN(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);
	else GIO_OUT(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);
}

//set / clear a pin
inline void digitalWrite(PIN_TypeDef pin, uint8_t val) {
	if (val==LOW) GIO_CLR(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);
	else GIO_SET(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);
}

//read a pin
inline int digitalRead(PIN_TypeDef pin) {
	return (GIO_GET(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask))?HIGH:LOW;
}
//end GPIO

//initialize the chip
void chip_init(void) {
	WDTCTL = WDTPW | WDTHOLD;           // Stop watchdog timer

	//select the clock source
	//default GPIO clock turned on

	//enable clock to GPIO
	//or use default clock

	//update SystemCoreClock - done in mcu_init()
	//SystemCoreClockUpdate();

}


