//ARMduino port for ARM family of Cortex-M chips
//software environment: gcc-arm
//
//version: v0.11a @6/4/2017
//fixed a minor error in systick initialization
//
//version: v0.11 @ 5/27/2017
//minor improvements
//version: v0.10 @ 5/26/2017
//initial release, support for ARM Cortex-M chips
//Supported functions:
//GPIO: pinMode(), digitalWrite(), digitalRead(), via chip-specific files
//Time: millis(), micros(), delay(), delaymicroseconds()
//Math: min(), max(), abs(), constrain(), map(), pow(), sqrt()
//Trigonometry: sin(), cos(), tan()
//Characters: isAlphaNumeric(), isAlpha(), isAscii(), isWhitespace(), isControl(), isDigit(), isGraph(), isLowerCase(), isPrintable, isPunct(), isSpace(), isUpperCase(), isHexadecimalDigit()
//Random Numbers: randomSeed(), random(max). random(min, max) ported to random2(min, max)
//Bits and Bytes: lowByte(), highByte(), bitRead(), bitWrite(), bitSet(), bitClear(), bit()


#ifndef _ARMduino_H
#define _ARMduino_H

#include <stdlib.h>							//we use rand()
#include <ctype.h>							//we use char-functions
//include chip-specific files here
#include "armduino_stm32f10x.h"				//we use STM32F10x chips

//global defines

//global variables

#define NOP()         			__NOP()						//__no_operation()			//asm("nop")            		//nop
#define NOP2()					{NOP(); NOP();}
#define NOP4()					{NOP2(); NOP2();}
#define NOP8()					{NOP4(); NOP4();}
#define NOP16()					{NOP8(); NOP8();}
#define NOP16()					{NOP8(); NOP8();}
#define NOP24()					{NOP16(); NOP8();}
#define NOP32()					{NOP16(); NOP16();}
#define NOP40()					{NOP32(); NOP8();}
#define NOP64()					{NOP32(); NOP32();}

//interrupts always enabled and never disabled
#ifndef ei
	#define ei()							//asm("rim")					//enable all interrupts
#endif

#ifndef di
	#define di()							//asm("sim")					//disable all interrupts
#endif

#define F_CPU					(SystemCoreClock)							//default fcpu, SystemCoreClock -> more code space (+200 bytes)

//arduino-specific defs
#define INPUT				0									//gpio input, floating
#define OUTPUT				1									//gpio output, pushpull
#define INPUT_PULLUP		2									//input with pull-up
//additional defs in device specific file

#define LOW					0
#define HIGH				1									//(!LOW)

#define PI 					3.1415926535897932384626433832795
#define HALF_PI 			(PI / 2)							//1.5707963267948966192313216916398
#define TWO_PI 				(PI + PI)							//6.283185307179586476925286766559
#define DEG_TO_RAD 			(TWO_PI / 360)						//0.017453292519943295769236907684886
#define RAD_TO_DEG 			(360 / TWO_PI)						//57.295779513082320876798154814105
#define EULER 				2.718281828459045235360287471352	//Euler's number

#define SERIAL  			0x0
#define DISPLAY 			0x1

#define LSBFIRST 			0
#define MSBFIRST 			1									//(!LSBFIRST)							//1

#define CHANGE 				0x01
#define FALLING 			0x02
#define RISING 				0x03

#define min(a,b) 			((a)<(b)?(a):(b))
#define max(a,b) 			((a)>(b)?(a):(b))
#define abs(x) 				((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     		((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) 		((deg)*DEG_TO_RAD)
#define degrees(rad) 		((rad)*RAD_TO_DEG)
#define sq(x) 				((x)*(x))

#define interrupts() 		ei()
#define noInterrupts() 		di()

#define clockCyclesPerMicrosecond() 	( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) 	( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) 	( (a) * clockCyclesPerMicrosecond() )

#define lowByte(w) 			((uint8_t) ((w) & 0xff))
#define highByte(w) 		((uint8_t) ((w) >> 8))

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) 	((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))
#define bit(n)				(1ul<<(n))

#define false				0
#define true				(!false)

//characters
#define isAlphaNumeric(c)	isalnum(c)
#define isAlpha(c)			isalpha(c)
#define isAscii(c)			isascii(c)
#define isWhitespace(c)		isblank(c)
#define isControl(c)		iscntrl(c)
#define isDigit(c)			isdigit(c)
#define isGraph(c)			isgraph(c)
#define isLowerCase(c)		islower(c)
#define isPrintable(c)		isprint(c)
#define isPunct(c)			ispunct(c)
#define isSpace(c)			isspace(c)
#define isUpperCase(c)		isupper(c)
#define isHexadecimalDigit(c)	isxdigit(c)

//random number
#define randomSeed(seed)	srand(seed)
#define random(max)			random2(0, max)
#define random2(min, max)	((min) + (int32_t) ((max) - (min)) * rand() / 32768)

//random() will need manual porting

//external setup/loop - defined by user
extern void setup(void);
extern void loop(void);

//time base
uint32_t millis(void);
uint32_t micros(void);
void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);

//advanced io
//shiftin/out: bitOrder = MSBFIRST or LSBFIRST
uint8_t shiftIn(PIN_TypeDef dataPin, PIN_TypeDef clockPin, uint8_t bitOrder);
void shiftOut(PIN_TypeDef dataPin, PIN_TypeDef clockPin, uint8_t bitOrder, uint8_t val);
//wait for a pulse and return timing
uint32_t pulseIn(PIN_TypeDef pin, uint8_t state);

//================chip-specific functions, user-implemented for target chip
//gpio functions - chip-specific
inline void pinMode(PIN_TypeDef pin, uint8_t mode);
inline void digitalWrite(PIN_TypeDef pin, uint8_t mode);
inline int digitalRead(PIN_TypeDef pin);

//analogWrite()/pwm output - chip-specific
void analogWrite(PIN_TypeDef pin, uint16_t dc);

//analogRead() - chip-specific
uint16_t analogRead(AIN_TypeDef ain);

//Serial IO

//Advanced IO
//tone()
//noTone()

//External Interrupts
void attachInterrupt(PIN_TypeDef pin, void (*isrptr)(void), uint8_t mode);
void detachInterrupt(PIN_TypeDef pin);

//================end chip-specific functions
#endif
