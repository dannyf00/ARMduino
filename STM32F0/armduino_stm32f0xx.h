//Arduino port for STM328F0xx family
//software environment: GCC-ARM + CMSIS.h from ARM/ST
//no reliance on SPL or HAL
//
//version: v0.11 @ 7/7/2017
//added support for analogRead(), attachInterrupt() / detachInterrupt();
//added support for TIM1/2/3/14/15/16/17.
//major function not yet supported: analogWrite()

//version: v0.10 @ 7/7/2017
//initial release, support for STM32F chips
//
//Supported functions:
//GPIO: pinMode(), digitalWrite(), digitalRead()
//Time: millis(), micros(), delay(), delayMicroseconds()
//Math: min(), max(), abs(), constrain(), map(), pow(), sqrt()
//Trigonometry: sin(), cos(), tan()
//Characters: isAlphaNumeric(), isAlpha(), isAscii(), isWhitespace(), isControl(), isDigit(), isGraph(), isLowerCase(), isPrintable, isPunct(), isSpace(), isUpperCase(), isHexadecimalDigit()
//Random Numbers: randomSeed(), random(max). random(min, max) ported to random2(min, max)
//Bits and Bytes: lowByte(), highByte(), bitRead(), bitWrite(), bitSet(), bitClear(), bit()
//Analog IO: analogRead(), /*analogWrite(), analogReference()*/
//Advanced IO: /*tone(), noTone(), */shiftOut(), shiftIn(), pulseIn()
//External Interrupts: attachInterrupt(), detachInterrupt()
//serial: serialBegin(), serialWrite(), serialRead(), serialPrint(), serialPrintln(), serialAvailable(), and serialBusy()
//
//STM32F0duino extensions:
//serial1: serial1Begin(), serial1Write(), serial1Read(), serial1Print(), serial1Println(), serial1Available(), serial1Busy()
//serial2: serial2Begin(), serial2Write(), serial2Read(), serial2Print(), serial2Println(), serial2Available(), serial2Busy()
//Timers: periodic timers
//TIM1: tim1_init(), tim1_setpr1()/tim1_act1(),tim1_setpr2()/tim1_act2(), tim1_setpr3()/tim1_act3(), tim1_setpr4()/tim1_act4()
//TIM2: tim2_init(), tim2_setpr1()/tim2_act1(),tim2_setpr2()/tim2_act2(), tim2_setpr3()/tim2_act3(), tim2_setpr4()/tim2_act4()
//TIM1: tim3_init(), tim3_setpr1()/tim3_act1(),tim3_setpr2()/tim3_act2(), tim3_setpr3()/tim3_act3(), tim3_setpr4()/tim3_act4()
//TIM14:tim14_init(), tim14_setpr()/tim14_act();
//TIM15:tim15_init(), tim15_setpr()/tim15_act();
//TIM16:tim16_init(), tim16_setpr()/tim16_act();
//TIM17:tim17_init(), tim17_setpr()/tim17_act();

#ifndef _ARMDUINO_STM32F0XX_H
#define _ARMDUINO_STM32F0XX_H

#include "stm32f0xx.h"				//we use ST's STM32f10x chips

//STM32F0duino module configuration
//GPIO is always used
//#define USE_PWM1							//comment out if not used - tmr1 pwm
//#define USE_PWM2							//comment out if not used - tmr2 pwm
//#define USE_PWM3							//comment out if not used - tmr3 pwm
//#define USE_PWM4							//comment out if not used - tmr4 pwm
//#define USE_ADC1							//comment out if not used
//#define USE_SPI1							//comment out if not used
//#define USE_SPI2							//comment out if not used
//#define USE_I2C1							//comment out if not used
//#define USE_I2C2							//comment out if not used
//#define USE_UART1							//comment out if not used - present on f030
//#define USE_UART2							//comment out if not used - not present on all chips
//#define USE_UART3							//comment out if not used - not present on all chips
//#define USE_TIM1							//use tim1 output compare channels as virtual timers
//#define USE_TIM2							//use tim2 output compare channels as virtual timers, not present on all chips
//#define USE_TIM3							//use tim3 output compare channels as virtual timers
//#define USE_TIM14							//use tim14 overflow
//#define USE_TIM15							//use tim15 overflow, not present on all chips
#define USE_TIM16							//use tim16 overflow
#define USE_TIM17							//use tim17 overflow
//end STM32F0duino module configuration

//STM32duino hardware configuration
#define F_UART			F_CPU				//uart frequency = cpu frequency
#define PWMOUT_BITs		16					//in bits. PWM output / analogWrite() resolution. 8-14 suggested -> default = 12bits
#define PWMOUT_PS		1					//16-bit prescaler for pwm timebase, [1..65536]
#define ADC_SMPR		7					//ADC sample time, 0b000(1.5us)..0b111(239.5us), applied to all channels
#define SPI1_PS			7					//3-bit SPI1 prescaler, [0..7], 0->2x, 7->256x
#define SPI2_PS			7					//3-bit SPI1 prescaler, [0..7], 0->2x, 7->256x
#define SPI3_PS			7					//3-bit SPI1 prescaler, [0..7], 0->2x, 7->256x
//end STM32duino hardware configuration

//global defines

//gpio enums
//needs to match GPIO_PinDef[]
typedef enum {
	PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9, PA10, PA11, PA12, PA13, PA14, PA15,			//GPIOA pin defs
	PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15,			//GPIOA pin defs
	PC0, PC1, PC2, PC3, PC4, PC5, PC6, PC7, PC8, PC9, PC10, PC11, PC12, PC13, PC14, PC15,			//GPIOA pin defs
	PD0, PD1, PD2, PD3, PD4, PD5, PD6, PD7, PD8, PD9, PD10, PD11, PD12, PD13, PD14, PD15,			//GPIOA pin defs
	PE0, PE1, PE2, PE3, PE4, PE5, PE6, PE7, PE8, PE9, PE10, PE11, PE12, PE13, PE14, PE15,			//GPIOA pin defs
#if defined(RCC_AHBENR_GPIOFEN)
	PF0, PF1, PF2, PF3, PF4, PF5, PF6, PF7, PF8, PF9, PF10, PF11, PF12, PF13, PF14, PF15,			//GPIOA pin defs
#endif
#if defined(RCC_AHBENR_GPIOGEN)
	PG0, PG1, PG2, PG3, PG4, PG5, PG6, PG7, PG8, PG9, PG10, PG11, PG12, PG13, PG14, PG15,			//GPIOA pin defs
#endif
} PIN_TypeDef;

typedef enum {
	A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15,
	AVREFINT,									//AIN16 = Vrefint
	ATEMP,										//AIN17 = temperature sensor
} AIN_TypeDef;

//convert temperature sensor adc reading into temperature x100
//follow the datasheet. 3.3v Vref (3.0v for my board), 12bit adc
#define Tx10(adc)		(14300/43*10+250 - ((33000ul*10/43 * (adc)) >> 12))

//MODE:
//Output modes: GPIOMODE_OUTPP, GPIOMODE_OUTOD
//Input mode: GPIOMODE_INPUT, GPIOMODE_INPUTPD, GPIOMODE_INPUTPU
//Alternate function: GPIOMODE_AFPP, GPIOMODE_AFOD
//for moder
#define GPIOMODE_INPUT		(0<<0)		//(0<<0)
#define GPIOMODE_OUTPUT		(1<<0)		//(1<<0)
#define GPIOMODE_AF			(2<<0)
#define GPIOMODE_AN			(3<<0)
//for otyper
#define GPIOMODE_PP			(1<<3)
#define GPIOMODE_OD			(0<<3)
//for pupdr
#define GPIOMODE_FL			(0<<4)
#define GPIOMODE_PU			(1<<4)
#define GPIOMODE_PD			(2<<4)

//AF defs
#define GPIOMODE_AF0		0
#define GPIOMODE_AF1		1
#define GPIOMODE_AF2		2
#define GPIOMODE_AF3		3
#define GPIOMODE_AF4		4
#define GPIOMODE_AF5		5
#define GPIOMODE_AF6		6
#define GPIOMODE_AF7		7

#define GPIOMODE_OUTPP		(GPIOMODE_OUTPUT | GPIOMODE_PP)		//gpio, output, push-pull
#define GPIOMODE_OUTOD		(GPIOMODE_OUTPUT | GPIOMODE_OD)		//gpio, output, od
#define GPIOMODE_INFL		(GPIOMODE_INPUT)
#define GPIOMODE_INPU		(GPIOMODE_INPUT | GPIOMODE_PU)
#define GPIOMODE_INPD		(GPIOMODE_INPUT | GPIOMODE_PD)
#define GPIOMODE_AFPP		(GPIOMODE_AF | GPIOMODE_PP)
#define GPIOMODE_AFOD		(GPIOMODE_AF | GPIOMODE_OD)
#define GPIOMODE_INAN		(GPIOMODE_AN)

//device-specific extension of GPIO mode
#define INPUT_PULLDN		(INPUT_PULLUP + 1)					//gpio input, with pull-down
#define INPUT_ANALOG		(INPUT_PULLUP + 2)					//analog input
#define OUTPUT_OD			(INPUT_PULLUP + 3)					//gpio output, open-drain
#define OUTPUT_AFPP			(INPUT_PULLUP + 4)					//alternative function, pushpull output
#define OUTPUT_AFOD			(INPUT_PULLUP + 5)					//alternative function, pushpull output

//exti / attachInterrupt() mode
//#define EDGE_FALLING		0x01			//falling edge for EXTI
//#define EDGE_RISING		0x02			//rising edge for EXTI
//#define EDGE_CHANGE		(EDGE_FALLING | EDGE_RISING)


//global variables

//port/gpio oriented macros
#define IO_SET(port, pins)					port->ODR |= (pins)				//set bits on port
#define IO_CLR(port, pins)					port->ODR &=~(pins)				//clear bits on port
#define IO_FLP(port, pins)					port->ODR ^= (pins)				//flip bits on port
#define IO_GET(port, pins)					((port->IDR) & (pins))			//return bits on port
//set a pin to output/input
#define IO_OUTPP(port, pins)				GPIO_DDR(port, pins, GPIOMODE_OUTPP)	//push-pull mode (CR1 set, CR2 cleared)	//IO_OUTPP(GPIOx, GPIO_Pins).
#define IO_OUTOD(port, pins)				GPIO_DDR(port, pins, GPIOMODE_OUTOD)	//open drain mode (cr1 + cr2 cleared)	//_GPIO_Init(GPIOx, GPIO_Pins, GPIO_MODE_OUT_OD_LOW_FAST)
#define IO_OUT(port, pins)					IO_OUTPP(port, pins)					//_GPIO_Init(GPIOx, GPIO_Pins, GPIO_MODE_OUT_PP_LOW_FAST)
#define IO_INFL(port, pins)					GPIO_DDR(port, pins, GPIOMODE_INFL)		//floating input, no interrupt			//IO_INFL(GPIOx, GPIO_Pins)
#define IO_INPU(port, pins)					GPIO_DDR(port, pins, GPIOMODE_INPU)		//pull-up, no interrupt					//_GPIO_Init(GPIOx, GPIO_Pins, GPIO_MODE_IN_PU_NO_IT)
#define IO_INPD(port, pins)					GPIO_DDR(port, pins, GPIOMODE_INPD)		//pull-up, no interrupt					//_GPIO_Init(GPIOx, GPIO_Pins, GPIO_MODE_IN_PU_NO_IT)
#define IO_IN(port, pins)					IO_INFL(port, pins)					//IO_IN(port, pins)				//_GPIO_Init(GPIOx, GPIO_Pins, GPIO_MODE_IN_FL_NO_IT)
#define IO_AFPP(port, pins)					GPIO_DDR(port, pins, GPIOMODE_AFPP)		//configure pin for alternative function output, push-pull, 10Mhz
#define IO_AFOD(port, pins)					GPIO_DDR(port, pins, GPIOMODE_AFOD)		//configure pin for alternative function output, open-drain, 10Mhz
#define IO_AN(port, pins)					GPIO_DDR(port, pins, GPIOMODE_INAN)

//fast routines through BRR/BSRR registers
#define FIO_SET(port, pins)					port->BSRR = (pins)
#define FIO_CLR(port, pins)					port->BRR = (pins)
#define FIO_FLP(port, pins)					IO_FLP(port, pins)
#define FIO_GET(port, pins)					IO_GET(port, pins)

//configure gpio mode (cnf1..0 + mod1..0 bits)
void GPIO_DDR(GPIO_TypeDef * gpio, uint32_t mask, uint32_t mode);

//extensions to arduino
//hardware dependent routines
//spi
uint8_t SPI1Write(uint8_t order, uint8_t dat);
uint8_t SPI1Read(uint8_t order);
uint8_t SPI2Write(uint8_t order, uint8_t dat);
uint8_t SPI2Read(uint8_t order);

//return timebase ticks
uint32_t ticks(void);

//usart routines
//uart1
void serial1Begin(uint32_t baudrate);
void serial1Print(char *str);
void serial1Println(char *str);
void serial1Write(char ch);
unsigned char serial1Read(void);
uint8_t serial1Available(void);
uint8_t serial1Busy(void);

//uart2
void serial2Begin(uint32_t baudrate);
void serial2Print(char *str);
void serial2Println(char *str);
void serial2Write(char ch);
unsigned char serial2Read(void);
uint8_t serial2Available(void);
uint8_t serial2Busy(void);

//uart3
void serial3Begin(uint32_t baudrate);
void serial3Print(char *str);
void serial3Println(char *str);
void serial3Write(char ch);
unsigned char serial3Read(void);
uint8_t serial3Available(void);
uint8_t serial3Busy(void);

//timer related routines
void tim1_init(uint32_t ps);
void tim1_setpr1(uint32_t pr); void tim1_act1(void (*isr_ptr)(void));
void tim1_setpr2(uint32_t pr); void tim1_act2(void (*isr_ptr)(void));
void tim1_setpr3(uint32_t pr); void tim1_act3(void (*isr_ptr)(void));
void tim1_setpr4(uint32_t pr); void tim1_act4(void (*isr_ptr)(void));
//tim2 not present on all chips
void tim2_init(uint32_t ps);
void tim2_setpr1(uint32_t pr); void tim2_act1(void (*isr_ptr)(void));
void tim2_setpr2(uint32_t pr); void tim2_act2(void (*isr_ptr)(void));
void tim2_setpr3(uint32_t pr); void tim2_act3(void (*isr_ptr)(void));
void tim2_setpr4(uint32_t pr); void tim2_act4(void (*isr_ptr)(void));

void tim3_init(uint32_t ps);
void tim3_setpr1(uint32_t pr); void tim3_act1(void (*isr_ptr)(void));
void tim3_setpr2(uint32_t pr); void tim3_act2(void (*isr_ptr)(void));
void tim3_setpr3(uint32_t pr); void tim3_act3(void (*isr_ptr)(void));
void tim3_setpr4(uint32_t pr); void tim3_act4(void (*isr_ptr)(void));

void tim14_init(uint32_t ps);
void tim14_setpr(uint32_t pr); void tim14_act(void (*isr_ptr)(void));

//tim15 not present on some variants
void tim15_init(uint32_t ps);
void tim15_setpr(uint32_t pr); void tim15_act(void (*isr_ptr)(void));

void tim16_init(uint32_t ps);
void tim16_setpr(uint32_t pr); void tim16_act(void (*isr_ptr)(void));

void tim17_init(uint32_t ps);
void tim17_setpr(uint32_t pr); void tim17_act(void (*isr_ptr)(void));

//initialize the chip
void chip_init(void);
#endif
