//Arduino GPIO port for LPC12xx family
//software environment: GCC-ARM + CMSIS.h from ARM/ST
//to-do:
//1. serial comms
//2. analogRead() on pins, not ain
//3. clock management
//
//version: v0.10a
//analogWrite() implemented @ 5/29/2017: upto 13-ch PWM, user-specified PWM frequency
//12-bit DAC1Write() and DAC2Write() implemented 5/30/2017
//12-bit analogRead() implemented 5/30/2017
//
//version: v0.10
//initial release, support for STM32F chips
//Supported functions:
//GPIO: pinMode(), digitalWrite(), digitalRead()


#ifndef _ARMDUINO_LPC12XX_H
#define _ARMDUINO_LPC12XX_H

#include "lpc122x.h"				//we use nxp lpc122x chips

//STM32duino module configuration
//GPIO is always used
//none of the peripherals is enabled
//#define USE_PWM1							//comment out if not used - tmr1 pwm
//#define USE_PWM2							//comment out if not used - tmr2 pwm
//#define USE_PWM3							//comment out if not used - tmr3 pwm
//#define USE_PWM4							//comment out if not used - tmr4 pwm
//#define USE_EXTI							//comment out if not used
//#define USE_ADC1							//comment out if not used
//#define USE_ADC2							//comment out if not used - not present on all chips
//#define USE_DAC								//comment out if not used
//#define USE_SPI1							//comment out if not used
//#define USE_SPI2							//comment out if not used
//#define USE_I2C1							//comment out if not used
//#define USE_I2C2							//comment out if not used
//#define USE_UART1							//comment out if not used - present on f100xx
//#define USE_UART2							//comment out if not used - present on f100xx
//#define USE_UART3							//comment out if not used - present on f100xx
//end STM8Sduino module configuration

//STM32duino hardware configuration
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
	P0_0, P0_1, P0_2, P0_3, P0_4, P0_5, P0_6, P0_7, P0_8, P0_9, P0_10, P0_11, P0_12, P0_13, P0_14, P0_15, P0_16, P0_17, P0_18, P0_19, P0_20, P0_21, P0_22, P0_23, P0_24, P0_25, P0_26, P0_27, P0_28, P0_29, P0_30, P0_31, 			//P0 pin defs
	P1_0, P1_1, P1_2, P1_3, P1_4, P1_5, P1_6, P1_7, P1_8, P1_9, P1_10, P1_11, P1_12, P1_13, P1_14, P1_15, P1_16, P1_17, P1_18, P1_19, P1_20, P1_21, P1_22, P1_23, P1_24, P1_25, P1_26, P1_27, P1_28, P1_29, P1_30, P1_31, 			//P0 pin defs
	P2_0, P2_1, P2_2, P2_3, P2_4, P2_5, P2_6, P2_7, P2_8, P2_9, P2_10, P2_11, P2_12, P2_13, P2_14, P2_15, P2_16, P2_17, P2_18, P2_19, P2_20, P2_21, P2_22, P2_23, P2_24, P2_25, P2_26, P2_27, P2_28, P2_29, P2_30, P2_31, 			//P0 pin defs
#if defined(LPC_GPIO3)
	P3_0, P3_1, P3_2, P3_3, P3_4, P3_5, P3_6, P3_7, P3_8, P3_9, P3_10, P3_11, P3_12, P3_13, P3_14, P3_15, P3_16, P3_17, P3_18, P3_19, P3_20, P3_21, P3_22, P3_23, P3_24, P3_25, P3_26, P3_27, P3_28, P3_29, P3_30, P3_31, 			//P0 pin defs
#endif
} PIN_TypeDef;

typedef enum {
	A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15,
	AVREFINT,									//AIN16 = Vrefint
	ATEMP,										//AIN17 = temperature sensor
} AIN_TypeDef;

//gpio cnf1..0 + mod1..0
#define GPIOMODE_AN			(1<<0)			//0b0000->analog input vs. digital
#define GPIOMODE_DG			(0<<0)
#define GPIOMODE_PU			(1<<1)			//0b0010->floating vs. pull-up
#define GPIOMODE_FL			(0<<1)
#define GPIOMODE_OUT		(1<<2)			//0b0100->output vs. input
#define GPIOMODE_IN			(0<<2)

//device-specific extension of GPIO mode
//#define INPUT_PULLDN		(INPUT_PULLUP + 1)					//gpio input, with pull-down
#define INPUT_ANALOG		(INPUT_PULLUP + 2)					//analog input
//#define OUTPUT_OD			(INPUT_PULLUP + 3)					//gpio output, open-drain
//#define OUTPUT_AFPP			(INPUT_PULLUP + 4)					//alternative function, pushpull output
//#define OUTPUT_AFOD			(INPUT_PULLUP + 5)					//alternative function, pushpull output

//exti / attachInterrupt() mode
//#define EDGE_FALLING		0x01			//falling edge for EXTI
//#define EDGE_RISING		0x02			//rising edge for EXTI
//#define EDGE_CHANGE		(EDGE_FALLING | EDGE_RISING)


//global variables

//port/gpio oriented macros
#define GIO_SET(port, pins)					port->SET = (pins)				//set bits on port
#define GIO_CLR(port, pins)					port->CLR = (pins)				//clear bits on port
#define GIO_FLP(port, pins)					port->NOT = (pins)				//flip bits on port
#define GIO_GET(port, pins)					((port->PIN) & (pins))			//return bits on port
//set a pin to output/input
#define GIO_OUTPP(port, pins)				GPIO_DDR(port, pins, GPIOMODE_OUTPP)	//push-pull mode (CR1 set, CR2 cleared)	//IO_OUTPP(GPIOx, GPIO_Pins).
#define GIO_OUTOD(port, pins)				GPIO_DDR(port, pins, GPIOMODE_OUTOD)	//open drain mode (cr1 + cr2 cleared)	//_GPIO_Init(GPIOx, GPIO_Pins, GPIO_MODE_OUT_OD_LOW_FAST)
#define GIO_OUT(port, pins)					GIO_OUTPP(port, pins)					//_GPIO_Init(GPIOx, GPIO_Pins, GPIO_MODE_OUT_PP_LOW_FAST)
#define GIO_INFL(port, pins)				GPIO_DDR(port, pins, GPIOMODE_INFL)		//floating input, no interrupt			//IO_INFL(GPIOx, GPIO_Pins)
#define GIO_INPU(port, pins)				GPIO_DDR(port, pins, GPIOMODE_INPU)		//pull-up, no interrupt					//_GPIO_Init(GPIOx, GPIO_Pins, GPIO_MODE_IN_PU_NO_IT)
#define GIO_IN(port, pins)					GIO_INFL(port, pins)					//IO_IN(port, pins)				//_GPIO_Init(GPIOx, GPIO_Pins, GPIO_MODE_IN_FL_NO_IT)
#define GIO_AFPP(port, pins)				GPIO_DDR(port, pins, GPIOMODE_AFPP)		//configure pin for alternative function output, push-pull, 10Mhz
#define GIO_AFOD(port, pins)				GPIO_DDR(port, pins, GPIOMODE_AFOD)		//configure pin for alternative function output, open-drain, 10Mhz
#define GIO_AN(port, pins)					GPIO_DDR(port, pins, GPIOMODE_INAN)


//Arduino GPIO port
//configure gpio mode (cnf1..0 + mod1..0 bits)
void GPIO_DDR(LPC_GPIO_Type * gpio, uint32_t mask, uint32_t mode);

//extensions to arduino
//hardware dependent routines
//output a 12-bit value on DACn ch 1/2
void DAC1Write(uint16_t val);
void DAC2Write(uint16_t val);

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
void serial1Print(unsigned char *str);
void serial1Println(unsigned char *str);
void serial1Write(unsigned char ch);
unsigned char serial1Read(void);
uint8_t serial2Available(void);

//uart2
void serial2Begin(uint32_t baudrate);
void serial2Print(unsigned char *str);
void serial2Println(unsigned char *str);
void serial2Write(unsigned char ch);
unsigned char serial2Read(void);
uint8_t serial2Available(void);

//uart3
void serial3Begin(uint32_t baudrate);
void serial3Print(unsigned char *str);
void serial3Println(unsigned char *str);
void serial3Write(unsigned char ch);
unsigned char serial3Read(void);
uint8_t serial3Available(void);


//initialize the chip
void chip_init(void);
#endif
