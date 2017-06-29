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


#ifndef _ARMDUINO_LPC17XX_H
#define _ARMDUINO_LPC17XX_H

#include "lpc17xx.h"				//we use NXP LPC17xx chips

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
//GPIO00..31->PA0..31
//GPIO10..31->PB0..31
//GPIO20..31->PC0..31
//GPIO30..31->PD0..31
//GPIO40..31->PE0..31
typedef enum {
	PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9, PA10, PA11, PA12, PA13, PA14, PA15, PA16, PA17, PA18, PA19, PA20, PA21, PA22, PA23, PA24, PA25, PA26, PA27, PA28, PA29, PA30, PA31,			//GPIO0 pin defs
	PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15, PB16, PB17, PB18, PB19, PB20, PB21, PB22, PB23, PB24, PB25, PB26, PB27, PB28, PB29, PB30, PB31,			//GPIO1 pin defs
	PC0, PC1, PC2, PC3, PC4, PC5, PC6, PC7, PC8, PC9, PC10, PC11, PC12, PC13, PC14, PC15, PC16, PC17, PC18, PC19, PC20, PC21, PC22, PC23, PC24, PC25, PC26, PC27, PC28, PC29, PC30, PC31,			//GPIO2 pin defs
	PD0, PD1, PD2, PD3, PD4, PD5, PD6, PD7, PD8, PD9, PD10, PD11, PD12, PD13, PD14, PD15, PD16, PD17, PD18, PD19, PD20, PD21, PD22, PD23, PD24, PD25, PD26, PD27, PD28, PD29, PD30, PD31,			//GPIO3 pin defs
	PE0, PE1, PE2, PE3, PE4, PE5, PE6, PE7, PE8, PE9, PE10, PE11, PE12, PE13, PE14, PE15, PE16, PE17, PE18, PE19, PE20, PE21, PE22, PE23, PE24, PE25, PE26, PE27, PE28, PE29, PE30, PE31,			//GPIO4 pin defs
} PIN_TypeDef;

typedef enum {
	A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15,
	AVREFINT,									//AIN16 = Vrefint
	ATEMP,										//AIN17 = temperature sensor
} AIN_TypeDef;

//gpio cnf1..0 + mod1..0
//not yet supported
#define GPIOMODE_INAN		0x00			//0b0000->analog input
#define GPIOMODE_INFL		0x04			//0b0100->floating input
#define GPIOMODE_INPU		0x08			//0b1000->input, pull-up (odr=1)
#define GPIOMODE_INDN		0x08			//0b1000->input, pull-dn (odr=0)
#define GPIOMODE_OUTPP		0x01			//0b0001->pushpull output @ 10Mhz
#define GPIOMODE_OUTOD		0x09			//0b1001->open drain output @ 10Mhz
#define GPIOMODE_AFPP		0x09			//0b1001->alternative function output, push-pull, 10Mhz
#define GPIOMODE_AFOD		0x0d			//0b1101->alternative function output, open-drain, 10Mhz

//device-specific extension of GPIO mode
//not yet supported
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
#define GIO_SET(port, pins)					port->FIOSET = (pins)				//set bits on port
#define GIO_CLR(port, pins)					port->FIOCLR = (pins)				//clear bits on port
#define GIO_FLP(port, pins)					port->FIOPIN ^= (pins)				//flip bits on port
#define GIO_GET(port, pins)					((port->FIOPIN) & (pins))			//return bits on port
//set a pin to output/input
#define GIO_OUTPP(port, pins)				port->FIODIR |= (pins)				//push-pull mode (CR1 set, CR2 cleared)	//IO_OUTPP(GPIOx, GPIO_Pins).
//#define GIO_OUTOD(port, pins)				port->FIODIR |= (pins)				//open drain mode (cr1 + cr2 cleared)	//_GPIO_Init(GPIOx, GPIO_Pins, GPIO_MODE_OUT_OD_LOW_FAST)
#define GIO_OUT(port, pins)					GIO_OUTPP(port, pins)				//_GPIO_Init(GPIOx, GPIO_Pins, GPIO_MODE_OUT_PP_LOW_FAST)
#define GIO_INFL(port, pins)				port->FIODIR &=~(pins)				//floating input, no interrupt			//IO_INFL(GPIOx, GPIO_Pins)
//#define GIO_INPU(port, pins)				port->FIODIR &=~(pins)				//pull-up, no interrupt					//_GPIO_Init(GPIOx, GPIO_Pins, GPIO_MODE_IN_PU_NO_IT)
#define GIO_IN(port, pins)					GIO_INFL(port, pins)					//IO_IN(port, pins)				//_GPIO_Init(GPIOx, GPIO_Pins, GPIO_MODE_IN_FL_NO_IT)
//#define GIO_AFPP(port, pins)				GPIO_DDR(port, pins, GPIOMODE_AFPP)		//configure pin for alternative function output, push-pull, 10Mhz
//#define GIO_AFOD(port, pins)				GPIO_DDR(port, pins, GPIOMODE_AFOD)		//configure pin for alternative function output, open-drain, 10Mhz
//#define GIO_AN(port, pins)					GPIO_DDR(port, pins, GPIOMODE_INAN)


//Arduino GPIO port
//configure gpio mode (cnf1..0 + mod1..0 bits)
//void GPIO_DDR(GPIO_TypeDef * gpio, uint32_t mask, uint32_t mode);

//extensions to arduino
//hardware dependent routines
//output a 12-bit value on DACn ch 1/2
//void DAC1Write(uint16_t val);
//void DAC2Write(uint16_t val);

//spi
//uint8_t SPI1Write(uint8_t order, uint8_t dat);
//uint8_t SPI1Read(uint8_t order);
//uint8_t SPI2Write(uint8_t order, uint8_t dat);
//uint8_t SPI2Read(uint8_t order);

//return timebase ticks
//uint32_t ticks(void);

//usart routines
/*

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
*/

//initialize the chip
void chip_init(void);
#endif
