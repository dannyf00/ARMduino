//Arduino GPIO port for LM4F120 family
//software environment: ccs, no CMSIS
//
//version: 0.12 @ 6/8/2017
//serial comms implemented
//attachInterrupt() / detachInterrupt() implemented
//
//version: 0.11 @ 6/5/2017
//implemented over Tivaware
//
//version: v0.10 @ 5/27/2017
//initial release, support for LM4F120 chips
//Supported functions:
//GPIO: pinMode(), digitalWrite(), digitalRead()
//Timing: micros(), millis() and ticks()


#ifndef _ARMDUINO_LM4F120_H
#define _ARMDUINO_LM4F120_H

//we use TI's LM4F120 / with driverlib
#ifndef PART_TM4C1233H6PM
#define PART_TM4C1233H6PM					//LM4F120XL = TM4C1233H6PM. Needs modification for other chips or in the IDE
#endif

//standard defines
#include <stdint.h>							//standard data types
#include <stdbool.h>						//we use bool

//device specific defs
#include <inc/hw_memmap.h>					//needs memory address
//#include <inc/hw_gpio.h>					//fast port access
//#include <inc/hw_types.h>					//fast port access
//driverlib defines
#include <driverlib/sysctl.h>				//we use system control
#include <driverlib/systick.h>				//we use systick
#include <driverlib/interrupt.h>			//nvic
#include <driverlib/gpio.h>					//we use gpio
#include <driverlib/timer.h>				//timers, 16/32 and 32/64
#include <driverlib/pwm.h>					//pwm generator - not available on LM4F120
#include <driverlib/adc.h>					//we use adc0/1
#include <driverlib/ssi.h>					//we use ssi/spi
#include <driverlib/i2c.h>					//i2c defs
#include <driverlib/uart.h>					//uart
#include <driverlib/rom.h>
#include <driverlib/rom_map.h>				//in case to use rom code
#include <driverlib/pin_map.h>				//for pin mux on Blizzard class devices

//hardware configuration
#define PWMOUT_BITs				12			//pwm output resolution, [0..16]bits
#define USE_PWMOUT0							//if use pwm mode on timer0, both channels
#define USE_PWMOUT1							//if use pwm mode on timer1, both channels
//#define USE_PWMOUT2							//if use pwm mode on timer2, both channels
//#define USE_PWMOUT3							//if use pwm mode on timer3, both channels
//#define USE_PWMOUT4							//if use pwm mode on timer4, both channels
//#define USE_PWMOUT5							//if use pwm mode on timer5, both channels

//#define USE_PWMOUTW0						//if use pwm mode on wtimer0, both channels
//#define USE_PWMOUTW1						//if use pwm mode on wtimer1, both channels
//#define USE_PWMOUTW2						//if use pwm mode on wtimer2, both channels
//#define USE_PWMOUTW3						//if use pwm mode on wtimer3, both channels
//#define USE_PWMOUTW4						//if use pwm mode on wtimer4, both channels
//#define USE_PWMOUTW5						//if use pwm mode on wtimer5, both channels

#define USE_ADC0							//use adc0
#define USE_ADC1							//use adc1
#define ADC0_SEQ				3			//adc sequence for analogread0. [0..3]
#define ADC1_SEQ				3			//adc sequence for analogread1. [0..3]
#define ADC_PS					64			//adc prescaler / divider, [1..64]

#define USE_SPI0							//use ssi0
#define USE_SPI1							//use ssi1
#define USE_SPI2							//use ssi0
#define USE_SPI3							//use ssi1
#define SPI0_BPS				200000ul	//baud rate for SSI0
#define SPI1_BPS				100000ul	//baud rate for SSI1
#define SPI2_BPS				200000ul	//baud rate for SSI0
#define SPI3_BPS				100000ul	//baud rate for SSI1

//end hardware configuration

//global defines
//port/gpio oriented macros
#define GIO_SET(port, pins)					GPIOPinWrite(port, pins, pins)	//port->OUT |= (pins)				//set bits on port
#define GIO_CLR(port, pins)					GPIOPinWrite(port, pins, 0)		//port->OUT &=~(pins)				//clear bits on port
//#define GIO_FLP(port, pins)					port->OUT ^= (pins)				//flip bits on port
#define GIO_GET(port, pins)					GPIOPinRead(port, pins)			//((port->IN) & (pins))			//return bits on port
//set a pin to output/input
//#define GIO_OUTPP(port, pins)				GPIO_DDR(port, pins, GPIOMODE_OUTPP)	//push-pull mode (CR1 set, CR2 cleared)	//IO_OUTPP(GPIOx, GPIO_Pins).
//#define GIO_OUTOD(port, pins)				GPIO_DDR(port, pins, GPIOMODE_OUTOD)	//open drain mode (cr1 + cr2 cleared)	//_GPIO_Init(GPIOx, GPIO_Pins, GPIO_MODE_OUT_OD_LOW_FAST)
#define GIO_OUT(port, pins)					GPIOPinTypeGPIOOutput(port, pins)	//port->DIR |= (pins)				//GIO_OUTPP(port, pins)						//_GPIO_Init(GPIOx, GPIO_Pins, GPIO_MODE_OUT_PP_LOW_FAST)
//#define GIO_INFL(port, pins)				GPIO_DDR(port, pins, GPIOMODE_INFL)		//floating input, no interrupt			//IO_INFL(GPIOx, GPIO_Pins)
//#define GIO_INPU(port, pins)				GPIO_DDR(port, pins, GPIOMODE_INPU)		//pull-up, no interrupt					//_GPIO_Init(GPIOx, GPIO_Pins, GPIO_MODE_IN_PU_NO_IT)
#define GIO_IN(port, pins)					GPIOPinTypeGPIOInput(port, pins)	//port->DIR &=~(pins)				//GIO_INFL(port, pins)						//IO_IN(port, pins)				//_GPIO_Init(GPIOx, GPIO_Pins, GPIO_MODE_IN_FL_NO_IT)


//pwm output period
#define PWMOUT_PR				((1ul<<(PWMOUT_BITs)) - 1)	//PWM output period

//gpio enums - needs to match GPIO_PinDef[]
typedef enum {
	PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7,
	PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7,
	PC0, PC1, PC2, PC3, PC4, PC5, PC6, PC7,
	PD0, PD1, PD2, PD3, PD4, PD5, PD6, PD7,
	PE0, PE1, PE2, PE3, PE4, PE5, PE6, PE7,
	PF0, PF1, PF2, PF3, PF4, PF5, PF6, PF7,
	PG0, PG1, PG2, PG3, PG4, PG5, PG6, PG7,
	PH0, PH1, PH2, PH3, PH4, PH5, PH6, PH7,
#if defined(GPIOI)
	PI0, PI1, PI2, PI3, PI4, PI5, PI6, PI7,
#endif
	PJ0, PJ1, PJ2, PJ3, PJ4, PJ5, PJ6, PJ7,
	PK0, PK1, PK2, PK3, PK4, PK5, PK6, PK7,
	PINMAX,													//last entry, for error checking - not used
} PIN_TypeDef;

//analog input pin defs
typedef enum {
    A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11,       //AIN0..AIN11, on LM4F120
    //A12, A13, A14, A15, A16, A17, A18, A19, A20, A21, A22, A23       //AIN0..AIN11, on LM4F120
    ATEMP,                    	                          //temperature sensor
	AINMAX,													//last entry, for error checking
} AIN_TypeDef;

//PWM output pin defs
typedef enum {
	PWMPB6, PWMPF0, PWMPB7, PWMPF1,							//timer0 pwm outputs
	PWMPF2, PWMPB4, PWMPF3, PWMPB5, 						//timer1 pwm outputs
	PWMPF4, PWMPB0, PWMPB1,									//timer2 pwm outputs
	PWMPB2, PWMPB3,											//timer3 pwm outputs
	PWMPC0, PWMPC1,											//timer4 pwm outputs
	PWMPC2, PWMPC3,											//timer5 pwm outputs

	PWMPC4, PWMPC5,											//Wtimer0 pwm outputs
	PWMPC6, PWMPC7,											//Wtimer1 pwm outputs
	PWMPD0, PWMPD1,											//Wtimer2 pwm outputs
	PWMPD2, PWMPD3,											//Wtimer3 pwm outputs
	PWMPD4, PWMPD5,											//Wtimer4 pwm outputs
	PWMPD6, PWMPD7,											//Wtimer5 pwm outputs
	PWMMAX,													//last entry, for error checking
} PWMPIN_TypeDef;

//analog reference
typedef enum {
	DEFAULT, INTERNAL,									//ADC_REF_INT
	EXTERNAL,											//ADC_REF_EXT_3V
} AREF_TypeDef;

//uart defs
typedef enum {
	UART0, UART1, UART2, UART3,
	UART4, UART5, UART6, UART7,
	UARTMAX,										//max uart. for error checking
} UART_TypeDef;

//spi pin enum
typedef enum {
	SPI0SCK, SPI0SDO, SPI0SDI,						//defs for SPIx
	SPI1SCK, SPI1SDO, SPI1SDI,						//defs for SPIx
	SPI2SCK, SPI2SDO, SPI2SDI,						//defs for SPIx
	SPI3SCK, SPI3SDO, SPI3SDI,						//defs for SPIx
} SPIPIN_TypeDef;

//struct used to map a pin to GPIO+mask
typedef struct {
	volatile uint32_t gpio;							//gpio base address for a port
	uint8_t mask;									//pin mask - only 16-bit formated is supported in this port
} PIN2GPIO;

//Analog input pin defs
typedef struct {
	//volatile uint32_t gpio;							//gpio base address for a port
	//uint8_t mask;									//pin mask
	PIN_TypeDef pin;								//gpio pin + mask
	uint32_t ch;									//adc input channel
} AIN2GPIO;

//pwm output pin defs
typedef struct {
	//volatile uint32_t gpio;							//gpio base address for a port
	//uint8_t mask;									//pin mask
	PIN_TypeDef pin;								//gpio pin + mask
	uint32_t timer;									//timer base address associated with the pwm pin
	uint32_t ch;									//timer channel, A or B
	uint32_t pinconfig;								//pwm pin configuration from pin_map.h
} PWM2GPIO;

//uart pin defs
typedef struct {
	//uint32_t periph_uart;							//sysctrl_periph_uartn
	//uint32_t periph_gpio;							//sysctrl_periph_gpion - already enabed in the initialization call
	//uint32_t uart;									//uart base address
	//uint32_t gpio;									//gpio base address for a port
	//uint32_t mask_rx;								//mask for rx pin
	PIN_TypeDef pin_rx;								//rx pin
	uint32_t pinconfig_rx;							//mux for rx pin
	//uint32_t mask_tx;								//mask for tx pin
	PIN_TypeDef pin_tx;								//tx pin
	uint32_t pinconfig_tx;							//mux for tx pin
} UART2GPIO;

//spi pin defs
typedef struct {
	PIN_TypeDef pin;							//spi pin + mask
	uint32_t pinconfig;							//pinconfig from pin_map.h
} SPI2GPIO;

extern const PIN2GPIO GPIO_PinDef[];				//used by attachinterrupt to clear the int status

//global variables

//GPIO defs - shorter names
#define GPIOA				(GPIO_PORTA_BASE)
#define GPIOB				(GPIO_PORTB_BASE)
#define GPIOC				(GPIO_PORTC_BASE)
#define GPIOD				(GPIO_PORTD_BASE)
#define GPIOE				(GPIO_PORTE_BASE)
#define GPIOF				(GPIO_PORTF_BASE)
#define GPIOG				(GPIO_PORTG_BASE)
#define GPIOH				(GPIO_PORTH_BASE)
#if defined(GPIO_PORTI_BASE)
#define GPIOI				(GPIO_PORTI_BASE)
#endif
#define GPIOJ				(GPIO_PORTJ_BASE)
#define GPIOK				(GPIO_PORTK_BASE)
#define GPIOL				(GPIO_PORTL_BASE)
#define GPIOM				(GPIO_PORTM_BASE)
#define GPION				(GPIO_PORTN_BASE)
#define GPIOO				(GPIO_PORTO_BASE)
#define GPIOP				(GPIO_PORTP_BASE)
#define GPIOQ				(GPIO_PORTQ_BASE)

extern volatile uint32_t SystemCoreClock;				//default value for SystemCoreClock

uint32_t analognRead(uint32_t ADCn_BASE, AIN_TypeDef ain);
    void analognReference(uint32_t ADCn_BASE, AREF_TypeDef aref);

//analogRead() - chip-specific
//replicate for adc0/adc1 via a macro
#define analog0Read(ain)        analognRead(ADC0_BASE, ain)
#define analog1Read(ain)        analognRead(ADC1_BASE, ain)
#define analog0Reference(aref)	analognReference(ADC0_BASE, aref)
#define analog1Reference(aref)	analognReference(ADC1_BASE, aref)

//12-bit adc on the temperature sensor, from adc to temp x 10
#define Tx10(adc)				(1475 - ((75ul * (adc) * 30) >> 12))			//"3" = 3v, Vref_int

//serial comms
//initialize UART: 8-bit data, 1 stop bit, no parity
void serialnBegin(uint32_t uart, uint32_t bps);
#define serialnBusy(uart)		UARTBusy(uart)
#define serialnAvailable(uart)	UARTCharsAvail(uart)
#define serialnWrite(uart, ch)	UARTCharPut(uart, ch)
#define serialnRead(uart)		UARTCharGet(uart)
void serialnPrint(uint32_t uart, char *str);
void serialnPrintln(uint32_t uart, char *str);

//serial comms for uart0..7
#define serial0Begin(bps)		serialnBegin(UART0_BASE, bps)
#define serial0Busy()			serialnBusy(UART0_BASE)
#define serial0Available()		serialnAvailable(UART0_BASE)
#define serial0Write(ch)		serialnWrite(UART0_BASE, ch)
#define serial0Read()			serialnRead(UART0_BASE)
#define serial0Print(str)		serialnPrint(UART0_BASE, str)
#define serial0Println(str)		serialnPrintln(UART0_BASE, str)

#define serial1Begin(bps)		serialnBegin(UART1_BASE, bps)
#define serial1Busy()			serialnBusy(UART1_BASE)
#define serial1Available()		serialnAvailable(UART1_BASE)
#define serial1Write(ch)		serialnWrite(UART1_BASE, ch)
#define serial1Read()			serialnRead(UART1_BASE)
#define serial1Print(str)		serialnPrint(UART1_BASE, str)
#define serial1Println(str)		serialnPrintln(UART1_BASE, str)

#define serial2Begin(bps)		serialnBegin(UART2_BASE, bps)
#define serial2Busy()			serialnBusy(UART2_BASE)
#define serial2Available()		serialnAvailable(UART2_BASE)
#define serial2Write(ch)		serialnWrite(UART2_BASE, ch)
#define serial2Read()			serialnRead(UART2_BASE)
#define serial2Print(str)		serialnPrint(UART2_BASE, str)
#define serial2Println(str)		serialnPrintln(UART2_BASE, str)

#define serial3Begin(bps)		serialnBegin(UART3_BASE, bps)
#define serial3Busy()			serialnBusy(UART3_BASE)
#define serial3Available()		serialnAvailable(UART3_BASE)
#define serial3Write(ch)		serialnWrite(UART3_BASE, ch)
#define serial3Read()			serialnRead(UART3_BASE)
#define serial3Print(str)		serialnPrint(UART3_BASE, str)
#define serial3Println(str)		serialnPrintln(UART3_BASE, str)

#define serial4Begin(bps)		serialnBegin(UART4_BASE, bps)
#define serial4Busy()			serialnBusy(UART4_BASE)
#define serial4Available()		serialnAvailable(UART4_BASE)
#define serial4Write(ch)		serialnWrite(UART4_BASE, ch)
#define serial4Read()			serialnRead(UART4_BASE)
#define serial4Print(str)		serialnPrint(UART4_BASE, str)
#define serial4Println(str)		serialnPrintln(UART4_BASE, str)

#define serial5Begin(bps)		serialnBegin(UART5_BASE, bps)
#define serial5Busy()			serialnBusy(UART5_BASE)
#define serial5Available()		serialnAvailable(UART5_BASE)
#define serial5Write(ch)		serialnWrite(UART5_BASE, ch)
#define serial5Read()			serialnRead(UART5_BASE)
#define serial5Print(str)		serialnPrint(UART5_BASE, str)
#define serial5Println(str)		serialnPrintln(UART5_BASE, str)

#define serial6Begin(bps)		serialnBegin(UART6_BASE, bps)
#define serial6Busy()			serialnBusy(UART6_BASE)
#define serial6Available()		serialnAvailable(UART6_BASE)
#define serial6Write(ch)		serialnWrite(UART6_BASE, ch)
#define serial6Read()			serialnRead(UART6_BASE)
#define serial6Print(str)		serialnPrint(UART6_BASE, str)
#define serial6Println(str)		serialnPrintln(UART6_BASE, str)

#define serial7Begin(bps)		serialnBegin(UART7_BASE, bps)
#define serial7Busy()			serialnBusy(UART7_BASE)
#define serial7Available()		serialnAvailable(UART7_BASE)
#define serial7Write(ch)		serialnWrite(UART7_BASE, ch)
#define serial7Read()			serialnRead(UART7_BASE)
#define serial7Print(str)		serialnPrint(UART7_BASE, str)
#define serial7Println(str)		serialnPrintln(UART7_BASE, str)

//spi write to and then read from the bus
uint32_t SPInWrite(uint32_t spi, uint32_t dat);
#define SPI0Write(dat)			SPInWrite(SSI0_BASE, dat)
#define SPI1Write(dat)			SPInWrite(SSI1_BASE, dat)
#define SPI2Write(dat)			SPInWrite(SSI2_BASE, dat)
#define SPI3Write(dat)			SPInWrite(SSI3_BASE, dat)

//initialize the chip
void chip_init(void);
#endif
