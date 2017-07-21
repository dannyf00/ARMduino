//hardware specific port for ARMduino to lpc12xx family

#include "armduino_lpc12xx.h"						//we use aruidno GPIO port for lpc12xx
#include "armduino.h"								//use ARMduino specific definitions

//global definitions

#define PWM_PR			((1ul<<PWMOUT_BITs) - 1)			//period for 16-bit pwm

//struct used to map a pin to GPIO+mask
typedef struct {
	LPC_GPIO_Type *gpio;				//gpio for a pin
	uint32_t mask;						//pin mask - 32bit
} PIN2GPIO;


//empty handler
static void empty_handler(void) {
	//do nothing here
}

#if defined(USE_EXTI)
//user isr
void (*_isrptr_exti[16])(void);			//user handler for EXTI interrupt

//exti interrupt handlers
void EXTI0_IRQHandler(void) {
	EXTI->PR |= (1<<0);					//clear the flag by writing '1' to it
	_isrptr_exti[0]();					//execute the user handler
}

void EXTI1_IRQHandler(void) {
	EXTI->PR |= (1<<1);					//clear the flag by writing '1' to it
	_isrptr_exti[1]();					//execute the user handler
}

void EXTI2_IRQHandler(void) {
	EXTI->PR |= (1<<2);					//clear the flag by writing '1' to it
	_isrptr_exti[2]();					//execute the user handler
}

void EXTI3_IRQHandler(void) {
	EXTI->PR |= (1<<3);					//clear the flag by writing '1' to it
	_isrptr_exti[3]();					//execute the user handler
}

void EXTI4_IRQHandler(void) {
	EXTI->PR |= (1<<4);					//clear the flag by writing '1' to it
	_isrptr_exti[4]();					//execute the user handler
}

void EXTI9_5_IRQHandler(void) {
	if (EXTI->PR & (1<<5)) {
		EXTI->PR |= (1<<5);				//clear the flag by writing '1' to it
		_isrptr_exti[5]();				//execute the user handler
	}

	if (EXTI->PR & (1<<6)) {
		EXTI->PR |= (1<<6);				//clear the flag by writing '1' to it
		_isrptr_exti[6]();				//execute the user handler
	}

	if (EXTI->PR & (1<<7)) {
		EXTI->PR |= (1<<7);				//clear the flag by writing '1' to it
		_isrptr_exti[7]();				//execute the user handler
	}

	if (EXTI->PR & (1<<8)) {
		EXTI->PR |= (1<<8);				//clear the flag by writing '1' to it
		_isrptr_exti[8]();				//execute the user handler
	}

	if (EXTI->PR & (1<<9)) {
		EXTI->PR |= (1<<9);				//clear the flag by writing '1' to it
		_isrptr_exti[9]();				//execute the user handler
	}
}

void EXTI15_10_IRQHandler(void) {
	if (EXTI->PR & (1<<10)) {
		EXTI->PR |= (1<<10);			//clear the flag by writing '1' to it
		_isrptr_exti[10]();				//execute the user handler
	}

	if (EXTI->PR & (1<<11)) {
		EXTI->PR |= (1<<11);			//clear the flag by writing '1' to it
		_isrptr_exti[11]();				//execute the user handler
	}

	if (EXTI->PR & (1<<12)) {
		EXTI->PR |= (1<<12);			//clear the flag by writing '1' to it
		_isrptr_exti[12]();				//execute the user handler
	}

	if (EXTI->PR & (1<<13)) {
		EXTI->PR |= (1<<13);			//clear the flag by writing '1' to it
		_isrptr_exti[13]();				//execute the user handler
	}

	if (EXTI->PR & (1<<14)) {
		EXTI->PR |= (1<<14);			//clear the flag by writing '1' to it
		_isrptr_exti[14]();				//execute the user handler
	}

	if (EXTI->PR & (1<<15)) {
		EXTI->PR |= (1<<15);			//clear the flag by writing '1' to it
		_isrptr_exti[15]();				//execute the user handler
	}
}
//end exti handlers
#endif

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
//Pin 80..95 -> GPIOF
//Pin 96..111 -> GPIOG
const PIN2GPIO GPIO_PinDef[]={
		{LPC_GPIO0, 1<< 0},						//ARMduino Pin  0 = P0_0
		{LPC_GPIO0, 1<< 1},						//ARMduino Pin  1 = P0_1
		{LPC_GPIO0, 1<< 2}, 					//ARMduino Pin  2 = P0_2
		{LPC_GPIO0, 1<< 3},						//ARMduino Pin  3 = P0_3
		{LPC_GPIO0, 1<< 4},						//ARMduino Pin  4 = P0_4
		{LPC_GPIO0, 1<< 5},						//ARMduino Pin  5 = P0_5
		{LPC_GPIO0, 1<< 6},						//ARMduino Pin  6 = P0_6
		{LPC_GPIO0, 1<< 7},						//ARMduino Pin  7 = P0_7
		{LPC_GPIO0, 1<< 8},						//ARMduino Pin  8 = P0_8
		{LPC_GPIO0, 1<< 9},						//ARMduino Pin  9 = P0_9
		{LPC_GPIO0, 1<<10},						//ARMduino Pin 10 = P0_10
		{LPC_GPIO0, 1<<11},						//ARMduino Pin 11 = P0_11
		{LPC_GPIO0, 1<<12},						//ARMduino Pin 12 = P0_12
		{LPC_GPIO0, 1<<13},						//ARMduino Pin 13 = P0_13
		{LPC_GPIO0, 1<<14},						//ARMduino Pin 14 = P0_14
		{LPC_GPIO0, 1<<15},						//ARMduino Pin 15 = P0_15
		{LPC_GPIO0, 1<<16},						//ARMduino Pin  0 = P0_16
		{LPC_GPIO0, 1<<17},						//ARMduino Pin  1 = P0_17
		{LPC_GPIO0, 1<<18}, 					//ARMduino Pin  2 = P0_18
		{LPC_GPIO0, 1<<19},						//ARMduino Pin  3 = P0_19
		{LPC_GPIO0, 1<<20},						//ARMduino Pin  4 = P0_20
		{LPC_GPIO0, 1<<21},						//ARMduino Pin  5 = P0_21
		{LPC_GPIO0, 1<<22},						//ARMduino Pin  6 = P0_22
		{LPC_GPIO0, 1<<23},						//ARMduino Pin  7 = P0_23
		{LPC_GPIO0, 1<<24},						//ARMduino Pin  8 = P0_24
		{LPC_GPIO0, 1<<25},						//ARMduino Pin  9 = P0_25
		{LPC_GPIO0, 1<<26},						//ARMduino Pin 10 = P0_26
		{LPC_GPIO0, 1<<27},						//ARMduino Pin 11 = P0_27
		{LPC_GPIO0, 1<<28},						//ARMduino Pin 12 = P0_28
		{LPC_GPIO0, 1<<29},						//ARMduino Pin 13 = P0_29
		{LPC_GPIO0, 1<<30},						//ARMduino Pin 14 = P0_30
		{LPC_GPIO0, 1<<31},						//ARMduino Pin 15 = P0_31

		{LPC_GPIO1, 1<< 0},						//ARMduino Pin  0 = P1_0
		{LPC_GPIO1, 1<< 1},						//ARMduino Pin  1 = P1_1
		{LPC_GPIO1, 1<< 2}, 					//ARMduino Pin  2 = P1_2
		{LPC_GPIO1, 1<< 3},						//ARMduino Pin  3 = P1_3
		{LPC_GPIO1, 1<< 4},						//ARMduino Pin  4 = P1_4
		{LPC_GPIO1, 1<< 5},						//ARMduino Pin  5 = P1_5
		{LPC_GPIO1, 1<< 6},						//ARMduino Pin  6 = P1_6
		{LPC_GPIO1, 1<< 7},						//ARMduino Pin  7 = P1_7
		{LPC_GPIO1, 1<< 8},						//ARMduino Pin  8 = P1_8
		{LPC_GPIO1, 1<< 9},						//ARMduino Pin  9 = P1_9
		{LPC_GPIO1, 1<<10},						//ARMduino Pin 10 = P1_10
		{LPC_GPIO1, 1<<11},						//ARMduino Pin 11 = P1_11
		{LPC_GPIO1, 1<<12},						//ARMduino Pin 12 = P1_12
		{LPC_GPIO1, 1<<13},						//ARMduino Pin 13 = P1_13
		{LPC_GPIO1, 1<<14},						//ARMduino Pin 14 = P1_14
		{LPC_GPIO1, 1<<15},						//ARMduino Pin 15 = P1_15
		{LPC_GPIO1, 1<<16},						//ARMduino Pin  0 = P1_16
		{LPC_GPIO1, 1<<17},						//ARMduino Pin  1 = P1_17
		{LPC_GPIO1, 1<<18}, 					//ARMduino Pin  2 = P1_18
		{LPC_GPIO1, 1<<19},						//ARMduino Pin  3 = P1_19
		{LPC_GPIO1, 1<<20},						//ARMduino Pin  4 = P1_20
		{LPC_GPIO1, 1<<21},						//ARMduino Pin  5 = P1_21
		{LPC_GPIO1, 1<<22},						//ARMduino Pin  6 = P1_22
		{LPC_GPIO1, 1<<23},						//ARMduino Pin  7 = P1_23
		{LPC_GPIO1, 1<<24},						//ARMduino Pin  8 = P1_24
		{LPC_GPIO1, 1<<25},						//ARMduino Pin  9 = P1_25
		{LPC_GPIO1, 1<<26},						//ARMduino Pin 10 = P1_26
		{LPC_GPIO1, 1<<27},						//ARMduino Pin 11 = P1_27
		{LPC_GPIO1, 1<<28},						//ARMduino Pin 12 = P1_28
		{LPC_GPIO1, 1<<29},						//ARMduino Pin 13 = P1_29
		{LPC_GPIO1, 1<<30},						//ARMduino Pin 14 = P1_30
		{LPC_GPIO1, 1<<31},						//ARMduino Pin 15 = P1_31

		{LPC_GPIO2, 1<< 0},						//ARMduino Pin  0 = P2_0
		{LPC_GPIO2, 1<< 1},						//ARMduino Pin  1 = P2_1
		{LPC_GPIO2, 1<< 2}, 					//ARMduino Pin  2 = P2_2
		{LPC_GPIO2, 1<< 3},						//ARMduino Pin  3 = P2_3
		{LPC_GPIO2, 1<< 4},						//ARMduino Pin  4 = P2_4
		{LPC_GPIO2, 1<< 5},						//ARMduino Pin  5 = P2_5
		{LPC_GPIO2, 1<< 6},						//ARMduino Pin  6 = P2_6
		{LPC_GPIO2, 1<< 7},						//ARMduino Pin  7 = P2_7
		{LPC_GPIO2, 1<< 8},						//ARMduino Pin  8 = P2_8
		{LPC_GPIO2, 1<< 9},						//ARMduino Pin  9 = P2_9
		{LPC_GPIO2, 1<<10},						//ARMduino Pin 10 = P2_10
		{LPC_GPIO2, 1<<11},						//ARMduino Pin 11 = P2_11
		{LPC_GPIO2, 1<<12},						//ARMduino Pin 12 = P2_12
		{LPC_GPIO2, 1<<13},						//ARMduino Pin 13 = P2_13
		{LPC_GPIO2, 1<<14},						//ARMduino Pin 14 = P2_14
		{LPC_GPIO2, 1<<15},						//ARMduino Pin 15 = P2_15
		{LPC_GPIO2, 1<<16},						//ARMduino Pin  0 = P2_16
		{LPC_GPIO2, 1<<17},						//ARMduino Pin  1 = P2_17
		{LPC_GPIO2, 1<<18}, 					//ARMduino Pin  2 = P2_18
		{LPC_GPIO2, 1<<19},						//ARMduino Pin  3 = P2_19
		{LPC_GPIO2, 1<<20},						//ARMduino Pin  4 = P2_20
		{LPC_GPIO2, 1<<21},						//ARMduino Pin  5 = P2_21
		{LPC_GPIO2, 1<<22},						//ARMduino Pin  6 = P2_22
		{LPC_GPIO2, 1<<23},						//ARMduino Pin  7 = P2_23
		{LPC_GPIO2, 1<<24},						//ARMduino Pin  8 = P2_24
		{LPC_GPIO2, 1<<25},						//ARMduino Pin  9 = P2_25
		{LPC_GPIO2, 1<<26},						//ARMduino Pin 10 = P2_26
		{LPC_GPIO2, 1<<27},						//ARMduino Pin 11 = P2_27
		{LPC_GPIO2, 1<<28},						//ARMduino Pin 12 = P2_28
		{LPC_GPIO2, 1<<29},						//ARMduino Pin 13 = P2_29
		{LPC_GPIO2, 1<<30},						//ARMduino Pin 14 = P2_30
		{LPC_GPIO2, 1<<31},						//ARMduino Pin 15 = P2_31
#if defined(LPC_GPIO3)
		{LPC_GPIO3, 1<< 0},						//ARMduino Pin  0 = P3_0
		{LPC_GPIO3, 1<< 1},						//ARMduino Pin  1 = P3_1
		{LPC_GPIO3, 1<< 2}, 					//ARMduino Pin  2 = P3_2
		{LPC_GPIO3, 1<< 3},						//ARMduino Pin  3 = P3_3
		{LPC_GPIO3, 1<< 4},						//ARMduino Pin  4 = P3_4
		{LPC_GPIO3, 1<< 5},						//ARMduino Pin  5 = P3_5
		{LPC_GPIO3, 1<< 6},						//ARMduino Pin  6 = P3_6
		{LPC_GPIO3, 1<< 7},						//ARMduino Pin  7 = P3_7
		{LPC_GPIO3, 1<< 8},						//ARMduino Pin  8 = P3_8
		{LPC_GPIO3, 1<< 9},						//ARMduino Pin  9 = P3_9
		{LPC_GPIO3, 1<<10},						//ARMduino Pin 10 = P3_10
		{LPC_GPIO3, 1<<11},						//ARMduino Pin 11 = P3_11
		{LPC_GPIO3, 1<<12},						//ARMduino Pin 12 = P3_12
		{LPC_GPIO3, 1<<13},						//ARMduino Pin 13 = P3_13
		{LPC_GPIO3, 1<<14},						//ARMduino Pin 14 = P3_14
		{LPC_GPIO3, 1<<15},						//ARMduino Pin 15 = P3_15
		{LPC_GPIO3, 1<<16},						//ARMduino Pin  0 = P3_16
		{LPC_GPIO3, 1<<17},						//ARMduino Pin  1 = P3_17
		{LPC_GPIO3, 1<<18}, 					//ARMduino Pin  2 = P3_18
		{LPC_GPIO3, 1<<19},						//ARMduino Pin  3 = P3_19
		{LPC_GPIO3, 1<<20},						//ARMduino Pin  4 = P3_20
		{LPC_GPIO3, 1<<21},						//ARMduino Pin  5 = P3_21
		{LPC_GPIO3, 1<<22},						//ARMduino Pin  6 = P3_22
		{LPC_GPIO3, 1<<23},						//ARMduino Pin  7 = P3_23
		{LPC_GPIO3, 1<<24},						//ARMduino Pin  8 = P3_24
		{LPC_GPIO3, 1<<25},						//ARMduino Pin  9 = P3_25
		{LPC_GPIO3, 1<<26},						//ARMduino Pin 10 = P3_26
		{LPC_GPIO3, 1<<27},						//ARMduino Pin 11 = P3_27
		{LPC_GPIO3, 1<<28},						//ARMduino Pin 12 = P3_28
		{LPC_GPIO3, 1<<29},						//ARMduino Pin 13 = P3_29
		{LPC_GPIO3, 1<<30},						//ARMduino Pin 14 = P3_30
		{LPC_GPIO3, 1<<31},						//ARMduino Pin 15 = P3_31
#endif
};

//configure gpio DDR mode (cnf1..0 + mod1..0 bits)
void GPIO_DDR(LPC_GPIO_Type * gpio, uint32_t mask, uint32_t mode) {
	__IO uint32_t *pio;							//pointer to LPC_IOCON->PIOx_y

	//set/clear output direction register
	if (mode & GPIOMODE_OUT) gpio->DIR |= mask; else gpio->DIR &=~mask;
	//find out PIOx_y associated with the mask
	if (gpio == LPC_GPIO0) {
		if (mask & (1ul <<  0)) pio = &(LPC_IOCON->PIO0_0);
		else if (mask & (1ul <<  1)) pio = &(LPC_IOCON->PIO0_1);
		else if (mask & (1ul <<  2)) pio = &(LPC_IOCON->PIO0_2);
		else if (mask & (1ul <<  3)) pio = &(LPC_IOCON->PIO0_3);
		else if (mask & (1ul <<  4)) pio = &(LPC_IOCON->PIO0_4);
		else if (mask & (1ul <<  5)) pio = &(LPC_IOCON->PIO0_5);
		else if (mask & (1ul <<  6)) pio = &(LPC_IOCON->PIO0_6);
		else if (mask & (1ul <<  7)) pio = &(LPC_IOCON->PIO0_7);
		else if (mask & (1ul <<  8)) pio = &(LPC_IOCON->PIO0_8);
		else if (mask & (1ul <<  9)) pio = &(LPC_IOCON->PIO0_9);
		else if (mask & (1ul << 10)) pio = &(LPC_IOCON->PIO0_10);
		else if (mask & (1ul << 11)) pio = &(LPC_IOCON->PIO0_11);
		else if (mask & (1ul << 12)) pio = &(LPC_IOCON->PIO0_12);
		else if (mask & (1ul << 13)) pio = &(LPC_IOCON->RESET_PIO0_13);
		else if (mask & (1ul << 14)) pio = &(LPC_IOCON->PIO0_14);
		else if (mask & (1ul << 15)) pio = &(LPC_IOCON->PIO0_15);
		else if (mask & (1ul << 16)) pio = &(LPC_IOCON->PIO0_16);
		else if (mask & (1ul << 17)) pio = &(LPC_IOCON->PIO0_17);
		else if (mask & (1ul << 18)) pio = &(LPC_IOCON->PIO0_18);
		else if (mask & (1ul << 19)) pio = &(LPC_IOCON->PIO0_19);
		else if (mask & (1ul << 20)) pio = &(LPC_IOCON->PIO0_20);
		else if (mask & (1ul << 21)) pio = &(LPC_IOCON->PIO0_21);
		else if (mask & (1ul << 22)) pio = &(LPC_IOCON->PIO0_22);
		else if (mask & (1ul << 23)) pio = &(LPC_IOCON->PIO0_23);
		else if (mask & (1ul << 24)) pio = &(LPC_IOCON->PIO0_24);
		//else if (mask & (1ul << 25)) pio = &(LPC_IOCON->PIO0_25);
		//else if (mask & (1ul << 26)) pio = &(LPC_IOCON->PIO0_26);
		else if (mask & (1ul << 27)) pio = &(LPC_IOCON->PIO0_27);
		else if (mask & (1ul << 28)) pio = &(LPC_IOCON->PIO0_28);
		else if (mask & (1ul << 29)) pio = &(LPC_IOCON->PIO0_29);
		//else if (mask & (1ul << 30)) pio = &(LPC_IOCON->PIO0_30);
		//else if (mask & (1ul << 31)) pio = &(LPC_IOCON->PIO0_31);
	}
	else if (gpio == LPC_GPIO1) {}
	else if (gpio == LPC_GPIO2) {}
#if defined(LPC_GPIO3)
#endif
	//set the pull-up/floating
	if (mode & GPIOMODE_PU) *pio |= (1ul<<4); else *pio &=~(1ul<<4); if (mode & GPIOMODE_AN) *pio &=~(1ul<<7); else *pio |= (1ul<<7);
}

//Arduino Functions: GPIO
//set a pin mode to INPUT or OUTPUT
//no error checking on PIN
inline void pinMode(PIN_TypeDef pin, uint8_t mode) {
	//simple implementation - just INPUT/OUTPUT supported
	//if (mode==INPUT) GIO_IN(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);
	//else GIO_OUT(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);

	//fancier implementation: supports INPUT, INPUT_PULLUP, OUTPUT
	//additional features commented out
	switch (mode) {
	case INPUT: 		GPIO_DDR(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask, GPIOMODE_IN); break;			//floating input
	case INPUT_PULLUP: 	GPIO_DDR(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask, GPIOMODE_IN | GPIOMODE_PU); break;			//input with pullup
	//case INPUT_PULLDN:	GPIO_DDR(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask, GPIOMODE_INDN); break;			//input with pulldown
	//case INPUT_ANALOG:	GPIO_DDR(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask, GPIOMODE_INAN); break;			//analog input
	case OUTPUT:		GPIO_DDR(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask, GPIOMODE_OUT); break;			//floating input
	//case OUTPUT_OD:		GPIO_DDR(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask, GPIOMODE_OUTOD); break;			//floating input
	//case OUTPUT_AFPP:	GPIO_DDR(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask, GPIOMODE_AFPP); break;			//floating input
	//case OUTPUT_AFOD:	GPIO_DDR(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask, GPIOMODE_AFOD); break;			//floating input
	default: break;			//do nothing
	}

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

#if defined(USE_PWM)
//analogWrite
void analogWrite(PIN_TypeDef pin, uint16_t dc) {
	dc=(dc>PWM_PR)?PWM_PR:dc;				//bound dc to [0..PWM_PR]
	switch (pin) {							//configure TIMx-OCi channels based on pin # -> for STM32F100
#if defined(TIM1)
	case PA8:									//pin 8 = PA8/TIM1-CH1
		GIO_AFPP(GPIO_PinDef[8].gpio, GPIO_PinDef[8].mask);
		TIM1->CCR1 = dc;					//set duty cycle
		TIM1->CCER|= (1<<(4*(1-1)));		//1->enable the output
		break;
	case PA9:									//pin 9 = PA9/TIM1-CH2
		GIO_AFPP(GPIO_PinDef[9].gpio, GPIO_PinDef[9].mask);
		TIM1->CCR2 = dc;					//set duty cycle
		TIM1->CCER|= (1<<(4*(2-1)));		//1->enable the output
		break;
	case PA10:								//pin 10 = PA10/TIM1-CH3
		GIO_AFPP(GPIO_PinDef[10].gpio, GPIO_PinDef[10].mask);
		TIM1->CCR3 = dc;					//set duty cycle
		TIM1->CCER|= (1<<(4*(3-1)));		//1->enable the output
		break;
	case PA11:								//pin 11 = PA11/TIM1-CH4
		GIO_AFPP(GPIO_PinDef[11].gpio, GPIO_PinDef[11].mask);
		TIM1->CCR4 = dc;					//set duty cycle
		TIM1->CCER|= (1<<(4*(4-1)));		//1->enable the output
		break;
#endif
#if defined(TIM2)
	case PA1:									//pin 1 = PA1/TIM2-CH2
		GIO_AFPP(GPIO_PinDef[1].gpio, GPIO_PinDef[1].mask);
		TIM2->CCR2 = dc;					//set duty cycle
		TIM2->CCER|= (1<<(4*(2-1)));		//1->enable the output
		break;
	case PA2:									//pin 2 = PA2/TIM2-CH3
		GIO_AFPP(GPIO_PinDef[2].gpio, GPIO_PinDef[2].mask);
		TIM2->CCR3 = dc;					//set duty cycle
		TIM2->CCER|= (1<<(4*(3-1)));		//1->enable the output
		break;
	case PA3:									//pin 3 = PA3/TIM2-CH4
		GIO_AFPP(GPIO_PinDef[3].gpio, GPIO_PinDef[3].mask);
		TIM2->CCR4 = dc;					//set duty cycle
		TIM2->CCER|= (1<<(4*(4-1)));		//1->enable the output
		break;
#endif
#if defined(TIM3)
	case PA6:									//pin 6 = PA6/TIM3-CH1
		GIO_AFPP(GPIO_PinDef[6].gpio, GPIO_PinDef[6].mask);
		TIM3->CCR1 = dc;					//set duty cycle
		TIM3->CCER|= (1<<(4*(1-1)));		//1->enable the output
		break;
	case PA7:									//pin 7 = PA7/TIM3-CH2
		GIO_AFPP(GPIO_PinDef[7].gpio, GPIO_PinDef[7].mask);
		TIM3->CCR2 = dc;					//set duty cycle
		TIM3->CCER|= (1<<(4*(2-1)));		//1->enable the output
		break;
	case PB0:								//pin 16 = PB0/TIM3-CH3
		GIO_AFPP(GPIO_PinDef[16].gpio, GPIO_PinDef[16].mask);
		TIM3->CCR3 = dc;					//set duty cycle
		TIM3->CCER|= (1<<(4*(3-1)));		//1->enable the output
		break;
	case PB1:								//pin 17 = PB1/TIM3-CH4
		GIO_AFPP(GPIO_PinDef[17].gpio, GPIO_PinDef[17].mask);
		TIM3->CCR4 = dc;					//set duty cycle
		TIM3->CCER|= (1<<(4*(4-1)));		//1->enable the output
		break;
#endif
#if defined(TIM4)
	case PB6:								//pin 22 = PB6/TIM4-CH1
		GIO_AFPP(GPIO_PinDef[22].gpio, GPIO_PinDef[22].mask);
		TIM4->CCR1 = dc;					//set duty cycle
		TIM4->CCER|= (1<<(4*(1-1)));		//1->enable the output
		break;
	case PB7:								//pin 23 = PB7/TIM4-CH2
		GIO_AFPP(GPIO_PinDef[23].gpio, GPIO_PinDef[23].mask);
		TIM4->CCR2 = dc;					//set duty cycle
		TIM4->CCER|= (1<<(4*(2-1)));		//1->enable the output
		break;
	case PB8:								//pin 24 = PB8/TIM4-CH3
		GIO_AFPP(GPIO_PinDef[24].gpio, GPIO_PinDef[24].mask);
		TIM4->CCR3 = dc;					//set duty cycle
		TIM4->CCER|= (1<<(4*(3-1)));		//1->enable the output
		break;
	case PB9:								//pin 25 = PB9/TIM4-CH4
		GIO_AFPP(GPIO_PinDef[25].gpio, GPIO_PinDef[25].mask);
		TIM4->CCR4 = dc;					//set duty cycle
		TIM4->CCER|= (1<<(4*(4-1)));		//1->enable the output
		break;
#endif
	default: break;
	}

}
#endif										//pwm

#if defined(USE_DAC)
//extended function
//output a 12-bit value on DACn ch 1
//DAC1 = PA4 = Pin 4
void DAC1Write(uint16_t val) {
	GIO_AFPP(GPIO_PinDef[4].gpio, GPIO_PinDef[4].mask);	//configure pin 4 as afio output
	DAC->DHR12R1 = val & 0x0ffful;			//bound the value to [0..4095]
	DAC->SWTRIGR|= (1<<0);					//output on DAC1 - cleared by hardware
}

//output a 12-bit value on DACn ch 2
//DAC2 = PA5 = Pin 5
void DAC2Write(uint16_t val) {
	GIO_AFPP(GPIO_PinDef[5].gpio, GPIO_PinDef[5].mask);	//configure pin 5 as afio output
	DAC->DHR12R2 = val & 0x0ffful;			//bound the value to [0..4095]
	DAC->SWTRIGR|= (1<<1);					//output on DAC2 - cleared by hardware
}
#endif										//dac

#if defined(USE_ADC1)
//analog to digital converter on ADC1
//ain/analog input channel: ain valid values: 0..15, 16=temperature sensor, 17=Vrefint
//***user needs to be configured as floating input***
uint16_t analogRead(AIN_TypeDef ain) {
	ADC1->SR &= ~(1<<1);					//clear the eoc flag
	//ADC1->CR1 = (ADC1->CR1 &~0x1f) | (ain & 0x1f);	//pick the adc channel
	//ADC1->CR2|= (1<<0);						//start the conversion
	ADC1->SQR3 = ain & 0x1f;					//define the first (and only) adc ch
	ADC1->CR2 |= (1<<22);						//start conversion
	while ((ADC1->SR & (1<<1)) == 0) continue;	//wait for conversion to end (EOC set at end of conversion)
	return ADC1->DR;						//return adc results
}
#endif										//adc1

#if defined(USE_EXTI)
//attach an exti
//pin 0..15
void attachInterrupt(PIN_TypeDef pin, void (*isrptr)(void), uint8_t mode) {
    GPIO_TypeDef *gpio = GPIO_PinDef[pin].gpio;
    uint16_t mask;                  // = GPIO_PinDef[pin].mask;
    uint16_t pos;                    //pin position

    //mode = mode & 0x0f
    //figure out the mask
    if (gpio == GPIOA) mask = 0x00;	//exti on GPIOx: 0x00->A, 0x01->B, ..., 0x06->G
    if (gpio == GPIOB) mask = 0x01;
    if (gpio == GPIOC) mask = 0x02;
    if (gpio == GPIOD) mask = 0x03;
    if (gpio == GPIOE) mask = 0x04;
    if (gpio == GPIOF) mask = 0x05;
    if (gpio == GPIOG) mask = 0x06;

    //scan for pin position
    for (pos = 0; pos < 16; pos ++)
        if (GPIO_PinDef[pin].mask & (1<<pos)) break;
    //not pos has the pin position

    //figure out the line 0..15
         if (pos < 4) AFIO->EXTICR[0] = (AFIO->EXTICR[0] &~(0x0f << (4*(pos- 0)))) | (mask << (4*(pos - 0)));  //set the line 0..4
    else if (pos < 8) AFIO->EXTICR[1] = (AFIO->EXTICR[1] &~(0x0f << (4*(pos- 4)))) | (mask << (4*(pos - 4)));  //set the line 5..8
    else if (pos <12) AFIO->EXTICR[2] = (AFIO->EXTICR[2] &~(0x0f << (4*(pos- 8)))) | (mask << (4*(pos - 8)));  //set the line 9..12
    else if (pos <16) AFIO->EXTICR[3] = (AFIO->EXTICR[3] &~(0x0f << (4*(pos-12)))) | (mask << (4*(pos -12)));  //set the line 13..16

    //figure out the trigger: rising or falling
    switch (mode) {
    case CHANGE:	EXTI->FTSR |= (1<<pos); EXTI->RTSR |= (1<<pos); break;		//set falling trigger
    case RISING: 	EXTI->RTSR |= (1<<pos); break;		//set rising trigger
    case FALLING:
    default: 		EXTI->FTSR |= (1<<pos); break;		//set falling trigger (default)
    }

    //install the user handler
	_isrptr_exti[pos] = isrptr;
	//set it to the lowest priority
    switch (pos) {
    case 0: NVIC_SetPriority(EXTI0_IRQn, 15); NVIC_EnableIRQ(EXTI0_IRQn); break;
    case 1: NVIC_SetPriority(EXTI1_IRQn, 15); NVIC_EnableIRQ(EXTI1_IRQn); break;
    case 2: NVIC_SetPriority(EXTI2_IRQn, 15); NVIC_EnableIRQ(EXTI2_IRQn); break;
    case 3: NVIC_SetPriority(EXTI3_IRQn, 15); NVIC_EnableIRQ(EXTI3_IRQn); break;
    case 4: NVIC_SetPriority(EXTI4_IRQn, 15); NVIC_EnableIRQ(EXTI4_IRQn); break;
    case 5:
    case 6:
    case 7:
    case 8:
    case 9: NVIC_SetPriority(EXTI9_5_IRQn, 15); NVIC_EnableIRQ(EXTI9_5_IRQn); break;
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15: NVIC_SetPriority(EXTI15_10_IRQn, 15); NVIC_EnableIRQ(EXTI15_10_IRQn); break;
    default: break;		//do nothing
    }

    //activate the interrupt
    EXTI->PR |= (1<<pos);                           //clear the flag
    EXTI->IMR |= (1<<pos);                          //enable the exti
}

//deactivate an interrupt
void detachInterrupt(PIN_TypeDef pin) {
    //GPIO_TypeDef *gpio = GPIO_PinDef[pin].gpio;
    //uint16_t mask;                  // = GPIO_PinDef[pin].mask;
    uint16_t pos;                    //pin position

    //mode = mode & 0x0f
    //figure out the mask
    //if (gpio == GPIOA) mask = 0x00;	//exti on GPIOx: 0x00->A, 0x01->B, ..., 0x06->G
    //if (gpio == GPIOB) mask = 0x01;
    //if (gpio == GPIOC) mask = 0x02;
    //if (gpio == GPIOD) mask = 0x03;
    //if (gpio == GPIOE) mask = 0x04;
    //if (gpio == GPIOF) mask = 0x05;
    //if (gpio == GPIOG) mask = 0x06;

    //scan for pin position
    for (pos = 0; pos < 16; pos ++)
        if (GPIO_PinDef[pin].mask & (1<<pos)) break;
    //not pos has the pin position

    //figure out the line 0..16
    //     if (pos <= 4) AFIO->EXTICR[0] = (AFIO->EXTICR[0] &~(0x0f << (pos- 0))) | (mask << (pos - 0));  //set the line 0..4
    //else if (pos <= 8) AFIO->EXTICR[1] = (AFIO->EXTICR[1] &~(0x0f << (pos- 4))) | (mask << (pos - 4));  //set the line 5..8
    //else if (pos <=12) AFIO->EXTICR[2] = (AFIO->EXTICR[2] &~(0x0f << (pos- 8))) | (mask << (pos - 8));  //set the line 9..12
    //else if (pos <=16) AFIO->EXTICR[3] = (AFIO->EXTICR[3] &~(0x0f << (pos-12))) | (mask << (pos -12));  //set the line 13..16

    //figure out the trigger: rising or falling
    //if (mode & EDGE_RISING) EXTI->RTSR |= (1<<pos);  //set rising trigger
    //if (mode & EDGE_FALLING) EXTI->FTSR |= (1<<pos);    //set falling trigger

    //install the default handler
	_isrptr_exti[pos] = empty_handler;					//isrptr;
	//set it to the lowest priority, and disable the IRQ
    switch (pos) {
    case 0: NVIC_SetPriority(EXTI0_IRQn, 15); NVIC_DisableIRQ(EXTI0_IRQn); break;
    case 1: NVIC_SetPriority(EXTI1_IRQn, 15); NVIC_DisableIRQ(EXTI1_IRQn); break;
    case 2: NVIC_SetPriority(EXTI2_IRQn, 15); NVIC_DisableIRQ(EXTI2_IRQn); break;
    case 3: NVIC_SetPriority(EXTI3_IRQn, 15); NVIC_DisableIRQ(EXTI3_IRQn); break;
    case 4: NVIC_SetPriority(EXTI4_IRQn, 15); NVIC_DisableIRQ(EXTI4_IRQn); break;
    case 5:
    case 6:
    case 7:
    case 8:
    case 9: NVIC_SetPriority(EXTI9_5_IRQn, 15); NVIC_DisableIRQ(EXTI9_5_IRQn); break;
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15: NVIC_SetPriority(EXTI15_10_IRQn, 15); NVIC_DisableIRQ(EXTI15_10_IRQn); break;
    default: break;		//do nothing
    }

    //activate the interrupt
    EXTI->PR |= (1<<pos);                           //clear the flag
    EXTI->IMR &=~(1<<pos);                          //disable the exti
}
#endif												//exti

#if defined(USE_SPI1)
//spi routines
//send char via hardware spi
//MISO on PC7, MOSI on PC6, SCK on PC5
//Order = LSBFIRST or MSBFIRST
uint8_t SPI1Write(uint8_t order, uint8_t dat) {
    if (order == LSBFIRST) SPI1->CR1 |= SPI_CR1_LSBFIRST;  	//'1'->LSB first
    else SPI1->CR1 &=~SPI_CR1_LSBFIRST;                    	//'0'->MSB first
    while ((SPI1->SR & SPI_SR_TXE)==0) continue;     		//wait for transmit buffer to be empty (bit 1 goes to 1)
    SPI1->DR = dat;                          				//load the transmission buffer -> transmission starts. this approach can be risk for reads
    //consider test busy signal
    while (SPI1->SR & SPI_SR_BSY) continue;      			//'1'->SPI is buy, '0'->SPI is not busy
    //comment the above line for higher throughput if you don't care about the read-back
    return SPI1->DR;                         				//return spi data buffer
}

//read spi
//MISO on PC7, MOSI on PC6, SCK on PC5
//Order = LSBFIRST or MSBFIRST
uint8_t SPI1Read(uint8_t order) {
    //uint8_t tmp;
    if (order == LSBFIRST) SPI1->CR1 |= SPI_CR1_LSBFIRST;  	//'1'->LSB first
    else SPI1->CR1 &=~SPI_CR1_LSBFIRST;                    	//'0'->MSB first
    while ((SPI1->SR & SPI_SR_TXE)==0) continue;     		//wait for transmit buffer to be empty (bit 1 goes to 1)
    SPI1->DR = /*dat*/0x00;                      			//load the transmission buffer -> transmission starts. this approach can be risk for reads
    //consider test busy signal
    while (SPI1->SR & SPI_SR_BSY) continue;      			//'1'->SPI is buy, '0'->SPI is not busy
    //comment the above line for higher throughput if you don't care about the read-back
    return SPI1->DR;                         				//return spi data buffer
}

#endif											//spi1

#if defined(USE_SPI2)
//spi routines
//send char via hardware spi
//MISO on PC7, MOSI on PC6, SCK on PC5
//Order = LSBFIRST or MSBFIRST
uint8_t SPI2Write(uint8_t order, uint8_t dat) {
    if (order == LSBFIRST) SPI2->CR1 |= SPI_CR1_LSBFIRST;  //'1'->LSB first
    else SPI2->CR1 &=~SPI_CR1_LSBFIRST;                    //'0'->MSB first
    while ((SPI2->SR & SPI_SR_TXE)==0) continue;     //wait for transmit buffer to be empty (bit 1 goes to 1)
    SPI2->DR = dat;                          	//load the transmission buffer -> transmission starts. this approach can be risk for reads
    //consider test busy signal
    while (SPI2->SR & SPI_SR_BSY) continue;      //'1'->SPI is buy, '0'->SPI is not busy
    //comment the above line for higher throughput if you don't care about the read-back
    return SPI2->DR;                         	//return spi data buffer
}

//read spi
//MISO on PC7, MOSI on PC6, SCK on PC5
//Order = LSBFIRST or MSBFIRST
uint8_t SPI2Read(uint8_t order) {
    //uint8_t tmp;
    if (order == LSBFIRST) SPI2->CR1 |= SPI_CR1_LSBFIRST;  //'1'->LSB first
    else SPI2->CR1 &=~SPI_CR1_LSBFIRST;                    //'0'->MSB first
    while ((SPI2->SR & SPI_SR_TXE)==0) continue;      //wait for transmit buffer to be empty (bit 1 goes to 1)
    SPI2->DR = /*dat*/0x00;                      //load the transmission buffer -> transmission starts. this approach can be risk for reads
    //consider test busy signal
    while (SPI2->SR & SPI_SR_BSY) continue;      //'1'->SPI is buy, '0'->SPI is not busy
    //comment the above line for higher throughput if you don't care about the read-back
    return SPI2->DR;                         	//return spi data buffer
}

#endif											//spi2

#if defined(USE_UART1)
//initialize uart1
void serial1Begin(uint32_t baudrate) {
    //configure uart1
    //route clock to uart1
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    USART1->CR1 &=~(1<<13);			//'0'->disable uart, '1'->enable uart
    USART1->CR1 = 	(0<<15) |		//'0'->oversample by 16, '1'->oversample by 8
    				(0<<13) |		//'0'->disable uart, '1'->enable uart
    				(0<<12) |		//'0'->1 start bit, 8 data bit, n stop bit, '1'->1 start bit, 9 data bit, n stop bit
    				(0<<11) |		//'0'->idle line, '1'->wake by address
    				(0<<10) |		//'0'->disable parity, '1'->enable parity
    				(0<< 9) |		//'0'->even parity, '1'->odd parity
    				(0<< 8) |		//'0'->PE interrupt disabled, '1'->pe interrupt enabled
    				(0<< 7) |		//'0'->tx empty interrupt disabled, '1'-tx empty interrupt enabled
    				(0<< 6) |		//'0'->transmission completion interrupt disabled, '1'->transmission complete interrupt enabled
    				(0<< 5) |		//'0'->rx not empty interrupt disabled, '1'->rx not empty interrupt enabled
    				(0<< 4) |		//'0'->idle interrupt disabled, '1'->idle interrupt enabled
    				(1<< 3) |		//'0'->disable tx, '1'->enable tx
    				(1<< 2) |		//'0'->disable rx, '1'->enable rx
    				(0<< 1) |		//'0'->disable receiver wake-up, '1'->enable receiver wake-up
    				(0<< 0) |		//'0'->no break char is transmitted, '1'->break char will be transmitted
    				0x00;
    USART1->CR2 = 	(0<<14) |		//'0'->LIN mode disabled, '1'->LIN mode enabled
    				(0<<12) |		//0->1 stop bit, 1->0.5 stop bit, 2->2 stop bit, 3 -> 1.5 stop bit
    				(0<<11) |		//'0'->SCLK pin disabled, '1'->SCLK pin enabled
    				(0<<10) |		//'0'->SCLK idles low, '1'->SCLK idles high
    				(0<< 9) |		//'0'->first clock transition is first data capture edge, '1'->second clock transition is the first data capture edge
    				(0<< 8) |		//'0'->clock pulse of the last data bit is not output to the sclk pin, '1'->clock pulse of the last data bit is output to the sclk pin
    				(0<< 6) |		//'0'->LIN break detection interrupt disabled, '1'->LIN break detection interrupt enabled
    				(0<< 5) |		//'0'->LIN break detection is 10-bit, '1'->LIN break detection is 11 bit
    				(0<< 0) |		//address of the uart node
    				0x00;
    USART1->CR3 = 	(0<<11) |		//'0'->three sample bit, '1'->one sample bit
    				(0<<10) |		//'0'->CTS interrupt disabled, '1'->CTS interrupt enabled
    				(0<< 9) |		//'0'->CTS disabled, '1'->CTS enabled
    				(0<< 8) |		//'0'->RTS interrupt disabled, '1'->RTS interrupt enabled
    				(0<< 5) |		//'0'->smartcard mode disabled, '1'->smartcard mode enabled
    				(0<< 4) |		//'0'->smartcard nack disabled, '1'->smartcard nack enabled
    				(0<< 3) |		//'0'->half duplex mode not selected, '1'->half duplex mode selected
    				(0<< 2) |		//'0'->irda normal mode, '1'->irda low-power mode
    				(0<< 1) |		//'0'->irda disabled, '1'->irda enabled
    				(0<< 0) |		//'0'->error interrupt disabled, '1'->error interrupt enabled
    				0x00;
    //set the baudrate register
    USART1->BRR = SystemCoreClock / baudrate / 2 / ((USART1->CR1 & (1<<15))?1:2);		//per datasheet, for OVER8=0 or 1
    USART1->DR = 0;					//reset the data register
    USART1->SR = 0;					//reset the data register
    //enable uart1
    USART1->CR1 |= (1<<13);			//'0'->disable uart, '1'->enable uart

    //configure the RX-PA9/TX-PA10 pins - GPIO clock assumed enabled here previously
    GIO_INFL(GPIO_PinDef[PA9].gpio, GPIO_PinDef[PA9].mask);		//RX as floating input
    GIO_AFPP(GPIO_PinDef[PA10].gpio, GPIO_PinDef[PA10].mask);	//TX as AFPP

}

//uart1 send a char
void serial1Write(unsigned char dat) {
    USART1->DR = dat;                        	//load the data buffer
    //while (!(USART1->SR & (1<<6)));    		//wait for the transmission to complete
    while (!(USART1->SR & USART_SR_TXE));    			//wait for the transmission buffer to be empty
}

//uart1 returns a char
unsigned char serial1Read(void) {
    while (!(USART1->SR & USART_SR_RXNE));  			//wait for the receipt buffer to be empty
    return USART1->DR;                       	//save the transmission buffer
}

//uart1 print a string
void serial1Print(unsigned char *str) {
	do {
		while (!(USART1->SR & USART_SR_TXE));			//wait for the transmission buffer to empty
		USART1->DR = *str++;					//load the data into transmission buffer
	} while (*str);
}

//uart1 print a string + return
void serial1Println(unsigned char *str) {
	serial1Print(str);						//print the string
	serial1Print("\n\r");					//print the return
}

//test if uart1 is available
//return true if transmission on uart1 has completed
uint8_t serial1Available(void) {
	return (USART1->SR & USART_SR_TC)?true:false;
}
#endif								//usart1

#if defined(USE_UART2)
//initialize uart2
void serial2Begin(uint32_t baudrate) {
    //configure uart1
    //route clock to uart1
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    USART2->CR1 &=~(1<<13);			//'0'->disable uart, '1'->enable uart
    USART2->CR1 = 	(0<<15) |		//'0'->oversample by 16, '1'->oversample by 8
    				(0<<13) |		//'0'->disable uart, '1'->enable uart
    				(0<<12) |		//'0'->1 start bit, 8 data bit, n stop bit, '1'->1 start bit, 9 data bit, n stop bit
    				(0<<11) |		//'0'->idle line, '1'->wake by address
    				(0<<10) |		//'0'->disable parity, '1'->enable parity
    				(0<< 9) |		//'0'->even parity, '1'->odd parity
    				(0<< 8) |		//'0'->PE interrupt disabled, '1'->pe interrupt enabled
    				(0<< 7) |		//'0'->tx empty interrupt disabled, '1'-tx empty interrupt enabled
    				(0<< 6) |		//'0'->transmission completion interrupt disabled, '1'->transmission complete interrupt enabled
    				(0<< 5) |		//'0'->rx not empty interrupt disabled, '1'->rx not empty interrupt enabled
    				(0<< 4) |		//'0'->idle interrupt disabled, '1'->idle interrupt enabled
    				(1<< 3) |		//'0'->disable tx, '1'->enable tx
    				(1<< 2) |		//'0'->disable rx, '1'->enable rx
    				(0<< 1) |		//'0'->disable receiver wake-up, '1'->enable receiver wake-up
    				(0<< 0) |		//'0'->no break char is transmitted, '1'->break char will be transmitted
    				0x00;
    USART2->CR2 = 	(0<<14) |		//'0'->LIN mode disabled, '1'->LIN mode enabled
    				(0<<12) |		//0->1 stop bit, 1->0.5 stop bit, 2->2 stop bit, 3 -> 1.5 stop bit
    				(0<<11) |		//'0'->SCLK pin disabled, '1'->SCLK pin enabled
    				(0<<10) |		//'0'->SCLK idles low, '1'->SCLK idles high
    				(0<< 9) |		//'0'->first clock transition is first data capture edge, '1'->second clock transition is the first data capture edge
    				(0<< 8) |		//'0'->clock pulse of the last data bit is not output to the sclk pin, '1'->clock pulse of the last data bit is output to the sclk pin
    				(0<< 6) |		//'0'->LIN break detection interrupt disabled, '1'->LIN break detection interrupt enabled
    				(0<< 5) |		//'0'->LIN break detection is 10-bit, '1'->LIN break detection is 11 bit
    				(0<< 0) |		//address of the uart node
    				0x00;
    USART2->CR3 = 	(0<<11) |		//'0'->three sample bit, '1'->one sample bit
    				(0<<10) |		//'0'->CTS interrupt disabled, '1'->CTS interrupt enabled
    				(0<< 9) |		//'0'->CTS disabled, '1'->CTS enabled
    				(0<< 8) |		//'0'->RTS interrupt disabled, '1'->RTS interrupt enabled
    				(0<< 5) |		//'0'->smartcard mode disabled, '1'->smartcard mode enabled
    				(0<< 4) |		//'0'->smartcard nack disabled, '1'->smartcard nack enabled
    				(0<< 3) |		//'0'->half duplex mode not selected, '1'->half duplex mode selected
    				(0<< 2) |		//'0'->irda normal mode, '1'->irda low-power mode
    				(0<< 1) |		//'0'->irda disabled, '1'->irda enabled
    				(0<< 0) |		//'0'->error interrupt disabled, '1'->error interrupt enabled
    				0x00;
    //set the baudrate register
    USART2->BRR = SystemCoreClock / baudrate / 2 / ((USART2->CR1 & (1<<15))?1:2);		//per datasheet, for OVER8=0 or 1
    USART2->DR = 0;					//reset the data register
    USART2->SR = 0;					//reset the data register
    //enable uart1
    USART2->CR1 |= (1<<13);			//'0'->disable uart, '1'->enable uart

    //configure the RX-PA2/TX-PA3 pins - GPIO clock assumed enabled here previously
    GIO_INFL(GPIO_PinDef[PA2].gpio, GPIO_PinDef[PA2].mask);			//RX as floating input
    GIO_AFPP(GPIO_PinDef[PA3].gpio, GPIO_PinDef[PA3].mask);			//TX as AFPP

}

//uart1 send a char
void serial2Write(unsigned char dat) {
    USART2->DR = dat;                        	//load the data buffer
    //while (!(USART2->SR & (1<<6)));    		//wait for the transmission to complete
    while (!(USART2->SR & USART_SR_TXE));    			//wait for the transmission buffer to be empty
}

//uart1 returns a char
unsigned char serial2Read(void) {
    while (!(USART2->SR & USART_SR_RXNE));  			//wait for the receipt buffer to be empty
    return USART2->DR;                       	//save the transmission buffer
}

//uart1 print a string
void serial2Print(unsigned char *str) {
	do {
		while (!(USART2->SR & USART_SR_TXE));			//wait for the transmission buffer to empty
		USART2->DR = *str++;					//load the data into transmission buffer
	} while (*str);
}

//uart1 print a string + return
void serial2Println(unsigned char *str) {
	serial2Print(str);						//print the string
	serial2Print("\n\r");					//print the return
}

//test if uart1 is available
//return true if transmission on uart1 has completed
uint8_t serial2Available(void) {
	return (USART2->SR & USART_SR_TC)?true:false;
}
#endif										//usart2

#if defined(USE_UART3)
//initialize uart3
void serial3Begin(uint32_t baudrate) {
    //configure uart1
    //route clock to uart1
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

    USART3->CR1 &=~(1<<13);			//'0'->disable uart, '1'->enable uart
    USART3->CR1 = 	(0<<15) |		//'0'->oversample by 16, '1'->oversample by 8
    				(0<<13) |		//'0'->disable uart, '1'->enable uart
    				(0<<12) |		//'0'->1 start bit, 8 data bit, n stop bit, '1'->1 start bit, 9 data bit, n stop bit
    				(0<<11) |		//'0'->idle line, '1'->wake by address
    				(0<<10) |		//'0'->disable parity, '1'->enable parity
    				(0<< 9) |		//'0'->even parity, '1'->odd parity
    				(0<< 8) |		//'0'->PE interrupt disabled, '1'->pe interrupt enabled
    				(0<< 7) |		//'0'->tx empty interrupt disabled, '1'-tx empty interrupt enabled
    				(0<< 6) |		//'0'->transmission completion interrupt disabled, '1'->transmission complete interrupt enabled
    				(0<< 5) |		//'0'->rx not empty interrupt disabled, '1'->rx not empty interrupt enabled
    				(0<< 4) |		//'0'->idle interrupt disabled, '1'->idle interrupt enabled
    				(1<< 3) |		//'0'->disable tx, '1'->enable tx
    				(1<< 2) |		//'0'->disable rx, '1'->enable rx
    				(0<< 1) |		//'0'->disable receiver wake-up, '1'->enable receiver wake-up
    				(0<< 0) |		//'0'->no break char is transmitted, '1'->break char will be transmitted
    				0x00;
    USART3->CR2 = 	(0<<14) |		//'0'->LIN mode disabled, '1'->LIN mode enabled
    				(0<<12) |		//0->1 stop bit, 1->0.5 stop bit, 2->2 stop bit, 3 -> 1.5 stop bit
    				(0<<11) |		//'0'->SCLK pin disabled, '1'->SCLK pin enabled
    				(0<<10) |		//'0'->SCLK idles low, '1'->SCLK idles high
    				(0<< 9) |		//'0'->first clock transition is first data capture edge, '1'->second clock transition is the first data capture edge
    				(0<< 8) |		//'0'->clock pulse of the last data bit is not output to the sclk pin, '1'->clock pulse of the last data bit is output to the sclk pin
    				(0<< 6) |		//'0'->LIN break detection interrupt disabled, '1'->LIN break detection interrupt enabled
    				(0<< 5) |		//'0'->LIN break detection is 10-bit, '1'->LIN break detection is 11 bit
    				(0<< 0) |		//address of the uart node
    				0x00;
    USART3->CR3 = 	(0<<11) |		//'0'->three sample bit, '1'->one sample bit
    				(0<<10) |		//'0'->CTS interrupt disabled, '1'->CTS interrupt enabled
    				(0<< 9) |		//'0'->CTS disabled, '1'->CTS enabled
    				(0<< 8) |		//'0'->RTS interrupt disabled, '1'->RTS interrupt enabled
    				(0<< 5) |		//'0'->smartcard mode disabled, '1'->smartcard mode enabled
    				(0<< 4) |		//'0'->smartcard nack disabled, '1'->smartcard nack enabled
    				(0<< 3) |		//'0'->half duplex mode not selected, '1'->half duplex mode selected
    				(0<< 2) |		//'0'->irda normal mode, '1'->irda low-power mode
    				(0<< 1) |		//'0'->irda disabled, '1'->irda enabled
    				(0<< 0) |		//'0'->error interrupt disabled, '1'->error interrupt enabled
    				0x00;
    //set the baudrate register
    USART3->BRR = SystemCoreClock / baudrate / 2 / ((USART3->CR1 & (1<<15))?1:2);		//per datasheet, for OVER8=0 or 1
    USART3->DR = 0;					//reset the data register
    USART3->SR = 0;					//reset the data register
    //enable uart1
    USART3->CR1 |= (1<<13);			//'0'->disable uart, '1'->enable uart

    //configure the RX-PB11/TX-PB10 pins - GPIO clock assumed enabled here previously
    GIO_INFL(GPIO_PinDef[PB11].gpio, GPIO_PinDef[PB11].mask);		//RX as floating input
    GIO_AFPP(GPIO_PinDef[PB10].gpio, GPIO_PinDef[PB10].mask);		//TX as AFPP

}

//uart1 send a char
void serial3Write(unsigned char dat) {
    USART3->DR = dat;                        	//load the data buffer
    //while (!(USART3->SR & (1<<6)));    		//wait for the transmission to complete
    while (!(USART3->SR & USART_SR_TXE));    			//wait for the transmission buffer to be empty
}

//uart1 returns a char
unsigned char serial3Read(void) {
    while (!(USART3->SR & USART_SR_RXNE));  			//wait for the receipt buffer to be empty
    return USART3->DR;                       	//save the transmission buffer
}

//uart1 print a string
void serial3Print(unsigned char *str) {
	do {
		while (!(USART3->SR & USART_SR_TXE));			//wait for the transmission buffer to empty
		USART3->DR = *str++;					//load the data into transmission buffer
	} while (*str);
}

//uart1 print a string + return
void serial3Println(unsigned char *str) {
	serial3Print(str);						//print the string
	serial3Print("\n\r");					//print the return
}

//test if uart1 is available
//return true if transmission on uart1 has completed
uint8_t serial3Available(void) {
	return (USART3->SR & USART_SR_TC)?true:false;
}
#endif										//USART3

//initialize the chip
void chip_init(void) {
	//select the clock source
	//or use default clock

	//enable clock to GPIO
	/* Enable AHB clock to the GPIO domain. */
	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<31);		//31:enable gpio0
	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<30);		//30:enable gpio1
	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<29);		//29:enable gpio2

#if defined(TIM1) && defined(USE_PWM1)
	//configure TIM1 for PWM generation
	//enable clock to TIM1
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	TIM1->CR1 = (0x00<<8) |					//00->division ratio is 1
				(1<<7) |					//1->auto reload is buffered. 0->auto reload is not buffered
				(1<<5) |					//0->edge aligned, 1/2/3->center aligned mode 1/2/3
				(0<<4) |					//0->upcounter, 1->downcounter
				(0<<3) |					//0->continuous counting; 1->one pulse mode
				(1<<0);						//1->enable timer, 0->disable timer
	TIM1->CR2 = 0;							//default value
	TIM1->CNT = 0;							//reset counter
	TIM1->PSC = (PWMOUT_PS) - 1;			//16-bit prescaler. 1->prescaler is 1:1
	TIM1->ARR = PWM_PR;						//auto reload value, 16-bit
	TIM1->CR1|= (1<<0);						//start the timer

	//set up OC1/2/3/4 channels
	TIM1->CCER = 0;							//disable all oc channels
	TIM1->CCMR1= (0<<15) |					//0->OC2 output clear disabled
				 (6<<12) |					//0b110: output compare mode 2 - 3-bit field
				 (0<<11) |					//0->disable oc preload
				 (0<<10) |					//0->disable oc fast
				 (0<<8) |					//0->oc2 as output
				 (0<<7) |					//0->OC1 output clear disabled
				 (6<<4) |					//0b110: output compare mode 2 - 3-bit field
				 (0<<3) |					//0->disable oc preload
				 (0<<2) |					//0->disable oc fast
				 (0<<0) |					//0->oc1 as output
				 0x00;
	TIM1->CCMR2 = TIM1->CCMR1;				//replicate ccmr1 to ccmr2
	TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = TIM3->CCR4 = 0;
#endif

#if defined(TIM2) && defined(USE_PWM2)
	//configure TIM2 for PWM generation
	//enable clock to TIM2
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	TIM2->CR1 = (0x00<<8) |					//00->division ratio is 1
				(1<<7) |					//1->auto reload is buffered. 0->auto reload is not buffered
				(1<<5) |					//0->edge aligned, 1/2/3->center aligned mode 1/2/3
				(0<<4) |					//0->upcounter, 1->downcounter
				(0<<3) |					//0->continuous counting; 1->one pulse mode
				(1<<0);						//1->enable timer, 0->disable timer
	TIM2->CR2 = 0;							//default value
	TIM2->CNT = 0;							//reset counter
	TIM2->PSC = (PWMOUT_PS) - 1;			//16-bit prescaler. 1->prescaler is 1:1
	TIM2->ARR = PWM_PR;						//auto reload value, 16-bit
	TIM2->CR1|= (1<<0);						//start the timer

	//set up OC1/2/3/4 channels
	TIM2->CCER = 0;							//disable all oc channels
	TIM2->CCMR1= (0<<15) |					//0->OC2 output clear disabled
				 (6<<12) |					//0b110: output compare mode 2 - 3-bit field
				 (0<<11) |					//0->disable oc preload
				 (0<<10) |					//0->disable oc fast
				 (0<<8) |					//0->oc2 as output
				 (0<<7) |					//0->OC1 output clear disabled
				 (6<<4) |					//0b110: output compare mode 2 - 3-bit field
				 (0<<3) |					//0->disable oc preload
				 (0<<2) |					//0->disable oc fast
				 (0<<0) |					//0->oc1 as output
				 0x00;
	TIM2->CCMR2 = TIM2->CCMR1;				//replicate ccmr1 to ccmr2
	TIM2->CCR1 = TIM2->CCR2 = TIM2->CCR3 = TIM3->CCR4 = 0;
#endif

#if defined(TIM3) && defined(USE_PWM3)
	//configure TIM3 for PWM generation
	//enable clock to TIM3
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	TIM3->CR1 = (0x00<<8) |					//00->division ratio is 1
				(1<<7) |					//1->auto reload is buffered. 0->auto reload is not buffered
				(1<<5) |					//0->edge aligned, 1/2/3->center aligned mode 1/2/3
				(0<<4) |					//0->upcounter, 1->downcounter
				(0<<3) |					//0->continuous counting; 1->one pulse mode
				(1<<0);						//1->enable timer, 0->disable timer
	TIM3->CR2 = 0;							//default value
	TIM3->CNT = 0;							//reset counter
	TIM3->PSC = (PWMOUT_PS) - 1;			//16-bit prescaler. 1->prescaler is 1:1
	TIM3->ARR = PWM_PR;						//auto reload value, 16-bit
	TIM3->CR1|= (1<<0);						//start the timer

	//set up OC1/2/3/4 channels
	TIM3->CCER = 0;							//disable all oc channels
	TIM3->CCMR1= (0<<15) |					//0->OC2 output clear disabled
				 (6<<12) |					//0b110: output compare mode 2 - 3-bit field
				 (0<<11) |					//0->disable oc preload
				 (0<<10) |					//0->disable oc fast
				 (0<<8) |					//0->oc2 as output
				 (0<<7) |					//0->OC1 output clear disabled
				 (6<<4) |					//0b110: output compare mode 2 - 3-bit field
				 (0<<3) |					//0->disable oc preload
				 (0<<2) |					//0->disable oc fast
				 (0<<0) |					//0->oc1 as output
				 0x00;
	TIM3->CCMR2 = TIM3->CCMR1;				//replicate ccmr1 to ccmr2
	TIM3->CCR1 = TIM3->CCR2 = TIM3->CCR3 = TIM3->CCR4 = 0;
#endif

#if defined(TIM4) && defined(USE_PWM4)
	//configure TIM4 for PWM generation
	//enable clock to TIM4
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	TIM4->CR1 = (0x00<<8) |					//00->division ratio is 1
				(1<<7) |					//1->auto reload is buffered. 0->auto reload is not buffered
				(1<<5) |					//0->edge aligned, 1/2/3->center aligned mode 1/2/3
				(0<<4) |					//0->upcounter, 1->downcounter
				(0<<3) |					//0->continuous counting; 1->one pulse mode
				(1<<0);						//1->enable timer, 0->disable timer
	TIM4->CR2 = 0;							//default value
	TIM4->CNT = 0;							//reset counter
	TIM4->PSC = (PWMOUT_PS) - 1;			//16-bit prescaler. 1->prescaler is 1:1
	TIM4->ARR = PWM_PR;						//auto reload value, 16-bit
	TIM4->CR1|= (1<<0);						//start the timer

	//set up OC1/2/3/4 channels
	TIM4->CCER = 0;							//disable all oc channels
	TIM4->CCMR1= (0<<15) |					//0->OC2 output clear disabled
				 (6<<12) |					//0b110: output compare mode 2 - 3-bit field
				 (0<<11) |					//0->disable oc preload
				 (0<<10) |					//0->disable oc fast
				 (0<<8) |					//0->oc2 as output
				 (0<<7) |					//0->OC1 output clear disabled
				 (6<<4) |					//0b110: output compare mode 2 - 3-bit field
				 (0<<3) |					//0->disable oc preload
				 (0<<2) |					//0->disable oc fast
				 (0<<0) |					//0->oc1 as output
				 0x00;
	TIM4->CCMR2 = TIM4->CCMR1;				//replicate ccmr1 to ccmr2
	TIM4->CCR1 = TIM4->CCR2 = TIM4->CCR3 = TIM3->CCR4 = 0;
#endif

#if defined(USE_ADC1)
	//initialize adc
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;		//enable clock to ADC1
	ADC1->CR1 = (0<<23) |					//0->disable analog watchdog on regular channels
				(0<<22) |					//0->disable analog watchdog on injected channels
				(0<<13) |					//0->1 ch for discontinous conversion
				(0<<12) |					//0->discontinous mode for injected channels disabled
				(0<<11) |					//0->discontinous mode for regular channel disabled
				(0<<10) |					//0->automatic injected conversion disabled
				(0<< 9) |					//0->analog watchdog enabled on all channels
				(0<< 8) |					//0->scan mode disabled
				(0<< 7) |					//0->EOC interrupt on injected conversion disabled
				(0<< 6) |					//0->interrupt on analog watchdog disabled
				(0<< 5) |					//0->EOC interrupt on regular channels disabled
				(17<<0) |					//ADC channels, 0..17 (16=Temperature sensor, 17 = Vrefint)
				0x00;
	ADC1->CR2 = (1<<23) |					//0->temperature sensor disabled
				(0<<22) |					//1->start conversion on regular channel. 0->reset state. cleared by software
				(0<<21) |					//1->start conversion on injected channels. 0->reset state. cleared by s oftware
				(1<<20) |					//0->regular conversion on external event disabled
				(7<<17) |					//7->regular conversion triggered by software
				(0<<15) |					//0->injected conversion on external event disabled
				(7<<12) |					//7->injected conversion triggered by software
				(0<<11) |					//0->right aligned, 1->left aligned
				(0<< 3) |					//0->caliberatin stablized, 1->caliberatin initialized
				(0<< 2) |					//0->calibration complete, 1->initialize calibration
				(0<< 1) |					//0->single conversion mode, 1->continous conversion mode
				(1<< 0);					//1->adc1 on, 0->adc1 off
	//set adc sample time
	//0b111->239.5 cycles for all channels
	ADC1->SMPR1 = 	(ADC_SMPR << (3 * 7)) |
					(ADC_SMPR << (3 * 6)) |
					(ADC_SMPR << (3 * 5)) |
					(ADC_SMPR << (3 * 4)) |
					(ADC_SMPR << (3 * 3)) |
					(ADC_SMPR << (3 * 2)) |
					(ADC_SMPR << (3 * 1)) |
					(ADC_SMPR << (3 * 0)) |
					0x00;
	ADC1->SMPR2 = 	(ADC_SMPR << (3 * 9)) |
					(ADC_SMPR << (3 * 8)) |
					(ADC_SMPR << (3 * 7)) |
					(ADC_SMPR << (3 * 6)) |
					(ADC_SMPR << (3 * 5)) |
					(ADC_SMPR << (3 * 4)) |
					(ADC_SMPR << (3 * 3)) |
					(ADC_SMPR << (3 * 2)) |
					(ADC_SMPR << (3 * 1)) |
					(ADC_SMPR << (3 * 0)) |
					0x00;
	//set adc channel sequence
	ADC1->SQR3 = ADC1->SQR2 = ADC1->SQR1 = 0;							//0->1 conversion

	//optional: adc self-calibration
	//start caliberation
	ADC1->CR2 |= (1<<3);					//1->initialize calibration registers
	while ((ADC1->CR2) & (1<<3)) continue;	//wait for calibration to initialize
	ADC1->CR2 |= (1<<2);					//1->initialize calibration
	while ((ADC1->CR2) & (1<<2)) continue;	//wait for calibration to finish
	//now adc is calibrated
#endif

#if defined(USE_DAC)
	//initialize the DAC
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;		//enable clock to DAC
	DAC->CR = 	(0<<29) |					//0=DAC2 under-run interrupt disabled
				(0<<28) |					//0->DAC2 DMA disabled
				(0<<24) |					//0->unmask = 1
				(0<<22) |					//0->wave generation disabled
				(7<<19) |					//7->triggered by software
				(1<<18) |					//1->trigge enabled for DACn
				(0<<17) |					//0->output buffered ***enabled***
				(1<<16) |					//0->disable DACn, 1->enable DACn
				(0<<14) |					//0=DAC2 under-run interrupt disabled
				(0<<13) |					//0->DAC2 DMA disabled
				(0<< 8) |					//0->unmask = 1
				(0<< 6) |					//0->wave generation disabled
				(7<< 3) |					//7->triggered by software
				(1<< 2) |					//1->trigge enabled for DACn
				(0<< 1) |					//0->output buffered ***enabled***
				(1<< 0) |					//0->disable DACn, 1->enable DACn
				0x00;
#endif

#if defined(USE_EXTI)
	//initialize exti
	//route clock to afio
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;		//enable clock to DAC
    AFIO->EXTICR[0] = AFIO->EXTICR[1] = AFIO->EXTICR[2] = AFIO->EXTICR[3] = 0;
    //clear all the flags
    EXTI->PR = 0xfffffffful;        //clear the flags by writing 1 to it
    //clear IMR
    EXTI->IMR = 0;                  //disable all IMR
#endif

#if defined(USE_SPI1)
    //configure spi1
    //route clock to spi1
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    //disable spi module before configuring it
    SPI1->CR1 &=~(1<<6);			//'1'->enable spi, '0'->disable spi

    SPI1->CR1 = (0<<15) |			//'0'->2-line unidirectional enabled, '1'->1 line bidirectional enabled
    			(0<<14) |			//'0'->output disabled in bidirectional mode, '1'->output enabled in bidirectional mode
    			(0<<13) |			//'0'->disable crc calculation, '1'->enable crc calculation
    			(0<<12) |			//'0'->next transmission if from tx buffer, '1'->next transmission is from crc
    			(0<<11) |			//'0'->8-bit per frame selected, '1'->16-bit per frame selected
    			(0<<10) |			//'0'->full duplex (rx and tx), '1'->rx only (tx disabled)
    			(0<< 9) |			//'0'->disable software slave management, '1'->enable software slave management
    			(0<< 8) |			//'0'->software slave select -> the state is forced to NSS pin if bit 9 is set
    			(0<< 7) |			//'0'->msb first, '1'->lsb first
    			(0<< 6) |			//'0'->spi disabled, '1'->spi enabled
    			(SPI1_PS<< 3) |		//3-bit value to select spi baud rate. 0->2x,..., 7->256
    			(1<< 2) |			//'0'->slave mode, '1'->master mode
    			(0<< 1) |			//'0'->SCK idles at 0, '1'->SCK idles at 1
    			(0<< 0) |			//'0'->first clock transition is the first data capture, '1'->2nd clock transition is the first data capture
    			0x00;
    SPI1->CR2 = (0<<7) |			//'0'->tx interrupt disabled, '1'->tx interrupt enabled
    			(0<<6) |			//'0'->rx buffer not empty interrupt disabled, '1'->rx buffer not empty interrupt enabled
    			(0<<5) |			//'0'->error interrupt disabled, '1'->error interrupt enabled
    			(0<<3) |			//'0'->slave select output disabled, '1'->slave select output enabled
    			(0<<1) |			//'0'->tx dma disabled, '1'->tx dma enabled
    			(0<<0) |			//'0'->rx dma disabled, '1'->rx dma enabled
    			0x00;
    SPI1->DR = 0x00;				//clear the buffer
    SPI1->SR = 0x00;				//clear the status register
    SPI1->CR1 |= (1<<6);			//enable spi1

    //configure output pins
#endif

#if defined(USE_SPI2)
    //configure SPI2
    //route clock to SPI2
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    //disable spi module before configuring it
    SPI2->CR1 &=~(1<<6);			//'1'->enable spi, '0'->disable spi

    SPI2->CR1 = (0<<15) |			//'0'->2-line unidirectional enabled, '1'->1 line bidirectional enabled
    			(0<<14) |			//'0'->output disabled in bidirectional mode, '1'->output enabled in bidirectional mode
    			(0<<13) |			//'0'->disable crc calculation, '1'->enable crc calculation
    			(0<<12) |			//'0'->next transmission if from tx buffer, '1'->next transmission is from crc
    			(0<<11) |			//'0'->8-bit per frame selected, '1'->16-bit per frame selected
    			(0<<10) |			//'0'->full duplex (rx and tx), '1'->rx only (tx disabled)
    			(0<< 9) |			//'0'->disable software slave management, '1'->enable software slave management
    			(0<< 8) |			//'0'->software slave select -> the state is forced to NSS pin if bit 9 is set
    			(0<< 7) |			//'0'->msb first, '1'->lsb first
    			(0<< 6) |			//'0'->spi disabled, '1'->spi enabled
    			(SPI2_PS<< 3) |		//3-bit value to select spi baud rate. 0->2x,..., 7->256
    			(1<< 2) |			//'0'->slave mode, '1'->master mode
    			(0<< 1) |			//'0'->SCK idles at 0, '1'->SCK idles at 1
    			(0<< 0) |			//'0'->first clock transition is the first data capture, '1'->2nd clock transition is the first data capture
    			0x00;
    SPI2->CR2 = (0<<7) |			//'0'->tx interrupt disabled, '1'->tx interrupt enabled
    			(0<<6) |			//'0'->rx buffer not empty interrupt disabled, '1'->rx buffer not empty interrupt enabled
    			(0<<5) |			//'0'->error interrupt disabled, '1'->error interrupt enabled
    			(0<<3) |			//'0'->slave select output disabled, '1'->slave select output enabled
    			(0<<1) |			//'0'->tx dma disabled, '1'->tx dma enabled
    			(0<<0) |			//'0'->rx dma disabled, '1'->rx dma enabled
    			0x00;
    SPI2->DR = 0x00;				//clear the buffer
    SPI2->SR = 0x00;				//clear the status register
    SPI2->CR1 |= (1<<6);			//enable SPI2

    //configure output pins
#endif

#if defined(USE_SPI3)
    //configure SPI3
    //route clock to SPI3
    RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;

    //disable spi module before configuring it
    SPI3->CR1 &=~(1<<6);			//'1'->enable spi, '0'->disable spi

    SPI3->CR1 = (0<<15) |			//'0'->2-line unidirectional enabled, '1'->1 line bidirectional enabled
    			(0<<14) |			//'0'->output disabled in bidirectional mode, '1'->output enabled in bidirectional mode
    			(0<<13) |			//'0'->disable crc calculation, '1'->enable crc calculation
    			(0<<12) |			//'0'->next transmission if from tx buffer, '1'->next transmission is from crc
    			(0<<11) |			//'0'->8-bit per frame selected, '1'->16-bit per frame selected
    			(0<<10) |			//'0'->full duplex (rx and tx), '1'->rx only (tx disabled)
    			(0<< 9) |			//'0'->disable software slave management, '1'->enable software slave management
    			(0<< 8) |			//'0'->software slave select -> the state is forced to NSS pin if bit 9 is set
    			(0<< 7) |			//'0'->msb first, '1'->lsb first
    			(0<< 6) |			//'0'->spi disabled, '1'->spi enabled
    			(SPI3_PS<< 3) |		//3-bit value to select spi baud rate. 0->2x,..., 7->256
    			(1<< 2) |			//'0'->slave mode, '1'->master mode
    			(0<< 1) |			//'0'->SCK idles at 0, '1'->SCK idles at 1
    			(0<< 0) |			//'0'->first clock transition is the first data capture, '1'->2nd clock transition is the first data capture
    			0x00;
    SPI3->CR2 = (0<<7) |			//'0'->tx interrupt disabled, '1'->tx interrupt enabled
    			(0<<6) |			//'0'->rx buffer not empty interrupt disabled, '1'->rx buffer not empty interrupt enabled
    			(0<<5) |			//'0'->error interrupt disabled, '1'->error interrupt enabled
    			(0<<3) |			//'0'->slave select output disabled, '1'->slave select output enabled
    			(0<<1) |			//'0'->tx dma disabled, '1'->tx dma enabled
    			(0<<0) |			//'0'->rx dma disabled, '1'->rx dma enabled
    			0x00;
    SPI3->DR = 0x00;				//clear the buffer
    SPI3->SR = 0x00;				//clear the status register
    SPI3->CR1 |= (1<<6);			//enable SPI3

    //configure output pins
#endif

#if defined(USE_I2C)
    //configure i2c
#endif

	//update SystemCoreClock - done in mcu_init()
	//SystemCoreClockUpdate();

}


