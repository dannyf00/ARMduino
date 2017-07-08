//hardware specific port for ARMduino to STM32F10x family

#include "armduino_stm32f0xx.h"						//we use aruidno GPIO port for stm32F
#include "armduino.h"								//use ARMduino specific definitions

//global definitions

#define PWM_PR			((1ul<<PWMOUT_BITs) - 1)			//period for 16-bit pwm

//struct used to map a pin to GPIO+mask
typedef struct {
	GPIO_TypeDef *gpio;					//gpio for a pin
	uint16_t mask;						//pin mask - 16bit
} PIN2GPIO;

//empty handler
static void empty_handler(void) {
	//do nothing here
}

//user isr
void (*_isrptr_exti[16])(void);			//user handler for EXTI interrupt

//exti interrupt handlers
void EXTI0_1_IRQHandler(void) {
	if (EXTI->PR & (1<<0)) {
		EXTI->PR |= (1<<0);				//clear the flag by writing '1' to it
		_isrptr_exti[0]();				//execute the user handler
	}

	if (EXTI->PR & (1<<1)) {
		EXTI->PR |= (1<<1);				//clear the flag by writing '1' to it
		_isrptr_exti[1]();				//execute the user handler
	}
}

void EXTI2_3_IRQHandler(void) {
	if (EXTI->PR & (1<<2)) {
		EXTI->PR |= (1<<2);				//clear the flag by writing '1' to it
		_isrptr_exti[2]();				//execute the user handler
	}
	if (EXTI->PR & (1<<3)) {
		EXTI->PR |= (1<<3);				//clear the flag by writing '1' to it
		_isrptr_exti[3]();				//execute the user handler
	}
}

void EXTI4_15IRQHandler(void) {
	if (EXTI->PR & (1<<4)) {
		EXTI->PR |= (1<<4);				//clear the flag by writing '1' to it
		_isrptr_exti[4]();				//execute the user handler
	}

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

//timer handlers
#if defined(USE_TIM1)
//global variables
static void (* _tim1_oc1isrptr)(void)=empty_handler;				//tim1_ptr pointing to empty_handler by default
static void (* _tim1_oc2isrptr)(void)=empty_handler;				//tim1_ptr pointing to empty_handler by default
static void (* _tim1_oc3isrptr)(void)=empty_handler;				//tim1_ptr pointing to empty_handler by default
static void (* _tim1_oc4isrptr)(void)=empty_handler;				//tim1_ptr pointing to empty_handler by default

static uint32_t _tim1_oc1=0;				//output compare registers
static uint32_t _tim1_oc2=0;
static uint32_t _tim1_oc3=0;
static uint32_t _tim1_oc4=0;

//isr for timer1 capture / compare
void TIM1_CC_IRQHandler(void) {
	//oc1 portion
	if (TIM1->SR & TIM_SR_CC1IF) {		//output compare 1 flag is set
		TIM1->SR &=~TIM_SR_CC1IF;		//clear the flag
		TIM1->CCR1 += _tim1_oc1;			//update the output compare register
		_tim1_oc1isrptr();				//execute user handler
	}

	//oc2 portion
	if (TIM1->SR & TIM_SR_CC2IF) {		//output compare 2 flag is set
		TIM1->SR &=~TIM_SR_CC2IF;		//clear the flag
		TIM1->CCR2 += _tim1_oc2;			//update the output compare register
		_tim1_oc2isrptr();				//execute user handler
	}

	//oc3 portion
	if (TIM1->SR & TIM_SR_CC3IF) {		//output compare 2 flag is set
		TIM1->SR &=~TIM_SR_CC3IF;		//clear the flag
		TIM1->CCR3 += _tim1_oc3;			//update the output compare register
		_tim1_oc3isrptr();				//execute user handler
	}

	//oc4 portion
	if (TIM1->SR & TIM_SR_CC4IF) {		//output compare 2 flag is set
		TIM1->SR &=~TIM_SR_CC4IF;		//clear the flag
		TIM1->CCR4 += _tim1_oc4;			//update the output compare register
		_tim1_oc4isrptr();				//execute user handler
	}
}
#endif
//TIM2 not present on all chips
#if defined(USE_TIM2)
//global variables
static void (* _tim2_oc1isrptr)(void)=empty_handler;				//tim2_ptr pointing to empty_handler by default
static void (* _tim2_oc2isrptr)(void)=empty_handler;				//tim2_ptr pointing to empty_handler by default
static void (* _tim2_oc3isrptr)(void)=empty_handler;				//tim2_ptr pointing to empty_handler by default
static void (* _tim2_oc4isrptr)(void)=empty_handler;				//tim2_ptr pointing to empty_handler by default

static uint32_t _tim2_oc1=0;				//output compare registers
static uint32_t _tim2_oc2=0;
static uint32_t _tim2_oc3=0;
static uint32_t _tim2_oc4=0;

//isr for timer1 capture / compare
void TIM2_IRQHandler(void) {
	//oc1 portion
	if (TIM2->SR & TIM_SR_CC1IF) {		//output compare 1 flag is set
		TIM2->SR &=~TIM_SR_CC1IF;		//clear the flag
		TIM2->CCR1 += _tim2_oc1;			//update the output compare register
		_tim2_oc1isrptr();				//execute user handler
	}

	//oc2 portion
	if (TIM2->SR & TIM_SR_CC2IF) {		//output compare 2 flag is set
		TIM2->SR &=~TIM_SR_CC2IF;		//clear the flag
		TIM2->CCR2 += _tim2_oc2;			//update the output compare register
		_tim2_oc2isrptr();				//execute user handler
	}

	//oc3 portion
	if (TIM2->SR & TIM_SR_CC3IF) {		//output compare 2 flag is set
		TIM2->SR &=~TIM_SR_CC3IF;		//clear the flag
		TIM2->CCR3 += _tim2_oc3;			//update the output compare register
		_tim2_oc3isrptr();				//execute user handler
	}

	//oc4 portion
	if (TIM2->SR & TIM_SR_CC4IF) {		//output compare 2 flag is set
		TIM2->SR &=~TIM_SR_CC4IF;		//clear the flag
		TIM2->CCR4 += _tim2_oc4;			//update the output compare register
		_tim2_oc4isrptr();				//execute user handler
	}
}
#endif

#if defined(USE_TIM3)
//global variables
static void (* _tim3_oc1isrptr)(void)=empty_handler;				//tim3_ptr pointing to empty_handler by default
static void (* _tim3_oc2isrptr)(void)=empty_handler;				//tim3_ptr pointing to empty_handler by default
static void (* _tim3_oc3isrptr)(void)=empty_handler;				//tim3_ptr pointing to empty_handler by default
static void (* _tim3_oc4isrptr)(void)=empty_handler;				//tim3_ptr pointing to empty_handler by default

static uint32_t _tim3_oc1=0;				//output compare registers
static uint32_t _tim3_oc2=0;
static uint32_t _tim3_oc3=0;
static uint32_t _tim3_oc4=0;

//isr for timer1 capture / compare
void TIM3_IRQHandler(void) {
	//oc1 portion
	if (TIM3->SR & TIM_SR_CC1IF) {		//output compare 1 flag is set
		TIM3->SR &=~TIM_SR_CC1IF;		//clear the flag
		TIM3->CCR1 += _tim3_oc1;			//update the output compare register
		_tim3_oc1isrptr();				//execute user handler
	}

	//oc2 portion
	if (TIM3->SR & TIM_SR_CC2IF) {		//output compare 2 flag is set
		TIM3->SR &=~TIM_SR_CC2IF;		//clear the flag
		TIM3->CCR2 += _tim3_oc2;			//update the output compare register
		_tim3_oc2isrptr();				//execute user handler
	}

	//oc3 portion
	if (TIM3->SR & TIM_SR_CC3IF) {		//output compare 2 flag is set
		TIM3->SR &=~TIM_SR_CC3IF;		//clear the flag
		TIM3->CCR3 += _tim3_oc3;			//update the output compare register
		_tim3_oc3isrptr();				//execute user handler
	}

	//oc4 portion
	if (TIM3->SR & TIM_SR_CC4IF) {		//output compare 2 flag is set
		TIM3->SR &=~TIM_SR_CC4IF;		//clear the flag
		TIM3->CCR4 += _tim3_oc4;			//update the output compare register
		_tim3_oc4isrptr();				//execute user handler
	}
}
#endif

#if defined(USE_TIM14)
//global variables
static void (* _tim14_isrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default

//isr for timer1 capture / compare
void TIM14_IRQHandler(void) {
	TIM14->SR &=~TIM_SR_UIF;		//clear the flag
	_tim14_isrptr();				//execute user handler
}
#endif

//TIM15 not present on all chips
#if defined(USE_TIM15)
//global variables
static void (* _tim15_isrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default

//isr for timer1 capture / compare
void TIM15_IRQHandler(void) {
	TIM15->SR &=~TIM_SR_UIF;		//clear the flag
	_tim15_isrptr();				//execute user handler
}
#endif

#if defined(USE_TIM16)
//global variables
static void (* _tim16_isrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default

//isr for timer1 capture / compare
void TIM16_IRQHandler(void) {
	TIM16->SR &=~TIM_SR_UIF;		//clear the flag
	_tim16_isrptr();				//execute user handler
}
#endif

#if defined(USE_TIM17)
//global variables
static void (* _tim17_isrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default

//isr for timer1 capture / compare
void TIM17_IRQHandler(void) {
	TIM17->SR &=~TIM_SR_UIF;		//clear the flag
	_tim17_isrptr();				//execute user handler
}
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
	
#if defined(RCC_AHBENR_GPIOFEN)			//GPIOF not present on all chips
	{GPIOF, 1<< 0},						//ARMduino Pin 40 = PF0
	{GPIOF, 1<< 1},						//ARMduino Pin 41 = PF1
	{GPIOF, 1<< 2},						//ARMduino Pin 42 = PF2
	{GPIOF, 1<< 3},						//ARMduino Pin 43 = PF3
	{GPIOF, 1<< 4},						//ARMduino Pin 44 = PF4
	{GPIOF, 1<< 5},						//ARMduino Pin 45 = PF5
	{GPIOF, 1<< 6},						//ARMduino Pin 46 = PF6
	{GPIOF, 1<< 7},						//ARMduino Pin 47 = PF7
	{GPIOF, 1<< 8},						//ARMduino Pin 40 = PF8
	{GPIOF, 1<< 9},						//ARMduino Pin 41 = PF9
	{GPIOF, 1<<10},						//ARMduino Pin 42 = PF10
	{GPIOF, 1<<11},						//ARMduino Pin 43 = PF11
	{GPIOF, 1<<12},						//ARMduino Pin 44 = PF12
	{GPIOF, 1<<13},						//ARMduino Pin 45 = PF13
	{GPIOF, 1<<14},						//ARMduino Pin 46 = PF14
	{GPIOF, 1<<15},						//ARMduino Pin 47 = PF15
#endif

#if defined(RCC_AHBENR_GPIOGEN)						//GPIOG not present on all chips
	{GPIOG, 1<< 0},						//ARMduino Pin 48 = PG0
	{GPIOG, 1<< 1},						//ARMduino Pin 49 = PG1
	{GPIOG, 1<< 2},						//ARMduino Pin 50 = PG2
	{GPIOG, 1<< 3},						//ARMduino Pin 51 = PG3
	{GPIOG, 1<< 4},						//ARMduino Pin 52 = PG4
	{GPIOG, 1<< 5},						//ARMduino Pin 53 = PG5
	{GPIOG, 1<< 6},						//ARMduino Pin 54 = PG6
	{GPIOG, 1<< 7},						//ARMduino Pin 55 = PG7
	{GPIOG, 1<< 8},						//ARMduino Pin 48 = PG8
	{GPIOG, 1<< 9},						//ARMduino Pin 49 = PG9
	{GPIOG, 1<<10},						//ARMduino Pin 50 = PG10
	{GPIOG, 1<<11},						//ARMduino Pin 51 = PG11
	{GPIOG, 1<<12},						//ARMduino Pin 52 = PG12
	{GPIOG, 1<<13},						//ARMduino Pin 53 = PG13
	{GPIOG, 1<<14},						//ARMduino Pin 54 = PG14
	{GPIOG, 1<<15},						//ARMduino Pin 55 = PG15
#endif
};

//configure gpio DDR mode (cnf1..0 + mod1..0 bits)
void GPIO_DDR(GPIO_TypeDef * GPIOx, uint32_t mask, uint32_t mode) {
	uint8_t pos;
	//uint32_t pin_mask;

	for (pos=0; pos < 16; pos++) {
		//looking for pin position / mask
		//pin_mask = 1ul << pos;
		if (mask & (1ul << pos)) {
			//we have found the pos / pin_mask
			if ((mode & GPIOMODE_OUTPUT) || (mode & GPIOMODE_AF)) {
				GPIOx->OSPEEDR &=~(0x03ul << (2 * pos));	//clear ospeeder
				GPIOx->OSPEEDR |= (0x01ul << (2 * pos));	//set to medium speed (0x01)

				GPIOx->OTYPER &=~(1ul << pos);				//clear otyper
				GPIOx->OTYPER |= ((mode & GPIOMODE_PP)?0ul:1ul) << pos;	//0->pp, 1->od
			}

			GPIOx->MODER &=~(0x03 << (2 * pos));			//clear moder
			GPIOx->MODER |= (mode & 0x03) << (2 * pos);		//set moder

			GPIOx->PUPDR &=~(0x03 << (2 * pos));			//clear pupdr
			GPIOx->PUPDR |= ((mode >> 4) & 0x03) << (2 * pos);	//set pupdr
		}
	}
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
	case INPUT: 		GPIO_DDR(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask, GPIOMODE_INFL); break;			//floating input
	case INPUT_PULLUP: 	GPIO_DDR(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask, GPIOMODE_INPU); /*IO_SET(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask); */break;			//input with pullup
	case INPUT_PULLDN:	GPIO_DDR(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask, GPIOMODE_INPD); /*IO_CLR(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask); */break;			//input with pulldown
	case INPUT_ANALOG:	GPIO_DDR(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask, GPIOMODE_INAN); break;			//analog input
	case OUTPUT:		GPIO_DDR(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask, GPIOMODE_OUTPP); break;			//floating input
	case OUTPUT_OD:		GPIO_DDR(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask, GPIOMODE_OUTOD); break;			//floating input
	case OUTPUT_AFPP:	GPIO_DDR(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask, GPIOMODE_AFPP); break;			//floating input
	case OUTPUT_AFOD:	GPIO_DDR(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask, GPIOMODE_AFOD); break;			//floating input
	default: break;			//do nothing
	}

}

//set / clear a pin
inline void digitalWrite(PIN_TypeDef pin, uint8_t val) {
	if (val==LOW) IO_CLR(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);
	else IO_SET(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);
}

//read a pin
inline int digitalRead(PIN_TypeDef pin) {
	return (IO_GET(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask))?HIGH:LOW;
}
//end GPIO

#if defined(USE_PWM)
//analogWrite
void analogWrite(PIN_TypeDef pin, uint16_t dc) {
	dc=(dc>PWM_PR)?PWM_PR:dc;				//bound dc to [0..PWM_PR]
	switch (pin) {							//configure TIM2-OCi channels based on pin # -> for STM32F100
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

#if defined(USE_ADC1)
//analog to digital converter on ADC1
//ain/analog input channel: ain valid values: 0..15, 16=temperature sensor, 17=Vrefint
//***user needs to be configured as floating input***
uint16_t analogRead(AIN_TypeDef ain) {
	ADC1->ISR &= ~(1<<2);					//clear the eoc flag
	//ADC1->CR1 = (ADC1->CR1 &~0x1f) | (ain & 0x1f);	//pick the adc channel
	//ADC1->CR2|= (1<<0);					//start the conversion
	ADC1->CHSELR = (1ul<<ain) & 0x03fffful;		//define the first (and only) adc ch
	ADC1->CR |= (1<<2);						//start conversion
	while ((ADC1->ISR & (1<<2)) == 0) continue;	//wait for conversion to end (EOC set at end of conversion)
	return ADC1->DR;						//return adc result and clears the EOC bit
}
#endif										//adc1

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
    //if (gpio == GPIOE) mask = 0x04;
    if (gpio == GPIOF) mask = 0x05;
    //if (gpio == GPIOG) mask = 0x06;

    //scan for pin position
    for (pos = 0; pos < 16; pos ++)
        if (GPIO_PinDef[pin].mask & (1<<pos)) break;
    //not pos has the pin position

    //figure out the line 0..15
         if (pos < 4) SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] &~(0x0f << (4*(pos- 0)))) | (mask << (4*(pos - 0)));  //set the line 0..4
    else if (pos < 8) SYSCFG->EXTICR[1] = (SYSCFG->EXTICR[1] &~(0x0f << (4*(pos- 4)))) | (mask << (4*(pos - 4)));  //set the line 5..8
    else if (pos <12) SYSCFG->EXTICR[2] = (SYSCFG->EXTICR[2] &~(0x0f << (4*(pos- 8)))) | (mask << (4*(pos - 8)));  //set the line 9..12
    else if (pos <16) SYSCFG->EXTICR[3] = (SYSCFG->EXTICR[3] &~(0x0f << (4*(pos-12)))) | (mask << (4*(pos -12)));  //set the line 13..16

    //figure out the trigger: rising or falling
    switch (mode) {
    case CHANGE:	EXTI->FTSR |= (1<<pos); EXTI->RTSR |= (1<<pos); break;		//set falling trigger
    case RISING: 	EXTI->FTSR &=~(1<<pos); EXTI->RTSR |= (1<<pos); break;		//set rising trigger
    case FALLING:
    default: 		EXTI->FTSR |= (1<<pos); EXTI->RTSR &=~(1<<pos); break;		//set falling trigger (default)
    }

    //install the user handler
	_isrptr_exti[pos] = isrptr;
	//set it to the lowest priority
    switch (pos) {
    case 0:
    case 1: NVIC_SetPriority(EXTI0_1_IRQn, 15); NVIC_EnableIRQ(EXTI0_1_IRQn); break;
    case 2:
    case 3: NVIC_SetPriority(EXTI2_3_IRQn, 15); NVIC_EnableIRQ(EXTI2_3_IRQn); break;
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15: NVIC_SetPriority(EXTI4_15_IRQn, 15); NVIC_EnableIRQ(EXTI4_15_IRQn); break;
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
    //     if (pos <= 4) SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] &~(0x0f << (pos- 0))) | (mask << (pos - 0));  //set the line 0..4
    //else if (pos <= 8) SYSCFG->EXTICR[1] = (SYSCFG->EXTICR[1] &~(0x0f << (pos- 4))) | (mask << (pos - 4));  //set the line 5..8
    //else if (pos <=12) SYSCFG->EXTICR[2] = (SYSCFG->EXTICR[2] &~(0x0f << (pos- 8))) | (mask << (pos - 8));  //set the line 9..12
    //else if (pos <=16) SYSCFG->EXTICR[3] = (SYSCFG->EXTICR[3] &~(0x0f << (pos-12))) | (mask << (pos -12));  //set the line 13..16

    //figure out the trigger: rising or falling
    //switch (mode) {
    //case CHANGE:	EXTI->FTSR |= (1<<pos); EXTI->RTSR |= (1<<pos); break;		//set falling trigger
    //case RISING: 	EXTI->RTSR |= (1<<pos); break;		//set rising trigger
    //case FALLING:
    //default: 		EXTI->FTSR |= (1<<pos); break;		//set falling trigger (default)
    //}

    //install the default handler
	_isrptr_exti[pos] = empty_handler;					//isrptr;
	//set it to the lowest priority, and disable the IRQ
    switch (pos) {
    case 0:
    case 1: NVIC_SetPriority(EXTI0_1_IRQn, 15); NVIC_DisableIRQ(EXTI0_1_IRQn); break;
    case 2:
    case 3: NVIC_SetPriority(EXTI2_3_IRQn, 15); NVIC_DisableIRQ(EXTI2_3_IRQn); break;
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15: NVIC_SetPriority(EXTI4_15_IRQn, 15); NVIC_DisableIRQ(EXTI4_15_IRQn); break;
    default: break;		//do nothing
    }

    //activate the interrupt
    EXTI->PR |= (1<<pos);                           //clear the flag
    EXTI->IMR &=~(1<<pos);                          //disable the exti
}

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
	uint16_t uartdiv;

	//configure uart1
    //route clock to uart1
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    USART1->CR1 &=~(1<<0);			//'0'->disable uart, '1'->enable uart
    USART1->CR1 =	(0<<28) | (0<<12) |	//0b00->1 start bit, 8 data bits, n stop bit; 0b01->1 start bit, 9 data bits, n stop bit, 0b10->1 start bit 7 data bits, n stop bit
    				(0<<27) |		//0->disable end of block interrupt
    				(0<<26) |		//0->receiver timeout interrupt disabled
    				(0x00<<21) |	//0b00000->driver enable assertion time
    				(0x00<<16) |	//0b00000->driver enable disassertion time
    				(0<<15) |		//0->oversampling by 16
    				(0<<14) |		//0->character match interrupt disabled
    				(0<<13) |		//0->receiver in active mode permanently
    				//bit 12 set earlier
    				(0<<11) |		//0->idle line to wake up uart
    				(0<<10) |		//0->no parity
    				(0<<9) |		//0->even parity
    				(0<<8) |		//0->disable PE interrupt
    				(0<<7) |		//0->disable txie)
    				(0<<6) |		//0->disable transmission complete interrupt
    				(0<<5) |		//0->disable receiver buffer not empty interrupt
    				(0<<4) |		//0->disable idle interrupt
    				(1<<3) |		//0->transmitter disabled, 1->transmitter enabled
    				(1<<2) |		//0->receiver disabled, 1->receiver enabled
    				//bit 1 reserved
    				(0<<0) |		//0->disable uart, 1->enable uart
    				0x00;
    USART1->CR2 = 	(0x00<<28) |	//address of the uart
    				(0x00<<24) |	//address of the uart
    				(0<<23) |		//0->disable receiver time out
    				(0x00<<21) |	//00->measurement of the start bit used to detect baud rate
    				(0<<20) |		//auto baud rate disabled
    				(0<<19) |		//0->data bit 0 first
    				(0<<18) |		//0->data transmitted / received in positive logic
    				(0<<17) |		//0->tx in positive logic
    				(0<<16) |		//0->rx in positive logic
    				(0<<15) |		//0->tx/rx pins not swapped, 1->tx/rx pins swapped
    				(0x00<<12) |	//00->1 stop bit, 10->2 stop bit, 11->1.5 stop bit
    				(0<<11) |		//0->sclk disabled
    				(0<<10) |		//0->sclk idles low
    				(0<<9) |		//0->clock on first data capture
    				(0<<8) |		//0->clock on the last bit is not data pulse
    				(0<<4) |		//0->4 bit address detection
    				0x00;
    USART1->CR3 =	(0<<15) |		//0->driver enable signal active high
    				(0<<14) |		//0->disable driver more
    				0x00;			//reset value
    //set the baudrate register
    uartdiv = F_UART / baudrate;
    if (USART1->CR1 & (1<<15)) {		//oversample by 8
    	uartdiv = uartdiv * 2;
    	uartdiv = 	(uartdiv &~0x000f) |	//clear lowest 4 bits
    				(1<<3) |			//bit 3 is always set
    				((uartdiv >> 1) & 0x07);	//keep the lowest 3 bits
    }
    USART1->BRR = uartdiv;
    //UARTx->BRR = F_UART / baudrate * ((UARTx->CR1 & (1<<15))?2:1);		//per datasheet, for OVER8=0 or 1

    //UARTx->DR = 0;					//reset the data register
    //UARTx->SR = 0;					//reset the data register

    //enable uart1
    USART1->CR1 |= (1<<0);			//'0'->disable uart, '1'->enable uart

    //configure the TX-PA9/RX-PA10 pins - GPIO clock assumed enabled here previously
    //RX as floating input/AF input, AF1
    IO_INFL(GPIOA, 1<<10); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f << (4 * (10-8)))) | (GPIOMODE_AF1<<(4 * (10-8)));
	//TX as AFPP, AF1
    IO_AFPP(GPIOA, 1<< 9); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f << (4 * ( 9-8)))) | (GPIOMODE_AF1<<(4 * ( 9-8)));
}

//uart1 send a char
void serial1Write(char dat) {
    while (!(USART1->ISR & USART_ISR_TXE));    	//wait for the transmission buffer to be empty
    //while (uart1_busy()) continue;    			//wait for the transmission buffer to be empty
    USART1->TDR = dat;                        	//load the data buffer
    //while (!(UARTx->SR & (1<<6)));    		//wait for the transmission to complete
}

//uart1 returns a char
unsigned char serial1Read(void) {
    //while (!(UARTx->SR & USART_SR_RXNE));  	//wait for the receipt buffer to be empty
    return USART1->RDR;                       	//save the transmission buffer
}

//uart1 print a string
void serial1Print(char *str) {
	while (*str) serial1Write(*str++);
}

//uart1 print a string + return
void serial1Println(char *str) {
	serial1Print(str);						//print the string
	serial1Print((char *)"\n\r");					//print the return
}

//test if uart1 receiver is available (=has data)
//return true if transmission on uart1 has completed
uint8_t serial1Available(void) {
	//return (UARTx->SR & USART_SR_TC)?true:false;
	return (USART1->ISR & USART_ISR_RXNE);
}

//test if uart1 transmitter is busy
//return true if transmission on uart1 has completed
uint8_t serial1Busy(void) {
	//return (UARTx->SR & USART_SR_TC)?true:false;
	return !(USART1->ISR & USART_ISR_TXE);
}
#endif								//usart1

#if defined(USE_UART2)
//initialize uart2
void serial2Begin(uint32_t baudrate) {
	uint16_t uartdiv;

	//configure uart2
    //route clock to uart2
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    USART2->CR1 &=~(1<<0);			//'0'->disable uart, '1'->enable uart
    USART2->CR1 =	(0<<28) | (0<<12) |	//0b00->1 start bit, 8 data bits, n stop bit; 0b01->1 start bit, 9 data bits, n stop bit, 0b10->1 start bit 7 data bits, n stop bit
    				(0<<27) |		//0->disable end of block interrupt
    				(0<<26) |		//0->receiver timeout interrupt disabled
    				(0x00<<21) |	//0b00000->driver enable assertion time
    				(0x00<<16) |	//0b00000->driver enable disassertion time
    				(0<<15) |		//0->oversampling by 16
    				(0<<14) |		//0->character match interrupt disabled
    				(0<<13) |		//0->receiver in active mode permanently
    				//bit 12 set earlier
    				(0<<11) |		//0->idle line to wake up uart
    				(0<<10) |		//0->no parity
    				(0<<9) |		//0->even parity
    				(0<<8) |		//0->disable PE interrupt
    				(0<<7) |		//0->disable txie)
    				(0<<6) |		//0->disable transmission complete interrupt
    				(0<<5) |		//0->disable receiver buffer not empty interrupt
    				(0<<4) |		//0->disable idle interrupt
    				(1<<3) |		//0->transmitter disabled, 1->transmitter enabled
    				(1<<2) |		//0->receiver disabled, 1->receiver enabled
    				//bit 1 reserved
    				(0<<0) |		//0->disable uart, 1->enable uart
    				0x00;
    USART2->CR2 = 	(0x00<<28) |	//address of the uart
    				(0x00<<24) |	//address of the uart
    				(0<<23) |		//0->disable receiver time out
    				(0x00<<21) |	//00->measurement of the start bit used to detect baud rate
    				(0<<20) |		//auto baud rate disabled
    				(0<<19) |		//0->data bit 0 first
    				(0<<18) |		//0->data transmitted / received in positive logic
    				(0<<17) |		//0->tx in positive logic
    				(0<<16) |		//0->rx in positive logic
    				(0<<15) |		//0->tx/rx pins not swapped, 1->tx/rx pins swapped
    				(0x00<<12) |	//00->1 stop bit, 10->2 stop bit, 11->1.5 stop bit
    				(0<<11) |		//0->sclk disabled
    				(0<<10) |		//0->sclk idles low
    				(0<<9) |		//0->clock on first data capture
    				(0<<8) |		//0->clock on the last bit is not data pulse
    				(0<<4) |		//0->4 bit address detection
    				0x00;
    USART2->CR3 =	(0<<15) |		//0->driver enable signal active high
    				(0<<14) |		//0->disable driver more
    				0x00;			//reset value
    //set the baudrate register
    uartdiv = F_UART / baudrate;
    if (USART2->CR1 & (1<<15)) {		//oversample by 8
    	uartdiv = uartdiv * 2;
    	uartdiv = 	(uartdiv &~0x000f) |	//clear lowest 4 bits
    				(1<<3) |			//bit 3 is always set
    				((uartdiv >> 1) & 0x07);	//keep the lowest 3 bits
    }
    USART2->BRR = uartdiv;
    //UARTx->BRR = F_UART / baudrate * ((UARTx->CR1 & (1<<15))?2:1);		//per datasheet, for OVER8=0 or 1

    //UARTx->DR = 0;					//reset the data register
    //UARTx->SR = 0;					//reset the data register

    //enable uart2
    USART2->CR1 |= (1<<0);			//'0'->disable uart, '1'->enable uart

    //configure the TX-PA9/RX-PA10 pins - GPIO clock assumed enabled here previously
    //RX as floating input/AF input, AF1
    IO_INFL(GPIOA, 1<<10); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f << (4 * (10-8)))) | (GPIOMODE_AF1<<(4 * (10-8)));
	//TX as AFPP, AF1
    IO_AFPP(GPIOA, 1<< 9); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f << (4 * ( 9-8)))) | (GPIOMODE_AF1<<(4 * ( 9-8)));
}

//uart2 send a char
void serial2Write(char dat) {
    while (!(USART2->ISR & USART_ISR_TXE));    	//wait for the transmission buffer to be empty
    //while (uart2_busy()) continue;    			//wait for the transmission buffer to be empty
    USART2->TDR = dat;                        	//load the data buffer
    //while (!(UARTx->SR & (1<<6)));    		//wait for the transmission to complete
}

//uart2 returns a char
unsigned char serial2Read(void) {
    //while (!(UARTx->SR & USART_SR_RXNE));  	//wait for the receipt buffer to be empty
    return USART2->RDR;                       	//save the transmission buffer
}

//uart2 print a string
void serial2Print(char *str) {
	while (*str) serial2Write(*str++);
}

//uart2 print a string + return
void seria21Println(char *str) {
	serial2Print(str);						//print the string
	serial2Print((char *) "\n\r");					//print the return
}

//test if uart2 receiver is available (=has data)
//return true if transmission on uart2 has completed
uint8_t serial2Available(void) {
	//return (UARTx->SR & USART_SR_TC)?true:false;
	return (USART2->ISR & USART_ISR_RXNE);
}

//test if uart2 transmitter is busy
//return true if transmission on uart2 has completed
uint8_t serial2Busy(void) {
	//return (UARTx->SR & USART_SR_TC)?true:false;
	return !(USART2->ISR & USART_ISR_TXE);
}
#endif								//usart1

#if defined(USE_UART3)
//initialize uart3
void serial3Begin(uint32_t baudrate) {
	uint16_t uartdiv;

	//configure uart1
    //route clock to uart1
    RCC->APB2ENR |= RCC_APB2ENR_USART3EN;

    USART3->CR1 &=~(1<<0);			//'0'->disable uart, '1'->enable uart
    USART3->CR1 =	(0<<28) | (0<<12) |	//0b00->1 start bit, 8 data bits, n stop bit; 0b01->1 start bit, 9 data bits, n stop bit, 0b10->1 start bit 7 data bits, n stop bit
    				(0<<27) |		//0->disable end of block interrupt
    				(0<<26) |		//0->receiver timeout interrupt disabled
    				(0x00<<21) |	//0b00000->driver enable assertion time
    				(0x00<<16) |	//0b00000->driver enable disassertion time
    				(0<<15) |		//0->oversampling by 16
    				(0<<14) |		//0->character match interrupt disabled
    				(0<<13) |		//0->receiver in active mode permanently
    				//bit 12 set earlier
    				(0<<11) |		//0->idle line to wake up uart
    				(0<<10) |		//0->no parity
    				(0<<9) |		//0->even parity
    				(0<<8) |		//0->disable PE interrupt
    				(0<<7) |		//0->disable txie)
    				(0<<6) |		//0->disable transmission complete interrupt
    				(0<<5) |		//0->disable receiver buffer not empty interrupt
    				(0<<4) |		//0->disable idle interrupt
    				(1<<3) |		//0->transmitter disabled, 1->transmitter enabled
    				(1<<2) |		//0->receiver disabled, 1->receiver enabled
    				//bit 1 reserved
    				(0<<0) |		//0->disable uart, 1->enable uart
    				0x00;
    USART3->CR2 = 	(0x00<<28) |	//address of the uart
    				(0x00<<24) |	//address of the uart
    				(0<<23) |		//0->disable receiver time out
    				(0x00<<21) |	//00->measurement of the start bit used to detect baud rate
    				(0<<20) |		//auto baud rate disabled
    				(0<<19) |		//0->data bit 0 first
    				(0<<18) |		//0->data transmitted / received in positive logic
    				(0<<17) |		//0->tx in positive logic
    				(0<<16) |		//0->rx in positive logic
    				(0<<15) |		//0->tx/rx pins not swapped, 1->tx/rx pins swapped
    				(0x00<<12) |	//00->1 stop bit, 10->2 stop bit, 11->1.5 stop bit
    				(0<<11) |		//0->sclk disabled
    				(0<<10) |		//0->sclk idles low
    				(0<<9) |		//0->clock on first data capture
    				(0<<8) |		//0->clock on the last bit is not data pulse
    				(0<<4) |		//0->4 bit address detection
    				0x00;
    USART3->CR3 =	(0<<15) |		//0->driver enable signal active high
    				(0<<14) |		//0->disable driver more
    				0x00;			//reset value
    //set the baudrate register
    uartdiv = F_UART / baudrate;
    if (USART3->CR1 & (1<<15)) {		//oversample by 8
    	uartdiv = uartdiv * 2;
    	uartdiv = 	(uartdiv &~0x000f) |	//clear lowest 4 bits
    				(1<<3) |			//bit 3 is always set
    				((uartdiv >> 1) & 0x07);	//keep the lowest 3 bits
    }
    USART3->BRR = uartdiv;
    //UARTx->BRR = F_UART / baudrate * ((UARTx->CR1 & (1<<15))?2:1);		//per datasheet, for OVER8=0 or 1

    //UARTx->DR = 0;					//reset the data register
    //UARTx->SR = 0;					//reset the data register

    //enable uart1
    USART3->CR1 |= (1<<0);			//'0'->disable uart, '1'->enable uart

    //configure the TX-PA9/RX-PA10 pins - GPIO clock assumed enabled here previously
    //RX as floating input/AF input, AF1
    IO_INFL(GPIOA, 1<<10); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f << (4 * (10-8)))) | (GPIOMODE_AF1<<(4 * (10-8)));
	//TX as AFPP, AF1
    IO_AFPP(GPIOA, 1<< 9); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f << (4 * ( 9-8)))) | (GPIOMODE_AF1<<(4 * ( 9-8)));
}

//uart3 send a char
void serial3Write(char dat) {
    while (!(USART3->ISR & USART_ISR_TXE));    	//wait for the transmission buffer to be empty
    //while (uart1_busy()) continue;    			//wait for the transmission buffer to be empty
    USART3->TDR = dat;                        	//load the data buffer
    //while (!(UARTx->SR & (1<<6)));    		//wait for the transmission to complete
}

//uart3 returns a char
unsigned char seria31Read(void) {
    //while (!(UARTx->SR & USART_SR_RXNE));  	//wait for the receipt buffer to be empty
    return USART3->RDR;                       	//save the transmission buffer
}

//uart3 print a string
void serial3Print(char *str) {
	while (*str) uart1_put(*str++);
}

//uart3 print a string + return
void seria31Println(char *str) {
	serial3Print(str);						//print the string
	serial3Print((char *)"\n\r");					//print the return
}

//test if uart3 receiver is available (=has data)
//return true if transmission on uart3 has completed
uint8_t seria31Available(void) {
	//return (UARTx->SR & USART_SR_TC)?true:false;
	return (USART3->ISR & USART_ISR_RXNE);
}

//test if uart3 transmitter is busy
//return true if transmission on uart3 has completed
uint8_t seria31Busy(void) {
	//return (UARTx->SR & USART_SR_TC)?true:false;
	return !(USART3->ISR & USART_ISR_TXE);
}
#endif								//usart1

#if defined(USE_TIM1)
//initialize tim1 to use compare channels as timers
//16-bit prescaler. 32-bit used for compatability
void tim1_init(uint32_t ps) {
	//route the clock to timer
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	//source from internal clock -> disable slave mode
	TIM1->SMCR &=~TIM_SMCR_SMS;			//clear sms->use internal clock

	//stop the timer to configure it
	TIM1->CR1 &=~TIM_CR1_CEN;			//clear cen. 0=disable the timer, 1=enable the timer
	TIM1->CR1 &=~TIM_CR1_CKD;			//clear CKD0..1. 0b00->1x clock; 0b01->2:1 clock, 0b10->4:1 clk; 0b11->reserved
	TIM1->CR1 &=~TIM_CR1_DIR;			//clear DIR bit. 0=upcounter, 1=downcounter
	TIM1->CR1 &=~TIM_CR1_OPM;			//clear opm bit. 0=periodic timer, 1=one-shot timer
	//or to simply zero the register
	//TIM1->CR1 = 0;						//much easier

	//clear the status register bits for capture / compare flags
	TIM1->SR &=~(TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF);
	//disable the interrupt by clearing the enable bits
	TIM1->DIER &=~(TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE);

	//set the prescaler
	TIM1->PSC = ps - 1;					//set the prescaler
	TIM1->RCR = 0;						//repetition counter = 0 (=no repetition)
	TIM1->ARR = -1;						//auto reload register / period = 0; - need to change for downcounters
	TIM1->CNT = 0;						//reset the counter

	//enable the timer.
	TIM1->CR1 |= TIM_CR1_CEN;			//enable the timer
}

//set tim1_oc1 period
//pr is 16-bit. 32-bit used for compatability;
void tim1_setpr1(uint32_t pr) {
	//save the period value
	_tim1_oc1 = pr - 1;
	TIM1->CCR1 = _tim1_oc1;

	//clear the flag
	//TIM1->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM1->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//install user handler
void tim1_act1(void (*isr_ptr)(void)) {
	NVIC_DisableIRQ(TIM1_CC_IRQn);		//disable irq

	_tim1_oc1isrptr = isr_ptr;			//install user handler

	//clear the flag
	TIM1->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	TIM1->DIER |= TIM_DIER_CC1IE;		//enable the isr

	NVIC_EnableIRQ(TIM1_CC_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//set tim1_oc2 period
//pr is 16-bit. 32-bit used for compatability;
void tim1_setpr2(uint32_t pr) {
	//save the period value
	_tim1_oc2 = pr - 1;
	TIM1->CCR2 = _tim1_oc2;

	//clear the flag
	//TIM1->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM1->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//install user handler
void tim1_act2(void (*isr_ptr)(void)) {
	NVIC_DisableIRQ(TIM1_CC_IRQn);		//disable irq

	_tim1_oc2isrptr = isr_ptr;			//install user handler

	//clear the flag
	TIM1->SR &=~TIM_SR_CC2IF;			//clear the interrupt flag
	TIM1->DIER |= TIM_DIER_CC2IE;		//enable the isr

	NVIC_EnableIRQ(TIM1_CC_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//set tim1_oc2 period
//pr is 16-bit. 32-bit used for compatability;
void tim1_setpr3(uint32_t pr) {
	//save the period value
	_tim1_oc3 = pr - 1;
	TIM1->CCR3 = _tim1_oc3;

	//clear the flag
	//TIM1->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM1->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//install user handler
void tim1_act3(void (*isr_ptr)(void)) {
	NVIC_DisableIRQ(TIM1_CC_IRQn);		//disable irq

	_tim1_oc3isrptr = isr_ptr;			//install user handler

	//clear the flag
	TIM1->SR &=~TIM_SR_CC3IF;			//clear the interrupt flag
	TIM1->DIER |= TIM_DIER_CC3IE;		//enable the isr

	NVIC_EnableIRQ(TIM1_CC_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//set tim1_oc2 period
//pr is 16-bit. 32-bit used for compatability;
void tim1_setpr4(uint32_t pr) {
	//save the period value
	_tim1_oc4 = pr - 1;
	TIM1->CCR4 = _tim1_oc4;

	//clear the flag
	//TIM1->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM1->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//install user handler
void tim1_act4(void (*isr_ptr)(void)) {
	NVIC_DisableIRQ(TIM1_CC_IRQn);		//disable irq

	_tim1_oc4isrptr = isr_ptr;			//install user handler

	//clear the flag
	TIM1->SR &=~TIM_SR_CC4IF;			//clear the interrupt flag
	TIM1->DIER |= TIM_DIER_CC4IE;		//enable the isr

	NVIC_EnableIRQ(TIM1_CC_IRQn);		//enable irq
	//priorities not set -> default values used.
}
#endif

#if defined(USE_TIM2)
//initialize tim2 to use compare channels as timers
//16-bit prescaler. 32-bit used for compatability
void tim2_init(uint32_t ps) {
	//route the clock to timer
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	//source from internal clock -> disable slave mode
	TIM2->SMCR &=~TIM_SMCR_SMS;			//clear sms->use internal clock

	//stop the timer to configure it
	TIM2->CR1 &=~TIM_CR1_CEN;			//clear cen. 0=disable the timer, 1=enable the timer
	TIM2->CR1 &=~TIM_CR1_CKD;			//clear CKD0..1. 0b00->1x clock; 0b01->2:1 clock, 0b10->4:1 clk; 0b11->reserved
	TIM2->CR1 &=~TIM_CR1_DIR;			//clear DIR bit. 0=upcounter, 1=downcounter
	TIM2->CR1 &=~TIM_CR1_OPM;			//clear opm bit. 0=periodic timer, 1=one-shot timer
	//or to simply zero the register
	//TIM2->CR1 = 0;						//much easier

	//clear the status register bits for capture / compare flags
	TIM2->SR &=~(TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF);
	//disable the interrupt by clearing the enable bits
	TIM2->DIER &=~(TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE);

	//set the prescaler
	TIM2->PSC = ps - 1;					//set the prescaler
	TIM2->RCR = 0;						//repetition counter = 0 (=no repetition)
	TIM2->ARR = -1;						//auto reload register / period = 0; - need to change for downcounters
	TIM2->CNT = 0;						//reset the counter

	//enable the timer.
	TIM2->CR1 |= TIM_CR1_CEN;			//enable the timer
}

//set tim2_oc1 period
//pr is 16-bit. 32-bit used for compatability;
void tim2_setpr1(uint32_t pr) {
	//save the period value
	_tim2_oc1 = pr - 1;
	TIM2->CCR1 = _tim2_oc1;

	//clear the flag
	//TIM2->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM2->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//install user handler
void tim2_act1(void (*isr_ptr)(void)) {
	NVIC_DisableIRQ(TIM2_IRQn);		//disable irq

	_tim2_oc1isrptr = isr_ptr;			//install user handler

	//clear the flag
	TIM2->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	TIM2->DIER |= TIM_DIER_CC1IE;		//enable the isr

	NVIC_EnableIRQ(TIM2_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//set tim2_oc2 period
//pr is 16-bit. 32-bit used for compatability;
void tim2_setpr2(uint32_t pr) {
	//save the period value
	_tim2_oc2 = pr - 1;
	TIM2->CCR2 = _tim2_oc2;

	//clear the flag
	//TIM2->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM2->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//install user handler
void tim2_act2(void (*isr_ptr)(void)) {
	NVIC_DisableIRQ(TIM2_IRQn);		//disable irq

	_tim2_oc2isrptr = isr_ptr;			//install user handler

	//clear the flag
	TIM2->SR &=~TIM_SR_CC2IF;			//clear the interrupt flag
	TIM2->DIER |= TIM_DIER_CC2IE;		//enable the isr

	NVIC_EnableIRQ(TIM2_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//set tim2_oc2 period
//pr is 16-bit. 32-bit used for compatability;
void tim2_setpr3(uint32_t pr) {
	//save the period value
	_tim2_oc3 = pr - 1;
	TIM2->CCR3 = _tim2_oc3;

	//clear the flag
	//TIM2->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM2->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//install user handler
void tim2_act3(void (*isr_ptr)(void)) {
	NVIC_DisableIRQ(TIM2_IRQn);		//disable irq

	_tim2_oc3isrptr = isr_ptr;			//install user handler

	//clear the flag
	TIM2->SR &=~TIM_SR_CC3IF;			//clear the interrupt flag
	TIM2->DIER |= TIM_DIER_CC3IE;		//enable the isr

	NVIC_EnableIRQ(TIM2_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//set tim2_oc2 period
//pr is 16-bit. 32-bit used for compatability;
void tim2_setpr4(uint32_t pr) {
	//save the period value
	_tim2_oc4 = pr - 1;
	TIM2->CCR4 = _tim2_oc4;

	//clear the flag
	//TIM2->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM2->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//install user handler
void tim2_act4(void (*isr_ptr)(void)) {
	NVIC_DisableIRQ(TIM2_IRQn);		//disable irq

	_tim2_oc4isrptr = isr_ptr;			//install user handler

	//clear the flag
	TIM2->SR &=~TIM_SR_CC4IF;			//clear the interrupt flag
	TIM2->DIER |= TIM_DIER_CC4IE;		//enable the isr

	NVIC_EnableIRQ(TIM2_IRQn);		//enable irq
	//priorities not set -> default values used.
}
#endif

#if defined(USE_TIM3)
//initialize tim3 to use compare channels as timers
//16-bit prescaler. 32-bit used for compatability
void tim3_init(uint32_t ps) {
	//route the clock to timer
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	//source from internal clock -> disable slave mode
	TIM3->SMCR &=~TIM_SMCR_SMS;			//clear sms->use internal clock

	//stop the timer to configure it
	TIM3->CR1 &=~TIM_CR1_CEN;			//clear cen. 0=disable the timer, 1=enable the timer
	TIM3->CR1 &=~TIM_CR1_CKD;			//clear CKD0..1. 0b00->1x clock; 0b01->2:1 clock, 0b10->4:1 clk; 0b11->reserved
	TIM3->CR1 &=~TIM_CR1_DIR;			//clear DIR bit. 0=upcounter, 1=downcounter
	TIM3->CR1 &=~TIM_CR1_OPM;			//clear opm bit. 0=periodic timer, 1=one-shot timer
	//or to simply zero the register
	//TIM3->CR1 = 0;						//much easier

	//clear the status register bits for capture / compare flags
	TIM3->SR &=~(TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF);
	//disable the interrupt by clearing the enable bits
	TIM3->DIER &=~(TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE);

	//set the prescaler
	TIM3->PSC = ps - 1;					//set the prescaler
	TIM3->RCR = 0;						//repetition counter = 0 (=no repetition)
	TIM3->ARR = -1;						//auto reload register / period = 0; - need to change for downcounters
	TIM3->CNT = 0;						//reset the counter

	//enable the timer.
	TIM3->CR1 |= TIM_CR1_CEN;			//enable the timer
}

//set tim3_oc1 period
//pr is 16-bit. 32-bit used for compatability;
void tim3_setpr1(uint32_t pr) {
	//save the period value
	_tim3_oc1 = pr - 1;
	TIM3->CCR1 = _tim3_oc1;

	//clear the flag
	//TIM3->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM3->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//install user handler
void tim3_act1(void (*isr_ptr)(void)) {
	NVIC_DisableIRQ(TIM3_IRQn);		//disable irq

	_tim3_oc1isrptr = isr_ptr;			//install user handler

	//clear the flag
	TIM3->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	TIM3->DIER |= TIM_DIER_CC1IE;		//enable the isr

	NVIC_EnableIRQ(TIM3_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//set tim3_oc2 period
//pr is 16-bit. 32-bit used for compatability;
void tim3_setpr2(uint32_t pr) {
	//save the period value
	_tim3_oc2 = pr - 1;
	TIM3->CCR2 = _tim3_oc2;

	//clear the flag
	//TIM3->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM3->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//install user handler
void tim3_act2(void (*isr_ptr)(void)) {
	NVIC_DisableIRQ(TIM3_IRQn);		//disable irq

	_tim3_oc2isrptr = isr_ptr;			//install user handler

	//clear the flag
	TIM3->SR &=~TIM_SR_CC2IF;			//clear the interrupt flag
	TIM3->DIER |= TIM_DIER_CC2IE;		//enable the isr

	NVIC_EnableIRQ(TIM3_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//set tim3_oc2 period
//pr is 16-bit. 32-bit used for compatability;
void tim3_setpr3(uint32_t pr) {
	//save the period value
	_tim3_oc3 = pr - 1;
	TIM3->CCR3 = _tim3_oc3;

	//clear the flag
	//TIM3->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM3->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//install user handler
void tim3_act3(void (*isr_ptr)(void)) {
	NVIC_DisableIRQ(TIM3_IRQn);		//disable irq

	_tim3_oc3isrptr = isr_ptr;			//install user handler

	//clear the flag
	TIM3->SR &=~TIM_SR_CC3IF;			//clear the interrupt flag
	TIM3->DIER |= TIM_DIER_CC3IE;		//enable the isr

	NVIC_EnableIRQ(TIM3_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//set tim3_oc2 period
//pr is 16-bit. 32-bit used for compatability;
void tim3_setpr4(uint32_t pr) {
	//save the period value
	_tim3_oc4 = pr - 1;
	TIM3->CCR4 = _tim3_oc4;

	//clear the flag
	//TIM3->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM3->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//install user handler
void tim3_act4(void (*isr_ptr)(void)) {
	NVIC_DisableIRQ(TIM3_IRQn);		//disable irq

	_tim3_oc4isrptr = isr_ptr;			//install user handler

	//clear the flag
	TIM3->SR &=~TIM_SR_CC4IF;			//clear the interrupt flag
	TIM3->DIER |= TIM_DIER_CC4IE;		//enable the isr

	NVIC_EnableIRQ(TIM3_IRQn);		//enable irq
	//priorities not set -> default values used.
}
#endif

#if defined(USE_TIM14)
//initialize tim4 to use compare channels as timers
//16-bit prescaler. 32-bit used for compatability
void tim14_init(uint32_t ps) {
	//route the clock to timer
	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;

	//source from internal clock -> disable slave mode
	TIM14->SMCR &=~TIM_SMCR_SMS;			//clear sms->use internal clock

	//stop the timer to configure it
	TIM14->CR1 &=~TIM_CR1_CEN;			//clear cen. 0=disable the timer, 1=enable the timer
	TIM14->CR1 &=~TIM_CR1_CKD;			//clear CKD0..1. 0b00->1x clock; 0b01->2:1 clock, 0b10->4:1 clk; 0b11->reserved
	TIM14->CR1 &=~TIM_CR1_DIR;			//clear DIR bit. 0=upcounter, 1=downcounter
	TIM14->CR1 &=~TIM_CR1_OPM;			//clear opm bit. 0=periodic timer, 1=one-shot timer
	//or to simply zero the register
	//TIM14->CR1 = 0;						//much easier

	//clear the status register bits for capture / compare flags
	TIM14->SR &=~(TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF | TIM_SR_UIF);
	//disable the interrupt by clearing the enable bits
	TIM14->DIER &=~(TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE | TIM_DIER_UIE);

	//set the prescaler
	TIM14->PSC = ps - 1;					//set the prescaler
	TIM14->RCR = 0;						//repetition counter = 0 (=no repetition)
	TIM14->ARR = -1;						//auto reload register / period = 0; - need to change for downcounters
	TIM14->CNT = 0;						//reset the counter

	//enable the timer.
	TIM14->CR1 |= TIM_CR1_CEN;			//enable the timer
}

//set tim4_oc1 period
//pr is 16-bit. 32-bit used for compatability;
void tim14_setpr(uint32_t pr) {
	//save the period value
	TIM14->ARR = pr - 1;

	//clear the flag
	//TIM14->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM14->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//install user handler
void tim14_act(void (*isr_ptr)(void)) {
	NVIC_DisableIRQ(TIM14_IRQn);		//disable irq

	_tim14_isrptr = isr_ptr;			//install user handler

	//clear the flag
	TIM14->SR &=~TIM_SR_UIF;			//clear the interrupt flag
	TIM14->DIER |= TIM_DIER_UIE;		//enable the isr

	NVIC_EnableIRQ(TIM14_IRQn);		//enable irq
	//priorities not set -> default values used.
}
#endif

#if defined(USE_TIM15)
//initialize tim4 to use compare channels as timers
//16-bit prescaler. 32-bit used for compatability
void tim15_init(uint32_t ps) {
	//route the clock to timer
	RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;

	//source from internal clock -> disable slave mode
	TIM15->SMCR &=~TIM_SMCR_SMS;			//clear sms->use internal clock

	//stop the timer to configure it
	TIM15->CR1 &=~TIM_CR1_CEN;			//clear cen. 0=disable the timer, 1=enable the timer
	TIM15->CR1 &=~TIM_CR1_CKD;			//clear CKD0..1. 0b00->1x clock; 0b01->2:1 clock, 0b10->4:1 clk; 0b11->reserved
	TIM15->CR1 &=~TIM_CR1_DIR;			//clear DIR bit. 0=upcounter, 1=downcounter
	TIM15->CR1 &=~TIM_CR1_OPM;			//clear opm bit. 0=periodic timer, 1=one-shot timer
	//or to simply zero the register
	//TIM15->CR1 = 0;						//much easier

	//clear the status register bits for capture / compare flags
	TIM15->SR &=~(TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF | TIM_SR_UIF);
	//disable the interrupt by clearing the enable bits
	TIM15->DIER &=~(TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE | TIM_DIER_UIE);

	//set the prescaler
	TIM15->PSC = ps - 1;					//set the prescaler
	TIM15->RCR = 0;						//repetition counter = 0 (=no repetition)
	TIM15->ARR = -1;						//auto reload register / period = 0; - need to change for downcounters
	TIM15->CNT = 0;						//reset the counter

	//enable the timer.
	TIM15->CR1 |= TIM_CR1_CEN;			//enable the timer
}

//set tim4_oc1 period
//pr is 16-bit. 32-bit used for compatability;
void tim15_setpr(uint32_t pr) {
	//save the period value
	TIM15->ARR = pr - 1;

	//clear the flag
	//TIM15->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM15->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//install user handler
void tim15_act(void (*isr_ptr)(void)) {
	NVIC_DisableIRQ(TIM15_IRQn);		//disable irq

	_tim15_isrptr = isr_ptr;			//install user handler

	//clear the flag
	TIM15->SR &=~TIM_SR_UIF;			//clear the interrupt flag
	TIM15->DIER |= TIM_DIER_UIE;		//enable the isr

	NVIC_EnableIRQ(TIM15_IRQn);		//enable irq
	//priorities not set -> default values used.
}
#endif

#if defined(USE_TIM16)
//initialize tim4 to use compare channels as timers
//16-bit prescaler. 32-bit used for compatability
void tim16_init(uint32_t ps) {
	//route the clock to timer
	RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;

	//source from internal clock -> disable slave mode
	TIM16->SMCR &=~TIM_SMCR_SMS;			//clear sms->use internal clock

	//stop the timer to configure it
	TIM16->CR1 &=~TIM_CR1_CEN;			//clear cen. 0=disable the timer, 1=enable the timer
	TIM16->CR1 &=~TIM_CR1_CKD;			//clear CKD0..1. 0b00->1x clock; 0b01->2:1 clock, 0b10->4:1 clk; 0b11->reserved
	TIM16->CR1 &=~TIM_CR1_DIR;			//clear DIR bit. 0=upcounter, 1=downcounter
	TIM16->CR1 &=~TIM_CR1_OPM;			//clear opm bit. 0=periodic timer, 1=one-shot timer
	//or to simply zero the register
	//TIM16->CR1 = 0;						//much easier

	//clear the status register bits for capture / compare flags
	TIM16->SR &=~(TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF | TIM_SR_UIF);
	//disable the interrupt by clearing the enable bits
	TIM16->DIER &=~(TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE | TIM_DIER_UIE);

	//set the prescaler
	TIM16->PSC = ps - 1;					//set the prescaler
	TIM16->RCR = 0;						//repetition counter = 0 (=no repetition)
	TIM16->ARR = -1;						//auto reload register / period = 0; - need to change for downcounters
	TIM16->CNT = 0;						//reset the counter

	//enable the timer.
	TIM16->CR1 |= TIM_CR1_CEN;			//enable the timer
}

//set tim4_oc1 period
//pr is 16-bit. 32-bit used for compatability;
void tim16_setpr(uint32_t pr) {
	//save the period value
	TIM16->ARR = pr - 1;

	//clear the flag
	//TIM16->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM16->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//install user handler
void tim16_act(void (*isr_ptr)(void)) {
	NVIC_DisableIRQ(TIM16_IRQn);		//disable irq

	_tim16_isrptr = isr_ptr;			//install user handler

	//clear the flag
	TIM16->SR &=~TIM_SR_UIF;			//clear the interrupt flag
	TIM16->DIER |= TIM_DIER_UIE;		//enable the isr

	NVIC_EnableIRQ(TIM16_IRQn);		//enable irq
	//priorities not set -> default values used.
}

#endif

#if defined(USE_TIM17)
//initialize tim4 to use compare channels as timers
//16-bit prescaler. 32-bit used for compatability
void tim17_init(uint32_t ps) {
	//route the clock to timer
	RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;

	//source from internal clock -> disable slave mode
	TIM17->SMCR &=~TIM_SMCR_SMS;			//clear sms->use internal clock

	//stop the timer to configure it
	TIM17->CR1 &=~TIM_CR1_CEN;			//clear cen. 0=disable the timer, 1=enable the timer
	TIM17->CR1 &=~TIM_CR1_CKD;			//clear CKD0..1. 0b00->1x clock; 0b01->2:1 clock, 0b10->4:1 clk; 0b11->reserved
	TIM17->CR1 &=~TIM_CR1_DIR;			//clear DIR bit. 0=upcounter, 1=downcounter
	TIM17->CR1 &=~TIM_CR1_OPM;			//clear opm bit. 0=periodic timer, 1=one-shot timer
	//or to simply zero the register
	//TIM17->CR1 = 0;						//much easier

	//clear the status register bits for capture / compare flags
	TIM17->SR &=~(TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF | TIM_SR_UIF);
	//disable the interrupt by clearing the enable bits
	TIM17->DIER &=~(TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE | TIM_DIER_UIE);

	//set the prescaler
	TIM17->PSC = ps - 1;					//set the prescaler
	TIM17->RCR = 0;						//repetition counter = 0 (=no repetition)
	TIM17->ARR = -1;						//auto reload register / period = 0; - need to change for downcounters
	TIM17->CNT = 0;						//reset the counter

	//enable the timer.
	TIM17->CR1 |= TIM_CR1_CEN;			//enable the timer
}

//set tim4_oc1 period
//pr is 16-bit. 32-bit used for compatability;
void tim17_setpr(uint32_t pr) {
	//save the period value
	TIM17->ARR = pr - 1;

	//clear the flag
	//TIM17->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM17->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//install user handler
void tim17_act(void (*isr_ptr)(void)) {
	NVIC_DisableIRQ(TIM17_IRQn);		//disable irq

	_tim17_isrptr = isr_ptr;			//install user handler

	//clear the flag
	TIM17->SR &=~TIM_SR_UIF;			//clear the interrupt flag
	TIM17->DIER |= TIM_DIER_UIE;		//enable the isr

	NVIC_EnableIRQ(TIM17_IRQn);		//enable irq
	//priorities not set -> default values used.
}
#endif

//clock mgmt
//switch clock to HSI
//reset clock to its default state
void SystemCoreClock2HSI(void) {
    // FLASH prefetch = on, wait state to 1
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;

#if 1
    /* Set HSION bit */
    RCC->CR |= (uint32_t)0x00000001;

  #if defined (STM32F051)
    /* Reset SW[1:0], HPRE[3:0], PPRE[2:0] and MCOSEL[2:0] bits */
    RCC->CFGR &= (uint32_t)0xF8FFB80C;
  #else
    /* Reset SW[1:0], HPRE[3:0], PPRE[2:0], ADCPRE, MCOSEL[2:0], MCOPRE[2:0] and PLLNODIV bits */
    RCC->CFGR &= (uint32_t)0x08FFB80C;
  #endif /* STM32F051 */

    /* Reset HSEON, CSSON and PLLON bits */
    RCC->CR &= (uint32_t)0xFEF6FFFF;

    /* Reset HSEBYP bit */
    RCC->CR &= (uint32_t)0xFFFBFFFF;

    /* Reset PLLSRC, PLLXTPRE and PLLMUL[3:0] bits */
    RCC->CFGR &= (uint32_t)0xFFC0FFFF;

    /* Reset PREDIV1[3:0] bits */
    RCC->CFGR2 &= (uint32_t)0xFFFFFFF0;

    /* Reset USARTSW[1:0], I2CSW, CECSW and ADCSW bits */
    RCC->CFGR3 &= (uint32_t)0xFFFFFEAC;

    /* Reset HSI14 bit */
    RCC->CR2 &= (uint32_t)0xFFFFFFFE;

    /* Disable all interrupts */
    RCC->CIR = 0x00000000;
#else
    // PLL input = HSI/2 = 4MHz, PLL Mult = 12X
    //RCC->CFGR = mul & RCC_CFGR_PLLMULL;	//RCC_CFGR_PLLMUL12;
   	RCC->CR |= RCC_CR_HSION;

   	// Wait for HSI ready
    while(!(RCC->CR & RCC_CR_HSIRDY)) continue;

    // Switch to HSI as clock source
    RCC->CFGR |= RCC_CFGR_SW_HSI;

    // wait for clock switching
    while((RCC->CFGR & RCC_CFGR_SWS)!= RCC_CFGR_SWS_HSI) continue;
#endif
}

//switch clock to HSE
void SystemCoreClock2HSE(void) {
    // FLASH prefetch = on, wait state to 1
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;

    // PLL input = HSI/2 = 4MHz, PLL Mult = 12X
    //RCC->CFGR = mul & RCC_CFGR_PLLMULL;	//RCC_CFGR_PLLMUL12;
   	RCC->CR |= RCC_CR_HSEON;

   	// Wait for HSE ready
   	while(!(RCC->CR & RCC_CR_HSERDY)) continue;

   	// Switch to HSE as clock source
   	RCC->CFGR |= RCC_CFGR_SW_HSE;

   	// wait for clock switching
   	while((RCC->CFGR & RCC_CFGR_SWS)!= RCC_CFGR_SWS_HSE) continue;
}

//to switch clocks, switch to SystemCoreClock2HSI() first (default state)
//switch to HSIxPLL
void SystemCoreClock2HSIPLL(uint32_t mul) {
    // FLASH prefetch = on, wait state to 1
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;

    // PLL input = HSI/2 = 4MHz, PLL Mult = 12X
    RCC->CFGR|= RCC_CFGR_PLLSRC_HSI_Div2;	//select HSI as PLL source
    RCC->CFGR = (RCC->CFGR &~RCC_CFGR_PLLMULL) | (mul & RCC_CFGR_PLLMULL);	//RCC_CFGR_PLLMUL12;
    //RCC->CFGR2= (RCC->CFGR2&~RCC_CFGR2_PREDIV1)| (mul_div & RCC_CFGR2_PREDIV1);	//set divider
    RCC->CR |= RCC_CR_PLLON;

    // Wait for PLL ready
    while(!(RCC->CR & RCC_CR_PLLRDY)) continue;

    // Switch to PLL as clock source
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    // wait for clock switching
    while((RCC->CFGR & RCC_CFGR_SWS)!= RCC_CFGR_SWS_PLL) continue;
}

//switch to HSExPLL
void SystemCoreClock2HSEPLL(uint32_t mul_div) {
    // FLASH prefetch = on, wait state to 1
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;

    // PLL input = HSE = 4MHz, PLL Mult = 12X
    RCC->CFGR&=~RCC_CFGR_PLLSRC_HSI_Div2;	//select HSE as PLL source
    RCC->CFGR = (RCC->CFGR &~RCC_CFGR_PLLMULL) | (mul_div & RCC_CFGR_PLLMULL);	//RCC_CFGR_PLLMUL12;
    RCC->CFGR2= (RCC->CFGR2&~RCC_CFGR2_PREDIV1)| (mul_div & RCC_CFGR2_PREDIV1);	//set divider
    RCC->CR |= RCC_CR_PLLON;

    // Wait for PLL ready
    while(!(RCC->CR & RCC_CR_PLLRDY)) continue;

    // Switch to PLL as clock source
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    // wait for clock switching
    while((RCC->CFGR & RCC_CFGR_SWS)!= RCC_CFGR_SWS_PLL) continue;
}


//initialize the chip
void chip_init(void) {
	//select the clock source
	//or use default clock
	//SystemCoreClock2HSI_16Mhz(); SystemCoreClock2HSI();
	//SystemCoreClock2HSI(); SystemCoreClock2HSI_48Mhz();
	//SystemCoreClock2HSI(); SystemCoreClock2HSI_24Mhz();
	SystemCoreClock2HSI();						//default clock = 8Mhz

	//enable clock to GPIO
	RCC->AHBENR |=
			RCC_AHBENR_GPIOAEN |
			RCC_AHBENR_GPIOBEN |
			RCC_AHBENR_GPIOCEN |
			RCC_AHBENR_GPIODEN |
			//RCC_AHBENR_GPIOEEN |
#if defined(RCC_AHBENR_GPIOFEN)
			RCC_AHBENR_GPIOFEN |
#endif
#if defined(RCC_AHBENR_GPIOGEN)
			RCC_AHBENR_GPIOGEN |
#endif
			0x00;

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
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;		//enable clock to ADCx

	//configure adc
	ADC1->CFGR1 = 	(0x00<<26) |			//0->awd on channel 0
					(0<<23) |				//0->AWD disabled
					(0<<22) |				//0->AWD on all channels (but disabled by bit 22
					(1<<16) |				//1->enable discontinuous conversion
					(0<<15) |				//0->auto off disabled
					(0<<14) |				//0->wait conversion disabled
					(0<<13) |				//0->single conversion mode, 1->continuous mode
					(0<<12) |				//0->adc data register preserved in overrun
					(0x00<<10) |			//00->hardware external trigger disabled
					(0x00<<6) |				//0000->external on TRG0 - but disabled
					(0<<5) |				//0->right aligned, 1->left aligned
					(0x00<<3) |				//00->data resolution = 12bit, 01->10bit, 10->8bit, 11->6bit
					(0<<2) |				//0->upward scan
					(0<<1) |				//0->DMA one shot mode selected
					(0<<0) |				//0->DMA disabled
					0x00;
	ADC1->CFGR2 = 	(0x02 << 30);			//00->adc clock, 01->PCLK/2, 10->PCLK/4 -> no jitter
	//set adc sample time
	//0b111->239.5 cycles for all channels
	ADC1->SMPR = 	(ADC_SMPR << (3 * 0)) |
					0x00;
	//set adc channel sequence
	//ADCx->SQR3 = ADCx->SQR2 = ADCx->SQR1 = 0;							//0->1 conversion

	//start self-calibration
	ADC1->CR =	0;							//reset CR
	ADC1->CR = (1<<15);						//start the calibration
	while (ADC1->CR & (1<<15)) continue;	//wait for ADC calibration to finish
	//uint32_t _adc_calfactor = ADCx->DR;				//save adc calibration factor

	//optional: enable temperature sensors
	ADC->CCR |= 	(1ul<<23) |				//1->enable temperature sensor
					(1ul<<22) |				//1->enable Vrefint. 1.20v nominal
					0x00;

	ADC1->CR = 	(1<<0);						//enable aden
	while ((ADC1->ISR & (1<<0)) == 0) continue;	//wait for the adc to be ready
	//now adc is calibrated
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


