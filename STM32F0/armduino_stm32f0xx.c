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
	case INPUT_PULLUP: 	GPIO_DDR(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask, GPIOMODE_INPU); IO_SET(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask); break;			//input with pullup
	//case INPUT_PULLDN:	GPIO_DDR(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask, GPIOMODE_INDN); GIO_CLR(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask); break;			//input with pulldown
	//case INPUT_ANALOG:	GPIO_DDR(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask, GPIOMODE_INAN); break;			//analog input
	case OUTPUT:		GPIO_DDR(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask, GPIOMODE_OUTPP); break;			//floating input
	//case OUTPUT_OD:		GPIO_DDR(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask, GPIOMODE_OUTOD); break;			//floating input
	//case OUTPUT_AFPP:	GPIO_DDR(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask, GPIOMODE_AFPP); break;			//floating input
	//case OUTPUT_AFOD:	GPIO_DDR(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask, GPIOMODE_AFOD); break;			//floating input
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
	ADC1->ISR &= ~(1<<2);					//clear the eoc flag
	//ADC1->CR1 = (ADC1->CR1 &~0x1f) | (ain & 0x1f);	//pick the adc channel
	//ADC1->CR2|= (1<<0);					//start the conversion
	ADC1->CHSELR = ain & 0x03fffful;		//define the first (and only) adc ch
	ADC1->CR |= (1<<2);						//start conversion
	while ((ADC1->ISR & (1<<2)) == 0) continue;	//wait for conversion to end (EOC set at end of conversion)
	return ADC1->DR;						//return adc result and clears the EOC bit
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
    //if (gpio == GPIOE) mask = 0x04;
    if (gpio == GPIOF) mask = 0x05;
    //if (gpio == GPIOG) mask = 0x06;

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
	ADCx->CFGR1 = 	(0x00<<26) |			//0->awd on channel 0
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
	ADCx->CFGR2 = 	(0x02 << 30);			//00->adc clock, 01->PCLK/2, 10->PCLK/4 -> no jitter
	//set adc sample time
	//0b111->239.5 cycles for all channels
	ADCx->SMPR = 	(ADC_SMPR << (3 * 0)) |
					0x00;
	//set adc channel sequence
	//ADCx->SQR3 = ADCx->SQR2 = ADCx->SQR1 = 0;							//0->1 conversion

	//start self-calibration
	ADCx->CR =	0;							//reset CR
	ADCx->CR = (1<<15);						//start the calibration
	while (ADCx->CR & (1<<15)) continue;	//wait for ADC calibration to finish
	//uint32_t _adc_calfactor = ADCx->DR;				//save adc calibration factor

	//optional: enable temperature sensors
	ADC->CCR |= 	(1ul<<23) |				//1->enable temperature sensor
					(1ul<<22) |				//1->enable Vrefint. 1.20v nominal
					0x00;

	ADCx->CR = 	(1<<0);						//enable aden
	while ((ADCx->ISR & (1<<0)) == 0) continue;	//wait for the adc to be ready
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


