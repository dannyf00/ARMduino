#include "armduino.h"					//we use aruidno port for stm8s

//global definitions

//global variables
//for time base off SysTick (24-bit counter)
//volatile uint32_t timer1_millis = 0;
volatile uint32_t timer_ticks = 0;								//time base on Systick -> SysTick->VAL being the lowest 24-bits
//static uint16_t timer1_fract = 0;
//volatile uint32_t SystemCoreClock=16000000ul/8;				//systemcoreclock, use CMSIS' value

//systick handler - to provide time base for millis()/micros()
void SysTick_Handler(void) {
	//clear the flag
	timer_ticks += 1ul<<24;						//increment systick counter - 24bit, 1:1 prescaler
}

//user-supplied code prototype
extern void setup(void);			//user set-up
extern void loop(void);				//user loop

//Arduino Functions: GPIO
//see chip-specific source/header files
//end GPIO

//Arduino Functions: Time
//systick ticks
uint32_t ticks(void) {
	uint32_t m;					//stores overflow count
	uint32_t f;					//return the fractions / Systick->VAL

	//use double reads
	do {
		m = timer_ticks;
		f = SysTick->VAL;		//24-bit only
	} while (m != timer_ticks);
	//now m and f are atomic
	return (m - f);				//SysTick is a 24-bit downcounter
}

//return microseconds
uint32_t micros(void) {
	uint32_t m;					//stores overflow count
	uint32_t f;					//return the fractions / Systick->VAL
	
	//use double reads
	do {
		m = timer_ticks;
		f = SysTick->VAL;		//24-bit only
	} while (m != timer_ticks);
	//now m and f are atomic
	return (m - f) / clockCyclesPerMicrosecond();					//SysTick is a 24-bit downcounter
}
	
//return milliseconds
//not exact -> 1024us rather than 1000us
uint32_t millis(void) {
	uint32_t m;
	uint32_t f;
	
	//use double reads
	do {
		m = timer_ticks;
		f = SysTick->VAL;				//24-bit only
	} while (m != timer_ticks);
	//not m and f are atomic
	//return ((m + ((1ul<<24) - f)) >> 10) / clockCyclesPerMicrosecond();	//SysTick is a 24-bit downcounter
	return ((m - f) / 1000) / clockCyclesPerMicrosecond();			//SysTick is a 24-bit downcounter
}

//delay milliseconds
void delay(uint32_t ms) {
#if 0												//using millis
	uint32_t start_time = millis();

	while (millis() - start_time < ms) continue;	//wait until desired time runs out
#else												//usingn ticks
	uint32_t start_time = ticks();
	ms *= (SystemCoreClock / 1000);					//convert ms to ticks
	while (ticks() - start_time < ms) continue;
#endif
}

//delay micro seconds
void delayMicroseconds(uint32_t us) {
#if 0												//using micros()
	uint32_t start_time = micros();
	
	while (micros() - start_time < us) continue;	//wait until desired time runs out
#else												//using ticks
	uint32_t start_time = ticks();
	us *= (SystemCoreClock / 1000000ul);			//convert us to ticks
	while (ticks() - start_time < us) continue;
#endif
}
//end Time


//Arduino Functions: Advanced IO
//shift in - from arduino code base / not optimized
uint8_t shiftIn(PIN_TypeDef dataPin, PIN_TypeDef clockPin, uint8_t bitOrder) {
	uint8_t value = 0;
	uint8_t i;

	for (i = 0; i < 8; ++i) {
		digitalWrite(clockPin, HIGH);
		if (bitOrder == LSBFIRST)
			value |= digitalRead(dataPin) << i;
		else
			value |= digitalRead(dataPin) << (7 - i);
		digitalWrite(clockPin, LOW);
	}
	return value;
}

//shift out - from arduino code base / not optimized
void shiftOut(PIN_TypeDef dataPin, PIN_TypeDef clockPin, uint8_t bitOrder, uint8_t val) {
	uint8_t i;

	for (i = 0; i < 8; i++)  {
		if (bitOrder == LSBFIRST)
			digitalWrite(dataPin, !!(val & (1 << i)));
		else	
			digitalWrite(dataPin, !!(val & (1 << (7 - i))));
			
		digitalWrite(clockPin, HIGH);
		digitalWrite(clockPin, LOW);		
	}
}

//end Advanced IO

//initialize the mcu
//reset the mcu
void mcu_init(void){
	//arm core initialization

	//configure Systick as the time base for millis()/micros()
	timer_ticks = 1ul<<24;								//SysTick is a 24-bit downcounter
	//for chips where SysTick_Config() is not defined in cmsis
	SysTick->LOAD  = 	(0xfffffful/*ticks*/ & SysTick_LOAD_RELOAD_Msk) - 1*0;      /* set reload register */
	NVIC_SetPriority 	(SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);  /* set Priority for Systick Interrupt */
	SysTick->VAL   = 	0;                                          /* Load the SysTick Counter Value */
	SysTick->CTRL  = 	SysTick_CTRL_CLKSOURCE_Msk |
						SysTick_CTRL_TICKINT_Msk   |
						SysTick_CTRL_ENABLE_Msk;                    /* Enable SysTick IRQ and SysTick Timer */
	//alternative - for CMSIS-equip'd chips
	//SysTick_Config(SysTick_LOAD_RELOAD_Msk);			//reload all 24 bits

	//enable interrupts
	interrupts();

	//chip initialization
	chip_init();										//defined in chip-specific files

	//Update SystemCoreClock
	SystemCoreClockUpdate();
}

//stylized code for main()
int main(void) {
	mcu_init();							//reset the chip
	setup();							//user-set up the code
	while (1) {
		loop();							//user specified loop
	}
	return 0;
}
