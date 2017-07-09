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
//return timer ticks
uint32_t ticks(void) {
	uint32_t m;
	uint32_t f;

	//use double reads
	do {
		m = timer_ticks;
		f = SysTickValueGet();	//SysTick->VAL;				//24-bit only
	} while (m != timer_ticks);
	//now m and f are atomic
	//return ((m + ((1ul<<24) - f)) >> 10) / clockCyclesPerMicrosecond();	//SysTick is a 24-bit downcounter
	return (m - f);											//SysTick is a 24-bit downcounter
}

//return microseconds
uint32_t micros(void) {
	uint32_t m;					//stores overflow count
	uint32_t f;					//return the fractions / Systick->VAL
	
	//use double reads
	do {
		m = timer_ticks;
		f = SysTickValueGet();	//SysTick->VAL;		//24-bit only
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
		f = SysTickValueGet();	//SysTick->VAL;				//24-bit only
	} while (m != timer_ticks);
	//now m and f are atomic
	//return ((m + ((1ul<<24) - f)) >> 10) / clockCyclesPerMicrosecond();	//SysTick is a 24-bit downcounter
	return ((m - f) / 1000) / clockCyclesPerMicrosecond();			//SysTick is a 24-bit downcounter
}

//delay milliseconds
void delay(uint32_t ms) {
#if 0												//typical implementation
	uint32_t start_time = millis();

	while (millis() - start_time < ms) continue;	//wait until desired time runs out
#else												//implementation over ticks()
	uint32_t start_time = ticks();
	ms *= SystemCoreClock / 1000;					//convert ms to ticks

	while (ticks() - start_time < ms) continue;		//wait until desired time runs out
#endif
}

//delay micro seconds
void delayMicroseconds(uint32_t us) {
#if 0												//typical implementation
	uint32_t start_time = micros();
	
	while (micros() - start_time < us) continue;	//wait until desired time runs out
#else												//implementation over ticks()
	uint32_t start_time = ticks();
	us *= SystemCoreClock / 1000000ul;				//convert us to ticks

	while (ticks() - start_time < us) continue;		//wait until the desired time runs out
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
	//delay - to prevent jtag pin lock-out on LM3S/TM4C devices
	//do NOT comment out
	SysCtlDelay(1000000ul);

	//arm core initialization

	//configure Systick as the time base for millis()/micros()
	timer_ticks = 1ul<<24;								//SysTick is a 24-bit downcounter
	//for chips where SysTick_Config() is not defined in cmsis
	SysTickDisable();								//disable systick before configuration
	SysTickPeriodSet(1ul<<24);						//Systick is a 24-bit downcounter
	SysTickIntRegister(SysTick_Handler);				//install systick handler
	SysTickEnable();								//enable systick
	//alternative - for CMSIS-equip'd chips
	//SysTick_Config(SysTick_LOAD_RELOAD_Msk);			//reload all 24 bits

	//enable interrupts
	interrupts();

	//chip initialization
	chip_init();										//defined in chip-specific files

	//Update SystemCoreClock
	SystemCoreClock = SysCtlClockGet();					//SystemCoreClock is updated in mcu_init()
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
