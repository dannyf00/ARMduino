#include "armduino.h"							//include ARMduino defs

#define LED1			PA3						//LED attached to pin PA3/PA4
#define LED2			PA4
#define LED_DLY			100						//waste some time, in ms

//user setup
void setup(void) {
	digitalWrite(LED1, HIGH); pinMode(LED1, OUTPUT);						//set LED as output
	digitalWrite(LED2, LOW); pinMode(LED2, OUTPUT);
}

volatile uint16_t tmp;
//user loop
void loop(void) {
	uint32_t time0, time1;

	//test gpio
	digitalWrite(LED1, !digitalRead(LED1));		//flip LED1
	//digitalWrite(LED2, !digitalRead(LED2));

	//test timing
	time0=ticks();
	delay(LED_DLY);								//waste some time
	time0=ticks() - time0;
	if (time0) NOP();							//break point
}
