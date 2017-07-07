#include "armduino.h"							//include ARMduino defs

#define LED1			PA3						//LED attached to pin PA3/PA4
#define LED2			PA4
#define LED_DLY			100						//waste some time, in ms

//flip the led
void led1_flp(void) {
	digitalWrite(LED1, !digitalRead(LED1));
}

void led2_flp(void) {
	digitalWrite(LED2, !digitalRead(LED2));
}

//user setup
void setup(void) {
	digitalWrite(LED1, HIGH); pinMode(LED1, OUTPUT);						//set LED as output
	digitalWrite(LED2, LOW); pinMode(LED2, OUTPUT);

	//adc1_init();
	tim17_init(1);
	tim17_setpr(F_CPU / 100);						//make sure it doesn't overflow - 16bit type
	tim17_act(led1_flp);
	tim16_init(1);
	tim16_setpr(F_CPU / 100 + 100);						//make sure it doesn't overflow - 16bit type
	tim16_act(led2_flp);

	ei();
}

volatile uint16_t tmp;
//user loop
void loop(void) {
	uint32_t time0, time1;
	uint32_t tmp;

	//test gpio
	//digitalWrite(LED1, !digitalRead(LED1));		//flip LED1
	//digitalWrite(LED2, !digitalRead(LED2));

	time0=ticks();
	delay(LED_DLY);
	time1=ticks() - time0;
	if (time1) NOP();
}
