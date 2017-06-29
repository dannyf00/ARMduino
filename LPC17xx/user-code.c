#include "armduino.h"							//include ARMduino defs
#include "math.h"

#define LED			PA22						//LED attached to pin 0.22
#define LED_DLY		100							//waste some time, in ms

//user setup
void setup(void) {
	pinMode(LED, OUTPUT);						//set LED as output
}

//user loop
void loop(void) {
	volatile uint32_t time0, time1;

	digitalWrite(LED, !digitalRead(LED));		//flip led
	time0 = ticks();							//time stamp time0
	systick_delayms(LED_DLY);					//waste some time
	time1 = ticks() - time0;					//calculate time elapsed
	if (time1) NOP();							//break point
}
