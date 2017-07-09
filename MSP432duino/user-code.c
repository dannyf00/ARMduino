#include "armduino.h"							//include ARMduino defs

#define LED			PA8							//LED attached to pin 8/9/10 (=PA8RED/PA9GREEN/PA10BLUE on TI MSP432 Launchpad)
#define LED_DLY		50							//waste some time, in ms

//user setup
void setup(void) {
	pinMode(LED, OUTPUT);						//set LED as output
}

//user loop
void loop(void) {
	digitalWrite(LED, !digitalRead(LED));		//flip LED
	delay(LED_DLY);								//waste some time
}
