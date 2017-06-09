#include "armduino.h"							//include ARMduino defs

#define LED		PC8						//LED attached to pin 40/41 (=PC8BLUE/9GREEN on STM32VLDiscovery)
#define LED_DLY		100						//waste some time, in ms

//user setup
void setup(void) {
	pinMode(LED, OUTPUT);						//set LED as output
}

//user loop
void loop(void) {
	digitalWrite(LED, !digitalRead(LED));				//flip LED
	delay(LED_DLY);							//waste some time
}
