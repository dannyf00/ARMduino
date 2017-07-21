#include "armduino.h"							//include ARMduino defs
#include "math.h"

#define LED			P0_7							//LED attached to pin 40/41 (=PC8BLUE/9GREEN on STM32VLDiscovery)
#define LED_DLY		100							//waste some time, in ms
#define AIN			0
#define ADC_CNT		1000
#define BTN			P0_0							//user button on pin 0 / PA0

uint16_t adc_results[ADC_CNT];

void led_flp(void) {
	digitalWrite(LED, !digitalRead(LED));		//flip led
}

//user setup
void setup(void) {
	pinMode(LED, OUTPUT);						//set LED as output
}

volatile uint16_t tmp;
//user loop
void loop(void) {
	volatile uint32_t time0, time1;

	//test gpio
	digitalWrite(LED, !digitalRead(LED));		//flip LED

	//test timing
	//delay(LED_DLY);							//waste some time

	//test analogRead()
	time0 = ticks();
	//for (i=0; i<ADC_CNT; i++) adc_results[i]=analogRead(0);
	//digitalWrite(LED, !digitalRead(LED));		//flip LED
	delay(LED_DLY);
	time1 = ticks() - time0;
	if (time1) NOP();							//break point
}
