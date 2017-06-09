#include "armduino.h"							//include ARMduino defs
#include "math.h"

#define LED			PC8							//LED attached to pin 40/41 (=PC8BLUE/9GREEN on STM32VLDiscovery)
#define LED_DLY		100							//waste some time, in ms
#define AIN			0
#define ADC_CNT		1000
#define BTN			PA0							//user button on pin 0 / PA0

uint16_t adc_results[ADC_CNT];

void led_flp(void) {
	digitalWrite(LED, !digitalRead(LED));		//flip led
}

//user setup
void setup(void) {
	pinMode(LED, OUTPUT);						//set LED as output
	//test analogWrite()
	//analogWrite(1, 0x0fff/10);

	//test dacwrite
	//DAC1Write(4096 / 2);						//generate a fixed value on DAC1
	//DAC2Write(4096 / 4);						//generate a fixed value on DAC2

	//test interrupt
	pinMode(BTN, INPUT_PULLDN);					//btn as input, pull-up
	attachInterrupt(BTN, led_flp, FALLING);
}

volatile uint16_t tmp;
//user loop
void loop(void) {
	uint16_t i;
	volatile uint32_t tmp0, tmp1;

	//test gpio
	//digitalWrite(LED, !digitalRead(LED));		//flip LED

	//test timing
	//delay(LED_DLY);								//waste some time

	//test analogRead()
	tmp0 = micros();
	for (i=0; i<ADC_CNT; i++) adc_results[i]=analogRead(0);
	tmp1 = micros() - tmp0;
	if (tmp1) NOP();							//break point
}
