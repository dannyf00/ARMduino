#include "armduino.h"							//include ARMduino defs
#include "math.h"

#define LED			PC8							//LED attached to pin 40/41 (=PC8BLUE/9GREEN on STM32VLDiscovery)
#define LED_DLY		100							//waste some time, in ms
#define AIN			0
#define ADC_CNT		1000
#define BTN			PA0							//user button on pin 0 / PA0

uint32_t adc_results[ADC_CNT];

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
	//attachInterrupt(BTN, led_flp, FALLING);

/*	tim4_init(10);
	tim4_setpr1(10000ul);
	tim4_act1(led_flp);
	tim4_setpr2(10000ul+100);
	tim4_act2(led_flp);
*/
/*	tim17_init(10);
	tim17_setpr(10000);
	tim17_act(led_flp);
	tim15_init(10);
	tim15_setpr(10000+100);
	tim15_act(led_flp);
*/

	tim6_init(10);
	tim6_setpr(10000);
	tim6_act(led_flp);
	tim7_init(10);
	tim7_setpr(10000+100);
	tim7_act(led_flp);

	ei();
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
/*	tmp0 = micros();
	for (adc_results[0]=0, i=0; i<ADC_CNT; i++) adc_results[0]+=analogRead(AVREFINT);
	tmp1 = micros() - tmp0;
	adc_results[0]/=ADC_CNT;
	adc_results[1] =ADC2mv(adc_results[0]);
	if (tmp1) NOP();							//break point
*/
}
