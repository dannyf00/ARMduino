#include "armduino.h"							//include ARMduino defs

#define LEDR		PF1							//LED attached to PF1R/2B/3G
#define LEDB		PF2
#define LEDG		PF3
#define LED_DLY		100							//waste some time, in ms

#define SW1			PF4							//USER_SW1 on Launchpad

char uRAM[50];									//ram buffer for uart

//button read isr
void sw_read(void) {
	//if (GPIOIntStatus(GPIO_PinDef[SW1].gpio, false) & GPIO_PinDef[SW1].mask) {	//sw1 is the source
		GPIOIntClear(GPIO_PinDef[SW1].gpio, GPIO_PinDef[SW1].mask);				//clear the status
		digitalWrite(LEDR, !digitalRead(LEDR));									//sw1 controls ledr
	//}
}

//user setup
void setup(void) {
	pinMode(LEDB, OUTPUT);						//set LED as output
	//analogWrite(PWMPF3, 1);						//generate pwm output

	//uart initialization
	//serial0Begin(38400);						//serial at 38400bps

	//configure ledr/ledg
	//pinMode(LEDR, OUTPUT);
	//pinMode(LEDG, OUTPUT);
	//configure two input pins
	//pinMode(SW1, INPUT_PULLUP);
	//pinMode(SW2, INPUT_PULLUP);
	//attachInterrupt(SW1, sw_read, RISING);		//rising edge on sw1 to trigger sw_read

	//GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
	//GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4,GPIO_STRENGTH_2MA,               GPIO_PIN_TYPE_STD_WPU);
}

//user loop
void loop(void) {
	uint32_t time0, time1, index;

	//flip an led
	time0=ticks();
	for (index=0; index<1000; index++) digitalWrite(LEDB, !digitalRead(LEDB));	//4903us
	//digitalWrite(LEDR, digitalRead(SW1));
	time1=ticks() - time0;

	delay(LED_DLY);

#if 0
	//send data over serial
	sprintf(uRAM, "time1=%4u, %4u\n\r", analog0Read(A0), analog1Read(A1));		//convert time1 to a string
	if (!serial0Busy()) serial0Print(uRAM);
#endif

	//SPI1Write(0x55);
}
