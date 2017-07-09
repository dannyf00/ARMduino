//hardware specific port for ARMduino to LM4F120 family

#include "armduino_lm4f120.h"						//we use aruidno GPIO port for lm4f120
#include "armduino.h"								//use ARMduino specific definitions

//global definitions

//global variables

volatile uint32_t SystemCoreClock = 16000000ul;		//default value for SystemCoreClock

//define your chips here
//declare pins based on chip packaging
//ALL PINS ARE MAPPED, WHETHER THEY EXIST OR NOT
//*********************************************************
//SO MAKE SURE THAT THE PINS YOU PICKED ACTUALLY EXIST FOR YOUR CHIP/PACKAGE
//*********************************************************
//Pin  0..15 -> GPIOA
//Pin 16..31 -> GPIOB
//Pin 32..47 -> GPIOC
//Pin 48..63 -> GPIOD
//Pin 64..79 -> GPIOE
//Pin 80..95 -> GPIOJ
const PIN2GPIO GPIO_PinDef[]={
		{GPIOA, 1<< 0},						//ARMduino Pin  0 = PA0
		{GPIOA, 1<< 1},						//ARMduino Pin  1 = PA1
		{GPIOA, 1<< 2}, 					//ARMduino Pin  2 = PA2
		{GPIOA, 1<< 3},						//ARMduino Pin  3 = PA3
		{GPIOA, 1<< 4},						//ARMduino Pin  4 = PA4
		{GPIOA, 1<< 5},						//ARMduino Pin  5 = PA5
		{GPIOA, 1<< 6},						//ARMduino Pin  6 = PA6
		{GPIOA, 1<< 7},						//ARMduino Pin  7 = PA7

		{GPIOB, 1<< 0},						//ARMduino Pin 16 = PB0
		{GPIOB, 1<< 1},						//ARMduino Pin 17 = PB1
		{GPIOB, 1<< 2},						//ARMduino Pin 18 = PB2
		{GPIOB, 1<< 3},						//ARMduino Pin 19 = PB3
		{GPIOB, 1<< 4},						//ARMduino Pin 20 = PB4
		{GPIOB, 1<< 5},						//ARMduino Pin 21 = PB5
		{GPIOB, 1<< 6},						//ARMduino Pin 22 = PB6
		{GPIOB, 1<< 7},						//ARMduino Pin 23 = PB7

		{GPIOC, 1<< 0},						//ARMduino Pin 32 = PC0
		{GPIOC, 1<< 1},						//ARMduino Pin 33 = PC1
		{GPIOC, 1<< 2},						//ARMduino Pin 34 = PC2
		{GPIOC, 1<< 3},						//ARMduino Pin 35 = PC3
		{GPIOC, 1<< 4},						//ARMduino Pin 36 = PC4
		{GPIOC, 1<< 5},						//ARMduino Pin 37 = PC5
		{GPIOC, 1<< 6},						//ARMduino Pin 38 = PC6
		{GPIOC, 1<< 7},						//ARMduino Pin 39 = PC7

		{GPIOD, 1<< 0},						//ARMduino Pin 48 = PD0
		{GPIOD, 1<< 1},						//ARMduino Pin 49 = PD1
		{GPIOD, 1<< 2},						//ARMduino Pin 50 = PD2
		{GPIOD, 1<< 3},						//ARMduino Pin 51 = PD3
		{GPIOD, 1<< 4},						//ARMduino Pin 52 = PD4
		{GPIOD, 1<< 5},						//ARMduino Pin 53 = PD5
		{GPIOD, 1<< 6},						//ARMduino Pin 54 = PD6
		{GPIOD, 1<< 7},						//ARMduino Pin 55 = PD7

		{GPIOE, 1<< 0},						//ARMduino Pin 64 = PE0
		{GPIOE, 1<< 1},						//ARMduino Pin 65 = PE1
		{GPIOE, 1<< 2},						//ARMduino Pin 66 = PE2
		{GPIOE, 1<< 3},						//ARMduino Pin 67 = PE3
		{GPIOE, 1<< 4},						//ARMduino Pin 68 = PE4
		{GPIOE, 1<< 5},						//ARMduino Pin 69 = PE5
		{GPIOE, 1<< 6},						//ARMduino Pin 71 = PE6
		{GPIOE, 1<< 7},						//ARMduino Pin 72 = PE7

		{GPIOF, 1<< 0},						//ARMduino Pin 64 = PF0
		{GPIOF, 1<< 1},						//ARMduino Pin 65 = PF1
		{GPIOF, 1<< 2},						//ARMduino Pin 66 = PF2
		{GPIOF, 1<< 3},						//ARMduino Pin 67 = PF3
		{GPIOF, 1<< 4},						//ARMduino Pin 68 = PF4
		{GPIOF, 1<< 5},						//ARMduino Pin 69 = PF5
		{GPIOF, 1<< 6},						//ARMduino Pin 71 = PF6
		{GPIOF, 1<< 7},						//ARMduino Pin 72 = PF7

		{GPIOG, 1<< 0},						//ARMduino Pin 64 = PG0
		{GPIOG, 1<< 1},						//ARMduino Pin 65 = PG1
		{GPIOG, 1<< 2},						//ARMduino Pin 66 = PG2
		{GPIOG, 1<< 3},						//ARMduino Pin 67 = PG3
		{GPIOG, 1<< 4},						//ARMduino Pin 68 = PG4
		{GPIOG, 1<< 5},						//ARMduino Pin 69 = PG5
		{GPIOG, 1<< 6},						//ARMduino Pin 71 = PG6
		{GPIOG, 1<< 7},						//ARMduino Pin 72 = PG7

		{GPIOH, 1<< 0},						//ARMduino Pin 64 = PH0
		{GPIOH, 1<< 1},						//ARMduino Pin 65 = PH1
		{GPIOH, 1<< 2},						//ARMduino Pin 66 = PH2
		{GPIOH, 1<< 3},						//ARMduino Pin 67 = PH3
		{GPIOH, 1<< 4},						//ARMduino Pin 68 = PH4
		{GPIOH, 1<< 5},						//ARMduino Pin 69 = PH5
		{GPIOH, 1<< 6},						//ARMduino Pin 71 = PH6
		{GPIOH, 1<< 7},						//ARMduino Pin 72 = PH7

#if defined(GPIOI)
		{GPIOI, 1<< 0},						//ARMduino Pin 81 = PF0
		{GPIOI, 1<< 1},						//ARMduino Pin 82 = PF1
		{GPIOI, 1<< 2},						//ARMduino Pin 83 = PF2
		{GPIOI, 1<< 3},						//ARMduino Pin 84 = PF3
		{GPIOI, 1<< 4},						//ARMduino Pin 85 = PF4
		{GPIOI, 1<< 5},						//ARMduino Pin 86 = PF5
		{GPIOI, 1<< 6},						//ARMduino Pin 87 = PF6
		{GPIOI, 1<< 7},						//ARMduino Pin 88 = PF7
#endif

		{GPIOJ, 1<< 0},						//ARMduino Pin 81 = PF0
		{GPIOJ, 1<< 1},						//ARMduino Pin 82 = PF1
		{GPIOJ, 1<< 2},						//ARMduino Pin 83 = PF2
		{GPIOJ, 1<< 3},						//ARMduino Pin 84 = PF3
		{GPIOJ, 1<< 4},						//ARMduino Pin 85 = PF4
		{GPIOJ, 1<< 5},						//ARMduino Pin 86 = PF5
		{GPIOJ, 1<< 6},						//ARMduino Pin 87 = PF6
		{GPIOJ, 1<< 7},						//ARMduino Pin 88 = PF7

		{GPIOK, 1<< 0},						//ARMduino Pin 64 = PK0
		{GPIOK, 1<< 1},						//ARMduino Pin 65 = PK1
		{GPIOK, 1<< 2},						//ARMduino Pin 66 = PK2
		{GPIOK, 1<< 3},						//ARMduino Pin 67 = PK3
		{GPIOK, 1<< 4},						//ARMduino Pin 68 = PK4
		{GPIOK, 1<< 5},						//ARMduino Pin 69 = PK5
		{GPIOK, 1<< 6},						//ARMduino Pin 71 = PK6
		{GPIOK, 1<< 7},						//ARMduino Pin 72 = PK7

};

//ain pin def
//needs to match AIN_TypeDef in order
const AIN2GPIO AIN_PinDef[]= {
		{PE3, ADC_CTL_CH0},			//PE3, CH0
		{PE2, ADC_CTL_CH1},			//PE3, CH1
		{PE1, ADC_CTL_CH2},			//PE3, CH2
		{PE0, ADC_CTL_CH3},			//PE3, CH3
		{PD3, ADC_CTL_CH4},			//PE3, CH4
		{PD2, ADC_CTL_CH5},			//PE3, CH5
		{PD1, ADC_CTL_CH6},			//PE3, CH6
		{PD0, ADC_CTL_CH7},			//PE3, CH7
		{PE5, ADC_CTL_CH8},			//PE3, CH8
		{PE4, ADC_CTL_CH9},			//PE3, CH9
		{PB4, ADC_CTL_CH10},			//PE3, CH10
		{PB5, ADC_CTL_CH11},			//PE3, CH11
		//{GPIOE, 1<<3, ADC_CTL_CH12},			//PE3, CH12
		//{GPIOE, 1<<3, ADC_CTL_CH13},			//PE3, CH13
		//{GPIOE, 1<<3, ADC_CTL_CH14},			//PE3, CH14
		//{GPIOE, 1<<3, ADC_CTL_CH15},			//PE3, CH15
		//{GPIOE, 1<<3, ADC_CTL_CH16},			//PE3, CH16
		//{GPIOE, 1<<3, ADC_CTL_CH17},			//PE3, CH17
		//{GPIOE, 1<<3, ADC_CTL_CH18},			//PE3, CH18
		//{GPIOE, 1<<3, ADC_CTL_CH19},			//PE3, CH19
		//{GPIOE, 1<<3, ADC_CTL_CH20},			//PE3, CH20
		//{GPIOE, 1<<3, ADC_CTL_CH21},			//PE3, CH21
		//{GPIOE, 1<<3, ADC_CTL_CH22},			//PE3, CH22
		//{GPIOE, 1<<3, ADC_CTL_CH23},			//PE3, CH23
		//ATEMP defined separately
};

//pwm output pin def
//order has to match PWM_TypeDef
const PWM2GPIO PWM_PinDef[]={
		{PB6, TIMER0_BASE, TIMER_A, GPIO_PB6_T0CCP0},		//PB6 = T0CCP0
		{PF0, TIMER0_BASE, TIMER_A, GPIO_PF0_T0CCP0},		//PF0 = T0CCP0
		{PB7, TIMER0_BASE, TIMER_B, GPIO_PB7_T0CCP1},		//PB7 = T0CCP1
		{PF1, TIMER0_BASE, TIMER_B, GPIO_PF1_T0CCP1},		//PF1 = T0CCP1

		{PF2, TIMER1_BASE, TIMER_A, GPIO_PF2_T1CCP0},		//PF2 = T1CCP0
		{PB4, TIMER1_BASE, TIMER_A, GPIO_PB4_T1CCP0},		//PB4 = T1CCP0
		{PF3, TIMER1_BASE, TIMER_B, GPIO_PF3_T1CCP1},		//PF3 = T1CCP1
		{PB5, TIMER1_BASE, TIMER_B, GPIO_PB5_T1CCP1},		//PB5 = T1CCP1

		{PF4, TIMER2_BASE, TIMER_A, GPIO_PF4_T2CCP0},		//PF4 = T2CCP0
		{PB0, TIMER2_BASE, TIMER_A, GPIO_PB0_T2CCP0},		//PB0 = T2CCP0
		{PB1, TIMER2_BASE, TIMER_B, GPIO_PB1_T2CCP1},		//PB1 = T2CCP1

		{PB2, TIMER3_BASE, TIMER_A, GPIO_PB2_T3CCP0},		//PB3 = T3CCP0
		{PB3, TIMER3_BASE, TIMER_B, GPIO_PB3_T3CCP1},		//PB3 = T3CCP1

		{PC0, TIMER4_BASE, TIMER_A, GPIO_PC0_T4CCP0},		//PC0 = T4CCP0
		{PC1, TIMER4_BASE, TIMER_B, GPIO_PC1_T4CCP1},		//PC1 = T4CCP1

		{PC2, TIMER5_BASE, TIMER_A, GPIO_PC2_T5CCP0},		//PC2 = T5CCP0
		{PC3, TIMER5_BASE, TIMER_B, GPIO_PC3_T5CCP1},		//PC3 = T5CCP1

		{PC4, WTIMER0_BASE, TIMER_A, GPIO_PC4_WT0CCP0},	//PC4 = WT0CCP0
		{PC5, WTIMER0_BASE, TIMER_B, GPIO_PC5_WT0CCP1},	//PC5 = WT0CCP1

		{PC6, WTIMER1_BASE, TIMER_A, GPIO_PC6_WT1CCP0},	//PC6 = WT1CCP0
		{PC7, WTIMER1_BASE, TIMER_B, GPIO_PC7_WT1CCP1},	//PC7 = WT1CCP1

		{PD0, WTIMER2_BASE, TIMER_A, GPIO_PD0_WT2CCP0},	//PD0 = WT2CCP0
		{PD1, WTIMER2_BASE, TIMER_B, GPIO_PD1_WT2CCP1},	//PD1 = WT2CCP1

		{PD2, WTIMER3_BASE, TIMER_A, GPIO_PD2_WT3CCP0},	//PD2 = WT3CCP0
		{PD3, WTIMER3_BASE, TIMER_B, GPIO_PD3_WT3CCP1},	//PD3 = WT3CCP1

		{PD4, WTIMER4_BASE, TIMER_A, GPIO_PD4_WT4CCP0},	//PD4 = WT4CCP0
		{PD5, WTIMER4_BASE, TIMER_B, GPIO_PD5_WT4CCP1},	//PD5 = WT4CCP1

		{PD6, WTIMER5_BASE, TIMER_A, GPIO_PD6_WT5CCP0},	//PD6 = WT5CCP0
		{PD7, WTIMER5_BASE, TIMER_B, GPIO_PD7_WT5CCP1},	//PD7 = WT5CCP1
};

//uart pins for uart0..7
const UART2GPIO UART_PinDef[]={
		{/*SYSCTL_PERIPH_UART0, UART0_BASE, */PA0, GPIO_PA0_U0RX, PA1, GPIO_PA1_U0TX},
		{/*SYSCTL_PERIPH_UART1, UART1_BASE, */PB0, GPIO_PB0_U1RX, PB1, GPIO_PB1_U1TX},
		{/*SYSCTL_PERIPH_UART2, UART2_BASE, */PD6, GPIO_PD6_U2RX, PD7, GPIO_PD7_U2TX},
		{/*SYSCTL_PERIPH_UART3, UART3_BASE, */PC6, GPIO_PC6_U3RX, PC7, GPIO_PC7_U3TX},
		{/*SYSCTL_PERIPH_UART4, UART4_BASE, */PC4, GPIO_PC4_U4RX, PC5, GPIO_PC5_U4TX},
		{/*SYSCTL_PERIPH_UART5, UART5_BASE, */PE4, GPIO_PE4_U5RX, PE5, GPIO_PE5_U5TX},
		{/*SYSCTL_PERIPH_UART6, UART6_BASE, */PD4, GPIO_PD4_U6RX, PD5, GPIO_PD5_U6TX},
		{/*SYSCTL_PERIPH_UART7, UART7_BASE, */PE0, GPIO_PE0_U7RX, PE1, GPIO_PE1_U7TX},
};

//spi pins
const SPI2GPIO SPI_PinDef[]={						//sck/sdo/sdi
	{PA2, GPIO_PA2_SSI0CLK}, {PA5, GPIO_PA5_SSI0TX}, {PA4, GPIO_PA5_SSI0TX},				//spi0
	{PF2, GPIO_PF2_SSI1CLK}, {PF1, GPIO_PF1_SSI1TX}, {PF0, GPIO_PF0_SSI1RX},				//spi1
	{PB4, GPIO_PB4_SSI2CLK}, {PB7, GPIO_PB7_SSI2TX}, {PB6, GPIO_PB6_SSI2RX},				//spi2
	{PD0, GPIO_PD0_SSI3CLK}, {PD3, GPIO_PD3_SSI3TX}, {PD2, GPIO_PD2_SSI3RX},				//spi3
	//PD0, PD3, PD2,								//spi1 alternative
};


//Arduino Functions: GPIO
//set a pin mode to INPUT or OUTPUT
//no error checking on PIN
inline void pinMode(PIN_TypeDef pin, uint8_t mode) {
	//if (pin>=PINMAX) return;				//invalid pin number
	switch (mode) {
	case OUTPUT:			GIO_OUT(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask); break;
	case INPUT_PULLUP:		GIO_IN(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask); GPIOPadConfigSet(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); break;
	case INPUT:
	default: 				GIO_IN(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask); break;
	}
}

//set / clear a pin
inline void digitalWrite(PIN_TypeDef pin, uint8_t val) {
	if (val==LOW) GIO_CLR(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);
	else GIO_SET(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);
}

//read a pin
inline int digitalRead(PIN_TypeDef pin) {
	return (GIO_GET(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask))?HIGH:LOW;
}
//end GPIO

//analogIO
//pwm output
void analogWrite(PWMPIN_TypeDef pwmpin, uint16_t dc) {
	uint8_t inversion=1;								//defualt to inversion
	PIN_TypeDef pin;

	dc = (dc > PWMOUT_PR)?PWMOUT_PR:dc;					//bound duty cycle to PWMOUT_PR
	//there is a bug in the pwm generator.
	//for 100% dc, set dc to 0, and inversion to 0
	if (dc == PWMOUT_PR) dc = inversion = 0;

	if (pwmpin >= PWMMAX) return;							//invalid pin
	//no error checking on pin
	pin = PWM_PinDef[pwmpin].pin;						//get the pin
	TimerControlLevel(PWM_PinDef[pwmpin].timer, PWM_PinDef[pwmpin].ch, inversion);	//invert the output
	TimerMatchSet(PWM_PinDef[pwmpin].timer, PWM_PinDef[pwmpin].ch, dc);				//set duty cy cle
	GPIOPinConfigure(PWM_PinDef[pwmpin].pinconfig); GPIOPinTypeTimer(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);	//configure gpio pin as TnCCPn

}

#if defined(USE_ADC0) || defined(USE_ADC1)
//adc conversion on ain channel using ADCn_BASE
//AIN0/PE3, AIN1/PE2, AIN2/PE1, AIN3/PE0, AIN4/PD3, AIN5/PD2,
//AIN6/PD1, AIN7/PD0, AIN8/PE5, AIN9/PE4, AIN10/PB4, AIN11/PB5
//ATEMP: Temperature sensor
uint32_t analognRead(uint32_t ADCn_BASE, AIN_TypeDef ain) {
    uint32_t tmp;
    uint32_t seq = (ADCn_BASE == ADC0_BASE)?ADC0_SEQ:ADC1_SEQ;
    PIN_TypeDef pin;

    //make sure that adc isn't busy
    //while (ADCBusy(ADCn_BASE)) continue;

    //disable adc sequencing 0
    ADCSequenceDisable(ADCn_BASE, seq);      //disable sequencing for ANALOGREAD
    //clear ADCInt flag
    ADCIntClear(ADCn_BASE, seq);             //clear the flag
    ain = (ain >= AINMAX)?ATEMP:ain;			//default input is ATEMP
    switch (ain) {
    case ATEMP:
        ADCSequenceStepConfigure(ADCn_BASE, seq, 0, ADC_CTL_TS | ADC_CTL_IE | ADC_CTL_END);
        break;
    default:
        pin = AIN_PinDef[ain].pin;
		GPIOPinTypeADC(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);
        //configure adc sequency
        ADCSequenceStepConfigure(ADCn_BASE, seq, 0, GPIO_PinDef[pin].mask | ADC_CTL_IE | ADC_CTL_END);
        break;
    }
   	ADCSequenceEnable(ADCn_BASE, seq);       		//enable the sequency
    ADCProcessorTrigger(ADCn_BASE, seq);     		//start the conversion
    while (!ADCIntStatus(ADCn_BASE, seq, false)) continue;   //wait for the conversion to finish
    ADCIntClear(ADCn_BASE, seq);             		//clear the flag
    ADCSequenceDataGet(ADCn_BASE, seq, &tmp);    	//obtain the adc reult
    return tmp;                                     //return the adc result
}

//select analog reference
void analognReference(uint32_t ADCn_BASE, AREF_TypeDef aref) {
	switch (aref) {
	case DEFAULT:
	case INTERNAL: ADCReferenceSet(ADCn_BASE, ADC_REF_INT); break;		//use internal 3v reference
	case EXTERNAL: ADCReferenceSet(ADCn_BASE, ADC_REF_EXT_3V); break;	//use external 3v reference
	default: break;
	}
}
#endif

//initialize UART: 8-bit data, 1 stop bit, no parity
void serialnBegin(uint32_t uart, uint32_t bps) {
	uint32_t periph;
	PIN_TypeDef pin_rx, pin_tx;
	uint32_t gpio, mask, pinconfig_rx, pinconfig_tx;

	//if (uart >= UARTMAX) return;							//invalid uart number
	switch (uart) {
	case UART0_BASE: periph = SYSCTL_PERIPH_UART0; pin_rx = UART_PinDef[0].pin_rx; pin_tx = UART_PinDef[0].pin_tx; pinconfig_rx = UART_PinDef[0].pinconfig_rx; pinconfig_tx = UART_PinDef[0].pinconfig_tx; break;
	case UART1_BASE: periph = SYSCTL_PERIPH_UART1; pin_rx = UART_PinDef[1].pin_rx; pin_tx = UART_PinDef[1].pin_tx; pinconfig_rx = UART_PinDef[1].pinconfig_rx; pinconfig_tx = UART_PinDef[1].pinconfig_tx; break;
	case UART2_BASE: periph = SYSCTL_PERIPH_UART2; pin_rx = UART_PinDef[2].pin_rx; pin_tx = UART_PinDef[2].pin_tx; pinconfig_rx = UART_PinDef[2].pinconfig_rx; pinconfig_tx = UART_PinDef[2].pinconfig_tx; break;
	case UART3_BASE: periph = SYSCTL_PERIPH_UART3; pin_rx = UART_PinDef[3].pin_rx; pin_tx = UART_PinDef[3].pin_tx; pinconfig_rx = UART_PinDef[3].pinconfig_rx; pinconfig_tx = UART_PinDef[3].pinconfig_tx; break;
	case UART4_BASE: periph = SYSCTL_PERIPH_UART4; pin_rx = UART_PinDef[4].pin_rx; pin_tx = UART_PinDef[4].pin_tx; pinconfig_rx = UART_PinDef[4].pinconfig_rx; pinconfig_tx = UART_PinDef[4].pinconfig_tx; break;
	case UART5_BASE: periph = SYSCTL_PERIPH_UART5; pin_rx = UART_PinDef[5].pin_rx; pin_tx = UART_PinDef[5].pin_tx; pinconfig_rx = UART_PinDef[5].pinconfig_rx; pinconfig_tx = UART_PinDef[5].pinconfig_tx; break;
	case UART6_BASE: periph = SYSCTL_PERIPH_UART6; pin_rx = UART_PinDef[6].pin_rx; pin_tx = UART_PinDef[6].pin_tx; pinconfig_rx = UART_PinDef[6].pinconfig_rx; pinconfig_tx = UART_PinDef[6].pinconfig_tx; break;
	case UART7_BASE: periph = SYSCTL_PERIPH_UART7; pin_rx = UART_PinDef[7].pin_rx; pin_tx = UART_PinDef[7].pin_tx; pinconfig_rx = UART_PinDef[7].pinconfig_rx; pinconfig_tx = UART_PinDef[7].pinconfig_tx; break;
	default: return;										//invalid uart base address
	}
	//configure UART
    SysCtlPeripheralEnable(periph);
    // Wait for the UART0 module to be ready.
    while(!SysCtlPeripheralReady(periph)) continue;
    //configure UART: baud rate, 8-bit data, 1 stop bit, no parity
    //configure rx/tx pins
    gpio = GPIO_PinDef[pin_rx].gpio; mask = GPIO_PinDef[pin_rx].mask; GPIOPinConfigure(pinconfig_rx); GPIOPinTypeUART(gpio, mask);
    gpio = GPIO_PinDef[pin_tx].gpio; mask = GPIO_PinDef[pin_tx].mask; GPIOPinConfigure(pinconfig_tx); GPIOPinTypeUART(gpio, mask);
    UARTConfigSetExpClk(uart, SysCtlClockGet(), bps, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    UARTEnable(uart);										//enable uart
}

//write a string over serial
void serialnPrint(uint32_t uart, char *str) {
	while (*str) UARTCharPut(uart, *str++);
}

//write a string over serial + return
void serialnPrintln(uint32_t uart, char *str) {
	serialnPrint(uart, str);
	serialnPrint(uart, "\n\r");
}

//attach gpio interrupts
//all pins on the same port share the same isr ->
//the last isr installed by the user for that port prevails
void attachInterrupt(PIN_TypeDef pin, void (*isr)(void), uint32_t mode) {
	//all gpio port already activated.
	//pin pull-up / float / input mode assumed

	if (pin >= PINMAX) return;							//return if invalid pin specified
	switch (mode) {
	case RISING: 	mode = GPIO_RISING_EDGE; break;
	case FALLING: 	mode = GPIO_FALLING_EDGE; break;
	case CHANGE: 	mode = GPIO_BOTH_EDGES; break;
	default: break;
	}

	//disable interrupt
	GPIOIntDisable(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);		//disable interrupt for the pin -> just in case
	GPIOIntClear(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);			//clear pending interrupt flag
	GPIOIntTypeSet(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask, mode);	//specify interrupt mode
	GPIOIntRegister(GPIO_PinDef[pin].gpio, isr);						//register user isr
	GPIOIntEnable(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);		//enable interrupt
}

//detach gpio interrupt
void detachInterrupt(PIN_TypeDef pin) {
	uint32_t mode = GPIO_FALLING_EDGE;					//default sensitivity
	if (pin >= PINMAX) return;							//return if invalid pin specified
	//switch (mode) {
	//case RISING: 	mode = GPIO_RISING_EDGE; break;
	//case FALLING: 	mode = GPIO_FALLING_EDGE; break;
	//case CHANGE: 	mode = GPIO_BOTH_EDGES; break;
	//default: break;
	//}

	//disable interrupt
	GPIOIntDisable(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);		//disable interrupt for the pin -> just in case
	GPIOIntClear(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);			//clear pending interrupt flag
	GPIOIntTypeSet(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask, mode);	//specify interrupt mode
	GPIOIntUnregister(GPIO_PinDef[pin].gpio/*, isr*/);						//unregister user isr
	//GPIOIntEnable(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);		//disable interrupt
}


//spi routines
uint32_t SPInWrite(uint32_t spi, uint32_t dat) {
	uint32_t tmp;
	SSIDataPut(spi, dat);
	SSIDataGet(spi, &tmp);
	return tmp;
}

//initialize the chip
void chip_init(void) {
	//select the clock source
	//or use default clock
	//configure systl clock to use xtal
	//LM4F120 launchpad uses a 16Mhz xtal.
	SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);	//use xtal/main oscillator
	//SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_INT);							//use internal oscillator
	//resulting clock = 200Mhz / 10;

	//enable GPIOx
	//default GPIO clock turned off
	if ( SysCtlPeripheralPresent(SYSCTL_PERIPH_GPIOA)) SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	if ( SysCtlPeripheralPresent(SYSCTL_PERIPH_GPIOB)) SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	if ( SysCtlPeripheralPresent(SYSCTL_PERIPH_GPIOC)) SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	if ( SysCtlPeripheralPresent(SYSCTL_PERIPH_GPIOD)) SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	if ( SysCtlPeripheralPresent(SYSCTL_PERIPH_GPIOE)) SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	if ( SysCtlPeripheralPresent(SYSCTL_PERIPH_GPIOF)) SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	if ( SysCtlPeripheralPresent(SYSCTL_PERIPH_GPIOG)) SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
	if ( SysCtlPeripheralPresent(SYSCTL_PERIPH_GPIOH)) SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
#if defined(GPIOI)
	if ( SysCtlPeripheralPresent(SYSCTL_PERIPH_GPIOI)) SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOI);
#endif
	if ( SysCtlPeripheralPresent(SYSCTL_PERIPH_GPIOJ)) SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);

	//configure pwm
#if defined(USE_PWMOUT0)
	//don't yet enable the gpio pins
	//GPIOPinConfigure(GPIO_PF2_T1CCP0); GPIOPinTypeTimer(GPIOF, 1<<2);	//configure PF2 to be T1CCP0
	//GPIOPinConfigure(GPIO_PF3_T1CCP1); GPIOPinTypeTimer(GPIOF, 1<<3);	//configure PF3 to be T1CCP1
	//enable clock to TMR0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)) continue;	//wait for peripheral to be ready
	TimerDisable(TIMER0_BASE, TIMER_BOTH);						//stop timers
	TimerPrescaleSet(TIMER0_BASE, TIMER_BOTH, 0000);			//set prescaler
	TimerControlLevel(TIMER0_BASE, TIMER_BOTH, 1);				//invert the output
	TimerLoadSet(TIMER0_BASE, TIMER_BOTH, PWMOUT_PR);			//set period
	TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);
	//enable timer
	TimerEnable(TIMER0_BASE, TIMER_BOTH);						//start timer a

	TimerMatchSet(TIMER0_BASE, TIMER_BOTH, 0);					//set duty cycle to 0
#endif

#if defined(USE_PWMOUT1)
	//don't yet enable the gpio pins
	//GPIOPinConfigure(GPIO_PF2_T1CCP0); GPIOPinTypeTimer(GPIOF, 1<<2);	//configure PF2 to be T1CCP0
	//GPIOPinConfigure(GPIO_PF3_T1CCP1); GPIOPinTypeTimer(GPIOF, 1<<3);	//configure PF3 to be T1CCP1
	//enable clock to TMR0A
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER1)) continue;	//wait for peripheral to be ready
	TimerDisable(TIMER1_BASE, TIMER_BOTH);						//stop timers
	TimerPrescaleSet(TIMER1_BASE, TIMER_BOTH, 0000);			//set prescaler
	TimerControlLevel(TIMER1_BASE, TIMER_BOTH, 1);				//invert the output
	TimerLoadSet(TIMER1_BASE, TIMER_BOTH, PWMOUT_PR);			//set period
	TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);
	//enable timer
	TimerEnable(TIMER1_BASE, TIMER_BOTH);						//start timer a

	TimerMatchSet(TIMER1_BASE, TIMER_BOTH, 0);					//set duty cycle to 0
#endif

#if defined(USE_PWMOUT2)
	//don't yet enable the gpio pins
	//GPIOPinConfigure(GPIO_PF2_T1CCP0); GPIOPinTypeTimer(GPIOF, 1<<2);	//configure PF2 to be T1CCP0
	//GPIOPinConfigure(GPIO_PF3_T1CCP1); GPIOPinTypeTimer(GPIOF, 1<<3);	//configure PF3 to be T1CCP1
	//enable clock to TMR0A
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER2)) continue;	//wait for peripheral to be ready
	TimerDisable(TIMER2_BASE, TIMER_BOTH);						//stop timers
	TimerPrescaleSet(TIMER2_BASE, TIMER_BOTH, 0000);			//set prescaler
	TimerControlLevel(TIMER2_BASE, TIMER_BOTH, 1);				//invert the output
	TimerLoadSet(TIMER2_BASE, TIMER_BOTH, PWMOUT_PR);			//set period
	TimerConfigure(TIMER2_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);
	//enable timer
	TimerEnable(TIMER2_BASE, TIMER_BOTH);						//start timer a

	TimerMatchSet(TIMER2_BASE, TIMER_BOTH, 0);					//set duty cycle to 0
#endif

#if defined(USE_PWMOUT3)
	//don't yet enable the gpio pins
	//GPIOPinConfigure(GPIO_PF2_T1CCP0); GPIOPinTypeTimer(GPIOF, 1<<2);	//configure PF2 to be T1CCP0
	//GPIOPinConfigure(GPIO_PF3_T1CCP1); GPIOPinTypeTimer(GPIOF, 1<<3);	//configure PF3 to be T1CCP1
	//enable clock to TMR0A
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER3)) continue;	//wait for peripheral to be ready
	TimerDisable(TIMER3_BASE, TIMER_BOTH);						//stop timers
	TimerPrescaleSet(TIMER3_BASE, TIMER_BOTH, 0000);			//set prescaler
	TimerControlLevel(TIMER3_BASE, TIMER_BOTH, 1);				//invert the output
	TimerLoadSet(TIMER3_BASE, TIMER_BOTH, PWMOUT_PR);			//set period
	TimerConfigure(TIMER3_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);
	//enable timer
	TimerEnable(TIMER3_BASE, TIMER_BOTH);						//start timer a

	TimerMatchSet(TIMER3_BASE, TIMER_BOTH, 0);					//set duty cycle to 0
#endif

#if defined(USE_PWMOUT4)
	//don't yet enable the gpio pins
	//GPIOPinConfigure(GPIO_PF2_T1CCP0); GPIOPinTypeTimer(GPIOF, 1<<2);	//configure PF2 to be T1CCP0
	//GPIOPinConfigure(GPIO_PF3_T1CCP1); GPIOPinTypeTimer(GPIOF, 1<<3);	//configure PF3 to be T1CCP1
	//enable clock to TMR0A
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER4)) continue;	//wait for peripheral to be ready
	TimerDisable(TIMER4_BASE, TIMER_BOTH);						//stop timers
	TimerPrescaleSet(TIMER4_BASE, TIMER_BOTH, 0000);			//set prescaler
	TimerControlLevel(TIMER4_BASE, TIMER_BOTH, 1);				//invert the output
	TimerLoadSet(TIMER4_BASE, TIMER_BOTH, PWMOUT_PR);			//set period
	TimerConfigure(TIMER4_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);
	//enable timer
	TimerEnable(TIMER4_BASE, TIMER_BOTH);						//start timer a

	TimerMatchSet(TIMER4_BASE, TIMER_BOTH, 0);					//set duty cycle to 0
#endif

#if defined(USE_PWMOUT5)
	//don't yet enable the gpio pins
	//GPIOPinConfigure(GPIO_PF2_T1CCP0); GPIOPinTypeTimer(GPIOF, 1<<2);	//configure PF2 to be T1CCP0
	//GPIOPinConfigure(GPIO_PF3_T1CCP1); GPIOPinTypeTimer(GPIOF, 1<<3);	//configure PF3 to be T1CCP1
	//enable clock to TMR0A
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER5)) continue;	//wait for peripheral to be ready
	TimerDisable(TIMER5_BASE, TIMER_BOTH);						//stop timers
	TimerPrescaleSet(TIMER5_BASE, TIMER_BOTH, 0000);			//set prescaler
	TimerControlLevel(TIMER5_BASE, TIMER_BOTH, 1);				//invert the output
	TimerLoadSet(TIMER5_BASE, TIMER_BOTH, PWMOUT_PR);			//set period
	TimerConfigure(TIMER5_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);
	//enable timer
	TimerEnable(TIMER5_BASE, TIMER_BOTH);						//start timer a

	TimerMatchSet(TIMER5_BASE, TIMER_BOTH, 0);					//set duty cycle to 0
#endif

#if defined(USE_PWMOUTW0)
	//don't yet enable the gpio pins
	//GPIOPinConfigure(GPIO_PF2_T1CCP0); GPIOPinTypeWTIMER(GPIOF, 1<<2);	//configure PF2 to be T1CCP0
	//GPIOPinConfigure(GPIO_PF3_T1CCP1); GPIOPinTypeWTIMER(GPIOF, 1<<3);	//configure PF3 to be T1CCP1
	//enable clock to TMR0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_WTIMER0)) continue;	//wait for peripheral to be ready
	TimerDisable(WTIMER0_BASE, TIMER_BOTH);						//stop WTIMERs
	TimerPrescaleSet(WTIMER0_BASE, TIMER_BOTH, 0000);			//set prescaler
	TimerControlLevel(WTIMER0_BASE, TIMER_BOTH, 1);				//invert the output
	TimerLoadSet(WTIMER0_BASE, TIMER_BOTH, PWMOUT_PR);			//set period
	TimerConfigure(WTIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);
	//enable WTIMER
	TimerEnable(WTIMER0_BASE, TIMER_BOTH);						//start WTIMER a

	TimerMatchSet(WTIMER0_BASE, TIMER_BOTH, 0);					//set duty cycle to 0
#endif

#if defined(USE_PWMOUTW1)
	//don't yet enable the gpio pins
	//GPIOPinConfigure(GPIO_PF2_T1CCP0); GPIOPinTypeWTIMER(GPIOF, 1<<2);	//configure PF2 to be T1CCP0
	//GPIOPinConfigure(GPIO_PF3_T1CCP1); GPIOPinTypeWTIMER(GPIOF, 1<<3);	//configure PF3 to be T1CCP1
	//enable clock to TMR0A
	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_WTIMER1)) continue;	//wait for peripheral to be ready
	TimerDisable(WTIMER1_BASE, TIMER_BOTH);						//stop WTIMERs
	TimerPrescaleSet(WTIMER1_BASE, TIMER_BOTH, 0000);			//set prescaler
	TimerControlLevel(WTIMER1_BASE, TIMER_BOTH, 1);				//invert the output
	TimerLoadSet(WTIMER1_BASE, TIMER_BOTH, PWMOUT_PR);			//set period
	TimerConfigure(WTIMER1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);
	//enable WTIMER
	TimerEnable(WTIMER1_BASE, TIMER_BOTH);						//start WTIMER a

	TimerMatchSet(WTIMER1_BASE, TIMER_BOTH, 0);					//set duty cycle to 0
#endif

#if defined(USE_PWMOUTW2)
	//don't yet enable the gpio pins
	//GPIOPinConfigure(GPIO_PF2_T1CCP0); GPIOPinTypeWTIMER(GPIOF, 1<<2);	//configure PF2 to be T1CCP0
	//GPIOPinConfigure(GPIO_PF3_T1CCP1); GPIOPinTypeWTIMER(GPIOF, 1<<3);	//configure PF3 to be T1CCP1
	//enable clock to TMR0A
	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER2);
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_WTIMER2)) continue;	//wait for peripheral to be ready
	TimerDisable(WTIMER2_BASE, TIMER_BOTH);						//stop WTIMERs
	TimerPrescaleSet(WTIMER2_BASE, TIMER_BOTH, 0000);			//set prescaler
	TimerControlLevel(WTIMER2_BASE, TIMER_BOTH, 1);				//invert the output
	TimerLoadSet(WTIMER2_BASE, TIMER_BOTH, PWMOUT_PR);			//set period
	TimerConfigure(WTIMER2_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);
	//enable WTIMER
	TimerEnable(WTIMER2_BASE, TIMER_BOTH);						//start WTIMER a

	TimerMatchSet(WTIMER2_BASE, TIMER_BOTH, 0);					//set duty cycle to 0
#endif

#if defined(USE_PWMOUTW3)
	//don't yet enable the gpio pins
	//GPIOPinConfigure(GPIO_PF2_T1CCP0); GPIOPinTypeWTIMER(GPIOF, 1<<2);	//configure PF2 to be T1CCP0
	//GPIOPinConfigure(GPIO_PF3_T1CCP1); GPIOPinTypeWTIMER(GPIOF, 1<<3);	//configure PF3 to be T1CCP1
	//enable clock to TMR0A
	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER3);
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_WTIMER3)) continue;	//wait for peripheral to be ready
	TimerDisable(WTIMER3_BASE, TIMER_BOTH);						//stop WTIMERs
	TimerPrescaleSet(WTIMER3_BASE, TIMER_BOTH, 0000);			//set prescaler
	TimerControlLevel(WTIMER3_BASE, TIMER_BOTH, 1);				//invert the output
	TimerLoadSet(WTIMER3_BASE, TIMER_BOTH, PWMOUT_PR);			//set period
	TimerConfigure(WTIMER3_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);
	//enable WTIMER
	TimerEnable(WTIMER3_BASE, TIMER_BOTH);						//start WTIMER a

	TimerMatchSet(WTIMER3_BASE, TIMER_BOTH, 0);					//set duty cycle to 0
#endif

#if defined(USE_PWMOUTW4)
	//don't yet enable the gpio pins
	//GPIOPinConfigure(GPIO_PF2_T1CCP0); GPIOPinTypeWTIMER(GPIOF, 1<<2);	//configure PF2 to be T1CCP0
	//GPIOPinConfigure(GPIO_PF3_T1CCP1); GPIOPinTypeWTIMER(GPIOF, 1<<3);	//configure PF3 to be T1CCP1
	//enable clock to TMR0A
	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER4);
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_WTIMER4)) continue;	//wait for peripheral to be ready
	TimerDisable(WTIMER4_BASE, TIMER_BOTH);					//stop WTIMERs
	TimerPrescaleSet(WTIMER4_BASE, TIMER_BOTH, 0000);			//set prescaler
	TimerControlLevel(WTIMER4_BASE, TIMER_BOTH, 1);				//invert the output
	TimerLoadSet(WTIMER4_BASE, TIMER_BOTH, PWMOUT_PR);			//set period
	TimerConfigure(WTIMER4_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);
	//enable WTIMER
	TimerEnable(WTIMER4_BASE, TIMER_BOTH);						//start WTIMER a

	TimerMatchSet(WTIMER4_BASE, TIMER_BOTH, 0);					//set duty cycle to 0
#endif

#if defined(USE_PWMOUTW5)
	//don't yet enable the gpio pins
	//GPIOPinConfigure(GPIO_PF2_T1CCP0); GPIOPinTypeWTIMER(GPIOF, 1<<2);	//configure PF2 to be T1CCP0
	//GPIOPinConfigure(GPIO_PF3_T1CCP1); GPIOPinTypeWTIMER(GPIOF, 1<<3);	//configure PF3 to be T1CCP1
	//enable clock to TMR0A
	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER5);
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_WTIMER5)) continue;	//wait for peripheral to be ready
	TimerDisable(WTIMER5_BASE, TIMER_BOTH);						//stop WTIMERs
	TimerPrescaleSet(WTIMER5_BASE, TIMER_BOTH, 0000);			//set prescaler
	TimerControlLevel(WTIMER5_BASE, TIMER_BOTH, 1);				//invert the output
	TimerLoadSet(WTIMER5_BASE, TIMER_BOTH, PWMOUT_PR);			//set period
	TimerConfigure(WTIMER5_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);
	//enable WTIMER
	TimerEnable(WTIMER5_BASE, TIMER_BOTH);						//start WTIMER a

	TimerMatchSet(WTIMER5_BASE, TIMER_BOTH, 0);					//set duty cycle to 0
#endif

	//configure adc
#if defined(USE_ADC0)
	//configure adc0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)) continue;	//wait for peripheral to be ready
	ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PIOSC | ADC_CLOCK_RATE_FULL, ADC_PS);    //ADC_SP [1..64]
	ADCReferenceSet(ADC0_BASE, ADC_REF_INT);						//set reference to Vrefint
	//GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3|GPIO_PIN_2);
    ADCSequenceDisable(ADC0_BASE, ADC0_SEQ);								//disable sequence before enabling it
    ADCSequenceConfigure(ADC0_BASE, ADC0_SEQ, ADC_TRIGGER_PROCESSOR, 0);	//configure sequency 0 to be initialized by the processor / software triggered
    //ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0 | ADC_CTL_END);			//configure sequence 0 to have ch0 as the only channel to be adc'd
    //ADCSequenceEnable(ADC0_BASE, 0);								//enable ADC0
    ADCIntClear(ADC0_BASE, 0);										//clear the int flag -> adc0 not finished
#endif

#if defined(USE_ADC1)
    //configure ADC1
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC1)) continue;	//wait for peripheral to be ready
	ADCClockConfigSet(ADC1_BASE, ADC_CLOCK_SRC_PIOSC | ADC_CLOCK_RATE_FULL, ADC_PS);    //ADC_SP [1..64]
	ADCReferenceSet(ADC1_BASE, ADC_REF_INT);						//set reference to Vrefint
	//GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3|GPIO_PIN_2);
    ADCSequenceDisable(ADC1_BASE, ADC1_SEQ);								//disable sequence before enabling it
    ADCSequenceConfigure(ADC1_BASE, ADC1_SEQ, ADC_TRIGGER_PROCESSOR, 0);	//configure sequency 0 to be initialized by the processor / software triggered
    //ADCSequenceStepConfigure(ADC1_BASE, ADC1_SEQ, 0, ADC_CTL_CH0 | ADC_CTL_END);			//configure sequence 0 to have ch0 as the only channel to be adc'd
    //ADCSequenceEnable(ADC1_BASE, 0);								//enable ADC1
    ADCIntClear(ADC1_BASE, 0);										//clear the int flag -> ADC1 not finished
#endif

    PIN_TypeDef pin;
#if defined(USE_SPI0)
    //configure spi0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    //
    // Wait for the SSI0 module to be ready.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_SSI0)) continue;
    //
    // Configure the SSI.
    //
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI0_BPS, 8);

    //configure the pins
    pin = SPI_PinDef[SPI0SCK].pin; GPIOPinTypeSSI(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask); GPIOPinConfigure(SPI_PinDef[SPI0SCK].pinconfig);
    pin = SPI_PinDef[SPI0SDO].pin; GPIOPinTypeSSI(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask); GPIOPinConfigure(SPI_PinDef[SPI0SDO].pinconfig);
    pin = SPI_PinDef[SPI0SDI].pin; GPIOPinTypeSSI(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask); GPIOPinConfigure(SPI_PinDef[SPI0SDI].pinconfig);

    //
    // Enable the SSI module.
    //
    SSIEnable(SSI0_BASE);
#endif

#if defined(USE_SPI1)
    //configure spi1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
    //
    // Wait for the SSI1 module to be ready.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_SSI1)) continue;
    //
    // Configure the SSI.
    //
    SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI1_BPS, 8);

    //configure the pins
    //PIN_TypeDef pin;
    pin = SPI_PinDef[SPI1SCK].pin; GPIOPinTypeSSI(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask); GPIOPinConfigure(SPI_PinDef[SPI1SCK].pinconfig);
    pin = SPI_PinDef[SPI1SDO].pin; GPIOPinTypeSSI(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask); GPIOPinConfigure(SPI_PinDef[SPI1SDO].pinconfig);
    pin = SPI_PinDef[SPI1SDI].pin; GPIOPinTypeSSI(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask); GPIOPinConfigure(SPI_PinDef[SPI1SDI].pinconfig);

    //
    // Enable the SSI module.
    //
    SSIEnable(SSI1_BASE);
#endif

    //spi2
#if defined(USE_SPI2)
    //configure spi2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
    //
    // Wait for the SSI2 module to be ready.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_SSI2)) continue;
    //
    // Configure the SSI.
    //
    SSIConfigSetExpClk(SSI2_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI2_BPS, 8);

    //configure the pins
    //PIN_TypeDef pin;
    pin = SPI_PinDef[SPI2SCK].pin; GPIOPinTypeSSI(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask); GPIOPinConfigure(SPI_PinDef[SPI2SCK].pinconfig);
    pin = SPI_PinDef[SPI2SDO].pin; GPIOPinTypeSSI(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask); GPIOPinConfigure(SPI_PinDef[SPI2SDO].pinconfig);
    pin = SPI_PinDef[SPI2SDI].pin; GPIOPinTypeSSI(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask); GPIOPinConfigure(SPI_PinDef[SPI2SDI].pinconfig);

    //
    // Enable the SSI module.
    //
    SSIEnable(SSI2_BASE);
#endif

    //spi3
#if defined(USE_SPI3)
    //configure spi3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);
    //
    // Wait for the SSI3 module to be ready.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_SSI3)) continue;
    //
    // Configure the SSI.
    //
    SSIConfigSetExpClk(SSI3_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI3_BPS, 8);

    //configure the pins
    //PIN_TypeDef pin;
    pin = SPI_PinDef[SPI3SCK].pin; GPIOPinTypeSSI(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask); GPIOPinConfigure(SPI_PinDef[SPI3SCK].pinconfig);
    pin = SPI_PinDef[SPI3SDO].pin; GPIOPinTypeSSI(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask); GPIOPinConfigure(SPI_PinDef[SPI3SDO].pinconfig);
    pin = SPI_PinDef[SPI3SDI].pin; GPIOPinTypeSSI(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask); GPIOPinConfigure(SPI_PinDef[SPI3SDI].pinconfig);

    //
    // Enable the SSI module.
    //
    SSIEnable(SSI3_BASE);
#endif


    //update SystemCoreClock - done in mcu_init()
	//SystemCoreClock = SysCtlClockGet();

}
