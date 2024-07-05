#include "stm32f4xx.h"

/* System clock: 16Mhz HSI, APB1 and APB2 clock: 16Mhz */
/*==== UART4 is connected in PC10,11==== */
/*==== HCSR04 connected @PA07 trigg, @PA15 @echo TIMER2 CH1 Input Capture mode, PA10 Warning LED===*/
/*==== Servo connected @PB06 TIMER4 CH1 output compare PWM mode===*/
/*=== PD14	O	O	PD11
			PD13	O	O	PD10
			PD12	O	O	PD09 ===Tactile pins==*/
			 
/*=====UART configurations: UART4 used; note: Keep Arduino Tx, Rx pin in INPUT mode, and dump a blank into the arduino connect tx-tx, rx-rx======================================== */
/*=====USART 9600, 1 start bit, 1 stop bit, 8 data bit, 0 parity bit ====*/
/*==== UART4 is connected in PC10,11==== */

void init_UART(void){
	
	RCC->APB1ENR |= 1<<19; //UART4 EN clock
	
	//Multiplex pin of TX PC10, RX 11 to be mapped to UART4
	RCC->AHB1ENR |= 1<<2; //PortC clock enaabled
	GPIOC->MODER &= ~(0xFUL << 20);// set as alternate pin function 
	GPIOC->MODER |= (0xAUL << 20);// set as alternate pin function 
	GPIOC->AFR[1]=0x88UL<<8;//AFRH : AF8 is USART4
	
	// Enable UART ; set M-Bit 0 -> 1start bit, 8bit data, 1stop bit(default)
	UART4->CR1 |= 1<<13; // UE 1
	//set baud rate to 9600. over8=0
	UART4->BRR=0x0683; // DIV=104.1875 or integer part[104] fraction part[3]  (3=0.1875*16)
	
	//enable Receiver buffer not-empty interrupt
	UART4->CR1 |= 1<<5; // RXNE IE
	NVIC_EnableIRQ(UART4_IRQn);
	
	//start Tx & Rx enable. (it will also send 1st frame idle)
	UART4->CR1 |= 1<<3; // TE 1
	UART4->CR1 |= 1<<2; // RE 1

}

void sendChar(int8_t data){
	while (!(UART4->SR&(1<<7))); // wait if TXE is not 0
	UART4->DR = data & 0x000000FF; // make sure only first 8 bits are copied.
}

void sendString(char *StringPtr){ //Send Striing into the transmitter buffer individually
		for(int i=0;StringPtr[i]!='\0';i++)
		sendChar(StringPtr[i]);
}

int8_t receivedData[100]={0x00}; //String storage buffer
int8_t noOfBytesAvailable=0; // holds the detail of noOfBytesReceived
int8_t BufferIndex=0; // temporary vaiable used to hold the index of receivedData

void UART4_IRQHandler(void){   // ISR read reciever buffer
		receivedData[BufferIndex]=UART4->DR;
		BufferIndex++; // increament noOfBytesAvailable
		if(receivedData[BufferIndex-1]=='\n'){noOfBytesAvailable=BufferIndex;BufferIndex=0;} // informByteAvailable= complete string; reset BufferIndex
		if(BufferIndex>100)BufferIndex=0; //overflow reset
}

/*=====Ultrasonic HR-SC04 configurations========================================================================================================================================*/
/*=====Trigger PA07, PA15 echo TIMER2 CH1 Input Capture mode, PA10 Warning LED===*/

/*Initialise timer TIMER2 CH1 -> PA15 as ECHO pin*/
void init_USTimer(void){
	//Config alternate function @PA15
	RCC->AHB1ENR |= 1<<0; //Enable clock in GPIO A
	GPIOA->MODER |= 2<<12; //alternate fn chose
	GPIOA->AFR[1] |= 1<<28; // AF1 is timer function
	
	RCC->APB1ENR |= 1<<0; // TIM2 clock supply enabled
	TIM2->PSC = 16-1; // 16Mhz/16 -> 1Mhz clock source
	
	TIM2->CCMR1 |= 1<<0; // channel 1 as IC1 mapped on TI1
	TIM2->DIER |= 1<<1; // Capture interrupt enable
	NVIC_EnableIRQ(TIM2_IRQn);
	TIM2->CCER |= 1<<0 | 1<<1 | 1<<3; //enable capture, 11-> both edge
	TIM2->CR1 |= 1<<0; //Counter enable
}

void US_GPIOSetup(void) // Trigger PA07, Warning led PA10
{
	RCC->AHB1ENR |= (1UL<<0);//GPIOCEN : Clock enabled in port A
	GPIOA->MODER |= (1UL<<14 | 1UL<<20); // PA07, PA10 as Output
	GPIOA->BSRR=0xFFFF0000; // reset LED
}

uint32_t PosedgeCapturedValue=0x00, NegedgeCapturedValue=0x00; // variables used to capture timestamp and get pulse width of Ultrasonic output
uint32_t TimePeriodCalulated=0x00;
float DistanceCalculated=0.0; // distance = duration * 0.034 / 2 Speed of sound wave divided by 2 (go and back); Observation: Using TIMER2 in IC interrupt, the cm -> count*0.001;

uint8_t posedge=0;
void TIM2_IRQHandler (void){
	if(posedge==0){PosedgeCapturedValue=TIM2->CCR1;posedge=1;} // stamp posedge
	else {
	NegedgeCapturedValue=TIM2->CCR1;posedge=0; // stamp negedge
		//Calculate distance from pulse width count
		if(NegedgeCapturedValue>PosedgeCapturedValue) TimePeriodCalulated=(NegedgeCapturedValue-PosedgeCapturedValue); // check if not any count overflow in between
		else TimePeriodCalulated=((int32_t)0xFFFFFFFF - PosedgeCapturedValue)+NegedgeCapturedValue; // adjustment if overflow
		DistanceCalculated = TimePeriodCalulated/1000; //Observation: Using TIMER2 in IC interrupt, the cm -> count*0.001;
	}
}

void ReadDistance(void){ // Reads the obstacle distance, and warns if distance < threshold
		
		GPIOA->ODR &= ~(1<<7); //reset 1 to TRIGGER
		for(unsigned int i=0;i<1000;i++);//small delay
		GPIOA->ODR |= 1<<7; //set 1 to TRIGGER
		for(unsigned int i=0;i<10;i++);// 10uS delay (experimentally checked)
		GPIOA->ODR &= ~(1<<7); //reset 1 to TRIGGER
		for(unsigned int i=0;i<100000;i++);//small delay
		
		if(DistanceCalculated<10){GPIOA->ODR |= (1<<10);} //Warning LED lit if obstacle distance <10cm
		else GPIOA->ODR &= ~(1<<10); // normalise warning
	
	/*	sendString("UltraSonic: ");
				char int_string[10];
				unsigned int i, cm=DistanceCalculated;
				for(i=0;(cm!=0) && (i<10);i++)
				{
					int_string[9-i]=cm%10 + 48; // 48->'0'
					cm=cm/10;
				}
				for(;i>0;i--)sendChar(int_string[10-i]);
				sendString("cm \n"); */
		
}

/*=== Servo configuration ============================================================================================================================================================*/
/*Initialise timer TIMER4 CH1 at PB06*/
void init_ServoTimer(void){
	//Config alternate function @PB06
	RCC->AHB1ENR |= 1<<1; //Enable clock in GPIO B
	GPIOB->MODER |= 2<<12; //alternate fn chose
	GPIOB->AFR[0] |= 2<<24; // AF2 is timer function
	
	RCC->APB1ENR |= 1<<2; // TIM4 clock supply enabled
	TIM4->PSC = 5-1; // 16Mhz/5 -> 3.2Mhz clock source
	TIM4->ARR = 0xFFFF-1; // 65536 ;=> 48.82Hz ~ 20.48mS	
	
	TIM4->CCMR1 &= ~(7<<4); TIM4->CCMR1 |= (6<<4); 	//SET PWM mode-1
	TIM4->CCMR1 |= 1<<3; //OC1PE access enable to pre-load register upon read/write
	TIM4->CR1 |= 1<<7; // ARPE use a buffer to enable; preload registers are transferred to the shadow registers only when an update event occurs
	TIM4->CCR1=0x12DE; // CCR value to set 90deg/Centre
	TIM4->EGR |= 1<<0; //Update generate to first init registers and counters before starting
	TIM4->CCER |= 1<<0; // Enable output in channel1
	
	//TIM3->DIER=(3UL<<0); // enable interrupt for OC event @CH1 and update event // used for troubleshooting PWM
	//NVIC_EnableIRQ(TIM3_IRQn); // Interrupt enable // used for troubleshooting PWM
	
	TIM4->CR1 |= 1<<0; //Counter enable
}

void SetServoDeg(uint16_t deg){ //(0,180) (0.5mS, 2.5mS) (1500, 8200)
	TIM4->CCR1 = 1500 + (deg*37);
}

void DirectServo(uint8_t dir){ //upon command instruction from OCR, the servo is turned accordingly
	sendString(" Servo: ");
	switch(dir){
		case 'L': SetServoDeg(0);sendChar('0'); break;
		case 'l': SetServoDeg(0);sendChar('0'); break;
		case 'R': SetServoDeg(180);sendString("180"); break;
		case 'r': SetServoDeg(180);sendString("180"); break;
		case 'C': SetServoDeg(90);sendString("90"); break;
		case 'c': SetServoDeg(90);sendString("90"); break;
		case '1': SetServoDeg(18);sendString("18"); break;
		case '2': SetServoDeg(36);sendString("36"); break;
		case '3': SetServoDeg(54);sendString("54"); break;
		case '4': SetServoDeg(73);sendString("73"); break;
		case '5': SetServoDeg(90);sendString("90"); break;
		case '6': SetServoDeg(108);sendString("108"); break;
		case '7': SetServoDeg(126);sendString("126"); break;
		case '8': SetServoDeg(144);sendString("144"); break;
		case '9': SetServoDeg(172);sendString("172"); break;
		case '0': SetServoDeg(0);sendChar('0'); break;
		default: break;
		sendChar('\n');
	}
	
}
/*uint8_t togg=0x0F; // used for troubleshooting PWM
void TIM3_IRQHandler(void){ // TIM3 interrupt service
		TIM3->SR &=~(3<<0); // clear interrupt flag
		togg=~togg;
}*/

/*==== Tactiles configuration ========================================================================================================================================================*/
/*=== PD14	O	O	PD11
			PD13	O	O	PD10
			PD12	O	O	PD09 =====*/

void init_TactileGPIOs(void) // GPIO pins to be initialised
{
	RCC->AHB1ENR |= (1UL<<3);//GPIODEN : Clock enabled in port D
	GPIOD->MODER &= ~(0xFFFUL<<18); GPIOD->MODER |= (0x555UL<<18); // clear bits 29-18; set as output pin9,10,11,12,13,14
	GPIOD->BSRR=0xFFFF0000; // reset all LEDs
}

void printBraillePattern(uint8_t data){ // only valid ascii from Alphabets and numbers and space // this function prints the braile pattern into the tactile
	uint32_t OutputPattern=0x00000000;
	if(data>96)data=data-32; // small letters - 32 -> Capital letter
	switch (data){//					   ......
		case 'A': OutputPattern=0b0100000000000000;break;
		case 'B': OutputPattern=0b0110000000000000;break;
		case 'C': OutputPattern=0b0100100000000000;break;
		case 'D': OutputPattern=0b0100110000000000;break;
		case 'E': OutputPattern=0b0100010000000000;break;
		case 'F': OutputPattern=0b0110100000000000;break;
		case 'G': OutputPattern=0b0110110000000000;break;
		case 'H': OutputPattern=0b0110010000000000;break;
		case 'I': OutputPattern=0b0010100000000000;break;
		case 'J': OutputPattern=0b0010110000000000;break;
		case 'K': OutputPattern=0b0101000000000000;break;
		case 'L': OutputPattern=0b0111000000000000;break;
		case 'M': OutputPattern=0b0101100000000000;break;
		case 'N': OutputPattern=0b0101110000000000;break;
		case 'O': OutputPattern=0b0101010000000000;break;
		case 'P': OutputPattern=0b0111100000000000;break;
		case 'Q': OutputPattern=0b0111110000000000;break;
		case 'R': OutputPattern=0b0111010000000000;break;
		case 'S': OutputPattern=0b0011100000000000;break;
		case 'T': OutputPattern=0b0011110000000000;break;
		case 'U': OutputPattern=0b0101001000000000;break;
		case 'V': OutputPattern=0b0111001000000000;break;
		case 'W': OutputPattern=0b0010111000000000;break;
		case 'X': OutputPattern=0b0101101000000000;break;
		case 'Y': OutputPattern=0b0101111000000000;break;
		case 'Z': OutputPattern=0b0101011000000000;break;
		case '1': OutputPattern=0b0100000000000000;break;
		case '2': OutputPattern=0b0110000000000000;break;
		case '3': OutputPattern=0b0100100000000000;break;
		case '4': OutputPattern=0b0100110000000000;break;
		case '5': OutputPattern=0b0100010000000000;break;
		case '6': OutputPattern=0b0110100000000000;break;
		case '7': OutputPattern=0b0110110000000000;break;
		case '8': OutputPattern=0b0110010000000000;break;
		case '9': OutputPattern=0b0010100000000000;break;
		case '0': OutputPattern=0b0010110000000000;break;
		case '.': OutputPattern=0b0010011000000000;break;
		case ',': OutputPattern=0b0010000000000000;break;
		default : OutputPattern=0b0000000000000000;
		}
	GPIOD->ODR = OutputPattern; // push output
}
			 
void SerialPrintBraille(void){ //Serially prints the buffer char, until then Rx is disabled
	if(noOfBytesAvailable>0){
		UART4->CR1 &= ~(1<<2); // disable Reception
		sendString("Tactile: ");
		
		for(int i=0;i<noOfBytesAvailable && receivedData[i]!='\0';i++){ // iterate through the buffer and print each char into the tactile
			if(receivedData[i]=='#'){ // check if Servo command i.e #LEFT, #9 -> 180deg
				DirectServo(receivedData[i+1]); // push command to servo
				break;
			}
			printBraillePattern(receivedData[i]); // print char into tactile
			ReadDistance(); // check for obstacle
			sendChar(receivedData[i]); // logging purpose
			for(unsigned int i = 0; i < 850000; i++); //delay
		}
		sendChar('\n');
		noOfBytesAvailable=0x00; // after completion clear buffer
		UART4->CR1 |= (1<<2); // enable Reception
		}
	else {ReadDistance();} // else part read obstacle and warn if to
}
/*============================================================================================================================================================================================*/
int main(void){
	init_UART(); // initialise UART as full-duplex, 9600 BR
	init_TactileGPIOs(); // GPIOs of tactile 
	init_ServoTimer(); // Servo timer init
	init_USTimer(); // Ultrasonic module timer init
	US_GPIOSetup(); // Ultrasonic trigger and warning gpio pin init
	
	sendString("/*===BRAILLE GRADE-I INTERPRETER===*/\n");
	sendString("Log: Initialisation done\n");
	
	while(1){
		
		SerialPrintBraille(); // keep reading input buffer and print into tactile
		
	}
	return(0);
}