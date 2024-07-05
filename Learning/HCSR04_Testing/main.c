#include"stm32f4xx.h"

/* sys clock and ABP running in 16MHz */

void GPIOSetupTriggerPin(void) // Trigger PA07
{
	RCC->AHB1ENR |= (1UL<<0);//GPIOCEN : Clock enabled in port A
	GPIOA->MODER |= (1UL<<14); // PA07 as Output
	GPIOA->BSRR=0xFFFF0000; // reset LED
}


/*Initialise timer TIMER2 CH1 -> PA15 as ECHO pin*/
void init_TIMER(void){
	//Config alternate function @PA06
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

uint32_t PosedgeCapturedValue=0x00, NegedgeCapturedValue=0x00;
uint32_t TimePeriodCalulated=0x00;
float DistanceCalculated=0.0;

uint32_t mills=0x00; uint32_t dummy=0x00;

uint8_t posedge=0, togg=0;
void TIM2_IRQHandler (void){
	if(posedge==0){PosedgeCapturedValue=TIM2->CCR1;posedge=1;}
	else {NegedgeCapturedValue=TIM2->CCR1;posedge=0;}
	togg=~togg;
}

int main(void){
	GPIOSetupTriggerPin();
	init_TIMER();
	GPIOA->ODR &= ~(1<<7); //reset 1 to TRIGGER
	for(unsigned int i=0;i<32000;i++);
	int x=1;
	while(1){
		if(x==1){
		GPIOA->ODR &= ~(1<<7); //reset 1 to TRIGGER
		for(unsigned int i=0;i<32000;i++);
		//set Trigger true
		//mills=TIM2->CNT;
		dummy=1;
		GPIOA->ODR |= 1<<7; //set 1 to TRIGGER
		for(unsigned int i=0;i<10;i++);//while(TIM3->CNT - mills < 10); //delay for 10uS
		GPIOA->ODR &= ~(1<<7); //reset 1 to TRIGGER
		dummy=0; x=0;}
		/*
		mills=TIM2->CNT;
		while(!(TIM2->SR & 1<<1)&& TIM2->CNT-mills <0xFFFF); // wait for the posedge capture to occur
		PosedgeCapturedValue = TIM2->CCR1;
		
		mills=TIM2->CNT;
		while(!(TIM2->SR & 1<<1)&& TIM2->CNT-mills <0xFFFF); // wait for the capture to occur
		NegedgeCapturedValue = TIM2->CCR1;
		
		//TimePeriodCalulated=(NegedgeCapturedValue-PosedgeCapturedValue);
		*/
		if(NegedgeCapturedValue>PosedgeCapturedValue) TimePeriodCalulated=(NegedgeCapturedValue-PosedgeCapturedValue);
		else TimePeriodCalulated=((int32_t)0xFFFFFFFF - PosedgeCapturedValue)+NegedgeCapturedValue;
		
		DistanceCalculated = TimePeriodCalulated/1000; //(TimePeriodCalulated * 0.034) / 2;
		
		for(unsigned int i=0;i<32000;i++);
	}
	
	return (0);
}
/*Note- I used power from uno board 3.3v from own stm32 board power is not healthy*/