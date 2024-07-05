#include"stm32f4xx.h"

/* System clock is 16Mhz, APB1 also 16Mhz */

/*Initialise timer TIMER3 CH1 -> PA06*/
void init_TIMER(void){
	//Config alternate function @PA06
	RCC->AHB1ENR |= 1<<0; //Enable clock in GPIO A
	GPIOA->MODER |= 2<<12; //alternate fn chose
	GPIOA->AFR[0] |= 2<<24; // AF2 is timer function
	
	RCC->APB1ENR |= 1<<1; // TIM3 clock supply enabled
	TIM3->PSC = 16000-1; // 16Mhz/16000 -> 1Khz clock source
	
	TIM3->CCMR1 |= 1<<0; // channel 1 as IC1 mapped on TI1
	TIM3->CCER |= 1<<0 | 1<<1 | 1<<3; //enable capture, 11-> both edge
	TIM3->CR1 |= 1<<0; //Counter enable
}
unsigned int PreviousCapturedValue=0x00, CurrentCapturedValue=0x00;
unsigned int TimePeriodCalulated=0x00;

int main(void){
	init_TIMER();
	
	while(1){
		while(!(TIM3->SR & 1<<1)){} // wait for the capture to occur
		CurrentCapturedValue = TIM3->CCR1;
		TimePeriodCalulated=(CurrentCapturedValue-PreviousCapturedValue);
		PreviousCapturedValue=CurrentCapturedValue;
	}
	
	return (0);
}
