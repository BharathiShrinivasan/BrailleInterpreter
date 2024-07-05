#include"stm32f4xx.h"

/* sys clock and ABP running in 16MHz */

/*Initialise timer TIMER3 CH1 at PA06*/
void init_TIMER(void){
	//Config alternate function @PA06
	RCC->AHB1ENR |= 1<<0; //Enable clock in GPIO A
	GPIOA->MODER |= 2<<12; //alternate fn chose
	GPIOA->AFR[0] |= 2<<24; // AF2 is timer function
	
	RCC->APB1ENR |= 1<<1; // TIM3 clock supply enabled
	TIM3->PSC = 5-1; // 16Mhz/5 -> 3.2Mhz clock source
	TIM3->ARR = 0xFFFF-1; // 65536 ;=> 48.82Hz ~ 20.48mS
	
	TIM3->CCMR1 &= ~(7<<4); TIM3->CCMR1 |= (6<<4); 	//SET PWM mode-1
	TIM3->CCMR1 |= 1<<3; //OC1PE access enable to pre-load register upon read/write
	TIM3->CR1 |= 1<<7; // ARPE use a buffer to enable; preload registers are transferred to the shadow registers only when an update event occurs
	TIM3->CCR1=0x00FF; // CCR value as 25% 
	TIM3->EGR |= 1<<0; //Update generate to first init registers and counters before starting
	TIM3->CCER |= 1<<0; // Enable output in channel1
	
	TIM3->DIER=(3UL<<0); // enable interrupt for OC event @CH1 and update event
	NVIC_EnableIRQ(TIM3_IRQn); // Interrupt enable
	
	TIM3->CR1 |= 1<<0; //Counter enable
}

uint8_t togg=0x0F;

void TIM3_IRQHandler(void){ // TIM3 interrupt service
		TIM3->SR &=~(3<<0); // clear interrupt flag
		togg=~togg;
}

volatile uint32_t AutoRR=0x00, Count=0x00;
volatile uint32_t CCRValue=0x00;

uint32_t mills=0x00; uint32_t dummy=0x00FF;

int main(void){
	init_TIMER();
	
	while(1){	
					TIM3->CCR1=dummy;
					for(unsigned int i=0;i<32000;i++);
	}
	
	return (0);
}

/* note there's only 1 global interrupt for timer3 which is evoked under (update event i.e overflow, new register data; and/or CC event compare event).
And i saw trace going Overflow Data Overflow - because of some repeated capturing event-- make sure some no interrupt is repeadly called. like-> once you go inside isr reset Status reg*/
