#include "stm32f4xx.h"

/*==== 16MHz sys clock and peripheral ===*/


/*=====USART 9600, 1 start bit, 1 stop bit, 8 data bit, 0 parity bit ====*/
/* UART4 is connected in PC10,11 */
void init_UART(void){
	
	RCC->APB1ENR |= 1<<19; //UART4 EN clock
	
	//Multiplex pin of PC10,11 to be mapped to UART4
	RCC->AHB1ENR |= 1<<2; //PortC clock enaabled
	GPIOC->MODER &= ~(0xFUL << 20);// set as alternate pin function 
	GPIOC->MODER |= (0xAUL << 20);// set as alternate pin function 
	GPIOC->AFR[1]=0x88UL<<8;//AFRH : AF8 is USART4
	
	// Enable UART ; set M-Bit 0 -> 1start bit, 8bit data, 1stop bit(default)
	UART4->CR1 |= 1<<13; // UE 1
	//set baud rate to 9600. over8=0
	UART4->BRR=0x0683; // DIV=104.1875 or integer part[104] fraction part[3]  (3=0.1875*16)
	
	//start Tx enable. (it will also send 1st frame idle)
	UART4->CR1 |= 1<<3; // TE 1
	
}

void sendChar(int8_t data){
	while (!(UART4->SR&(1<<7))); // wait if TXE is not 0
	UART4->DR = data & 0x000000FF; // make sure only first 8 bits are copied.
}

int main(void)
{
	int dummy=0;
	init_UART();
	int8_t data[5]={0x41, 0x42, 0x43, 0x44, 0x45};
	while(1){
		
		for(int i=0;i<5;i++)
		{
			sendChar(data[i]);
		}
		dummy++;
		for(unsigned int i=0;i<1600000;i++);
	}
	
	return (0);
}