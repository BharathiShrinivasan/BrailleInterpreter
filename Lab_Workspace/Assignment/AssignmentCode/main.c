/* include libraries used*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "Board_GLCD.h"
#include "GLCD_Config.h"

/* macros and labels if used */
#define USE_GLCD        1 
uint32_t adcValue = 0; // global ADC read value
uint16_t LED_Toggle = 0; // global toggle variable
extern GLCD_FONT GLCD_Font_6x8;
extern GLCD_FONT GLCD_Font_16x24;

#ifdef RTE_CMSIS_RTOS_RTX
extern   uint32_t os_time;
uint32_t HAL_GetTick (void) {
  return os_time;
}
#endif

/* ===============System clock configuration and delayMs()============*/
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the
     device is clocked below the maximum system frequency (see datasheet). */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE; // High speed external xtal
  RCC_OscInitStruct.HSEState = RCC_HSE_ON; // enable
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON; // PLL ON
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE; // HSE as its source clock
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |  RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

/* millisec delay function */
void Delay_ms(volatile int time_ms)
{
	int j;
	for(j=0;j<time_ms*4000;j++)  /* excute NOP for 1ms */
		{ __ASM volatile ("NOP");} 
}

/*system error handler -> infinite loop()*/
static void Error_Handler (void) {
  while (1);
}


/*============GPIO for reading UserPIN and writing WarningLED===============*/
/* GPIO initialisation() USER PIN -> PG15, LED connected -> PD15*/
void GPIO_Init(void)
{
	  __HAL_RCC_GPIOG_CLK_ENABLE() ;// Clock enable for GPIO port G for UserButton PG15
		__HAL_RCC_GPIOH_CLK_ENABLE() ;// Clock enable for GPIO port H for WarningLED PH02
	
	    GPIO_InitTypeDef GPIO_UserPIN; // USER Pin @PORT-G
	    GPIO_UserPIN.Pin = GPIO_PIN_15; //PIN-15
	    GPIO_UserPIN.Mode = GPIO_MODE_INPUT;
			GPIO_UserPIN.Pull = GPIO_PULLUP; //upon press the value goes GND
	    GPIO_UserPIN.Speed = GPIO_SPEED_FREQ_LOW;
	    HAL_GPIO_Init(GPIOG, &GPIO_UserPIN);
			GPIOG->BSRR=0xFFFF0000; // Reset init

			GPIO_InitTypeDef GPIO_WarningLED;
		  GPIO_WarningLED.Pin = GPIO_PIN_2;// Warning LED pin @PORT-H
	    GPIO_WarningLED.Mode = GPIO_MODE_OUTPUT_PP;  
	    GPIO_WarningLED.Speed = GPIO_SPEED_FREQ_LOW;
	    HAL_GPIO_Init(GPIOH, &GPIO_WarningLED);
			GPIOH->BSRR=0xFFFF0000; // reset LED
}

void GPIO_ReadWrite(void) /* Read Pin and copy to WarningLED */
{
	bool PinStatus;
	PinStatus= HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_15); // USER Pin
	if(~PinStatus) // pressed
	{
		HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2, GPIO_PIN_SET);
	}
	else HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2, GPIO_PIN_RESET);
}

/*===== SOFTWARE CLOCK MIN::SEC ==*/
uint8_t togg=0;
volatile uint16_t sec=0;
volatile uint16_t min=0;

void TIM2_Initialize (void){ // Used for 1 sec interrupt -> to updateClock()
	const uint16_t PSC_val =  8400; //prescalar value 84MHZ/8400
  const uint16_t ARR_val= 10000;//10000;//Auto Reload Register value (10000) every 1 sec the event occurs
	RCC->APB1ENR|=(1UL<<0);//enable timer clock

  	TIM2->PSC=PSC_val-1;//comparison of ARR and Timer cont will take 1 cycle hence count-1 has to be uploaded for both PSC & ARR
		TIM2->ARR=ARR_val-1;//
		TIM2->DIER=(1UL<<0); // enable interrupt for event
		NVIC_EnableIRQ(TIM2_IRQn);
		//TIM2->CR1=(1UL<<0);//set control register >> counter enable (enable/disable via software)
}

void ADCSample();// Declare ADCSample()
void TIM2_IRQHandler(void){ // TIM2 interrupt service
		TIM2->SR &=~(1<<0); // clear interrupt flag
		
		/*==== Update clock ==*/
		togg=~togg; //timer pulse monitoring
	  sec=sec+1; // update sec
		if (sec>59)
		{
			sec=0;
			min=min+1; // update min
			if(min>59) {min=0;}
			/*==== Sampling section ==*/
			int remainderMin=min; // @min= 0,5,10.. initiate an ADC sample()
			for(;remainderMin>=1;)remainderMin=remainderMin-1;
			if (remainderMin==0) 
				{ADCSample();}//Sample }
		}
}

/*======== ADC sampling and reading ===*/
void ADC_Init(void) /*PF7<--> 5th channel of ADC3*/
{
	RCC->APB2ENR|=(1UL<<10); //Enable ADC3 clock     
	RCC->AHB1ENR|=(1UL<<5);//Enable GPIOF Clock     
	GPIOF->MODER|=(3UL<<14);//GPIO mode PF7 in Analog mode,7th pin of PORT-F is in analog mode
	
	ADC3->SQR1=0; // Sequence of channel sampling     
	ADC3->SQR2=0; // Seq of channel sampling
	ADC3->SQR3=(5UL<<0); //SQ1 channel selection 5th channel in seq-0 or first conversion
	ADC3->SMPR1= 0;     // individual channel sampling  
	ADC3->SMPR2= (7UL<<15);//5th channel to be sampling period time=480 cycles; useful only in case multi adc channel sampling     
	ADC3->CR1 |= (2UL<<24); // ADC Resolution as 8bit
	ADC3->CR2 &=~2; //Single conversion mode  or Clear Continous-mode
	//ADC3->CR2|=(1UL<<0);// ADC Enable 
}

uint16_t ADCRead(void) /* Returns DR value initiating conversion and EOC */
{
	volatile uint16_t ADCData=0;
	ADC3->CR2 |= ADC_CR2_SWSTART; // start conversion software initiate
	Delay_ms(100); // wait for EOC
	//while(~((ADC3->SR) & 1UL)); // wait for EOC
	ADCData=(int)ADC3->DR; //Read DR as well it resets EOC flag in SR.
	return ADCData;	
}

uint16_t StorageArray[30]={0}; // to store sampled data in array
int SampleCount=0;
void Display_ADCValue(int ADCValue); // declaration of the function.
void ADCSample(void)
{
	uint16_t ValueRead=ADCRead();
	Display_ADCValue(ValueRead);
	uint16_t level=3;
	if(ValueRead>200) //Level-0
	{
		HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2, GPIO_PIN_SET); // Warning LED lit
		level=0;
	}
	else
	{
		HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2, GPIO_PIN_RESET); // Safe zone LED off
		if(ValueRead>150) // level-1
		{level=1;}
		else
		{
			if(ValueRead>100) // level-2
			{ level=2;}
			else
			{
				level=3;
			}
		}
	}
	StorageArray[SampleCount]=level;
	SampleCount=SampleCount+1;
	if(SampleCount>30) SampleCount=0;
}

/*=================GLCD display==========================*/
void initialize_DisplayLCD()
{
	GLCD_Initialize         ();
  GLCD_SetBackgroundColor (GLCD_COLOR_BLUE);
  GLCD_SetForegroundColor (GLCD_COLOR_WHITE);
  GLCD_ClearScreen        ();
  GLCD_SetFont            (&GLCD_Font_16x24);
  GLCD_DrawString         (0, 0*24, "                   ");
  GLCD_DrawString         (0, 1*24, " GLCD-Initialised");
  GLCD_DrawString         (0, 2*24, "                ");
	GLCD_DrawString         (0, 3*24, "                ");
}

void Display_ADCValue(int ADCValue)
{
		GLCD_DrawString        (0, 3*24, "Value:");
		GLCD_DrawString        (0, 4*24, "                       ");
		char int_string[10];
		int i;
		for(i=0;~(ADCValue==0) && (i<10);i++)
		{
			int_string[9-i]=ADCValue%10 + 48; // 48->'0'
			ADCValue=ADCValue/10;
		}

		GLCD_DrawString        (0, 4*24, int_string);
}

void Display_Level0_Count (void)
{
		GLCD_DrawString        (0, 5*24, "                       ");
		GLCD_DrawString        (0, 6*24, "                       ");
		int Level0Count=0;
		for(int i=0; i<SampleCount;i++)
		{
			if(StorageArray[i]==0) Level0Count++; // if level-0 count
		}
		GLCD_DrawString        (0, 5*24, "Level-0 count:         ");
		//Display_ADCValue(Level0Count);
		
		char int_string[10];
		int i;
		for(i=0;~(Level0Count==0) && (i<10);i++)
		{
			int_string[9-i]=Level0Count%10 + 48; // 48->'0'
			Level0Count=Level0Count/10;
		}

		GLCD_DrawString        (0, 6*24, int_string);
}

/* ======================TESTING PHASE======================================== */
/*
int main (void) {

	HAL_Init();                           // Initialize the HAL Library
  SystemClock_Config();                 // Configure the System Clock
  SystemCoreClockUpdate();              // Update system clock
	GPIO_Init();													// GPIO
	TIM2_Initialize();										// Timer init
	initialize_DisplayLCD();							// LCD init
	ADC_Init();														// init ADC
	ADC3->CR2|=(1UL<<0);// ADC Enable 
	
	int temp=0;
	int tempCount=34;
	int DataRead=0;
	
	while(1)
	{
		ADCSample();
		Display_ADCValue(StorageArray[SampleCount-1]);
		Delay_ms(2000);
		
			//temp=HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_15);
			if (temp==1){ // normal
				HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2, GPIO_PIN_RESET);
				Delay_ms(2000);
			}
			else //pressed
			{
				HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2, GPIO_PIN_SET);
				Delay_ms(2000);
			}
						
	}
		
}
*/
/*=====================Assignment-2 ===========================================*/

int main(void)
{
	HAL_Init();                           // Initialize the HAL Library
  SystemClock_Config();                 // Configure the System Clock
  SystemCoreClockUpdate();              // Update system clock
	GPIO_Init();													// GPIO
	initialize_DisplayLCD();							// Initialise GLCD
	TIM2_Initialize();										// Timer init
	ADC_Init();														// init ADC
	ADC3->CR2|=(1UL<<0);									// ADC Enable	

	while (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_15)==1); // wait till 1st time user button is pressed
	GLCD_DrawString         (0, 1*24, " CLOCK Started:   ");
	TIM2->CR1=(1UL<<0);//Enable timer to start
	Delay_ms(4000);
	
	while(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_15)==1); // wait till 2nd time user button is pressed
	GLCD_DrawString         (0, 1*24, " CLOCK Stoped:    ");
	TIM2->CR1=(0UL<<0);//disable timer
	ADC3->CR2|=(0UL<<0);// ADC disable
	Delay_ms(4000);
	
	while(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_15)==1); // wait till 3rd time user button is pressed
	GLCD_DrawString         (0, 1*24, "                   ");
	Display_Level0_Count(); //Display LCD
	Delay_ms(4000);
	
	while(1); // let system reset to restart the program
	
	return(0);
}
