#include <stm32f10x.h>
#include "includes\main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
enum {
	TRANSFER_WAIT,
	TRANSFER_COMPLETE,
	TRANSFER_ERROR
};


int main(void)
{
/*************************Clock Configuration*****************************
	Microcontroller clock o/p = No Clock
	PLL Multiplier						= PLL input clock *9
	HSE Divider for PLL entry	= HSE not Divided
	PLL Clock Source					=	HSE Oscillator Clock
	APB high speed Prescalar	=	HCLK not divided
	APB low speed Prescalar		=	HCLK /2
	AHB Prescalar							=	Sysclk not divided
	System clock status				= PLL is the system clock
	System clock switch				=	PLL Selected
**************************************************************************/
/*****Configure the SysTick to have interrupt in 1ms time basis*****/
SysTick_Config(SystemCoreClock / 1000);


while(1)
	Delay(1000);
}


void SysTick_Handler(void)  {
	msTicks++;
}


/**
  * @brief  Initializes the System Timer and its interrupt, and starts the System Tick Timer.
  *         Counter is in free running mode to generate periodic interrupts.
  * @param  TicksNumb: Specifies the ticks Number of ticks between two interrupts.
  * @retval status:  - 0  Function succeeded.
  *                  - 1  Function failed.
  */

uint32_t HAL_SYSTICK_Config(uint32_t TicksNumb)
{
   return SysTick_Config(TicksNumb);
}

/**********************Delay (in dlyTicks ms)*********************/
void Delay (unsigned int dlyTicks) 
{                                              
  
  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks);
}
