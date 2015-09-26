/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H



uint32_t HAL_SYSTICK_Config(uint32_t TicksNumb);
uint32_t reg_value;
volatile uint32_t msTicks;
volatile uint32_t curTicks;

void SysTick_Handler(void);
void Delay (unsigned int dlyTicks);


#endif
