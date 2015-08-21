//=============================================================================
// STM32VLDISCOVERY tutorial
// Lesson 4. Blinking and sending data with timer interrupts.
// Copyright : Ajmal M Ali
// e-mail : ajeeshb4u@gmail.com
//=============================================================================
#include "stm32f10x.h"
#include "antilib_gpio.h" 			//For LED setup
#include "Serial.h"							//For serial Port setup

//=============================================================================
// Defines
//=============================================================================
#define LED_BLUE_GPIO	GPIOC
#define LED_BLUE_PIN	13
char g[20]={0x55,0xaa,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,0x10,0x11,0x12};

//=============================================================================
// main function
//=============================================================================
int main(void)
{

SER_Init();
	
RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN;
RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

#if (LED_BLUE_PIN > 7)
  LED_BLUE_GPIO->CRH = (LED_BLUE_GPIO->CRH & CONFMASKH(LED_BLUE_PIN)) | GPIOPINCONFH(LED_BLUE_PIN,     GPIOCONF(GPIO_MODE_OUTPUT2MHz, GPIO_CNF_OUTPUT_PUSHPULL));
#else
  LED_BLUE_GPIO->CRL = (LED_BLUE_GPIO->CRL & CONFMASKL(LED_BLUE_PIN)) | GPIOPINCONFL(LED_BLUE_PIN,     GPIOCONF(GPIO_MODE_OUTPUT2MHz, GPIO_CNF_OUTPUT_PUSHPULL));
#endif

TIM3->PSC = 35000;	        // Set prescaler to 24 000 (PSC + 1)
TIM3->ARR = 2000;	          // Auto reload value 1000
TIM3->DIER = TIM_DIER_UIE; // Enable update interrupt (timer level)
TIM3->CR1 = TIM_CR1_CEN;   // Enable timer

NVIC_EnableIRQ(TIM3_IRQn); // Enable interrupt from TIM3 (NVIC level)

while (1) {}
}

//=============================================================================
// TIM3 Interrupt Handler
//=============================================================================
void TIM3_IRQHandler(void)
{
	char i=0;
  TIM3->SR &= ~TIM_SR_UIF; // clear UIF flag
  LED_BLUE_GPIO->ODR ^= (1 << LED_BLUE_PIN); // toggle LED state
	for(i=0;i<20;i++)
	{
		SER_PutChar (g[i]);
	}

}
//=============================================================================
// End of file
//=============================================================================