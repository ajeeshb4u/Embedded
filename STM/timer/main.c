//=============================================================================
// Lesson 3. Blinking with timer.
// Copyright : Radoslaw Kwiecien
// http://en.radzio.dxp.pl
// e-mail : radek(at)dxp.pl
//=============================================================================
#include "stm32f10x.h"
#include "antilib_gpio.h"
//=============================================================================
// Defines
//=============================================================================
#define LED_BLUE_GPIO	GPIOC
#define LED_BLUE_PIN	 13
//=============================================================================
// main function
//=============================================================================
int main(void)
{
RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN;
RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

#if (LED_BLUE_PIN > 7)
  LED_BLUE_GPIO->CRH = (LED_BLUE_GPIO->CRH & CONFMASKH(LED_BLUE_PIN)) | GPIOPINCONFH(LED_BLUE_PIN,     GPIOCONF(GPIO_MODE_OUTPUT2MHz, GPIO_CNF_OUTPUT_PUSHPULL));
#else
  LED_BLUE_GPIO->CRL = (LED_BLUE_GPIO->CRL & CONFMASKL(LED_BLUE_PIN)) | GPIOPINCONFL(LED_BLUE_PIN,     GPIOCONF(GPIO_MODE_OUTPUT2MHz, GPIO_CNF_OUTPUT_PUSHPULL));
#endif

TIM3->PSC = 38421;	     // Set prescaler to 24 000 (PSC + 1)
TIM3->ARR = 2000;	      // Auto reload value 1000
TIM3->CR1 = TIM_CR1_CEN;// Enable timer

while (1) {
if(TIM3->SR & TIM_SR_UIF) // if UIF flag is set
  {
  TIM3->SR &= ~TIM_SR_UIF; // clear UIF flag
  LED_BLUE_GPIO->ODR ^= (1 << LED_BLUE_PIN); // toggle LED state
  }
}
}
//=============================================================================
// End of file
//=============================================================================