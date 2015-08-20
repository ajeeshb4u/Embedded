//=============================================================================
// STM32VLDISCOVERY tutorial
// Lesson 2. Reading the button.
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
#define LED_BLUE_PIN	14

#define SW_USER_GPIO	GPIOA
#define SW_USER_PIN	0 
//=============================================================================
// main function
//=============================================================================
int main(void)
{
RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN;

#if (LED_BLUE_PIN > 7)
  LED_BLUE_GPIO->CRH = (LED_BLUE_GPIO->CRH & CONFMASKH(LED_BLUE_PIN)) | GPIOPINCONFH(LED_BLUE_PIN,     GPIOCONF(GPIO_MODE_OUTPUT2MHz, GPIO_CNF_OUTPUT_PUSHPULL));
#else
  LED_BLUE_GPIO->CRL = (LED_BLUE_GPIO->CRL & CONFMASKL(LED_BLUE_PIN)) | GPIOPINCONFL(LED_BLUE_PIN,     GPIOCONF(GPIO_MODE_OUTPUT2MHz, GPIO_CNF_OUTPUT_PUSHPULL));
#endif

#if (SW_USER_PIN > 7)
  SW_USER_GPIO->CRH	=	(SW_USER_GPIO->CRH & CONFMASKH(SW_USER_PIN)) | GPIOPINCONFH(SW_USER_PIN,     GPIOCONF(GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOATING));
#else
  SW_USER_GPIO->CRL	=	(SW_USER_GPIO->CRH & CONFMASKL(SW_USER_PIN)) | GPIOPINCONFL(SW_USER_PIN,     GPIOCONF(GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOATING));
#endif

while (1) {
if(SW_USER_GPIO->IDR & (1 << SW_USER_PIN))
  LED_BLUE_GPIO->BSRR = (1 << LED_BLUE_PIN); 
else
  LED_BLUE_GPIO->BRR = (1 << LED_BLUE_PIN);
}
}
//=============================================================================
// End of file
//=============================================================================