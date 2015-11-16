/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f1xx_hal.h"

/** @defgroup General Led Constants
  * @{
  */
#define GENLEDn                             1

#define GENLED_PIN                         GPIO_PIN_13
#define GENLED_GPIO_PORT                   GPIOC
#define GENLED_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOC_CLK_ENABLE()  
#define GENLED_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOC_CLK_DISABLE()  

#define GENLEDx_GPIO_CLK_ENABLE(__INDEX__)  do {GENLED_GPIO_CLK_ENABLE(); } while(0)

#define GENLEDx_GPIO_CLK_DISABLE(__INDEX__) GENLED_GPIO_CLK_DISABLE())

/**
  * @}
  */ 

/** @defgroup General Led Constants
  * @{
  */
#define Buttonn                             1

#define Button_PIN                         GPIO_PIN_0
#define Button_GPIO_PORT                   GPIOA
#define Button_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()  
#define Button_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOA_CLK_DISABLE()  

#define Buttonx_GPIO_CLK_ENABLE(__INDEX__)  do {Button_GPIO_CLK_ENABLE(); } while(0)

#define Buttonx_GPIO_CLK_DISABLE(__INDEX__) Button_GPIO_CLK_DISABLE())

/**
  * @}
  */ 


#endif
