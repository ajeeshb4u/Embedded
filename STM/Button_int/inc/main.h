/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f1xx_hal.h"

//void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim);
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

/* Definition for TIMx clock resources */
#define TIMx                           TIM3
#define TIMx_CLK_ENABLE()              __HAL_RCC_TIM3_CLK_ENABLE()


/* Definition for TIMx's NVIC */
#define TIMx_IRQn                      TIM3_IRQn
#define TIMx_IRQHandler                TIM3_IRQHandler


/* Definition for USARTx clock resources */
#define USARTx                           USART1
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART1_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __HAL_RCC_USART1_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART1_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_9
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_RX_PIN                    GPIO_PIN_10
#define USARTx_RX_GPIO_PORT              GPIOA

/* Definition for USARTx's NVIC */
#define USARTx_IRQn                      USART1_IRQn
#define USARTx_IRQHandler                USART1_IRQHandler

/* Size of Trasmission buffer */
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
/* Size of Reception buffer */
#define RXBUFFERSIZE                      TXBUFFERSIZE
  
/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Exported functions ------------------------------------------------------- */




#endif
