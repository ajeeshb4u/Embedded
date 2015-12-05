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



/* Definition for SPIx clock resources */
#define SPIx                             SPI2
#define SPIx_CLK_ENABLE()                __HAL_RCC_SPI2_CLK_ENABLE()
#define SPIx_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPIx_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPIx_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()

/* Definition for SPIx Pins */
#define SPIx_SCK_PIN                     GPIO_PIN_13
#define SPIx_SCK_GPIO_PORT               GPIOB
#define SPIx_MISO_PIN                    GPIO_PIN_14
#define SPIx_MISO_GPIO_PORT              GPIOB
#define SPIx_MOSI_PIN                    GPIO_PIN_15
#define SPIx_MOSI_GPIO_PORT              GPIOB

/* Definition for SPIx's NVIC */
#define SPIx_IRQn                        SPI2_IRQn
#define SPIx_IRQHandler                  SPI2_IRQHandler


/* Size of Trasmission buffer */
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
/* Size of Reception buffer */
#define RXBUFFERSIZE                      TXBUFFERSIZE
  
/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Exported functions ------------------------------------------------------- */


/** @defgroup DRDY Constants
  * @{
  */
#define DRDYn                             1

#define DRDY_PIN                         GPIO_PIN_0
#define DRDY_GPIO_PORT                   GPIOA
#define DRDY_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()  
#define DRDY_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOA_CLK_DISABLE()  

#define DRDYx_GPIO_CLK_ENABLE(__INDEX__)  do {DRDY_GPIO_CLK_ENABLE(); } while(0)

#define DRDYx_GPIO_CLK_DISABLE(__INDEX__) DRDY_GPIO_CLK_DISABLE())

/**
  * @}
  */ 

/** @defgroup CS Constants
  * @{
  */
#define CSn                             1

#define CS_PIN                         GPIO_PIN_3
#define CS_GPIO_PORT                   GPIOA
#define CS_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()  
#define CS_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOA_CLK_DISABLE()  

#define CSx_GPIO_CLK_ENABLE(__INDEX__)  do {CS_GPIO_CLK_ENABLE(); } while(0)

#define CSx_GPIO_CLK_DISABLE(__INDEX__) CS_GPIO_CLK_DISABLE())

/**
  * @}
  */ 


/** @defgroup START Constants
  * @{
  */
#define STARTn                             1

#define START_PIN                         GPIO_PIN_4
#define START_GPIO_PORT                   GPIOA
#define START_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()  
#define START_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOA_CLK_DISABLE()  

#define STARTx_GPIO_CLK_ENABLE(__INDEX__)  do {START_GPIO_CLK_ENABLE(); } while(0)

#define STARTx_GPIO_CLK_DISABLE(__INDEX__) START_GPIO_CLK_DISABLE())

/**
  * @}
  */ 

/** @defgroup RESET Constants
  * @{
  */
#define RESETn                             1

#define RESET_PIN                         GPIO_PIN_5
#define RESET_GPIO_PORT                   GPIOA
#define RESET_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()  
#define RESET_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOA_CLK_DISABLE()  

#define RESETx_GPIO_CLK_ENABLE(__INDEX__)  do {RESET_GPIO_CLK_ENABLE(); } while(0)

#define RESETx_GPIO_CLK_DISABLE(__INDEX__) RESET_GPIO_CLK_DISABLE())

/**
  * @}
  */ 

/** @defgroup CLKSEL Constants
  * @{
  */
#define CLKSELn                             1

#define CLKSEL_PIN                         GPIO_PIN_6
#define CLKSEL_GPIO_PORT                   GPIOA
#define CLKSEL_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()  
#define CLKSEL_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOA_CLK_DISABLE()  

#define CLKSELx_GPIO_CLK_ENABLE(__INDEX__)  do {CLKSEL_GPIO_CLK_ENABLE(); } while(0)

#define CLKSELx_GPIO_CLK_DISABLE(__INDEX__) CLKSEL_GPIO_CLK_DISABLE())

/**
  * @}
  */ 

#endif
