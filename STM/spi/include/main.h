/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/***********	Includes	***************/
#include "macros.h"


#define SPIx_SCK_GPIO_PORT              GPIOB
#define SPIx_SCK_PIN                    GPIO_PIN_13
#define SPIx_MISO_GPIO_PORT             GPIOB
#define SPIx_MISO_PIN                   GPIO_PIN_14
#define SPIx_MOSI_GPIO_PORT             GPIOB
#define SPIx_MOSI_PIN                   GPIO_PIN_15

/* Definition for SPIx clock resources */
#define SPIx                             SPI2
#define SPIx_CLK_ENABLE()                __HAL_RCC_SPI2_CLK_ENABLE() 	/*enable SPIx Clock */
#define SPIx_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPIx_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPIx_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()

/* Size of buffer */
#define BUFFERSIZE                       (COUNTOF(aTxBuffer) - 1)

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))


#endif
