/** @addtogroup Exported_macros
  * @{
  */
	
#ifndef __MACROS_H
#define __MACROS_H
	
#include "spi_def.h"	
#include "nvic_def.h"
#include "stm32f1xx_hal_rcc.c"


__weak void HAL_MspInit(void);
/*
#ifndef SPIx
#define SPIx                             SPI2
#endif
*/	


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

#define __HAL_SPI_DISABLE(__HANDLE__) CLEAR_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_SPE)

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))


#define __HAL_RCC_SPI2_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI2EN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI2EN);\
                                      } while(0)

/*#define __HAL_RCC_GPIOB_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPBEN);\
 */                                       /* Delay after an RCC peripheral clock enabling */ \
/*                                        tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPBEN);\
                                      } while(0)
*/

/** 
  * @brief  SPI handle Structure definition
  */
#ifndef SPI_HandleTypeDef
typedef struct __SPI_HandleTypeDef
{
  SPI_TypeDef                *Instance;    /*!< SPI registers base address */

  SPI_InitTypeDef            Init;         /*!< SPI communication parameters */

  uint8_t                    *pTxBuffPtr;  /*!< Pointer to SPI Tx transfer Buffer */

  uint16_t                   TxXferSize;   /*!< SPI Tx transfer size */
  
  uint16_t                   TxXferCount;  /*!< SPI Tx Transfer Counter */

  uint8_t                    *pRxBuffPtr;  /*!< Pointer to SPI Rx transfer Buffer */

  uint16_t                   RxXferSize;   /*!< SPI Rx transfer size */

  uint16_t                   RxXferCount;  /*!< SPI Rx Transfer Counter */

  DMA_HandleTypeDef          *hdmatx;      /*!< SPI Tx DMA handle parameters */

  DMA_HandleTypeDef          *hdmarx;      /*!< SPI Rx DMA handle parameters */

  void                       (*RxISR)(struct __SPI_HandleTypeDef * hspi); /*!< function pointer on Rx ISR */

  void                       (*TxISR)(struct __SPI_HandleTypeDef * hspi); /*!< function pointer on Tx ISR */

  HAL_LockTypeDef            Lock;         /*!< SPI locking object */

  __IO HAL_SPI_StateTypeDef  State;        /*!< SPI communication state */

  __IO uint32_t  ErrorCode;    /*!< SPI Error code */

}SPI_HandleTypeDef;
#endif

/**
  * @brief This function configures the source of the time base. 
  *        The time source is configured  to have 1ms time base with a dedicated 
  *        Tick interrupt priority.
  * Note: This function is called  automatically at the beginning of program after
  *       reset by HAL_Init() or at any time when clock is reconfigured  by HAL_RCC_ClockConfig(). 
  * Note: In the default implementation, SysTick timer is the source of time base. 
  *       It is used to generate interrupts at regular time intervals. 
  *       Care must be taken if HAL_Delay() is called from a peripheral ISR process, 
  *       The the SysTick interrupt must have higher priority (numerically lower) 
  *       than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
  *       The function is declared as __Weak  to be overwritten  in case of other
  *       implementation  in user file.
  * @param TickPriority: Tick interrupt priority.
  * @retval HAL status
  */
__weak HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  /*Configure the SysTick to have interrupt in 1ms time basis*/
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  /*Configure the SysTick IRQ priority */
  HAL_NVIC_SetPriority(SysTick_IRQn, TickPriority ,0);

   /* Return function status */
  return HAL_OK;
}



/**
  * @brief This function configures the Flash prefetch, 
  *        Configures time base source, NVIC and Low level hardware
  * Note: This function is called at the beginning of program after reset and before 
  *       the clock configuration
  * Note: The time base configuration is based on MSI clock when exiting from Reset.
  *       Once done, time base tick start incrementing.
  *        In the default implementation,Systick is used as source of time base.
  *        the tick variable is incremented each 1ms in its ISR.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_Init(void)
{
  /* Configure Flash prefetch */
#if (PREFETCH_ENABLE != 0)
#if defined(STM32F101x6) || defined(STM32F101xB) || defined(STM32F101xE) || defined(STM32F101xG) || \
    defined(STM32F102x6) || defined(STM32F102xB) || \
    defined(STM32F103x6) || defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
    defined(STM32F105xC) || defined(STM32F107xC)

  /* Prefetch buffer is not available on value line devices */
  __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
#endif
#endif /* PREFETCH_ENABLE */

  /* Set Interrupt Group Priority */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* Use systick as time base source and configure 1ms tick (default clock after Reset is MSI) */
  HAL_InitTick(TICK_INT_PRIORITY);

  /* Init the low level hardware */
  HAL_MspInit();

  /* Return function status */
  return HAL_OK;
}


/**
  * @brief  Initializes the MSP.
  * @retval None
  */
__weak void HAL_MspInit(void)
{
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_MspInit could be implemented in the user file
   */
}


/**
  * @brief SPI MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  * @param hspi: SPI handle pointer
  * @retval None
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
GPIO_InitTypeDef  GPIO_InitStruct;

  if(hspi->Instance == SPIx)
  {     
    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO TX/RX clock */
    SPIx_SCK_GPIO_CLK_ENABLE();
    SPIx_MISO_GPIO_CLK_ENABLE();
    SPIx_MOSI_GPIO_CLK_ENABLE();
    /* Enable SPI clock */
    SPIx_CLK_ENABLE(); 
    
    /*##-2- Configure peripheral GPIO ##########################################*/  
    /* SPI SCK GPIO pin configuration  */
    GPIO_InitStruct.Pin       = SPIx_SCK_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
    HAL_GPIO_Init(SPIx_SCK_GPIO_PORT, &GPIO_InitStruct);

    /* SPI MISO GPIO pin configuration  */
    GPIO_InitStruct.Pin = SPIx_MISO_PIN;
    HAL_GPIO_Init(SPIx_MISO_GPIO_PORT, &GPIO_InitStruct);

    /* SPI MOSI GPIO pin configuration  */
    GPIO_InitStruct.Pin = SPIx_MOSI_PIN;
    HAL_GPIO_Init(SPIx_MOSI_GPIO_PORT, &GPIO_InitStruct);
  }
}


HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi)
{
  /* Check the SPI handle allocation */
  if(hspi == NULL)
  {
    return HAL_ERROR;
  }

  if(hspi->State == HAL_SPI_STATE_RESET)
  {
    /* Init the low level hardware : GPIO, CLOCK, NVIC... */
    HAL_SPI_MspInit(hspi);
  }
  
  hspi->State = HAL_SPI_STATE_BUSY;

  /* Disble the selected SPI peripheral */
  __HAL_SPI_DISABLE(hspi);

  /*----------------------- SPIx CR1 & CR2 Configuration ---------------------*/
  /* Configure : SPI Mode, Communication Mode, Data size, Clock polarity and phase, NSS management,
  Communication speed, First bit and CRC calculation state */
  WRITE_REG(hspi->Instance->CR1, (hspi->Init.Mode | hspi->Init.Direction | hspi->Init.DataSize |
                                  hspi->Init.CLKPolarity | hspi->Init.CLKPhase | (hspi->Init.NSS & SPI_CR1_SSM) |
                                  hspi->Init.BaudRatePrescaler | hspi->Init.FirstBit  | hspi->Init.CRCCalculation) );

  /* Configure : NSS management */
  WRITE_REG(hspi->Instance->CR2, (((hspi->Init.NSS >> 16) & SPI_CR2_SSOE) | hspi->Init.TIMode));

  /*---------------------------- SPIx CRCPOLY Configuration ------------------*/
  /* Configure : CRC Polynomial */
  WRITE_REG(hspi->Instance->CRCPR, hspi->Init.CRCPolynomial);

#if defined (STM32F101x6) || defined (STM32F101xB) || defined (STM32F101xE) || defined (STM32F101xG) || defined (STM32F102x6) || defined (STM32F102xB) || defined (STM32F103x6) || defined (STM32F103xB) || defined (STM32F103xE) || defined (STM32F103xG) || defined (STM32F105xC) || defined (STM32F107xC)
  /* Activate the SPI mode (Make sure that I2SMOD bit in I2SCFGR register is reset) */
  CLEAR_BIT(hspi->Instance->I2SCFGR, SPI_I2SCFGR_I2SMOD);
#endif

  hspi->ErrorCode = HAL_SPI_ERROR_NONE;
  hspi->State = HAL_SPI_STATE_READY;
  
  return HAL_OK;
}



<<<<<<< HEAD
typedef enum 
{  
  BUTTON_USER = 0,
} Button_TypeDef;

typedef enum 
{  
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef; 


/**
  * @brief  Configures Button GPIO and EXTI Line.
  * @param  Button: Specifies the Button to be configured.
  *   This parameter should be: BUTTON_USER
  * @param  ButtonMode: Specifies Button mode.
  *   This parameter can be one of following parameters:   
  *     @arg BUTTON_MODE_GPIO: Button will be used as simple IO 
  *     @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line with interrupt
  *                     generation capability  
  * @retval None
  */
void BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode)
{
  GPIO_InitTypeDef gpioinitstruct = {0};

  /* Enable the BUTTON Clock */
  BUTTONx_GPIO_CLK_ENABLE(Button);

  if (ButtonMode == BUTTON_MODE_GPIO)
  {
    /* Configure Button pin as input */
    gpioinitstruct.Pin    = BUTTON_PIN[Button];
    gpioinitstruct.Mode   = GPIO_MODE_INPUT;
    gpioinitstruct.Pull   = GPIO_NOPULL;
    gpioinitstruct.Speed  = GPIO_SPEED_MEDIUM;
  
    HAL_GPIO_Init(BUTTON_PORT[Button], &gpioinitstruct);
  }
 
//  if (ButtonMode == BUTTON_MODE_EXTI)
//  {
//    /* Configure Button pin as input with External interrupt */
/*    gpioinitstruct.Pin    = BUTTON_PIN[Button];
    gpioinitstruct.Pull   = GPIO_NOPULL;
    gpioinitstruct.Speed  = GPIO_SPEED_MEDIUM;
    gpioinitstruct.Mode   = GPIO_MODE_IT_FALLING; 
    HAL_GPIO_Init(BUTTON_PORT[Button], &gpioinitstruct);
*/
//    /* Enable and set Button EXTI Interrupt to the lowest priority */
/*    HAL_NVIC_SetPriority((IRQn_Type)(BUTTON_IRQn[Button]), 0x0F, 0);
    HAL_NVIC_EnableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
  }*/
}


/**
  * @brief  Reads the specified input port pin.
  * @param  GPIOx: where x can be (A..G depending on device used) to select the GPIO peripheral
  * @param  GPIO_Pin: specifies the port bit to read.
  *         This parameter can be GPIO_PIN_x where x can be (0..15).
  * @retval The input port pin value.
  */
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  GPIO_PinState bitstatus;

  /* Check the parameters */
  //assert_param(IS_GPIO_PIN(GPIO_Pin));

  if ((GPIOx->IDR & GPIO_Pin) != (uint32_t)GPIO_PIN_RESET)
  {
    bitstatus = GPIO_PIN_SET;
  }
  else
  {
    bitstatus = GPIO_PIN_RESET;
  }
  return bitstatus;
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on. 
  *   This parameter can be one of following parameters:
  *     @arg LED2
  * @retval None
  */
void BSP_LED_On(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET); 
}


/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off. 
  *   This parameter can be one of following parameters:
  *     @arg LED2
  * @retval None
  */
void BSP_LED_Off(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET); 
}


/**
  * @brief  Returns the selected Button state.
  * @param  Button: Specifies the Button to be checked.
  *   This parameter should be: BUTTON_USER  
  * @retval Button state.
  */
uint32_t BSP_PB_GetState(Button_TypeDef Button)
{
  return HAL_GPIO_ReadPin(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}


/**
  * @brief  Transmit and Receive an amount of data in blocking mode 
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *                the configuration information for SPI module.
  * @param  pTxData: pointer to transmission data buffer
  * @param  pRxData: pointer to reception data buffer to be
  * @param  Size: amount of data to be sent
  * @param  Timeout: Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout)
{
  __IO uint16_t tmpreg = 0;

  if((hspi->State == HAL_SPI_STATE_READY) || (hspi->State == HAL_SPI_STATE_BUSY_RX))
  {
    if((pTxData == NULL ) || (pRxData == NULL ) || (Size == 0))
    {
      return  HAL_ERROR;
    }

    /* Check the parameters */
//    assert_param(IS_SPI_DIRECTION_2LINES(hspi->Init.Direction));

    /* Process Locked */
    __HAL_LOCK(hspi);
 
    /* Don't overwrite in case of HAL_SPI_STATE_BUSY_RX */
    if(hspi->State == HAL_SPI_STATE_READY)
    {
      hspi->State = HAL_SPI_STATE_BUSY_TX_RX;
    }

     /* Configure communication */   
    hspi->ErrorCode   = HAL_SPI_ERROR_NONE;

    hspi->pRxBuffPtr  = pRxData;
    hspi->RxXferSize  = Size;
    hspi->RxXferCount = Size;  
    
    hspi->pTxBuffPtr  = pTxData;
    hspi->TxXferSize  = Size; 
    hspi->TxXferCount = Size;

    /*Init field not used in handle to zero */
    hspi->RxISR = 0;
    hspi->TxISR = 0;

    /* Reset CRC Calculation */
    if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
      SPI_RESET_CRC(hspi);
    }

    /* Check if the SPI is already enabled */ 
    if((hspi->Instance->CR1 &SPI_CR1_SPE) != SPI_CR1_SPE)
    {
      /* Enable SPI peripheral */
      __HAL_SPI_ENABLE(hspi);
    }

    /* Transmit and Receive data in 16 Bit mode */
    if(hspi->Init.DataSize == SPI_DATASIZE_16BIT)
    {
      if((hspi->Init.Mode == SPI_MODE_SLAVE) || ((hspi->Init.Mode == SPI_MODE_MASTER) && (hspi->TxXferCount == 0x01)))
      {
        hspi->Instance->DR = *((uint16_t*)hspi->pTxBuffPtr);
        hspi->pTxBuffPtr+=2;
        hspi->TxXferCount--;
      }
      if(hspi->TxXferCount == 0)
      {
        /* Enable CRC Transmission */
        if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
        {
          SET_BIT(hspi->Instance->CR1, SPI_CR1_CRCNEXT);
        }

        /* Wait until RXNE flag is set */
        if(SPI_WaitOnFlagUntilTimeout(hspi, SPI_FLAG_RXNE, RESET, Timeout) != HAL_OK)
        { 
          return HAL_TIMEOUT;
        }

        *((uint16_t*)hspi->pRxBuffPtr) = hspi->Instance->DR;
        hspi->pRxBuffPtr+=2;
        hspi->RxXferCount--;
      }
      else
      {
        while(hspi->TxXferCount > 0)
        {
          /* Wait until TXE flag is set to send data */
          if(SPI_WaitOnFlagUntilTimeout(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK)
          { 
            return HAL_TIMEOUT;
          }

          hspi->Instance->DR = *((uint16_t*)hspi->pTxBuffPtr);
          hspi->pTxBuffPtr+=2;
          hspi->TxXferCount--;

          /* Enable CRC Transmission */
          if((hspi->TxXferCount == 0) && (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE))
          {
            SET_BIT(hspi->Instance->CR1, SPI_CR1_CRCNEXT);
          }

          /* Wait until RXNE flag is set */
          if(SPI_WaitOnFlagUntilTimeout(hspi, SPI_FLAG_RXNE, RESET, Timeout) != HAL_OK)
          { 
            return HAL_TIMEOUT;
          }

          *((uint16_t*)hspi->pRxBuffPtr) = hspi->Instance->DR;
          hspi->pRxBuffPtr+=2;
          hspi->RxXferCount--;
        }
        /* Receive the last byte */
        if(hspi->Init.Mode == SPI_MODE_SLAVE)
        {
          /* Wait until RXNE flag is set */
          if(SPI_WaitOnFlagUntilTimeout(hspi, SPI_FLAG_RXNE, RESET, Timeout) != HAL_OK)
          {
            return HAL_TIMEOUT;
          }
          
          *((uint16_t*)hspi->pRxBuffPtr) = hspi->Instance->DR;
          hspi->pRxBuffPtr+=2;
          hspi->RxXferCount--;
        }
      }
    }
    /* Transmit and Receive data in 8 Bit mode */
    else
    {
      if((hspi->Init.Mode == SPI_MODE_SLAVE) || ((hspi->Init.Mode == SPI_MODE_MASTER) && (hspi->TxXferCount == 0x01)))
      {
        hspi->Instance->DR = (*hspi->pTxBuffPtr++);
        hspi->TxXferCount--;
      }
      if(hspi->TxXferCount == 0)
      {
        /* Enable CRC Transmission */
        if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
        {
          SET_BIT(hspi->Instance->CR1, SPI_CR1_CRCNEXT);
        }

        /* Wait until RXNE flag is set */
        if(SPI_WaitOnFlagUntilTimeout(hspi, SPI_FLAG_RXNE, RESET, Timeout) != HAL_OK)
        {
          return HAL_TIMEOUT;
        }

        (*hspi->pRxBuffPtr) = hspi->Instance->DR;
        hspi->RxXferCount--;
      }
      else
      {
        while(hspi->TxXferCount > 0)
        {
          /* Wait until TXE flag is set to send data */
          if(SPI_WaitOnFlagUntilTimeout(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK)
          {
            return HAL_TIMEOUT;
          }

          hspi->Instance->DR = (*hspi->pTxBuffPtr++);
          hspi->TxXferCount--;

          /* Enable CRC Transmission */
          if((hspi->TxXferCount == 0) && (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE))
          {
            SET_BIT(hspi->Instance->CR1, SPI_CR1_CRCNEXT);
          }

          /* Wait until RXNE flag is set */
          if(SPI_WaitOnFlagUntilTimeout(hspi, SPI_FLAG_RXNE, RESET, Timeout) != HAL_OK)
          {
            return HAL_TIMEOUT;
          }

          (*hspi->pRxBuffPtr++) = hspi->Instance->DR;
          hspi->RxXferCount--;
        }
        if(hspi->Init.Mode == SPI_MODE_SLAVE)
        {
          /* Wait until RXNE flag is set */
          if(SPI_WaitOnFlagUntilTimeout(hspi, SPI_FLAG_RXNE, RESET, Timeout) != HAL_OK)
          {
            return HAL_TIMEOUT;
          }
          
          (*hspi->pRxBuffPtr++) = hspi->Instance->DR;
          hspi->RxXferCount--;
        }
      }
    }

    /* Read CRC from DR to close CRC calculation process */
    if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
      /* Wait until RXNE flag is set */
      if(SPI_WaitOnFlagUntilTimeout(hspi, SPI_FLAG_RXNE, RESET, Timeout) != HAL_OK)
      {
        SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_CRC);
        return HAL_TIMEOUT;
      }
      /* Read CRC */
      tmpreg = hspi->Instance->DR;
      UNUSED(tmpreg);
    }

    /* Wait until Busy flag is reset before disabling SPI */
    if(SPI_WaitOnFlagUntilTimeout(hspi, SPI_FLAG_BSY, SET, Timeout) != HAL_OK)
    {
      SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
      return HAL_TIMEOUT;
    }
    
    hspi->State = HAL_SPI_STATE_READY;

    /* Check if CRC error occurred */
    if((hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE) && (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_CRCERR) != RESET))
    {
      SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_CRC);

      SPI_RESET_CRC(hspi);

      /* Process Unlocked */
      __HAL_UNLOCK(hspi);
      
      return HAL_ERROR; 
    }

    /* Process Unlocked */
    __HAL_UNLOCK(hspi);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}



=======
>>>>>>> parent of 6987c97... HAL_SPI_TransmitReceive
#endif
