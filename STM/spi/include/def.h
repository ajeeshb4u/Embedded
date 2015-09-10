/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DEF_H
#define __DEF_H


#include "stm32f1xx_it.h"
#include "flash_def.h"
#include "nvic_def.h"
#include "stm32f1xx_hal_rcc.h"
#include "spi_def.h"

#define LED_BLUE_GPIO										GPIOC
#define LED_BLUE_PIN	 									13


/**
  * @brief This is the HAL system configuration section
  */     
#define  VDD_VALUE                    ((uint32_t)3300) /*!< Value of VDD in mv */           
#define  TICK_INT_PRIORITY            ((uint32_t)0x000F)    /*!< tick interrupt priority */            
#define  USE_RTOS                     0     
#define  PREFETCH_ENABLE              1

/******************  GPIO Conf  ************************************************/
#define GPIO_CNF_INPUT_ANALOG		0
#define GPIO_CNF_INPUT_FLOATING		1
#define GPIO_CNF_INPUT_PULLUPDOWN	2

#define GPIO_CNF_OUTPUT_PUSHPULL	0
#define GPIO_CNF_OUTPUT_OPENDRAIN	1
#define GPIO_CNF_AFIO_PUSHPULL		2
#define GPIO_CNF_AFIO_OPENDRAIN		3

#define  GPIO_MODE_INPUT                        ((uint32_t)0x00000000)   /*!< Input Floating Mode                   */


#define GPIO_MODE_OUTPUT10MHz		1
#define GPIO_MODE_OUTPUT2MHz		2
#define GPIO_MODE_OUTPUT50MHz		3

#define GPIOCONF(mode, cnf)	((cnf << 2) | (mode))
#define GPIOPINCONFL(pin, conf) (conf << (pin * 4))
#define GPIOPINCONFH(pin, conf) (conf << ((pin - 8) * 4))
#define CONFMASKL(pin) ((u32)~(15 << (pin * 4)))
#define CONFMASKH(pin) ((u32)~(15 << ((pin - 8) * 4)))


/** @defgroup SPI_Interrupt_configuration_definition SPI Interrupt configuration definition
  * @{
  */
#define SPI_IT_TXE                      SPI_CR2_TXEIE
#define SPI_IT_RXNE                     SPI_CR2_RXNEIE
#define SPI_IT_ERR                      SPI_CR2_ERRIE


/** @defgroup SPI_Flag_definition SPI Flag definition
  * @{
  */
#define SPI_FLAG_RXNE                   SPI_SR_RXNE
#define SPI_FLAG_TXE                    SPI_SR_TXE
#define SPI_FLAG_CRCERR                 SPI_SR_CRCERR
#define SPI_FLAG_MODF                   SPI_SR_MODF
#define SPI_FLAG_OVR                    SPI_SR_OVR
#define SPI_FLAG_BSY                    SPI_SR_BSY

/** @brief  Resets the CRC calculation of the SPI.
  * @param  __HANDLE__: specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define SPI_RESET_CRC(__HANDLE__) do{CLEAR_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_CRCEN);\
                                     SET_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_CRCEN);}while(0)


  #define __HAL_LOCK(__HANDLE__)                                           \
                                do{                                        \
                                    if((__HANDLE__)->Lock == HAL_LOCKED)  \
                                    {                                      \
                                       return HAL_BUSY;                    \
                                    }                                      \
                                    else                                   \
                                    {                                      \
                                       (__HANDLE__)->Lock = HAL_LOCKED;    \
                                    }                                      \
                                  }while (0)

  #define __HAL_UNLOCK(__HANDLE__)                                          \
                                  do{                                       \
                                      (__HANDLE__)->Lock = HAL_UNLOCKED;   \
                                    }while (0)

																		
/** @brief  Disables the SPI.
  * @param  __HANDLE__: specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */                                           
#define __HAL_SPI_DISABLE(__HANDLE__) CLEAR_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_SPE)


/** @brief  Enable the specified SPI interrupts.
  * @param  __HANDLE__: specifies the SPI handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @param  __INTERRUPT__: specifies the interrupt source to enable.
  *         This parameter can be one of the following values:
  *            @arg SPI_IT_TXE: Tx buffer empty interrupt enable
  *            @arg SPI_IT_RXNE: RX buffer not empty interrupt enable
  *            @arg SPI_IT_ERR: Error interrupt enable
  * @retval None
  */
#define __HAL_SPI_ENABLE_IT(__HANDLE__, __INTERRUPT__)   SET_BIT((__HANDLE__)->Instance->CR2, (__INTERRUPT__))

/** @brief  Disable the specified SPI interrupts.
  * @param  __HANDLE__: specifies the SPI handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @param  __INTERRUPT__: specifies the interrupt source to disable.
  *         This parameter can be one of the following values:
  *            @arg SPI_IT_TXE: Tx buffer empty interrupt enable
  *            @arg SPI_IT_RXNE: RX buffer not empty interrupt enable
  *            @arg SPI_IT_ERR: Error interrupt enable
  * @retval None
  */
#define __HAL_SPI_DISABLE_IT(__HANDLE__, __INTERRUPT__)  CLEAR_BIT((__HANDLE__)->Instance->CR2, (__INTERRUPT__))
																		
																		
																		
																		

#define HAL_MAX_DELAY      0xFFFFFFFF

#define HAL_IS_BIT_SET(REG, BIT)         (((REG) & (BIT)) != RESET)
#define HAL_IS_BIT_CLR(REG, BIT)         (((REG) & (BIT)) == RESET)

#define __HAL_LINKDMA(__HANDLE__, __PPP_DMA_FIELD_, __DMA_HANDLE_)           \
                        do{                                                  \
                              (__HANDLE__)->__PPP_DMA_FIELD_ = &(__DMA_HANDLE_); \
                              (__DMA_HANDLE_).Parent = (__HANDLE__);             \
                          } while(0)



typedef enum 
{
  LED2 = 0,

  LED_GREEN = LED2,
  
} Led_TypeDef;

/** @defgroup GPIO_pins_define GPIO pins define
  * @{
  */ 
#define GPIO_PIN_0                 ((uint16_t)0x0001)  /* Pin 0 selected    */
#define GPIO_PIN_1                 ((uint16_t)0x0002)  /* Pin 1 selected    */
#define GPIO_PIN_2                 ((uint16_t)0x0004)  /* Pin 2 selected    */
#define GPIO_PIN_3                 ((uint16_t)0x0008)  /* Pin 3 selected    */
#define GPIO_PIN_4                 ((uint16_t)0x0010)  /* Pin 4 selected    */
#define GPIO_PIN_5                 ((uint16_t)0x0020)  /* Pin 5 selected    */
#define GPIO_PIN_6                 ((uint16_t)0x0040)  /* Pin 6 selected    */
#define GPIO_PIN_7                 ((uint16_t)0x0080)  /* Pin 7 selected    */
#define GPIO_PIN_8                 ((uint16_t)0x0100)  /* Pin 8 selected    */
#define GPIO_PIN_9                 ((uint16_t)0x0200)  /* Pin 9 selected    */
#define GPIO_PIN_10                ((uint16_t)0x0400)  /* Pin 10 selected   */
#define GPIO_PIN_11                ((uint16_t)0x0800)  /* Pin 11 selected   */
#define GPIO_PIN_12                ((uint16_t)0x1000)  /* Pin 12 selected   */
#define GPIO_PIN_13                ((uint16_t)0x2000)  /* Pin 13 selected   */
#define GPIO_PIN_14                ((uint16_t)0x4000)  /* Pin 14 selected   */
#define GPIO_PIN_15                ((uint16_t)0x8000)  /* Pin 15 selected   */
#define GPIO_PIN_All               ((uint16_t)0xFFFF)  /* All pins selected */

#define GPIO_PIN_MASK              ((uint32_t)0x0000FFFF) /* PIN mask for assert test */


/** @defgroup STM32F1XX_NUCLEO_LED LED Constants
  * @{
  */
#define LEDn                             1

#define LED2_PIN                         GPIO_PIN_5
#define LED2_GPIO_PORT                   GPIOA
#define LED2_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()  
#define LED2_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOA_CLK_DISABLE()  

#define LEDx_GPIO_CLK_ENABLE(__INDEX__)  do {LED2_GPIO_CLK_ENABLE(); } while(0)

#define LEDx_GPIO_CLK_DISABLE(__INDEX__) LED2_GPIO_CLK_DISABLE())

GPIO_TypeDef* GPIO_PORT[LEDn] = {LED2_GPIO_PORT};

const uint16_t GPIO_PIN[LEDn] = {LED2_PIN};

void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  /* Check the parameters */
//  assert_param(IS_GPIO_PIN(GPIO_Pin));

  GPIOx->ODR ^= GPIO_Pin;
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled. 
  *   This parameter can be one of following parameters:
  *            @arg  LED2
  * @retval None
  */
void BSP_LED_Toggle(Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(GPIO_PORT[Led], GPIO_PIN[Led]);
}

/**
  * @brief This function is called to increment  a global variable "uwTick"
  *        used as application time base.
  * Note: In the default implementation, this variable is incremented each 1ms
  *       in Systick ISR.
 * Note: This function is declared as __weak to be overwritten in case of other 
  *      implementations in user file.
  * @retval None
  */
static __IO uint32_t uwTick;



__weak void HAL_IncTick(void)
{
  uwTick++;
}




/**
  * @brief Provides a tick value in millisecond.
  * Note: This function is declared as __weak to be overwritten in case of other 
  *       implementations in user file.
  * @retval tick value
  */

__weak uint32_t HAL_GetTick(void)
{
  return uwTick;
}

/**
  * @brief This function provides accurate delay (in milliseconds) based 
  *        on variable incremented.
  * Note: In the default implementation , SysTick timer is the source of time base.
  *       It is used to generate interrupts at regular time intervals where uwTick
  *       is incremented.
  * Note: ThiS function is declared as __weak to be overwritten in case of other
  *       implementations in user file.
  * @param Delay: specifies the delay time length, in milliseconds.
  * @retval None
  */
__weak void HAL_Delay(__IO uint32_t Delay)
{
  uint32_t tickstart = 0;
  tickstart = HAL_GetTick();
  while((HAL_GetTick() - tickstart) < Delay)
  {
  }
}


/** 
  * @brief   GPIO Init structure definition  
  */ 
typedef struct
{
  uint32_t Pin;       /*!< Specifies the GPIO pins to be configured.
                           This parameter can be any value of @ref GPIO_pins_define */

  uint32_t Mode;      /*!< Specifies the operating mode for the selected pins.
                           This parameter can be a value of @ref GPIO_mode_define */
                           
  uint32_t Pull;      /*!< Specifies the Pull-up or Pull-Down activation for the selected pins.
                           This parameter can be a value of @ref GPIO_pull_define */
                           
  uint32_t Speed;     /*!< Specifies the speed for the selected pins.
                           This parameter can be a value of @ref GPIO_speed_define */
}GPIO_InitTypeDef;
 
/** 
  * @brief  GPIO Bit SET and Bit RESET enumeration 
  */
typedef enum
{ 
  GPIO_PIN_RESET = 0,
  GPIO_PIN_SET
}GPIO_PinState;


/** @defgroup GPIO_mode_define GPIO mode define
  * @brief GPIO Configuration Mode 
  *        Elements values convention: 0xX0yz00YZ
  *           - X  : GPIO mode or EXTI Mode
  *           - y  : External IT or Event trigger detection 
  *           - z  : IO configuration on External IT or Event
  *           - Y  : Output type (Push Pull or Open Drain)
  *           - Z  : IO Direction mode (Input, Output, Alternate or Analog)
  * @{
  */ 
//#define  GPIO_MODE_INPUT                        ((uint32_t)0x00000000)   /*!< Input Floating Mode                   */
#define  GPIO_MODE_OUTPUT_PP                    ((uint32_t)0x00000001)   /*!< Output Push Pull Mode                 */
#define  GPIO_MODE_OUTPUT_OD                    ((uint32_t)0x00000011)   /*!< Output Open Drain Mode                */
#define  GPIO_MODE_AF_PP                        ((uint32_t)0x00000002)   /*!< Alternate Function Push Pull Mode     */
#define  GPIO_MODE_AF_OD                        ((uint32_t)0x00000012)   /*!< Alternate Function Open Drain Mode    */
#define  GPIO_MODE_AF_INPUT                     GPIO_MODE_INPUT          /*!< Alternate Function Input Mode         */

#define  GPIO_MODE_ANALOG                       ((uint32_t)0x00000003)   /*!< Analog Mode  */
    
#define  GPIO_MODE_IT_RISING                    ((uint32_t)0x10110000)   /*!< External Interrupt Mode with Rising edge trigger detection          */
#define  GPIO_MODE_IT_FALLING                   ((uint32_t)0x10210000)   /*!< External Interrupt Mode with Falling edge trigger detection         */
#define  GPIO_MODE_IT_RISING_FALLING            ((uint32_t)0x10310000)   /*!< External Interrupt Mode with Rising/Falling edge trigger detection  */
 
#define  GPIO_MODE_EVT_RISING                   ((uint32_t)0x10120000)   /*!< External Event Mode with Rising edge trigger detection               */
#define  GPIO_MODE_EVT_FALLING                  ((uint32_t)0x10220000)   /*!< External Event Mode with Falling edge trigger detection              */
#define  GPIO_MODE_EVT_RISING_FALLING           ((uint32_t)0x10320000)   /*!< External Event Mode with Rising/Falling edge trigger detection       */


 /** @defgroup GPIO_pull_define GPIO pull define
   * @brief GPIO Pull-Up or Pull-Down Activation
   * @{
   */  
#define  GPIO_NOPULL        ((uint32_t)0x00000000)   /*!< No Pull-up or Pull-down activation  */
#define  GPIO_PULLUP        ((uint32_t)0x00000001)   /*!< Pull-up activation                  */
#define  GPIO_PULLDOWN      ((uint32_t)0x00000002)   /*!< Pull-down activation                */


/** @defgroup GPIO_speed_define GPIO speed define
  * @brief GPIO Output Maximum frequency
  * @{
  */  
#define  GPIO_SPEED_LOW              (GPIO_CRL_MODE0_1) /*!< Low speed */
#define  GPIO_SPEED_MEDIUM           (GPIO_CRL_MODE0_0) /*!< Medium speed */
#define  GPIO_SPEED_HIGH             (GPIO_CRL_MODE0)   /*!< High speed */


#define GPIO_MODE             ((uint32_t)0x00000003)
#define EXTI_MODE             ((uint32_t)0x10000000)
#define GPIO_MODE_IT          ((uint32_t)0x00010000)
#define GPIO_MODE_EVT         ((uint32_t)0x00020000)
#define RISING_EDGE           ((uint32_t)0x00100000) 
#define FALLING_EDGE          ((uint32_t)0x00200000) 
#define GPIO_OUTPUT_TYPE      ((uint32_t)0x00000010) 
#define GPIO_NUMBER           ((uint32_t)16)

/* Definitions for bit manipulation of CRL and CRH register */
#define  GPIO_CR_MODE_INPUT         ((uint32_t)0x00000000) /*!< 00: Input mode (reset state)  */
#define  GPIO_CR_CNF_ANALOG         ((uint32_t)0x00000000) /*!< 00: Analog mode  */
#define  GPIO_CR_CNF_INPUT_FLOATING ((uint32_t)0x00000004) /*!< 01: Floating input (reset state)  */
#define  GPIO_CR_CNF_INPUT_PU_PD    ((uint32_t)0x00000008) /*!< 10: Input with pull-up / pull-down  */
#define  GPIO_CR_CNF_GP_OUTPUT_PP   ((uint32_t)0x00000000) /*!< 00: General purpose output push-pull  */
#define  GPIO_CR_CNF_GP_OUTPUT_OD   ((uint32_t)0x00000004) /*!< 01: General purpose output Open-drain  */
#define  GPIO_CR_CNF_AF_OUTPUT_PP   ((uint32_t)0x00000008) /*!< 10: Alternate function output Push-pull  */
#define  GPIO_CR_CNF_AF_OUTPUT_OD   ((uint32_t)0x0000000C) /*!< 11: Alternate function output Open-drain  */


#define GPIO_GET_INDEX(__GPIOx__) (((__GPIOx__) == (GPIOA))? 0U :\
                                   ((__GPIOx__) == (GPIOB))? 1U :\
                                   ((__GPIOx__) == (GPIOC))? 2U :\
                                   ((__GPIOx__) == (GPIOD))? 3U :4U)



/**
  * @brief  Initializes the GPIOx peripheral according to the specified parameters in the GPIO_Init.
  * @param  GPIOx: where x can be (A..G depending on device used) to select the GPIO peripheral
  * @param  GPIO_Init: pointer to a GPIO_InitTypeDef structure that contains
  *         the configuration information for the specified GPIO peripheral.
  * @retval None
  */
void HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init)
{
  uint32_t position;
  uint32_t ioposition = 0x00;
  uint32_t iocurrent = 0x00;
  uint32_t temp = 0x00;
  uint32_t config = 0x00;
  __IO uint32_t *configregister; /* Store the address of CRL or CRH register based on pin number */
  uint32_t registeroffset = 0; /* offset used during computation of CNF and MODE bits placement inside CRL or CRH register */
  
  /* Check the parameters */
//  assert_param(IS_GPIO_ALL_INSTANCE(GPIOx));
//  assert_param(IS_GPIO_PIN(GPIO_Init->Pin));
//  assert_param(IS_GPIO_MODE(GPIO_Init->Mode));
//  assert_param(IS_GPIO_PULL(GPIO_Init->Pull)); 

  /* Configure the port pins */
  for (position = 0; position < GPIO_NUMBER; position++)
  {
    /* Get the IO position */
    ioposition = ((uint32_t)0x01) << position;
    
    /* Get the current IO position */
    iocurrent = (uint32_t)(GPIO_Init->Pin) & ioposition;

    if (iocurrent == ioposition)
    {
      /* Check the Alternate function parameters */
  //    assert_param(IS_GPIO_AF_INSTANCE(GPIOx));

      /* Based on the required mode, filling config variable with MODEy[1:0] and CNFy[3:2] corresponding bits */
      switch (GPIO_Init->Mode)
      {
        /* If we are configuring the pin in OUTPUT push-pull mode */
        case GPIO_MODE_OUTPUT_PP:
          /* Check the GPIO speed parameter */
 //         assert_param(IS_GPIO_SPEED(GPIO_Init->Speed));
          config = GPIO_Init->Speed + GPIO_CR_CNF_GP_OUTPUT_PP;
          break;
          
        /* If we are configuring the pin in OUTPUT open-drain mode */
        case GPIO_MODE_OUTPUT_OD:
          /* Check the GPIO speed parameter */
  //        assert_param(IS_GPIO_SPEED(GPIO_Init->Speed));
          config = GPIO_Init->Speed + GPIO_CR_CNF_GP_OUTPUT_OD;
          break;
          
        /* If we are configuring the pin in ALTERNATE FUNCTION push-pull mode */
        case GPIO_MODE_AF_PP:
          /* Check the GPIO speed parameter */
   //       assert_param(IS_GPIO_SPEED(GPIO_Init->Speed));
          config = GPIO_Init->Speed + GPIO_CR_CNF_AF_OUTPUT_PP;
          break;
          
        /* If we are configuring the pin in ALTERNATE FUNCTION open-drain mode */
        case GPIO_MODE_AF_OD:
          /* Check the GPIO speed parameter */
   //       assert_param(IS_GPIO_SPEED(GPIO_Init->Speed));
          config = GPIO_Init->Speed + GPIO_CR_CNF_AF_OUTPUT_OD;
          break;
          
        /* If we are configuring the pin in INPUT (also applicable to EVENT and IT mode) */
        case GPIO_MODE_INPUT:
        case GPIO_MODE_IT_RISING:
        case GPIO_MODE_IT_FALLING:
        case GPIO_MODE_IT_RISING_FALLING:
        case GPIO_MODE_EVT_RISING:
        case GPIO_MODE_EVT_FALLING:
        case GPIO_MODE_EVT_RISING_FALLING:
          /* Check the GPIO pull parameter */
     //     assert_param(IS_GPIO_PULL(GPIO_Init->Pull));
          if(GPIO_Init->Pull == GPIO_NOPULL)
          {  
            config = GPIO_CR_MODE_INPUT + GPIO_CR_CNF_INPUT_FLOATING;
          }
          else if(GPIO_Init->Pull == GPIO_PULLUP)
          {
            config = GPIO_CR_MODE_INPUT + GPIO_CR_CNF_INPUT_PU_PD;
            
            /* Set the corresponding ODR bit */
            GPIOx->BSRR = ioposition;
          }
          else /* GPIO_PULLDOWN */
          {
            config = GPIO_CR_MODE_INPUT + GPIO_CR_CNF_INPUT_PU_PD;
            
            /* Reset the corresponding ODR bit */
            GPIOx->BRR = ioposition;
          }
          break; 
          
        /* If we are configuring the pin in INPUT analog mode */
        case GPIO_MODE_ANALOG:
            config = GPIO_CR_MODE_INPUT + GPIO_CR_CNF_ANALOG;
          break;
        
        /* Parameters are checked with assert_param */
        default:
          break;
      }
      
      /* Check if the current bit belongs to first half or last half of the pin count number
       in order to address CRH or CRL register*/
      configregister = (iocurrent < GPIO_PIN_8) ? &GPIOx->CRL     : &GPIOx->CRH;
      registeroffset = (iocurrent < GPIO_PIN_8) ? (position << 2) : ((position - 8) << 2);
      
      /* Apply the new configuration of the pin to the register */
      MODIFY_REG((*configregister), ((GPIO_CRL_MODE0 | GPIO_CRL_CNF0) << registeroffset ), (config << registeroffset));
      
      /*--------------------- EXTI Mode Configuration ------------------------*/
      /* Configure the External Interrupt or event for the current IO */
      if((GPIO_Init->Mode & EXTI_MODE) == EXTI_MODE) 
      {
        /* Enable AFIO Clock */
        __HAL_RCC_AFIO_CLK_ENABLE();
        temp = AFIO->EXTICR[position >> 2];
        CLEAR_BIT(temp, ((uint32_t)0x0F) << (4 * (position & 0x03)));
        SET_BIT(temp, (GPIO_GET_INDEX(GPIOx)) << (4 * (position & 0x03)));
        AFIO->EXTICR[position >> 2] = temp;
        

        /* Configure the interrupt mask */
        if((GPIO_Init->Mode & GPIO_MODE_IT) == GPIO_MODE_IT)
        {
          SET_BIT(EXTI->IMR, iocurrent); 
        } 
        else
        {
          CLEAR_BIT(EXTI->IMR, iocurrent); 
        } 
        
        /* Configure the event mask */
        if((GPIO_Init->Mode & GPIO_MODE_EVT) == GPIO_MODE_EVT)
        {
          SET_BIT(EXTI->EMR, iocurrent); 
        } 
        else
        {
          CLEAR_BIT(EXTI->EMR, iocurrent); 
        }
        
        /* Enable or disable the rising trigger */
        if((GPIO_Init->Mode & RISING_EDGE) == RISING_EDGE)
        {
          SET_BIT(EXTI->RTSR, iocurrent); 
        } 
        else
        {
          CLEAR_BIT(EXTI->RTSR, iocurrent); 
        }
        
        /* Enable or disable the falling trigger */
        if((GPIO_Init->Mode & FALLING_EDGE) == FALLING_EDGE)
        {
          SET_BIT(EXTI->FTSR, iocurrent); 
        } 
        else
        {
          CLEAR_BIT(EXTI->FTSR, iocurrent); 
        }
      }
    }
  }
}


/**
  * @brief  Sets or clears the selected data port bit.
  * 
  * @note   This function uses GPIOx_BSRR register to allow atomic read/modify 
  *         accesses. In this way, there is no risk of an IRQ occurring between
  *         the read and the modify access.
  *               
  * @param  GPIOx: where x can be (A..G depending on device used) to select the GPIO peripheral
  * @param  GPIO_Pin: specifies the port bit to be written.
  *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
  * @param  PinState: specifies the value to be written to the selected bit.
  *          This parameter can be one of the GPIO_PinState enum values:
  *            @arg GPIO_BIT_RESET: to clear the port pin
  *            @arg GPIO_BIT_SET: to set the port pin
  * @retval None
  */
void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
  /* Check the parameters */
//  assert_param(IS_GPIO_PIN(GPIO_Pin));
//  assert_param(IS_GPIO_PIN_ACTION(PinState));

  if(PinState != GPIO_PIN_RESET)
  {
    GPIOx->BSRR = GPIO_Pin;
  }
  else
  {
    GPIOx->BSRR = (uint32_t)GPIO_Pin << 16;
  }
}



/**
  * @brief  Configures LED GPIO.
  * @param  Led: LED to be configured. 
  *          This parameter can be one of the following values:
  *     @arg LED2
  * @retval None
  */
void BSP_LED_Init(Led_TypeDef Led)
{
  GPIO_InitTypeDef  gpioinitstruct = {0};
  
  /* Enable the GPIO_LED Clock */
  LEDx_GPIO_CLK_ENABLE(Led);

  /* Configure the GPIO_LED pin */
  gpioinitstruct.Pin    = GPIO_PIN[Led];
  gpioinitstruct.Mode   = GPIO_MODE_OUTPUT_PP;
  gpioinitstruct.Pull   = GPIO_PULLUP;
  gpioinitstruct.Speed  = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIO_PORT[Led], &gpioinitstruct);

  /* Reset PIN to switch off the LED */
  HAL_GPIO_WritePin(GPIO_PORT[Led],GPIO_PIN[Led], GPIO_PIN_RESET);
}

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


/** @brief  Check whether the specified SPI flag is set or not.
  * @param  __HANDLE__: specifies the SPI handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @param  __FLAG__: specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg SPI_FLAG_RXNE: Receive buffer not empty flag
  *            @arg SPI_FLAG_TXE: Transmit buffer empty flag
  *            @arg SPI_FLAG_CRCERR: CRC error flag
  *            @arg SPI_FLAG_MODF: Mode fault flag
  *            @arg SPI_FLAG_OVR: Overrun flag
  *            @arg SPI_FLAG_BSY: Busy flag
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_SPI_GET_FLAG(__HANDLE__, __FLAG__) ((((__HANDLE__)->Instance->SR) & (__FLAG__)) == (__FLAG__))

/** @brief  Clear the SPI CRCERR pending flag.
  * @param  __HANDLE__: specifies the SPI handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define __HAL_SPI_CLEAR_CRCERRFLAG(__HANDLE__) ((__HANDLE__)->Instance->SR = ~(SPI_FLAG_CRCERR))

/** @brief  Clear the SPI MODF pending flag.
  * @param  __HANDLE__: specifies the SPI handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral. 
  * @retval None
  */



/**
  * @brief  This function handles SPI Communication Timeout.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *                the configuration information for SPI module.
  * @param  Flag: SPI flag to check
  * @param  Status: Flag status to check: RESET or set
  * @param  Timeout: Timeout duration
  * @retval HAL status
  */
static HAL_StatusTypeDef SPI_WaitOnFlagUntilTimeout(SPI_HandleTypeDef *hspi, uint32_t Flag, FlagStatus Status, uint32_t Timeout)  
{
  uint32_t tickstart = 0;

  /* Get tick */ 
  tickstart = HAL_GetTick();

  /* Wait until flag is set */
  if(Status == RESET)
  {
    while(__HAL_SPI_GET_FLAG(hspi, Flag) == RESET)
    {
      if(Timeout != HAL_MAX_DELAY)
      {
        if((Timeout == 0) || ((HAL_GetTick() - tickstart ) > Timeout))
        {
          /* Disable the SPI and reset the CRC: the CRC value should be cleared
             on both master and slave sides in order to resynchronize the master
             and slave for their respective CRC calculation */

          /* Disable TXE, RXNE and ERR interrupts for the interrupt process */
          __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

          /* Disable SPI peripheral */
          __HAL_SPI_DISABLE(hspi);

          /* Reset CRC Calculation */
          if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
          {
            SPI_RESET_CRC(hspi);
          }

          hspi->State= HAL_SPI_STATE_READY;

          /* Process Unlocked */
          __HAL_UNLOCK(hspi);

          return HAL_TIMEOUT;
        }
      }
    }
  }
  else
  {
    while(__HAL_SPI_GET_FLAG(hspi, Flag) != RESET)
    {
      if(Timeout != HAL_MAX_DELAY)
      {
        if((Timeout == 0) || ((HAL_GetTick() - tickstart ) > Timeout))
        {
          /* Disable the SPI and reset the CRC: the CRC value should be cleared
             on both master and slave sides in order to resynchronize the master
             and slave for their respective CRC calculation */

          /* Disable TXE, RXNE and ERR interrupts for the interrupt process */
          __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

          /* Disable SPI peripheral */
          __HAL_SPI_DISABLE(hspi);

          /* Reset CRC Calculation */
          if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
          {
            SPI_RESET_CRC(hspi);
          }

          hspi->State= HAL_SPI_STATE_READY;

          /* Process Unlocked */
          __HAL_UNLOCK(hspi);

          return HAL_TIMEOUT;
        }
      }
    }
  }
  return HAL_OK;
}




#endif
