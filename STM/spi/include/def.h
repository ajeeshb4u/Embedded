/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DEF_H
#define __DEF_H


#include "stm32f1xx_it.h"
#include "flash_def.h"
#include "nvic_def.h"

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

#define GPIO_MODE_INPUT				0
#define GPIO_MODE_OUTPUT10MHz		1
#define GPIO_MODE_OUTPUT2MHz		2
#define GPIO_MODE_OUTPUT50MHz		3

#define GPIOCONF(mode, cnf)	((cnf << 2) | (mode))
#define GPIOPINCONFL(pin, conf) (conf << (pin * 4))
#define GPIOPINCONFH(pin, conf) (conf << ((pin - 8) * 4))
#define CONFMASKL(pin) ((u32)~(15 << (pin * 4)))
#define CONFMASKH(pin) ((u32)~(15 << ((pin - 8) * 4)))


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

#define LED2_PIN                         GPIO_PIN_13
#define LED2_GPIO_PORT                   GPIOC
#define LED2_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOC_CLK_ENABLE()  
#define LED2_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOC_CLK_DISABLE()  

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
  * @brief  HAL Status structures definition  
  */  
typedef enum 
{
  HAL_OK       = 0x00,
  HAL_ERROR    = 0x01,
  HAL_BUSY     = 0x02,
  HAL_TIMEOUT  = 0x03
} HAL_StatusTypeDef;



#endif
