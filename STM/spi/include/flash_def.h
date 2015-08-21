/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASH_DEF_H
#define __FLASH_DEF_H

/** @defgroup FLASH_Latency Latency configuration
 *  @brief macros to set the FLASH latency
 * @{
 */ 

/**
  * @brief  Set the FLASH Latency.
  * @param  __LATENCY__: FLASH Latency                   
  *          This parameter can be one of the following values:
  *            @arg FLASH_LATENCY_0:  FLASH Zero Latency cycle
  *            @arg FLASH_LATENCY_1:  FLASH One Latency cycle
  *            @arg FLASH_LATENCY_2:  FLASH Two Latency cycle
  * @retval None
  */ 
#define __HAL_FLASH_SET_LATENCY(__LATENCY__)    (FLASH->ACR = (FLASH->ACR&(~FLASH_ACR_LATENCY)) | (__LATENCY__))

/** @brief  Get the FLASH Latency.
  * @retval FLASH Latency                   
  *          This parameter can be one of the following values:
  *            @arg FLASH_LATENCY_0:  FLASH Zero Latency cycle
  *            @arg FLASH_LATENCY_1:  FLASH One Latency cycle
  *            @arg FLASH_LATENCY_2:  FLASH Two Latency cycle
  */
#define __HAL_FLASH_GET_LATENCY()               (READ_BIT((FLASH->ACR), FLASH_ACR_LATENCY))

/**
  * @}
  */  

/** @defgroup FLASH_Prefetch Prefetch activation or deactivation
 *  @brief macros to set the FLASH Prefetch
 * @{
 */ 

/**
  * @brief  Enable the FLASH prefetch buffer.
  * @retval None
  */ 
#define __HAL_FLASH_PREFETCH_BUFFER_ENABLE()    (FLASH->ACR |= FLASH_ACR_PRFTBE)

/**
  * @brief  Disable the FLASH prefetch buffer.
  * @retval None
  */
#define __HAL_FLASH_PREFETCH_BUFFER_DISABLE()   (FLASH->ACR &= (~FLASH_ACR_PRFTBE))

/**
  * @}
  */  


#endif
