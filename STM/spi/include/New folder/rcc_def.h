#ifndef __RCC_DEF_H
#define __RCC_DEF_H
/** 
  * @brief  RCC System, AHB and APB busses clock configuration structure definition  
  */
typedef struct
{
  uint32_t ClockType;             /*!< The clock to be configured.
                                       This parameter can be a value of @ref RCC_System_Clock_Type */
  
  uint32_t SYSCLKSource;          /*!< The clock source (SYSCLKS) used as system clock.
                                       This parameter can be a value of @ref RCC_System_Clock_Source */

  uint32_t AHBCLKDivider;         /*!< The AHB clock (HCLK) divider. This clock is derived from the system clock (SYSCLK).
                                       This parameter can be a value of @ref RCC_AHB_Clock_Source */
  
  uint32_t APB1CLKDivider;        /*!< The APB1 clock (PCLK1) divider. This clock is derived from the AHB clock (HCLK).
                                       This parameter can be a value of @ref RCC_APB1_APB2_Clock_Source */
  
  uint32_t APB2CLKDivider;        /*!< The APB2 clock (PCLK2) divider. This clock is derived from the AHB clock (HCLK).
                                       This parameter can be a value of @ref RCC_APB1_APB2_Clock_Source */

} RCC_ClkInitTypeDef;


/** 
  * @brief  RCC PLL configuration structure definition  
  */
typedef struct
{
  uint32_t PLLState;     /*!< The new state of the PLL.
                              This parameter can be a value of @ref __HAL_RCC_PLL_CONFIG */

  uint32_t PLLSource;    /*!< PLLSource: PLL entry clock source.
                              This parameter must be a value of @ref RCC_PLL_Clock_Source */           

  uint32_t PLLMUL;         /*!< PLLMUL: Multiplication factor for PLL VCO input clock
                              This parameter must be a value of @ref RCCEx_PLL_Multiplication_Factor */        
} RCC_PLLInitTypeDef;



/** 
  * @brief  RCC Internal/External Oscillator (HSE, HSI, LSE and LSI) configuration structure definition  
  */
typedef struct
{
  uint32_t OscillatorType;       /*!< The oscillators to be configured.
                                       This parameter can be a value of @ref RCC_Oscillator_Type */

#if defined(STM32F105xC) || defined(STM32F107xC)
  uint32_t Prediv1Source;       /*!<  The Prediv1 source value.
                                       This parameter can be a value of @ref RCCEx_Prediv1_Source */
#endif /* STM32F105xC || STM32F107xC */

  uint32_t HSEState;              /*!< The new state of the HSE.
                                       This parameter can be a value of @ref __HAL_RCC_HSE_CONFIG */
                          
  uint32_t HSEPredivValue;       /*!<  The Prediv1 factor value (named PREDIV1 or PLLXTPRE in RM)
                                       This parameter can be a value of @ref RCCEx_Prediv1_Factor */

  uint32_t LSEState;              /*!<  The new state of the LSE.
                                        This parameter can be a value of @ref __HAL_RCC_LSE_CONFIG */
                                          
  uint32_t HSIState;              /*!< The new state of the HSI.
                                       This parameter can be a value of @ref RCC_HSI_Config */

  uint32_t HSICalibrationValue;   /*!< The HSI calibration trimming value (default is RCC_HSICALIBRATION_DEFAULT).
                                       This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F */
                               
  uint32_t LSIState;              /*!<  The new state of the LSI.
                                        This parameter can be a value of @ref RCC_LSI_Config */

  RCC_PLLInitTypeDef PLL;         /*!< PLL structure parameters */      

#if defined(STM32F105xC) || defined(STM32F107xC)
  RCC_PLL2InitTypeDef PLL2;         /*!< PLL2 structure parameters */      
#endif /* STM32F105xC || STM32F107xC */
} RCC_OscInitTypeDef;

/** @defgroup RCC_Oscillator_Type Oscillator Type
  * @{
  */
#define RCC_OSCILLATORTYPE_NONE            ((uint32_t)0x00000000)
#define RCC_OSCILLATORTYPE_HSE             ((uint32_t)0x00000001)
#define RCC_OSCILLATORTYPE_HSI             ((uint32_t)0x00000002)
#define RCC_OSCILLATORTYPE_LSE             ((uint32_t)0x00000004)
#define RCC_OSCILLATORTYPE_LSI             ((uint32_t)0x00000008)


/** @defgroup __HAL_RCC_HSE_CONFIG HSE Config
  * @{
  */
#define RCC_HSE_OFF                      ((uint32_t)0x00000000) /*!< HSE clock deactivation */
#define RCC_HSE_ON                       ((uint32_t)0x00000001) /*!< HSE clock activation */
#define RCC_HSE_BYPASS                   ((uint32_t)0x00000005) /*!< External clock source for HSE clock */

/**
  * @}
  */

/** @defgroup __HAL_RCC_LSE_CONFIG LSE Config
  * @{
  */
#define RCC_LSE_OFF                      ((uint32_t)0x00000000) /*!< LSE clock deactivation */
#define RCC_LSE_ON                       ((uint32_t)0x00000001) /*!< LSE clock activation */
#define RCC_LSE_BYPASS                   ((uint32_t)0x00000005) /*!< External clock source for LSE clock */

/** @defgroup RCC_HSI_Config HSI Config
  * @{
  */
#define RCC_HSI_OFF                      ((uint32_t)0x00000000)   /*!< HSI clock deactivation */
#define RCC_HSI_ON                       RCC_CR_HSION             /*!< HSI clock activation */

#define RCC_HSICALIBRATION_DEFAULT       ((uint32_t)0x10)         /* Default HSI calibration trimming value */


/** @defgroup RCCEx_Prediv1_Factor HSE Prediv1 Factor
  * @{
  */

#define RCC_HSE_PREDIV_DIV1              ((uint32_t)0x00000000)


/** @defgroup __HAL_RCC_PLL_CONFIG PLL Config
  * @{
  */
#define RCC_PLL_NONE                      ((uint32_t)0x00000000)  /*!< PLL is not configured */
#define RCC_PLL_OFF                       ((uint32_t)0x00000001)  /*!< PLL deactivation */
#define RCC_PLL_ON                        ((uint32_t)0x00000002)  /*!< PLL activation */


/** @defgroup RCC_PLL_Clock_Source PLL Clock Source
  * @{
  */

#define RCC_PLLSOURCE_HSI_DIV2      ((uint32_t)0x00000000)  /*!< HSI clock divided by 2 selected as PLL entry clock source */
#define RCC_PLLSOURCE_HSE           RCC_CFGR_PLLSRC         /*!< HSE clock selected as PLL entry clock source */



/** @defgroup RCCEx_PLL_Multiplication_Factor PLL Multiplication Factor
  * @{
  */

#if defined(STM32F105xC) || defined(STM32F107xC)
#else
#define RCC_PLL_MUL2                    RCC_CFGR_PLLMULL2
#define RCC_PLL_MUL3                    RCC_CFGR_PLLMULL3
#endif /* STM32F105xC || STM32F107xC */
#define RCC_PLL_MUL4                    RCC_CFGR_PLLMULL4
#define RCC_PLL_MUL5                    RCC_CFGR_PLLMULL5
#define RCC_PLL_MUL6                    RCC_CFGR_PLLMULL6
#define RCC_PLL_MUL7                    RCC_CFGR_PLLMULL7
#define RCC_PLL_MUL8                    RCC_CFGR_PLLMULL8
#define RCC_PLL_MUL9                    RCC_CFGR_PLLMULL9
#if defined(STM32F105xC) || defined(STM32F107xC)
#define RCC_PLL_MUL6_5                  RCC_CFGR_PLLMULL6_5
#else
#define RCC_PLL_MUL10                   RCC_CFGR_PLLMULL10
#define RCC_PLL_MUL11                   RCC_CFGR_PLLMULL11
#define RCC_PLL_MUL12                   RCC_CFGR_PLLMULL12
#define RCC_PLL_MUL13                   RCC_CFGR_PLLMULL13
#define RCC_PLL_MUL14                   RCC_CFGR_PLLMULL14
#define RCC_PLL_MUL15                   RCC_CFGR_PLLMULL15
#define RCC_PLL_MUL16                   RCC_CFGR_PLLMULL16
#endif /* STM32F105xC || STM32F107xC */

/** @defgroup RCC_System_Clock_Type System Clock Type
  * @{
  */
#define RCC_CLOCKTYPE_SYSCLK             ((uint32_t)0x00000001) /*!< SYSCLK to configure */
#define RCC_CLOCKTYPE_HCLK               ((uint32_t)0x00000002) /*!< HCLK to configure */
#define RCC_CLOCKTYPE_PCLK1              ((uint32_t)0x00000004) /*!< PCLK1 to configure */
#define RCC_CLOCKTYPE_PCLK2              ((uint32_t)0x00000008) /*!< PCLK2 to configure */

/** @defgroup RCC_System_Clock_Source System Clock Source
  * @{
  */
#define RCC_SYSCLKSOURCE_HSI             RCC_CFGR_SW_HSI      /*!< HSI selected as system clock */
#define RCC_SYSCLKSOURCE_HSE             RCC_CFGR_SW_HSE      /*!< HSE selected as system clock */
#define RCC_SYSCLKSOURCE_PLLCLK          RCC_CFGR_SW_PLL      /*!< PLL selected as system clock */


/** @defgroup RCC_AHB_Clock_Source AHB Clock Source
  * @{
  */
#define RCC_SYSCLK_DIV1                  (RCC_CFGR_HPRE_DIV1)   /*!< SYSCLK not divided */
#define RCC_SYSCLK_DIV2                  (RCC_CFGR_HPRE_DIV2)   /*!< SYSCLK divided by 2 */
#define RCC_SYSCLK_DIV4                  (RCC_CFGR_HPRE_DIV4)   /*!< SYSCLK divided by 4 */
#define RCC_SYSCLK_DIV8                  (RCC_CFGR_HPRE_DIV8)   /*!< SYSCLK divided by 8 */
#define RCC_SYSCLK_DIV16                 (RCC_CFGR_HPRE_DIV16)  /*!< SYSCLK divided by 16 */
#define RCC_SYSCLK_DIV64                 (RCC_CFGR_HPRE_DIV64)  /*!< SYSCLK divided by 64 */
#define RCC_SYSCLK_DIV128                (RCC_CFGR_HPRE_DIV128) /*!< SYSCLK divided by 128 */
#define RCC_SYSCLK_DIV256                (RCC_CFGR_HPRE_DIV256) /*!< SYSCLK divided by 256 */
#define RCC_SYSCLK_DIV512                (RCC_CFGR_HPRE_DIV512) /*!< SYSCLK divided by 512 */


/** @defgroup FLASH_Latency_Values Latency Values
  * @{
  */ 
#define FLASH_LATENCY_0            ((uint8_t)0x0000)    /*!< FLASH Zero Latency cycle */

#if defined(STM32F100xB) || defined(STM32F100xE)
/* Only Latency0 supported on value lines */
#else
#define FLASH_LATENCY_1            FLASH_ACR_LATENCY_0  /*!< FLASH One Latency cycle */
#define FLASH_LATENCY_2            FLASH_ACR_LATENCY_1  /*!< FLASH Two Latency cycles */

#endif


/** @defgroup RCC_APB1_APB2_Clock_Source APB1 APB2 Clock Source
  * @{
  */
#define RCC_HCLK_DIV1                    (RCC_CFGR_PPRE1_DIV1)  /*!< HCLK not divided */
#define RCC_HCLK_DIV2                    (RCC_CFGR_PPRE1_DIV2)  /*!< HCLK divided by 2 */
#define RCC_HCLK_DIV4                    (RCC_CFGR_PPRE1_DIV4)  /*!< HCLK divided by 4 */
#define RCC_HCLK_DIV8                    (RCC_CFGR_PPRE1_DIV8)  /*!< HCLK divided by 8 */
#define RCC_HCLK_DIV16                   (RCC_CFGR_PPRE1_DIV16) /*!< HCLK divided by 16 */




#endif