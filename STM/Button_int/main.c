#include "main.h"

__IO uint32_t UserButtonStatus = 0;  /* set to 1 after User Button interrupt  */


void SystemClock_Config(void);
void	GENLED_Init(void);
void startup_led(void);
void	Button_Init(void);

int main(void)
{
  /* STM32F103xB HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

/*Configure the clock*/	
	SystemClock_Config();	
	HAL_Delay(10);
	
/*configure PC13 as LED*/
	GENLED_Init();
	startup_led();
	
/*configure PC13 as LED*/
	Button_Init();
	UserButtonStatus=0;
	while(1)
	{
		while(UserButtonStatus == 1)
		{
		HAL_GPIO_WritePin(GENLED_GPIO_PORT, GENLED_PIN, GPIO_PIN_SET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(GENLED_GPIO_PORT, GENLED_PIN, GPIO_PIN_RESET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(GENLED_GPIO_PORT, GENLED_PIN, GPIO_PIN_SET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(GENLED_GPIO_PORT, GENLED_PIN, GPIO_PIN_RESET);
		HAL_Delay(500);
		UserButtonStatus=0;
		}
	}
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 72000000
  *            HCLK(Hz)                       = 72000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            PLLMUL                         = 9
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
 RCC_ClkInitTypeDef clkinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};
  /* Configure PLL ------------------------------------------------------*/
  /* PLL configuration: PLLCLK = (HSI / 2) * PLLMUL = (8 / 2) * 16 = 64 MHz */
  /* PREDIV1 configuration: PREDIV1CLK = PLLCLK / HSEPredivValue = 64 / 1 = 64 MHz */
  /* Enable HSI and activate PLL with HSi_DIV2 as source */
  oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
  oscinitstruct.HSEState        = RCC_HSE_ON;
  oscinitstruct.LSEState        = RCC_LSE_ON;
  oscinitstruct.HSIState        = RCC_HSI_OFF;
  oscinitstruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  oscinitstruct.HSEPredivValue    = RCC_HSE_PREDIV_DIV1;
  oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
  oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE;
  oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&oscinitstruct)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }
}


void	GENLED_Init(void)
{
	static GPIO_InitTypeDef GENLED_InitStruct;
/* -1- Enable DRDY(PA2) Clock (to be able to program the configuration registers) */
    GENLED_GPIO_CLK_ENABLE();
/* Configure Button pin as input */
    GENLED_InitStruct.Pin    = GENLED_PIN;
    GENLED_InitStruct.Mode   = GPIO_MODE_OUTPUT_PP;
    GENLED_InitStruct.Pull   = GPIO_PULLUP;
    GENLED_InitStruct.Speed  = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GENLED_GPIO_PORT, &GENLED_InitStruct);
}


void	Button_Init(void)
{
	static GPIO_InitTypeDef Button_InitStruct;
/* -1- Enable DRDY(PA2) Clock (to be able to program the configuration registers) */
    Button_GPIO_CLK_ENABLE();
/* Configure Button pin as input */
    Button_InitStruct.Pin    = Button_PIN;
    Button_InitStruct.Mode   = GPIO_MODE_IT_FALLING;
    Button_InitStruct.Pull   = GPIO_PULLUP;
    Button_InitStruct.Speed  = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init(Button_GPIO_PORT, &Button_InitStruct);
/* Enable and set Button EXTI Interrupt to the lowest priority */
    HAL_NVIC_SetPriority((IRQn_Type)(EXTI0_IRQn), 0x0F, 0);
    HAL_NVIC_EnableIRQ((IRQn_Type)(EXTI0_IRQn));
}



/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == Button_PIN)
  {  
    UserButtonStatus = 1;
  }
}

void startup_led(void)
{
			HAL_GPIO_WritePin(GENLED_GPIO_PORT, GENLED_PIN, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(GENLED_GPIO_PORT, GENLED_PIN, GPIO_PIN_RESET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(GENLED_GPIO_PORT, GENLED_PIN, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(GENLED_GPIO_PORT, GENLED_PIN, GPIO_PIN_RESET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(GENLED_GPIO_PORT, GENLED_PIN, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(GENLED_GPIO_PORT, GENLED_PIN, GPIO_PIN_RESET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(GENLED_GPIO_PORT, GENLED_PIN, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(GENLED_GPIO_PORT, GENLED_PIN, GPIO_PIN_RESET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(GENLED_GPIO_PORT, GENLED_PIN, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(GENLED_GPIO_PORT, GENLED_PIN, GPIO_PIN_RESET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(GENLED_GPIO_PORT, GENLED_PIN, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(GENLED_GPIO_PORT, GENLED_PIN, GPIO_PIN_RESET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(GENLED_GPIO_PORT, GENLED_PIN, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(GENLED_GPIO_PORT, GENLED_PIN, GPIO_PIN_RESET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(GENLED_GPIO_PORT, GENLED_PIN, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(GENLED_GPIO_PORT, GENLED_PIN, GPIO_PIN_RESET);
		HAL_Delay(100);

}
