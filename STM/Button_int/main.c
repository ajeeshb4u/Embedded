#include "main.h"

/** @addtogroup TIM_TimeBase
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* TIM handle declaration */
TIM_HandleTypeDef    TimHandle;
UART_HandleTypeDef UartHandle;

__IO ITStatus UartReady = RESET;
__IO ITStatus RxReady = RESET;
__IO ITStatus RxReadycmd = RESET;
__IO uint32_t UserButtonStatus = 0;  /* set to 1 after User Button interrupt  */
__IO uint32_t uwPrescalerValue = 0;  /* Prescaler for tomer */
__IO uint32_t int_no=0;

/* Buffer used for transmission */
uint8_t aTxBuffer[] = " ****UART_TwoBoards_ComIT****  ****UART_TwoBoards_ComIT****  ****UART_TwoBoards_ComIT**** ";

/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];

void SystemClock_Config(void);
void	GENLED_Init(void);
void startup_led(void);
void	Button_Init(void);
static void Error_Handler(void);

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

	  /*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART configured as follows:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = None
      - BaudRate = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle.Instance        = USARTx;

  UartHandle.Init.BaudRate   = 9600;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;
  if(HAL_UART_DeInit(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }  
  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }

/*##Put UART peripheral in reception process ###########################*/  
  if(HAL_UART_Receive_IT(&UartHandle, (uint8_t *)aRxBuffer, 0x0a) != HAL_OK)
  {
    Error_Handler();
  }

  /*##-1- Configure the TIM peripheral #######################################*/
  /* -----------------------------------------------------------------------
    In this example TIM3 input clock (TIM3CLK)  is set to APB1 clock (PCLK1) x2,
    since APB1 prescaler is set to 4 (0x100).
       TIM3CLK = PCLK1*2
       PCLK1   = HCLK/2
    => TIM3CLK = PCLK1*2 = (HCLK/2)*2 = HCLK = SystemCoreClock
    To get TIM3 counter clock at 10 KHz, the Prescaler is computed as following:
    Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    Prescaler = (SystemCoreClock /10 KHz) - 1

    Note:
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f1xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
  ----------------------------------------------------------------------- */

  /* Compute the prescaler value to have TIMx counter clock equal to 10000 Hz */
//   uwPrescalerValue = (uint32_t)(SystemCoreClock / 10000) - 1;
uwPrescalerValue = 36000;	/* For 1 sec cnt = 72000000 */

  /* Set TIMx instance */
  TimHandle.Instance = TIMx;

  /* Initialize TIMx peripheral as follows:
       + Period = 10000 - 1
       + Prescaler = (SystemCoreClock/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
  TimHandle.Init.Period            = 2000;
  TimHandle.Init.Prescaler         = uwPrescalerValue;
  TimHandle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle.Init.RepetitionCounter = 0;

  if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if (HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }

	
/*configure PC13 as LED*/
	Button_Init();
	UserButtonStatus=0;

	while(1)
	{
		if (RxReady==SET)
		{
			while(1);
			RxReady=RESET;
// 			/*##Put UART peripheral in reception process ###########################*/  
// 			if(HAL_UART_Receive_IT(&UartHandle, (uint8_t *)aRxBuffer, 0x0a) != HAL_OK)
// 			{
// 				Error_Handler();
// 			}
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
	startup_led();		
//     UserButtonStatus = 1;
  }
}

void startup_led(void)
{
		HAL_GPIO_TogglePin(GENLED_GPIO_PORT, GENLED_PIN);
		HAL_Delay(100);
		HAL_GPIO_TogglePin(GENLED_GPIO_PORT, GENLED_PIN);
		HAL_Delay(100);
		HAL_GPIO_TogglePin(GENLED_GPIO_PORT, GENLED_PIN);
		HAL_Delay(100);
		HAL_GPIO_TogglePin(GENLED_GPIO_PORT, GENLED_PIN);
		HAL_Delay(100);
		HAL_GPIO_TogglePin(GENLED_GPIO_PORT, GENLED_PIN);
		HAL_Delay(100);
		HAL_GPIO_TogglePin(GENLED_GPIO_PORT, GENLED_PIN);
		HAL_Delay(100);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
	NVIC_SystemReset();
//  while (1);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	HAL_GPIO_TogglePin(GENLED_GPIO_PORT, GENLED_PIN);
  /*##Start the transmission process #####################################*/  
  /* While the UART in reception process, user can transmit data through 
     "aTxBuffer" buffer */
  if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)aTxBuffer, TXBUFFERSIZE)!= HAL_OK)
  {
    Error_Handler();
  }
  /*##Wait for the end of the transfer ###################################*/   
  while (UartReady != SET)
  {
  }
  /* Reset transmission flag */
  UartReady = RESET;
}

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of IT Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
  UartReady = SET;
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
  RxReady = SET;
}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
    Error_Handler();
}

