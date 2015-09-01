#include <stm32f10x.h>
#include "include/def.h"
#include "include/Serial.h"							//For serial Port setup
#include "include/main.h"
#include "include/stm32f1xx_hal_rcc.h"
#include "include/stm32f1xx_hal_rcc_ex.h"

/**********************************	USART DATA	******************************************/
char g[20]={0x55,0xaa,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,0x10,0x11,0x12};

/* SPI handler declaration */
SPI_HandleTypeDef SpiHandle; 				/*defined a struct type SPI_HandleTypeDef */
	
/* Buffer used for transmission */
uint8_t aTxBuffer[] = "****SPI - Two Boards communication based on Polling **** SPI Message ******** SPI Message ******** SPI Message ****";

/* Buffer used for reception */
uint8_t aRxBuffer[BUFFERSIZE];
	
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Error_Handler(void);	
static uint16_t Buffercmp(uint8_t *pBuffer1, uint8_t *pBuffer2, uint16_t BufferLength);

	
/******************************************************************************************/
/*																Main Function																						*/	
/******************************************************************************************/	
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
	
/* Configure the system clock to 64 MHz */
SystemClock_Config();
	
/* Configure LED2 */
BSP_LED_Init(LED2);
/* After removing all the errors.... change the led pin to PA0 */

	
SER_Init();

RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN;
RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	
#if (LED_BLUE_PIN > 7)
  LED_BLUE_GPIO->CRH = (LED_BLUE_GPIO->CRH & CONFMASKH(LED_BLUE_PIN)) | GPIOPINCONFH(LED_BLUE_PIN,     GPIOCONF(GPIO_MODE_OUTPUT2MHz, GPIO_CNF_OUTPUT_PUSHPULL));
#else
  LED_BLUE_GPIO->CRL = (LED_BLUE_GPIO->CRL & CONFMASKL(LED_BLUE_PIN)) | GPIOPINCONFL(LED_BLUE_PIN,     GPIOCONF(GPIO_MODE_OUTPUT2MHz, GPIO_CNF_OUTPUT_PUSHPULL));
#endif

TIM3->PSC = 38461;	     // Set prescaler to 24 000 (PSC + 1)
TIM3->ARR = 2000;	      // Auto reload value 1000
TIM3->DIER = TIM_DIER_UIE; // Enable update interrupt (timer level)
TIM3->CR1 = TIM_CR1_CEN;   // Enable timer

NVIC_EnableIRQ(TIM3_IRQn); // Enable interrupt from TIM3 (NVIC level)

while (1);
}

//=============================================================================
// TIM3 Interrupt Handler
//=============================================================================
void TIM3_IRQHandler(void)
{
if(TIM3->SR & TIM_SR_UIF) // if UIF flag is set
  {
	char i=0;
  TIM3->SR &= ~TIM_SR_UIF; // clear UIF flag
  LED_BLUE_GPIO->ODR ^= (1 << LED_BLUE_PIN); // toggle LED state
	for(i=0;i<20;i++)
	{
		SER_PutChar (g[i]);
	}
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  while(1)
  {
    /* Toogle LED2 for error */
    BSP_LED_Toggle(LED2);
    HAL_Delay(1000);
  }
}


/******************************************************************************/
/*  @brief  System Clock Configuration																				*/
/*  	        The system Clock is configured as follow : 											*/
/*            System Clock source            = PLL (HSI)											*/
/*            SYSCLK(Hz)                     = 64000000												*/
/*            HCLK(Hz)                       = 64000000												*/
/*            AHB Prescaler                  = 1															*/
/*            APB1 Prescaler                 = 2															*/
/*            APB2 Prescaler                 = 1															*/
/*            PLLMUL                         = 16															*/
/*            Flash Latency(WS)              = 2															*/
/*	@param  None																															*/
/* 	@retval None																															*/
/******************************************************************************/

void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef clkinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};
  
  /* Configure PLL ------------------------------------------------------*/
  /* PLL configuration: PLLCLK = (HSI / 2) * PLLMUL = (8 / 2) * 16 = 64 MHz */
  /* PREDIV1 configuration: PREDIV1CLK = PLLCLK / HSEPredivValue = 64 / 1 = 64 MHz */
  /* Enable HSI and activate PLL with HSi_DIV2 as source */
  oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSI;
  oscinitstruct.HSEState        = RCC_HSE_OFF;
  oscinitstruct.LSEState        = RCC_LSE_OFF;
  oscinitstruct.HSIState        = RCC_HSI_ON;
  oscinitstruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  oscinitstruct.HSEPredivValue    = RCC_HSE_PREDIV_DIV1;
  oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
  oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSI_DIV2;
  oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL16;
//  if (HAL_RCC_OscConfig(&oscinitstruct)!= HAL_OK)
 // {
    /* Initialization Error */
   // while(1); 
  //}

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;  
 // if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2)!= HAL_OK)
  //{
    /* Initialization Error */
    //while(1); 
  //}
}



/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval 0  : pBuffer1 identical to pBuffer2
  *         >0 : pBuffer1 differs from pBuffer2
  */
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if((*pBuffer1) != *pBuffer2)
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }

  return 0;
}

/***************End Of Program***************/
