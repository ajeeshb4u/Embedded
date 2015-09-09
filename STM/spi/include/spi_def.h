#ifndef __SPI_DEF_H
#define __SPI_DEF_H

#include "dma_def.h"

#ifndef NULL
  #define NULL      0
#endif



/** 
  * @brief  SPI Configuration Structure definition  
  */
typedef struct
{
  uint32_t Mode;               /*!< Specifies the SPI operating mode.
                                    This parameter can be a value of @ref SPI_mode */

  uint32_t Direction;          /*!< Specifies the SPI Directional mode state.
                                    This parameter can be a value of @ref SPI_Direction_mode */

  uint32_t DataSize;           /*!< Specifies the SPI data size.
                                    This parameter can be a value of @ref SPI_data_size */

  uint32_t CLKPolarity;        /*!< Specifies the serial clock steady state.
                                    This parameter can be a value of @ref SPI_Clock_Polarity */

  uint32_t CLKPhase;           /*!< Specifies the clock active edge for the bit capture.
                                    This parameter can be a value of @ref SPI_Clock_Phase */

  uint32_t NSS;                /*!< Specifies whether the NSS signal is managed by
                                    hardware (NSS pin) or by software using the SSI bit.
                                    This parameter can be a value of @ref SPI_Slave_Select_management */

  uint32_t BaudRatePrescaler;  /*!< Specifies the Baud Rate prescaler value which will be
                                    used to configure the transmit and receive SCK clock.
                                    This parameter can be a value of @ref SPI_BaudRate_Prescaler
                                    @note The communication clock is derived from the master
                                    clock. The slave clock does not need to be set */

  uint32_t FirstBit;           /*!< Specifies whether data transfers start from MSB or LSB bit.
                                    This parameter can be a value of @ref SPI_MSB_LSB_transmission */

  uint32_t TIMode;             /*!< Specifies if the TI mode is enabled or not.
                                    This parameter can be a value of @ref SPI_TI_mode */

  uint32_t CRCCalculation;     /*!< Specifies if the CRC calculation is enabled or not.
                                    This parameter can be a value of @ref SPI_CRC_Calculation */

  uint32_t CRCPolynomial;      /*!< Specifies the polynomial used for the CRC calculation.
                                    This parameter must be a number between Min_Data = 0 and Max_Data = 65535 */

}SPI_InitTypeDef;


/** @defgroup SPI_BaudRate_Prescaler SPI BaudRate Prescaler
  * @{
  */
#define SPI_BAUDRATEPRESCALER_2         ((uint32_t)0x00000000)
#define SPI_BAUDRATEPRESCALER_4         ((uint32_t)SPI_CR1_BR_0)
#define SPI_BAUDRATEPRESCALER_8         ((uint32_t)SPI_CR1_BR_1)
#define SPI_BAUDRATEPRESCALER_16        ((uint32_t)SPI_CR1_BR_1 | SPI_CR1_BR_0)
#define SPI_BAUDRATEPRESCALER_32        ((uint32_t)SPI_CR1_BR_2)
#define SPI_BAUDRATEPRESCALER_64        ((uint32_t)SPI_CR1_BR_2 | SPI_CR1_BR_0)
#define SPI_BAUDRATEPRESCALER_128       ((uint32_t)SPI_CR1_BR_2 | SPI_CR1_BR_1)
#define SPI_BAUDRATEPRESCALER_256       ((uint32_t)SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0)

/** @defgroup SPI_Direction_mode SPI Direction mode
  * @{
  */
#define SPI_DIRECTION_2LINES            ((uint32_t)0x00000000)
#define SPI_DIRECTION_2LINES_RXONLY     SPI_CR1_RXONLY
#define SPI_DIRECTION_1LINE             SPI_CR1_BIDIMODE


/** @defgroup SPI_Clock_Phase SPI Clock Phase
  * @{
  */
#define SPI_PHASE_1EDGE                 ((uint32_t)0x00000000)
#define SPI_PHASE_2EDGE                 SPI_CR1_CPHA


/** @defgroup SPI_Clock_Polarity SPI Clock Polarity
  * @{
  */
#define SPI_POLARITY_LOW                ((uint32_t)0x00000000)
#define SPI_POLARITY_HIGH               SPI_CR1_CPOL

/** @defgroup SPI_data_size SPI data size
  * @{
  */
#define SPI_DATASIZE_8BIT               ((uint32_t)0x00000000)
#define SPI_DATASIZE_16BIT              SPI_CR1_DFF

/** @defgroup SPI_MSB_LSB_transmission SPI MSB LSB transmission
  * @{
  */
#define SPI_FIRSTBIT_MSB                ((uint32_t)0x00000000)
#define SPI_FIRSTBIT_LSB                SPI_CR1_LSBFIRST

/** @defgroup SPI_TI_mode SPI TI mode disable
  * @brief  SPI TI Mode not supported for STM32F1xx family 
  * @{
  */
#define SPI_TIMODE_DISABLE             ((uint32_t)0x00000000)

/** @defgroup SPI_CRC_Calculation SPI CRC Calculation
  * @{
  */
#define SPI_CRCCALCULATION_DISABLE      ((uint32_t)0x00000000)
#define SPI_CRCCALCULATION_ENABLE       SPI_CR1_CRCEN


/** @defgroup SPI_Slave_Select_management SPI Slave Select management
  * @{
  */
#define SPI_NSS_SOFT                    SPI_CR1_SSM
#define SPI_NSS_HARD_INPUT              ((uint32_t)0x00000000)
#define SPI_NSS_HARD_OUTPUT             ((uint32_t)(SPI_CR2_SSOE << 16))


/** @defgroup SPI_mode SPI mode
  * @{
  */
#define SPI_MODE_SLAVE                  ((uint32_t)0x00000000)
#define SPI_MODE_MASTER                 (SPI_CR1_MSTR | SPI_CR1_SSI)




/** 
  * @brief  DMA handle Structure definition  
  */ 
typedef struct __DMA_HandleTypeDef
{  
  DMA_Channel_TypeDef   *Instance;                                                    /*!< Register base address                  */
  
  DMA_InitTypeDef       Init;                                                         /*!< DMA communication parameters           */ 
  
  HAL_LockTypeDef       Lock;                                                         /*!< DMA locking object                     */  
  
  HAL_DMA_StateTypeDef  State;                                                        /*!< DMA transfer state                     */
  
  void                  *Parent;                                                      /*!< Parent object state                    */  
  
  void                  (* XferCpltCallback)( struct __DMA_HandleTypeDef * hdma);     /*!< DMA transfer complete callback         */
  
  void                  (* XferHalfCpltCallback)( struct __DMA_HandleTypeDef * hdma); /*!< DMA Half transfer complete callback    */
  
  void                  (* XferErrorCallback)( struct __DMA_HandleTypeDef * hdma);    /*!< DMA transfer error callback            */
  
  __IO uint32_t         ErrorCode;                                                    /*!< DMA Error code                         */
  
} DMA_HandleTypeDef;    

typedef enum
{
  HAL_SPI_STATE_RESET      = 0x00,  /*!< SPI not yet initialized or disabled                */
  HAL_SPI_STATE_READY      = 0x01,  /*!< SPI initialized and ready for use                  */
  HAL_SPI_STATE_BUSY       = 0x02,  /*!< SPI process is ongoing                             */
  HAL_SPI_STATE_BUSY_TX    = 0x12,  /*!< Data Transmission process is ongoing               */
  HAL_SPI_STATE_BUSY_RX    = 0x22,  /*!< Data Reception process is ongoing                  */
  HAL_SPI_STATE_BUSY_TX_RX = 0x32,  /*!< Data Transmission and Reception process is ongoing */
  HAL_SPI_STATE_ERROR      = 0x03   /*!< SPI error state                                    */
    
}HAL_SPI_StateTypeDef;





/** @defgroup SPI_Error_Codes SPI Error Codes
  * @{
  */ 
#define HAL_SPI_ERROR_NONE      ((uint32_t)0x00)    /*!< No error             */
#define HAL_SPI_ERROR_MODF      ((uint32_t)0x01)    /*!< MODF error           */
#define HAL_SPI_ERROR_CRC       ((uint32_t)0x02)    /*!< CRC error            */
#define HAL_SPI_ERROR_OVR       ((uint32_t)0x04)    /*!< OVR error            */
#define HAL_SPI_ERROR_DMA       ((uint32_t)0x08)    /*!< DMA transfer error   */
#define HAL_SPI_ERROR_FLAG      ((uint32_t)0x10)    /*!< Flag: RXNE,TXE, BSY  */
/**
  * @}
  */







#endif
