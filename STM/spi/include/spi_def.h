#ifndef __SPI_DEF_H
#define __SPI_DEF_H

#include "dma_def.h"

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





#endif
