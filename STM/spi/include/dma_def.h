#ifndef __DMA_DEF_H
#define __DMA_DEF_H


/** 
  * @brief  DMA Configuration Structure definition  
  */
typedef struct
{
  uint32_t Direction;                 /*!< Specifies if the data will be transferred from memory to peripheral, 
                                           from memory to memory or from peripheral to memory.
                                           This parameter can be a value of @ref DMA_Data_transfer_direction */

  uint32_t PeriphInc;                 /*!< Specifies whether the Peripheral address register should be incremented or not.
                                           This parameter can be a value of @ref DMA_Peripheral_incremented_mode */
                               
  uint32_t MemInc;                    /*!< Specifies whether the memory address register should be incremented or not.
                                           This parameter can be a value of @ref DMA_Memory_incremented_mode */
  
  uint32_t PeriphDataAlignment;       /*!< Specifies the Peripheral data width.
                                           This parameter can be a value of @ref DMA_Peripheral_data_size */

  uint32_t MemDataAlignment;          /*!< Specifies the Memory data width.
                                           This parameter can be a value of @ref DMA_Memory_data_size */
                               
  uint32_t Mode;                      /*!< Specifies the operation mode of the DMAy Channelx.
                                           This parameter can be a value of @ref DMA_mode
                                           @note The circular buffer mode cannot be used if the memory-to-memory
                                                 data transfer is configured on the selected Channel */ 

  uint32_t Priority;                   /*!< Specifies the software priority for the DMAy Channelx.
                                            This parameter can be a value of @ref DMA_Priority_level */

} DMA_InitTypeDef;

typedef enum 
{
  HAL_UNLOCKED = 0x00,
  HAL_LOCKED   = 0x01  
} HAL_LockTypeDef;



/**
  * @brief  HAL DMA State structures definition  
  */
typedef enum
{
  HAL_DMA_STATE_RESET             = 0x00,  /*!< DMA not yet initialized or disabled */  
  HAL_DMA_STATE_READY             = 0x01,  /*!< DMA process success and ready for use   */
  HAL_DMA_STATE_READY_HALF        = 0x11,  /*!< DMA Half process success            */
  HAL_DMA_STATE_BUSY              = 0x02,  /*!< DMA process is ongoing              */     
  HAL_DMA_STATE_TIMEOUT           = 0x03,  /*!< DMA timeout state                   */  
  HAL_DMA_STATE_ERROR             = 0x04,  /*!< DMA error state                     */
                                                                        
}HAL_DMA_StateTypeDef;

#endif
