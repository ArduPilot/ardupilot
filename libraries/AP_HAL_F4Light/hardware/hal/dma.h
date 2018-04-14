/******************************************************************************
 * The MIT License

(c) 2017 night_ghost@ykoctpa.ru
 
based on:

 *
 * Copyright (c) 2010 Michael Hope.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

#ifndef _DMA_H_
#define _DMA_H_

#include "hal_types.h"

#ifdef __cplusplus
extern "C"{
#endif


/*
 * Register maps
 */

/**
 * @brief DMA stream type.
 *
 */
typedef struct dma_stream_t {
    __IO uint32_t CR;           /**< Stream configuration register */
    __IO uint32_t NDTR;         /**< Stream number of data register */
    __IO uint32_t PAR;          /**< Stream peripheral address register */
    __IO uint32_t M0AR;         /**< Stream memory address register 0 */
    __IO uint32_t M1AR;         /**< Stream memory address register 1 */
    __IO uint32_t FCR;           /**< Stream FIFO configuration register */
} dma_stream_t;

/**
 * @brief DMA channels
 *
 * Notes:
 * - This is also the dma_tube type for STM32F1.
 * - Channel 0 is not available on all STM32 series.
 *
 * @see dma_tube
 */
typedef enum dma_channel {
    DMA_CH0 = 0,                /**< Channel 0 */
    DMA_CH1 = 1,                /**< Channel 1 */
    DMA_CH2 = 2,                /**< Channel 2 */
    DMA_CH3 = 3,                /**< Channel 3 */
    DMA_CH4 = 4,                /**< Channel 4 */
    DMA_CH5 = 5,                /**< Channel 5 */
    DMA_CH6 = 6,                /**< Channel 6 */
    DMA_CH7 = 7,                /**< Channel 7 */
} dma_channel;

/**
 * @brief DMA register map type.
 *
 */
typedef struct dma_reg_map {
    __IO uint32_t LISR;           /**< Low interrupt status register */
    __IO uint32_t HISR;           /**< High interrupt status register */
    __IO uint32_t LIFCR;          /**< Low interrupt flag clear register */
    __IO uint32_t HIFCR;          /**< High interrupt flag clear register */
    dma_stream_t  STREAM[8];
} dma_reg_map;


/*
 * Register bit definitions
 */

/* Channel configuration register */

#define DMA_CR_CH0                      (0x0 << 25)
#define DMA_CR_CH1                      (0x1 << 25)
#define DMA_CR_CH2                      (0x2 << 25)
#define DMA_CR_CH3                      (0x3 << 25)
#define DMA_CR_CH4                      (0x4 << 25)
#define DMA_CR_CH5                      (0x5 << 25)
#define DMA_CR_CH6                      (0x6 << 25)
#define DMA_CR_CH7                      (0x7 << 25)
#define DMA_CR_MBURST0                  (0x0 << 23)
#define DMA_CR_MBURST4                  (0x1 << 23)
#define DMA_CR_MBURST8                  (0x2 << 23)
#define DMA_CR_MBURST16                 (0x3 << 23)
#define DMA_CR_PBURST0                  (0x0 << 21)
#define DMA_CR_PBURST4                  (0x1 << 21)
#define DMA_CR_PBURST8                  (0x2 << 21)
#define DMA_CR_PBURST16                 (0x3 << 21)
#define DMA_CR_CT0                      (0x0 << 19)
#define DMA_CR_CT1                      (0x1 << 19)
#define DMA_CR_DBM                      (0x1 << 18)

#define DMA_CR_PL_LOW                   (0x0 << 16)
#define DMA_CR_PL_MEDIUM                (0x1 << 16)
#define DMA_CR_PL_HIGH                  (0x2 << 16)
#define DMA_CR_PL_VERY_HIGH             (0x3 << 16)
#define DMA_CR_PL_MASK                  (0x3 << 16)

#define DMA_CR_PINCOS                   (0x1 << 15)

#define DMA_CR_MSIZE_8BITS              (0x0 << 13)
#define DMA_CR_MSIZE_16BITS             (0x1 << 13)
#define DMA_CR_MSIZE_32BITS             (0x2 << 13)

#define DMA_CR_PSIZE_8BITS              (0x0 << 11)
#define DMA_CR_PSIZE_16BITS             (0x1 << 11)
#define DMA_CR_PSIZE_32BITS             (0x2 << 11)

#define DMA_CR_MINC                     (0x1 << 10)
#define DMA_CR_PINC                     (0x1 << 9)
#define DMA_CR_CIRC                     (0x1 << 8)
#define DMA_CR_DIR_P2M                  (0x0 << 6)
#define DMA_CR_DIR_M2P                  (0x1 << 6)
#define DMA_CR_DIR_M2M                  (0x2 << 6)

#define DMA_CR_PFCTRL                   (0x1 << 5)
#define DMA_CR_TCIE                     (0x1 << 4)
#define DMA_CR_HTIE                     (0x1 << 3)
#define DMA_CR_TEIE                     (0x1 << 2)
#define DMA_CR_DMEIE                    (0x1 << 1)
#define DMA_CR_EN                       (0x1)

#define DMA_FLAG_FEIF                    ((uint32_t)0x01)
#define DMA_FLAG_DMEIF                   ((uint32_t)0x04)
#define DMA_FLAG_TEIF                    ((uint32_t)0x08)
#define DMA_FLAG_HTIF                    ((uint32_t)0x10)
#define DMA_FLAG_TCIF                    ((uint32_t)0x20)

#define DMA_FIFOMode_Disable              ((uint32_t)0x00000000) 
#define DMA_FIFOMode_Enable               ((uint32_t)0x00000004)

#define DMA_FIFOThreshold_1QuarterFull    ((uint32_t)0x00000000)
#define DMA_FIFOThreshold_HalfFull        ((uint32_t)0x00000001) 
#define DMA_FIFOThreshold_3QuartersFull   ((uint32_t)0x00000002)
#define DMA_FIFOThreshold_Full            ((uint32_t)0x00000003)

#define DMA_Priority_Low                  ((uint32_t)0x00000000)
#define DMA_Priority_Medium               ((uint32_t)0x00010000) 
#define DMA_Priority_High                 ((uint32_t)0x00020000)
#define DMA_Priority_VeryHigh             ((uint32_t)0x00030000)

/** DMA channels 
    переписано по образу и подобию либы от СТ, позволяющей не возиться с выяснением какой поток на каком ДМА
*/
typedef enum Dma_stream {
    DMA1_STREAM0 = 0,                /**< Stream 0 */
    DMA1_STREAM1 = 1,                /**< Stream 1 */
    DMA1_STREAM2 = 2,                /**< Stream 2 */
    DMA1_STREAM3 = 3,                /**< Stream 3 */
    DMA1_STREAM4 = 4,                /**< Stream 4 */
    DMA1_STREAM5 = 5,                /**< Stream 5 */
    DMA1_STREAM6 = 6,                /**< Stream 6 */
    DMA1_STREAM7 = 7,                /**< Stream 7 */
    DMA2_STREAM0 = 0x10 + 0,                /**< Stream 0 */
    DMA2_STREAM1 = 0x10 + 1,                /**< Stream 1 */
    DMA2_STREAM2 = 0x10 + 2,                /**< Stream 2 */
    DMA2_STREAM3 = 0x10 + 3,                /**< Stream 3 */
    DMA2_STREAM4 = 0x10 + 4,                /**< Stream 4 */
    DMA2_STREAM5 = 0x10 + 5,                /**< Stream 5 */
    DMA2_STREAM6 = 0x10 + 6,                /**< Stream 6 */
    DMA2_STREAM7 = 0x10 + 7,                /**< Stream 7 */
    NUM_DMA_STREAMS,
} dma_stream;


/*
 * Devices
 */


/** Encapsulates state related to a DMA channel interrupt. */


/** DMA device type */
typedef struct dma_dev {
    dma_reg_map *regs;             /**< Register map */
    uint32_t clk_id;             /**< Clock ID */
    IRQn_Type irq_lines[8];
    Handler  *handlers;     // pointer to RAM array of handlers
} dma_dev;



//extern const dma_dev * const _DMA1;
//extern const dma_dev * const _DMA2;
#define _DMA1 (&dma1);
#define _DMA2 (&dma2);


/*
 * Convenience functions
 */

void dma_init(dma_stream stream);

/** Flags for DMA transfer configuration. */
typedef enum dma_mode_flags {
    DMA_MEM_2_MEM  = 1 << 14, /**< Memory to memory mode */
    DMA_MINC_MODE  = 1 << 7,  /**< Auto-increment memory address */
    DMA_PINC_MODE  = 1 << 6,  /**< Auto-increment peripheral address */
    DMA_CIRC_MODE  = 1 << 5,  /**< Circular mode */
    DMA_FROM_MEM   = 1 << 4,  /**< Read from memory to peripheral */
    DMA_TRNS_ERR   = 1 << 3,  /**< Interrupt on transfer error */
    DMA_HALF_TRNS  = 1 << 2,  /**< Interrupt on half-transfer */
    DMA_TRNS_CMPLT = 1 << 1   /**< Interrupt on transfer completion */
} dma_mode_flags;

/** Source and destination transfer sizes. */
typedef enum dma_xfer_size {
    DMA_SIZE_8BITS  = 0,        /**< 8-bit transfers */
    DMA_SIZE_16BITS = 1,        /**< 16-bit transfers */
    DMA_SIZE_32BITS = 2         /**< 32-bit transfers */
} dma_xfer_size;


    
void dma_setup_transfer(dma_stream    stream,
                          __IO void     *peripheral_address,
                          __IO void     *memory_address0,
                          uint32_t       flags,
                          uint32_t       fifo_flags);

// memory-memory
void dma_setup_transfer_mm(dma_stream    stream,
                              __IO void     *memory_address0,
                              __IO void     *memory_address1,
                              uint32_t       flags,
                              uint32_t       fifo_flags);
    

// copied from ST lib but all flags combined
typedef struct
{
  uint32_t DMA_PeripheralBaseAddr; /*!< Specifies the peripheral base address for DMAy Streamx. */

  uint32_t DMA_Memory0BaseAddr;    /*!< Specifies the memory 0 base address for DMAy Streamx. 
                                        This memory is the default memory used when double buffer mode is
                                        not enabled. */

  uint32_t DMA_BufferSize;         /*!< Specifies the buffer size, in data unit, of the specified Stream. 
                                        The data unit is equal to the configuration set in DMA_PeripheralDataSize
                                        or DMA_MemoryDataSize members depending in the transfer direction. */


  uint32_t DMA_FIFO_flags;          /*!< Specifies if the FIFO mode or Direct mode will be used for the specified Stream, and FIFO threshold level.
                                        This parameter can be a value of @ref DMA_fifo_direct_mode ORed DMA_fifo_threshold_level
                                        @note The Direct mode (FIFO mode disabled) cannot be used if the 
                                               memory-to-memory data transfer is configured on the selected Stream */

  uint32_t DMA_flags;  // specifies all below

#if 0
  uint32_t DMA_Channel;            /*!< Specifies the channel used for the specified stream. 
                                        This parameter can be a value of @ref DMA_channel */


  uint32_t DMA_DIR;                /*!< Specifies if the data will be transferred from memory to peripheral, 
                                        from memory to memory or from peripheral to memory.
                                        This parameter can be a value of @ref DMA_data_transfer_direction */

  uint32_t DMA_PeripheralInc;      /*!< Specifies whether the Peripheral address register should be incremented or not.
                                        This parameter can be a value of @ref DMA_peripheral_incremented_mode */

  uint32_t DMA_MemoryInc;          /*!< Specifies whether the memory address register should be incremented or not.
                                        This parameter can be a value of @ref DMA_memory_incremented_mode */


  uint32_t DMA_PeripheralDataSize; /*!< Specifies the Peripheral data width.
                                        This parameter can be a value of @ref DMA_peripheral_data_size */

  uint32_t DMA_MemoryDataSize;     /*!< Specifies the Memory data width.
                                        This parameter can be a value of @ref DMA_memory_data_size */

  uint32_t DMA_Mode;               /*!< Specifies the operation mode of the DMAy Streamx.
                                        This parameter can be a value of @ref DMA_circular_normal_mode
                                        @note The circular buffer mode cannot be used if the memory-to-memory
                                              data transfer is configured on the selected Stream */

  uint32_t DMA_Priority;           /*!< Specifies the software priority for the DMAy Streamx.
                                        This parameter can be a value of @ref DMA_priority_level */


  uint32_t DMA_MemoryBurst;        /*!< Specifies the Burst transfer configuration for the memory transfers. 
                                        It specifies the amount of data to be transferred in a single non interruptable 
                                        transaction. This parameter can be a value of @ref DMA_memory_burst 
                                        @note The burst mode is possible only if the address Increment mode is enabled. */

  uint32_t DMA_PeripheralBurst;    /*!< Specifies the Burst transfer configuration for the peripheral transfers. 
                                        It specifies the amount of data to be transferred in a single non interruptable 
                                        transaction. This parameter can be a value of @ref DMA_peripheral_burst
                                        @note The burst mode is possible only if the address Increment mode is enabled. */
#endif
} DMA_InitType;


void dma_init_transfer(dma_stream stream, DMA_InitType *);

void dma_set_num_transfers(dma_stream stream, uint16_t num_transfers);

void dma_attach_interrupt(dma_stream stream, Handler handler, uint8_t flag);

void dma_detach_interrupt(dma_stream stream);

void dma_enable(dma_stream stream);

void dma_disable(dma_stream stream);

/**
 * @brief Check if a DMA stream is enabled
 * @param dev DMA device
 * @param stream Stream whose enabled bit to check.
 */
uint8_t dma_is_stream_enabled(dma_stream stream);

/**
 * @brief Get the ISR status bits for a DMA stream.
 *
 * The bits are returned right-aligned, in the following order:
 * transfer error flag, half-transfer flag, transfer complete flag,
 * global interrupt flag.
 *
 * @param dev DMA device
 * @param stream Stream whose ISR bits to return.
 */
uint8_t dma_get_isr_bits(dma_stream stream);

/**
 * @brief Clear the ISR status bits for a given DMA stream.
 *
 * @param dev DMA device
 * @param stream Stream whose ISR bits to clear.
 */
void dma_clear_isr_bits(dma_stream stream);

void DMA1_Stream0_IRQHandler(void);
void DMA1_Stream1_IRQHandler(void);
void DMA1_Stream2_IRQHandler(void);
void DMA1_Stream3_IRQHandler(void);
void DMA1_Stream4_IRQHandler(void);
void DMA1_Stream5_IRQHandler(void);
void DMA1_Stream6_IRQHandler(void);
void DMA1_Stream7_IRQHandler(void);
void DMA2_Stream0_IRQHandler(void);
void DMA2_Stream1_IRQHandler(void);
void DMA2_Stream2_IRQHandler(void);
void DMA2_Stream3_IRQHandler(void);
void DMA2_Stream4_IRQHandler(void);
void DMA2_Stream5_IRQHandler(void);
void DMA2_Stream6_IRQHandler(void);
void DMA2_Stream7_IRQHandler(void);



#ifdef __cplusplus
} // extern "C"
#endif

#endif
