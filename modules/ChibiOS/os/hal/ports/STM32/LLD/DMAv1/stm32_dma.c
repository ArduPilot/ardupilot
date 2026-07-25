/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    DMAv1/stm32_dma.c
 * @brief   DMA helper driver code.
 *
 * @addtogroup STM32_DMA
 * @details DMA sharing helper driver. In the STM32 the DMA streams are a
 *          shared resource, this driver allows to allocate and free DMA
 *          streams at runtime in order to allow all the other device
 *          drivers to coordinate the access to the resource.
 * @note    The DMA ISR handlers are all declared into this module because
 *          sharing, the various device drivers can associate a callback to
 *          ISRs when allocating streams.
 * @{
 */

#include "hal.h"

/* The following macro is only defined if some driver requiring DMA services
   has been enabled.*/
#if defined(STM32_DMA_REQUIRED) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @brief   Mask of the DMA1 streams in @p dma_streams_mask.
 */
#define STM32_DMA1_STREAMS_MASK     ((1U << STM32_DMA1_NUM_CHANNELS) - 1U)

/**
 * @brief   Mask of the DMA2 streams in @p dma_streams_mask.
 */
#define STM32_DMA2_STREAMS_MASK     (((1U << STM32_DMA2_NUM_CHANNELS) -     \
                                      1U) << STM32_DMA1_NUM_CHANNELS)

#if STM32_DMA_SUPPORTS_CSELR == TRUE

#if defined(DMA1_CSELR)
#define __DMA1_CSELR                &DMA1_CSELR->CSELR
#else
#define __DMA1_CSELR                &DMA1->CSELR
#endif

#if defined(DMA2_CSELR)
#define __DMA2_CSELR                &DMA2_CSELR->CSELR
#else
#define __DMA2_CSELR                &DMA2->CSELR
#endif

#define DMA1_CH1_VARIANT            __DMA1_CSELR
#define DMA1_CH2_VARIANT            __DMA1_CSELR
#define DMA1_CH3_VARIANT            __DMA1_CSELR
#define DMA1_CH4_VARIANT            __DMA1_CSELR
#define DMA1_CH5_VARIANT            __DMA1_CSELR
#define DMA1_CH6_VARIANT            __DMA1_CSELR
#define DMA1_CH7_VARIANT            __DMA1_CSELR
#define DMA1_CH8_VARIANT            __DMA1_CSELR
#define DMA2_CH1_VARIANT            __DMA2_CSELR
#define DMA2_CH2_VARIANT            __DMA2_CSELR
#define DMA2_CH3_VARIANT            __DMA2_CSELR
#define DMA2_CH4_VARIANT            __DMA2_CSELR
#define DMA2_CH5_VARIANT            __DMA2_CSELR
#define DMA2_CH6_VARIANT            __DMA2_CSELR
#define DMA2_CH7_VARIANT            __DMA2_CSELR
#define DMA2_CH8_VARIANT            __DMA2_CSELR

#elif STM32_DMA_SUPPORTS_DMAMUX == TRUE

#define DMAMUX1_CHANNEL(id)         (DMAMUX1_BASE + ((id) * 4U))

#if !defined(STM32G4XX)
#define DMA1_CH1_VARIANT            ((DMAMUX_Channel_TypeDef *)DMAMUX1_CHANNEL(0))
#define DMA1_CH2_VARIANT            ((DMAMUX_Channel_TypeDef *)DMAMUX1_CHANNEL(1))
#define DMA1_CH3_VARIANT            ((DMAMUX_Channel_TypeDef *)DMAMUX1_CHANNEL(2))
#define DMA1_CH4_VARIANT            ((DMAMUX_Channel_TypeDef *)DMAMUX1_CHANNEL(3))
#define DMA1_CH5_VARIANT            ((DMAMUX_Channel_TypeDef *)DMAMUX1_CHANNEL(4))
#define DMA1_CH6_VARIANT            ((DMAMUX_Channel_TypeDef *)DMAMUX1_CHANNEL(5))
#define DMA1_CH7_VARIANT            ((DMAMUX_Channel_TypeDef *)DMAMUX1_CHANNEL(6))
#define DMA1_CH8_VARIANT            ((DMAMUX_Channel_TypeDef *)DMAMUX1_CHANNEL(7))
#define DMA2_CH1_VARIANT            ((DMAMUX_Channel_TypeDef *)DMAMUX1_CHANNEL(0 + STM32_DMA1_NUM_CHANNELS))
#define DMA2_CH2_VARIANT            ((DMAMUX_Channel_TypeDef *)DMAMUX1_CHANNEL(1 + STM32_DMA1_NUM_CHANNELS))
#define DMA2_CH3_VARIANT            ((DMAMUX_Channel_TypeDef *)DMAMUX1_CHANNEL(2 + STM32_DMA1_NUM_CHANNELS))
#define DMA2_CH4_VARIANT            ((DMAMUX_Channel_TypeDef *)DMAMUX1_CHANNEL(3 + STM32_DMA1_NUM_CHANNELS))
#define DMA2_CH5_VARIANT            ((DMAMUX_Channel_TypeDef *)DMAMUX1_CHANNEL(4 + STM32_DMA1_NUM_CHANNELS))
#define DMA2_CH6_VARIANT            ((DMAMUX_Channel_TypeDef *)DMAMUX1_CHANNEL(5 + STM32_DMA1_NUM_CHANNELS))
#define DMA2_CH7_VARIANT            ((DMAMUX_Channel_TypeDef *)DMAMUX1_CHANNEL(6 + STM32_DMA1_NUM_CHANNELS))
#define DMA2_CH8_VARIANT            ((DMAMUX_Channel_TypeDef *)DMAMUX1_CHANNEL(7 + STM32_DMA1_NUM_CHANNELS))
#else
#define DMA1_CH1_VARIANT            ((DMAMUX_Channel_TypeDef *)DMAMUX1_CHANNEL(0))
#define DMA1_CH2_VARIANT            ((DMAMUX_Channel_TypeDef *)DMAMUX1_CHANNEL(1))
#define DMA1_CH3_VARIANT            ((DMAMUX_Channel_TypeDef *)DMAMUX1_CHANNEL(2))
#define DMA1_CH4_VARIANT            ((DMAMUX_Channel_TypeDef *)DMAMUX1_CHANNEL(3))
#define DMA1_CH5_VARIANT            ((DMAMUX_Channel_TypeDef *)DMAMUX1_CHANNEL(4))
#define DMA1_CH6_VARIANT            ((DMAMUX_Channel_TypeDef *)DMAMUX1_CHANNEL(5))
#define DMA1_CH7_VARIANT            ((DMAMUX_Channel_TypeDef *)DMAMUX1_CHANNEL(6))
#define DMA1_CH8_VARIANT            ((DMAMUX_Channel_TypeDef *)DMAMUX1_CHANNEL(7))
#define DMA2_CH1_VARIANT            ((DMAMUX_Channel_TypeDef *)DMAMUX1_CHANNEL(8))
#define DMA2_CH2_VARIANT            ((DMAMUX_Channel_TypeDef *)DMAMUX1_CHANNEL(9))
#define DMA2_CH3_VARIANT            ((DMAMUX_Channel_TypeDef *)DMAMUX1_CHANNEL(10))
#define DMA2_CH4_VARIANT            ((DMAMUX_Channel_TypeDef *)DMAMUX1_CHANNEL(11))
#define DMA2_CH5_VARIANT            ((DMAMUX_Channel_TypeDef *)DMAMUX1_CHANNEL(12))
#define DMA2_CH6_VARIANT            ((DMAMUX_Channel_TypeDef *)DMAMUX1_CHANNEL(13))
#define DMA2_CH7_VARIANT            ((DMAMUX_Channel_TypeDef *)DMAMUX1_CHANNEL(14))
#define DMA2_CH8_VARIANT            ((DMAMUX_Channel_TypeDef *)DMAMUX1_CHANNEL(15))
#endif

#else /* !(STM32_DMA_SUPPORTS_DMAMUX == TRUE) */

#define DMA1_CH1_VARIANT            0
#define DMA1_CH2_VARIANT            0
#define DMA1_CH3_VARIANT            0
#define DMA1_CH4_VARIANT            0
#define DMA1_CH5_VARIANT            0
#define DMA1_CH6_VARIANT            0
#define DMA1_CH7_VARIANT            0
#define DMA2_CH1_VARIANT            0
#define DMA2_CH2_VARIANT            0
#define DMA2_CH3_VARIANT            0
#define DMA2_CH4_VARIANT            0
#define DMA2_CH5_VARIANT            0
#define DMA2_CH6_VARIANT            0
#define DMA2_CH7_VARIANT            0

#endif /* !(STM32_DMA_SUPPORTS_DMAMUX == TRUE) */

/*
 * Default ISR collision masks.
 */
#if !defined(STM32_DMA1_CH1_CMASK)
#define STM32_DMA1_CH1_CMASK        (1U << 0U)
#endif

#if !defined(STM32_DMA1_CH2_CMASK)
#define STM32_DMA1_CH2_CMASK        (1U << 1U)
#endif

#if !defined(STM32_DMA1_CH3_CMASK)
#define STM32_DMA1_CH3_CMASK        (1U << 2U)
#endif

#if !defined(STM32_DMA1_CH4_CMASK)
#define STM32_DMA1_CH4_CMASK        (1U << 3U)
#endif

#if !defined(STM32_DMA1_CH5_CMASK)
#define STM32_DMA1_CH5_CMASK        (1U << 4U)
#endif

#if !defined(STM32_DMA1_CH6_CMASK)
#define STM32_DMA1_CH6_CMASK        (1U << 5U)
#endif

#if !defined(STM32_DMA1_CH7_CMASK)
#define STM32_DMA1_CH7_CMASK        (1U << 6U)
#endif

#if !defined(STM32_DMA1_CH8_CMASK)
#define STM32_DMA1_CH8_CMASK        (1U << 7U)
#endif

#if !defined(STM32_DMA2_CH1_CMASK)
#define STM32_DMA2_CH1_CMASK        (1U << (STM32_DMA1_NUM_CHANNELS + 0U))
#endif

#if !defined(STM32_DMA2_CH2_CMASK)
#define STM32_DMA2_CH2_CMASK        (1U << (STM32_DMA1_NUM_CHANNELS + 1U))
#endif

#if !defined(STM32_DMA2_CH3_CMASK)
#define STM32_DMA2_CH3_CMASK        (1U << (STM32_DMA1_NUM_CHANNELS + 2U))
#endif

#if !defined(STM32_DMA2_CH4_CMASK)
#define STM32_DMA2_CH4_CMASK        (1U << (STM32_DMA1_NUM_CHANNELS + 3U))
#endif

#if !defined(STM32_DMA2_CH5_CMASK)
#define STM32_DMA2_CH5_CMASK        (1U << (STM32_DMA1_NUM_CHANNELS + 4U))
#endif

#if !defined(STM32_DMA2_CH6_CMASK)
#define STM32_DMA2_CH6_CMASK        (1U << (STM32_DMA1_NUM_CHANNELS + 5U))
#endif

#if !defined(STM32_DMA2_CH7_CMASK)
#define STM32_DMA2_CH7_CMASK        (1U << (STM32_DMA1_NUM_CHANNELS + 6U))
#endif

#if !defined(STM32_DMA2_CH8_CMASK)
#define STM32_DMA2_CH8_CMASK        (1U << (STM32_DMA1_NUM_CHANNELS + 7U))
#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   DMA streams descriptors.
 * @details This table keeps the association between an unique stream
 *          identifier and the involved physical registers.
 * @note    Don't use this array directly, use the appropriate wrapper macros
 *          instead: @p STM32_DMA1_STREAM1, @p STM32_DMA1_STREAM2 etc.
 */
const stm32_dma_stream_t _stm32_dma_streams[STM32_DMA_STREAMS] = {
#if STM32_DMA1_NUM_CHANNELS > 0
  {DMA1, DMA1_Channel1, STM32_DMA1_CH1_CMASK, DMA1_CH1_VARIANT,  0, 0, STM32_DMA1_CH1_NUMBER},
#endif
#if STM32_DMA1_NUM_CHANNELS > 1
  {DMA1, DMA1_Channel2, STM32_DMA1_CH2_CMASK, DMA1_CH2_VARIANT,  4, 1, STM32_DMA1_CH2_NUMBER},
#endif
#if STM32_DMA1_NUM_CHANNELS > 2
  {DMA1, DMA1_Channel3, STM32_DMA1_CH3_CMASK, DMA1_CH3_VARIANT,  8, 2, STM32_DMA1_CH3_NUMBER},
#endif
#if STM32_DMA1_NUM_CHANNELS > 3
  {DMA1, DMA1_Channel4, STM32_DMA1_CH4_CMASK, DMA1_CH4_VARIANT, 12, 3, STM32_DMA1_CH4_NUMBER},
#endif
#if STM32_DMA1_NUM_CHANNELS > 4
  {DMA1, DMA1_Channel5, STM32_DMA1_CH5_CMASK, DMA1_CH5_VARIANT, 16, 4, STM32_DMA1_CH5_NUMBER},
#endif
#if STM32_DMA1_NUM_CHANNELS > 5
  {DMA1, DMA1_Channel6, STM32_DMA1_CH6_CMASK, DMA1_CH6_VARIANT, 20, 5, STM32_DMA1_CH6_NUMBER},
#endif
#if STM32_DMA1_NUM_CHANNELS > 6
  {DMA1, DMA1_Channel7, STM32_DMA1_CH7_CMASK, DMA1_CH7_VARIANT, 24, 6, STM32_DMA1_CH7_NUMBER},
#endif
#if STM32_DMA1_NUM_CHANNELS > 7
  {DMA1, DMA1_Channel8, STM32_DMA1_CH8_CMASK, DMA1_CH8_VARIANT, 28, 7, STM32_DMA1_CH8_NUMBER},
#endif
#if STM32_DMA2_NUM_CHANNELS > 0
  {DMA2, DMA2_Channel1, STM32_DMA2_CH1_CMASK, DMA2_CH1_VARIANT,  0, 0 + STM32_DMA1_NUM_CHANNELS, STM32_DMA2_CH1_NUMBER},
#endif
#if STM32_DMA2_NUM_CHANNELS > 1
  {DMA2, DMA2_Channel2, STM32_DMA2_CH2_CMASK, DMA2_CH2_VARIANT,  4, 1 + STM32_DMA1_NUM_CHANNELS, STM32_DMA2_CH2_NUMBER},
#endif
#if STM32_DMA2_NUM_CHANNELS > 2
  {DMA2, DMA2_Channel3, STM32_DMA2_CH3_CMASK, DMA2_CH3_VARIANT,  8, 2 + STM32_DMA1_NUM_CHANNELS, STM32_DMA2_CH3_NUMBER},
#endif
#if STM32_DMA2_NUM_CHANNELS > 3
  {DMA2, DMA2_Channel4, STM32_DMA2_CH4_CMASK, DMA2_CH4_VARIANT, 12, 3 + STM32_DMA1_NUM_CHANNELS, STM32_DMA2_CH4_NUMBER},
#endif
#if STM32_DMA2_NUM_CHANNELS > 4
  {DMA2, DMA2_Channel5, STM32_DMA2_CH5_CMASK, DMA2_CH5_VARIANT, 16, 4 + STM32_DMA1_NUM_CHANNELS, STM32_DMA2_CH5_NUMBER},
#endif
#if STM32_DMA2_NUM_CHANNELS > 5
  {DMA2, DMA2_Channel6, STM32_DMA2_CH6_CMASK, DMA2_CH6_VARIANT, 20, 5 + STM32_DMA1_NUM_CHANNELS, STM32_DMA2_CH6_NUMBER},
#endif
#if STM32_DMA2_NUM_CHANNELS > 6
  {DMA2, DMA2_Channel7, STM32_DMA2_CH7_CMASK, DMA2_CH7_VARIANT, 24, 6 + STM32_DMA1_NUM_CHANNELS, STM32_DMA2_CH7_NUMBER},
#endif
#if STM32_DMA2_NUM_CHANNELS > 7
  {DMA2, DMA2_Channel8, STM32_DMA2_CH8_CMASK, DMA2_CH8_VARIANT, 28, 7 + STM32_DMA1_NUM_CHANNELS, STM32_DMA2_CH8_NUMBER},
#endif
};

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   Global DMA-related data structures.
 */
static struct {
  /**
   * @brief   Mask of the allocated streams.
   */
  uint32_t          allocated_mask;
  /**
   * @brief   Mask of the enabled streams ISRs.
   */
  uint32_t          isr_mask;
  /**
   * @brief   DMA IRQ redirectors.
   */
  struct {
    /**
     * @brief   DMA callback function.
     */
    stm32_dmaisr_t    func;
    /**
     * @brief   DMA callback parameter.
     */
    void              *param;
  } streams[STM32_DMA_STREAMS];
} dma;

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if defined(STM32_DMA1_CH1_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA1 stream 1 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA1_CH1_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(STM32_DMA1_STREAM1);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA1_CH2_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA1 stream 2 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA1_CH2_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(STM32_DMA1_STREAM2);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA1_CH3_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA1 stream 3 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA1_CH3_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(STM32_DMA1_STREAM3);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA1_CH4_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA1 stream 4 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA1_CH4_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(STM32_DMA1_STREAM4);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA1_CH5_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA1 stream 5 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA1_CH5_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(STM32_DMA1_STREAM5);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA1_CH6_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA1 stream 6 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA1_CH6_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(STM32_DMA1_STREAM6);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA1_CH7_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA1 stream 7 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA1_CH7_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(STM32_DMA1_STREAM7);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA1_CH8_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA1 stream 8 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA1_CH8_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(STM32_DMA1_STREAM8);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA2_CH1_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA2 stream 1 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA2_CH1_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(STM32_DMA2_STREAM1);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA2_CH2_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA2 stream 2 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA2_CH2_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(STM32_DMA2_STREAM2);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA2_CH3_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA2 stream 3 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA2_CH3_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(STM32_DMA2_STREAM3);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA2_CH4_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA2 stream 4 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA2_CH4_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(STM32_DMA2_STREAM4);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA2_CH5_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA2 stream 5 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA2_CH5_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(STM32_DMA2_STREAM5);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA2_CH6_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA2 stream 6 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA2_CH6_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(STM32_DMA2_STREAM6);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA2_CH7_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA2 stream 7 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA2_CH7_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(STM32_DMA2_STREAM7);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA2_CH8_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA2 stream 8 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA2_CH8_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dmaServeInterrupt(STM32_DMA2_STREAM8);

  OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   STM32 DMA helper initialization.
 *
 * @init
 */
void dmaInit(void) {
  int i;

  dma.allocated_mask = 0U;
  dma.isr_mask       = 0U;
  for (i = 0; i < STM32_DMA_STREAMS; i++) {
    _stm32_dma_streams[i].channel->CCR = STM32_DMA_CCR_RESET_VALUE;
    dma.streams[i].func = NULL;
  }
  DMA1->IFCR = 0xFFFFFFFFU;
#if STM32_DMA2_NUM_CHANNELS > 0
  DMA2->IFCR = 0xFFFFFFFFU;
#endif
}

/**
 * @brief   Allocates a DMA stream.
 * @details The stream is allocated and, if required, the DMA clock enabled.
 *          The function also enables the IRQ vector associated to the stream
 *          and initializes its priority.
 *
 * @param[in] id        numeric identifiers of a specific stream or:
 *                      - @p STM32_DMA_STREAM_ID_ANY for any stream.
 *                      - @p STM32_DMA_STREAM_ID_ANY_DMA1 for any stream
 *                        on DMA1.
 *                      - @p STM32_DMA_STREAM_ID_ANY_DMA2 for any stream
 *                        on DMA2.
 *                      .
 * @param[in] priority  IRQ priority for the DMA stream
 * @param[in] func      handling function pointer, can be @p NULL
 * @param[in] param     a parameter to be passed to the handling function
 * @return              Pointer to the allocated @p stm32_dma_stream_t
 *                      structure.
 * @retval NULL         if a/the stream is not available.
 *
 * @iclass
 */
const stm32_dma_stream_t *dmaStreamAllocI(uint32_t id,
                                          uint32_t priority,
                                          stm32_dmaisr_t func,
                                          void *param) {
  uint32_t i, startid, endid;

  osalDbgCheckClassI();

  if (id < STM32_DMA_STREAMS) {
    startid = id;
    endid   = id;
  }
#if STM32_DMA_SUPPORTS_DMAMUX == TRUE
  else if (id == STM32_DMA_STREAM_ID_ANY) {
    startid = 0U;
    endid   = STM32_DMA_STREAMS - 1U;
  }
  else if (id == STM32_DMA_STREAM_ID_ANY_DMA1) {
    startid = 0U;
    endid   = STM32_DMA1_NUM_CHANNELS - 1U;
  }
#if STM32_DMA2_NUM_CHANNELS > 0
  else if (id == STM32_DMA_STREAM_ID_ANY_DMA2) {
    startid = STM32_DMA1_NUM_CHANNELS;
    endid   = STM32_DMA_STREAMS - 1U;
  }
#endif
#endif
  else {
    osalDbgCheck(false);
    return NULL;
  }

  for (i = startid; i <= endid; i++) {
    uint32_t mask = (1U << i);
    if ((dma.allocated_mask & mask) == 0U) {
      const stm32_dma_stream_t *dmastp = STM32_DMA_STREAM(i);

      /* Installs the DMA handler.*/
      dma.streams[i].func  = func;
      dma.streams[i].param = param;
      dma.allocated_mask  |= mask;

      /* Enabling DMA clocks required by the current streams set.*/
      if ((STM32_DMA1_STREAMS_MASK & mask) != 0U) {
        rccEnableDMA1(true);
      }
#if STM32_DMA2_NUM_CHANNELS > 0
      if ((STM32_DMA2_STREAMS_MASK & mask) != 0U) {
        rccEnableDMA2(true);
      }
#endif

#if (STM32_DMA_SUPPORTS_DMAMUX == TRUE) && defined(rccEnableDMAMUX)
      /* Enabling DMAMUX if present.*/
      if (dma.allocated_mask != 0U) {
        rccEnableDMAMUX(true);
      }
#endif

      /* Enables the associated IRQ vector if not already enabled and if a
         callback is defined.*/
      if (func != NULL) {
        if ((dma.isr_mask & dmastp->cmask) == 0U) {
          nvicEnableVector(dmastp->vector, priority);
        }
        dma.isr_mask |= mask;
      }

      /* Putting the stream in a known state.*/
      dmaStreamDisable(dmastp);
      dmastp->channel->CCR = STM32_DMA_CCR_RESET_VALUE;

      return dmastp;
    }
  }

  return NULL;
}

/**
 * @brief   Allocates a DMA stream.
 * @details The stream is allocated and, if required, the DMA clock enabled.
 *          The function also enables the IRQ vector associated to the stream
 *          and initializes its priority.
 *
 * @param[in] id        numeric identifiers of a specific stream or:
 *                      - @p STM32_DMA_STREAM_ID_ANY for any stream.
 *                      - @p STM32_DMA_STREAM_ID_ANY_DMA1 for any stream
 *                        on DMA1.
 *                      - @p STM32_DMA_STREAM_ID_ANY_DMA2 for any stream
 *                        on DMA2.
 *                      .
 * @param[in] priority  IRQ priority for the DMA stream
 * @param[in] func      handling function pointer, can be @p NULL
 * @param[in] param     a parameter to be passed to the handling function
 * @return              Pointer to the allocated @p stm32_dma_stream_t
 *                      structure.
 * @retval NULL         if a/the stream is not available.
 *
 * @api
 */
const stm32_dma_stream_t *dmaStreamAlloc(uint32_t id,
                                         uint32_t priority,
                                         stm32_dmaisr_t func,
                                         void *param) {
  const stm32_dma_stream_t *dmastp;

  osalSysLock();
  dmastp = dmaStreamAllocI(id, priority, func, param);
  osalSysUnlock();

  return dmastp;
}

/**
 * @brief   Releases a DMA stream.
 * @details The stream is freed and, if required, the DMA clock disabled.
 *          Trying to release a unallocated stream is an illegal operation
 *          and is trapped if assertions are enabled.
 *
 * @param[in] dmastp    pointer to a stm32_dma_stream_t structure
 *
 * @iclass
 */
void dmaStreamFreeI(const stm32_dma_stream_t *dmastp) {
  uint32_t selfindex = (uint32_t)dmastp->selfindex;

  osalDbgCheck(dmastp != NULL);

  /* Check if the streams is not taken.*/
  osalDbgAssert((dma.allocated_mask & (1 << selfindex)) != 0U,
                "not allocated");

  /* Marks the stream as not allocated.*/
  dma.allocated_mask &= ~(1U << selfindex);
  dma.isr_mask &= ~(1U << selfindex);

  /* Disables the associated IRQ vector if it is no more in use.*/
  if ((dma.isr_mask & dmastp->cmask) == 0U) {
    nvicDisableVector(dmastp->vector);
  }

  /* Removes the DMA handler.*/
  dma.streams[selfindex].func  = NULL;
  dma.streams[selfindex].param = NULL;

  /* Shutting down clocks that are no more required, if any.*/
  if ((dma.allocated_mask & STM32_DMA1_STREAMS_MASK) == 0U) {
    rccDisableDMA1();
  }
#if STM32_DMA2_NUM_CHANNELS > 0
  if ((dma.allocated_mask & STM32_DMA2_STREAMS_MASK) == 0U) {
    rccDisableDMA2();
  }
#endif

#if (STM32_DMA_SUPPORTS_DMAMUX == TRUE) && defined(rccDisableDMAMUX)
  /* Shutting down DMAMUX if present.*/
  if (dma.allocated_mask == 0U) {
    rccDisableDMAMUX();
  }
#endif
}

/**
 * @brief   Releases a DMA stream.
 * @details The stream is freed and, if required, the DMA clock disabled.
 *          Trying to release a unallocated stream is an illegal operation
 *          and is trapped if assertions are enabled.
 *
 * @param[in] dmastp    pointer to a stm32_dma_stream_t structure
 *
 * @api
 */
void dmaStreamFree(const stm32_dma_stream_t *dmastp) {

  osalSysLock();
  dmaStreamFreeI(dmastp);
  osalSysUnlock();
}

/**
 * @brief   Serves a DMA IRQ.
 *
 * @param[in] dmastp    pointer to a stm32_dma_stream_t structure
 *
 * @special
 */
void dmaServeInterrupt(const stm32_dma_stream_t *dmastp) {
  uint32_t flags;
  uint32_t selfindex = (uint32_t)dmastp->selfindex;

  flags = (dmastp->dma->ISR >> dmastp->shift) & STM32_DMA_ISR_MASK;
  if (flags & dmastp->channel->CCR) {
    dmastp->dma->IFCR = flags << dmastp->shift;
    if (dma.streams[selfindex].func) {
      dma.streams[selfindex].func(dma.streams[selfindex].param, flags);
    }
  }
}

#if (STM32_DMA_SUPPORTS_DMAMUX == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Associates a peripheral request to a DMA stream.
 * @note    This function can be invoked in both ISR or thread context.
 *
 * @param[in] dmastp    pointer to a @p stm32_dma_stream_t structure
 * @param[in] per       peripheral identifier
 *
 * @special
 */
void dmaSetRequestSource(const stm32_dma_stream_t *dmastp, uint32_t per) {

  osalDbgCheck(per < 256U);

  dmastp->mux->CCR = per;
}
#endif

#endif /* STM32_DMA_REQUIRED */

/** @} */
