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
 * @file    BDMAv1/stm32_bdma.c
 * @brief   BDMA helper driver code.
 *
 * @addtogroup STM32_BDMA
 * @details BDMA sharing helper driver. In the STM32 the BDMA streams are a
 *          shared resource, this driver allows to allocate and free BDMA
 *          streams at runtime in order to allow all the other device
 *          drivers to coordinate the access to the resource.
 * @note    The BDMA ISR handlers are all declared into this module because
 *          sharing, the various device drivers can associate a callback to
 *          ISRs when allocating streams.
 * @{
 */

#include "hal.h"

/* The following macro is only defined if some driver requiring BDMA services
   has been enabled.*/
#if defined(STM32_BDMA_REQUIRED) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @brief   Mask of the BDMA streams in @p bdma_allocated_mask.
 */
#define STM32_BDMA_STREAMS_MASK     0x000000FFU

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   BDMA streams descriptors.
 * @details This table keeps the association between an unique stream
 *          identifier and the involved physical registers.
 * @note    Don't use this array directly, use the appropriate wrapper macros
 *          instead: @p STM32_BDMA1_STREAM1, @p STM32_BDMA1_STREAM2 etc.
 */
const stm32_bdma_stream_t _stm32_bdma_streams[STM32_BDMA_STREAMS] = {
  {BDMA, BDMA_Channel0,  0, DMAMUX2_Channel0, 0, STM32_BDMA1_CH0_NUMBER},
  {BDMA, BDMA_Channel1,  4, DMAMUX2_Channel1, 1, STM32_BDMA1_CH1_NUMBER},
  {BDMA, BDMA_Channel2,  8, DMAMUX2_Channel2, 2, STM32_BDMA1_CH2_NUMBER},
  {BDMA, BDMA_Channel3, 12, DMAMUX2_Channel3, 3, STM32_BDMA1_CH3_NUMBER},
  {BDMA, BDMA_Channel4, 16, DMAMUX2_Channel4, 4, STM32_BDMA1_CH4_NUMBER},
  {BDMA, BDMA_Channel5, 20, DMAMUX2_Channel5, 5, STM32_BDMA1_CH5_NUMBER},
  {BDMA, BDMA_Channel6, 24, DMAMUX2_Channel6, 6, STM32_BDMA1_CH6_NUMBER},
  {BDMA, BDMA_Channel7, 28, DMAMUX2_Channel7, 7, STM32_BDMA1_CH7_NUMBER}
};

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   BDMA ISR redirector type.
 */
typedef struct {
  stm32_bdmaisr_t       func;           /**< @brief BDMA callback function. */
  void                  *param;         /**< @brief BDMA callback parameter.*/
} bdma_isr_redir_t;

/**
 * @brief   BDMA driver base structure.
 */
static struct {
  /**
   * @brief   Mask of the allocated streams.
   */
  uint32_t              allocated_mask;
  /**
   * @brief   DMA IRQ redirectors.
   */
  struct {
    /**
     * @brief   DMA callback function.
     */
    stm32_bdmaisr_t    func;
    /**
     * @brief   DMA callback parameter.
     */
    void              *param;
  } streams[STM32_BDMA_STREAMS];
} bdma;

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   BDMA1 stream 0 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_BDMA1_CH0_HANDLER) {
  uint32_t flags;

  OSAL_IRQ_PROLOGUE();

  flags = (BDMA->ISR >> 0U) & STM32_BDMA_ISR_MASK;
  BDMA->IFCR = flags << 0U;
  if (bdma.streams[0].func)
    bdma.streams[0].func(bdma.streams[0].param, flags);

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   BDMA1 stream 1 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_BDMA1_CH1_HANDLER) {
  uint32_t flags;

  OSAL_IRQ_PROLOGUE();

  flags = (BDMA->ISR >> 4U) & STM32_BDMA_ISR_MASK;
  BDMA->IFCR = flags << 4U;
  if (bdma.streams[1].func)
    bdma.streams[1].func(bdma.streams[1].param, flags);

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   BDMA1 stream 2 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_BDMA1_CH2_HANDLER) {
  uint32_t flags;

  OSAL_IRQ_PROLOGUE();

  flags = (BDMA->ISR >> 8U) & STM32_BDMA_ISR_MASK;
  BDMA->IFCR = flags << 8U;
  if (bdma.streams[2].func)
    bdma.streams[2].func(bdma.streams[2].param, flags);

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   BDMA1 stream 3 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_BDMA1_CH3_HANDLER) {
  uint32_t flags;

  OSAL_IRQ_PROLOGUE();

  flags = (BDMA->ISR >> 12U) & STM32_BDMA_ISR_MASK;
  BDMA->IFCR = flags << 12U;
  if (bdma.streams[3].func)
    bdma.streams[3].func(bdma.streams[3].param, flags);

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   BDMA1 stream 4 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_BDMA1_CH4_HANDLER) {
  uint32_t flags;

  OSAL_IRQ_PROLOGUE();

  flags = (BDMA->ISR >> 16U) & STM32_BDMA_ISR_MASK;
  BDMA->IFCR = flags << 16U;
  if (bdma.streams[4].func)
    bdma.streams[4].func(bdma.streams[4].param, flags);

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   BDMA1 stream 5 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_BDMA1_CH5_HANDLER) {
  uint32_t flags;

  OSAL_IRQ_PROLOGUE();

  flags = (BDMA->ISR >> 20U) & STM32_BDMA_ISR_MASK;
  BDMA->IFCR = flags << 20U;
  if (bdma.streams[5].func)
    bdma.streams[5].func(bdma.streams[5].param, flags);

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   BDMA1 stream 6 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_BDMA1_CH6_HANDLER) {
  uint32_t flags;

  OSAL_IRQ_PROLOGUE();

  flags = (BDMA->ISR >> 24U) & STM32_BDMA_ISR_MASK;
  BDMA->IFCR = flags << 24U;
  if (bdma.streams[6].func)
    bdma.streams[6].func(bdma.streams[6].param, flags);

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   BDMA1 stream 7 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_BDMA1_CH7_HANDLER) {
  uint32_t flags;

  OSAL_IRQ_PROLOGUE();

  flags = (BDMA->ISR >> 28U) & STM32_BDMA_ISR_MASK;
  BDMA->IFCR = flags << 28U;
  if (bdma.streams[7].func)
    bdma.streams[7].func(bdma.streams[7].param, flags);

  OSAL_IRQ_EPILOGUE();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   STM32 BDMA helper initialization.
 *
 * @init
 */
void bdmaInit(void) {
  unsigned i;

  bdma.allocated_mask = 0U;
  for (i = 0; i < STM32_BDMA_STREAMS; i++) {
    _stm32_bdma_streams[i].channel->CCR = 0U;
    bdma.streams[i].func  = NULL;
    bdma.streams[i].param = NULL;
  }
  BDMA->IFCR = 0xFFFFFFFFU;
}

/**
 * @brief   Allocates a BDMA stream.
 * @details The stream is allocated and, if required, the BDMA clock enabled.
 *          The function also enables the IRQ vector associated to the stream
 *          and initializes its priority.
 *
 * @param[in] id        numeric identifiers of a specific stream or:
 *                      - @p STM32_BDMA_STREAM_ID_ANY for any stream.
 *                      .
 * @param[in] priority  IRQ priority for the BDMA stream
 * @param[in] func      handling function pointer, can be @p NULL
 * @param[in] param     a parameter to be passed to the handling function
 * @return              Pointer to the allocated @p stm32_bdma_stream_t
 *                      structure.
 * @retval NULL         if a/the stream is not available.
 *
 * @iclass
 */
const stm32_bdma_stream_t *bdmaStreamAllocI(uint32_t id,
                                            uint32_t priority,
                                            stm32_bdmaisr_t func,
                                            void *param) {
  uint32_t i, startid=id, endid=id;

  osalDbgCheckClassI();

  if (id < STM32_BDMA_STREAMS) {
    startid = id;
    endid   = id;
  }
  else if (id == STM32_BDMA_STREAM_ID_ANY) {
    startid = 0U;
    endid   = STM32_BDMA_STREAMS - 1U;
  }
  else {
    osalDbgCheck(false);
    return NULL;
  }

  for (i = startid; i <= endid; i++) {
    uint32_t mask = (1U << i);
    if ((bdma.allocated_mask & mask) == 0U) {
      const stm32_bdma_stream_t *stp = STM32_BDMA_STREAM(i);

      /* Installs the DMA handler.*/
      bdma.streams[i].func  = func;
      bdma.streams[i].param = param;
      bdma.allocated_mask  |= mask;

      /* Enabling DMA clocks required by the current streams set.*/
      if ((STM32_BDMA_STREAMS_MASK & mask) != 0U) {
        rccEnableBDMA1(true);
      }

#if defined(rccEnableDMAMUX)
      /* Enabling DMAMUX if present.*/
      if (bdma.allocated_mask != 0U) {
        rccEnableDMAMUX(true);
      }
#endif

      /* Enables the associated IRQ vector if not already enabled and if a
         callback is defined.*/
      if (func != NULL) {
        nvicEnableVector(stp->vector, priority);
      }

      /* Putting the stream in a known state.*/
      bdmaStreamDisable(stp);
      stp->channel->CCR = STM32_BDMA_CR_RESET_VALUE;

      return stp;
    }
  }

  return NULL;
}

/**
 * @brief   Allocates a BDMA stream.
 * @details The stream is allocated and, if required, the BDMA clock enabled.
 *          The function also enables the IRQ vector associated to the stream
 *          and initializes its priority.
 *
 * @param[in] id        numeric identifiers of a specific stream or:
 *                      - @p STM32_BDMA_STREAM_ID_ANY for any stream.
 *                      .
 * @param[in] priority  IRQ priority for the BDMA stream
 * @param[in] func      handling function pointer, can be @p NULL
 * @param[in] param     a parameter to be passed to the handling function
 * @return              Pointer to the allocated @p stm32_bdma_stream_t
 *                      structure.
 * @retval NULL         if a/the stream is not available.
 *
 * @api
 */
const stm32_bdma_stream_t *bdmaStreamAlloc(uint32_t id,
                                           uint32_t priority,
                                           stm32_bdmaisr_t func,
                                           void *param) {
  const stm32_bdma_stream_t *stp;

  osalSysLock();
  stp = bdmaStreamAllocI(id, priority, func, param);
  osalSysUnlock();

  return stp;
}

/**
 * @brief   Releases a BDMA stream.
 * @details The stream is freed and, if required, the BDMA clock disabled.
 *          Trying to release a unallocated stream is an illegal operation
 *          and is trapped if assertions are enabled.
 *
 * @param[in] stp       pointer to an @p stm32_bdma_stream_t structure
 *
 * @iclass
 */
void bdmaStreamFreeI(const stm32_bdma_stream_t *stp) {

  osalDbgCheck(stp != NULL);

  /* Check if the streams is not taken.*/
  osalDbgAssert((bdma.allocated_mask & (1U << stp->selfindex)) != 0U,
                "not allocated");

  /* Disables the associated IRQ vector.*/
  nvicDisableVector(stp->vector);

  /* Marks the stream as not allocated.*/
  bdma.allocated_mask &= ~(1U << stp->selfindex);

  /* Clearing associated handler and parameter.*/
  bdma.streams[stp->selfindex].func  = NULL;
  bdma.streams[stp->selfindex].param = NULL;

  /* Shutting down clocks that are no more required, if any.*/
  if ((bdma.allocated_mask & STM32_BDMA_STREAMS_MASK) == 0U) {
    rccDisableBDMA1();
  }
}

/**
 * @brief   Releases a BDMA stream.
 * @details The stream is freed and, if required, the BDMA clock disabled.
 *          Trying to release a unallocated stream is an illegal operation
 *          and is trapped if assertions are enabled.
 *
 * @param[in] stp       pointer to an @p stm32_bdma_stream_t structure
 *
 * @api
 */
void bdmaStreamFree(const stm32_bdma_stream_t *stp) {

  osalSysLock();
  bdmaStreamFreeI(stp);
  osalSysUnlock();
}

/**
 * @brief   Associates a peripheral request to a BDMA stream.
 * @note    This function can be invoked in both ISR or thread context.
 *
 * @param[in] stp       pointer to a @p stm32_bdma_stream_t structure
 * @param[in] per       peripheral identifier
 *
 * @special
 */
void bdmaSetRequestSource(const stm32_bdma_stream_t *stp, uint32_t per) {

  osalDbgCheck(per < 256U);

  stp->mux->CCR = per;
}

#endif /* STM32_BDMA_REQUIRED */

/** @} */
