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
 * @file    DMA3v1/stm32_dma3.c
 * @brief   DMA3 helper driver code.
 *
 * @addtogroup STM32_DMA3
 * @details DMA3 sharing helper driver. In the STM32 the DMA3 channels are a
 *          shared resource, this driver allows to allocate and free DMA3
 *          channels at runtime in order to allow all the other device
 *          drivers to coordinate the access to the resource.
 * @note    The DMA3 ISR handlers are all declared into this module because
 *          sharing, the various device drivers can associate a callback to
 *          ISRs when allocating channels.
 * @{
 */

#include "hal.h"

/* The following macro is only defined if some driver requiring DMA3 services
   has been enabled.*/
#if defined(STM32_DMA3_REQUIRED) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#if defined(GPDMA1_Channel0)
/* DMA3 is named GPDMA in this device.*/
#define STM32_DMA31_CH0                 GPDMA1_Channel0
#define STM32_DMA31_CH1                 GPDMA1_Channel1
#define STM32_DMA31_CH2                 GPDMA1_Channel2
#define STM32_DMA31_CH3                 GPDMA1_Channel3
#define STM32_DMA31_CH4                 GPDMA1_Channel4
#define STM32_DMA31_CH5                 GPDMA1_Channel5
#define STM32_DMA31_CH6                 GPDMA1_Channel6
#define STM32_DMA31_CH7                 GPDMA1_Channel7
#define STM32_DMA32_CH0                 GPDMA2_Channel0
#define STM32_DMA32_CH1                 GPDMA2_Channel1
#define STM32_DMA32_CH2                 GPDMA2_Channel2
#define STM32_DMA32_CH3                 GPDMA2_Channel3
#define STM32_DMA32_CH4                 GPDMA2_Channel4
#define STM32_DMA32_CH5                 GPDMA2_Channel5
#define STM32_DMA32_CH6                 GPDMA2_Channel6
#define STM32_DMA32_CH7                 GPDMA2_Channel7

#else
#error "DMA3 definitions not found or not recognized"
#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   DMA3 channels descriptors.
 * @details This table keeps the association between an unique channel
 *          identifier and the involved physical registers.
 * @note    Don't use this array directly, use the appropriate wrapper macros
 *          instead: @p STM32_DMA31_CHANNEL1, @p STM32_DMA31_CHANNEL2 etc.
 */
const stm32_dma3_channel_t __stm32_dma3_channels[STM32_DMA3_NUM_CHANNELS] = {
#if STM32_DMA31_NUM_CHANNELS > 0
  {STM32_DMA31_CH0, STM32_DMA31_CH0_NUMBER},
#endif
#if STM32_DMA31_NUM_CHANNELS > 1
  {STM32_DMA31_CH1, STM32_DMA31_CH1_NUMBER},
#endif
#if STM32_DMA31_NUM_CHANNELS > 2
  {STM32_DMA31_CH2, STM32_DMA31_CH2_NUMBER},
#endif
#if STM32_DMA31_NUM_CHANNELS > 3
  {STM32_DMA31_CH3, STM32_DMA31_CH3_NUMBER},
#endif
#if STM32_DMA31_NUM_CHANNELS > 4
  {STM32_DMA31_CH4, STM32_DMA31_CH4_NUMBER},
#endif
#if STM32_DMA31_NUM_CHANNELS > 5
  {STM32_DMA31_CH5, STM32_DMA31_CH5_NUMBER},
#endif
#if STM32_DMA31_NUM_CHANNELS > 6
  {STM32_DMA31_CH6, STM32_DMA31_CH6_NUMBER},
#endif
#if STM32_DMA31_NUM_CHANNELS > 7
  {STM32_DMA31_CH7, STM32_DMA31_CH7_NUMBER},
#endif
#if STM32_DMA32_NUM_CHANNELS > 0
  {STM32_DMA32_CH0, STM32_DMA32_CH0_NUMBER},
#endif
#if STM32_DMA32_NUM_CHANNELS > 1
  {STM32_DMA32_CH1, STM32_DMA32_CH1_NUMBER},
#endif
#if STM32_DMA32_NUM_CHANNELS > 2
  {STM32_DMA32_CH2, STM32_DMA32_CH2_NUMBER},
#endif
#if STM32_DMA32_NUM_CHANNELS > 3
  {STM32_DMA32_CH3, STM32_DMA32_CH3_NUMBER},
#endif
#if STM32_DMA32_NUM_CHANNELS > 4
  {STM32_DMA32_CH4, STM32_DMA32_CH4_NUMBER},
#endif
#if STM32_DMA32_NUM_CHANNELS > 5
  {STM32_DMA32_CH5, STM32_DMA32_CH5_NUMBER},
#endif
#if STM32_DMA32_NUM_CHANNELS > 6
  {STM32_DMA32_CH6, STM32_DMA32_CH6_NUMBER},
#endif
#if STM32_DMA32_NUM_CHANNELS > 7
  {STM32_DMA32_CH7, STM32_DMA32_CH7_NUMBER},
#endif
};

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   Global DMA3-related data structures.
 */
static struct {
  /**
   * @brief   Mask of the allocated channels.
   */
  uint32_t              allocated_mask;
  /**
   * @brief   DMA3 IRQ redirectors.
   */
  struct {
    /**
     * @brief   DMA3 callback function.
     */
    stm32_dma3isr_t     func;
    /**
     * @brief   DMA3 callback parameter.
     */
    void                *param;
  } channels[STM32_DMA3_NUM_CHANNELS];
} dma3;

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if defined(STM32_DMA31_CH0_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA31 channel 0 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA31_CH0_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dma3ServeInterrupt(STM32_DMA31_CHANNEL0);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA31_CH1_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA31 channel 1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA31_CH1_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dma3ServeInterrupt(STM32_DMA31_CHANNEL1);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA31_CH2_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA31 channel 2 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA31_CH2_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dma3ServeInterrupt(STM32_DMA31_CHANNEL2);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA31_CH3_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA31 channel 3 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA31_CH3_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dma3ServeInterrupt(STM32_DMA31_CHANNEL3);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA31_CH4_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA31 channel 4 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA31_CH4_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dma3ServeInterrupt(STM32_DMA31_CHANNEL4);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA31_CH5_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA31 channel 5 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA31_CH5_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dma3ServeInterrupt(STM32_DMA31_CHANNEL5);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA31_CH6_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA31 channel 6 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA31_CH6_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dma3ServeInterrupt(STM32_DMA31_CHANNEL6);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA31_CH7_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA31 channel 7 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA31_CH7_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dma3ServeInterrupt(STM32_DMA31_CHANNEL7);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA32_CH0_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA32 channel 0 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA32_CH0_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dma3ServeInterrupt(STM32_DMA32_CHANNEL0);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA32_CH1_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA32 channel 2 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA32_CH1_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dma3ServeInterrupt(STM32_DMA32_CHANNEL1);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA32_CH2_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA32 channel 2 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA32_CH2_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dma3ServeInterrupt(STM32_DMA32_CHANNEL2);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA32_CH3_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA32 channel 3 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA32_CH3_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dma3ServeInterrupt(STM32_DMA32_CHANNEL3);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA32_CH4_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA32 channel 4 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA32_CH4_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dma3ServeInterrupt(STM32_DMA32_CHANNEL4);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA32_CH5_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA32 channel 5 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA32_CH5_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dma3ServeInterrupt(STM32_DMA32_CHANNEL5);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA32_CH6_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA32 channel 6 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA32_CH6_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dma3ServeInterrupt(STM32_DMA32_CHANNEL6);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_DMA32_CH7_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA32 channel 7 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA32_CH7_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  dma3ServeInterrupt(STM32_DMA32_CHANNEL7);

  OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   STM32 DMA3 helper initialization.
 *
 * @init
 */
void dma3Init(void) {
  unsigned i;

  dma3.allocated_mask = 0U;
  for (i = 0; i < STM32_DMA3_NUM_CHANNELS; i++) {
    __stm32_dma3_channels[i].channel->CCR = 0U;
    dma3.channels[i].func = NULL;
  }
#if STM32_DMA32_NUM_CHANNELS > 0
#endif
}

/**
 * @brief   Allocates a DMA3 channel.
 * @details The channel is allocated and, if required, the DMA3 clock enabled.
 *          The function also enables the IRQ vector associated to the channel
 *          and initializes its priority.
 *
 * @param[in] cmask     channels mask where to search for an available channel
 * @param[in] irqprio   IRQ priority for the DMA3 channel
 * @param[in] func      handling function pointer, can be @p NULL
 * @param[in] param     a parameter to be passed to the handling function
 * @return              Pointer to the allocated @p stm32_dma_channel_t
 *                      structure.
 * @retval NULL         if a/the channel is not available.
 *
 * @iclass
 */
const stm32_dma3_channel_t *dma3ChannelAllocI(uint32_t cmask,
                                              uint32_t irqprio,
                                              stm32_dma3isr_t func,
                                              void *param) {
  unsigned i;
  uint32_t available;

  osalDbgCheckClassI();

  /* Mask of the available channels within the specified channels.*/
  available = ~dma3.allocated_mask & cmask;

  /* Searching for a free channel.*/
  for (i = 0U; i <= STM32_DMA3_NUM_CHANNELS; i++) {
    uint32_t mask = (uint32_t)(1U << i);
    if ((available & mask) != 0U) {
      /* Channel found.*/
      const stm32_dma3_channel_t *dmachp = STM32_DMA3_CHANNEL(i);
      DMA_Channel_TypeDef *chp = dmachp->channel;

      /* Installs the DMA3 handler.*/
      dma3.channels[i].func  = func;
      dma3.channels[i].param = param;
      dma3.allocated_mask  |= mask;

      /* Enabling DMA3 clocks required by the current channels set.*/
      if ((STM32_DMA31_MASK_ANY & mask) != 0U) {
        rccEnableDMA31(true);
      }
#if STM32_DMA32_NUM_CHANNELS > 0
      if ((STM32_DMA32_MASK_ANY & mask) != 0U) {
        rccEnableDMA32(true);
      }
#endif

      /* Enables the associated IRQ vector if not already enabled and if a
         callback is defined.*/
      if (func != NULL) {
        /* Could be already enabled but no problem.*/
        nvicEnableVector(dmachp->vector, irqprio);
      }

      /* Putting the channel in a known state.*/
      chp->CLBAR = (uint32_t)&__dma3_base__;
      chp->CFCR  = STM32_DMA3_CFCR_ALL_FLAGS;
      chp->CCR   = 0U;
      chp->CTR1  = 0U;
      chp->CTR2  = 0U;
      chp->CBR1  = 0U;
      chp->CSAR  = 0U;
      chp->CDAR  = 0U;
      chp->CTR3  = 0U;
      chp->CBR2  = 0U;
      chp->CLLR  = 0U;

      return dmachp;
    }
  }

  return NULL;
}

/**
 * @brief   Allocates a DMA3 channel.
 * @details The channel is allocated and, if required, the DMA3 clock enabled.
 *          The function also enables the IRQ vector associated to the channel
 *          and initializes its priority.
 *
 * @param[in] cmask     channels mask where to search for an available channel
 * @param[in] irqprio   IRQ priority for the DMA3 channel
 * @param[in] func      handling function pointer, can be @p NULL
 * @param[in] param     a parameter to be passed to the handling function
 * @return              Pointer to the allocated @p stm32_dma_channel_t
 *                      structure.
 * @retval NULL         if a/the channel is not available.
 *
 * @api
 */
const stm32_dma3_channel_t *dma3ChannelAlloc(uint32_t cmask,
                                             uint32_t irqprio,
                                             stm32_dma3isr_t func,
                                             void *param) {
  const stm32_dma3_channel_t *dmachp;

  osalSysLock();
  dmachp = dma3ChannelAllocI(cmask, irqprio, func, param);
  osalSysUnlock();

  return dmachp;
}

/**
 * @brief   Releases a DMA3 channel.
 * @details The channel is freed and, if required, the DMA3 clock disabled.
 *          Trying to release a unallocated channel is an illegal operation
 *          and is trapped if assertions are enabled.
 *
 * @param[in] dmachp    pointer to a @p stm32_dma_channel_t structure
 *
 * @iclass
 */
void dma3ChannelFreeI(const stm32_dma3_channel_t *dmachp) {
  uint32_t selfindex = (uint32_t)(dmachp - __stm32_dma3_channels);

  osalDbgCheck(dmachp != NULL);

  /* Check if the channels is not taken.*/
  osalDbgAssert((dma3.allocated_mask & (1U << selfindex)) != 0U,
                "not allocated");

  /* Putting the channel in a known state.*/
  dma3ChannelDisable(dmachp);

  /* Marks the channel as not allocated.*/
  dma3.allocated_mask &= ~(1U << selfindex);

  /* Disables the associated IRQ vector if it is no more in use.*/
  nvicDisableVector(dmachp->vector);

  /* Removes the DMA3 handler.*/
  dma3.channels[selfindex].func  = NULL;
  dma3.channels[selfindex].param = NULL;

  /* Shutting down clocks that are no more required, if any.*/
  if ((dma3.allocated_mask & STM32_DMA31_MASK_ANY) == 0U) {
    rccDisableDMA31();
  }
#if STM32_DMA32_NUM_CHANNELS > 0
  if ((dma3.allocated_mask & STM32_DMA32_MASK_ANY) == 0U) {
    rccDisableDMA32();
  }
#endif
}

/**
 * @brief   Releases a DMA3 channel.
 * @details The channel is freed and, if required, the DMA3 clock disabled.
 *          Trying to release a unallocated channel is an illegal operation
 *          and is trapped if assertions are enabled.
 *
 * @param[in] dmachp    pointer to a @p stm32_dma_channel_t structure
 *
 * @api
 */
void dma3ChannelFree(const stm32_dma3_channel_t *dmachp) {

  osalSysLock();
  dma3ChannelFreeI(dmachp);
  osalSysUnlock();
}

/**
 * @brief   DMA3 channel disable.
 * @details The function disables the specified channel and then clears any
 *          pending interrupt.
 * @note    This function can be invoked in both ISR or thread context.
 *
 * @param[in] dmachp    pointer to a @p stm32_dma3_channel_t structure
 * @return              Remaining data do be transferred.
 *
 * @special
 */
size_t dma3ChannelDisable(const stm32_dma3_channel_t *dmachp) {
  size_t n;

  /* Suspending the channel, it needs to be in idle.*/
  dma3ChannelSuspend(dmachp);
  dma3ChannelWaitIdle(dmachp);

  /* Getting remaining transfer size.*/
  n = dma3ChannelGetTransactionSize(dmachp);

  /* Now resetting the channel.*/
  dma3ChannelReset(dmachp);

  /* Resetting all sources and clearing interrupts.*/
  dmachp->channel->CCR  = 0U;
  dmachp->channel->CFCR = STM32_DMA3_CFCR_ALL_FLAGS;

  return n;
}

/**
 * @brief   Serves a DMA3 IRQ.
 *
 * @param[in] dmachp    pointer to a @p stm32_dma3_channel_t structure
 *
 * @special
 */
void dma3ServeInterrupt(const stm32_dma3_channel_t *dmachp) {
  uint32_t csr;
  uint32_t selfindex = (uint32_t)(dmachp - __stm32_dma3_channels);

  csr = dmachp->channel->CSR ;
  dmachp->channel->CFCR = csr;
  if ((csr & dmachp->channel->CCR & STM32_DMA3_CSR_ALL_FLAGS) != 0U) {
    if (dma3.channels[selfindex].func) {
      dma3.channels[selfindex].func(dma3.channels[selfindex].param, csr);
    }
  }
}

#endif /* defined(STM32_DMA3_REQUIRED) */

/** @} */
