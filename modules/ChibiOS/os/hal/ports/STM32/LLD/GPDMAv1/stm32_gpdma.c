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
 * @file    GPDMAv1/stm32_gpdma.c
 * @brief   GPDMA helper driver code.
 *
 * @addtogroup STM32_GPDMA
 * @details GPDMA sharing helper driver. In the STM32 the GPDMA channels are a
 *          shared resource, this driver allows to allocate and free GPDMA
 *          channels at runtime in order to allow all the other device
 *          drivers to coordinate the access to the resource.
 * @note    The GPDMA ISR handlers are all declared into this module because
 *          sharing, the various device drivers can associate a callback to
 *          ISRs when allocating channels.
 * @{
 */

#include "hal.h"

/* The following macro is only defined if some driver requiring GPDMA services
   has been enabled.*/
#if defined(STM32_GPDMA_REQUIRED) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   GPDMA channels descriptors.
 * @details This table keeps the association between an unique channel
 *          identifier and the involved physical registers.
 * @note    Don't use this array directly, use the appropriate wrapper macros
 *          instead: @p STM32_GPDMA1_CHANNEL1, @p STM32_GPDMA1_CHANNEL2 etc.
 */
const stm32_gpdma_channel_t __stm32_gpdma_channels[STM32_GPDMA_NUM_CHANNELS] = {
#if STM32_GPDMA1_NUM_CHANNELS > 0
  {GPDMA1_Channel0, STM32_GPDMA1_CH0_NUMBER},
#endif
#if STM32_GPDMA1_NUM_CHANNELS > 1
  {GPDMA1_Channel1, STM32_GPDMA1_CH1_NUMBER},
#endif
#if STM32_GPDMA1_NUM_CHANNELS > 2
  {GPDMA1_Channel2, STM32_GPDMA1_CH2_NUMBER},
#endif
#if STM32_GPDMA1_NUM_CHANNELS > 3
  {GPDMA1_Channel3, STM32_GPDMA1_CH3_NUMBER},
#endif
#if STM32_GPDMA1_NUM_CHANNELS > 4
  {GPDMA1_Channel4, STM32_GPDMA1_CH4_NUMBER},
#endif
#if STM32_GPDMA1_NUM_CHANNELS > 5
  {GPDMA1_Channel5, STM32_GPDMA1_CH5_NUMBER},
#endif
#if STM32_GPDMA1_NUM_CHANNELS > 6
  {GPDMA1_Channel6, STM32_GPDMA1_CH6_NUMBER},
#endif
#if STM32_GPDMA1_NUM_CHANNELS > 7
  {GPDMA1_Channel7, STM32_GPDMA1_CH7_NUMBER},
#endif
#if STM32_GPDMA2_NUM_CHANNELS > 0
  {GPDMA2_Channel0, STM32_GPDMA2_CH0_NUMBER},
#endif
#if STM32_GPDMA2_NUM_CHANNELS > 1
  {GPDMA2_Channel1, STM32_GPDMA2_CH1_NUMBER},
#endif
#if STM32_GPDMA2_NUM_CHANNELS > 2
  {GPDMA2_Channel2, STM32_GPDMA2_CH2_NUMBER},
#endif
#if STM32_GPDMA2_NUM_CHANNELS > 3
  {GPDMA2_Channel3, STM32_GPDMA2_CH3_NUMBER},
#endif
#if STM32_GPDMA2_NUM_CHANNELS > 4
  {GPDMA2_Channel4, STM32_GPDMA2_CH4_NUMBER},
#endif
#if STM32_GPDMA2_NUM_CHANNELS > 5
  {GPDMA2_Channel5, STM32_GPDMA2_CH5_NUMBER},
#endif
#if STM32_GPDMA2_NUM_CHANNELS > 6
  {GPDMA2_Channel6, STM32_GPDMA2_CH6_NUMBER},
#endif
#if STM32_GPDMA2_NUM_CHANNELS > 7
  {GPDMA2_Channel7, STM32_GPDMA2_CH7_NUMBER},
#endif
};

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   Global GPDMA-related data structures.
 */
static struct {
  /**
   * @brief   Mask of the allocated channels.
   */
  uint32_t              allocated_mask;
  /**
   * @brief   GPDMA IRQ redirectors.
   */
  struct {
    /**
     * @brief   GPDMA callback function.
     */
    stm32_gpdmaisr_t    func;
    /**
     * @brief   GPDMA callback parameter.
     */
    void                *param;
  } channels[STM32_GPDMA_NUM_CHANNELS];
} gpdma;

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if defined(STM32_GPDMA1_CH0_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   GPDMA1 channel 0 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_GPDMA1_CH0_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  gpdmaServeInterrupt(STM32_GPDMA1_CHANNEL0);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_GPDMA1_CH1_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   GPDMA1 channel 1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_GPDMA1_CH1_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  gpdmaServeInterrupt(STM32_GPDMA1_CHANNEL1);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_GPDMA1_CH2_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   GPDMA1 channel 2 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_GPDMA1_CH2_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  gpdmaServeInterrupt(STM32_GPDMA1_CHANNEL2);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_GPDMA1_CH3_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   GPDMA1 channel 3 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_GPDMA1_CH3_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  gpdmaServeInterrupt(STM32_GPDMA1_CHANNEL3);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_GPDMA1_CH4_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   GPDMA1 channel 4 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_GPDMA1_CH4_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  gpdmaServeInterrupt(STM32_GPDMA1_CHANNEL4);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_GPDMA1_CH5_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   GPDMA1 channel 5 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_GPDMA1_CH5_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  gpdmaServeInterrupt(STM32_GPDMA1_CHANNEL5);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_GPDMA1_CH6_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   GPDMA1 channel 6 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_GPDMA1_CH6_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  gpdmaServeInterrupt(STM32_GPDMA1_CHANNEL6);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_GPDMA1_CH7_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   GPDMA1 channel 7 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_GPDMA1_CH7_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  gpdmaServeInterrupt(STM32_GPDMA1_CHANNEL7);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_GPDMA2_CH0_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   GPDMA2 channel 0 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_GPDMA2_CH0_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  gpdmaServeInterrupt(STM32_GPDMA2_CHANNEL0);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_GPDMA2_CH1_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   GPDMA1 channel 2 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_GPDMA2_CH1_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  gpdmaServeInterrupt(STM32_GPDMA2_CHANNEL1);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_GPDMA2_CH2_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   GPDMA2 channel 2 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_GPDMA2_CH2_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  gpdmaServeInterrupt(STM32_GPDMA2_CHANNEL2);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_GPDMA2_CH3_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   GPDMA2 channel 3 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_GPDMA2_CH3_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  gpdmaServeInterrupt(STM32_GPDMA2_CHANNEL3);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_GPDMA2_CH4_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   GPDMA2 channel 4 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_GPDMA2_CH4_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  gpdmaServeInterrupt(STM32_GPDMA2_CHANNEL4);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_GPDMA2_CH5_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   GPDMA2 channel 5 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_GPDMA2_CH5_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  gpdmaServeInterrupt(STM32_GPDMA2_CHANNEL5);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_GPDMA2_CH6_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   GPDMA2 channel 6 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_GPDMA2_CH6_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  gpdmaServeInterrupt(STM32_GPDMA2_CHANNEL6);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_GPDMA2_CH7_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   GPDMA2 channel 7 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_GPDMA2_CH7_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  gpdmaServeInterrupt(STM32_GPDMA2_CHANNEL7);

  OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   STM32 GPDMA helper initialization.
 *
 * @init
 */
void gpdmaInit(void) {
  unsigned i;

  gpdma.allocated_mask = 0U;
  for (i = 0; i < STM32_GPDMA_NUM_CHANNELS; i++) {
    __stm32_gpdma_channels[i].channel->CCR = 0U;
    gpdma.channels[i].func = NULL;
  }
#if STM32_GPDMA2_NUM_CHANNELS > 0
#endif
}

/**
 * @brief   Allocates a GPDMA channel.
 * @details The channel is allocated and, if required, the GPDMA clock enabled.
 *          The function also enables the IRQ vector associated to the channel
 *          and initializes its priority.
 *
 * @param[in] cmask     channels mask where to search for an available channel
 * @param[in] irqprio   IRQ priority for the GPDMA channel
 * @param[in] func      handling function pointer, can be @p NULL
 * @param[in] param     a parameter to be passed to the handling function
 * @return              Pointer to the allocated @p stm32_dma_channel_t
 *                      structure.
 * @retval NULL         if a/the channel is not available.
 *
 * @iclass
 */
const stm32_gpdma_channel_t *gpdmaChannelAllocI(uint32_t cmask,
                                                uint32_t irqprio,
                                                stm32_gpdmaisr_t func,
                                                void *param) {
  unsigned i;
  uint32_t available;

  osalDbgCheckClassI();

  /* Mask of the available channels within the specified channels.*/
  available = ~gpdma.allocated_mask & cmask;

  /* Searching for a free channel.*/
  for (i = 0U; i <= STM32_GPDMA_NUM_CHANNELS; i++) {
    uint32_t mask = (uint32_t)(1U << i);
    if ((available & mask) != 0U) {
      /* Channel found.*/
      const stm32_gpdma_channel_t *dmachp = STM32_GPDMA_CHANNEL(i);
      DMA_Channel_TypeDef *chp = dmachp->channel;

      /* Installs the GPDMA handler.*/
      gpdma.channels[i].func  = func;
      gpdma.channels[i].param = param;
      gpdma.allocated_mask  |= mask;

      /* Enabling GPDMA clocks required by the current channels set.*/
      if ((STM32_GPDMA1_MASK_ANY & mask) != 0U) {
        rccEnableGPDMA1(true);
      }
#if STM32_GPDMA2_NUM_CHANNELS > 0
      if ((STM32_GPDMA2_MASK_ANY & mask) != 0U) {
        rccEnableGPDMA2(true);
      }
#endif

      /* Enables the associated IRQ vector if not already enabled and if a
         callback is defined.*/
      if (func != NULL) {
        /* Could be already enabled but no problem.*/
        nvicEnableVector(dmachp->vector, irqprio);
      }

      /* Putting the channel in a known state.*/
      chp->CLBAR = (uint32_t)&__gpdma_base__;
      chp->CFCR  = STM32_GPDMA_CFCR_ALL_FLAGS;
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
 * @brief   Allocates a GPDMA channel.
 * @details The channel is allocated and, if required, the GPDMA clock enabled.
 *          The function also enables the IRQ vector associated to the channel
 *          and initializes its priority.
 *
 * @param[in] cmask     channels mask where to search for an available channel
 * @param[in] irqprio   IRQ priority for the GPDMA channel
 * @param[in] func      handling function pointer, can be @p NULL
 * @param[in] param     a parameter to be passed to the handling function
 * @return              Pointer to the allocated @p stm32_dma_channel_t
 *                      structure.
 * @retval NULL         if a/the channel is not available.
 *
 * @api
 */
const stm32_gpdma_channel_t *gpdmaChannelAlloc(uint32_t cmask,
                                               uint32_t irqprio,
                                               stm32_gpdmaisr_t func,
                                               void *param) {
  const stm32_gpdma_channel_t *dmachp;

  osalSysLock();
  dmachp = gpdmaChannelAllocI(cmask, irqprio, func, param);
  osalSysUnlock();

  return dmachp;
}

/**
 * @brief   Releases a GPDMA channel.
 * @details The channel is freed and, if required, the GPDMA clock disabled.
 *          Trying to release a unallocated channel is an illegal operation
 *          and is trapped if assertions are enabled.
 *
 * @param[in] dmachp    pointer to a @p stm32_dma_channel_t structure
 *
 * @iclass
 */
void gpdmaChannelFreeI(const stm32_gpdma_channel_t *dmachp) {
  uint32_t selfindex = (uint32_t)(dmachp - __stm32_gpdma_channels);

  osalDbgCheck(dmachp != NULL);

  /* Check if the channels is not taken.*/
  osalDbgAssert((gpdma.allocated_mask & (1U << selfindex)) != 0U,
                "not allocated");

  /* Putting the channel in a known state.*/
  gpdmaChannelDisable(dmachp);

  /* Marks the channel as not allocated.*/
  gpdma.allocated_mask &= ~(1U << selfindex);

  /* Disables the associated IRQ vector if it is no more in use.*/
  nvicDisableVector(dmachp->vector);

  /* Removes the GPDMA handler.*/
  gpdma.channels[selfindex].func  = NULL;
  gpdma.channels[selfindex].param = NULL;

  /* Shutting down clocks that are no more required, if any.*/
  if ((gpdma.allocated_mask & STM32_GPDMA1_MASK_ANY) == 0U) {
    rccDisableGPDMA1();
  }
#if STM32_GPDMA2_NUM_CHANNELS > 0
  if ((gpdma.allocated_mask & STM32_GPDMA2_MASK_ANY) == 0U) {
    rccDisableGPDMA2();
  }
#endif
}

/**
 * @brief   Releases a GPDMA channel.
 * @details The channel is freed and, if required, the GPDMA clock disabled.
 *          Trying to release a unallocated channel is an illegal operation
 *          and is trapped if assertions are enabled.
 *
 * @param[in] dmachp    pointer to a @p stm32_dma_channel_t structure
 *
 * @api
 */
void gpdmaChannelFree(const stm32_gpdma_channel_t *dmachp) {

  osalSysLock();
  gpdmaChannelFreeI(dmachp);
  osalSysUnlock();
}

/**
 * @brief   GPDMA channel disable.
 * @details The function disables the specified channel and then clears any
 *          pending interrupt.
 * @note    This function can be invoked in both ISR or thread context.
 *
 * @param[in] dmachp    pointer to a @p stm32_gpdma_channel_t structure
 * @return              Remaining data do be transferred.
 *
 * @special
 */
size_t gpdmaChannelDisable(const stm32_gpdma_channel_t *dmachp) {
  size_t n;

  /* Suspending the channel, it needs to be in idle.*/
  gpdmaChannelSuspend(dmachp);
  gpdmaChannelWaitIdle(dmachp);

  /* Getting remaining transfer size.*/
  n = gpdmaChannelGetTransactionSize(dmachp);

  /* Now resetting the channel.*/
  gpdmaChannelReset(dmachp);

  /* Resetting all sources and clearing interrupts.*/
  dmachp->channel->CCR  = 0U;
  dmachp->channel->CFCR = STM32_GPDMA_CFCR_ALL_FLAGS;

  return n;
}

/**
 * @brief   Serves a GPDMA IRQ.
 *
 * @param[in] dmachp    pointer to a @p stm32_gpdma_channel_t structure
 *
 * @special
 */
void gpdmaServeInterrupt(const stm32_gpdma_channel_t *dmachp) {
  uint32_t csr;
  uint32_t selfindex = (uint32_t)(dmachp - __stm32_gpdma_channels);

  csr = dmachp->channel->CSR ;
  dmachp->channel->CFCR = csr;
  if ((csr & dmachp->channel->CCR & STM32_GPDMA_CSR_ALL_FLAGS) != 0U) {
    if (gpdma.channels[selfindex].func) {
      gpdma.channels[selfindex].func(gpdma.channels[selfindex].param, csr);
    }
  }
}

#endif /* defined(STM32_GPDMA_REQUIRED) */

/** @} */
