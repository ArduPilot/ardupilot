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
 * @file    MDMAv1/stm32_mdma.c
 * @brief   MDMA helper driver code.
 *
 * @addtogroup STM32_MDMA
 * @details MDMA sharing helper driver. In the STM32 the MDMA channels are a
 *          shared resource, this driver allows to allocate and free MDMA
 *          STM32 at runtime in order to allow all the other device
 *          drivers to coordinate the access to the resource.
 * @note    The MDMA ISR handlers are all declared into this module because
 *          sharing, the various device drivers can associate a callback to
 *          ISRs when allocating channels.
 * @{
 */

#include "hal.h"

/* The following macro is only defined if some driver requiring MDMA services
   has been enabled.*/
#if defined(STM32_MDMA_REQUIRED) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   Global MDMA-related data structures.
 */
static struct {
  /**
   * @brief   Mask of the allocated channels.
   */
  uint32_t              allocated_mask;
  /**
   * @brief   MDMA IRQ redirectors.
   */
  stm32_mdma_channel_t  channels[STM32_MDMA_CHANNELS];
} mdma;

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static void mdma_serve_interrupt(const stm32_mdma_channel_t *mdmachp) {
  uint32_t flags, eflags;

  flags = mdmachp->channel->CISR;
  eflags = mdmachp->channel->CESR;
  mdmachp->channel->CIFCR = flags;
  if (mdmachp->func != NULL) {
    mdmachp->func(mdmachp->param, flags | (eflags << 16));
  }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   MDMA shared interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_MDMA_HANDLER) {
  uint32_t gisr = MDMA->GISR0;
  OSAL_IRQ_PROLOGUE();

  if ((gisr & (1U << 0)) != 0U) {
    mdma_serve_interrupt(&mdma.channels[0]);
  }

  if ((gisr & (1U << 1)) != 0U) {
    mdma_serve_interrupt(&mdma.channels[1]);
  }

  if ((gisr & (1U << 2)) != 0U) {
    mdma_serve_interrupt(&mdma.channels[2]);
  }

  if ((gisr & (1U << 3)) != 0U) {
    mdma_serve_interrupt(&mdma.channels[3]);
  }

  if ((gisr & (1U << 4)) != 0U) {
    mdma_serve_interrupt(&mdma.channels[4]);
  }

  if ((gisr & (1U << 5)) != 0U) {
    mdma_serve_interrupt(&mdma.channels[5]);
  }

  if ((gisr & (1U << 6)) != 0U) {
    mdma_serve_interrupt(&mdma.channels[6]);
  }

  if ((gisr & (1U << 7)) != 0U) {
    mdma_serve_interrupt(&mdma.channels[7]);
  }

  if ((gisr & (1U << 8)) != 0U) {
    mdma_serve_interrupt(&mdma.channels[8]);
  }

  if ((gisr & (1U << 9)) != 0U) {
    mdma_serve_interrupt(&mdma.channels[9]);
  }

  if ((gisr & (1U << 10)) != 0U) {
    mdma_serve_interrupt(&mdma.channels[10]);
  }

  if ((gisr & (1U << 11)) != 0U) {
    mdma_serve_interrupt(&mdma.channels[11]);
  }

  if ((gisr & (1U << 12)) != 0U) {
    mdma_serve_interrupt(&mdma.channels[12]);
  }

  if ((gisr & (1U << 13)) != 0U) {
    mdma_serve_interrupt(&mdma.channels[13]);
  }

  if ((gisr & (1U << 14)) != 0U) {
    mdma_serve_interrupt(&mdma.channels[14]);
  }

  if ((gisr & (1U << 15)) != 0U) {
    mdma_serve_interrupt(&mdma.channels[15]);
  }

  OSAL_IRQ_EPILOGUE();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   STM32 MDMA helper initialization.
 *
 * @init
 */
void mdmaInit(void) {
  static MDMA_Channel_TypeDef * const channels[STM32_MDMA_CHANNELS] = {
    MDMA_Channel0,  MDMA_Channel1,  MDMA_Channel2,  MDMA_Channel3,
    MDMA_Channel4,  MDMA_Channel5,  MDMA_Channel6,  MDMA_Channel7,
    MDMA_Channel8,  MDMA_Channel9,  MDMA_Channel10, MDMA_Channel11,
    MDMA_Channel12, MDMA_Channel13, MDMA_Channel14, MDMA_Channel15
  };
  unsigned i;

  mdma.allocated_mask = 0U;
  for (i = 0U; i < STM32_MDMA_CHANNELS; i++) {
    MDMA_Channel_TypeDef *cp = channels[i];
    mdma.channels[i].channel        = cp;
    mdma.channels[i].func           = NULL;
    mdma.channels[i].channel->CCR   = STM32_MDMA_CCR_RESET_VALUE;
    mdma.channels[i].channel->CTCR  = STM32_MDMA_CTCR_RESET_VALUE;
    mdma.channels[i].channel->CIFCR = STM32_MDMA_CIFCR_CTEIF  |
                                      STM32_MDMA_CIFCR_CBRTIF |
                                      STM32_MDMA_CIFCR_CBRTIF |
                                      STM32_MDMA_CIFCR_CCTCIF |
                                      STM32_MDMA_CIFCR_CTEIF;
  }
}

/**
 * @brief   Allocates an MDMA channel.
 * @details The channel is allocated and, if required, the MDMA clock enabled.
 *          The function also enables the IRQ vector associated to the channel
 *          and initializes its priority.
 *
 * @param[in] id        numeric identifiers of a specific channel or:
 *                      - @p STM32_MDMA_CHANNEL_ID_ANY for any channel.
 *                      .
 * @param[in] func      handling function pointer, can be @p NULL
 * @param[in] param     a parameter to be passed to the handling function
 * @return              Pointer to the allocated @p stm32_mdma_channel_t
 *                      structure.
 * @retval NULL         if a/the channel is not available.
 *
 * @iclass
 */
const stm32_mdma_channel_t *mdmaChannelAllocI(uint32_t id,
                                             stm32_mdmaisr_t func,
                                             void *param) {
  uint32_t i, startid, endid;

  osalDbgCheckClassI();

  if (id < STM32_MDMA_CHANNELS) {
    startid = id;
    endid   = id;
  }
  else if (id == STM32_MDMA_CHANNEL_ID_ANY) {
    startid = 0U;
    endid   = STM32_MDMA_CHANNELS - 1U;
  }
  else {
    osalDbgCheck(false);
    return NULL;
  }

  for (i = startid; i <= endid; i++) {
    uint32_t mask = (1U << i);
    if ((mdma.allocated_mask & mask) == 0U) {
      stm32_mdma_channel_t *mdmachp = &mdma.channels[i];

      /* Installs the MDMA handler.*/
      mdma.allocated_mask |= mask;
      mdmachp->func  = func;
      mdmachp->param = param;

      /* Enabling MDMA clocks required by the current channels set.*/
      if (mdma.allocated_mask != 0U) {
        rccEnableMDMA(true);
      }

      return mdmachp;
    }
  }

  return NULL;
}

/**
 * @brief   Allocates a MDMA channel.
 * @details The channel is allocated and, if required, the MDMA clock enabled.
 *          The function also enables the IRQ vector associated to the channel
 *          and initializes its priority.
 *
 * @param[in] id        numeric identifiers of a specific channel or:
 *                      - @p STM32_MDMA_CHANNEL_ID_ANY for any channel.
 *                      .
 * @param[in] func      handling function pointer, can be @p NULL
 * @param[in] param     a parameter to be passed to the handling function
 * @return              Pointer to the allocated @p stm32_mdma_channel_t
 *                      structure.
 * @retval NULL         if a/the channel is not available.
 *
 * @api
 */
const stm32_mdma_channel_t *mdmaChannelAlloc(uint32_t id,
                                            stm32_mdmaisr_t func,
                                            void *param) {
  const stm32_mdma_channel_t *mdmachp;

  osalSysLock();
  mdmachp = mdmaChannelAllocI(id, func, param);
  osalSysUnlock();

  return mdmachp;
}

/**
 * @brief   Releases a MDMA channel.
 * @details The channel is freed and, if required, the MDMA clock disabled.
 *          Trying to release a unallocated channel is an illegal operation
 *          and is trapped if assertions are enabled.
 *
 * @param[in] mdmachp   pointer to a stm32_mdma_channel_t structure
 *
 * @iclass
 */
void mdmaChannelFreeI(const stm32_mdma_channel_t *mdmachp) {
  uint32_t channel = mdmachp - mdma.channels;
  osalDbgCheck(mdmachp != NULL);

  /* Check if the channels is not taken.*/
  osalDbgAssert((mdma.allocated_mask & (1U << channel)) != 0U,
                "not allocated");

  /* Marks the channel as not allocated.*/
  mdma.allocated_mask &= ~(1U << channel);

  /* Shutting down clocks that are no more required, if any.*/
  if (mdma.allocated_mask == 0U) {
    rccDisableMDMA();
  }
}

/**
 * @brief   Releases a MDMA channel.
 * @details The channel is freed and, if required, the MDMA clock disabled.
 *          Trying to release a unallocated channel is an illegal operation
 *          and is trapped if assertions are enabled.
 *
 * @param[in] mdmachp   pointer to a stm32_mdma_channel_t structure
 *
 * @api
 */
void mdmaChannelFree(const stm32_mdma_channel_t *mdmachp) {

  osalSysLock();
  mdmaChannelFreeI(mdmachp);
  osalSysUnlock();
}

/**
 * @brief   MDMA stream disable.
 * @details The function disables the specified stream, waits for the disable
 *          operation to complete and then clears any pending interrupt.
 * @pre     The stream must have been allocated using @p mdmaChannelAlloc().
 * @post    After use the stream can be released using @p mdmaChannelFree().
 *
 * @param[in] mdmachp   pointer to a stm32_mdma_channel_t structure
 *
 * @xclass
 */
void mdmaChannelDisableX(const stm32_mdma_channel_t *mdmachp) {
  uint32_t ccr = mdmachp->channel->CCR;

  /* Clearing CCR regardless of previous state.*/
  mdmachp->channel->CCR &= ~(STM32_MDMA_CCR_TCIE  | STM32_MDMA_CCR_BTIE  |
                             STM32_MDMA_CCR_BRTIE | STM32_MDMA_CCR_CTCIE |
                             STM32_MDMA_CCR_TEIE  | STM32_MDMA_CCR_EN);

  /* If the channel was enabled then waiting for ongoing operations
     to finish.*/
  if ((ccr & STM32_MDMA_CCR_EN) != 0U) {
    while (((mdmachp)->channel->CISR & STM32_MDMA_CISR_CTCIF) == 0U)
      ;
  }

  /* Clearing IRQ sources.*/
  mdmaChannelClearInterruptX(mdmachp);
}

#endif /* defined(STM32_MDMA_REQUIRED) */

/** @} */
