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
 * @file    DMAv1/rp_dma.h
 * @brief   DMA helper driver header.
 *
 * @addtogroup RP_DMA
 * @{
 */

#ifndef RP_DMA_H
#define RP_DMA_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Total number of DMA channels.
 */
#define RP_DMA_CHANNELS                 12U

/**
 * @brief   Checks if a DMA channels id is within the valid range.
 *
 * @param[in] id        DMA channels id
 * @retval              The check result.
 * @retval false        invalid DMA channel.
 * @retval true         correct DMA channel.
 */
#define RP_DMA_IS_VALID_CHANNEL(chn)    (((chn) >= 0U) &&                   \
                                         ((id) <= (RP_DMA_CHANNELS + 1U)))

/**
 * @brief   Checks if a DMA priority is within the valid range.
 * @param[in] prio      DMA priority
 *
 * @retval              The check result.
 * @retval false        invalid DMA priority.
 * @retval true         correct DMA priority.
 */
#define RP_DMA_IS_VALID_PRIORITY(prio) (((prio) >= 0U) && ((prio) <= 1U))

/**
 * @brief   Any channel selector.
 */
#define RP_DMA_CHANNEL_ID_ANY           RP_DMA_CHANNELS

/**
 * @brief   Returns a pointer to a @p rp_dma_channel_t structure.
 *
 * @param[in] id        the stream numeric identifier
 * @return              A pointer to the @p rp_dma_channel_t constant structure
 *                      associated to the DMA channel.
 */
#define RP_DMA_CHANNEL(id)              (&__rp_dma_channels[id])

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a DMA callback.
 *
 * @param[in] p         parameter for the registered function
 * @param[in] ct        content of the CTRL_TRIG register
 */
typedef void (*rp_dmaisr_t)(void *p, uint32_t ct);

/**
 * @brief   RP DMA channel descriptor structure.
 */
typedef struct {
  DMA_TypeDef           *dma;           /**< @brief Associated DMA.         */
  DMA_Channel_Typedef   *channel;       /**< @brief Associated DMA channel. */
  uint32_t              chnidx;         /**< @brief Index to self in array. */
  uint32_t              chnmask;        /**< @brief Channel bit mask.       */
} rp_dma_channel_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(__DOXYGEN__)
extern const rp_dma_channel_t __rp_dma_channels[RP_DMA_CHANNELS];
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void dmaInit(void);
  const rp_dma_channel_t *dmaChannelAllocI(uint32_t id,
                                           uint32_t priority,
                                           rp_dmaisr_t func,
                                           void *param);
  const rp_dma_channel_t *dmaChannelAlloc(uint32_t id,
                                          uint32_t priority,
                                          rp_dmaisr_t func,
                                          void *param);
  void dmaChannelFreeI(const rp_dma_channel_t *dmachp);
  void dmaChannelFree(const rp_dma_channel_t *dmachp);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Driver inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief   Returns the channel busy state.
 *
 * @param[in] dmachp    pointer to a rp_dma_channel_t structure
 * @return              The channel busy state.
 * @retval false        if the channel is not busy.
 * @retval true         if the channel is busy.
 *
 * @special
 */
__STATIC_INLINE bool dmaChannelIsBusyX(const rp_dma_channel_t *dmachp) {

  return (bool)((dmachp->channel->CTRL_TRIG & DMA_CTRL_TRIG_BUSY) != 0U);
}

/**
 * @brief   Get and clears channel interrupts state.
 *
 * @param[in] dmachp    pointer to a rp_dma_channel_t structure
 * @return              The content of @p CTRL_TRIG register before clearing
 *                      interrupts.
 *
 * @special
 */
__STATIC_INLINE uint32_t dmaChannelGetAndClearInterrupts(const rp_dma_channel_t *dmachp) {
  uint32_t ctrl_trig;

  ctrl_trig = dmachp->channel->CTRL_TRIG;
  dmachp->channel->CTRL_TRIG = ctrl_trig |
                               DMA_CTRL_TRIG_READ_ERROR |
                               DMA_CTRL_TRIG_WRITE_ERROR;

  return ctrl_trig;
}

/**
 * @brief   Enables a channel interrupt.
 *
 * @param[in] dmachp    pointer to a rp_dma_channel_t structure
 *
 * @special
 */
__STATIC_INLINE void dmaChannelEnableInterruptX(const rp_dma_channel_t *dmachp) {

  if (SIO->CPUID == 0U) {
    dmachp->dma->SET.INTE0 = dmachp->chnmask;
  }
  else {
    dmachp->dma->SET.INTE1 = dmachp->chnmask;
  }
}

/**
 * @brief   Disables a channel interrupt.
 * @note    The interrupt is disabled for both cores to save a check.
 *
 * @param[in] dmachp    pointer to a rp_dma_channel_t structure
 *
 * @special
 */
__STATIC_INLINE void dmaChannelDisableInterruptX(const rp_dma_channel_t *dmachp) {

  dmachp->dma->CLR.INTE0 = dmachp->chnmask;
  dmachp->dma->CLR.INTE1 = dmachp->chnmask;
}

/**
 * @brief   Setup of the source DMA pointer.
 *
 * @param[in] dmachp    pointer to a rp_dma_channel_t structure
 * @param[in] addr      value to be written in the @p READ_ADDR register
 *
 * @special
 */
__STATIC_INLINE void dmaChannelSetSourceX(const rp_dma_channel_t *dmachp,
                                          uint32_t addr) {

  osalDbgAssert(dmaChannelIsBusyX(dmachp) == false, "channel is busy");

  dmachp->channel->READ_ADDR = addr;
}

/**
 * @brief   Setup of the destination DMA pointer.
 *
 * @param[in] dmachp    pointer to a rp_dma_channel_t structure
 * @param[in] addr      value to be written in the @p WRITE_ADDR register
 *
 * @special
 */
__STATIC_INLINE void dmaChannelSetDestinationX(const rp_dma_channel_t *dmachp,
                                               uint32_t addr) {

  osalDbgAssert(dmaChannelIsBusyX(dmachp) == false, "channel is busy");

  dmachp->channel->WRITE_ADDR = addr;
}

/**
 * @brief   Setup of the DMA transfer counter.
 *
 * @param[in] dmachp    pointer to a rp_dma_channel_t structure
 * @param[in] n         value to be written in the @p TRANS_COUNT register
 *
 * @special
 */
__STATIC_INLINE void dmaChannelSetCounterX(const rp_dma_channel_t *dmachp,
                                           uint32_t n) {

  osalDbgAssert(dmaChannelIsBusyX(dmachp) == false, "channel is busy");

  dmachp->channel->TRANS_COUNT = n;
}

/**
 * @brief   Setup of the DMA transfer mode without linking.
 * @note    The link field is enforced to "self" meaning no linking.
 *
 * @param[in] dmachp    pointer to a rp_dma_channel_t structure
 * @param[in] mode      value to be written in the @p CTRL_TRIG register
 *                      except link field
 *
 * @special
 */
__STATIC_INLINE void dmaChannelSetModeX(const rp_dma_channel_t *dmachp,
                                        uint32_t mode) {

  osalDbgAssert(dmaChannelIsBusyX(dmachp) == false, "channel is busy");

  dmachp->channel->CTRL_TRIG = (mode & ~DMA_CTRL_TRIG_CHAIN_TO_Msk) |
                               DMA_CTRL_TRIG_CHAIN_TO(dmachp->chnidx);
}

/**
 * @brief   Aborts the current transfer on a DMA channel.
 *
 * @param[in] dmachp    pointer to a rp_dma_channel_t structure
 *
 * @special
 */
__STATIC_INLINE void dmaChannelAbortX(const rp_dma_channel_t *dmachp) {

  dmachp->dma->SET.CHAN_ABORT = dmachp->chnmask;
  while ((dmachp->dma->CHAN_ABORT & dmachp->chnmask) != 0U) {
  }
}

/**
 * @brief   Enables a DMA channel.
 *
 * @param[in] dmachp    pointer to a rp_dma_channel_t structure
 *
 * @special
 */
__STATIC_INLINE void dmaChannelEnableX(const rp_dma_channel_t *dmachp) {

  dmachp->channel->CTRL_TRIG |= DMA_CTRL_TRIG_EN;
}

/**
 * @brief   Suspends a DMA channel.
 *
 * @param[in] dmachp    pointer to a rp_dma_channel_t structure
 *
 * @special
 */
__STATIC_INLINE void dmaChannelSuspendX(const rp_dma_channel_t *dmachp) {

  dmachp->channel->CTRL_TRIG &= ~DMA_CTRL_TRIG_EN;
}

/**
 * @brief   Disables a DMA channel aborting the current transfer.
 *
 * @param[in] dmachp    pointer to a rp_dma_channel_t structure
 *
 * @special
 */
__STATIC_INLINE void dmaChannelDisableX(const rp_dma_channel_t *dmachp) {

  dmaChannelSuspendX(dmachp);
  dmaChannelAbortX(dmachp);
  (void) dmaChannelGetAndClearInterrupts(dmachp);
}

#endif /* RP_DMA_H */

/** @} */
