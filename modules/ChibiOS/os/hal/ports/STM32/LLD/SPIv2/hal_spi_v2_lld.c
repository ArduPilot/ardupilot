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
 * @file    SPIv2/hal_spi_v2_lld.c
 * @brief   STM32 SPI (v2) subsystem low level driver source.
 *
 * @addtogroup SPI
 * @{
 */

#include "hal.h"

#if HAL_USE_SPI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define SPI1_RX_DMA_STREAM                                                  \
  STM32_DMA_GETCHANNEL(STM32_SPI_SPI1_RX_DMA_STREAM,                        \
                       STM32_SPI1_RX_DMA_CHN)

#define SPI1_TX_DMA_STREAM                                                  \
  STM32_DMA_GETCHANNEL(STM32_SPI_SPI1_TX_DMA_STREAM,                        \
                       STM32_SPI1_TX_DMA_CHN)

#define SPI2_RX_DMA_STREAM                                                  \
  STM32_DMA_GETCHANNEL(STM32_SPI_SPI2_RX_DMA_STREAM,                        \
                       STM32_SPI2_RX_DMA_CHN)

#define SPI2_TX_DMA_STREAM                                                  \
  STM32_DMA_GETCHANNEL(STM32_SPI_SPI2_TX_DMA_STREAM,                        \
                       STM32_SPI2_TX_DMA_CHN)

#define SPI3_RX_DMA_STREAM                                                  \
  STM32_DMA_GETCHANNEL(STM32_SPI_SPI3_RX_DMA_STREAM,                        \
                       STM32_SPI3_RX_DMA_CHN)

#define SPI3_TX_DMA_STREAM                                                  \
  STM32_DMA_GETCHANNEL(STM32_SPI_SPI3_TX_DMA_STREAM,                        \
                       STM32_SPI3_TX_DMA_CHN)

#define SPI4_RX_DMA_STREAM                                                  \
  STM32_DMA_GETCHANNEL(STM32_SPI_SPI4_RX_DMA_STREAM,                        \
                       STM32_SPI4_RX_DMA_CHN)

#define SPI4_TX_DMA_STREAM                                                  \
  STM32_DMA_GETCHANNEL(STM32_SPI_SPI4_TX_DMA_STREAM,                        \
                       STM32_SPI4_TX_DMA_CHN)

#define SPI5_RX_DMA_STREAM                                                  \
  STM32_DMA_GETCHANNEL(STM32_SPI_SPI5_RX_DMA_STREAM,                        \
                       STM32_SPI5_RX_DMA_CHN)

#define SPI5_TX_DMA_STREAM                                                  \
  STM32_DMA_GETCHANNEL(STM32_SPI_SPI5_TX_DMA_STREAM,                        \
                       STM32_SPI5_TX_DMA_CHN)

#define SPI6_RX_DMA_STREAM                                                  \
  STM32_DMA_GETCHANNEL(STM32_SPI_SPI6_RX_DMA_STREAM,                        \
                       STM32_SPI6_RX_DMA_CHN)

#define SPI6_TX_DMA_STREAM                                                  \
  STM32_DMA_GETCHANNEL(STM32_SPI_SPI6_TX_DMA_STREAM,                        \
                       STM32_SPI6_TX_DMA_CHN)

#if !defined(SPI_SPID1_MEMORY)
#define SPI_SPID1_MEMORY
#endif

#if !defined(SPI_SPID2_MEMORY)
#define SPI_SPID2_MEMORY
#endif

#if !defined(SPI_SPID3_MEMORY)
#define SPI_SPID3_MEMORY
#endif

#if !defined(SPI_SPID4_MEMORY)
#define SPI_SPID4_MEMORY
#endif

#if !defined(SPI_SPID5_MEMORY)
#define SPI_SPID5_MEMORY
#endif

#if !defined(SPI_SPID6_MEMORY)
#define SPI_SPID6_MEMORY
#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief SPI1 driver identifier.*/
#if STM32_SPI_USE_SPI1 || defined(__DOXYGEN__)
SPI_SPID1_MEMORY SPIDriver SPID1;
#endif

/** @brief SPI2 driver identifier.*/
#if STM32_SPI_USE_SPI2 || defined(__DOXYGEN__)
SPI_SPID2_MEMORY SPIDriver SPID2;
#endif

/** @brief SPI3 driver identifier.*/
#if STM32_SPI_USE_SPI3 || defined(__DOXYGEN__)
SPI_SPID3_MEMORY SPIDriver SPID3;
#endif

/** @brief SPI4 driver identifier.*/
#if STM32_SPI_USE_SPI4 || defined(__DOXYGEN__)
SPI_SPID4_MEMORY SPIDriver SPID4;
#endif

/** @brief SPI5 driver identifier.*/
#if STM32_SPI_USE_SPI5 || defined(__DOXYGEN__)
SPI_SPID5_MEMORY SPIDriver SPID5;
#endif

/** @brief SPI6 driver identifier.*/
#if STM32_SPI_USE_SPI6 || defined(__DOXYGEN__)
SPI_SPID6_MEMORY SPIDriver SPID6;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Configures and enables a SPI.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 */
static void spi_lld_enable(SPIDriver *spip) {
  uint32_t cr1, cr2;

  /* SPI setup.*/
  if (spip->config->slave) {
    cr1  = spip->config->cr1 & ~(SPI_CR1_MSTR | SPI_CR1_SPE);
    cr2  = spip->config->cr2 | SPI_CR2_FRXTH | SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;
  }
  else {
    cr1  = (spip->config->cr1 | SPI_CR1_MSTR) & ~SPI_CR1_SPE;
    cr2  = spip->config->cr2 | SPI_CR2_FRXTH | SPI_CR2_SSOE | SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;
  }

  spip->spi->CR1 = cr1;
  spip->spi->CR2 = cr2;
  spip->spi->CR1 = cr1 | SPI_CR1_SPE;
}

/**
 * @brief   Disables a SPI.
 * @note    This is done nicely or by brutally resetting it depending on
 *          the mode and settings.
 * @note    DMAs are also disabled because this is required by the correct
 *          disable procedure.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 */
static void spi_lld_disable(SPIDriver *spip) {

  if (!spip->config->slave) {
    /* Master mode, stopping gracefully.*/

    /* Stopping TX DMA channel.*/
    dmaStreamDisable(spip->dmatx);

    /* Waiting for current frame completion then stop SPI.*/
    while ((spip->spi->SR & SPI_SR_BSY) != 0U) {
    }

    /* Clearing SPE and the rest.*/
    spip->spi->CR1 &= ~SPI_CR1_SPE;

    /* Now it is idle, stopping RX DMA channel.*/
    dmaStreamDisable(spip->dmarx);
  }
  else {
    /* Slave mode, this will not be nice.*/

    /* Stopping DMAs.*/
    dmaStreamDisable(spip->dmatx);
    dmaStreamDisable(spip->dmarx);

    /* Resetting SPI, this will stop it for sure and leave it
       in a clean state.*/
    if (false) {
    }

#if STM32_SPI_USE_SPI1
    else if (&SPID1 == spip) {
      rccResetSPI1();
    }
#endif

#if STM32_SPI_USE_SPI2
    else if (&SPID2 == spip) {
      rccResetSPI2();
    }
#endif

#if STM32_SPI_USE_SPI3
    else if (&SPID3 == spip) {
      rccResetSPI3();
    }
#endif

#if STM32_SPI_USE_SPI4
    else if (&SPID4 == spip) {
      rccResetSPI4();
    }
#endif

#if STM32_SPI_USE_SPI5
    else if (&SPID5 == spip) {
      rccResetSPI5();
    }
#endif

#if STM32_SPI_USE_SPI6
    else if (&SPID6 == spip) {
      rccResetSPI6();
    }
#endif

    else {
      osalDbgAssert(false, "invalid SPI instance");
    }
  }
}

/**
 * @brief   Shared end-of-rx service routine.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] flags     pre-shifted content of the ISR register
 */
static void spi_lld_serve_rx_interrupt(SPIDriver *spip, uint32_t flags) {

  /* DMA errors handling.*/
  if ((flags & (STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF)) != 0) {
#if defined(STM32_SPI_DMA_ERROR_HOOK)
    /* Hook first, if defined.*/
    STM32_SPI_DMA_ERROR_HOOK(spip);
#endif
    /* Stopping DMAs.*/
    dmaStreamDisable(spip->dmatx);
    dmaStreamDisable(spip->dmarx);

    /* Reporting the failure.*/
    __spi_isr_error_code(spip, HAL_RET_HW_FAILURE);
  }
  else if (spip->config->circular) {
    if ((flags & STM32_DMA_ISR_HTIF) != 0U) {
      /* Half buffer interrupt.*/
      __spi_isr_half_code(spip);
    }
    if ((flags & STM32_DMA_ISR_TCIF) != 0U) {
      /* End buffer interrupt.*/
      __spi_isr_full_code(spip);
    }
  }
  else {
    /* Stopping DMAs.*/
    dmaStreamDisable(spip->dmatx);
    dmaStreamDisable(spip->dmarx);

    /* Operation finished interrupt.*/
    __spi_isr_complete_code(spip);
  }
}

/**
 * @brief   Shared end-of-tx service routine.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] flags     pre-shifted content of the ISR register
 */
static void spi_lld_serve_tx_interrupt(SPIDriver *spip, uint32_t flags) {

  /* DMA errors handling.*/
  if ((flags & (STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF)) != 0) {
#if defined(STM32_SPI_DMA_ERROR_HOOK)
    /* Hook first, if defined.*/
    STM32_SPI_DMA_ERROR_HOOK(spip);
#endif

    /* Stopping DMAs.*/
    dmaStreamDisable(spip->dmatx);
    dmaStreamDisable(spip->dmarx);

    /* Reporting the failure.*/
    __spi_isr_error_code(spip, HAL_RET_HW_FAILURE);
  }
}

/**
 * @brief   DMA streams allocation.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] rxstream  stream to be allocated for RX
 * @param[in] txstream  stream to be allocated for TX
 * @param[in] priority  streams IRQ priority
 * @return              The operation status.
 */
static msg_t spi_lld_get_dma(SPIDriver *spip, uint32_t rxstream,
                             uint32_t txstream, uint32_t priority) {

  spip->dmarx = dmaStreamAllocI(rxstream, priority,
                                (stm32_dmaisr_t)spi_lld_serve_rx_interrupt,
                                (void *)spip);
  if (spip->dmarx == NULL) {
    return HAL_RET_NO_RESOURCE;
  }

  spip->dmatx = dmaStreamAllocI(txstream, priority,
                                (stm32_dmaisr_t)spi_lld_serve_tx_interrupt,
                                (void *)spip);
  if (spip->dmatx == NULL) {
    dmaStreamFreeI(spip->dmarx);
    return HAL_RET_NO_RESOURCE;
  }

  return HAL_RET_SUCCESS;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level SPI driver initialization.
 *
 * @notapi
 */
void spi_lld_init(void) {

#if STM32_SPI_USE_SPI1
  spiObjectInit(&SPID1);
  SPID1.spi       = SPI1;
  SPID1.dmarx     = NULL;
  SPID1.dmatx     = NULL;
  SPID1.rxdmamode = STM32_DMA_CR_CHSEL(SPI1_RX_DMA_STREAM) |
                    STM32_DMA_CR_PL(STM32_SPI_SPI1_DMA_PRIORITY) |
                    STM32_DMA_CR_DIR_P2M |
                    STM32_DMA_CR_TCIE |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
  SPID1.txdmamode = STM32_DMA_CR_CHSEL(SPI1_TX_DMA_STREAM) |
                    STM32_DMA_CR_PL(STM32_SPI_SPI1_DMA_PRIORITY) |
                    STM32_DMA_CR_DIR_M2P |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
#endif

#if STM32_SPI_USE_SPI2
  spiObjectInit(&SPID2);
  SPID2.spi       = SPI2;
  SPID2.dmarx     = NULL;
  SPID2.dmatx     = NULL;
  SPID2.rxdmamode = STM32_DMA_CR_CHSEL(SPI2_RX_DMA_STREAM) |
                    STM32_DMA_CR_PL(STM32_SPI_SPI2_DMA_PRIORITY) |
                    STM32_DMA_CR_DIR_P2M |
                    STM32_DMA_CR_TCIE |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
  SPID2.txdmamode = STM32_DMA_CR_CHSEL(SPI2_TX_DMA_STREAM) |
                    STM32_DMA_CR_PL(STM32_SPI_SPI2_DMA_PRIORITY) |
                    STM32_DMA_CR_DIR_M2P |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
#endif

#if STM32_SPI_USE_SPI3
  spiObjectInit(&SPID3);
  SPID3.spi       = SPI3;
  SPID3.dmarx     = NULL;
  SPID3.dmatx     = NULL;
  SPID3.rxdmamode = STM32_DMA_CR_CHSEL(SPI3_RX_DMA_STREAM) |
                    STM32_DMA_CR_PL(STM32_SPI_SPI3_DMA_PRIORITY) |
                    STM32_DMA_CR_DIR_P2M |
                    STM32_DMA_CR_TCIE |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
  SPID3.txdmamode = STM32_DMA_CR_CHSEL(SPI3_TX_DMA_STREAM) |
                    STM32_DMA_CR_PL(STM32_SPI_SPI3_DMA_PRIORITY) |
                    STM32_DMA_CR_DIR_M2P |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
#endif

#if STM32_SPI_USE_SPI4
  spiObjectInit(&SPID4);
  SPID4.spi       = SPI4;
  SPID4.dmarx     = NULL;
  SPID4.dmatx     = NULL;
  SPID4.rxdmamode = STM32_DMA_CR_CHSEL(SPI4_RX_DMA_STREAM) |
                    STM32_DMA_CR_PL(STM32_SPI_SPI4_DMA_PRIORITY) |
                    STM32_DMA_CR_DIR_P2M |
                    STM32_DMA_CR_TCIE |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
  SPID4.txdmamode = STM32_DMA_CR_CHSEL(SPI4_TX_DMA_STREAM) |
                    STM32_DMA_CR_PL(STM32_SPI_SPI4_DMA_PRIORITY) |
                    STM32_DMA_CR_DIR_M2P |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
#endif

#if STM32_SPI_USE_SPI5
  spiObjectInit(&SPID5);
  SPID5.spi       = SPI5;
  SPID5.dmarx     = NULL;
  SPID5.dmatx     = NULL;
  SPID5.rxdmamode = STM32_DMA_CR_CHSEL(SPI5_RX_DMA_STREAM) |
                    STM32_DMA_CR_PL(STM32_SPI_SPI5_DMA_PRIORITY) |
                    STM32_DMA_CR_DIR_P2M |
                    STM32_DMA_CR_TCIE |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
  SPID5.txdmamode = STM32_DMA_CR_CHSEL(SPI5_TX_DMA_STREAM) |
                    STM32_DMA_CR_PL(STM32_SPI_SPI5_DMA_PRIORITY) |
                    STM32_DMA_CR_DIR_M2P |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
#endif

#if STM32_SPI_USE_SPI6
  spiObjectInit(&SPID6);
  SPID6.spi       = SPI6;
  SPID6.dmarx     = NULL;
  SPID6.dmatx     = NULL;
  SPID6.rxdmamode = STM32_DMA_CR_CHSEL(SPI6_RX_DMA_STREAM) |
                    STM32_DMA_CR_PL(STM32_SPI_SPI6_DMA_PRIORITY) |
                    STM32_DMA_CR_DIR_P2M |
                    STM32_DMA_CR_TCIE |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
  SPID6.txdmamode = STM32_DMA_CR_CHSEL(SPI6_TX_DMA_STREAM) |
                    STM32_DMA_CR_PL(STM32_SPI_SPI6_DMA_PRIORITY) |
                    STM32_DMA_CR_DIR_M2P |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
#endif
}

/**
 * @brief   Configures and activates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @return              The operation status.
 *
 * @notapi
 */
msg_t spi_lld_start(SPIDriver *spip) {
  uint32_t ds;
  msg_t msg;

  /* Resetting TX pattern source.*/
  spip->txsource = (uint32_t)STM32_SPI_FILLER_PATTERN;

  /* If in stopped state then enables the SPI and DMA clocks.*/
  if (spip->state == SPI_STOP) {

    /* Enables the peripheral.*/
    if (false) {
    }

#if STM32_SPI_USE_SPI1
    else if (&SPID1 == spip) {
      msg = spi_lld_get_dma(spip,
                            STM32_SPI_SPI1_RX_DMA_STREAM,
                            STM32_SPI_SPI1_TX_DMA_STREAM,
                            STM32_SPI_SPI1_IRQ_PRIORITY);
      if (msg != HAL_RET_SUCCESS) {
        return msg;
      }
      rccEnableSPI1(true);
      rccResetSPI1();
#if STM32_DMA_SUPPORTS_DMAMUX
      dmaSetRequestSource(spip->dmarx, STM32_DMAMUX1_SPI1_RX);
      dmaSetRequestSource(spip->dmatx, STM32_DMAMUX1_SPI1_TX);
#endif
    }
#endif

#if STM32_SPI_USE_SPI2
    else if (&SPID2 == spip) {
      msg = spi_lld_get_dma(spip,
                            STM32_SPI_SPI2_RX_DMA_STREAM,
                            STM32_SPI_SPI2_TX_DMA_STREAM,
                            STM32_SPI_SPI2_IRQ_PRIORITY);
      if (msg != HAL_RET_SUCCESS) {
        return msg;
      }
      rccEnableSPI2(true);
      rccResetSPI2();
#if STM32_DMA_SUPPORTS_DMAMUX
      dmaSetRequestSource(spip->dmarx, STM32_DMAMUX1_SPI2_RX);
      dmaSetRequestSource(spip->dmatx, STM32_DMAMUX1_SPI2_TX);
#endif
    }
#endif

#if STM32_SPI_USE_SPI3
    else if (&SPID3 == spip) {
      msg = spi_lld_get_dma(spip,
                            STM32_SPI_SPI3_RX_DMA_STREAM,
                            STM32_SPI_SPI3_TX_DMA_STREAM,
                            STM32_SPI_SPI3_IRQ_PRIORITY);
      if (msg != HAL_RET_SUCCESS) {
        return msg;
      }
      rccEnableSPI3(true);
      rccResetSPI3();
#if STM32_DMA_SUPPORTS_DMAMUX
      dmaSetRequestSource(spip->dmarx, STM32_DMAMUX1_SPI3_RX);
      dmaSetRequestSource(spip->dmatx, STM32_DMAMUX1_SPI3_TX);
#endif
    }
#endif

#if STM32_SPI_USE_SPI4
    else if (&SPID4 == spip) {
      msg = spi_lld_get_dma(spip,
                            STM32_SPI_SPI4_RX_DMA_STREAM,
                            STM32_SPI_SPI4_TX_DMA_STREAM,
                            STM32_SPI_SPI4_IRQ_PRIORITY);
      if (msg != HAL_RET_SUCCESS) {
        return msg;
      }
      rccEnableSPI4(true);
      rccResetSPI4();
#if STM32_DMA_SUPPORTS_DMAMUX
      dmaSetRequestSource(spip->dmarx, STM32_DMAMUX1_SPI4_RX);
      dmaSetRequestSource(spip->dmatx, STM32_DMAMUX1_SPI4_TX);
#endif
    }
#endif

#if STM32_SPI_USE_SPI5
    else if (&SPID5 == spip) {
      msg = spi_lld_get_dma(spip,
                            STM32_SPI_SPI5_RX_DMA_STREAM,
                            STM32_SPI_SPI5_TX_DMA_STREAM,
                            STM32_SPI_SPI5_IRQ_PRIORITY);
      if (msg != HAL_RET_SUCCESS) {
        return msg;
      }
      rccEnableSPI5(true);
      rccResetSPI5();
#if STM32_DMA_SUPPORTS_DMAMUX
      dmaSetRequestSource(spip->dmarx, STM32_DMAMUX1_SPI5_RX);
      dmaSetRequestSource(spip->dmatx, STM32_DMAMUX1_SPI5_TX);
#endif
    }
#endif

#if STM32_SPI_USE_SPI6
    else if (&SPID6 == spip) {
      msg = spi_lld_get_dma(spip,
                            STM32_SPI_SPI6_RX_DMA_STREAM,
                            STM32_SPI_SPI6_TX_DMA_STREAM,
                            STM32_SPI_SPI6_IRQ_PRIORITY);
      if (msg != HAL_RET_SUCCESS) {
        return msg;
      }
      rccEnableSPI6(true);
      rccResetSPI6();
#if STM32_DMA_SUPPORTS_DMAMUX
      dmaSetRequestSource(spip->dmarx, STM32_DMAMUX1_SPI6_RX);
      dmaSetRequestSource(spip->dmatx, STM32_DMAMUX1_SPI6_TX);
#endif
    }
#endif

    else {
      osalDbgAssert(false, "invalid SPI instance");
      return HAL_RET_IS_INVALID;
    }

    /* DMA setup.*/
    dmaStreamSetPeripheral(spip->dmarx, &spip->spi->DR);
    dmaStreamSetPeripheral(spip->dmatx, &spip->spi->DR);
  }
  else {
    /* De-activation before re-configuration.*/
    spi_lld_disable(spip);
  }

  /* Configuration-specific DMA setup.*/
  ds = spip->config->cr2 & SPI_CR2_DS;
  if (!ds || (ds <= (SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0))) {
    /* Frame width is 8 bits or smaller.*/
    spip->rxdmamode = (spip->rxdmamode & ~STM32_DMA_CR_SIZE_MASK) |
                      STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MSIZE_BYTE;
    spip->txdmamode = (spip->txdmamode & ~STM32_DMA_CR_SIZE_MASK) |
                      STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MSIZE_BYTE;
  }
  else {
    /* Frame width is larger than 8 bits.*/
    spip->rxdmamode = (spip->rxdmamode & ~STM32_DMA_CR_SIZE_MASK) |
                      STM32_DMA_CR_PSIZE_HWORD | STM32_DMA_CR_MSIZE_HWORD;
    spip->txdmamode = (spip->txdmamode & ~STM32_DMA_CR_SIZE_MASK) |
                      STM32_DMA_CR_PSIZE_HWORD | STM32_DMA_CR_MSIZE_HWORD;
  }

  if (spip->config->circular) {
    spip->rxdmamode |= (STM32_DMA_CR_CIRC | STM32_DMA_CR_HTIE);
    spip->txdmamode |= (STM32_DMA_CR_CIRC | STM32_DMA_CR_HTIE);
  }
  else {
    spip->rxdmamode &= ~(STM32_DMA_CR_CIRC | STM32_DMA_CR_HTIE);
    spip->txdmamode &= ~(STM32_DMA_CR_CIRC | STM32_DMA_CR_HTIE);
  }

  /* SPI setup.*/
  spi_lld_enable(spip);

  return HAL_RET_SUCCESS;
}

/**
 * @brief   Deactivates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_stop(SPIDriver *spip) {

  /* If in ready state then disables the SPI clock.*/
  if (spip->state == SPI_READY) {

    /* Just in case this has been called uncleanly.*/
    spi_lld_disable(spip);

    /* SPI cleanup.*/
    spip->spi->CR1  = 0;
    spip->spi->CR2  = 0;

    /* DMA channels release.*/
    dmaStreamFreeI(spip->dmatx);
    dmaStreamFreeI(spip->dmarx);
    spip->dmarx = NULL;
    spip->dmatx = NULL;

    /* Clock shutdown.*/
    if (false) {
    }

#if STM32_SPI_USE_SPI1
    else if (&SPID1 == spip) {
      rccDisableSPI1();
    }
#endif

#if STM32_SPI_USE_SPI2
    else if (&SPID2 == spip) {
      rccDisableSPI2();
    }
#endif

#if STM32_SPI_USE_SPI3
    else if (&SPID3 == spip) {
      rccDisableSPI3();
    }
#endif

#if STM32_SPI_USE_SPI4
    else if (&SPID4 == spip) {
      rccDisableSPI4();
    }
#endif

#if STM32_SPI_USE_SPI5
    else if (&SPID5 == spip) {
      rccDisableSPI5();
    }
#endif

#if STM32_SPI_USE_SPI6
    else if (&SPID6 == spip) {
      rccDisableSPI6();
    }
#endif

    else {
      osalDbgAssert(false, "invalid SPI instance");
    }
  }
}

#if (SPI_SELECT_MODE == SPI_SELECT_MODE_LLD) || defined(__DOXYGEN__)
/**
 * @brief   Asserts the slave select signal and prepares for transfers.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_select(SPIDriver *spip) {

  /* No implementation on STM32.*/
}

/**
 * @brief   Deasserts the slave select signal.
 * @details The previously selected peripheral is unselected.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_unselect(SPIDriver *spip) {

  /* No implementation on STM32.*/
}
#endif

/**
 * @brief   Ignores data on the SPI bus.
 * @details This synchronous function performs the transmission of a series of
 *          idle words on the SPI bus and ignores the received data.
 * @pre     In order to use this function the option @p SPI_USE_SYNCHRONIZATION
 *          must be enabled.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be ignored
 * @return              The operation status.
 *
 * @notapi
 */
msg_t spi_lld_ignore(SPIDriver *spip, size_t n) {

  osalDbgAssert(n <= STM32_DMA_MAX_TRANSFER, "unsupported DMA transfer size");

  dmaStreamSetMemory0(spip->dmarx, &spip->rxsink);
  dmaStreamSetTransactionSize(spip->dmarx, n);
  dmaStreamSetMode(spip->dmarx, spip->rxdmamode);

  dmaStreamSetMemory0(spip->dmatx, &spip->txsource);
  dmaStreamSetTransactionSize(spip->dmatx, n);
  dmaStreamSetMode(spip->dmatx, spip->txdmamode);

  dmaStreamEnable(spip->dmarx);
  dmaStreamEnable(spip->dmatx);

  return HAL_RET_SUCCESS;
}

/**
 * @brief   Exchanges data on the SPI bus.
 * @details This asynchronous function starts a simultaneous transmit/receive
 *          operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be exchanged
 * @param[in] txbuf     the pointer to the transmit buffer
 * @param[out] rxbuf    the pointer to the receive buffer
 * @return              The operation status.
 *
 * @notapi
 */
msg_t spi_lld_exchange(SPIDriver *spip, size_t n,
                       const void *txbuf, void *rxbuf) {

  osalDbgAssert(n <= STM32_DMA_MAX_TRANSFER, "unsupported DMA transfer size");

  dmaStreamSetMemory0(spip->dmarx, rxbuf);
  dmaStreamSetTransactionSize(spip->dmarx, n);
  dmaStreamSetMode(spip->dmarx, spip->rxdmamode | STM32_DMA_CR_MINC);

  dmaStreamSetMemory0(spip->dmatx, txbuf);
  dmaStreamSetTransactionSize(spip->dmatx, n);
  dmaStreamSetMode(spip->dmatx, spip->txdmamode | STM32_DMA_CR_MINC);

  dmaStreamEnable(spip->dmarx);
  dmaStreamEnable(spip->dmatx);

  return HAL_RET_SUCCESS;
}

/**
 * @brief   Sends data over the SPI bus.
 * @details This asynchronous function starts a transmit operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to send
 * @param[in] txbuf     the pointer to the transmit buffer
 * @return              The operation status.
 *
 * @notapi
 */
msg_t spi_lld_send(SPIDriver *spip, size_t n, const void *txbuf) {

  osalDbgAssert(n <= STM32_DMA_MAX_TRANSFER, "unsupported DMA transfer size");

  dmaStreamSetMemory0(spip->dmarx, &spip->rxsink);
  dmaStreamSetTransactionSize(spip->dmarx, n);
  dmaStreamSetMode(spip->dmarx, spip->rxdmamode);

  dmaStreamSetMemory0(spip->dmatx, txbuf);
  dmaStreamSetTransactionSize(spip->dmatx, n);
  dmaStreamSetMode(spip->dmatx, spip->txdmamode | STM32_DMA_CR_MINC);

  dmaStreamEnable(spip->dmarx);
  dmaStreamEnable(spip->dmatx);

  return HAL_RET_SUCCESS;
}

/**
 * @brief   Receives data from the SPI bus.
 * @details This asynchronous function starts a receive operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to receive
 * @param[out] rxbuf    the pointer to the receive buffer
 * @return              The operation status.
 *
 * @notapi
 */
msg_t spi_lld_receive(SPIDriver *spip, size_t n, void *rxbuf) {

  osalDbgAssert(n <= STM32_DMA_MAX_TRANSFER, "unsupported DMA transfer size");

  dmaStreamSetMemory0(spip->dmarx, rxbuf);
  dmaStreamSetTransactionSize(spip->dmarx, n);
  dmaStreamSetMode(spip->dmarx, spip->rxdmamode | STM32_DMA_CR_MINC);

  dmaStreamSetMemory0(spip->dmatx, &spip->txsource);
  dmaStreamSetTransactionSize(spip->dmatx, n);
  dmaStreamSetMode(spip->dmatx, spip->txdmamode);

  dmaStreamEnable(spip->dmarx);
  dmaStreamEnable(spip->dmatx);

  return HAL_RET_SUCCESS;
}

/**
 * @brief   Aborts the ongoing SPI operation, if any.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[out] sizep    pointer to the counter of frames not yet transferred
 *                      or @p NULL
 * @return              The operation status.
 *
 * @notapi
 */
msg_t spi_lld_stop_transfer(SPIDriver *spip, size_t *sizep) {

  /* Stopping TX DMA.*/
  dmaStreamDisable(spip->dmatx);

  /* Waiting for current frame completion then stop SPI.*/
  while ((spip->spi->SR & SPI_SR_BSY) != 0U) {
    /* Still busy.*/
  }

  /* Size of unprocessed data.*/
  if (sizep != NULL) {
    *sizep = dmaStreamGetTransactionSize(spip->dmarx);
  }

  /* Stopping RX DMA.*/
  dmaStreamDisable(spip->dmarx);

  return HAL_RET_SUCCESS;
}

/**
 * @brief   Exchanges one frame using a polled wait.
 * @details This synchronous function exchanges one frame using a polled
 *          synchronization method. This function is useful when exchanging
 *          small amount of data on high speed channels, usually in this
 *          situation is much more efficient just wait for completion using
 *          polling than suspending the thread waiting for an interrupt.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] frame     the data frame to send over the SPI bus
 * @return              The received data frame from the SPI bus.
 */
uint16_t spi_lld_polled_exchange(SPIDriver *spip, uint16_t frame) {

  /*
   * Data register must be accessed with the appropriate data size.
   * Byte size access (uint8_t *) for transactions that are <= 8-bit.
   * Halfword size access (uint16_t) for transactions that are <= 8-bit.
   */
  if ((spip->config->cr2 & SPI_CR2_DS) <= (SPI_CR2_DS_2 |
                                           SPI_CR2_DS_1 |
                                           SPI_CR2_DS_0)) {
    volatile uint8_t *dr8p = (volatile uint8_t *)&spip->spi->DR;
    *dr8p = (uint8_t)frame;
    while ((spip->spi->SR & SPI_SR_RXNE) == 0U) {
      /* Waiting frame transfer.*/
    }
    frame = (uint16_t)*dr8p;
  }
  else {
    volatile uint16_t *dr16p = (volatile uint16_t *)&spip->spi->DR;
    *dr16p = (uint16_t)frame;
    while ((spip->spi->SR & SPI_SR_RXNE) == 0U) {
      /* Waiting frame transfer.*/
    }
    frame = (uint16_t)*dr16p;
  }

  return frame;
}

#endif /* HAL_USE_SPI */

/** @} */
