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
 * @file    SPIv1/hal_spi_lld.c
 * @brief   STM32 SPI subsystem low level driver source.
 *
 * @addtogroup SPI
 * @{
 */

#include "hal.h"

#if HAL_USE_SPI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define SPI1_RX_DMA_CHANNEL                                                 \
  STM32_DMA_GETCHANNEL(STM32_SPI_SPI1_RX_DMA_STREAM,                        \
                       STM32_SPI1_RX_DMA_CHN)

#define SPI1_TX_DMA_CHANNEL                                                 \
  STM32_DMA_GETCHANNEL(STM32_SPI_SPI1_TX_DMA_STREAM,                        \
                       STM32_SPI1_TX_DMA_CHN)

#define SPI2_RX_DMA_CHANNEL                                                 \
  STM32_DMA_GETCHANNEL(STM32_SPI_SPI2_RX_DMA_STREAM,                        \
                       STM32_SPI2_RX_DMA_CHN)

#define SPI2_TX_DMA_CHANNEL                                                 \
  STM32_DMA_GETCHANNEL(STM32_SPI_SPI2_TX_DMA_STREAM,                        \
                       STM32_SPI2_TX_DMA_CHN)

#define SPI3_RX_DMA_CHANNEL                                                 \
  STM32_DMA_GETCHANNEL(STM32_SPI_SPI3_RX_DMA_STREAM,                        \
                       STM32_SPI3_RX_DMA_CHN)

#define SPI3_TX_DMA_CHANNEL                                                 \
  STM32_DMA_GETCHANNEL(STM32_SPI_SPI3_TX_DMA_STREAM,                        \
                       STM32_SPI3_TX_DMA_CHN)

#define SPI4_RX_DMA_CHANNEL                                                 \
  STM32_DMA_GETCHANNEL(STM32_SPI_SPI4_RX_DMA_STREAM,                        \
                       STM32_SPI4_RX_DMA_CHN)

#define SPI4_TX_DMA_CHANNEL                                                 \
  STM32_DMA_GETCHANNEL(STM32_SPI_SPI4_TX_DMA_STREAM,                        \
                       STM32_SPI4_TX_DMA_CHN)

#define SPI5_RX_DMA_CHANNEL                                                 \
  STM32_DMA_GETCHANNEL(STM32_SPI_SPI5_RX_DMA_STREAM,                        \
                       STM32_SPI5_RX_DMA_CHN)

#define SPI5_TX_DMA_CHANNEL                                                 \
  STM32_DMA_GETCHANNEL(STM32_SPI_SPI5_TX_DMA_STREAM,                        \
                       STM32_SPI5_TX_DMA_CHN)

#define SPI6_RX_DMA_CHANNEL                                                 \
  STM32_DMA_GETCHANNEL(STM32_SPI_SPI6_RX_DMA_STREAM,                        \
                       STM32_SPI6_RX_DMA_CHN)

#define SPI6_TX_DMA_CHANNEL                                                 \
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

static const uint16_t dummytx = 0xFFFFU;
static uint16_t dummyrx;

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Shared end-of-rx service routine.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] flags     pre-shifted content of the ISR register
 */
static void spi_lld_serve_rx_interrupt(SPIDriver *spip, uint32_t flags) {

  /* DMA errors handling.*/
#if defined(STM32_SPI_DMA_ERROR_HOOK)
  if ((flags & (STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF)) != 0) {
    STM32_SPI_DMA_ERROR_HOOK(spip);
  }
#else
  (void)flags;
#endif

  if (spip->config->circular) {
    if ((flags & STM32_DMA_ISR_HTIF) != 0U) {
      /* Half buffer interrupt.*/
      _spi_isr_half_code(spip);
    }
    if ((flags & STM32_DMA_ISR_TCIF) != 0U) {
      /* End buffer interrupt.*/
      _spi_isr_full_code(spip);
    }
  }
  else {
    /* Stopping DMAs.*/
    dmaStreamDisable(spip->dmatx);
    dmaStreamDisable(spip->dmarx);

    /* Portable SPI ISR code defined in the high level driver, note, it is
       a macro.*/
    _spi_isr_code(spip);
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
#if defined(STM32_SPI_DMA_ERROR_HOOK)
  (void)spip;
  if ((flags & (STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF)) != 0) {
    STM32_SPI_DMA_ERROR_HOOK(spip);
  }
#else
  (void)spip;
  (void)flags;
#endif
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
  SPID1.rxdmamode = STM32_DMA_CR_CHSEL(SPI1_RX_DMA_CHANNEL) |
                    STM32_DMA_CR_PL(STM32_SPI_SPI1_DMA_PRIORITY) |
                    STM32_DMA_CR_DIR_P2M |
                    STM32_DMA_CR_TCIE |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
  SPID1.txdmamode = STM32_DMA_CR_CHSEL(SPI1_TX_DMA_CHANNEL) |
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
  SPID2.rxdmamode = STM32_DMA_CR_CHSEL(SPI2_RX_DMA_CHANNEL) |
                    STM32_DMA_CR_PL(STM32_SPI_SPI2_DMA_PRIORITY) |
                    STM32_DMA_CR_DIR_P2M |
                    STM32_DMA_CR_TCIE |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
  SPID2.txdmamode = STM32_DMA_CR_CHSEL(SPI2_TX_DMA_CHANNEL) |
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
  SPID3.rxdmamode = STM32_DMA_CR_CHSEL(SPI3_RX_DMA_CHANNEL) |
                    STM32_DMA_CR_PL(STM32_SPI_SPI3_DMA_PRIORITY) |
                    STM32_DMA_CR_DIR_P2M |
                    STM32_DMA_CR_TCIE |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
  SPID3.txdmamode = STM32_DMA_CR_CHSEL(SPI3_TX_DMA_CHANNEL) |
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
  SPID4.rxdmamode = STM32_DMA_CR_CHSEL(SPI4_RX_DMA_CHANNEL) |
                    STM32_DMA_CR_PL(STM32_SPI_SPI4_DMA_PRIORITY) |
                    STM32_DMA_CR_DIR_P2M |
                    STM32_DMA_CR_TCIE |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
  SPID4.txdmamode = STM32_DMA_CR_CHSEL(SPI4_TX_DMA_CHANNEL) |
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
  SPID5.rxdmamode = STM32_DMA_CR_CHSEL(SPI5_RX_DMA_CHANNEL) |
                    STM32_DMA_CR_PL(STM32_SPI_SPI5_DMA_PRIORITY) |
                    STM32_DMA_CR_DIR_P2M |
                    STM32_DMA_CR_TCIE |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
  SPID5.txdmamode = STM32_DMA_CR_CHSEL(SPI5_TX_DMA_CHANNEL) |
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
  SPID6.rxdmamode = STM32_DMA_CR_CHSEL(SPI6_RX_DMA_CHANNEL) |
                    STM32_DMA_CR_PL(STM32_SPI_SPI6_DMA_PRIORITY) |
                    STM32_DMA_CR_DIR_P2M |
                    STM32_DMA_CR_TCIE |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
  SPID6.txdmamode = STM32_DMA_CR_CHSEL(SPI6_TX_DMA_CHANNEL) |
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
 *
 * @notapi
 */
void spi_lld_start(SPIDriver *spip) {

  /* If in stopped state then enables the SPI and DMA clocks.*/
  if (spip->state == SPI_STOP) {
#if STM32_SPI_USE_SPI1
    if (&SPID1 == spip) {
      spip->dmarx = dmaStreamAllocI(STM32_SPI_SPI1_RX_DMA_STREAM,
                                    STM32_SPI_SPI1_IRQ_PRIORITY,
                                    (stm32_dmaisr_t)spi_lld_serve_rx_interrupt,
                                    (void *)spip);
      osalDbgAssert(spip->dmarx != NULL, "unable to allocate stream");
      spip->dmatx = dmaStreamAllocI(STM32_SPI_SPI1_TX_DMA_STREAM,
                                    STM32_SPI_SPI1_IRQ_PRIORITY,
                                    (stm32_dmaisr_t)spi_lld_serve_tx_interrupt,
                                    (void *)spip);
      osalDbgAssert(spip->dmatx != NULL, "unable to allocate stream");
      rccResetSPI1();
      rccEnableSPI1(true);
    }
#endif
#if STM32_SPI_USE_SPI2
    if (&SPID2 == spip) {
      spip->dmarx = dmaStreamAllocI(STM32_SPI_SPI2_RX_DMA_STREAM,
                                    STM32_SPI_SPI2_IRQ_PRIORITY,
                                    (stm32_dmaisr_t)spi_lld_serve_rx_interrupt,
                                    (void *)spip);
      osalDbgAssert(spip->dmarx != NULL, "unable to allocate stream");
      spip->dmatx = dmaStreamAllocI(STM32_SPI_SPI2_TX_DMA_STREAM,
                                    STM32_SPI_SPI2_IRQ_PRIORITY,
                                    (stm32_dmaisr_t)spi_lld_serve_tx_interrupt,
                                    (void *)spip);
      osalDbgAssert(spip->dmatx != NULL, "unable to allocate stream");
      rccResetSPI2();
      rccEnableSPI2(true);
    }
#endif
#if STM32_SPI_USE_SPI3
    if (&SPID3 == spip) {
      spip->dmarx = dmaStreamAllocI(STM32_SPI_SPI3_RX_DMA_STREAM,
                                    STM32_SPI_SPI3_IRQ_PRIORITY,
                                    (stm32_dmaisr_t)spi_lld_serve_rx_interrupt,
                                    (void *)spip);
      osalDbgAssert(spip->dmarx != NULL, "unable to allocate stream");
      spip->dmatx = dmaStreamAllocI(STM32_SPI_SPI3_TX_DMA_STREAM,
                                    STM32_SPI_SPI3_IRQ_PRIORITY,
                                    (stm32_dmaisr_t)spi_lld_serve_tx_interrupt,
                                    (void *)spip);
      osalDbgAssert(spip->dmatx != NULL, "unable to allocate stream");
      rccResetSPI3();
      rccEnableSPI3(true);
    }
#endif
#if STM32_SPI_USE_SPI4
    if (&SPID4 == spip) {
      spip->dmarx = dmaStreamAllocI(STM32_SPI_SPI4_RX_DMA_STREAM,
                                    STM32_SPI_SPI4_IRQ_PRIORITY,
                                    (stm32_dmaisr_t)spi_lld_serve_rx_interrupt,
                                    (void *)spip);
      osalDbgAssert(spip->dmarx != NULL, "unable to allocate stream");
      spip->dmatx = dmaStreamAllocI(STM32_SPI_SPI4_TX_DMA_STREAM,
                                    STM32_SPI_SPI4_IRQ_PRIORITY,
                                    (stm32_dmaisr_t)spi_lld_serve_tx_interrupt,
                                    (void *)spip);
      osalDbgAssert(spip->dmatx != NULL, "unable to allocate stream");
      rccResetSPI4();
      rccEnableSPI4(true);
    }
#endif
#if STM32_SPI_USE_SPI5
    if (&SPID5 == spip) {
      spip->dmarx = dmaStreamAllocI(STM32_SPI_SPI5_RX_DMA_STREAM,
                                    STM32_SPI_SPI5_IRQ_PRIORITY,
                                    (stm32_dmaisr_t)spi_lld_serve_rx_interrupt,
                                    (void *)spip);
      osalDbgAssert(spip->dmarx != NULL, "unable to allocate stream");
      spip->dmatx = dmaStreamAllocI(STM32_SPI_SPI5_TX_DMA_STREAM,
                                    STM32_SPI_SPI5_IRQ_PRIORITY,
                                    (stm32_dmaisr_t)spi_lld_serve_tx_interrupt,
                                    (void *)spip);
      osalDbgAssert(spip->dmatx != NULL, "unable to allocate stream");
      rccResetSPI5();
      rccEnableSPI5(true);
    }
#endif
#if STM32_SPI_USE_SPI6
    if (&SPID6 == spip) {
      spip->dmarx = dmaStreamAllocI(STM32_SPI_SPI6_RX_DMA_STREAM,
                                    STM32_SPI_SPI6_IRQ_PRIORITY,
                                    (stm32_dmaisr_t)spi_lld_serve_rx_interrupt,
                                    (void *)spip);
      osalDbgAssert(spip->dmarx != NULL, "unable to allocate stream");
      spip->dmatx = dmaStreamAllocI(STM32_SPI_SPI6_TX_DMA_STREAM,
                                    STM32_SPI_SPI6_IRQ_PRIORITY,
                                    (stm32_dmaisr_t)spi_lld_serve_tx_interrupt,
                                    (void *)spip);
      osalDbgAssert(spip->dmatx != NULL, "unable to allocate stream");
      rccResetSPI6();
      rccEnableSPI6(true);
    }
#endif

    /* DMA setup.*/
    dmaStreamSetPeripheral(spip->dmarx, &spip->spi->DR);
    dmaStreamSetPeripheral(spip->dmatx, &spip->spi->DR);
  }

  /* Configuration-specific DMA setup.*/
  if ((spip->config->cr1 & SPI_CR1_DFF) == 0) {
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

  /* SPI setup and enable.*/
  spip->spi->CR1 &= ~SPI_CR1_SPE;
  spip->spi->CR1  = spip->config->cr1 | SPI_CR1_MSTR | SPI_CR1_SSM |
                    SPI_CR1_SSI;
  spip->spi->CR2  = spip->config->cr2 | SPI_CR2_SSOE | SPI_CR2_RXDMAEN |
                    SPI_CR2_TXDMAEN;
  spip->spi->CR1 |= SPI_CR1_SPE;
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

    /* SPI disable.*/
    spip->spi->CR1 &= ~SPI_CR1_SPE;
    spip->spi->CR1  = 0;
    spip->spi->CR2  = 0;
    dmaStreamFreeI(spip->dmarx);
    dmaStreamFreeI(spip->dmatx);
    spip->dmarx = NULL;
    spip->dmatx = NULL;

#if STM32_SPI_USE_SPI1
    if (&SPID1 == spip)
      rccDisableSPI1();
#endif
#if STM32_SPI_USE_SPI2
    if (&SPID2 == spip)
      rccDisableSPI2();
#endif
#if STM32_SPI_USE_SPI3
    if (&SPID3 == spip)
      rccDisableSPI3();
#endif
#if STM32_SPI_USE_SPI4
    if (&SPID4 == spip)
      rccDisableSPI4();
#endif
#if STM32_SPI_USE_SPI5
    if (&SPID5 == spip)
      rccDisableSPI5();
#endif
#if STM32_SPI_USE_SPI6
    if (&SPID6 == spip)
      rccDisableSPI6();
#endif
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
 * @details This asynchronous function starts the transmission of a series of
 *          idle words on the SPI bus and ignores the received data.
 * @post    At the end of the operation the configured callback is invoked.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be ignored
 *
 * @notapi
 */
void spi_lld_ignore(SPIDriver *spip, size_t n) {

  osalDbgAssert(n <= STM32_DMA_MAX_TRANSFER, "unsupported DMA transfer size");

  dmaStreamSetMemory0(spip->dmarx, &dummyrx);
  dmaStreamSetTransactionSize(spip->dmarx, n);
  dmaStreamSetMode(spip->dmarx, spip->rxdmamode);

  dmaStreamSetMemory0(spip->dmatx, &dummytx);
  dmaStreamSetTransactionSize(spip->dmatx, n);
  dmaStreamSetMode(spip->dmatx, spip->txdmamode);

  dmaStreamEnable(spip->dmarx);
  dmaStreamEnable(spip->dmatx);
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
 *
 * @notapi
 */
void spi_lld_exchange(SPIDriver *spip, size_t n,
                      const void *txbuf, void *rxbuf) {

  osalDbgAssert(n <= STM32_DMA_MAX_TRANSFER, "unsupported DMA transfer size");

  dmaStreamSetMemory0(spip->dmarx, rxbuf);
  dmaStreamSetTransactionSize(spip->dmarx, n);
  dmaStreamSetMode(spip->dmarx, spip->rxdmamode| STM32_DMA_CR_MINC);

  dmaStreamSetMemory0(spip->dmatx, txbuf);
  dmaStreamSetTransactionSize(spip->dmatx, n);
  dmaStreamSetMode(spip->dmatx, spip->txdmamode | STM32_DMA_CR_MINC);

  dmaStreamEnable(spip->dmarx);
  dmaStreamEnable(spip->dmatx);
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
 *
 * @notapi
 */
void spi_lld_send(SPIDriver *spip, size_t n, const void *txbuf) {

  osalDbgAssert(n <= STM32_DMA_MAX_TRANSFER, "unsupported DMA transfer size");

  dmaStreamSetMemory0(spip->dmarx, &dummyrx);
  dmaStreamSetTransactionSize(spip->dmarx, n);
  dmaStreamSetMode(spip->dmarx, spip->rxdmamode);

  dmaStreamSetMemory0(spip->dmatx, txbuf);
  dmaStreamSetTransactionSize(spip->dmatx, n);
  dmaStreamSetMode(spip->dmatx, spip->txdmamode | STM32_DMA_CR_MINC);

  dmaStreamEnable(spip->dmarx);
  dmaStreamEnable(spip->dmatx);
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
 *
 * @notapi
 */
void spi_lld_receive(SPIDriver *spip, size_t n, void *rxbuf) {

  osalDbgAssert(n <= STM32_DMA_MAX_TRANSFER, "unsupported DMA transfer size");

  dmaStreamSetMemory0(spip->dmarx, rxbuf);
  dmaStreamSetTransactionSize(spip->dmarx, n);
  dmaStreamSetMode(spip->dmarx, spip->rxdmamode | STM32_DMA_CR_MINC);

  dmaStreamSetMemory0(spip->dmatx, &dummytx);
  dmaStreamSetTransactionSize(spip->dmatx, n);
  dmaStreamSetMode(spip->dmatx, spip->txdmamode);

  dmaStreamEnable(spip->dmarx);
  dmaStreamEnable(spip->dmatx);
}

#if (SPI_SUPPORTS_CIRCULAR == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Aborts the ongoing SPI operation, if any.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_abort(SPIDriver *spip) {

  /* Stopping DMAs.*/
  dmaStreamDisable(spip->dmatx);
  dmaStreamDisable(spip->dmarx);
}
#endif /* SPI_SUPPORTS_CIRCULAR == TRUE */

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

  spip->spi->DR = frame;
  while ((spip->spi->SR & SPI_SR_RXNE) == 0)
    ;
  return spip->spi->DR;
}

#endif /* HAL_USE_SPI */

/** @} */
