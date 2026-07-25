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
 * @file    SPIv3/hal_spi_lld.c
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

static const uint32_t dummytx = STM32_SPI_FILLER_PATTERN;
static uint32_t dummyrx;

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static void spi_lld_wait_complete(SPIDriver *spip) {

  while ((spip->spi->CR1 & SPI_CR1_CSTART) != 0) {
  }
  spip->spi->IFCR = 0xFFFFFFFF;
}

#if defined(STM32_SPI_BDMA_REQUIRED)
/**
 * @brief   Shared DMA end-of-rx service routine.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] flags     pre-shifted content of the ISR register
 */
static void spi_lld_serve_bdma_rx_interrupt(SPIDriver *spip, uint32_t flags) {

  /* DMA errors handling.*/
#if defined(STM32_SPI_DMA_ERROR_HOOK)
  if ((flags & STM32_BDMA_ISR_TEIF) != 0U) {
    STM32_SPI_DMA_ERROR_HOOK(spip);
  }
#else
  (void)flags;
#endif

  if (spip->config->circular) {
    if ((flags & STM32_BDMA_ISR_HTIF) != 0U) {
      /* Half buffer interrupt.*/
      _spi_isr_half_code(spip);
    }
    if ((flags & STM32_BDMA_ISR_TCIF) != 0U) {
      /* End buffer interrupt.*/
      _spi_isr_full_code(spip);
    }
  }
  else {
    /* Stopping SPI.*/
    spip->spi->CR1 |= SPI_CR1_CSUSP;

    /* Stopping DMAs.*/
    bdmaStreamDisable(spip->tx.bdma);
    bdmaStreamDisable(spip->rx.bdma);

    /* Portable SPI ISR code defined in the high level driver, note, it is
       a macro.*/
    _spi_isr_code(spip);
  }
}

/**
 * @brief   Shared BDMA end-of-tx service routine.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] flags     pre-shifted content of the ISR register
 */
static void spi_lld_serve_bdma_tx_interrupt(SPIDriver *spip, uint32_t flags) {

  /* DMA errors handling.*/
#if defined(STM32_SPI_DMA_ERROR_HOOK)
  (void)spip;
  if ((flags & STM32_BDMA_ISR_TEIF) != 0) {
    STM32_SPI_DMA_ERROR_HOOK(spip);
  }
#else
  (void)spip;
  (void)flags;
#endif
}
#endif /* defined(STM32_SPI_BDMA_REQUIRED) */

#if defined(STM32_SPI_DMA_REQUIRED)
/**
 * @brief   Shared DMA end-of-rx service routine.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] flags     pre-shifted content of the ISR register
 */
static void spi_lld_serve_dma_rx_interrupt(SPIDriver *spip, uint32_t flags) {

  /* DMA errors handling.*/
#if defined(STM32_SPI_DMA_ERROR_HOOK)
  if ((flags & (STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF)) != 0U) {
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
    /* Stopping SPI.*/
    spip->spi->CR1 |= SPI_CR1_CSUSP;

    /* Stopping DMAs.*/
    dmaStreamDisable(spip->tx.dma);
    dmaStreamDisable(spip->rx.dma);

    /* Portable SPI ISR code defined in the high level driver, note, it is
       a macro.*/
    _spi_isr_code(spip);
  }
}

/**
 * @brief   Shared DMA end-of-tx service routine.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] flags     pre-shifted content of the ISR register
 */
static void spi_lld_serve_dma_tx_interrupt(SPIDriver *spip, uint32_t flags) {

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
#endif /* defined(STM32_SPI_DMA_REQUIRED) */

/**
 * @brief   Shared SPI service routine.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 */
static void spi_lld_serve_interrupt(SPIDriver *spip) {
  uint32_t sr;

  sr = spip->spi->SR & spip->spi->IER;
  spip->spi->IFCR = sr;

  if ((sr & SPI_SR_OVR) != 0U) {
    /* CHTODO: fault notification.*/
  }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if STM32_SPI_USE_SPI1 || defined(__DOXYGEN__)
#if !defined(STM32_SPI1_SUPPRESS_ISR)
#if !defined(STM32_SPI1_HANDLER)
#error "STM32_SPI1_HANDLER not defined"
#endif
/**
 * @brief   SPI1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_SPI1_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spi_lld_serve_interrupt(&SPID1);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(STM32_SPI1_SUPPRESS_ISR) */
#endif /* STM32_SPI_USE_SPI1 */

#if STM32_SPI_USE_SPI2 || defined(__DOXYGEN__)
#if !defined(STM32_SPI2_SUPPRESS_ISR)
#if !defined(STM32_SPI2_HANDLER)
#error "STM32_SPI2_HANDLER not defined"
#endif
/**
 * @brief   SPI2 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_SPI2_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spi_lld_serve_interrupt(&SPID2);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(STM32_SPI2_SUPPRESS_ISR) */
#endif /* STM32_SPI_USE_SPI2 */

#if STM32_SPI_USE_SPI3 || defined(__DOXYGEN__)
#if !defined(STM32_SPI3_SUPPRESS_ISR)
#if !defined(STM32_SPI3_HANDLER)
#error "STM32_SPI3_HANDLER not defined"
#endif
/**
 * @brief   SPI3 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_SPI3_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spi_lld_serve_interrupt(&SPID3);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(STM32_SPI3_SUPPRESS_ISR) */
#endif /* STM32_SPI_USE_SPI3 */

#if STM32_SPI_USE_SPI4 || defined(__DOXYGEN__)
#if !defined(STM32_SPI4_SUPPRESS_ISR)
#if !defined(STM32_SPI4_HANDLER)
#error "STM32_SPI4_HANDLER not defined"
#endif
/**
 * @brief   SPI4 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_SPI4_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spi_lld_serve_interrupt(&SPID4);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(STM32_SPI4_SUPPRESS_ISR) */
#endif /* STM32_SPI_USE_SPI4 */

#if STM32_SPI_USE_SPI5 || defined(__DOXYGEN__)
#if !defined(STM32_SPI5_SUPPRESS_ISR)
#if !defined(STM32_SPI5_HANDLER)
#error "STM32_SPI5_HANDLER not defined"
#endif
/**
 * @brief   SPI5 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_SPI5_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spi_lld_serve_interrupt(&SPID5);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(STM32_SPI5_SUPPRESS_ISR) */
#endif /* STM32_SPI_USE_SPI5 */

#if STM32_SPI_USE_SPI6 || defined(__DOXYGEN__)
#if !defined(STM32_SPI6_SUPPRESS_ISR)
#if !defined(STM32_SPI6_HANDLER)
#error "STM32_SPI6_HANDLER not defined"
#endif
/**
 * @brief   SPI6 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_SPI6_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spi_lld_serve_interrupt(&SPID6);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(STM32_SPI6_SUPPRESS_ISR) */
#endif /* STM32_SPI_USE_SPI6 */

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
#if defined(STM32_SPI_DMA_REQUIRED) && defined(STM32_SPI_BDMA_REQUIRED)
  SPID1.is_bdma   = false;
#endif
  SPID1.rx.dma    = NULL;
  SPID1.tx.dma    = NULL;
  SPID1.rxdmamode = STM32_DMA_CR_PL(STM32_SPI_SPI1_DMA_PRIORITY) |
                    STM32_DMA_CR_DIR_P2M |
                    STM32_DMA_CR_TCIE |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
  SPID1.txdmamode = STM32_DMA_CR_PL(STM32_SPI_SPI1_DMA_PRIORITY) |
                    STM32_DMA_CR_DIR_M2P |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
#if !defined(STM32_SPI1_SUPPRESS_ISR)
  nvicEnableVector(STM32_SPI1_NUMBER, STM32_SPI_SPI1_IRQ_PRIORITY);
#endif
#endif

#if STM32_SPI_USE_SPI2
  spiObjectInit(&SPID2);
  SPID2.spi       = SPI2;
#if defined(STM32_SPI_DMA_REQUIRED) && defined(STM32_SPI_BDMA_REQUIRED)
  SPID2.is_bdma   = false;
#endif
  SPID2.rx.dma    = NULL;
  SPID2.tx.dma    = NULL;
  SPID2.rxdmamode = STM32_DMA_CR_PL(STM32_SPI_SPI2_DMA_PRIORITY) |
                    STM32_DMA_CR_DIR_P2M |
                    STM32_DMA_CR_TCIE |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
  SPID2.txdmamode = STM32_DMA_CR_PL(STM32_SPI_SPI2_DMA_PRIORITY) |
                    STM32_DMA_CR_DIR_M2P |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
#if !defined(STM32_SPI2_SUPPRESS_ISR)
  nvicEnableVector(STM32_SPI2_NUMBER, STM32_SPI_SPI2_IRQ_PRIORITY);
#endif
#endif

#if STM32_SPI_USE_SPI3
  spiObjectInit(&SPID3);
  SPID3.spi       = SPI3;
#if defined(STM32_SPI_DMA_REQUIRED) && defined(STM32_SPI_BDMA_REQUIRED)
  SPID3.is_bdma   = false;
#endif
  SPID3.rx.dma    = NULL;
  SPID3.tx.dma    = NULL;
  SPID3.rxdmamode = STM32_DMA_CR_PL(STM32_SPI_SPI3_DMA_PRIORITY) |
                    STM32_DMA_CR_DIR_P2M |
                    STM32_DMA_CR_TCIE |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
  SPID3.txdmamode = STM32_DMA_CR_PL(STM32_SPI_SPI3_DMA_PRIORITY) |
                    STM32_DMA_CR_DIR_M2P |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
#if !defined(STM32_SPI3_SUPPRESS_ISR)
  nvicEnableVector(STM32_SPI3_NUMBER, STM32_SPI_SPI3_IRQ_PRIORITY);
#endif
#endif

#if STM32_SPI_USE_SPI4
  spiObjectInit(&SPID4);
  SPID4.spi       = SPI4;
#if defined(STM32_SPI_DMA_REQUIRED) && defined(STM32_SPI_BDMA_REQUIRED)
  SPID4.is_bdma   = false;
#endif
  SPID4.rx.dma    = NULL;
  SPID4.tx.dma    = NULL;
  SPID4.rxdmamode = STM32_DMA_CR_PL(STM32_SPI_SPI4_DMA_PRIORITY) |
                    STM32_DMA_CR_DIR_P2M |
                    STM32_DMA_CR_TCIE |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
  SPID4.txdmamode = STM32_DMA_CR_PL(STM32_SPI_SPI4_DMA_PRIORITY) |
                    STM32_DMA_CR_DIR_M2P |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
#if !defined(STM32_SPI4_SUPPRESS_ISR)
  nvicEnableVector(STM32_SPI4_NUMBER, STM32_SPI_SPI4_IRQ_PRIORITY);
#endif
#endif

#if STM32_SPI_USE_SPI5
  spiObjectInit(&SPID5);
  SPID5.spi       = SPI5;
#if defined(STM32_SPI_DMA_REQUIRED) && defined(STM32_SPI_BDMA_REQUIRED)
  SPID5.is_bdma   = false;
#endif
  SPID5.rx.dma    = NULL;
  SPID5.tx.dma    = NULL;
  SPID5.rxdmamode = STM32_DMA_CR_PL(STM32_SPI_SPI5_DMA_PRIORITY) |
                    STM32_DMA_CR_DIR_P2M |
                    STM32_DMA_CR_TCIE |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
  SPID5.txdmamode = STM32_DMA_CR_PL(STM32_SPI_SPI5_DMA_PRIORITY) |
                    STM32_DMA_CR_DIR_M2P |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
#if !defined(STM32_SPI5_SUPPRESS_ISR)
  nvicEnableVector(STM32_SPI5_NUMBER, STM32_SPI_SPI5_IRQ_PRIORITY);
#endif
#endif

#if STM32_SPI_USE_SPI6
  spiObjectInit(&SPID6);
  SPID6.spi       = SPI6;
#if defined(STM32_SPI_DMA_REQUIRED) && defined(STM32_SPI_BDMA_REQUIRED)
  SPID6.is_bdma   = true;
#endif
  SPID6.rx.bdma   = NULL;
  SPID6.tx.bdma   = NULL;
  SPID6.rxdmamode = STM32_BDMA_CR_PL(STM32_SPI_SPI6_DMA_PRIORITY) |
                    STM32_BDMA_CR_DIR_P2M |
                    STM32_BDMA_CR_TCIE |
                    STM32_BDMA_CR_TEIE;
  SPID6.txdmamode = STM32_BDMA_CR_PL(STM32_SPI_SPI6_DMA_PRIORITY) |
                    STM32_BDMA_CR_DIR_M2P |
                    STM32_BDMA_CR_TEIE;
#if !defined(STM32_SPI6_SUPPRESS_ISR)
  nvicEnableVector(STM32_SPI6_NUMBER, STM32_SPI_SPI6_IRQ_PRIORITY);
#endif
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
  uint32_t dsize;

  /* If in stopped state then enables the SPI and DMA clocks.*/
  if (spip->state == SPI_STOP) {
#if STM32_SPI_USE_SPI1
    if (&SPID1 == spip) {
      spip->rx.dma = dmaStreamAllocI(STM32_SPI_SPI1_RX_DMA_STREAM,
                                     STM32_SPI_SPI1_IRQ_PRIORITY,
                                     (stm32_dmaisr_t)spi_lld_serve_dma_rx_interrupt,
                                     (void *)spip);
      osalDbgAssert(spip->rx.dma != NULL, "unable to allocate stream");
      spip->tx.dma = dmaStreamAllocI(STM32_SPI_SPI1_TX_DMA_STREAM,
                                     STM32_SPI_SPI1_IRQ_PRIORITY,
                                     (stm32_dmaisr_t)spi_lld_serve_dma_tx_interrupt,
                                     (void *)spip);
      osalDbgAssert(spip->tx.dma != NULL, "unable to allocate stream");
      rccResetSPI1();
      rccEnableSPI1(true);
      dmaSetRequestSource(spip->rx.dma, STM32_DMAMUX1_SPI1_RX);
      dmaSetRequestSource(spip->tx.dma, STM32_DMAMUX1_SPI1_TX);
    }
#endif
#if STM32_SPI_USE_SPI2
    if (&SPID2 == spip) {
      spip->rx.dma = dmaStreamAllocI(STM32_SPI_SPI2_RX_DMA_STREAM,
                                     STM32_SPI_SPI2_IRQ_PRIORITY,
                                     (stm32_dmaisr_t)spi_lld_serve_dma_rx_interrupt,
                                     (void *)spip);
      osalDbgAssert(spip->rx.dma != NULL, "unable to allocate stream");
      spip->tx.dma = dmaStreamAllocI(STM32_SPI_SPI2_TX_DMA_STREAM,
                                     STM32_SPI_SPI2_IRQ_PRIORITY,
                                     (stm32_dmaisr_t)spi_lld_serve_dma_tx_interrupt,
                                     (void *)spip);
      osalDbgAssert(spip->tx.dma != NULL, "unable to allocate stream");
      rccResetSPI2();
      rccEnableSPI2(true);
      dmaSetRequestSource(spip->rx.dma, STM32_DMAMUX1_SPI2_RX);
      dmaSetRequestSource(spip->tx.dma, STM32_DMAMUX1_SPI2_TX);
    }
#endif
#if STM32_SPI_USE_SPI3
    if (&SPID3 == spip) {
      spip->rx.dma = dmaStreamAllocI(STM32_SPI_SPI3_RX_DMA_STREAM,
                                     STM32_SPI_SPI3_IRQ_PRIORITY,
                                     (stm32_dmaisr_t)spi_lld_serve_dma_rx_interrupt,
                                     (void *)spip);
      osalDbgAssert(spip->rx.dma != NULL, "unable to allocate stream");
      spip->tx.dma = dmaStreamAllocI(STM32_SPI_SPI3_TX_DMA_STREAM,
                                     STM32_SPI_SPI3_IRQ_PRIORITY,
                                     (stm32_dmaisr_t)spi_lld_serve_dma_tx_interrupt,
                                     (void *)spip);
      osalDbgAssert(spip->tx.dma != NULL, "unable to allocate stream");
      rccResetSPI3();
      rccEnableSPI3(true);
      dmaSetRequestSource(spip->rx.dma, STM32_DMAMUX1_SPI3_RX);
      dmaSetRequestSource(spip->tx.dma, STM32_DMAMUX1_SPI3_TX);
    }
#endif
#if STM32_SPI_USE_SPI4
    if (&SPID4 == spip) {
      spip->rx.dma = dmaStreamAllocI(STM32_SPI_SPI4_RX_DMA_STREAM,
                                     STM32_SPI_SPI4_IRQ_PRIORITY,
                                     (stm32_dmaisr_t)spi_lld_serve_dma_rx_interrupt,
                                     (void *)spip);
      osalDbgAssert(spip->rx.dma != NULL, "unable to allocate stream");
      spip->tx.dma = dmaStreamAllocI(STM32_SPI_SPI4_TX_DMA_STREAM,
                                     STM32_SPI_SPI4_IRQ_PRIORITY,
                                     (stm32_dmaisr_t)spi_lld_serve_dma_tx_interrupt,
                                     (void *)spip);
      osalDbgAssert(spip->tx.dma != NULL, "unable to allocate stream");
      rccResetSPI4();
      rccEnableSPI4(true);
      dmaSetRequestSource(spip->rx.dma, STM32_DMAMUX1_SPI4_RX);
      dmaSetRequestSource(spip->tx.dma, STM32_DMAMUX1_SPI4_TX);
    }
#endif
#if STM32_SPI_USE_SPI5
    if (&SPID5 == spip) {
      spip->rx.dma = dmaStreamAllocI(STM32_SPI_SPI5_RX_DMA_STREAM,
                                     STM32_SPI_SPI5_IRQ_PRIORITY,
                                     (stm32_dmaisr_t)spi_lld_serve_dma_rx_interrupt,
                                     (void *)spip);
      osalDbgAssert(spip->rx.dma != NULL, "unable to allocate stream");
      spip->tx.dma = dmaStreamAllocI(STM32_SPI_SPI5_TX_DMA_STREAM,
                                     STM32_SPI_SPI5_IRQ_PRIORITY,
                                     (stm32_dmaisr_t)spi_lld_serve_dma_tx_interrupt,
                                     (void *)spip);
      osalDbgAssert(spip->tx.dma != NULL, "unable to allocate stream");
      rccResetSPI5();
      rccEnableSPI5(true);
      dmaSetRequestSource(spip->rx.dma, STM32_DMAMUX1_SPI5_RX);
      dmaSetRequestSource(spip->tx.dma, STM32_DMAMUX1_SPI5_TX);
    }
#endif
#if STM32_SPI_USE_SPI6
    if (&SPID6 == spip) {
      spip->rx.bdma = bdmaStreamAllocI(STM32_SPI_SPI6_RX_BDMA_STREAM,
                                      STM32_SPI_SPI6_IRQ_PRIORITY,
                                      (stm32_dmaisr_t)spi_lld_serve_bdma_rx_interrupt,
                                      (void *)spip);
      osalDbgAssert(spip->rx.bdma != NULL, "unable to allocate stream");
      spip->tx.bdma = bdmaStreamAllocI(STM32_SPI_SPI6_TX_BDMA_STREAM,
                                      STM32_SPI_SPI6_IRQ_PRIORITY,
                                      (stm32_dmaisr_t)spi_lld_serve_bdma_tx_interrupt,
                                      (void *)spip);
      osalDbgAssert(spip->tx.bdma != NULL, "unable to allocate stream");
      rccResetSPI6();
      rccEnableSPI6(true);
      bdmaSetRequestSource(spip->rx.bdma, STM32_DMAMUX2_SPI6_RX);
      bdmaSetRequestSource(spip->tx.bdma, STM32_DMAMUX2_SPI6_TX);
    }
#endif

    /* DMA setup.*/
#if defined(STM32_SPI_DMA_REQUIRED) && defined(STM32_SPI_BDMA_REQUIRED)
    if (spip->is_bdma)
#endif
#if defined(STM32_SPI_BDMA_REQUIRED)
    {
      bdmaStreamSetPeripheral(spip->rx.bdma, &spip->spi->RXDR);
      bdmaStreamSetPeripheral(spip->tx.bdma, &spip->spi->TXDR);
    }
#endif
#if defined(STM32_SPI_DMA_REQUIRED) && defined(STM32_SPI_BDMA_REQUIRED)
    else
#endif
#if defined(STM32_SPI_DMA_REQUIRED)
    {
      dmaStreamSetPeripheral(spip->rx.dma, &spip->spi->RXDR);
      dmaStreamSetPeripheral(spip->tx.dma, &spip->spi->TXDR);
    }
#endif
 }

  /* Configuration-specific DMA setup.*/
  dsize = (spip->config->cfg1 & SPI_CFG1_DSIZE_Msk) + 1U;
#if defined(STM32_SPI_DMA_REQUIRED) && defined(STM32_SPI_BDMA_REQUIRED)
  if (spip->is_bdma)
#endif
#if defined(STM32_SPI_BDMA_REQUIRED)
  {
    if (dsize <= 8U) {
      /* Frame width is between 4 and 8 bits.*/
      spip->rxdmamode = (spip->rxdmamode & ~STM32_BDMA_CR_SIZE_MASK) |
                        STM32_BDMA_CR_PSIZE_BYTE | STM32_BDMA_CR_MSIZE_BYTE;
      spip->txdmamode = (spip->txdmamode & ~STM32_BDMA_CR_SIZE_MASK) |
                        STM32_BDMA_CR_PSIZE_BYTE | STM32_BDMA_CR_MSIZE_BYTE;
    }
    else if (dsize <= 16U) {
      /* Frame width is between 9 and 16 bits.*/
      spip->rxdmamode = (spip->rxdmamode & ~STM32_BDMA_CR_SIZE_MASK) |
                        STM32_BDMA_CR_PSIZE_HWORD | STM32_BDMA_CR_MSIZE_HWORD;
      spip->txdmamode = (spip->txdmamode & ~STM32_BDMA_CR_SIZE_MASK) |
                        STM32_BDMA_CR_PSIZE_HWORD | STM32_BDMA_CR_MSIZE_HWORD;
    }
    else {
      /* Frame width is between 16 and 32 bits.*/
      spip->rxdmamode = (spip->rxdmamode & ~STM32_BDMA_CR_SIZE_MASK) |
                        STM32_BDMA_CR_PSIZE_WORD | STM32_BDMA_CR_MSIZE_WORD;
      spip->txdmamode = (spip->txdmamode & ~STM32_BDMA_CR_SIZE_MASK) |
                        STM32_BDMA_CR_PSIZE_WORD | STM32_BDMA_CR_MSIZE_WORD;
    }
    if (spip->config->circular) {
      spip->rxdmamode |= (STM32_BDMA_CR_CIRC | STM32_BDMA_CR_HTIE);
      spip->txdmamode |= (STM32_BDMA_CR_CIRC | STM32_BDMA_CR_HTIE);
    }
    else {
      spip->rxdmamode &= ~(STM32_BDMA_CR_CIRC | STM32_BDMA_CR_HTIE);
      spip->txdmamode &= ~(STM32_BDMA_CR_CIRC | STM32_BDMA_CR_HTIE);
    }
  }
#endif
#if defined(STM32_SPI_DMA_REQUIRED) && defined(STM32_SPI_BDMA_REQUIRED)
  else
#endif
#if defined(STM32_SPI_DMA_REQUIRED)
  {
    if (dsize <= 8U) {
      /* Frame width is between 4 and 8 bits.*/
      spip->rxdmamode = (spip->rxdmamode & ~STM32_DMA_CR_SIZE_MASK) |
                        STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MSIZE_BYTE;
      spip->txdmamode = (spip->txdmamode & ~STM32_DMA_CR_SIZE_MASK) |
                        STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MSIZE_BYTE;
    }
    else if (dsize <= 16U) {
      /* Frame width is between 9 and 16 bits.*/
      spip->rxdmamode = (spip->rxdmamode & ~STM32_DMA_CR_SIZE_MASK) |
                        STM32_DMA_CR_PSIZE_HWORD | STM32_DMA_CR_MSIZE_HWORD;
      spip->txdmamode = (spip->txdmamode & ~STM32_DMA_CR_SIZE_MASK) |
                        STM32_DMA_CR_PSIZE_HWORD | STM32_DMA_CR_MSIZE_HWORD;
    }
    else {
      /* Frame width is between 16 and 32 bits.*/
      spip->rxdmamode = (spip->rxdmamode & ~STM32_DMA_CR_SIZE_MASK) |
                        STM32_DMA_CR_PSIZE_WORD | STM32_DMA_CR_MSIZE_WORD;
      spip->txdmamode = (spip->txdmamode & ~STM32_DMA_CR_SIZE_MASK) |
                        STM32_DMA_CR_PSIZE_WORD | STM32_DMA_CR_MSIZE_WORD;
    }
    if (spip->config->circular) {
      spip->rxdmamode |= (STM32_DMA_CR_CIRC | STM32_DMA_CR_HTIE);
      spip->txdmamode |= (STM32_DMA_CR_CIRC | STM32_DMA_CR_HTIE);
    }
    else {
      spip->rxdmamode &= ~(STM32_DMA_CR_CIRC | STM32_DMA_CR_HTIE);
      spip->txdmamode &= ~(STM32_DMA_CR_CIRC | STM32_DMA_CR_HTIE);
    }
  }
#endif

  /* SPI setup and enable.*/
  spip->spi->CR1 &= ~SPI_CR1_SPE;
  spip->spi->CR1  = SPI_CR1_MASRX;
  spip->spi->CR2  = 0U;
  spip->spi->CFG1 = (spip->config->cfg1 & ~SPI_CFG1_FTHLV_Msk) |
                    SPI_CFG1_RXDMAEN | SPI_CFG1_TXDMAEN;
  spip->spi->CFG2 = (spip->config->cfg2 | SPI_CFG2_MASTER | SPI_CFG2_SSOE) &
                    ~SPI_CFG2_COMM_Msk;
  spip->spi->IER  = SPI_IER_OVRIE;
  spip->spi->IFCR = 0xFFFFFFFFU;
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
    spip->spi->CR1  = 0U;
    spip->spi->CR2  = 0U;
    spip->spi->CFG1 = 0U;
    spip->spi->CFG2 = 0U;
    spip->spi->IER  = 0U;
#if defined(STM32_SPI_DMA_REQUIRED) && defined(STM32_SPI_BDMA_REQUIRED)
    if (spip->is_bdma)
#endif
#if defined(STM32_SPI_BDMA_REQUIRED)
    {
      bdmaStreamFreeI(spip->rx.bdma);
      bdmaStreamFreeI(spip->tx.bdma);
    }
#endif
#if defined(STM32_SPI_DMA_REQUIRED) && defined(STM32_SPI_BDMA_REQUIRED)
    else
#endif
#if defined(STM32_SPI_DMA_REQUIRED)
    {
      dmaStreamFreeI(spip->rx.dma);
      dmaStreamFreeI(spip->tx.dma);
    }
#endif

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

  spi_lld_wait_complete(spip);

#if defined(STM32_SPI_DMA_REQUIRED) && defined(STM32_SPI_BDMA_REQUIRED)
  if (spip->is_bdma)
#endif
#if defined(STM32_SPI_BDMA_REQUIRED)
  {
	bdmaStreamSetMemory(spip->rx.bdma, spip->config->dummyrx?spip->config->dummyrx:&dummyrx);
    bdmaStreamSetTransactionSize(spip->rx.bdma, n);
    bdmaStreamSetMode(spip->rx.bdma, spip->rxdmamode);

    bdmaStreamSetMemory(spip->tx.bdma, spip->config->dummytx?spip->config->dummytx:&dummytx);
    bdmaStreamSetTransactionSize(spip->tx.bdma, n);
    bdmaStreamSetMode(spip->tx.bdma, spip->txdmamode);

    bdmaStreamEnable(spip->rx.bdma);
    bdmaStreamEnable(spip->tx.bdma);
  }
#endif
#if defined(STM32_SPI_DMA_REQUIRED) && defined(STM32_SPI_BDMA_REQUIRED)
  else
#endif
#if defined(STM32_SPI_DMA_REQUIRED)
  {
    dmaStreamSetMemory0(spip->rx.dma, spip->config->dummyrx?spip->config->dummyrx:&dummyrx);
    dmaStreamSetTransactionSize(spip->rx.dma, n);
    dmaStreamSetMode(spip->rx.dma, spip->rxdmamode);

    dmaStreamSetMemory0(spip->tx.dma, spip->config->dummytx?spip->config->dummytx:&dummytx);
    dmaStreamSetTransactionSize(spip->tx.dma, n);
    dmaStreamSetMode(spip->tx.dma, spip->txdmamode);

    dmaStreamEnable(spip->rx.dma);
    dmaStreamEnable(spip->tx.dma);
  }
#endif

  spip->spi->CR1 |= SPI_CR1_CSTART;
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

  spi_lld_wait_complete(spip);

#if defined(STM32_SPI_DMA_REQUIRED) && defined(STM32_SPI_BDMA_REQUIRED)
  if (spip->is_bdma)
#endif
#if defined(STM32_SPI_BDMA_REQUIRED)
  {
    bdmaStreamSetMemory(spip->rx.bdma, rxbuf);
    bdmaStreamSetTransactionSize(spip->rx.bdma, n);
    bdmaStreamSetMode(spip->rx.bdma, spip->rxdmamode | STM32_BDMA_CR_MINC);

    bdmaStreamSetMemory(spip->tx.bdma, txbuf);
    bdmaStreamSetTransactionSize(spip->tx.bdma, n);
    bdmaStreamSetMode(spip->tx.bdma, spip->txdmamode | STM32_BDMA_CR_MINC);

    bdmaStreamEnable(spip->rx.bdma);
    bdmaStreamEnable(spip->tx.bdma);
  }
#endif
#if defined(STM32_SPI_DMA_REQUIRED) && defined(STM32_SPI_BDMA_REQUIRED)
  else
#endif
#if defined(STM32_SPI_DMA_REQUIRED)
  {
    dmaStreamSetMemory0(spip->rx.dma, rxbuf);
    dmaStreamSetTransactionSize(spip->rx.dma, n);
    dmaStreamSetMode(spip->rx.dma, spip->rxdmamode | STM32_DMA_CR_MINC);

    dmaStreamSetMemory0(spip->tx.dma, txbuf);
    dmaStreamSetTransactionSize(spip->tx.dma, n);
    dmaStreamSetMode(spip->tx.dma, spip->txdmamode | STM32_DMA_CR_MINC);

    dmaStreamEnable(spip->rx.dma);
    dmaStreamEnable(spip->tx.dma);
  }
#endif

  spip->spi->CR1 |= SPI_CR1_CSTART;
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

  spi_lld_wait_complete(spip);

#if defined(STM32_SPI_DMA_REQUIRED) && defined(STM32_SPI_BDMA_REQUIRED)
  if (spip->is_bdma)
#endif
#if defined(STM32_SPI_BDMA_REQUIRED)
  {
    bdmaStreamSetMemory(spip->rx.bdma, spip->config->dummyrx?spip->config->dummyrx:&dummyrx);
    bdmaStreamSetTransactionSize(spip->rx.bdma, n);
    bdmaStreamSetMode(spip->rx.bdma, spip->rxdmamode);

    bdmaStreamSetMemory(spip->tx.bdma, txbuf);
    bdmaStreamSetTransactionSize(spip->tx.bdma, n);
    bdmaStreamSetMode(spip->tx.bdma, spip->txdmamode | STM32_BDMA_CR_MINC);

    bdmaStreamEnable(spip->rx.bdma);
    bdmaStreamEnable(spip->tx.bdma);
  }
#endif
#if defined(STM32_SPI_DMA_REQUIRED) && defined(STM32_SPI_BDMA_REQUIRED)
  else
#endif
#if defined(STM32_SPI_DMA_REQUIRED)
  {
	dmaStreamSetMemory0(spip->rx.dma, spip->config->dummyrx?spip->config->dummyrx:&dummyrx);
    dmaStreamSetTransactionSize(spip->rx.dma, n);
    dmaStreamSetMode(spip->rx.dma, spip->rxdmamode);

    dmaStreamSetMemory0(spip->tx.dma, txbuf);
    dmaStreamSetTransactionSize(spip->tx.dma, n);
    dmaStreamSetMode(spip->tx.dma, spip->txdmamode | STM32_DMA_CR_MINC);

    dmaStreamEnable(spip->rx.dma);
    dmaStreamEnable(spip->tx.dma);
  }
#endif

  spip->spi->CR1 |= SPI_CR1_CSTART;
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

  spi_lld_wait_complete(spip);

#if defined(STM32_SPI_DMA_REQUIRED) && defined(STM32_SPI_BDMA_REQUIRED)
  if (spip->is_bdma)
#endif
#if defined(STM32_SPI_BDMA_REQUIRED)
  {
    bdmaStreamSetMemory(spip->rx.bdma, rxbuf);
    bdmaStreamSetTransactionSize(spip->rx.bdma, n);
    bdmaStreamSetMode(spip->rx.bdma, spip->rxdmamode | STM32_BDMA_CR_MINC);

    bdmaStreamSetMemory(spip->tx.bdma, spip->config->dummytx?spip->config->dummytx:&dummytx);
    bdmaStreamSetTransactionSize(spip->tx.bdma, n);
    bdmaStreamSetMode(spip->tx.bdma, spip->txdmamode);

    bdmaStreamEnable(spip->rx.bdma);
    bdmaStreamEnable(spip->tx.bdma);
  }
#endif
#if defined(STM32_SPI_DMA_REQUIRED) && defined(STM32_SPI_BDMA_REQUIRED)
  else
#endif
#if defined(STM32_SPI_DMA_REQUIRED)
  {
    dmaStreamSetMemory0(spip->rx.dma, rxbuf);
    dmaStreamSetTransactionSize(spip->rx.dma, n);
    dmaStreamSetMode(spip->rx.dma, spip->rxdmamode | STM32_DMA_CR_MINC);

    dmaStreamSetMemory0(spip->tx.dma, spip->config->dummytx?spip->config->dummytx:&dummytx);
    dmaStreamSetTransactionSize(spip->tx.dma, n);
    dmaStreamSetMode(spip->tx.dma, spip->txdmamode);

    dmaStreamEnable(spip->rx.dma);
    dmaStreamEnable(spip->tx.dma);
  }
#endif

  spip->spi->CR1 |= SPI_CR1_CSTART;
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

  /* Stopping SPI.*/
  spip->spi->CR1 |= SPI_CR1_CSUSP;

  spi_lld_wait_complete(spip);

  /* Stopping DMAs.*/
#if defined(STM32_SPI_DMA_REQUIRED) && defined(STM32_SPI_BDMA_REQUIRED)
  if (spip->is_bdma)
#endif
#if defined(STM32_SPI_BDMA_REQUIRED)
  {
    bdmaStreamDisable(spip->tx.bdma);
    bdmaStreamDisable(spip->rx.bdma);
  }
#endif
#if defined(STM32_SPI_DMA_REQUIRED) && defined(STM32_SPI_BDMA_REQUIRED)
  else
#endif
#if defined(STM32_SPI_DMA_REQUIRED)
  {
    dmaStreamDisable(spip->tx.dma);
    dmaStreamDisable(spip->rx.dma);
  }
#endif
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
 *
 * @notapi
 */
uint32_t spi_lld_polled_exchange(SPIDriver *spip, uint32_t frame) {
  uint32_t dsize = (spip->spi->CFG1 & SPI_CFG1_DSIZE_Msk) + 1U;
  uint32_t rxframe;

  spi_lld_wait_complete(spip);

  spip->spi->CR1 |= SPI_CR1_CSTART;

  /* wait for room in TX FIFO.*/
  while ((spip->spi->SR & SPI_SR_TXP) == 0U)
    ;

  /* Data register must be accessed with the appropriate data size.
     Byte size access (uint8_t *) for transactions that are <= 8-bit etc.*/
  if (dsize <= 8U) {
    /* Frame width is between 4 and 8 bits.*/
    volatile uint8_t *txdrp8 = (volatile uint8_t *)&spip->spi->TXDR;
    volatile uint8_t *rxdrp8 = (volatile uint8_t *)&spip->spi->RXDR;
    *txdrp8 = (uint8_t)frame;
    while ((spip->spi->SR & SPI_SR_RXP) == 0U)
      ;
    rxframe = (uint32_t)*rxdrp8;
  }
  else if (dsize <= 16U) {
    /* Frame width is between 9 and 16 bits.*/
    volatile uint16_t *txdrp16 = (volatile uint16_t *)&spip->spi->TXDR;
    volatile uint16_t *rxdrp16 = (volatile uint16_t *)&spip->spi->RXDR;
    *txdrp16 = (uint16_t)frame;
    while ((spip->spi->SR & SPI_SR_RXP) == 0U)
      ;
    rxframe = (uint32_t)*rxdrp16;
  }
  else {
    /* Frame width is between 16 and 32 bits.*/
    spip->spi->TXDR = frame;
    while ((spip->spi->SR & SPI_SR_RXP) == 0U)
      ;
    rxframe = spip->spi->RXDR;
  }

  spip->spi->CR1 |= SPI_CR1_CSUSP;

  return rxframe;
}

#endif /* HAL_USE_SPI */

/** @} */
