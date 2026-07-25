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
 * @file    SPIv2/hal_i2s_lld.c
 * @brief   STM32 I2S subsystem low level driver source.
 *
 * @addtogroup I2S
 * @{
 */

#include "hal.h"

#if HAL_USE_I2S || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define I2S1_RX_DMA_CHANNEL                                                 \
  STM32_DMA_GETCHANNEL(STM32_I2S_SPI1_RX_DMA_STREAM,                        \
                       STM32_SPI1_RX_DMA_CHN)

#define I2S1_TX_DMA_CHANNEL                                                 \
  STM32_DMA_GETCHANNEL(STM32_I2S_SPI1_TX_DMA_STREAM,                        \
                       STM32_SPI1_TX_DMA_CHN)

#define I2S2_RX_DMA_CHANNEL                                                 \
  STM32_DMA_GETCHANNEL(STM32_I2S_SPI2_RX_DMA_STREAM,                        \
                       STM32_SPI2_RX_DMA_CHN)

#define I2S2_TX_DMA_CHANNEL                                                 \
  STM32_DMA_GETCHANNEL(STM32_I2S_SPI2_TX_DMA_STREAM,                        \
                       STM32_SPI2_TX_DMA_CHN)

#define I2S3_RX_DMA_CHANNEL                                                 \
  STM32_DMA_GETCHANNEL(STM32_I2S_SPI3_RX_DMA_STREAM,                        \
                       STM32_SPI3_RX_DMA_CHN)

#define I2S3_TX_DMA_CHANNEL                                                 \
  STM32_DMA_GETCHANNEL(STM32_I2S_SPI3_TX_DMA_STREAM,                        \
                       STM32_SPI3_TX_DMA_CHN)

/*
 * Static I2S settings for I2S1.
 */
#if !STM32_I2S_IS_MASTER(STM32_I2S_SPI1_MODE)
#if STM32_I2S_TX_ENABLED(STM32_I2S_SPI1_MODE)
#define STM32_I2S1_CFGR_CFG                 0
#endif
#if STM32_I2S_RX_ENABLED(STM32_I2S_SPI1_MODE)
#define STM32_I2S1_CFGR_CFG                 SPI_I2SCFGR_I2SCFG_0
#endif
#else /* !STM32_I2S_IS_MASTER(STM32_I2S_SPI1_MODE) */
#if STM32_I2S_TX_ENABLED(STM32_I2S_SPI1_MODE)
#define STM32_I2S1_CFGR_CFG                 SPI_I2SCFGR_I2SCFG_1
#endif
#if STM32_I2S_RX_ENABLED(STM32_I2S_SPI1_MODE)
#define STM32_I2S1_CFGR_CFG                 (SPI_I2SCFGR_I2SCFG_1 |         \
                                             SPI_I2SCFGR_I2SCFG_0)
#endif
#endif /* !STM32_I2S_IS_MASTER(STM32_I2S_SPI1_MODE) */

/*
 * Static I2S settings for I2S2.
 */
#if !STM32_I2S_IS_MASTER(STM32_I2S_SPI2_MODE)
#if STM32_I2S_TX_ENABLED(STM32_I2S_SPI2_MODE)
#define STM32_I2S2_CFGR_CFG                 0
#endif
#if STM32_I2S_RX_ENABLED(STM32_I2S_SPI2_MODE)
#define STM32_I2S2_CFGR_CFG                 SPI_I2SCFGR_I2SCFG_0
#endif
#else /* !STM32_I2S_IS_MASTER(STM32_I2S_SPI2_MODE) */
#if STM32_I2S_TX_ENABLED(STM32_I2S_SPI2_MODE)
#define STM32_I2S2_CFGR_CFG                 SPI_I2SCFGR_I2SCFG_1
#endif
#if STM32_I2S_RX_ENABLED(STM32_I2S_SPI2_MODE)
#define STM32_I2S2_CFGR_CFG                 (SPI_I2SCFGR_I2SCFG_1 |         \
                                             SPI_I2SCFGR_I2SCFG_0)
#endif
#endif /* !STM32_I2S_IS_MASTER(STM32_I2S_SPI2_MODE) */

/*
 * Static I2S settings for I2S3.
 */
#if !STM32_I2S_IS_MASTER(STM32_I2S_SPI3_MODE)
#if STM32_I2S_TX_ENABLED(STM32_I2S_SPI3_MODE)
#define STM32_I2S3_CFGR_CFG                 0
#endif
#if STM32_I2S_RX_ENABLED(STM32_I2S_SPI3_MODE)
#define STM32_I2S3_CFGR_CFG                 SPI_I2SCFGR_I2SCFG_0
#endif
#else /* !STM32_I2S_IS_MASTER(STM32_I2S_SPI3_MODE) */
#if STM32_I2S_TX_ENABLED(STM32_I2S_SPI3_MODE)
#define STM32_I2S3_CFGR_CFG                 SPI_I2SCFGR_I2SCFG_1
#endif
#if STM32_I2S_RX_ENABLED(STM32_I2S_SPI3_MODE)
#define STM32_I2S3_CFGR_CFG                 (SPI_I2SCFGR_I2SCFG_1 |         \
                                             SPI_I2SCFGR_I2SCFG_0)
#endif
#endif /* !STM32_I2S_IS_MASTER(STM32_I2S_SPI3_MODE) */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief I2S1 driver identifier.*/
#if STM32_I2S_USE_SPI1 || defined(__DOXYGEN__)
I2SDriver I2SD1;
#endif

/** @brief I2S2 driver identifier.*/
#if STM32_I2S_USE_SPI2 || defined(__DOXYGEN__)
I2SDriver I2SD2;
#endif

/** @brief I2S3 driver identifier.*/
#if STM32_I2S_USE_SPI3 || defined(__DOXYGEN__)
I2SDriver I2SD3;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

#if STM32_I2S_RX_ENABLED(STM32_I2S_SPI1_MODE) ||                            \
    STM32_I2S_RX_ENABLED(STM32_I2S_SPI2_MODE) ||                            \
    STM32_I2S_RX_ENABLED(STM32_I2S_SPI3_MODE) || defined(__DOXYGEN__)
/**
 * @brief   Shared end-of-rx service routine.
 *
 * @param[in] i2sp      pointer to the @p I2SDriver object
 * @param[in] flags     pre-shifted content of the ISR register
 */
static void i2s_lld_serve_rx_interrupt(I2SDriver *i2sp, uint32_t flags) {

  (void)i2sp;

  /* DMA errors handling.*/
#if defined(STM32_I2S_DMA_ERROR_HOOK)
  if ((flags & (STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF)) != 0) {
    STM32_I2S_DMA_ERROR_HOOK(i2sp);
  }
#endif

  /* Callbacks handling, note it is portable code defined in the high
     level driver.*/
  if ((flags & STM32_DMA_ISR_TCIF) != 0) {
    /* Transfer complete processing.*/
    _i2s_isr_full_code(i2sp);
  }
  else if ((flags & STM32_DMA_ISR_HTIF) != 0) {
    /* Half transfer processing.*/
    _i2s_isr_half_code(i2sp);
  }
}
#endif

#if STM32_I2S_TX_ENABLED(STM32_I2S_SPI1_MODE) ||                            \
    STM32_I2S_TX_ENABLED(STM32_I2S_SPI2_MODE) ||                            \
    STM32_I2S_TX_ENABLED(STM32_I2S_SPI3_MODE) || defined(__DOXYGEN__)
/**
 * @brief   Shared end-of-tx service routine.
 *
 * @param[in] i2sp      pointer to the @p I2SDriver object
 * @param[in] flags     pre-shifted content of the ISR register
 */
static void i2s_lld_serve_tx_interrupt(I2SDriver *i2sp, uint32_t flags) {

  (void)i2sp;

  /* DMA errors handling.*/
#if defined(STM32_I2S_DMA_ERROR_HOOK)
  if ((flags & (STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF)) != 0) {
    STM32_I2S_DMA_ERROR_HOOK(i2sp);
  }
#endif

  /* Callbacks handling, note it is portable code defined in the high
     level driver.*/
  if ((flags & STM32_DMA_ISR_TCIF) != 0) {
    /* Transfer complete processing.*/
    _i2s_isr_full_code(i2sp);
  }
  else if ((flags & STM32_DMA_ISR_HTIF) != 0) {
    /* Half transfer processing.*/
    _i2s_isr_half_code(i2sp);
  }
}
#endif

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level I2S driver initialization.
 *
 * @notapi
 */
void i2s_lld_init(void) {

#if STM32_I2S_USE_SPI1
  i2sObjectInit(&I2SD1);
  I2SD1.spi       = SPI1;
  I2SD1.cfg       = STM32_I2S1_CFGR_CFG;
  I2SD1.dmarx     = NULL;
  I2SD1.dmatx     = NULL;
#if STM32_I2S_RX_ENABLED(STM32_I2S_SPI1_MODE)
  I2SD1.rxdmamode = STM32_DMA_CR_CHSEL(I2S1_RX_DMA_CHANNEL) |
                    STM32_DMA_CR_PL(STM32_I2S_SPI1_DMA_PRIORITY) |
                    STM32_DMA_CR_PSIZE_HWORD |
                    STM32_DMA_CR_MSIZE_HWORD |
                    STM32_DMA_CR_DIR_P2M |
                    STM32_DMA_CR_MINC |
                    STM32_DMA_CR_CIRC |
                    STM32_DMA_CR_HTIE |
                    STM32_DMA_CR_TCIE |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
#else
  I2SD1.rxdmamode = 0;
#endif
#if STM32_I2S_TX_ENABLED(STM32_I2S_SPI1_MODE)
  I2SD1.txdmamode = STM32_DMA_CR_CHSEL(I2S1_TX_DMA_CHANNEL) |
                    STM32_DMA_CR_PL(STM32_I2S_SPI1_DMA_PRIORITY) |
                    STM32_DMA_CR_PSIZE_HWORD |
                    STM32_DMA_CR_MSIZE_HWORD |
                    STM32_DMA_CR_DIR_M2P |
                    STM32_DMA_CR_MINC |
                    STM32_DMA_CR_CIRC |
                    STM32_DMA_CR_HTIE |
                    STM32_DMA_CR_TCIE |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
#else
  I2SD1.txdmamode = 0;
#endif
#endif

#if STM32_I2S_USE_SPI2
  i2sObjectInit(&I2SD2);
  I2SD2.spi       = SPI2;
  I2SD2.cfg       = STM32_I2S2_CFGR_CFG;
  I2SD2.dmarx     = NULL;
  I2SD2.dmatx     = NULL;
#if STM32_I2S_RX_ENABLED(STM32_I2S_SPI2_MODE)
  I2SD2.rxdmamode = STM32_DMA_CR_CHSEL(I2S2_RX_DMA_CHANNEL) |
                    STM32_DMA_CR_PL(STM32_I2S_SPI2_DMA_PRIORITY) |
                    STM32_DMA_CR_PSIZE_HWORD |
                    STM32_DMA_CR_MSIZE_HWORD |
                    STM32_DMA_CR_DIR_P2M |
                    STM32_DMA_CR_MINC |
                    STM32_DMA_CR_CIRC |
                    STM32_DMA_CR_HTIE |
                    STM32_DMA_CR_TCIE |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
#else
  I2SD2.rxdmamode = 0;
#endif
#if STM32_I2S_TX_ENABLED(STM32_I2S_SPI2_MODE)
  I2SD2.txdmamode = STM32_DMA_CR_CHSEL(I2S2_TX_DMA_CHANNEL) |
                    STM32_DMA_CR_PL(STM32_I2S_SPI2_DMA_PRIORITY) |
                    STM32_DMA_CR_PSIZE_HWORD |
                    STM32_DMA_CR_MSIZE_HWORD |
                    STM32_DMA_CR_DIR_M2P |
                    STM32_DMA_CR_MINC |
                    STM32_DMA_CR_CIRC |
                    STM32_DMA_CR_HTIE |
                    STM32_DMA_CR_TCIE |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
#else
  I2SD2.txdmamode = 0;
#endif
#endif

#if STM32_I2S_USE_SPI3
  i2sObjectInit(&I2SD3);
  I2SD3.spi       = SPI3;
  I2SD3.cfg       = STM32_I2S3_CFGR_CFG;
  I2SD3.dmarx     = NULL;
  I2SD3.dmatx     = NULL;
#if STM32_I2S_RX_ENABLED(STM32_I2S_SPI3_MODE)
  I2SD3.rxdmamode = STM32_DMA_CR_CHSEL(I2S3_RX_DMA_CHANNEL) |
                    STM32_DMA_CR_PL(STM32_I2S_SPI3_DMA_PRIORITY) |
                    STM32_DMA_CR_PSIZE_HWORD |
                    STM32_DMA_CR_MSIZE_HWORD |
                    STM32_DMA_CR_DIR_P2M |
                    STM32_DMA_CR_MINC |
                    STM32_DMA_CR_CIRC |
                    STM32_DMA_CR_HTIE |
                    STM32_DMA_CR_TCIE |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
#else
  I2SD3.rxdmamode = 0;
#endif
#if STM32_I2S_TX_ENABLED(STM32_I2S_SPI3_MODE)
  I2SD3.txdmamode = STM32_DMA_CR_CHSEL(I2S3_TX_DMA_CHANNEL) |
                    STM32_DMA_CR_PL(STM32_I2S_SPI3_DMA_PRIORITY) |
                    STM32_DMA_CR_PSIZE_HWORD |
                    STM32_DMA_CR_MSIZE_HWORD |
                    STM32_DMA_CR_DIR_M2P |
                    STM32_DMA_CR_MINC |
                    STM32_DMA_CR_CIRC |
                    STM32_DMA_CR_HTIE |
                    STM32_DMA_CR_TCIE |
                    STM32_DMA_CR_DMEIE |
                    STM32_DMA_CR_TEIE;
#else
  I2SD3.txdmamode = 0;
#endif
#endif
}

/**
 * @brief   Configures and activates the I2S peripheral.
 *
 * @param[in] i2sp      pointer to the @p I2SDriver object
 *
 * @notapi
 */
void i2s_lld_start(I2SDriver *i2sp) {

  /* If in stopped state then enables the SPI and DMA clocks.*/
  if (i2sp->state == I2S_STOP) {

#if STM32_I2S_USE_SPI1
    if (&I2SD1 == i2sp) {

      /* Enabling I2S unit clock.*/
      rccEnableSPI1(true);

#if STM32_I2S_RX_ENABLED(STM32_I2S_SPI1_MODE)
      i2sp->dmarx = dmaStreamAllocI(STM32_I2S_SPI1_RX_DMA_STREAM,
                                    STM32_I2S_SPI1_IRQ_PRIORITY,
                                    (stm32_dmaisr_t)i2s_lld_serve_rx_interrupt,
                                    (void *)i2sp);
      osalDbgAssert(i2sp->dmarx != NULL, "unable to allocate stream");

      /* CRs settings are done here because those never changes until
         the driver is stopped.*/
      i2sp->spi->CR1 = 0;
      i2sp->spi->CR2 = SPI_CR2_RXDMAEN;
#if STM32_DMA_SUPPORTS_DMAMUX
      dmaSetRequestSource(i2sp->dmarx, STM32_DMAMUX1_SPI1_RX);
#endif
#endif
#if STM32_I2S_TX_ENABLED(STM32_I2S_SPI1_MODE)
      i2sp->dmatx = dmaStreamAllocI(STM32_I2S_SPI1_TX_DMA_STREAM,
                                    STM32_I2S_SPI1_IRQ_PRIORITY,
                                    (stm32_dmaisr_t)i2s_lld_serve_tx_interrupt,
                                    (void *)i2sp);
      osalDbgAssert(i2sp->dmatx != NULL, "unable to allocate stream");

      /* CRs settings are done here because those never changes until
         the driver is stopped.*/
      i2sp->spi->CR1 = 0;
      i2sp->spi->CR2 = SPI_CR2_TXDMAEN;
#if STM32_DMA_SUPPORTS_DMAMUX
      dmaSetRequestSource(i2sp->dmatx, STM32_DMAMUX1_SPI1_TX);
#endif
#endif
    }
#endif

#if STM32_I2S_USE_SPI2
    if (&I2SD2 == i2sp) {

      /* Enabling I2S unit clock.*/
      rccEnableSPI2(true);

#if STM32_I2S_RX_ENABLED(STM32_I2S_SPI2_MODE)
      i2sp->dmarx = dmaStreamAllocI(STM32_I2S_SPI2_RX_DMA_STREAM,
                                    STM32_I2S_SPI2_IRQ_PRIORITY,
                                    (stm32_dmaisr_t)i2s_lld_serve_rx_interrupt,
                                    (void *)i2sp);
      osalDbgAssert(i2sp->dmarx != NULL, "unable to allocate stream");

      /* CRs settings are done here because those never changes until
         the driver is stopped.*/
      i2sp->spi->CR1 = 0;
      i2sp->spi->CR2 = SPI_CR2_RXDMAEN;
#if STM32_DMA_SUPPORTS_DMAMUX
      dmaSetRequestSource(i2sp->dmarx, STM32_DMAMUX1_SPI2_RX);
#endif
#endif
#if STM32_I2S_TX_ENABLED(STM32_I2S_SPI2_MODE)
      i2sp->dmatx = dmaStreamAllocI(STM32_I2S_SPI2_TX_DMA_STREAM,
                                    STM32_I2S_SPI2_IRQ_PRIORITY,
                                    (stm32_dmaisr_t)i2s_lld_serve_tx_interrupt,
                                    (void *)i2sp);
      osalDbgAssert(i2sp->dmatx != NULL, "unable to allocate stream");

      /* CRs settings are done here because those never changes until
         the driver is stopped.*/
      i2sp->spi->CR1 = 0;
      i2sp->spi->CR2 = SPI_CR2_TXDMAEN;
#if STM32_DMA_SUPPORTS_DMAMUX
      dmaSetRequestSource(i2sp->dmatx, STM32_DMAMUX1_SPI2_TX);
#endif
#endif
    }
#endif

#if STM32_I2S_USE_SPI3
    if (&I2SD3 == i2sp) {

      /* Enabling I2S unit clock.*/
      rccEnableSPI3(true);

#if STM32_I2S_RX_ENABLED(STM32_I2S_SPI3_MODE)
      i2sp->dmarx = dmaStreamAllocI(STM32_I2S_SPI3_RX_DMA_STREAM,
                                    STM32_I2S_SPI3_IRQ_PRIORITY,
                                    (stm32_dmaisr_t)i2s_lld_serve_rx_interrupt,
                                    (void *)i2sp);
      osalDbgAssert(i2sp->dmarx != NULL, "unable to allocate stream");

      /* CRs settings are done here because those never changes until
         the driver is stopped.*/
      i2sp->spi->CR1 = 0;
      i2sp->spi->CR2 = SPI_CR2_RXDMAEN;
#endif
#if STM32_I2S_TX_ENABLED(STM32_I2S_SPI3_MODE)
      i2sp->dmatx = dmaStreamAllocI(STM32_I2S_SPI3_TX_DMA_STREAM,
                                    STM32_I2S_SPI3_IRQ_PRIORITY,
                                    (stm32_dmaisr_t)i2s_lld_serve_tx_interrupt,
                                    (void *)i2sp);
      osalDbgAssert(i2sp->dmatx != NULL, "unable to allocate stream");

      /* CRs settings are done here because those never changes until
         the driver is stopped.*/
      i2sp->spi->CR1 = 0;
      i2sp->spi->CR2 = SPI_CR2_TXDMAEN;
#endif
    }
#endif
  }

  /* I2S (re)configuration.*/
  i2sp->spi->I2SPR   = i2sp->config->i2spr;
  i2sp->spi->I2SCFGR = i2sp->config->i2scfgr | i2sp->cfg | SPI_I2SCFGR_I2SMOD;
}

/**
 * @brief   Deactivates the I2S peripheral.
 *
 * @param[in] i2sp      pointer to the @p I2SDriver object
 *
 * @notapi
 */
void i2s_lld_stop(I2SDriver *i2sp) {

  /* If in ready state then disables the SPI clock.*/
  if (i2sp->state == I2S_READY) {

    /* SPI disable.*/
    i2sp->spi->CR2 = 0;
    if (NULL != i2sp->dmarx) {
      dmaStreamFreeI(i2sp->dmarx);
      i2sp->dmarx = NULL;
    }
    if (NULL != i2sp->dmatx) {
      dmaStreamFreeI(i2sp->dmatx);
      i2sp->dmatx = NULL;
    }

#if STM32_I2S_USE_SPI1
    if (&I2SD1 == i2sp)
      rccDisableSPI1();
#endif

#if STM32_I2S_USE_SPI2
    if (&I2SD2 == i2sp)
      rccDisableSPI2();
#endif

#if STM32_I2S_USE_SPI3
    if (&I2SD3 == i2sp)
      rccDisableSPI3();
#endif
  }
}

/**
 * @brief   Starts a I2S data exchange.
 *
 * @param[in] i2sp      pointer to the @p I2SDriver object
 *
 * @notapi
 */
void i2s_lld_start_exchange(I2SDriver *i2sp) {
  size_t size = i2sp->config->size;

  /* In 32 bit modes the DMA has to perform double operations because fetches
     are always performed using 16 bit accesses.
     DATLEN   CHLEN   SIZE
     00 (16)  0 (16)  16
     00 (16)  1 (32)  16
     01 (24)  X       32
     10 (32)  X       32
     11 (NA)  X       NA
     */
  if ((i2sp->config->i2scfgr & SPI_I2SCFGR_DATLEN) != 0)
    size *= 2;

  /* RX DMA setup.*/
  if (NULL != i2sp->dmarx) {
    dmaStreamSetMode(i2sp->dmarx, i2sp->rxdmamode);
    dmaStreamSetPeripheral(i2sp->dmarx, &i2sp->spi->DR);
    dmaStreamSetMemory0(i2sp->dmarx, i2sp->config->rx_buffer);
    dmaStreamSetTransactionSize(i2sp->dmarx, size);
    dmaStreamEnable(i2sp->dmarx);
  }

  /* TX DMA setup.*/
  if (NULL != i2sp->dmatx) {
    dmaStreamSetMode(i2sp->dmatx, i2sp->txdmamode);
    dmaStreamSetPeripheral(i2sp->dmatx, &i2sp->spi->DR);
    dmaStreamSetMemory0(i2sp->dmatx, i2sp->config->tx_buffer);
    dmaStreamSetTransactionSize(i2sp->dmatx, size);
    dmaStreamEnable(i2sp->dmatx);
  }

  /* Starting transfer.*/
  i2sp->spi->I2SCFGR |= SPI_I2SCFGR_I2SE;
}

/**
 * @brief   Stops the ongoing data exchange.
 * @details The ongoing data exchange, if any, is stopped, if the driver
 *          was not active the function does nothing.
 *
 * @param[in] i2sp      pointer to the @p I2SDriver object
 *
 * @notapi
 */
void i2s_lld_stop_exchange(I2SDriver *i2sp) {

  /* Stop TX DMA, if enabled.*/
  if (NULL != i2sp->dmatx) {
    dmaStreamDisable(i2sp->dmatx);

    /* From the RM: To switch off the I2S, by clearing I2SE, it is mandatory
       to wait for TXE = 1 and BSY = 0.*/
    while ((i2sp->spi->SR & (SPI_SR_TXE | SPI_SR_BSY)) != SPI_SR_TXE)
      ;
  }

  /* Stop SPI/I2S peripheral.*/
  i2sp->spi->I2SCFGR &= ~SPI_I2SCFGR_I2SE;

  /* Stop RX DMA, if enabled.*/
  if (NULL != i2sp->dmarx)
    dmaStreamDisable(i2sp->dmarx);
}

#endif /* HAL_USE_I2S */

/** @} */
