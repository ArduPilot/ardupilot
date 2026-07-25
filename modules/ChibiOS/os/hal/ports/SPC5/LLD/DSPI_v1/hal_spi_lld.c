/*
    SPC5 HAL - Copyright (C) 2013 STMicroelectronics

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
 * @file    SPC5xx/DSPI_v1/hal_spi_lld.c
 * @brief   SPC5xx SPI subsystem low level driver source.
 *
 * @addtogroup SPI
 * @{
 */

#include "hal.h"

#if HAL_USE_SPI || defined(__DOXYGEN__)

/* Some forward declarations.*/
#if SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE
static void spi_serve_rx_dma_irq(edma_channel_t channel, void *p);
#endif

#if SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_AND_TX
static void spi_serve_tx_dma_irq(edma_channel_t channel, void *p);
#endif

#if SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE
static void spi_serve_dma_error_irq(edma_channel_t channel,
                                    void *p,
                                    uint32_t esr);
#endif

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/* Excluded PUSHR bits.*/
#define DSPI_PUSHR_EXCLUDED_BITS    (SPC5_PUSHR_CTAS_MASK   |               \
                                     SPC5_PUSHR_EOQ         |               \
                                     SPC5_PUSHR_TXDATA_MASK)

#define DSPI_POPR8_ADDRESS(spip)    (((uint32_t)&(spip)->dspi->POPR.R) + 3)
#define DSPI_POPR16_ADDRESS(spip)   (((uint32_t)&(spip)->dspi->POPR.R) + 2)

/* Set of macros dealing with the variable number of DMAs depending on
   the chosen mode.*/
#if SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_AND_TX
#define spi_lld_setdma(spip, tx1_cfg, tx2_cfg, rx_cfg) {                    \
  (spip)->tx1_channel = edmaChannelAllocate(&(tx1_cfg));                    \
  (spip)->tx2_channel = edmaChannelAllocate(&(tx2_cfg));                    \
  (spip)->rx_channel = edmaChannelAllocate(&(rx_cfg));                      \
}
#endif

#if SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_ONLY
#define spi_lld_setdma(spip, tx1_cfg, tx2_cfg, rx_cfg) {                    \
  (spip)->rx_channel = edmaChannelAllocate(&(rx_cfg));                      \
}
#endif

#if SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_NONE
#define spi_lld_setdma(spip, tx1_cfg, tx2_cfg, rx_cfg) {                    \
}
#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   SPID1 driver identifier.
 */
#if SPC5_SPI_USE_DSPI0 || defined(__DOXYGEN__)
SPIDriver SPID1;
#endif

/**
 * @brief   SPID2 driver identifier.
 */
#if SPC5_SPI_USE_DSPI1 || defined(__DOXYGEN__)
SPIDriver SPID2;
#endif

/**
 * @brief   SPID3 driver identifier.
 */
#if SPC5_SPI_USE_DSPI2 || defined(__DOXYGEN__)
SPIDriver SPID3;
#endif

/**
 * @brief   SPID4 driver identifier.
 */
#if SPC5_SPI_USE_DSPI3 || defined(__DOXYGEN__)
SPIDriver SPID4;
#endif

/**
 * @brief   SPID5 driver identifier.
 */
#if SPC5_SPI_USE_DSPI4 || defined(__DOXYGEN__)
SPIDriver SPID5;
#endif

/**
 * @brief   SPID6 driver identifier.
 */
#if SPC5_SPI_USE_DSPI5 || defined(__DOXYGEN__)
SPIDriver SPID6;
#endif

/**
 * @brief   SPID7 driver identifier.
 */
#if SPC5_SPI_USE_DSPI6 || defined(__DOXYGEN__)
SPIDriver SPID7;
#endif

/**
 * @brief   SPID8 driver identifier.
 */
#if SPC5_SPI_USE_DSPI7 || defined(__DOXYGEN__)
SPIDriver SPID8;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

#if SPC5_SPI_USE_DSPI0 || defined(__DOXYGEN__)
#if (SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_AND_TX) || defined(__DOXYGEN__)
/**
 * @brief   DMA configuration for DSPI0 TX1.
 */
static const edma_channel_config_t spi_dspi0_tx1_dma_config = {
  SPC5_SPI_DSPI0_TX1_DMA_CH_ID,
#if SPC5_EDMA_HAS_MUX
  SPC5_DSPI0_TX1_DMA_DEV_ID,
#endif
  SPC5_SPI_DSPI0_DMA_IRQ_PRIO,
  spi_serve_tx_dma_irq, spi_serve_dma_error_irq, &SPID1
};

/**
 * @brief   DMA configuration for DSPI0 TX2.
 */
static const edma_channel_config_t spi_dspi0_tx2_dma_config = {
  SPC5_SPI_DSPI0_TX2_DMA_CH_ID,
#if SPC5_EDMA_HAS_MUX
  0,
#endif
  SPC5_SPI_DSPI0_DMA_IRQ_PRIO,
  spi_serve_tx_dma_irq, spi_serve_dma_error_irq, &SPID1
};
#endif /* SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_AND_TX */

#if (SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE) || defined(__DOXYGEN__)
/**
 * @brief   DMA configuration for DSPI0 RX.
 */
static const edma_channel_config_t spi_dspi0_rx_dma_config = {
  SPC5_SPI_DSPI0_RX_DMA_CH_ID,
#if SPC5_EDMA_HAS_MUX
  SPC5_DSPI0_RX_DMA_DEV_ID,
#endif
  SPC5_SPI_DSPI0_DMA_IRQ_PRIO,
  spi_serve_rx_dma_irq, spi_serve_dma_error_irq, &SPID1
};
#endif /* SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE */
#endif /* SPC5_SPI_USE_DSPI0 */

#if SPC5_SPI_USE_DSPI1 || defined(__DOXYGEN__)
#if (SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_AND_TX) || defined(__DOXYGEN__)
/**
 * @brief   DMA configuration for DSPI1 TX1.
 */
static const edma_channel_config_t spi_dspi1_tx1_dma_config = {
  SPC5_SPI_DSPI1_TX1_DMA_CH_ID,
#if SPC5_EDMA_HAS_MUX
  SPC5_DSPI1_TX1_DMA_DEV_ID,
#endif
  SPC5_SPI_DSPI1_DMA_IRQ_PRIO,
  spi_serve_tx_dma_irq, spi_serve_dma_error_irq, &SPID2
};

/**
 * @brief   DMA configuration for DSPI1 TX2.
 */
static const edma_channel_config_t spi_dspi1_tx2_dma_config = {
  SPC5_SPI_DSPI1_TX2_DMA_CH_ID,
#if SPC5_EDMA_HAS_MUX
  0,
#endif
  SPC5_SPI_DSPI1_DMA_IRQ_PRIO,
  spi_serve_tx_dma_irq, spi_serve_dma_error_irq, &SPID2
};
#endif /* SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_AND_TX */

#if (SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE) || defined(__DOXYGEN__)
/**
 * @brief   DMA configuration for DSPI1 RX.
 */
static const edma_channel_config_t spi_dspi1_rx_dma_config = {
  SPC5_SPI_DSPI1_RX_DMA_CH_ID,
#if SPC5_EDMA_HAS_MUX
  SPC5_DSPI1_RX_DMA_DEV_ID,
#endif
  SPC5_SPI_DSPI1_DMA_IRQ_PRIO,
  spi_serve_rx_dma_irq, spi_serve_dma_error_irq, &SPID2
};
#endif /* SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE */
#endif /* SPC5_SPI_USE_DSPI1 */

#if SPC5_SPI_USE_DSPI2 || defined(__DOXYGEN__)
#if (SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_AND_TX) || defined(__DOXYGEN__)
/**
 * @brief   DMA configuration for DSPI2 TX1.
 */
static const edma_channel_config_t spi_dspi2_tx1_dma_config = {
  SPC5_SPI_DSPI2_TX1_DMA_CH_ID,
#if SPC5_EDMA_HAS_MUX
  SPC5_DSPI2_TX1_DMA_DEV_ID,
#endif
  SPC5_SPI_DSPI2_DMA_IRQ_PRIO,
  spi_serve_tx_dma_irq, spi_serve_dma_error_irq, &SPID3
};

/**
 * @brief   DMA configuration for DSPI2 TX2.
 */
static const edma_channel_config_t spi_dspi2_tx2_dma_config = {
  SPC5_SPI_DSPI2_TX2_DMA_CH_ID,
#if SPC5_EDMA_HAS_MUX
  0,
#endif
  SPC5_SPI_DSPI2_DMA_IRQ_PRIO,
  spi_serve_tx_dma_irq, spi_serve_dma_error_irq, &SPID3
};
#endif /* SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_AND_TX */

#if (SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE) || defined(__DOXYGEN__)
/**
 * @brief   DMA configuration for DSPI2 RX.
 */
static const edma_channel_config_t spi_dspi2_rx_dma_config = {
  SPC5_SPI_DSPI2_RX_DMA_CH_ID,
#if SPC5_EDMA_HAS_MUX
  SPC5_DSPI2_RX_DMA_DEV_ID,
#endif
  SPC5_SPI_DSPI2_DMA_IRQ_PRIO,
  spi_serve_rx_dma_irq, spi_serve_dma_error_irq, &SPID3
};
#endif /* SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE */
#endif /* SPC5_SPI_USE_DSPI2 */

#if SPC5_SPI_USE_DSPI3 || defined(__DOXYGEN__)
#if (SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_AND_TX) || defined(__DOXYGEN__)
/**
 * @brief   DMA configuration for DSPI3 TX1.
 */
static const edma_channel_config_t spi_dspi3_tx1_dma_config = {
  SPC5_SPI_DSPI3_TX1_DMA_CH_ID,
#if SPC5_EDMA_HAS_MUX
  SPC5_DSPI3_TX1_DMA_DEV_ID,
#endif
  SPC5_SPI_DSPI3_DMA_IRQ_PRIO,
  spi_serve_tx_dma_irq, spi_serve_dma_error_irq, &SPID4
};

/**
 * @brief   DMA configuration for DSPI3 TX2.
 */
static const edma_channel_config_t spi_dspi3_tx2_dma_config = {
  SPC5_SPI_DSPI3_TX2_DMA_CH_ID,
#if SPC5_EDMA_HAS_MUX
  0,
#endif
  SPC5_SPI_DSPI3_DMA_IRQ_PRIO,
  spi_serve_tx_dma_irq, spi_serve_dma_error_irq, &SPID4
};
#endif /* SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_AND_TX */

#if (SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE) || defined(__DOXYGEN__)
/**
 * @brief   DMA configuration for DSPI3 RX.
 */
static const edma_channel_config_t spi_dspi3_rx_dma_config = {
  SPC5_SPI_DSPI3_RX_DMA_CH_ID,
#if SPC5_EDMA_HAS_MUX
  SPC5_DSPI3_RX_DMA_DEV_ID,
#endif
  SPC5_SPI_DSPI3_DMA_IRQ_PRIO,
  spi_serve_rx_dma_irq, spi_serve_dma_error_irq, &SPID4
};
#endif /* SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE */
#endif /* SPC5_SPI_USE_DSPI3 */

#if SPC5_SPI_USE_DSPI4 || defined(__DOXYGEN__)
#if (SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_AND_TX) || defined(__DOXYGEN__)
/**
 * @brief   DMA configuration for DSPI4 TX1.
 */
static const edma_channel_config_t spi_dspi4_tx1_dma_config = {
  SPC5_SPI_DSPI4_TX1_DMA_CH_ID,
#if SPC5_EDMA_HAS_MUX
  SPC5_DSPI4_TX1_DMA_DEV_ID,
#endif
  SPC5_SPI_DSPI4_DMA_IRQ_PRIO,
  spi_serve_tx_dma_irq, spi_serve_dma_error_irq, &SPID5
};

/**
 * @brief   DMA configuration for DSPI4 TX2.
 */
static const edma_channel_config_t spi_dspi4_tx2_dma_config = {
  SPC5_SPI_DSPI4_TX2_DMA_CH_ID,
#if SPC5_EDMA_HAS_MUX
  0,
#endif
  SPC5_SPI_DSPI4_DMA_IRQ_PRIO,
  spi_serve_tx_dma_irq, spi_serve_dma_error_irq, &SPID5
};
#endif /* SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_AND_TX */

#if (SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE) || defined(__DOXYGEN__)
/**
 * @brief   DMA configuration for DSPI4 RX.
 */
static const edma_channel_config_t spi_dspi4_rx_dma_config = {
  SPC5_SPI_DSPI4_RX_DMA_CH_ID,
#if SPC5_EDMA_HAS_MUX
  SPC5_DSPI4_RX_DMA_DEV_ID,
#endif
  SPC5_SPI_DSPI4_DMA_IRQ_PRIO,
  spi_serve_rx_dma_irq, spi_serve_dma_error_irq, &SPID5
};
#endif /* SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE */
#endif /* SPC5_SPI_USE_DSPI4 */

#if SPC5_SPI_USE_DSPI5 || defined(__DOXYGEN__)
#if (SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_AND_TX) || defined(__DOXYGEN__)
/**
 * @brief   DMA configuration for DSPI5 TX1.
 */
static const edma_channel_config_t spi_dspi5_tx1_dma_config = {
  SPC5_SPI_DSPI5_TX1_DMA_CH_ID,
#if SPC5_EDMA_HAS_MUX
  SPC5_DSPI5_TX1_DMA_DEV_ID,
#endif
  SPC5_SPI_DSPI5_DMA_IRQ_PRIO,
  spi_serve_tx_dma_irq, spi_serve_dma_error_irq, &SPID6
};

/**
 * @brief   DMA configuration for DSPI5 TX2.
 */
static const edma_channel_config_t spi_dspi5_tx2_dma_config = {
  SPC5_SPI_DSPI5_TX2_DMA_CH_ID,
#if SPC5_EDMA_HAS_MUX
  0,
#endif
  SPC5_SPI_DSPI5_DMA_IRQ_PRIO,
  spi_serve_tx_dma_irq, spi_serve_dma_error_irq, &SPID6
};
#endif /* SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_AND_TX */

#if (SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE) || defined(__DOXYGEN__)
/**
 * @brief   DMA configuration for DSPI5 RX.
 */
static const edma_channel_config_t spi_dspi5_rx_dma_config = {
  SPC5_SPI_DSPI5_RX_DMA_CH_ID,
#if SPC5_EDMA_HAS_MUX
  SPC5_DSPI5_RX_DMA_DEV_ID,
#endif
  SPC5_SPI_DSPI5_DMA_IRQ_PRIO,
  spi_serve_rx_dma_irq, spi_serve_dma_error_irq, &SPID6
};
#endif /* SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE */
#endif /* SPC5_SPI_USE_DSPI5 */

#if SPC5_SPI_USE_DSPI6 || defined(__DOXYGEN__)
#if (SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_AND_TX) || defined(__DOXYGEN__)
/**
 * @brief   DMA configuration for DSPI6 TX1.
 */
static const edma_channel_config_t spi_dspi6_tx1_dma_config = {
  SPC5_SPI_DSPI6_TX1_DMA_CH_ID,
#if SPC5_EDMA_HAS_MUX
  SPC5_DSPI6_TX1_DMA_DEV_ID,
#endif
  SPC5_SPI_DSPI6_DMA_IRQ_PRIO,
  spi_serve_tx_dma_irq, spi_serve_dma_error_irq, &SPID7
};

/**
 * @brief   DMA configuration for DSPI6 TX2.
 */
static const edma_channel_config_t spi_dspi6_tx2_dma_config = {
  SPC5_SPI_DSPI6_TX2_DMA_CH_ID,
#if SPC5_EDMA_HAS_MUX
  0,
#endif
  SPC5_SPI_DSPI6_DMA_IRQ_PRIO,
  spi_serve_tx_dma_irq, spi_serve_dma_error_irq, &SPID7
};
#endif /* SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_AND_TX */

#if (SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE) || defined(__DOXYGEN__)
/**
 * @brief   DMA configuration for DSPI6 RX.
 */
static const edma_channel_config_t spi_dspi6_rx_dma_config = {
  SPC5_SPI_DSPI6_RX_DMA_CH_ID,
#if SPC5_EDMA_HAS_MUX
  SPC5_DSPI6_RX_DMA_DEV_ID,
#endif
  SPC5_SPI_DSPI6_DMA_IRQ_PRIO,
  spi_serve_rx_dma_irq, spi_serve_dma_error_irq, &SPID7
};
#endif /* SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE */
#endif /* SPC5_SPI_USE_DSPI6 */

#if SPC5_SPI_USE_DSPI7 || defined(__DOXYGEN__)
#if (SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_AND_TX) || defined(__DOXYGEN__)
/**
 * @brief   DMA configuration for DSPI7 TX1.
 */
static const edma_channel_config_t spi_dspi7_tx1_dma_config = {
  SPC5_SPI_DSPI7_TX1_DMA_CH_ID,
#if SPC5_EDMA_HAS_MUX
  SPC5_DSPI7_TX1_DMA_DEV_ID,
#endif
  SPC5_SPI_DSPI7_DMA_IRQ_PRIO,
  spi_serve_tx_dma_irq, spi_serve_dma_error_irq, &SPID8
};

/**
 * @brief   DMA configuration for DSPI7 TX2.
 */
static const edma_channel_config_t spi_dspi7_tx2_dma_config = {
  SPC5_SPI_DSPI7_TX2_DMA_CH_ID,
#if SPC5_EDMA_HAS_MUX
  0,
#endif
  SPC5_SPI_DSPI7_DMA_IRQ_PRIO,
  spi_serve_tx_dma_irq, spi_serve_dma_error_irq, &SPID8
};
#endif /* SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_AND_TX */

#if (SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE) || defined(__DOXYGEN__)
/**
 * @brief   DMA configuration for DSPI7 RX.
 */
static const edma_channel_config_t spi_dspi7_rx_dma_config = {
  SPC5_SPI_DSPI7_RX_DMA_CH_ID,
#if SPC5_EDMA_HAS_MUX
  SPC5_DSPI7_RX_DMA_DEV_ID,
#endif
  SPC5_SPI_DSPI7_DMA_IRQ_PRIO,
  spi_serve_rx_dma_irq, spi_serve_dma_error_irq, &SPID8
};
#endif /* SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE */
#endif /* SPC5_SPI_USE_DSPI7 */

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Initializes the invariant part of the @p SPIDriver structure.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] dspi      the physical DSPI unit to be associated to the object
 *
 * @notapi
 */
static void spi_lld_obj_init(SPIDriver *spip, struct spc5_dspi *dspi) {

  spip->dspi = dspi;
#if SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_AND_TX
  spip->tx1_channel = EDMA_ERROR;
  spip->tx2_channel = EDMA_ERROR;
#endif
#if SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE
  spip->rx_channel  = EDMA_ERROR;
#endif
}

/**
 * @brief   DSPI unit setup for transfer.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
static void spi_dspi_start(SPIDriver *spip) {

  spip->dspi->SR.R = spip->dspi->SR.R;
  spip->dspi->MCR.B.HALT = 0;
}

/**
 * @brief   DSPI unit transfer stop.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
static void spi_dspi_stop(SPIDriver *spip) {

  /* Stops the DSPI and clears the queues.*/
  spip->dspi->MCR.R |= SPC5_MCR_HALT | SPC5_MCR_CLR_TXF | SPC5_MCR_CLR_RXF;
}

/**
 * @brief   Prefills the TX FIFO with idle frames.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in,out] np    pointer to the number of frames to send, must be
 *                      greater than zero, contains the number of remaining
 *                      frames on return
 *
 * @notapi
 */
static void spi_dspi_prefill_txfifo_idle(SPIDriver *spip, size_t *np) {
  uint32_t cmd = spip->config->pushr;

  while (spip->dspi->SR.B.TXCTR < SPC5_DSPI_FIFO_DEPTH) {
    if (--(*np) == 0) {
      spip->dspi->PUSHR.R = (SPC5_PUSHR_EOQ | cmd | 0xFFFF) & ~SPC5_PUSHR_CONT;
      break;
    }
    spip->dspi->PUSHR.R = cmd | 0x0000FFFF;
  }
}

/**
 * @brief   Prefills the TX FIFO using 8 bits frames.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in,out] np    pointer to the number of frames to send, must be
 *                      greater than zero, contains the number of remaining
 *                      frames on return
 * @param[in,out] txpp  pointer to the pointer to the transmit buffer
 *
 * @notapi
 */
static void spi_dspi_prefill_txfifo8(SPIDriver *spip,
                                     size_t *np,
                                     const uint8_t **txpp) {
  uint32_t cmd = spip->config->pushr;

  while (spip->dspi->SR.B.TXCTR < SPC5_DSPI_FIFO_DEPTH) {
    uint32_t frame = **txpp;
    (*txpp)++;

    if (--(*np) == 0) {
      spip->dspi->PUSHR.R = (SPC5_PUSHR_EOQ | cmd | frame) & ~SPC5_PUSHR_CONT;
      break;
    }
    spip->dspi->PUSHR.R = cmd | frame;
  }
}

/**
 * @brief   Prefills the TX FIFO using 16 bits frames.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in,out] np    pointer to the number of frames to send, must be
 *                      greater than zero, contains the number of remaining
 *                      frames on return
 * @param[in,out] txpp  pointer to the pointer to the transmit buffer
 *
 * @notapi
 */
static void spi_dspi_prefill_txfifo16(SPIDriver *spip,
                                      size_t *np,
                                      const uint16_t **txpp) {
  uint32_t cmd = spip->config->pushr;

  while (spip->dspi->SR.B.TXCTR < SPC5_DSPI_FIFO_DEPTH) {
    uint32_t frame = **txpp;
    (*txpp)++;

    if (--(*np) == 0) {
      spip->dspi->PUSHR.R = (SPC5_PUSHR_EOQ | cmd | frame) & ~SPC5_PUSHR_CONT;
      break;
    }
    spip->dspi->PUSHR.R = cmd | frame;
  }
}

/**
 * @brief   Starts reception using DMA ignoring the received data.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be exchanged
 *
 * @notapi
 */
static void spi_start_rx_ignore(SPIDriver *spip, size_t n) {

#if SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_NONE
  /* Setting up the fields required for operation continuation.*/
  spip->rx_ptr = NULL;
  spip->rx_cnt = n;

#else /* SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE */
  static uint32_t datasink;

  edmaChannelSetup(spip->rx_channel,            /* channel.                 */
                   DSPI_POPR8_ADDRESS(spip),    /* src.                     */
                   &datasink,                   /* dst.                     */
                   0,                           /* soff, do not advance.    */
                   0,                           /* doff, do not advance.    */
                   0,                           /* ssize, 8 bits transfers. */
                   0,                           /* dsize, 8 bits transfers. */
                   1,                           /* nbytes, always one.      */
                   n,                           /* iter.                    */
                   0,                           /* slast.                   */
                   0,                           /* dlast.                   */
                   EDMA_TCD_MODE_DREQ | EDMA_TCD_MODE_INT_END);     /* mode.*/

  edmaChannelStart(spip->rx_channel);
#endif /* SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE */
}

/**
 * @brief   Starts reception using DMA for frames up to 8 bits.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be exchanged
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */
static void spi_start_rx8(SPIDriver *spip, size_t n, uint8_t *rxbuf) {

#if SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_NONE
  /* Setting up the fields required for operation continuation.*/
  spip->rx_ptr8 = rxbuf;
  spip->rx_cnt  = n;

#else /* SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE */
  edmaChannelSetup(spip->rx_channel,            /* channel.                 */
                   DSPI_POPR8_ADDRESS(spip),    /* src.                     */
                   rxbuf,                       /* dst.                     */
                   0,                           /* soff, do not advance.    */
                   1,                           /* doff, advance by one.    */
                   0,                           /* ssize, 8 bits transfers. */
                   0,                           /* dsize, 8 bits transfers. */
                   1,                           /* nbytes, always one.      */
                   n,                           /* iter.                    */
                   0,                           /* slast.                   */
                   0,                           /* dlast.                   */
                   EDMA_TCD_MODE_DREQ | EDMA_TCD_MODE_INT_END);     /* mode.*/

  edmaChannelStart(spip->rx_channel);
#endif /* SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE */
}

/**
 * @brief   Starts reception using DMA for frames up to 16 bits.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be exchanged
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */
static void spi_start_rx16(SPIDriver *spip, size_t n, uint16_t *rxbuf) {

#if SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_NONE
  /* Setting up the fields required for operation continuation.*/
  spip->rx_ptr16 = rxbuf;
  spip->rx_cnt   = n;

#else /* SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE */
  edmaChannelSetup(spip->rx_channel,            /* channel.                 */
                   DSPI_POPR16_ADDRESS(spip),   /* src.                     */
                   rxbuf,                       /* dst.                     */
                   0,                           /* soff, do not advance.    */
                   2,                           /* doff, advance by two.    */
                   1,                           /* ssize, 16 bits transfers.*/
                   1,                           /* dsize, 16 bits transfers.*/
                   2,                           /* nbytes, always two.      */
                   n,                           /* iter.                    */
                   0,                           /* slast, no source adjust. */
                   0,                           /* dlast.                   */
                   EDMA_TCD_MODE_DREQ | EDMA_TCD_MODE_INT_END); /* mode.    */

  edmaChannelStart(spip->rx_channel);
#endif /* SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE */
}

/**
 * @brief   Starts transmission using DMA for frames up to 8 bits.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be exchanged
 *
 * @notapi
 */
static void spi_start_tx_ignore(SPIDriver *spip, size_t n) {

#if SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_RX_AND_TX
  /* Preloading the TX FIFO with as much frames as possible.*/
  spi_dspi_prefill_txfifo_idle(spip, &n);

  /* This is the case where the whole operation can be satisfied using the
     preloading alone.*/
  if (n == 0)
    return;

  /* Setting up the fields required for operation continuation.*/
  spip->tx_ptr = NULL;
  spip->tx_cnt = n;

  /* Enabling the TFFF interrupt source for transfer continuation.*/
  spip->dspi->RSER.B.TFFFRE = 1;

#else /* SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_AND_TX */
  /* Special case when the data to be transmitted can entirely fit into the
     TX FIFO, in this case the TX DMAs are not activated.*/
  if (n <= SPC5_DSPI_FIFO_DEPTH) {
    spi_dspi_prefill_txfifo_idle(spip, &n);
    return;
  }

  /* Preparing the TX intermediate buffer with the fixed part.*/
  spip->tx_cmd = spip->config->pushr | (uint32_t)0xFFFF;

  /* The first frame is pushed by the CPU, then the DMA is activated to
     send the following frames. This should reduce latency on the operation
     start.*/
  spip->dspi->PUSHR.R = spip->tx_last = spip->tx_cmd;

  /* Setting up TX1 DMA TCD parameters for 32 bits transfers.*/
  edmaChannelSetup(spip->tx1_channel,           /* channel.                 */
                   &spip->tx_cmd,               /* src.                     */
                   &spip->dspi->PUSHR.R,        /* dst.                     */
                   0,                           /* soff, do not advance.    */
                   0,                           /* doff, do not advance.    */
                   2,                           /* ssize, 32 bits transfers.*/
                   2,                           /* dsize, 32 bits transfers.*/
                   4,                           /* nbytes, always four.     */
                   n - 2,                       /* iter.                    */
                   0,                           /* slast, no source adjust. */
                   0,                           /* dlast, no dest.adjust.   */
                   EDMA_TCD_MODE_DREQ | EDMA_TCD_MODE_INT_END); /* mode.    */

  /* Starting TX1 DMA channel.*/
  edmaChannelStart(spip->tx1_channel);
#endif /* SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_AND_TX */
}

/**
 * @brief   Starts transmission using DMA for frames up to 8 bits.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be exchanged
 * @param[in] txbuf     the pointer to the transmit buffer
 *
 * @notapi
 */
static void spi_start_tx8(SPIDriver *spip, size_t n, const uint8_t *txbuf) {

#if SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_RX_AND_TX
  /* Preloading the TX FIFO with as much frames as possible.*/
  spi_dspi_prefill_txfifo8(spip, &n, &txbuf);

  /* This is the case where the whole operation can be satisfied using the
     preloading alone.*/
  if (n == 0)
    return;

  /* Setting up the fields required for operation continuation.*/
  spip->tx_ptr8 = txbuf;
  spip->tx_cnt = n;

  /* Enabling the TFFF interrupt source for transfer continuation.*/
  spip->dspi->RSER.B.TFFFRE = 1;

#else /* SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_AND_TX */
  /* Special case when the data to be transmitted can entirely fit into the
     TX FIFO, in this case the TX DMAs are not activated.*/
  if (n <= SPC5_DSPI_FIFO_DEPTH) {
    spi_dspi_prefill_txfifo8(spip, &n, &txbuf);
    return;
  }

  /* Preparing the TX intermediate buffer with the fixed part.*/
  spip->tx_cmd = spip->config->pushr;

  /* The first frame is pushed immediately, then the DMA is activated to
     send the following frames. This should reduce latency on the operation
     start.*/
  spip->dspi->PUSHR.R = spip->config->pushr | (uint32_t)*txbuf;

  /* The last frame is a special case, will be pushed by the TX FIFO drain
     interrupt handler or the DMA final callback.*/
  spip->tx_last = txbuf[n - 1];

  /* At least two frames left, the DMA is enabled in order to handle the
     long transfer, note that the final frame is not pushed by the DMA.*/
  /* Setting up TX1 DMA TCD parameters for 8 bits transfers.*/
  edmaChannelSetupLinked(
                   spip->tx1_channel,           /* channel.                 */
                   spip->tx2_channel,           /* linkch.                  */
                   txbuf + 1,                   /* src.                     */
                   ((const uint8_t *)&spip->tx_cmd) + 3,        /* dst.     */
                   1,                           /* soff, advance by 1.      */
                   0,                           /* doff, do not advance.    */
                   0,                           /* ssize, 8 bits transfers. */
                   0,                           /* dsize, 8 bits transfers. */
                   1,                           /* nbytes, always one.      */
                   n - 2,                       /* iter.                    */
                   0,                           /* slast, no source adjust. */
                   0,                           /* dlast, no dest.adjust.   */
                   EDMA_TCD_MODE_DREQ);         /* mode.                    */

  /* Setting up TX2 DMA TCD parameters for 32 bits transfers.*/
  edmaChannelSetup(spip->tx2_channel,           /* channel.                 */
                   &spip->tx_cmd,               /* src.                     */
                   &spip->dspi->PUSHR.R,        /* dst.                     */
                   0,                           /* soff, do not advance.    */
                   0,                           /* doff, do not advance.    */
                   2,                           /* ssize, 32 bits transfers.*/
                   2,                           /* dsize, 32 bits transfers.*/
                   4,                           /* nbytes, always four.     */
                   n - 2,                       /* iter.                    */
                   0,                           /* slast, no source adjust. */
                   0,                           /* dlast, no dest.adjust.   */
                   EDMA_TCD_MODE_DREQ | EDMA_TCD_MODE_INT_END); /* mode.    */

  /* Starting TX DMA channels.*/
  edmaChannelStart(spip->tx2_channel);
  edmaChannelStart(spip->tx1_channel);
#endif /* SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_AND_TX */
}

/**
 * @brief   Starts transmission using DMA for frames up to 16 bits.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be exchanged
 * @param[in] txbuf     the pointer to the transmit buffer
 *
 * @notapi
 */
static void spi_start_tx16(SPIDriver *spip, size_t n, const uint16_t *txbuf) {

#if SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_RX_AND_TX
  /* Preloading the TX FIFO with as much frames as possible.*/
  spi_dspi_prefill_txfifo16(spip, &n, &txbuf);

  /* This is the case where the whole operation can be satisfied using the
     preloading alone.*/
  if (n == 0)
    return;

  /* Setting up the fields required for operation continuation.*/
  spip->tx_ptr16 = txbuf;
  spip->tx_cnt = n;

  /* Enabling the TFFF interrupt source for transfer continuation.*/
  spip->dspi->RSER.B.TFFFRE = 1;

#else /* SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_AND_TX */
  /* Special case when the data to be transmitted can entirely fit into the
     TX FIFO, in this case the TX DMAs are not activated.*/
  if (n <= SPC5_DSPI_FIFO_DEPTH) {
    spi_dspi_prefill_txfifo16(spip, &n, &txbuf);
    return;
  }

  /* Preparing the TX intermediate buffer with the fixed part.*/
  spip->tx_cmd = spip->config->pushr;

  /* The first frame is pushed immediately, then the DMA is activated to
     send the following frames. This should reduce latency on the operation
     start.*/
  spip->dspi->PUSHR.R = spip->config->pushr | (uint32_t)*txbuf;

  /* The last frame is a special case, will be pushed by the TX FIFO drain
     interrupt handler or the DMA final callback.*/
  spip->tx_last = txbuf[n - 1];

  /* At least two frames left, the DMA is enabled in order to handle the
     long transfer, note that the final frame is not pushed by the DMA.*/
  /* Setting up TX1 DMA TCD parameters for 16 bits transfers.*/
  edmaChannelSetupLinked(
                   spip->tx1_channel,           /* channel.                 */
                   spip->tx2_channel,           /* linkch.                  */
                   txbuf + 1,                   /* src.                     */
                   ((const uint8_t *)&spip->tx_cmd) + 2,        /* dst.     */
                   2,                           /* soff, advance by 2.      */
                   0,                           /* doff, do not advance.    */
                   1,                           /* ssize, 16 bits transfers.*/
                   1,                           /* dsize, 16 bits transfers.*/
                   2,                           /* nbytes, always two.      */
                   n - 2,                       /* iter.                    */
                   0,                           /* slast, no source adjust. */
                   0,                           /* dlast, no dest.adjust.   */
                   EDMA_TCD_MODE_DREQ);         /* mode.                    */

  /* Setting up TX2 DMA TCD parameters for 32 bits transfers.*/
  edmaChannelSetup(spip->tx2_channel,           /* channel.                 */
                   &spip->tx_cmd,               /* src.                     */
                   &spip->dspi->PUSHR.R,        /* dst.                     */
                   0,                           /* soff, do not advance.    */
                   0,                           /* doff, do not advance.    */
                   2,                           /* ssize, 32 bits transfers.*/
                   2,                           /* dsize, 32 bits transfers.*/
                   4,                           /* nbytes, always four.     */
                   n - 2,                       /* iter.                    */
                   0,                           /* slast, no source adjust. */
                   0,                           /* dlast, no dest.adjust.   */
                   EDMA_TCD_MODE_DREQ | EDMA_TCD_MODE_INT_END); /* mode.    */

  /* Starting TX DMA channels.*/
  edmaChannelStart(spip->tx2_channel);
  edmaChannelStart(spip->tx1_channel);
#endif /* SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_AND_TX */
}

#if (SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE) || defined(__DOXYGEN__)
/**
 * @brief   Shared RX DMA events service routine.
 *
 * @param[in] channel   the channel number
 * @param[in] p         parameter for the registered function
 *
 * @notapi
 */
static void spi_serve_rx_dma_irq(edma_channel_t channel, void *p) {
  SPIDriver *spip = (SPIDriver *)p;

  /* Clearing RX channel state.*/
  edmaChannelStop(channel);

  /* Stops the transfer.*/
  spi_dspi_stop(spip);

  /* Portable SPI ISR code defined in the high level driver, note, it is
     a macro.*/
  _spi_isr_code(spip);
}
#endif /* SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE */

#if (SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_AND_TX) || defined(__DOXYGEN__)
/**
 * @brief   Shared TX1/TX2 DMA events service routine.
 *
 * @param[in] channel   the channel number
 * @param[in] p         parameter for the registered function
 *
 * @notapi
 */
static void spi_serve_tx_dma_irq(edma_channel_t channel, void *p) {
  SPIDriver *spip = (SPIDriver *)p;

  (void)channel;

  /* Clearing TX channels state.*/
  edmaChannelStop(spip->tx1_channel);
  edmaChannelStop(spip->tx2_channel);

  /* If the TX FIFO is full then the push of the last frame is delegated to
     an interrupt handler else it is performed immediately. Both conditions
     can be true depending on the SPI speed and ISR latency.*/
  if (spip->dspi->SR.B.TFFF) {
    spip->dspi->PUSHR.R = (spip->config->pushr | spip->tx_last | SPC5_PUSHR_EOQ) &
                          ~SPC5_PUSHR_CONT;
  }
  else {
    spip->dspi->RSER.B.TFFFDIRS = 0;
  }
}
#endif /* SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_AND_TX */

#if (SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE) || defined(__DOXYGEN__)
/**
 * @brief   Shared ISR for DMA error events.
 *
 * @param[in] channel   the channel number
 * @param[in] p         parameter for the registered function
 * @param[in] esr       content of the ESR register
 *
 * @notapi
 */
static void spi_serve_dma_error_irq(edma_channel_t channel,
                                    void *p,
                                    uint32_t esr) {
  SPIDriver *spip = (SPIDriver *)p;

  (void)channel;
  (void)esr;

  /* Stops the transfer.*/
  spi_dspi_stop(spip);

#if SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_AND_TX
  edmaChannelStop(spip->tx1_channel);
  edmaChannelStop(spip->tx2_channel);
#endif
  edmaChannelStop(spip->rx_channel);

  SPC5_SPI_DMA_ERROR_HOOK(spip);
}
#endif /* SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE */

#if (SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_NONE) || defined(__DOXYGEN__)
/**
 * @brief   Shared ISR for RFDF DSPI events.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_serve_dspi_rfdf(SPIDriver *spip) {

  osalSysLockFromISR();

  /* Emptying the RX FIFO.*/
  while ((spip->rx_cnt > 0) && (spip->dspi->SR.B.RXCTR > 0)) {
    uint32_t frame = spip->dspi->POPR.R;
    if (spip->rx_ptr != NULL) {
      if (spip->dspi->CTAR[0].B.FMSZ < 8)
        *spip->rx_ptr8++ = (uint8_t)frame;
      else
        *spip->rx_ptr16++ = (uint16_t)frame;
    }
    spip->rx_cnt--;
  }

  /* Interrupt served.*/
  spip->dspi->SR.B.RFDF = 1;

  if (spip->rx_cnt == 0) {
    /* Stops the transfer.*/
    spi_dspi_stop(spip);

    /* Portable SPI ISR code defined in the high level driver, note, it is
       a macro.*/
    _spi_isr_code(spip);
  }
  else {
    if (spip->tx_cnt > 0) {
      /* Filling the TX FIFO.*/
      if (spip->tx_ptr == NULL)
        spi_dspi_prefill_txfifo_idle(spip, &spip->tx_cnt);
      else {
        if (spip->dspi->CTAR[0].B.FMSZ < 8)
          spi_dspi_prefill_txfifo8(spip, &spip->tx_cnt, &spip->tx_ptr8);
        else
          spi_dspi_prefill_txfifo16(spip, &spip->tx_cnt, &spip->tx_ptr16);
      }
    }
  }

  osalSysUnlockFromISR();
}
#endif /* SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE */

#if (SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE) || defined(__DOXYGEN__)
/**
 * @brief   Shared ISR for TFFF DSPI events.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_serve_dspi_tfff(SPIDriver *spip) {

  osalSysLockFromISR();

#if SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_AND_TX
  /* Interrupt served and back to DMA mode.*/
  spip->dspi->RSER.B.TFFFDIRS = 1;
  spip->dspi->SR.B.TFFF = 1;

  /* Pushing last frame.*/
  spip->dspi->PUSHR.R = (spip->config->pushr | spip->tx_last | SPC5_PUSHR_EOQ) &
                         ~SPC5_PUSHR_CONT;
#endif /* SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_AND_TX */

#if SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_ONLY
  /* Pushing some more frames in the TX FIFO.*/
  if (spip->tx_ptr == NULL)
    spi_dspi_prefill_txfifo_idle(spip, &spip->tx_cnt);
  else {
    if (spip->dspi->CTAR[0].B.FMSZ < 8)
      spi_dspi_prefill_txfifo8(spip, &spip->tx_cnt, &spip->tx_ptr8);
    else
      spi_dspi_prefill_txfifo16(spip, &spip->tx_cnt, &spip->tx_ptr16);
  }

  /* Interrupt served.*/
  spip->dspi->SR.B.TFFF = 1;

  /* If there are no more frames to be pushed then the TFFF interrupt source
     is disabled.*/
  if (spip->tx_cnt == 0)
    spip->dspi->RSER.B.TFFFRE = 0;
#endif /* SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_ONLY */

  osalSysUnlockFromISR();
}
#endif /* SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE */

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if (SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_NONE) || defined(__DOXYGEN__)
#if SPC5_SPI_USE_DSPI0 || defined(__DOXYGEN__)
#if !defined(SPC5_DSPI0_RFDF_HANDLER)
#error "SPC5_DSPI0_RFDF_HANDLER not defined"
#endif
/**
 * @brief   DSPI0 RFDF interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_DSPI0_RFDF_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spi_serve_dspi_rfdf(&SPID1);

  OSAL_IRQ_EPILOGUE();
}
#endif /* SPC5_SPI_USE_DSPI0 */

#if SPC5_SPI_USE_DSPI1 || defined(__DOXYGEN__)
#if !defined(SPC5_DSPI1_RFDF_HANDLER)
#error "SPC5_DSPI1_RFDF_HANDLER not defined"
#endif
/**
 * @brief   DSPI1 RFDF interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_DSPI1_RFDF_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spi_serve_dspi_rfdf(&SPID2);

  OSAL_IRQ_EPILOGUE();
}
#endif /* SPC5_SPI_USE_DSPI1 */

#if SPC5_SPI_USE_DSPI2 || defined(__DOXYGEN__)
#if !defined(SPC5_DSPI2_RFDF_HANDLER)
#error "SPC5_DSPI2_RFDF_HANDLER not defined"
#endif
/**
 * @brief   DSPI2 RFDF interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_DSPI2_RFDF_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spi_serve_dspi_rfdf(&SPID3);

  OSAL_IRQ_EPILOGUE();
}
#endif /* SPC5_SPI_USE_DSPI2 */

#if SPC5_SPI_USE_DSPI3 || defined(__DOXYGEN__)
#if !defined(SPC5_DSPI3_RFDF_HANDLER)
#error "SPC5_DSPI3_RFDF_HANDLER not defined"
#endif
/**
 * @brief   DSPI3 RFDF interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_DSPI3_RFDF_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spi_serve_dspi_rfdf(&SPID4);

  OSAL_IRQ_EPILOGUE();
}
#endif /* SPC5_SPI_USE_DSPI3 */

#if SPC5_SPI_USE_DSPI4 || defined(__DOXYGEN__)
#if !defined(SPC5_DSPI4_RFDF_HANDLER)
#error "SPC5_DSPI4_RFDF_HANDLER not defined"
#endif
/**
 * @brief   DSPI4 RFDF interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_DSPI4_RFDF_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spi_serve_dspi_rfdf(&SPID5);

  OSAL_IRQ_EPILOGUE();
}
#endif /* SPC5_SPI_USE_DSPI4 */

#if SPC5_SPI_USE_DSPI5 || defined(__DOXYGEN__)
#if !defined(SPC5_DSPI5_RFDF_HANDLER)
#error "SPC5_DSPI5_RFDF_HANDLER not defined"
#endif
/**
 * @brief   DSPI5 RFDF interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_DSPI5_RFDF_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spi_serve_dspi_rfdf(&SPID6);

  OSAL_IRQ_EPILOGUE();
}
#endif /* SPC5_SPI_USE_DSPI5 */

#if SPC5_SPI_USE_DSPI6 || defined(__DOXYGEN__)
#if !defined(SPC5_DSPI6_RFDF_HANDLER)
#error "SPC5_DSPI6_RFDF_HANDLER not defined"
#endif
/**
 * @brief   DSPI6 RFDF interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_DSPI6_RFDF_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spi_serve_dspi_rfdf(&SPID7);

  OSAL_IRQ_EPILOGUE();
}
#endif /* SPC5_SPI_USE_DSPI6 */

#if SPC5_SPI_USE_DSPI7 || defined(__DOXYGEN__)
#if !defined(SPC5_DSPI7_RFDF_HANDLER)
#error "SPC5_DSPI7_RFDF_HANDLER not defined"
#endif
/**
 * @brief   DSPI7 RFDF interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_DSPI7_RFDF_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spi_serve_dspi_rfdf(&SPID8);

  OSAL_IRQ_EPILOGUE();
}
#endif /* SPC5_SPI_USE_DSPI7 */
#endif /* SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_NONE */

#if (SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE) || defined(__DOXYGEN__)
#if SPC5_SPI_USE_DSPI0 || defined(__DOXYGEN__)
#if !defined(SPC5_DSPI0_TFFF_HANDLER)
#error "SPC5_DSPI0_TFFF_HANDLER not defined"
#endif
/**
 * @brief   DSPI0 TFFF interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_DSPI0_TFFF_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spi_serve_dspi_tfff(&SPID1);

  OSAL_IRQ_EPILOGUE();
}
#endif /* SPC5_SPI_USE_DSPI0 */

#if SPC5_SPI_USE_DSPI1 || defined(__DOXYGEN__)
#if !defined(SPC5_DSPI1_TFFF_HANDLER)
#error "SPC5_DSPI1_TFFF_HANDLER not defined"
#endif
/**
 * @brief   DSPI1 TFFF interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_DSPI1_TFFF_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spi_serve_dspi_tfff(&SPID2);

  OSAL_IRQ_EPILOGUE();
}
#endif /* SPC5_SPI_USE_DSPI1 */

#if SPC5_SPI_USE_DSPI2 || defined(__DOXYGEN__)
#if !defined(SPC5_DSPI2_TFFF_HANDLER)
#error "SPC5_DSPI2_TFFF_HANDLER not defined"
#endif
/**
 * @brief   DSPI2 TFFF interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_DSPI2_TFFF_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spi_serve_dspi_tfff(&SPID3);

  OSAL_IRQ_EPILOGUE();
}
#endif /* SPC5_SPI_USE_DSPI2 */

#if SPC5_SPI_USE_DSPI3 || defined(__DOXYGEN__)
#if !defined(SPC5_DSPI3_TFFF_HANDLER)
#error "SPC5_DSPI3_TFFF_HANDLER not defined"
#endif
/**
 * @brief   DSPI3 TFFF interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_DSPI3_TFFF_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spi_serve_dspi_tfff(&SPID4);

  OSAL_IRQ_EPILOGUE();
}
#endif /* SPC5_SPI_USE_DSPI3 */

#if SPC5_SPI_USE_DSPI4 || defined(__DOXYGEN__)
#if !defined(SPC5_DSPI4_TFFF_HANDLER)
#error "SPC5_DSPI4_TFFF_HANDLER not defined"
#endif
/**
 * @brief   DSPI4 TFFF interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_DSPI4_TFFF_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spi_serve_dspi_tfff(&SPID5);

  OSAL_IRQ_EPILOGUE();
}
#endif /* SPC5_SPI_USE_DSPI4 */

#if SPC5_SPI_USE_DSPI5 || defined(__DOXYGEN__)
#if !defined(SPC5_DSPI5_TFFF_HANDLER)
#error "SPC5_DSPI5_TFFF_HANDLER not defined"
#endif
/**
 * @brief   DSPI5 TFFF interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_DSPI5_TFFF_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spi_serve_dspi_tfff(&SPID6);

  OSAL_IRQ_EPILOGUE();
}
#endif /* SPC5_SPI_USE_DSPI5 */

#if SPC5_SPI_USE_DSPI6 || defined(__DOXYGEN__)
#if !defined(SPC5_DSPI6_TFFF_HANDLER)
#error "SPC5_DSPI6_TFFF_HANDLER not defined"
#endif
/**
 * @brief   DSPI6 TFFF interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_DSPI6_TFFF_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spi_serve_dspi_tfff(&SPID7);

  OSAL_IRQ_EPILOGUE();
}
#endif /* SPC5_SPI_USE_DSPI6 */

#if SPC5_SPI_USE_DSPI7 || defined(__DOXYGEN__)
#if !defined(SPC5_DSPI7_TFFF_HANDLER)
#error "SPC5_DSPI7_TFFF_HANDLER not defined"
#endif
/**
 * @brief   DSPI7 TFFF interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_DSPI7_TFFF_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spi_serve_dspi_tfff(&SPID8);

  OSAL_IRQ_EPILOGUE();
}
#endif /* SPC5_SPI_USE_DSPI7 */
#endif /* SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level SPI driver initialization.
 *
 * @notapi
 */
void spi_lld_init(void) {

#if SPC5_SPI_USE_DSPI0
  /* Driver initialization.*/
  SPC5_DSPI0_ENABLE_CLOCK();
  spiObjectInit(&SPID1);
  spi_lld_obj_init(&SPID1, &SPC5_DSPI0);
  SPC5_DSPI0.MCR.R  = SPC5_MCR_MSTR | SPC5_MCR_HALT | SPC5_MCR_MDIS |
                      SPC5_SPI_DSPI0_MCR;
#if SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_NONE
  INTC.PSR[SPC5_DSPI0_RFDF_NUMBER].R = SPC5_SPI_DSPI0_IRQ_PRIO;
#else
  INTC.PSR[SPC5_DSPI0_TFFF_NUMBER].R = SPC5_SPI_DSPI0_IRQ_PRIO;
#endif
#endif /* SPC5_SPI_USE_DSPI0 */

#if SPC5_SPI_USE_DSPI1
  /* Driver initialization.*/
  SPC5_DSPI1_ENABLE_CLOCK();
  spiObjectInit(&SPID2);
  spi_lld_obj_init(&SPID2, &SPC5_DSPI1);
  SPC5_DSPI1.MCR.R  = SPC5_MCR_MSTR | SPC5_MCR_HALT | SPC5_MCR_MDIS |
                      SPC5_SPI_DSPI1_MCR;
#if SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_NONE
  INTC.PSR[SPC5_DSPI1_RFDF_NUMBER].R = SPC5_SPI_DSPI1_IRQ_PRIO;
#else
  INTC.PSR[SPC5_DSPI1_TFFF_NUMBER].R = SPC5_SPI_DSPI1_IRQ_PRIO;
#endif
#endif /* SPC5_SPI_USE_DSPI1 */

#if SPC5_SPI_USE_DSPI2
  /* Driver initialization.*/
  SPC5_DSPI2_ENABLE_CLOCK();
  spiObjectInit(&SPID3);
  spi_lld_obj_init(&SPID3, &SPC5_DSPI2);
  SPC5_DSPI2.MCR.R  = SPC5_MCR_MSTR | SPC5_MCR_HALT | SPC5_MCR_MDIS |
                      SPC5_SPI_DSPI2_MCR;
#if SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_NONE
  INTC.PSR[SPC5_DSPI2_RFDF_NUMBER].R = SPC5_SPI_DSPI2_IRQ_PRIO;
#else
  INTC.PSR[SPC5_DSPI2_TFFF_NUMBER].R = SPC5_SPI_DSPI2_IRQ_PRIO;
#endif
#endif /* SPC5_SPI_USE_DSPI2 */

#if SPC5_SPI_USE_DSPI3
  /* Driver initialization.*/
  SPC5_DSPI3_ENABLE_CLOCK();
  spiObjectInit(&SPID4);
  spi_lld_obj_init(&SPID4, &SPC5_DSPI3);
  SPC5_DSPI3.MCR.R  = SPC5_MCR_MSTR | SPC5_MCR_HALT | SPC5_MCR_MDIS |
                      SPC5_SPI_DSPI3_MCR;
#if SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_NONE
  INTC.PSR[SPC5_DSPI3_RFDF_NUMBER].R = SPC5_SPI_DSPI3_IRQ_PRIO;
#else
  INTC.PSR[SPC5_DSPI3_TFFF_NUMBER].R = SPC5_SPI_DSPI3_IRQ_PRIO;
#endif
#endif /* SPC5_SPI_USE_DSPI3 */

#if SPC5_SPI_USE_DSPI4
  /* Driver initialization.*/
  SPC5_DSPI4_ENABLE_CLOCK();
  spiObjectInit(&SPID5);
  spi_lld_obj_init(&SPID5, &SPC5_DSPI4);
  SPC5_DSPI4.MCR.R  = SPC5_MCR_MSTR | SPC5_MCR_HALT | SPC5_MCR_MDIS |
                      SPC5_SPI_DSPI4_MCR;
#if SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_NONE
  INTC.PSR[SPC5_DSPI4_RFDF_NUMBER].R = SPC5_SPI_DSPI4_IRQ_PRIO;
#else
  INTC.PSR[SPC5_DSPI4_TFFF_NUMBER].R = SPC5_SPI_DSPI4_IRQ_PRIO;
#endif
#endif /* SPC5_SPI_USE_DSPI4 */

#if SPC5_SPI_USE_DSPI5
  /* Driver initialization.*/
  SPC5_DSPI5_ENABLE_CLOCK();
  spiObjectInit(&SPID6);
  spi_lld_obj_init(&SPID6, &SPC5_DSPI5);
  SPC5_DSPI5.MCR.R  = SPC5_MCR_MSTR | SPC5_MCR_HALT | SPC5_MCR_MDIS |
                      SPC5_SPI_DSPI5_MCR;
#if SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_NONE
  INTC.PSR[SPC5_DSPI5_RFDF_NUMBER].R = SPC5_SPI_DSPI5_IRQ_PRIO;
#else
  INTC.PSR[SPC5_DSPI5_TFFF_NUMBER].R = SPC5_SPI_DSPI5_IRQ_PRIO;
#endif
#endif /* SPC5_SPI_USE_DSPI5 */

#if SPC5_SPI_USE_DSPI6
  /* Driver initialization.*/
  SPC5_DSPI6_ENABLE_CLOCK();
  spiObjectInit(&SPID7);
  spi_lld_obj_init(&SPID7, &SPC5_DSPI6);
  SPC5_DSPI6.MCR.R  = SPC5_MCR_MSTR | SPC5_MCR_HALT | SPC5_MCR_MDIS |
                      SPC5_SPI_DSPI6_MCR;
#if SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_NONE
  INTC.PSR[SPC5_DSPI6_RFDF_NUMBER].R = SPC5_SPI_DSPI6_IRQ_PRIO;
#else
  INTC.PSR[SPC5_DSPI6_TFFF_NUMBER].R = SPC5_SPI_DSPI6_IRQ_PRIO;
#endif
#endif /* SPC5_SPI_USE_DSPI6 */

#if SPC5_SPI_USE_DSPI7
  /* Driver initialization.*/
  SPC5_DSPI7_ENABLE_CLOCK();
  spiObjectInit(&SPID8);
  spi_lld_obj_init(&SPID8, &SPC5_DSPI7);
  SPC5_DSPI7.MCR.R  = SPC5_MCR_MSTR | SPC5_MCR_HALT | SPC5_MCR_MDIS |
                      SPC5_SPI_DSPI7_MCR;
#if SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_NONE
  INTC.PSR[SPC5_DSPI7_RFDF_NUMBER].R = SPC5_SPI_DSPI7_IRQ_PRIO;
#else
  INTC.PSR[SPC5_DSPI7_TFFF_NUMBER].R = SPC5_SPI_DSPI7_IRQ_PRIO;
#endif
#endif /* SPC5_SPI_USE_DSPI7 */
}

/**
 * @brief   Configures and activates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_start(SPIDriver *spip) {

  osalDbgAssert((spip->config->pushr & DSPI_PUSHR_EXCLUDED_BITS) == 0,
                "invalid PUSHR bits specified");

  if (spip->state == SPI_STOP) {
    /* Enables the peripheral.*/

#if SPC5_SPI_USE_DSPI0
    if (&SPID1 == spip) {
      spi_lld_setdma(spip,
                     spi_dspi0_tx1_dma_config,
                     spi_dspi0_tx2_dma_config,
                     spi_dspi0_rx_dma_config)
    }
#endif /* SPC5_SPI_USE_DSPI0 */

#if SPC5_SPI_USE_DSPI1
    if (&SPID2 == spip) {
      spi_lld_setdma(spip,
                     spi_dspi1_tx1_dma_config,
                     spi_dspi1_tx2_dma_config,
                     spi_dspi1_rx_dma_config)
    }
#endif /* SPC5_SPI_USE_DSPI1 */

#if SPC5_SPI_USE_DSPI2
    if (&SPID3 == spip) {
      spi_lld_setdma(spip,
                     spi_dspi2_tx1_dma_config,
                     spi_dspi2_tx2_dma_config,
                     spi_dspi2_rx_dma_config)
    }
#endif /* SPC5_SPI_USE_DSPI2 */

#if SPC5_SPI_USE_DSPI3
    if (&SPID4 == spip) {
      spi_lld_setdma(spip,
                     spi_dspi3_tx1_dma_config,
                     spi_dspi3_tx2_dma_config,
                     spi_dspi3_rx_dma_config)
    }
#endif /* SPC5_SPI_USE_DSPI3 */

#if SPC5_SPI_USE_DSPI4
    if (&SPID5 == spip) {
      spi_lld_setdma(spip,
                     spi_dspi4_tx1_dma_config,
                     spi_dspi4_tx2_dma_config,
                     spi_dspi4_rx_dma_config)
    }
#endif /* SPC5_SPI_USE_DSPI4 */

#if SPC5_SPI_USE_DSPI5
    if (&SPID6 == spip) {
      spi_lld_setdma(spip,
                     spi_dspi5_tx1_dma_config,
                     spi_dspi5_tx2_dma_config,
                     spi_dspi5_rx_dma_config)
    }
#endif /* SPC5_SPI_USE_DSPI5 */

#if SPC5_SPI_USE_DSPI6
    if (&SPID7 == spip) {
      spi_lld_setdma(spip,
                     spi_dspi6_tx1_dma_config,
                     spi_dspi6_tx2_dma_config,
                     spi_dspi6_rx_dma_config)
    }
#endif /* SPC5_SPI_USE_DSPI6 */

#if SPC5_SPI_USE_DSPI7
    if (&SPID8 == spip) {
      spi_lld_setdma(spip,
                     spi_dspi7_tx1_dma_config,
                     spi_dspi7_tx2_dma_config,
                     spi_dspi7_rx_dma_config)
    }
#endif /* SPC5_SPI_USE_DSPI7 */

#if SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_AND_TX
    osalDbgAssert((spip->tx1_channel != EDMA_ERROR) &&
                  (spip->tx2_channel != EDMA_ERROR),
                  "TX DMA channels cannot be allocated");
#endif
#if SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE
    osalDbgAssert(spip->rx_channel != EDMA_ERROR,
                  "RX DMA channels cannot be allocated");
#endif
  }

  /* Configures the peripheral, the RSER register setting depend on the
     chosen DMA use mode.*/
  spip->dspi->MCR.B.MDIS = 0;
  spip->dspi->CTAR[0].R  = spip->config->ctar0;
#if SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_NONE
  spip->dspi->RSER.R     = SPC5_RSER_RFDF_RE;
#endif
#if SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_ONLY
  spip->dspi->RSER.R     = SPC5_RSER_RFDF_RE | SPC5_RSER_RFDF_DIRS;
#endif
#if SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_AND_TX
  spip->dspi->RSER.R     = SPC5_RSER_TFFF_RE | SPC5_RSER_TFFF_DIRS |
                           SPC5_RSER_RFDF_RE | SPC5_RSER_RFDF_DIRS;
#endif
  spip->dspi->SR.R       = spip->dspi->SR.R;
}

/**
 * @brief   Deactivates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_stop(SPIDriver *spip) {

  if (spip->state == SPI_READY) {
    /* Releases the allocated EDMA channels.*/
#if SPC5_SPI_DMA_MODE == SPC5_SPI_DMA_RX_AND_TX
    edmaChannelRelease(spip->tx1_channel);
    edmaChannelRelease(spip->tx2_channel);
#endif
#if SPC5_SPI_DMA_MODE != SPC5_SPI_DMA_NONE
    edmaChannelRelease(spip->rx_channel);
#endif

    /* Resets the peripheral.*/
    spip->dspi->CTAR[0].R  = 0;
    spip->dspi->RSER.R     = 0;
    spip->dspi->SR.R       = spip->dspi->SR.R;
    spip->dspi->MCR.R     |= SPC5_MCR_HALT |
                             SPC5_MCR_CLR_TXF | SPC5_MCR_CLR_RXF;
    spip->dspi->MCR.B.MDIS = 1;
  }
}

/**
 * @brief   Asserts the slave select signal and prepares for transfers.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_select(SPIDriver *spip) {

  palClearPad(spip->config->ssport, spip->config->sspad);
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

  palSetPad(spip->config->ssport, spip->config->sspad);
}

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

  /* Starting transfer.*/
  spi_dspi_start(spip);

  /* Setting up the DMA channels.*/
  spi_start_rx_ignore(spip, n);
  spi_start_tx_ignore(spip, n);
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

  /* Starting transfer.*/
  spi_dspi_start(spip);

  /* DMAs require a different setup depending on the frame size.*/
  if (spip->dspi->CTAR[0].B.FMSZ < 8) {
    spi_start_rx8(spip, n, rxbuf);
    spi_start_tx8(spip, n, txbuf);
  }
  else {
    spi_start_rx16(spip, n, rxbuf);
    spi_start_tx16(spip, n, txbuf);
  }
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

  /* Starting transfer.*/
  spi_dspi_start(spip);

  /* Setting up the RX DMA channel.*/
  spi_start_rx_ignore(spip, n);

  /* DMAs require a different setup depending on the frame size.*/
  if (spip->dspi->CTAR[0].B.FMSZ < 8)
    spi_start_tx8(spip, n, txbuf);
  else
    spi_start_tx16(spip, n, txbuf);
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

  /* Starting transfer.*/
  spi_dspi_start(spip);

  /* DMAs require a different setup depending on the frame size.*/
  if (spip->dspi->CTAR[0].B.FMSZ < 8)
    spi_start_rx8(spip, n, rxbuf);
  else
    spi_start_rx16(spip, n, rxbuf);

  spi_start_tx_ignore(spip, n);
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
  uint32_t popr;

  /* Starting transfer.*/
  spi_dspi_start(spip);

  /* Data exchange.*/
  spip->dspi->PUSHR.R = (SPC5_PUSHR_EOQ | spip->config->pushr |
                         (uint32_t)frame) & ~SPC5_PUSHR_CONT;
  while (!spip->dspi->SR.B.RFDF)
    ;
  popr = spip->dspi->POPR.R;

  /* Stopping transfer.*/
  spi_dspi_stop(spip);

  return (uint16_t)popr;
}

#endif /* HAL_USE_SPI */

/** @} */
