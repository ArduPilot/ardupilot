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
 * @brief   RP SPI subsystem low level driver source.
 *
 * @addtogroup SPI
 * @{
 */

#include "hal.h"

#if (HAL_USE_SPI == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   SPI0 driver identifier.
 */
#if (RP_SPI_USE_SPI0 == TRUE) || defined(__DOXYGEN__)
SPIDriver SPID0;
#endif

/**
 * @brief   SPI1 driver identifier.
 */
#if (RP_SPI_USE_SPI1 == TRUE) || defined(__DOXYGEN__)
SPIDriver SPID1;
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
 * @param[in] ct        content of the CTRL_TRIG register
 */
static void spi_lld_serve_rx_interrupt(SPIDriver *spip, uint32_t ct) {

  /* DMA errors handling.*/
  if ((ct & DMA_CTRL_TRIG_AHB_ERROR) != 0U) {

    /* Stopping DMAs.*/
    dmaChannelDisableX(spip->dmatx);
    dmaChannelDisableX(spip->dmarx);
#if defined(RP_SPI_DMA_ERROR_HOOK)
    RP_SPI_DMA_ERROR_HOOK(spip);
#endif
  }

  /* Portable SPI ISR code defined in the high level driver, note, it is
     a macro.*/
  _spi_isr_code(spip);
}

/**
 * @brief   Shared end-of-tx service routine.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] ct        content of the CTRL_TRIG register
 */
static void spi_lld_serve_tx_interrupt(SPIDriver *spip, uint32_t ct) {

  /* DMA errors handling.*/
  if ((ct & DMA_CTRL_TRIG_AHB_ERROR) != 0U) {

    /* Stopping DMAs.*/
    dmaChannelDisableX(spip->dmatx);
    dmaChannelDisableX(spip->dmarx);

#if defined(RP_SPI_DMA_ERROR_HOOK)
    RP_SPI_DMA_ERROR_HOOK(spip);
#endif

    /* A completion is enforced here, since the RX interrupt will no
       more happen.*/
    _spi_isr_code(spip);
  }
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

#if RP_SPI_USE_SPI0 == TRUE
  /* Driver initialization.*/
  spiObjectInit(&SPID0);
  SPID0.spi       = SPI0;
  SPID0.dmarx     = NULL;
  SPID0.dmatx     = NULL;
  SPID0.rxdmamode = DMA_CTRL_TRIG_TREQ_SPI0_RX |
                    DMA_CTRL_TRIG_PRIORITY(RP_SPI_SPI0_DMA_PRIORITY);
  SPID0.txdmamode = DMA_CTRL_TRIG_TREQ_SPI0_TX |
                    DMA_CTRL_TRIG_PRIORITY(RP_SPI_SPI0_DMA_PRIORITY);
#endif
#if RP_SPI_USE_SPI1 == TRUE
  /* Driver initialization.*/
  spiObjectInit(&SPID1);
  SPID1.spi       = SPI1;
  SPID1.dmarx     = NULL;
  SPID1.dmatx     = NULL;
  SPID1.rxdmamode = DMA_CTRL_TRIG_TREQ_SPI1_RX |
                    DMA_CTRL_TRIG_PRIORITY(RP_SPI_SPI1_DMA_PRIORITY);
  SPID1.txdmamode = DMA_CTRL_TRIG_TREQ_SPI1_TX |
                    DMA_CTRL_TRIG_PRIORITY(RP_SPI_SPI1_DMA_PRIORITY);
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
  uint32_t dss;

  if (spip->state == SPI_STOP) {

    if (false) {
    }
#if RP_SPI_USE_SPI0 == TRUE
    else if (&SPID0 == spip) {
      spip->dmarx = dmaChannelAllocI(RP_SPI_SPI0_RX_DMA_CHANNEL,
                                     RP_IRQ_SPI0_PRIORITY,
                                    (rp_dmaisr_t)spi_lld_serve_rx_interrupt,
                                    (void *)spip);
      osalDbgAssert(spip->dmarx != NULL, "unable to allocate stream");
      spip->dmatx = dmaChannelAllocI(RP_SPI_SPI0_TX_DMA_CHANNEL,
                                     RP_IRQ_SPI0_PRIORITY,
                                     (rp_dmaisr_t)spi_lld_serve_tx_interrupt,
                                     (void *)spip);
      osalDbgAssert(spip->dmatx != NULL, "unable to allocate stream");
      dmaChannelEnableInterruptX(spip->dmarx);
      dmaChannelEnableInterruptX(spip->dmatx);
      hal_lld_peripheral_unreset(RESETS_ALLREG_SPI0);
    }
#endif
#if RP_SPI_USE_SPI1 == TRUE
    else if (&SPID1 == spip) {
      spip->dmarx = dmaChannelAllocI(RP_SPI_SPI1_RX_DMA_CHANNEL,
                                     RP_IRQ_SPI1_PRIORITY,
                                    (rp_dmaisr_t)spi_lld_serve_rx_interrupt,
                                    (void *)spip);
      osalDbgAssert(spip->dmarx != NULL, "unable to allocate stream");
      spip->dmatx = dmaChannelAllocI(RP_SPI_SPI1_TX_DMA_CHANNEL,
                                     RP_IRQ_SPI1_PRIORITY,
                                     (rp_dmaisr_t)spi_lld_serve_tx_interrupt,
                                     (void *)spip);
      osalDbgAssert(spip->dmatx != NULL, "unable to allocate stream");
      dmaChannelEnableInterruptX(spip->dmarx);
      dmaChannelEnableInterruptX(spip->dmatx);
      hal_lld_peripheral_unreset(RESETS_ALLREG_SPI1);
    }
#endif
    else {
      osalDbgAssert(false, "invalid SPI instance");
    }

    /* DMA setup for SPI DR.*/
    dmaChannelSetSourceX(spip->dmarx, (uint32_t)&spip->spi->SSPDR);
    dmaChannelSetDestinationX(spip->dmatx, (uint32_t)&spip->spi->SSPDR);
  }

  /* Configuration-dependent DMA settings.*/
  dss = (spip->config->SSPCR0 & SPI_SSPCR0_DSS_Msk);
  if (dss <= SPI_SSPCR0_DSS_8BIT) {
    /* Frame width is 8 bits or smaller.*/
    spip->rxdmamode = (spip->rxdmamode & ~DMA_CTRL_TRIG_DATA_SIZE_Msk) |
                      DMA_CTRL_TRIG_DATA_SIZE_BYTE;
    spip->txdmamode = (spip->txdmamode & ~DMA_CTRL_TRIG_DATA_SIZE_Msk) |
                      DMA_CTRL_TRIG_DATA_SIZE_BYTE;
  }
  else {
    /* Frame width is larger than 8 bits.*/
    spip->rxdmamode = (spip->rxdmamode & ~DMA_CTRL_TRIG_DATA_SIZE_Msk) |
                      DMA_CTRL_TRIG_DATA_SIZE_HWORD;
    spip->txdmamode = (spip->txdmamode & ~DMA_CTRL_TRIG_DATA_SIZE_Msk) |
                      DMA_CTRL_TRIG_DATA_SIZE_HWORD;
  }

  /* SPI setup and enable.*/
  spip->spi->SSPCR1   = 0U;
  spip->spi->SSPCR0   = spip->config->SSPCR0;
  spip->spi->SSPCPSR  = spip->config->SSPCPSR;
  spip->spi->SSPDMACR = SPI_SSPDMACR_RXDMAE | SPI_SSPDMACR_TXDMAE;
  spip->spi->SSPCR1   = SPI_SSPCR1_SSE;
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

    /* SPI disables.*/
    spip->spi->SSPCR1  = 0U;
    dmaChannelFreeI(spip->dmarx);
    dmaChannelFreeI(spip->dmatx);
    spip->dmarx = NULL;
    spip->dmatx = NULL;

    if (false) {
    }
#if RP_SPI_USE_SPI0 == TRUE
    else if (&SPID0 == spip) {
      hal_lld_peripheral_reset(RESETS_ALLREG_SPI0);
    }
#endif
#if RP_SPI_USE_SPI1 == TRUE
    else if (&SPID1 == spip) {
      hal_lld_peripheral_reset(RESETS_ALLREG_SPI1);
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

  /* No implementation on RP.*/
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

  /* No implementation on RP.*/
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

  dmaChannelSetDestinationX(spip->dmarx, (uint32_t)&dummyrx);
  dmaChannelSetCounterX(spip->dmarx, (uint32_t)n);
  dmaChannelSetModeX(spip->dmarx, spip->rxdmamode);

  dmaChannelSetSourceX(spip->dmatx, (uint32_t)&dummytx);
  dmaChannelSetCounterX(spip->dmatx, (uint32_t)n);
  dmaChannelSetModeX(spip->dmatx, spip->txdmamode);

  dmaChannelEnableX(spip->dmarx);
  dmaChannelEnableX(spip->dmatx);
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

  dmaChannelSetDestinationX(spip->dmarx, (uint32_t)rxbuf);
  dmaChannelSetCounterX(spip->dmarx, (uint32_t)n);
  dmaChannelSetModeX(spip->dmarx, spip->rxdmamode | DMA_CTRL_TRIG_INCR_WRITE);

  dmaChannelSetSourceX(spip->dmatx, (uint32_t)txbuf);
  dmaChannelSetCounterX(spip->dmatx, (uint32_t)n);
  dmaChannelSetModeX(spip->dmatx, spip->txdmamode | DMA_CTRL_TRIG_INCR_READ);

  dmaChannelEnableX(spip->dmarx);
  dmaChannelEnableX(spip->dmatx);
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

  dmaChannelSetDestinationX(spip->dmarx, (uint32_t)&dummyrx);
  dmaChannelSetCounterX(spip->dmarx, (uint32_t)n);
  dmaChannelSetModeX(spip->dmarx, spip->rxdmamode);

  dmaChannelSetSourceX(spip->dmatx, (uint32_t)txbuf);
  dmaChannelSetCounterX(spip->dmatx, (uint32_t)n);
  dmaChannelSetModeX(spip->dmatx, spip->txdmamode | DMA_CTRL_TRIG_INCR_READ);

  dmaChannelEnableX(spip->dmarx);
  dmaChannelEnableX(spip->dmatx);
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

  dmaChannelSetDestinationX(spip->dmarx, (uint32_t)rxbuf);
  dmaChannelSetCounterX(spip->dmarx, (uint32_t)n);
  dmaChannelSetModeX(spip->dmarx, spip->rxdmamode | DMA_CTRL_TRIG_INCR_WRITE);

  dmaChannelSetSourceX(spip->dmatx, (uint32_t)&dummytx);
  dmaChannelSetCounterX(spip->dmatx, (uint32_t)n);
  dmaChannelSetModeX(spip->dmatx, spip->txdmamode);

  dmaChannelEnableX(spip->dmarx);
  dmaChannelEnableX(spip->dmatx);
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

  (void)spip;
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
uint16_t spi_lld_polled_exchange(SPIDriver *spip, uint16_t frame) {

  (void)spip;
  (void)frame;

  return 0;
}

#endif /* HAL_USE_SPI == TRUE */

/** @} */
