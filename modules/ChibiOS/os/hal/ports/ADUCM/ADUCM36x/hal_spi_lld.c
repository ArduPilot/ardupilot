/*
    ChibiOS - Copyright (C) 2006..2020 Rocco Marco Guglielmi

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
 * @file    ADUCM36x/hal_spi_lld.c
 * @brief   ADUCM SPI subsystem low level driver source.
 *
 * @addtogroup SPI
 * @{
 */

#include "hal.h"

#if HAL_USE_SPI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define ADUCM_SPI_DIV_MASK              0x003FU

#define ADUCM_SPI_STA_IRQ               0x0001U
#define ADUCM_SPI_STA_TX                0x0020U

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief SPI0 driver identifier.*/
#if ADUCM_SPI_USE_SPI0 || defined(__DOXYGEN__)
SPIDriver SPID0;
#endif

/** @brief SPI1 driver identifier.*/
#if ADUCM_SPI_USE_SPI1 || defined(__DOXYGEN__)
SPIDriver SPID1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Shared interrupt service routine.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 */
static void spi_lld_serve_interrupt(SPIDriver *spip) {

  uint32_t sta = spip->spi->SPISTA;
  uint8_t dummy_rx;
  uint8_t dummy_tx = 0xFFU;

  if((sta & ADUCM_SPI_STA_TX) && (spip->size > 0)) {
    /* Decreasing the size. */
    (spip->size)--;

    /* Retrieving the RX. */
    if(spip->rxbuf != NULL) {
      *(spip->rxbuf) = spip->spi->SPIRX;
      (spip->rxbuf)++;
    }
    else {
      dummy_rx = spip->spi->SPIRX;
    }
    
    /* Pushing the new TX: this will start a new transfer. */
    if(spip->size > 0) {
      if(spip->txbuf != NULL) {
        (spip->txbuf)++;
        spip->spi->SPITX = *(spip->txbuf);
      }
      else {
        spip->spi->SPITX = dummy_tx;
      }
    }
    else {
      /* Portable SPI ISR code defined in the high level driver, note, it is
         a macro.*/
      _spi_isr_code(spip);
    }

    (void)dummy_rx;
  }
}

/**
 * @brief   Utility function useful to clean-up the SPI cell.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 */
static void spi_lld_reset_spi_cell(SPIDriver* spip) {

  uint32_t sta;

  /* Disabling the SPI. And flushing RX and TX buffers. */
  spip->spi->SPICON = ADUCM_SPI_CON_RFLUSH | ADUCM_SPI_CON_TFLUSH;
  spip->spi->SPICON = 0;

  /* Cleaning IRQs. */
  sta = spip->spi->SPISTA;
  (void) sta;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if ADUCM_SPI_USE_SPI0 || defined(__DOXYGEN__)
#if !defined(ADUCM_SPI0_HANDLER)
#error "ADUCM_SPI0_HANDLER not defined"
#endif
/**
 * @brief   SPI0 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(ADUCM_SPI0_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spi_lld_serve_interrupt(&SPID0);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if ADUCM_SPI_USE_SPI1 || defined(__DOXYGEN__)
#if !defined(ADUCM_SPI1_HANDLER)
#error "ADUCM_SPI1_HANDLER not defined"
#endif
/**
 * @brief   SPI1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(ADUCM_SPI1_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spi_lld_serve_interrupt(&SPID1);

  OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level SPI driver initialization.
 *
 * @notapi
 */
void spi_lld_init(void) {

#if ADUCM_SPI_USE_SPI0
  spiObjectInit(&SPID0);
  SPID0.spi       = pADI_SPI0;
  SPID0.rxbuf     = NULL;
  SPID0.txbuf     = NULL;
  SPID0.size      = 0;
#endif

#if ADUCM_SPI_USE_SPI1
  spiObjectInit(&SPID1);
  SPID1.spi       = pADI_SPI1;
  SPID1.rxbuf     = NULL;
  SPID1.txbuf     = NULL;
  SPID1.size      = 0;
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
#if ADUCM_SPI_USE_SPI0
    if (&SPID0 == spip) {
      /* Enabling peripheral clock branch. */
      ccEnableSPI0();

      /* Resetting the SPI cell. */
      spi_lld_reset_spi_cell(spip);
      
      /* Enabling peripheral interrupt. */
      nvicEnableVector(ADUCM_SPI0_NUMBER, ADUCM_SPI_SPI0_IRQ_PRIORITY);
    }
#endif
#if ADUCM_SPI_USE_SPI1
    if (&SPID1 == spip) {
      /* Enabling peripheral clock branch. */
      ccEnableSPI1();
      
      /* Resetting the SPI cell. */
      spi_lld_reset_spi_cell(spip);

      /* Enabling peripheral interrupt. */
      nvicEnableVector(ADUCM_SPI1_NUMBER, ADUCM_SPI_SPI1_IRQ_PRIORITY);
    }
#endif
  }

  /* SPI clock divider configuration.*/
  spip->spi->SPIDIV = (spip->config->div & ADUCM_SPI_DIV_MASK);
  
  /* SPI enabling and configuration. Note that some configuration are
     enforced to ensure the IRQ proper behavior. */
  spip->spi->SPICON = spip->config->con | ADUCM_SPI_CON_TIM |
                      ADUCM_SPI_CON_MASEN;
  spip->spi->SPICON &= ~ADUCM_SPI_CON_MOD_MASK;
  spip->spi->SPICON |= ADUCM_SPI_CON_ENABLE;
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
    spip->spi->SPICON = 0;
    spip->spi->SPIDIV = 0;
    spip->rxbuf = NULL;
    spip->txbuf = NULL;
    spip->size = 0;

    /* Resetting the SPI cell. */
    spi_lld_reset_spi_cell(spip);

#if ADUCM_SPI_USE_SPI0
    if (&SPID0 == spip) {
      /* Disabling peripheral clock branch. */
      ccDisableSPI0();

      /* Disabling peripheral interrupt. */
      nvicDisableVector(ADUCM_SPI0_NUMBER);
    }
#endif
#if ADUCM_SPI_USE_SPI1
    if (&SPID1 == spip) {
      /* Disabling peripheral clock branch. */
      ccDisableSPI1();

      /* Disabling peripheral interrupt. */
      nvicDisableVector(ADUCM_SPI1_NUMBER);
    }
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

  /* No implementation on ADUCM.*/
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

  /* No implementation on ADUCM.*/
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
  uint8_t dummy_tx = 0xFFU;

  spip->txbuf = NULL;
  spip->rxbuf = NULL;
  spip->size = n;
  spip->spi->SPITX = dummy_tx;
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
  spip->txbuf = (uint8_t*)txbuf;
  spip->rxbuf = (uint8_t*)rxbuf;
  spip->size = n;
  spip->spi->SPITX = *(spip->txbuf);
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
  spip->txbuf = (uint8_t*)txbuf;
  spip->rxbuf = NULL;
  spip->size = n;
  spip->spi->SPITX = *(spip->txbuf);
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
  uint8_t dummy_tx = 0xFFU;

  spip->txbuf = NULL;
  spip->rxbuf = (uint8_t*) rxbuf;
  spip->size = n;
  spip->spi->SPITX = dummy_tx;
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
uint8_t spi_lld_polled_exchange(SPIDriver *spip, uint8_t frame) {
  uint32_t sta = spip->spi->SPISTA;

#if ADUCM_SPI_USE_SPI0
    if (&SPID0 == spip) {
      /* Disabling ISR. */
      nvicDisableVector(ADUCM_SPI0_NUMBER);
    }
#endif

#if ADUCM_SPI_USE_SPI1
    if (&SPID1 == spip) {
      /* Disabling ISR. */
      nvicDisableVector(ADUCM_SPI1_NUMBER);
    }
#endif

  spip->spi->SPITX = frame;
  while((sta & (ADUCM_SPI_STA_TX | ADUCM_SPI_STA_IRQ))) {
    sta = spip->spi->SPISTA;
  }

#if ADUCM_SPI_USE_SPI0
    if (&SPID0 == spip) {
      /* Re-enabling peripheral interrupt. */
      nvicEnableVector(ADUCM_SPI0_NUMBER, ADUCM_SPI_SPI0_IRQ_PRIORITY);
    }
#endif

#if ADUCM_SPI_USE_SPI1
    if (&SPID1 == spip) {
      /* Re-enabling peripheral interrupt. */
      nvicEnableVector(ADUCM_SPI1_NUMBER, ADUCM_SPI_SPI1_IRQ_PRIORITY);
    }
#endif

  return spip->spi->SPIRX;
}

#endif /* HAL_USE_SPI */

/** @} */
