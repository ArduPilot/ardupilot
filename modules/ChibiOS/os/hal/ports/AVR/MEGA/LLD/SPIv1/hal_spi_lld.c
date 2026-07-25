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
 * @brief   AVR/MEGA SPI subsystem low level driver source.
 *
 * @addtogroup SPI
 * @{
 */

#include "hal.h"

#if HAL_USE_SPI || defined(__DOXYGEN__)

/*==========================================================================*/
/* Driver local definitions.                                                */
/*==========================================================================*/

#define DUMMY_SPI_SEND_VALUE    0xFF

/*==========================================================================*/
/* Driver exported variables.                                               */
/*==========================================================================*/

/**
 * @brief   SPI1 driver identifier.
 */
#if AVR_SPI_USE_SPI1 || defined(__DOXYGEN__)
SPIDriver SPID1;
#endif

/*==========================================================================*/
/* Driver local variables and types.                                        */
/*==========================================================================*/

/*==========================================================================*/
/* Driver local functions.                                                  */
/*==========================================================================*/

/*==========================================================================*/
/* Driver interrupt handlers.                                               */
/*==========================================================================*/

#if AVR_SPI_USE_SPI1 || defined(__DOXYGEN__)
/**
 * @brief   SPI event interrupt handler.
 *
 * @notapi
 */
OSAL_IRQ_HANDLER(SPI_STC_vect) {
  OSAL_IRQ_PROLOGUE();

  SPIDriver *spip = &SPID1;

  /* A new value has arrived, store it if we are interested in it. */
  if (spip->rxbuf) spip->rxbuf[spip->exidx] = SPDR;

  /* Check if we are done. */
  if (++(spip->exidx) >= spip->exbytes) {
    _spi_isr_code(spip);
  } else { /* If not done send the next byte. */
     if (spip->txbuf) { /* If there is a buffer with values to be send then use it. */
       SPDR = spip->txbuf[spip->exidx];
    } else {            /* If there isn't a buffer with values to be send then send a the dummy value. */
      SPDR = DUMMY_SPI_SEND_VALUE;
    }
  }
  OSAL_IRQ_EPILOGUE();
}
#endif /* AVR_SPI_USE_SPI1 */

/*==========================================================================*/
/* Driver exported functions.                                               */
/*==========================================================================*/

/**
 * @brief   Low level SPI driver initialization.
 *
 * @notapi
 */
void spi_lld_init(void) {

#if AVR_SPI_USE_SPI1
  /* Driver initialization. */
  spiObjectInit(&SPID1);
#endif /* AVR_SPI_USE_SPI1 */
}

/**
 * @brief   Configures and activates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_start(SPIDriver *spip) {

  uint8_t dummy;

  if (spip->state == SPI_STOP) {
    /* Enables the peripheral. */
#if AVR_SPI_USE_SPI1
    if (&SPID1 == spip) {
    /* Enable SPI clock using Power Reduction Register. */
#if defined(PRR0)
      PRR0 &= ~(1 << PRSPI);
#elif defined(PRR)
      PRR &= ~(1 << PRSPI);
#endif
#endif
    }
  }

#if AVR_SPI_USE_SPI1
  if (&SPID1 == spip) {
    /* Configures the peripheral. */
    /* Note that some bits are forced:
    SPI interrupt disabled,
    SPI enabled,
    SPI master enabled. */
    SPCR = (spip->config->spcr & ~(SPI_CR_SPIE)) | SPI_CR_MSTR | SPI_CR_SPE;
    SPSR = spip->config->spsr;

    /* Dummy reads before enabling interrupt. */
    dummy = SPSR;
    dummy = SPDR;
    (void) dummy; /* Suppress warning about unused variable. */

    /* Enable SPI interrupts. */
    SPCR |= SPI_CR_SPIE;
  }
#endif /* AVR_SPI_USE_SPI1 */
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
    /* Resets the peripheral. */

    /* Disables the peripheral. */
#if AVR_SPI_USE_SPI1
    if (&SPID1 == spip) {
      SPCR &= (SPI_CR_SPIE | SPI_CR_SPE);
    }
/* Disable SPI clock using Power Reduction Register. */
#if defined(PRR0)
      PRR0 |= (1 << PRSPI);
#elif defined(PRR)
      PRR |= (1 << PRSPI);
#endif
#endif /* AVR_SPI_USE_SPI1 */
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

  /**
   * NOTE: This should only be called in master mode.
   */
  spip->config->ssport->out &= ~(1 << spip->config->sspad);

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

  /**
   * NOTE: This should only be called in master mode.
   */
  spip->config->ssport->out |= (1 << spip->config->sspad);

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
void spi_lld_exchange(SPIDriver *spip, size_t n, const void *txbuf, void *rxbuf) {

  spip->txbuf = txbuf;
  spip->rxbuf = rxbuf;
  spip->exidx = 0;
  spip->exbytes = n;
  SPDR = (spip->txbuf ? spip->txbuf[0] : DUMMY_SPI_SEND_VALUE);
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
#if AVR_SPI_USE_16BIT_POLLED_EXCHANGE
uint16_t spi_lld_polled_exchange(SPIDriver *spip, uint16_t frame) {

  uint16_t spdr = 0;
  uint8_t dummy;
  (void)spip;

  /* Disable interrupt. */
  SPCR &= ~(SPI_CR_SPIE);

  SPDR = frame >> 8;
  while (!(SPSR & SPI_SR_SPIF));
  spdr = SPDR << 8;

  SPDR = frame & 0xFF;
  while (!(SPSR & SPI_SR_SPIF));
  spdr |= SPDR;

  dummy = SPSR;
  dummy = SPDR;
  (void) dummy; /* Suppress warning about unused variable. */
  SPCR |= SPI_CR_SPIE;

  return spdr;
}
#else /* AVR_SPI_USE_16BIT_POLLED_EXCHANGE */
uint8_t spi_lld_polled_exchange(SPIDriver *spip, uint8_t frame) {

  uint8_t spdr = 0;
  uint8_t dummy;
  (void)spip;

  /* Disable interrupt. */
  SPCR &= ~(SPI_CR_SPIE);

  SPDR = frame;
  while (!(SPSR & SPI_SR_SPIF));
  spdr = SPDR;

  dummy = SPSR;
  dummy = SPDR;
  (void) dummy; /* Suppress warning about unused variable. */
  SPCR |= SPI_CR_SPIE;

  return spdr;
}
#endif /* AVR_SPI_USE_16BIT_POLLED_EXCHANGE */

#endif /* HAL_USE_SPI */

/** @} */
