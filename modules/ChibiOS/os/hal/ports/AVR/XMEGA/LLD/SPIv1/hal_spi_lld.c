/*
    ChibiOS - Copyright (C) 2015..2021 Theodore Ateba

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

/**
 * @brief   SPI2 driver identifier.
 */
#if AVR_SPI_USE_SPI2 || defined(__DOXYGEN__)
SPIDriver SPID2;
#endif

/*==========================================================================*/
/* Driver local variables and types.                                        */
/*==========================================================================*/

/*==========================================================================*/
/* Driver local functions.                                                  */
/*==========================================================================*/

/* Configure the speed of the SPI interface. */
/*
Static void spi_set_speed(uint8_t ds) {
  We must make a test to see if we are in master mode.

  ds = double speed;
  if (ds == SPI_SPEED_DOUBLE) {
    SPIC.CTRL |= (1 << SPI_CLK2X_bp);  double speed.
  }
  else {
    SPIC.CTRL &= ~(1 << SPI_CLK2X_bp); simple speed.
  }
}
*/

/* Enable the SPI module. */
/*
Static void spi_lld_enable(SPIDriver *spip) {
  spip->spi->CTRL |= (1 << SPI_ENABLE_bp);
}
*/

/* Disable the SPI interface. */
/*
Static void spi_disable(void) {
  SPIC.CTRL &= ~(1 << SPI_ENABLE_bp);
}
*/

/* Configure the SPI bit order, LSB/MSB first. */
/*
Static void spi_set_bit_order(uint8_t bo) {
  // bo = bit order
  if (bo == SPI_MSB_FIRST)
    SPIC.CTRL &= ~(1 << SPI_DORD_bp);
  else
    SPIC.CTRL |= (1 << SPI_DORD_bp);
}
*/

/* Configure the SPI interface to Master or slave. */
/*
Static void spi_set_mode(uint8_t mode) {
  if (mode == SPI_MODE_SLAVE)
    SPIC.CTRL &= ~(1 << SPI_MASTER_bp);
  else
    SPIC.CTRL |= (1 << SPI_MASTER_bp);
}
*/

/*
Static void spi_set_stransfer_mode(SPI_MODE_t mode) {
  switch(mode) {
    case SPI_TRANSFER_MODE0:
      SPIC.CTRL = (SPIC.CTRL & ~SPI_MODE_gm) | SPI_MODE_0_gc;
    break;

    case SPI_TRANSFER_MODE1:
      SPIC.CTRL = (SPIC.CTRL & ~SPI_MODE_gm) | SPI_MODE_1_gc;
    break;

    case SPI_TRANSFER_MODE2:
      SPIC.CTRL = (SPIC.CTRL & ~SPI_MODE_gm) | SPI_MODE_2_gc;
    break;

    case SPI_TRANSFER_MODE3:
      SPIC.CTRL = (SPIC.CTRL & ~SPI_MODE_gm) | SPI_MODE_3_gc;
    break;

    default:
      SPIC.CTRL = (SPIC.CTRL & ~SPI_MODE_gm) | SPI_MODE_0_gc;
    break;
  }
}
*/

/**
 * @brief   Configure the SPI colck from the System clock.
 *
 * @param[in] prescaler   the prescaler used to divide the system clock
 */
/*
Static void spi_set_clock_prescaler(SPI_PRESCALER_t prescaler) {
  switch(prescaler) {
    case SPI_PRESCALER_4:
      SPIC.CTRL = (SPIC.CTRL & ~SPI_PRESCALER_gm) | SPI_PRESCALER_DIV4_gc;
    break;

    case SPI_PRESCALER_16:
      SPIC.CTRL = (SPIC.CTRL & ~SPI_PRESCALER_gm) | SPI_PRESCALER_DIV4_gc;
    break;

    case SPI_PRESCALER_64:
      SPIC.CTRL = (SPIC.CTRL & ~SPI_PRESCALER_gm) | SPI_PRESCALER_DIV4_gc;
    break;

    case SPI_PRESCALER_128:
      SPIC.CTRL = (SPIC.CTRL & ~SPI_PRESCALER_gm) | SPI_PRESCALER_DIV4_gc;
    break;

    default:
    break;
  }
}
*/

/**
 * @brief   Enable/Disable the interruption and set the interrupt level.
 *
 * @param[in] il  the interrupt level
 */
/*
Static void spi_set_irq_level(SPI_INTLVL_t il) {
  switch(il) {
    case SPI_INT_DISABLE:
      SPIC.INTCTRL = (SPIC.INTCTRL & ~SPI_INTLVL_gm) | SPI_INTLVL_OFF_gc;
    break;

    case SPI_INT_LEVEL_LOW:
      SPIC.INTCTRL = (SPIC.INTCTRL & ~SPI_INTLVL_gm) | SPI_INTLVL_LO_gc;
    break;

    case SPI_INT_LEVEL_MEDIUM:
      SPIC.INTCTRL = (SPIC.INTCTRL & ~SPI_INTLVL_gm) | SPI_INTLVL_MED_gc;
    break;

    case SPI_INT_LEVEL_HIGH:
      SPIC.INTCTRL = (SPIC.INTCTRL & ~SPI_INTLVL_gm) | SPI_INTLVL_HI_gc;
    break;

    default:
      SPIC.INTCTRL = (SPIC.INTCTRL & ~SPI_INTLVL_gm) | SPI_INTLVL_OFF_gc;
    break;
  }
}

Void spi_send_byte(uint8_t data) {
  SPIC.DATA = data;
  while(!(SPIC.STATUS & SPI_IF_bm));
}
*/

/*
 *
 * Spi_set_speed(SPI_SPEED_SIMPLE);            Configure the speed double.
 * Spi_set_bit_order(SPI_MSB_FIRST);           Configure the bit order.
 * Spi_set_mode(SPI_MODE_MASTER);              Configure the mode to master.
 * Spi_set_stransfer_mode(SPI_TRANSFER_MODE0); Configure the transfer mode.
 * Spi_set_clock_prescaler(SPI_PRESCALER_4);   Configure the clock prescaler.
 * Spi_set_irq_level(SPI_INT_LEVEL_LOW);       Configure the irq level.
 * Spi_enable();                               Enable the SPI interface.
 *
 * Spi_select();                               For the chip select.
 * Spi_send_byte(0xAA);                        For the chip select.
 * Spi_deselect();                             For the chip select.
 */

/*==========================================================================*/
/* Driver interrupt handlers.                                               */
/*==========================================================================*/

#if AVR_SPI_USE_SPI1 || defined(__DOXYGEN__)
/**
 * @brief   SPI event interrupt handler.
 *
 * @notapi
 *//*
OSAL_IRQ_HANDLER(SPI_STC_vect) {
  OSAL_IRQ_PROLOGUE();

  SPIDriver *spip = &SPID1;

  // A new value has arrived, store it if we are interested in it.
  if (spip->rxbuf) spip->rxbuf[spip->exidx] = SPDR;

  // Check if we are done.
  if (++(spip->exidx) >= spip->exbytes) {
    _spi_isr_code(spip);
  } else { // If not done send the next byte.
     if (spip->txbuf) { // If there is a buffer with values to be send then use it.
       SPDR = spip->txbuf[spip->exidx];
    } else {            // If there isn't a buffer with values to be send then send a the dummy value.
      SPDR = DUMMY_SPI_SEND_VALUE;
    }
  }
  OSAL_IRQ_EPILOGUE();
}*/
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
  SPID1.spi = &SPIC;
#endif /* AVR_SPI_USE_SPI1 */

#if AVR_SPI_USE_SPI2
  /* Driver initialization. */
  spiObjectInit(&SPID2);
  SPID2.spi = &SPID;
#endif /* AVR_SPI_USE_SPI2 */
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

  if (&SPID1 == spip) {
    /* Configures the peripheral,       */
    /* Note that some bits are forced,  */
    /* SPI interrupt disabled,          */
    /* SPI enabled,                     */
    /* SPI master enabled,              */

    spip->spi->INTCTRL = SPI_INTLVL_OFF_gc;

    spip->spi->CTRL = (spip->config->clk2x ? SPI_CLK2X_bm : 0)    |
                      (SPI_ENABLE_bm)                             |
                      (spip->config->dord ? SPI_DORD_bm : 0)      |
                      (spip->config->master ? SPI_MASTER_bm : 0)  |
                      (spip->config->mode)                        |
                      (spip->config->prescaler);

    /* Dummy reads before enabling interrupt. */
    dummy = spip->spi->STATUS;
    dummy = spip->spi->DATA;
    (void) dummy; /* Suppress warning about unused variable. */

    /* Enable SPI interrupts. */
    spip->spi->INTCTRL = spip->config->irqlevel;
  }
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
    /* Disable the peripheral. */
    spip->spi->CTRL &= ~(1 << SPI_ENABLE_bp);
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
  /* Spip->config->ssport->out &= ~(1 << spip->config->sspad); */

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
  /*
   * Spip->config->ssport->out |= (1 << spip->config->sspad);
   */

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
  spip->spi->DATA = (spip->txbuf ? spip->txbuf[0] : DUMMY_SPI_SEND_VALUE);
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
uint8_t spi_lld_polled_exchange(SPIDriver *spip, uint8_t frame) {

  uint8_t spdr = 0;
  uint8_t dummy;
  (void)spip;

  /* Disable interrupt. */
  spip->spi->INTCTRL = SPI_INTLVL_OFF_gc;

  spip->spi->DATA = frame;
  while (!(spip->spi->STATUS & SPI_IF_bm));
  spdr = spip->spi->DATA;

  dummy = spip->spi->STATUS;
  dummy = spip->spi->DATA;
  (void) dummy; /* Suppress warning about unused variable. */
  spip->spi->INTCTRL = SPI_INTLVL_LO_gc;

  return spdr;
}

#endif /* HAL_USE_SPI */

/** @} */
