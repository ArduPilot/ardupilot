/*
    ChibiOS - Copyright (C) 2016 Theodore Ateba

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

#include "ch.h"
#include "hal.h"
#include "chprintf.h"

/**
 * @brief Global variables
 */
BaseSequentialStream * chp = (BaseSequentialStream *) &SD1;

/*
 * @brief   Data to transmit over the spi bus.
 */
char txbuf[] = "ABCD";

/**
 * @brief   Spi callback.
 */
void spiCallback(SPIDriver *spip) {

  chnWrite(&SD1, (const uint8_t *)"*", 1);
  spip->txbuf = 0;
};

/**
 * @biref   SPI configuration strucrture.
 */
static const SPIConfig spiCfg = {
  false,
  spiCallback,                  /* SPI callback.                  */
  IOPORT2,                      /* SPI chip select port.          */
  7,                            /* SPI chip select pad.           */
  SPI_CR_DORD_MSB_FIRST     |   /* SPI Data order.                */
  SPI_CR_CPOL_CPHA_MODE(0)  |   /* SPI clock polarity and phase.  */
  SPI_CR_SCK_FOSC_128,          /* SPI clock.                     */
  SPI_SR_SCK_FOSC_2             /* SPI double speed bit.          */
};

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * Activates the serial driver 1 using the driver default configuration.
   */
  sdStart(&SD1, NULL);

  /*
   * Activate the spi driver 1 by using the defined configuration.
   */
  spiStart(&SPID1, &spiCfg);

  chprintf(chp, "AVR SPI program testhal program example.\r\n");

  while (TRUE) {
    spiSelect(&SPID1);
    spiStartSend(&SPID1, 4, txbuf);
    spiUnselect(&SPID1);
    chThdSleepMilliseconds(1000);
  }

  /*
   * Stop the spi driver.
   */
  spiStop(&SPID1);
}

