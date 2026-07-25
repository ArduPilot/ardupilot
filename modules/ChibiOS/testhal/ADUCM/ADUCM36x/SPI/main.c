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

#include "ch.h"
#include "hal.h"

#define LINE_SPI1_MOSI              PAL_LINE(GP0, 2U)
#define LINE_SPI1_MISO              PAL_LINE(GP0, 0U)
#define LINE_SPI1_SCK               PAL_LINE(GP0, 1U)
#define LINE_SPI1_CS                PAL_LINE(GP0, 3U)

/*
 * SPI TX and RX buffers.
 */
static uint8_t txbuf[512];
static uint8_t rxbuf[512];

/*
 * Maximum speed SPI configuration (16MHz/8, CPHA=1, CPOL=1, MSb first).
 */
const SPIConfig hs_spicfg = {
  NULL,
  LINE_SPI1_CS,
  ADUCM_SPI_CON_CPOL | ADUCM_SPI_CON_CPHA,
  ADUCM_SPI_DIV_2
};

/*
 * Low speed SPI configuration (16MHz/256, CPHA=1, CPOL=1, MSb first).
 */
const SPIConfig ls_spicfg = {
  NULL,
  LINE_SPI1_CS,
  ADUCM_SPI_CON_CPOL | ADUCM_SPI_CON_CPHA,
  ADUCM_SPI_DIV_2 | ADUCM_SPI_DIV_1 | ADUCM_SPI_DIV_0
};

/*
 * SPI bus contender 1.
 */
static THD_WORKING_AREA(spi_thread_1_wa, 256);
static THD_FUNCTION(spi_thread_1, p) {

  (void)p;
  chRegSetThreadName("SPI thread 1");
  while (true) {
    spiAcquireBus(&SPID1);        /* Acquire ownership of the bus.    */
    palSetLine(LINE_LED_GREEN);
    spiStart(&SPID1, &hs_spicfg); /* Setup transfer parameters.       */
    spiSelect(&SPID1);            /* Slave Select assertion.          */
    spiExchange(&SPID1, 512,
                txbuf, rxbuf);    /* Atomic transfer operations.      */
    spiUnselect(&SPID1);          /* Slave Select de-assertion.       */
    spiReleaseBus(&SPID1);        /* Ownership release.               */
  }
}

/*
 * SPI bus contender 2.
 */
static THD_WORKING_AREA(spi_thread_2_wa, 256);
static THD_FUNCTION(spi_thread_2, p) {

  (void)p;
  chRegSetThreadName("SPI thread 2");
  while (true) {
    spiAcquireBus(&SPID1);        /* Acquire ownership of the bus.    */
    palClearLine(LINE_LED_GREEN);
    spiStart(&SPID1, &ls_spicfg); /* Setup transfer parameters.       */
    spiSelect(&SPID1);            /* Slave Select assertion.          */
    spiExchange(&SPID1, 512,
                txbuf, rxbuf);    /* Atomic transfer operations.      */
    spiUnselect(&SPID1);          /* Slave Select de-assertion.       */
    spiReleaseBus(&SPID1);        /* Ownership release.               */
  }
}

/*
 * LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {
  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    bool key_pressed = !palReadLine(LINE_BUTTON);
    systime_t time = key_pressed ? 250 : 500;
    palToggleLine(LINE_LED_BLUE);
    chThdSleepMilliseconds(time);
  }
}

/*
 * Application entry point.
 */
int main(void) {
  unsigned i;

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
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 1, Thread1, NULL);

  /*
   * Prepare transmit pattern.
   */
  for (i = 0; i < sizeof(txbuf); i++)
    txbuf[i] = (uint8_t)i;

  /*
   * Configuring SPI GPIOs. This requires S1 set to 1 in order to have
   * the SPI PINs connected to the MCU.
   */
  palSetLineMode(LINE_SPI1_CS, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(LINE_SPI1_MISO, PAL_MODE_MULTIPLEXER(1) |
                                 PAL_ADUCM_PUL_PULLUP);
  palSetLineMode(LINE_SPI1_MOSI, PAL_MODE_MULTIPLEXER(1) |
                                 PAL_ADUCM_PUL_PULLUP);
  palSetLineMode(LINE_SPI1_SCK, PAL_MODE_MULTIPLEXER(1) |
                                PAL_ADUCM_PUL_PULLUP);
  /*
   * Starting the transmitter and receiver threads.
   */
  chThdCreateStatic(spi_thread_1_wa, sizeof(spi_thread_1_wa),
                    NORMALPRIO - 1, spi_thread_1, NULL);
  chThdCreateStatic(spi_thread_2_wa, sizeof(spi_thread_2_wa),
                    NORMALPRIO - 1, spi_thread_2, NULL);

  /*
   * Normal main() thread activity, in this demo it does nothing.
   */
  while (true) {
    chThdSleepMilliseconds(500);
  }
  return 0;
}
