/*
    ChibiOS - Copyright (C) 2016..2018 Theodore Ateba

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

BaseSequentialStream *chp = (BaseSequentialStream *) &SD1;
uint16_t cpt = 0; /* Counter for serial print.  */

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
  spiCallback,            /* SPI callback.            */
  IOPORT2,                /* SPI chip select port.    */
  7,                      /* SPI chip select pad.     */
  SPI_PRESCALER_DIV4_gc,  /* Prescaler.               */
  SPI_MODE_0_gc,          /* SPI clock mode.          */
  true,                   /* SPI role "master/slave". */
  false,                  /* SPI data order.          */
  false,                  /* SPI double speed bit.    */
  SPI_INTLVL_OFF_gc,      /* SPI interrupt level.     */
};

/**
 * @brief   Blink thread.
 */
static THD_WORKING_AREA(waThread1, 32);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("Blinker");

  while (true) {
    palClearPad(IOPORT5, PORTE_LED);
    chThdSleepMilliseconds(1000);
    palSetPad(IOPORT5, PORTE_LED);
    chThdSleepMilliseconds(100);
  }
}

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

  palClearPad(IOPORT5, PORTE_LED);

  /*
   * Configure TX (PINC3) and RX (PIN2) for the USART1.
   */
  palSetPadMode(IOPORT3, PIN3, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(IOPORT3, PIN2, PAL_MODE_INPUT_PULLUP);

  /*
   * Start the Serial driver 1 by using the default configuration.
   */
  sdStart(&SD1, NULL);

  /*
   * Activate the spi driver 1 by using the defined configuration.
   */
  spiStart(&SPID1, &spiCfg);

  /*
   * Starts the LED blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  chprintf(chp, "AVR SPI program testhal program example.\r\n");

  while (TRUE) {
    chprintf(chp, "Hello world %i.\r\n", cpt++);
    spiSelect(&SPID1);
    spiStartSend(&SPID1, 4, txbuf);
    spiUnselect(&SPID1);
    chThdSleepMilliseconds(1000);
  }

  return 0;
}
