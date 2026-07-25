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

#define SPI3_SELECT() {                                                     \
  PWR->SUBGHZSPICR = 0U;                                                    \
}

#define SPI3_UNSELECT() {                                                   \
  PWR->SUBGHZSPICR = PWR_SUBGHZSPICR_NSS;                                   \
}

#define RADIO_GET_STATUS_OPCODE               0xC0U

/*
 * SPI3(SUBGHZSPI) configuration (12MHz, CPHA=0, CPOL=0, MSb first).
 */
static const SPIConfig spicfg = {
  .circular = false,
  .slave    = false,
  .data_cb  = NULL,
  .error_cb = NULL,
  .cr1      = (SPI_CR1_BR_0),
  .cr2      = (0U)
};

void radioReset(void) {

  /* Set RF reset bit.*/
  RCC->CSR |= RCC_CSR_RFRST;
  chThdSleepMilliseconds(20);
  /* Clear RF reset bit.*/
  RCC->CSR &= ~RCC_CSR_RFRST;

  while ((RCC->CSR & RCC_CSR_RFRSTF) != 0) {
    /* Wait for RF reset.*/
  }

  SPI3_UNSELECT();
  /* Clear wakeup RF busy flag.*/
  PWR->SCR |= PWR_SCR_CWRFBUSYF;

  chThdSleepMilliseconds(100);
}


/*
 * Application entry point.
 */
int main(void) {

  uint8_t opcode = RADIO_GET_STATUS_OPCODE;
  uint8_t dummy;
  uint8_t status;

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  radioReset();

  spiAcquireBus(&SPID3);                /* Acquire ownership of the bus.    */
  spiStart(&SPID3, &spicfg);            /* Setup transfer parameters.       */
  SPI3_SELECT();                        /* Slave Select assertion.          */
  spiExchange(&SPID3, 1,
              &opcode, &dummy);         /* Atomic transfer operations.      */
  spiExchange(&SPID3, 1,
              &dummy, &status);         /* Atomic transfer operations.      */
  SPI3_UNSELECT();                      /* Slave Select de-assertion.       */
  spiReleaseBus(&SPID3);                /* Ownership release.               */

  /* Normal main() thread activity, nothing in this test.*/
  while (true) {
    chThdSleepMilliseconds(500);
  }
}
