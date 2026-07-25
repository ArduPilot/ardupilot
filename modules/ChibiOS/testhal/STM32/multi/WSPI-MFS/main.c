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

#include <string.h>

#include "ch.h"
#include "hal.h"

#include "hal_xsnor_micron_n25q.h"
#include "hal_xsnor_macronix_mx25.h"
#include "hal_mfs.h"

#include "mfs_test_root.h"

#include "portab.h"

static union {
  hal_xsnor_micron_n25q_c           n25q;
  hal_xsnor_macronix_mx25_c         mx25;
} snor1;

static xsnor_buffers_t __nocache_snor1buf;

static const xsnor_config_t snorcfg_n25q = {
  .bus_type         = XSNOR_BUS_MODE_WSPI_4LINES,
  .bus.wspi.drv     = &PORTAB_WSPI1,
  .bus.wspi.cfg     = &WSPIcfg1,
  .buffers          = &__nocache_snor1buf,
  .options          = N25Q_OPT_DUMMY_CYCLES(8) |
                      N25Q_OPT_USE_SUBSECTORS |
                      N25Q_OPT_NICE_WAITING
};

static const xsnor_config_t snorcfg_mx25 = {
  .bus_type         = XSNOR_BUS_MODE_WSPI_8LINES,
  .bus.wspi.drv     = &PORTAB_WSPI1,
  .bus.wspi.cfg     = &WSPIcfg1,
  .buffers          = &__nocache_snor1buf,
  .options          = MX25_OPT_DUMMY_CYCLES(14) |   /* Very conservative.*/
                      MX25_OPT_USE_SUBSECTORS |
                      MX25_OPT_NICE_WAITING
};

const MFSConfig mfscfg1 = {
  .flashp           = (BaseFlash *)&snor1.n25q.fls, /* Dirty trick, "fls" at same offset by design.*/
  .erased           = 0xFFFFFFFFU,
  .bank_size        = 4096U,
  .bank0_start      = 0U,
  .bank0_sectors    = 1U,
  .bank1_start      = 1U,
  .bank1_sectors    = 1U
};

/*
 * LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    palToggleLine(PORTAB_LINE_LED1);
    chThdSleepMilliseconds(500);
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

  /* Board-dependent GPIO setup code.*/
  portab_setup();

  /* Starting a serial port for test report output.*/
  sdStart(&PORTAB_SD1, NULL);

  /* Trying N25Q.*/
  n25qObjectInit(&snor1.n25q);
  if (xsnorStart(&snor1.n25q, &snorcfg_n25q) != FLASH_NO_ERROR) {

    /* Trying MX25.*/
    mx25ObjectInit(&snor1.mx25);
    if (xsnorStart(&snor1.mx25, &snorcfg_mx25) != FLASH_NO_ERROR) {
      chSysHalt("device not found");
    }
  }

#if 0
  /* Testing memory mapped mode.*/
  {
    uint8_t *addr;

    xsnorMemoryMap(&snor1, &addr);
    chThdSleepMilliseconds(50);
    xsnorMemoryUnmap(&snor1);
  }
#endif

  /* Creates the blinker thread.*/
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /* Normal main() thread activity, in this demo it does nothing.*/
  while (true) {
    if (palReadLine(PORTAB_LINE_BUTTON) == PORTAB_BUTTON_PRESSED) {
       test_execute((BaseSequentialStream *)&PORTAB_SD1, &mfs_test_suite);
    }
    chThdSleepMilliseconds(500);
  }
  return 0;
}
