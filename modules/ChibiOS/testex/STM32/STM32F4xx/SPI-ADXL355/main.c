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
#include "chprintf.h"
#include "adxl355.h"

/*===========================================================================*/
/* ADXL355 related.                                                          */
/*===========================================================================*/

/*
 * SPI TX and RX buffers.
 */
static uint8_t txbuf[32];
static uint8_t rxbuf[32];

/* ADXL355 Driver: This object represent an ADXL355 instance */
static ADXL355Driver ADXL355D1;

static int32_t accraw[ADXL355_ACC_NUMBER_OF_AXES];

static float acccooked[ADXL355_ACC_NUMBER_OF_AXES];

static char axisID[ADXL355_ACC_NUMBER_OF_AXES] = {'X', 'Y', 'Z'};
static uint32_t i;

static const SPIConfig spicfg = {
  .circular         = false,
  .slave            = false,
  .data_cb          = NULL,
  .error_cb         = NULL,
  .ssline           = LINE_ARD_D10,
  .cr1              = SPI_CR1_BR_1 | SPI_CR1_BR_0,
  .cr2              = 0U
};

static ADXL355Config adxl355cfg = {
  .spip             = &SPID1,
  .spicfg           = &spicfg,
  .accsensitivity   = NULL,
  .accbias          = NULL,
  .accfullscale     = ADXL355_ACC_FS_2G,
  .accodr           = ADXL355_ACC_ODR_125HZ,
#if ADXL355_USE_ADVANCED
  .acchighpass      = ADXL355_ACC_HP_LEV_3
#endif
};

/*===========================================================================*/
/* Generic code.                                                             */
/*===========================================================================*/

static BaseSequentialStream* chp = (BaseSequentialStream*)&SD2;

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

  sdStart(&SD2, NULL);

  palSetLineMode(LINE_ARD_D10, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(LINE_ARD_D11, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);
  palSetLineMode(LINE_ARD_D12, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);
  palSetLineMode(LINE_ARD_D13, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);

  /* ADXL355 Object Initialization.*/
  adxl355ObjectInit(&ADXL355D1, txbuf, rxbuf);

  /* Activates the ADXL355 driver.*/
  adxl355Start(&ADXL355D1, &adxl355cfg);

  /* Normal main() thread activity, printing MEMS data on the SDU1.*/
  while (true) {
    adxl355AccelerometerReadRaw(&ADXL355D1, accraw);
    chprintf(chp, "ADXL355 Accelerometer raw data...\r\n");
    for(i = 0; i < ADXL355_ACC_NUMBER_OF_AXES; i++) {
      chprintf(chp, "%c-axis: %d\r\n", axisID[i], accraw[i]);
    }

    adxl355AccelerometerReadCooked(&ADXL355D1, acccooked);
    chprintf(chp, "ADXL355 Accelerometer cooked data...\r\n");
    for(i = 0; i < ADXL355_ACC_NUMBER_OF_AXES; i++) {
      chprintf(chp, "%c-axis: %.3f\r\n", axisID[i], acccooked[i]);
    }
    chThdSleepMilliseconds(100);
  }
  adxl355Stop(&ADXL355D1);
}
