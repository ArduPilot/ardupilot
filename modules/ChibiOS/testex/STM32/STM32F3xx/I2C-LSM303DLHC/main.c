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
#include "lsm303dlhc.h"

/*===========================================================================*/
/* LSM303DLHC related.                                                       */
/*===========================================================================*/

/* LSM303DLHC Driver: This object represent an LSM303DLHC instance */
static LSM303DLHCDriver LSM303DLHCD1;

static int32_t accraw[LSM303DLHC_ACC_NUMBER_OF_AXES];
static int32_t compraw[LSM303DLHC_COMP_NUMBER_OF_AXES];

static float acccooked[LSM303DLHC_ACC_NUMBER_OF_AXES];
static float compcooked[LSM303DLHC_COMP_NUMBER_OF_AXES];

static char axisID[LSM303DLHC_ACC_NUMBER_OF_AXES] = {'X', 'Y', 'Z'};
static uint32_t i;

static const I2CConfig i2ccfg = {
  .timingr          = STM32_TIMINGR_PRESC(15U) | STM32_TIMINGR_SCLDEL(4U) | 
                      STM32_TIMINGR_SDADEL(2U) | STM32_TIMINGR_SCLH(15U) | 
                      STM32_TIMINGR_SCLL(21U),
  .cr1              = 0,
  .cr2              = 0
};

static const LSM303DLHCConfig lsm303dlhccfg = {
  .i2cp             = &I2CD1,
  .i2ccfg           = &i2ccfg,
  .accsensitivity   = NULL,
  .accbias          = NULL,
  .accfullscale     = LSM303DLHC_ACC_FS_4G,
  .accodr           = LSM303DLHC_ACC_ODR_100Hz,
#if LSM303DLHC_USE_ADVANCED || defined(__DOXYGEN__)
  .acclowpower      = LSM303DLHC_ACC_LP_DISABLED,
  .acchighresmode   = LSM303DLHC_ACC_HR_DISABLED,
  .accbdu           = LSM303DLHC_ACC_BDU_BLOCK,
  .accendianess     = LSM303DLHC_ACC_END_LITTLE,
#endif
  .compsensitivity  = NULL,
  .compbias         = NULL,
  .compfullscale    = LSM303DLHC_COMP_FS_1P3GA,
  .compodr          = LSM303DLHC_COMP_ODR_30HZ,
#if LSM303DLHC_USE_ADVANCED || defined(__DOXYGEN__)
  .compmode         = LSM303DLHC_COMP_MD_BLOCK
#endif
};

/*===========================================================================*/
/* Generic code.                                                             */
/*===========================================================================*/

static BaseSequentialStream* chp = (BaseSequentialStream*)&SD1;
/*
 * Red LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    palToggleLine(LINE_LED3_RED);
    chThdSleepMilliseconds(250);
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

  /* Activates the serial driver 1 using the driver default configuration.*/
  sdStart(&SD1, NULL);

  /* Creates the blinker thread.*/
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /* LSM303DLHC Object Initialization.*/
  lsm303dlhcObjectInit(&LSM303DLHCD1);

  /* Activates the LSM303DLHC driver.*/
  lsm303dlhcStart(&LSM303DLHCD1, &lsm303dlhccfg);

  /* Normal main() thread activity, printing MEMS data on the SD1. */
  while (true) {
    lsm303dlhcAccelerometerReadRaw(&LSM303DLHCD1, accraw);
    chprintf(chp, "LSM303DLHC Accelerometer raw data...\r\n");
    for(i = 0; i < LSM303DLHC_ACC_NUMBER_OF_AXES; i++) {
      chprintf(chp, "%c-axis: %d\r\n", axisID[i], accraw[i]);
    }

    lsm303dlhcCompassReadRaw(&LSM303DLHCD1, compraw);
    chprintf(chp, "LSM303DLHC Compass raw data...\r\n");
    for(i = 0; i < LSM303DLHC_COMP_NUMBER_OF_AXES; i++) {
      chprintf(chp, "%c-axis: %d\r\n", axisID[i], compraw[i]);
    }

    lsm303dlhcAccelerometerReadCooked(&LSM303DLHCD1, acccooked);
    chprintf(chp, "LSM303DLHC Accelerometer cooked data...\r\n");
    for(i = 0; i < LSM303DLHC_ACC_NUMBER_OF_AXES; i++) {
      chprintf(chp, "%c-axis: %.3f\r\n", axisID[i], acccooked[i]);
    }

    lsm303dlhcCompassReadCooked(&LSM303DLHCD1, compcooked);
    chprintf(chp, "LSM303DLHC Compass cooked data...\r\n");
    for(i = 0; i < LSM303DLHC_COMP_NUMBER_OF_AXES; i++) {
      chprintf(chp, "%c-axis: %.3f\r\n", axisID[i], compcooked[i]);
    }
    chThdSleepMilliseconds(100);
  }
  lsm303dlhcStop(&LSM303DLHCD1);
}
