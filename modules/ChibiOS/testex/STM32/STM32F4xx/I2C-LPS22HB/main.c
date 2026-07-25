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
#include "lps22hb.h"

/*===========================================================================*/
/* LPS22HB related.                                                           */
/*===========================================================================*/

/* LPS22HB Driver: This object represent an LPS22HB instance */
static  LPS22HBDriver LPS22HBD1;

static int32_t baroraw;
static int32_t thermoraw;

static float barocooked;
static float thermocooked;

static const I2CConfig i2ccfg = {
  .op_mode          = OPMODE_I2C,
  .clock_speed      = 400000,
  .duty_cycle       = FAST_DUTY_CYCLE_2
};

static const LPS22HBConfig lps22hbcfg = {
  .i2cp               = &I2CD1,
  .i2ccfg             = &i2ccfg,
  .slaveaddress       = LPS22HB_SAD_VCC,
  .barosensitivity    = NULL,
  .barobias           = NULL,
  .thermosensitivity  = NULL,
  .thermobias         = NULL,
  .outputdatarate     = LPS22HB_ODR_10HZ,
#if LPS22HB_USE_ADVANCED
  .blockdataupdate    = LPS22HB_BDU_CONTINUOUS,
  .lowpass_filter     = LPS22HB_LP_ODR_9
#endif
};

/*===========================================================================*/
/* Generic code.                                                             */
/*===========================================================================*/

static BaseSequentialStream* chp = (BaseSequentialStream*)&SD2;
/*
 * LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    palToggleLine(LINE_LED_GREEN);
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

  /* Configuring I2C SCK and I2C SDA related GPIOs .*/
  palSetLineMode(LINE_ARD_D15, PAL_MODE_ALTERNATE(4) |
                 PAL_STM32_OSPEED_HIGHEST | PAL_STM32_OTYPE_OPENDRAIN);
  palSetLineMode(LINE_ARD_D14, PAL_MODE_ALTERNATE(4) |
                 PAL_STM32_OSPEED_HIGHEST | PAL_STM32_OTYPE_OPENDRAIN);

  /* Activates the serial driver 2 using the driver default configuration.*/
  sdStart(&SD2, NULL);

  /* Creates the blinker thread.*/
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /* LPS22HB Object Initialization.*/
  lps22hbObjectInit(&LPS22HBD1);

  /* Activates the LPS22HB driver.*/
  lps22hbStart(&LPS22HBD1, &lps22hbcfg);

  /* Normal main() thread activity, printing MEMS data on the SD2. */
  while (true) {
    lps22hbBarometerReadRaw(&LPS22HBD1, &baroraw);
    chprintf(chp, "LPS22HBD1 Barometer raw data...\r\n");
    chprintf(chp, "Pres: %d\r\n", baroraw);

    lps22hbThermometerReadRaw(&LPS22HBD1, &thermoraw);
    chprintf(chp, "LPS22HBD1 Thermometer raw data...\r\n");
    chprintf(chp, "Temp: %d\r\n", &thermoraw);

    lps22hbBarometerReadCooked(&LPS22HBD1, &barocooked);
    chprintf(chp, "LPS22HBD1 Barometer cooked data...\r\n");
    chprintf(chp, "Pres: %.2f\r\n", barocooked);

    lps22hbThermometerReadCooked(&LPS22HBD1, &thermocooked);
    chprintf(chp, "LPS22HBD1 Thermometer cooked data...\r\n");
    chprintf(chp, "Temp: %.2f\r\n", thermocooked);

    chThdSleepMilliseconds(100);
  }
  lps22hbStop(&LPS22HBD1);
}
