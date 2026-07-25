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
#include "hts221.h"

/*===========================================================================*/
/* HTS221 related.                                                           */
/*===========================================================================*/

/* HTS221 Driver: This object represent an HTS221 instance */
static  HTS221Driver HTS221D1;

static int32_t hygroraw;
static int32_t thermoraw;

static float hygrocooked;
static float thermocooked;

static const I2CConfig i2ccfg = {
  .op_mode            = OPMODE_I2C,
  .clock_speed        = 400000,
  .duty_cycle         = FAST_DUTY_CYCLE_2
};

static const HTS221Config hts221cfg = {
  .i2cp               = &I2CD1,
  .i2ccfg             = &i2ccfg,
  .hygrosensitivity   = NULL,
  .hygrobias          = NULL,
  .thermosensitivity  = NULL,
  .thermobias         = NULL,
  .outputdatarate     = HTS221_ODR_7HZ
#if HTS221_USE_ADVANCED || defined(__DOXYGEN__)
  .blockdataupdate    = HTS221_BDU_CONTINUOUS,
  .hygroresolution    = HTS221_AVGH_256,
  .thermoresolution   = HTS221_AVGT_256
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

  /* Activates the serial driver 1 using the driver default configuration.*/
  sdStart(&SD2, NULL);

  /* Creates the blinker thread.*/
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /* HTS221 Object Initialization.*/
  hts221ObjectInit(&HTS221D1);

  /* Activates the HTS221 driver.*/
  hts221Start(&HTS221D1, &hts221cfg);

  /* Normal main() thread activity, printing MEMS data on the SD2. */
  while (true) {
    hts221HygrometerReadRaw(&HTS221D1, &hygroraw);
    chprintf(chp, "HTS221D1 Hygrometer raw data...\r\n");
    chprintf(chp, "Hum: %d\r\n", hygroraw);

    hts221ThermometerReadRaw(&HTS221D1, &thermoraw);
    chprintf(chp, "HTS221D1 Thermometer raw data...\r\n");
    chprintf(chp, "Temp: %d\r\n", &thermoraw);

    hts221HygrometerReadCooked(&HTS221D1, &hygrocooked);
    chprintf(chp, "HTS221D1 Hygrometer cooked data...\r\n");
    chprintf(chp, "Hum: %.2f\r\n", hygrocooked);

    hts221ThermometerReadCooked(&HTS221D1, &thermocooked);
    chprintf(chp, "HTS221D1 Thermometer cooked data...\r\n");
    chprintf(chp, "Temp: %.2f\r\n", thermocooked);

    chThdSleepMilliseconds(100);
  }
  hts221Stop(&HTS221D1);
}
