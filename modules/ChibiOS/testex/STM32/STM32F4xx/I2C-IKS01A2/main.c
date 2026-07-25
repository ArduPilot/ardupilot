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
#include "lps22hb.h"
#include "lsm303agr.h"
#include "lsm6dsl.h"

#define cls(chp)                            chprintf(chp, "\033[2J\033[1;1H")
#define MAX_AXIS_NUMBER                     3U

/* Array for data storage. */
static float cooked[MAX_AXIS_NUMBER];
/* Axis identifiers. */
static char axis_id[MAX_AXIS_NUMBER] = {'X', 'Y', 'Z'};

/* Generic I2C configuration for every MEMS. */
static const I2CConfig i2ccfg = {
  .op_mode            = OPMODE_I2C,
  .clock_speed        = 400000,
  .duty_cycle         = FAST_DUTY_CYCLE_2
};

static uint32_t i;
static BaseSequentialStream* chp = (BaseSequentialStream*)&SD2;

/*===========================================================================*/
/* HTS221 related.                                                           */
/*===========================================================================*/

/* HTS221 Driver: This object represent an HTS221 instance */
static  HTS221Driver HTS221D1;

static const HTS221Config hts221cfg = {
  .i2cp               = &I2CD1,
  .i2ccfg             = &i2ccfg,
  .hygrosensitivity   = NULL,
  .hygrobias          = NULL,
  .thermosensitivity  = NULL,
  .thermobias         = NULL,
  .outputdatarate     = HTS221_ODR_7HZ
};

/*===========================================================================*/
/* LPS22HB related.                                                           */
/*===========================================================================*/

/* LPS22HB Driver: This object represent an LPS22HB instance */
static  LPS22HBDriver LPS22HBD1;

static const LPS22HBConfig lps22hbcfg = {
  .i2cp               = &I2CD1,
  .i2ccfg             = &i2ccfg,
  .slaveaddress       = LPS22HB_SAD_VCC,
  .barosensitivity    = NULL,
  .barobias           = NULL,
  .thermosensitivity  = NULL,
  .thermobias         = NULL,
  .outputdatarate     = LPS22HB_ODR_10HZ
};
/*===========================================================================*/
/* LSM303AGR related.                                                        */
/*===========================================================================*/

/* LSM303AGR Driver: This object represent an LSM303AGR instance */
static LSM303AGRDriver LSM303AGRD1;

static const LSM303AGRConfig lsm303agrcfg = {
  .i2cp               = &I2CD1,
  .i2ccfg             = &i2ccfg,
  .accsensitivity     = NULL,
  .accbias            = NULL,
  .accfullscale       = LSM303AGR_ACC_FS_4G,
  .accodr             = LSM303AGR_ACC_ODR_100Hz,
  .compsensitivity    = NULL,
  .compbias           = NULL,
  .compodr            = LSM303AGR_COMP_ODR_50HZ,
};

/*===========================================================================*/
/* LSM6DSL related.                                                          */
/*===========================================================================*/

/* LSM6DSL Driver: This object represent an LSM6DSL instance */
static  LSM6DSLDriver LSM6DSLD1;

static const LSM6DSLConfig lsm6dslcfg = {
  .i2cp               = &I2CD1,
  .i2ccfg             = &i2ccfg,
  .slaveaddress       = LSM6DSL_SAD_VCC,
  .accsensitivity     = NULL,
  .accbias            = NULL,
  .accfullscale       = LSM6DSL_ACC_FS_2G,
  .accodr             = LSM6DSL_ACC_ODR_52HZ,
  .gyrosensitivity    = NULL,
  .gyrobias           = NULL,
  .gyrofullscale      = LSM6DSL_GYRO_FS_250DPS,
  .gyroodr            = LSM6DSL_GYRO_ODR_104HZ,
};

/*===========================================================================*/
/* Generic code.                                                             */
/*===========================================================================*/

/*
 * Green LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    palClearPad(GPIOA, GPIOA_LED_GREEN);
    chThdSleepMilliseconds(500);
    palSetPad(GPIOA, GPIOA_LED_GREEN);
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

  /*
   * Activates the serial driver 2 using the driver default configuration.
   */
  sdStart(&SD2, NULL);

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 1, Thread1, NULL);

  /* MEMS Driver Objects Initialization.*/
  hts221ObjectInit(&HTS221D1);
  lps22hbObjectInit(&LPS22HBD1);
  lsm303agrObjectInit(&LSM303AGRD1);
  lsm6dslObjectInit(&LSM6DSLD1);

  /* Activates all the MEMS related drivers.*/
  hts221Start(&HTS221D1, &hts221cfg);
  lps22hbStart(&LPS22HBD1, &lps22hbcfg);
  lsm303agrStart(&LSM303AGRD1, &lsm303agrcfg);
  lsm6dslStart(&LSM6DSLD1, &lsm6dslcfg);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
  while (true) {
    hts221HygrometerReadCooked(&HTS221D1, cooked);
    chprintf(chp, "HTS221D1 Hygrometer cooked data...\r\n");
    chprintf(chp, "Hum: %.2f\r\n", *cooked);

    hts221ThermometerReadCooked(&HTS221D1, cooked);
    chprintf(chp, "HTS221D1 Thermometer cooked data...\r\n");
    chprintf(chp, "Temp: %.2f\r\n", *cooked);

    lps22hbBarometerReadCooked(&LPS22HBD1, cooked);
    chprintf(chp, "LPS22HBD1 Barometer cooked data...\r\n");
    chprintf(chp, "Pres: %.2f\r\n", *cooked);

    lps22hbThermometerReadCooked(&LPS22HBD1, cooked);
    chprintf(chp, "LPS22HBD1 Thermometer cooked data...\r\n");
    chprintf(chp, "Temp: %.2f\r\n", *cooked);

    lsm303agrAccelerometerReadCooked(&LSM303AGRD1, cooked);
    chprintf(chp, "LSM303AGR Accelerometer cooked data...\r\n");
    for(i = 0; i < LSM303AGR_ACC_NUMBER_OF_AXES; i++) {
      chprintf(chp, "%c-axis: %.3f\r\n", axis_id[i], cooked[i]);
    }

    lsm303agrCompassReadCooked(&LSM303AGRD1, cooked);
    chprintf(chp, "LSM303AGR Compass cooked data...\r\n");
    for(i = 0; i < LSM303AGR_COMP_NUMBER_OF_AXES; i++) {
      chprintf(chp, "%c-axis: %.3f\r\n", axis_id[i], cooked[i]);
    }

    lsm6dslAccelerometerReadCooked(&LSM6DSLD1, cooked);
    chprintf(chp, "LSM6DSL Accelerometer cooked data...\r\n");
    for(i = 0; i < LSM6DSL_ACC_NUMBER_OF_AXES; i++) {
      chprintf(chp, "%c-axis: %.3f\r\n", axis_id[i], cooked[i]);
    }

    lsm6dslGyroscopeReadCooked(&LSM6DSLD1, cooked);
    chprintf(chp, "LSM6DSL Gyroscope cooked data...\r\n");
    for(i = 0; i < LSM6DSL_GYRO_NUMBER_OF_AXES; i++) {
      chprintf(chp, "%c-axis: %.3f\r\n", axis_id[i], cooked[i]);
    }

    chThdSleepMilliseconds(500);
  }
}
