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
#include "lsm6dsl.h"

/*===========================================================================*/
/* LSM6DSL related.                                                          */
/*===========================================================================*/

/* LSM6DSL Driver: This object represent an LSM6DSL instance */
static  LSM6DSLDriver LSM6DSLD1;

static int32_t accraw[LSM6DSL_ACC_NUMBER_OF_AXES];
static int32_t gyroraw[LSM6DSL_GYRO_NUMBER_OF_AXES];

static float acccooked[LSM6DSL_ACC_NUMBER_OF_AXES];
static float gyrocooked[LSM6DSL_GYRO_NUMBER_OF_AXES];

static char axisID[LSM6DSL_ACC_NUMBER_OF_AXES] = {'X', 'Y', 'Z'};
static uint32_t i;

static const I2CConfig i2ccfg = {
  .op_mode          = OPMODE_I2C,
  .clock_speed      = 400000,
  .duty_cycle       = FAST_DUTY_CYCLE_2
};

static const LSM6DSLConfig lsm6dslcfg = {
  .i2cp             = &I2CD1,
  .i2ccfg           = &i2ccfg,
  .slaveaddress     = LSM6DSL_SAD_VCC,
  .accsensitivity   = NULL,
  .accbias          = NULL,
  .accfullscale     = LSM6DSL_ACC_FS_2G,
  .accodr           = LSM6DSL_ACC_ODR_52HZ,
#if LSM6DSL_USE_ADVANCED
  .acclpmode        = LSM6DSL_ACC_LP_ENABLED,
#endif
  .gyrosensitivity  = NULL,
  .gyrobias         = NULL,
  .gyrofullscale    = LSM6DSL_GYRO_FS_250DPS,
  .gyroodr          = LSM6DSL_GYRO_ODR_104HZ,
#if LSM6DSL_USE_ADVANCED
  .gyrolpmode       = LSM6DSL_GYRO_LP_ENABLED,
  .gyrolpfilter     = LSM6DSL_GYRO_LPF_FTYPE1,
  .bdu              = LSM6DSL_BDU_BLOCKED,
  .endianness       = LSM6DSL_END_LITTLE
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
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 1, Thread1, NULL);

  /* LSM6DSL Object Initialization.*/
  lsm6dslObjectInit(&LSM6DSLD1);

  /* Activates the LSM6DSL driver.*/
  lsm6dslStart(&LSM6DSLD1, &lsm6dslcfg);

  lsm6dslGyroscopeSampleBias(&LSM6DSLD1);

  /* Normal main() thread activity, printing MEMS data on the SDU1.*/
  while (true) {
    lsm6dslAccelerometerReadRaw(&LSM6DSLD1, accraw);
    chprintf(chp, "LSM6DSL Accelerometer raw data...\r\n");
    for(i = 0; i < LSM6DSL_ACC_NUMBER_OF_AXES; i++) {
      chprintf(chp, "%c-axis: %d\r\n", axisID[i], accraw[i]);
    }

    lsm6dslGyroscopeReadRaw(&LSM6DSLD1, gyroraw);
    chprintf(chp, "LSM6DSL Gyroscope raw data...\r\n");
    for(i = 0; i < LSM6DSL_GYRO_NUMBER_OF_AXES; i++) {
      chprintf(chp, "%c-axis: %d\r\n", axisID[i], gyroraw[i]);
    }

    lsm6dslAccelerometerReadCooked(&LSM6DSLD1, acccooked);
    chprintf(chp, "LSM6DSL Accelerometer cooked data...\r\n");
    for(i = 0; i < LSM6DSL_ACC_NUMBER_OF_AXES; i++) {
      chprintf(chp, "%c-axis: %.3f\r\n", axisID[i], acccooked[i]);
    }

    lsm6dslGyroscopeReadCooked(&LSM6DSLD1, gyrocooked);
    chprintf(chp, "LSM6DSL Gyroscope cooked data...\r\n");
    for(i = 0; i < LSM6DSL_GYRO_NUMBER_OF_AXES; i++) {
      chprintf(chp, "%c-axis: %.3f\r\n", axisID[i], gyrocooked[i]);
    }
    chThdSleepMilliseconds(100);
  }
  lsm6dslStop(&LSM6DSLD1);
}
