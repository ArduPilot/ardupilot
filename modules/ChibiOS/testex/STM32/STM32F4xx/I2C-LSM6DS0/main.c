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
#include "lsm6ds0.h"

/*===========================================================================*/
/* LSM6DS0 related.                                                          */
/*===========================================================================*/

/* LSM6DS0 Driver: This object represent an LSM6DS0 instance */
static  LSM6DS0Driver LSM6DS0D1;

static int32_t accraw[LSM6DS0_ACC_NUMBER_OF_AXES];
static int32_t gyroraw[LSM6DS0_GYRO_NUMBER_OF_AXES];

static float acccooked[LSM6DS0_ACC_NUMBER_OF_AXES];
static float gyrocooked[LSM6DS0_GYRO_NUMBER_OF_AXES];

static char axisID[LSM6DS0_ACC_NUMBER_OF_AXES] = {'X', 'Y', 'Z'};
static uint32_t i;

static const I2CConfig i2ccfg = {
  .op_mode          = OPMODE_I2C,
  .clock_speed      = 400000,
  .duty_cycle       = FAST_DUTY_CYCLE_2
};

static const LSM6DS0Config lsm6ds0cfg = {
  .i2cp             = &I2CD1,
  .i2ccfg           = &i2ccfg,
  .slaveaddress     = LSM6DS0_SAD_VCC,
  .accsensitivity   = NULL,
  .accbias          = NULL,
  .accfullscale     = LSM6DS0_ACC_FS_2G,
  .accodr           = LSM6DS0_ACC_ODR_50Hz,
#if LSM6DS0_USE_ADVANCED
  .accdecmode       = LSM6DS0_ACC_DEC_X4,
#endif
  .gyrosensitivity  = NULL,
  .gyrobias         = NULL,
  .gyrofullscale    = LSM6DS0_GYRO_FS_245DPS,
  .gyroodr          = LSM6DS0_GYRO_ODR_119HZ_FC_31,
#if LSM6DS0_USE_ADVANCED
  .gyrolowmodecfg   = LSM6DS0_GYRO_LP_DISABLED,
  .gyrooutsel       = LSM6DS0_GYRO_OUT_SEL_0,
  .gyrohpfenable    = LSM6DS0_GYRO_HP_DISABLED,
  .gyrohpcfg        = LSM6DS0_GYRO_HPCF_0,
  .bdu              = LSM6DS0_BDU_BLOCKED,
  .endianness       = LSM6DS0_END_LITTLE
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

  /* LSM6DS0 Object Initialization.*/
  lsm6ds0ObjectInit(&LSM6DS0D1);

  /* Activates the LSM6DS0 driver.*/
  lsm6ds0Start(&LSM6DS0D1, &lsm6ds0cfg);

  lsm6ds0GyroscopeSampleBias(&LSM6DS0D1);

  /* Normal main() thread activity, printing MEMS data on the SDU1.*/
  while (true) {
    lsm6ds0AccelerometerReadRaw(&LSM6DS0D1, accraw);
    chprintf(chp, "LSM6DS0 Accelerometer raw data...\r\n");
    for(i = 0; i < LSM6DS0_ACC_NUMBER_OF_AXES; i++) {
      chprintf(chp, "%c-axis: %d\r\n", axisID[i], accraw[i]);
    }

    lsm6ds0GyroscopeReadRaw(&LSM6DS0D1, gyroraw);
    chprintf(chp, "LSM6DS0 Gyroscope raw data...\r\n");
    for(i = 0; i < LSM6DS0_GYRO_NUMBER_OF_AXES; i++) {
      chprintf(chp, "%c-axis: %d\r\n", axisID[i], gyroraw[i]);
    }

    lsm6ds0AccelerometerReadCooked(&LSM6DS0D1, acccooked);
    chprintf(chp, "LSM6DS0 Accelerometer cooked data...\r\n");
    for(i = 0; i < LSM6DS0_ACC_NUMBER_OF_AXES; i++) {
      chprintf(chp, "%c-axis: %.3f\r\n", axisID[i], acccooked[i]);
    }

    lsm6ds0GyroscopeReadCooked(&LSM6DS0D1, gyrocooked);
    chprintf(chp, "LSM6DS0 Gyroscope cooked data...\r\n");
    for(i = 0; i < LSM6DS0_GYRO_NUMBER_OF_AXES; i++) {
      chprintf(chp, "%c-axis: %.3f\r\n", axisID[i], gyrocooked[i]);
    }
    chThdSleepMilliseconds(100);
  }
  lsm6ds0Stop(&LSM6DS0D1);
}
