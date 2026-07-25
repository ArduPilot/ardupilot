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

#include "usbcfg.h"
#include "chprintf.h"
#include "lis3dsh.h"

/*===========================================================================*/
/* LIS3DSH related.                                                          */
/*===========================================================================*/

/* LIS3DSH Driver: This object represent an LIS3DSH instance */
static LIS3DSHDriver LIS3DSHD1;

static int32_t accraw[LIS3DSH_ACC_NUMBER_OF_AXES];

static float acccooked[LIS3DSH_ACC_NUMBER_OF_AXES];

static char axisID[LIS3DSH_ACC_NUMBER_OF_AXES] = {'X', 'Y', 'Z'};
static uint32_t i;

static const SPIConfig spicfg = {
  .circular         = false,
  .slave            = false,
  .data_cb          = NULL,
  .error_cb         = NULL,
  .ssport           = GPIOE,
  .sspad            = GPIOE_CS_SPI,
  .cr1              = SPI_CR1_BR_0 | SPI_CR1_CPOL | SPI_CR1_CPHA,
  .cr2              = 0U
};

static LIS3DSHConfig lis3dshcfg = {
  .spip             = &SPID1,
  .spicfg           = &spicfg,
  .accsensitivity   = NULL,
  .accbias          = NULL,
  .accfullscale     = LIS3DSH_ACC_FS_2G,
  .accodr           = LIS3DSH_ACC_ODR_100HZ,
#if LIS3DSH_USE_ADVANCED
  .accantialiasing  = LIS3DSH_ACC_BW_400HZ,
  .accbdu           = LIS3DSH_ACC_BDU_CONTINUOUS
#endif
};

/*===========================================================================*/
/* Generic code.                                                             */
/*===========================================================================*/

static BaseSequentialStream* chp = (BaseSequentialStream*)&SDU1;
/*
 * LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    systime_t time;

    time = serusbcfg.usbp->state == USB_ACTIVE ? 250 : 500;
    palToggleLine(LINE_LED6);
    chThdSleepMilliseconds(time);
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

  /* Initializes a serial-over-USB CDC driver.*/
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1500);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);

  /* Creates the blinker thread.*/
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 1, Thread1, NULL);

  /* LIS3DSH Object Initialization.*/
  lis3dshObjectInit(&LIS3DSHD1);

  /* Activates the LIS3DSH driver.*/
  lis3dshStart(&LIS3DSHD1, &lis3dshcfg);

  /* Normal main() thread activity, printing MEMS data on the SDU1.*/
  while (true) {
    lis3dshAccelerometerReadRaw(&LIS3DSHD1, accraw);
    chprintf(chp, "LIS3DSH Accelerometer raw data...\r\n");
    for(i = 0; i < LIS3DSH_ACC_NUMBER_OF_AXES; i++) {
      chprintf(chp, "%c-axis: %d\r\n", axisID[i], accraw[i]);
    }

    lis3dshAccelerometerReadCooked(&LIS3DSHD1, acccooked);
    chprintf(chp, "LIS3DSH Accelerometer cooked data...\r\n");
    for(i = 0; i < LIS3DSH_ACC_NUMBER_OF_AXES; i++) {
      chprintf(chp, "%c-axis: %.3f\r\n", axisID[i], acccooked[i]);
    }
    chThdSleepMilliseconds(100);
  }
  lis3dshStop(&LIS3DSHD1);
}
