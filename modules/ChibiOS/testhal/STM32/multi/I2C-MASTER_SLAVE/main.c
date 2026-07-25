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
#include <string.h>

#include "portab.h"

/*
 * I2C address
 */
#define I2C_ADDR              0x18

/*
 * BUFFER SIZE
 */
#define BUFFER_SIZE           32

/*
 * Master operations
 */
#define MASTER_TRAS_RECV      0x00
#define MASTER_TRAS           0x01
#define MASTER_RECV           0x02

/* Master buffer */
#if CACHE_LINE_SIZE > 0
CC_ALIGN_DATA(CACHE_LINE_SIZE)
#endif
uint8_t master_rx[CACHE_SIZE_ALIGN(uint8_t, BUFFER_SIZE)];

#if CACHE_LINE_SIZE > 0
CC_ALIGN_DATA(CACHE_LINE_SIZE)
#endif
uint8_t master_tx[CACHE_SIZE_ALIGN(uint8_t, BUFFER_SIZE)];

/* Slave buffer */
#if CACHE_LINE_SIZE > 0
CC_ALIGN_DATA(CACHE_LINE_SIZE)
#endif
PORTAB_SECTION
uint8_t slave_rx[CACHE_SIZE_ALIGN(uint8_t, BUFFER_SIZE)];

#if CACHE_LINE_SIZE > 0
CC_ALIGN_DATA(CACHE_LINE_SIZE)
#endif
PORTAB_SECTION
uint8_t slave_tx[CACHE_SIZE_ALIGN(uint8_t, BUFFER_SIZE)];

/*
 * LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 256);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    palToggleLine(PORTAB_LINE_LED);
    chThdSleepMilliseconds(500);
    palToggleLine(PORTAB_LINE_LED);
    chThdSleepMilliseconds(500);
  }
}

/*
 * This is a periodic thread waiting for master transmission.
 */
static THD_WORKING_AREA(waThread2, 512);
static THD_FUNCTION(Thread2, arg) {

  (void)arg;
  chRegSetThreadName("i2cslave");

  while (true) {
    /* Wait for a master transmission */
    i2cSlaveReceiveTimeout(&PORTAB_I2C_SLAVE, slave_rx, sizeof slave_rx, TIME_INFINITE);
    cacheBufferInvalidate(slave_rx, sizeof slave_rx);

    /* If the master needs a reply */
    if (i2cSlaveIsAnswerRequired(&PORTAB_I2C_SLAVE)) {
      cacheBufferFlush(slave_tx, sizeof slave_tx);
      i2cSlaveTransmitTimeout(&PORTAB_I2C_SLAVE, slave_tx, sizeof slave_tx, TIME_INFINITE);
    }
    memset(slave_rx, 0, sizeof slave_rx);
  }
}

/*
 * This is a periodic thread testing all I2C Master operation.
 */
static THD_WORKING_AREA(waThread3, 512);
static THD_FUNCTION(Thread3, arg) {

  (void)arg;
  chRegSetThreadName("i2cmaster");
  uint8_t op = MASTER_TRAS_RECV;

  while (true) {
    if (palReadLine(PORTAB_LINE_BUTTON)) {

      cacheBufferFlush(master_tx, sizeof master_tx);

      if (op == MASTER_TRAS_RECV) {
        i2cMasterTransmitTimeout(&PORTAB_I2C_MASTER, I2C_ADDR, master_tx, sizeof master_tx,
                                 master_rx, BUFFER_SIZE, TIME_INFINITE);
        cacheBufferInvalidate(master_rx, sizeof master_rx);
      }
      else if (op == MASTER_TRAS) {
        i2cMasterTransmitTimeout(&PORTAB_I2C_MASTER, I2C_ADDR, master_tx, sizeof master_tx,
                                 NULL, 0, TIME_INFINITE);
      }
      else {
        i2cMasterReceiveTimeout(&PORTAB_I2C_MASTER, I2C_ADDR, master_rx, BUFFER_SIZE, TIME_INFINITE);
        cacheBufferInvalidate(master_rx, sizeof master_rx);
      }



      /* Next operation */
      if (op == MASTER_RECV) {
        op = MASTER_TRAS_RECV;
      }
      else {
        op++;
      }

      /* Reset buffer */
      memset(master_rx, 0, sizeof master_rx);
    }
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

  /* Board-dependent setup code.*/
  portab_setup();

  /* Slave init */
  memset(slave_rx, 0, sizeof slave_rx);
  memcpy(slave_tx, "ChibiOS I2C Slave: reply phrase", sizeof slave_tx);

  /* Master init */
  memset(master_rx, 0, sizeof master_rx);
  memcpy(master_tx, "ChibiOS I2C Master: test phrase", sizeof master_tx);

  /* Start I2C Master */
  i2cStart(&PORTAB_I2C_MASTER, &i2c_cfg);

  /* Start I2C Slave */
  i2cStart(&PORTAB_I2C_SLAVE, &i2c_cfg);
  i2cSlaveMatchAddress(&PORTAB_I2C_SLAVE, I2C_ADDR);

  /* Creates the blinker thread.*/
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /* Creates the I2C slave thread. */
  chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, Thread2, NULL);

  /* Creates the I2C master thread. */
  chThdCreateStatic(waThread3, sizeof(waThread3), NORMALPRIO, Thread3, NULL);

  /* Normal main() thread activity, in this demo it does nothing.*/
  while (true) {
    chThdSleepMilliseconds(500);
  }
}
