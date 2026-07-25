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

/*
    Concepts and parts of this file have been contributed by Uladzimir Pylinsky
    aka barthess.
 */

/*
    This demo:
    1) turns LED on when you incline (~45 deg.) or shake the board;
    2) correctly handles absence of slave device on bus.
 */

#include <stdlib.h>
#include <math.h>

#include "ch.h"
#include "hal.h"

#include "lis3.h"
#include "fake.h"

/* measured acceleration components {x,y,z} */
static float acc[3];

/*
 * Accelerometer polling thread.
 */
static THD_WORKING_AREA(PollAccelThreadWA, 256);
static THD_FUNCTION(PollAccelThread, arg) {

  (void)arg;

  chRegSetThreadName("PollAccel");
  while (true) {
    osalThreadSleepMilliseconds(32);
    lis3GetAcc(acc);
  }
}

/*
 * Fake polling thread.
 */
static THD_WORKING_AREA(PollFakeThreadWA, 256);
static THD_FUNCTION(PollFakeThread, arg) {

  (void)arg;

  chRegSetThreadName("PollFake");
  while (true) {
    osalThreadSleepMilliseconds(16);
    request_fake();
  }
}

/*
 * I2C1 config.
 */
static const I2CConfig i2cfg1 = {
    OPMODE_I2C,
    400000,
    FAST_DUTY_CYCLE_2,
};

/*
 * Entry point, note, the main() function is already a thread in the system
 * on entry.
 */
int main(void) {

  halInit();
  chSysInit();

  i2cStart(&I2CD1, &i2cfg1);
  lis3Start();

  /* Create accelerometer thread. */
  chThdCreateStatic(PollAccelThreadWA,
          sizeof(PollAccelThreadWA),
          NORMALPRIO,
          PollAccelThread,
          NULL);

  /* Create not responding thread. */
  chThdCreateStatic(PollFakeThreadWA,
          sizeof(PollFakeThreadWA),
          NORMALPRIO,
          PollFakeThread,
          NULL);

  /* main loop handles LED */
  while (true) {
    if (sqrtf(acc[0]*acc[0] + acc[1]*acc[1]) > 0.5)
      palClearPad(IOPORT3, GPIOC_LED); /* on */
    else
      palSetPad(IOPORT3, GPIOC_LED); /* off */
    osalThreadSleepMilliseconds(20);
  }

  return 0;
}
