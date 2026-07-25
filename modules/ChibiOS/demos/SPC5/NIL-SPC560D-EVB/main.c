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
#include "nil_test_root.h"
#include "oslib_test_root.h"

/*
 * LEDs blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;

  /*
   * Activates the serial driver 1 using the driver default configuration.
   */
  sdStart(&SD1, NULL);

  while (true) {
    unsigned i;

    for (i = 0; i < 4; i++) {
      palClearPad(PORT_E, PE_LED1);
      chThdSleepMilliseconds(100);
      palClearPad(PORT_E, PE_LED2);
      chThdSleepMilliseconds(100);
      palClearPad(PORT_E, PE_LED3);
      chThdSleepMilliseconds(100);
      palClearPad(PORT_E, PE_LED4);
      chThdSleepMilliseconds(100);
      palSetPad(PORT_E, PE_LED1);
      chThdSleepMilliseconds(100);
      palSetPad(PORT_E, PE_LED2);
      chThdSleepMilliseconds(100);
      palSetPad(PORT_E, PE_LED3);
      chThdSleepMilliseconds(100);
      palSetPad(PORT_E, PE_LED4);
      chThdSleepMilliseconds(300);
    }

    for (i = 0; i < 4; i++) {
      palTogglePort(PORT_E, PAL_PORT_BIT(PE_LED1) | PAL_PORT_BIT(PE_LED2) |
                            PAL_PORT_BIT(PE_LED3) | PAL_PORT_BIT(PE_LED4));
      chThdSleepMilliseconds(500);
      palTogglePort(PORT_E, PAL_PORT_BIT(PE_LED1) | PAL_PORT_BIT(PE_LED2) |
                            PAL_PORT_BIT(PE_LED3) | PAL_PORT_BIT(PE_LED4));
      chThdSleepMilliseconds(500);
    }

    for (i = 0; i < 4; i++) {
      palTogglePad(PORT_E, PE_LED1);
      chThdSleepMilliseconds(250);
      palTogglePad(PORT_E, PE_LED1);
      palTogglePad(PORT_E, PE_LED2);
      chThdSleepMilliseconds(250);
      palTogglePad(PORT_E, PE_LED2);
      palTogglePad(PORT_E, PE_LED3);
      chThdSleepMilliseconds(250);
      palTogglePad(PORT_E, PE_LED3);
      palTogglePad(PORT_E, PE_LED4);
      chThdSleepMilliseconds(250);
      palTogglePad(PORT_E, PE_LED4);
    }

    for (i = 0; i < 4; i++) {
      palClearPort(PORT_E, PAL_PORT_BIT(PE_LED1) | PAL_PORT_BIT(PE_LED3));
      palSetPort(PORT_E, PAL_PORT_BIT(PE_LED2) | PAL_PORT_BIT(PE_LED4));
      chThdSleepMilliseconds(500);
      palClearPort(PORT_E, PAL_PORT_BIT(PE_LED2) | PAL_PORT_BIT(PE_LED4));
      palSetPort(PORT_E, PAL_PORT_BIT(PE_LED1) | PAL_PORT_BIT(PE_LED3));
      chThdSleepMilliseconds(500);
    }

    palSetPort(PORT_E, PAL_PORT_BIT(PE_LED1) | PAL_PORT_BIT(PE_LED2) |
                       PAL_PORT_BIT(PE_LED3) | PAL_PORT_BIT(PE_LED4));
  }
}

/*
 * Tester thread.
 */
THD_WORKING_AREA(waThread2, 128);
THD_FUNCTION(Thread2, arg) {

  (void)arg;

  /*
   * Activates the serial driver 1 using the driver default configuration.
   */
  sdStart(&SD1, NULL);

  /* Welcome message.*/
  chnWriteTimeout(&SD1, (uint8_t *)"Hello World!\r\n", 14, TIME_INFINITE);

  /* Waiting for button push and activation of the test suite.*/
  while (true) {
    if (palReadPad(PORT_E, PE_BUTTON1)) {
      test_execute((BaseSequentialStream *)&SD1, &nil_test_suite);
      test_execute((BaseSequentialStream *)&SD1, &oslib_test_suite);
    }
    chThdSleepMilliseconds(500);
  }
}

/*
 * Threads creation table, one entry per thread.
 */
THD_TABLE_BEGIN
  THD_TABLE_THREAD(0, "blinker",      waThread1,       Thread1,      NULL)
  THD_TABLE_THREAD(4, "tester",       waThread2,       Thread2,      NULL)
THD_TABLE_END

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

  /* This is now the idle thread loop, you may perform here a low priority
     task but you must never try to sleep or wait in this loop. Note that
     this tasks runs at the lowest priority level so any instruction added
     here will be executed after all other tasks have been started.*/
  while (true) {
  }
}
