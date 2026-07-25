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

#include "portab.h"

#define TEST_CYCLES 1000U

static time_measurement_t tm1, tm2;
static thread_reference_t tr;

/*
 * Flyback thread.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;

  chRegSetThreadName("flyback");

  while (true) {
    /* Waiting for wakeup from PENDSV then stopping measurement.*/
    chSysLock();
    (void) chThdSuspendS(&tr);
    chTMStopMeasurementX(&tm2);
    chSysUnlock();
  }
}

/*
 * IRQ 0 handler.
 */
CH_IRQ_HANDLER(Vector40) {

  CH_IRQ_PROLOGUE();

  chTMChainMeasurementToX(&tm1, &tm2);

  chSysLockFromISR();
  chThdResumeI(&tr, MSG_OK);
  chSysUnlockFromISR();

  CH_IRQ_EPILOGUE();
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

  /* Starting a serial port for test report output.*/
  sdStart(&PORTAB_SD1, NULL);

  /* Starting the flyback thread.*/
  tr = NULL;
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 1U, Thread1, NULL);

  /* Initializing a TM objects for measurement of latency.*/
  chTMObjectInit(&tm1);
  chTMObjectInit(&tm2);

  /* Setting up an IRQ for the latency test. Highest available priority
     is used.*/
  nvicEnableVector(0, CORTEX_MAX_KERNEL_PRIORITY);

  /* Printing banner.*/
  chprintf((BaseSequentialStream *)&PORTAB_SD1, "\r\n*** ChibiOS/RT WKP-STORM benchmark and test\r\n***\r\n");
  chprintf((BaseSequentialStream *)&PORTAB_SD1, "*** Kernel:       %s\r\n", CH_KERNEL_VERSION);
  chprintf((BaseSequentialStream *)&PORTAB_SD1, "*** Compiled:     %s\r\n", __DATE__ " - " __TIME__);
#ifdef PORT_COMPILER_NAME
  chprintf((BaseSequentialStream *)&PORTAB_SD1, "*** Compiler:     %s\r\n", PORT_COMPILER_NAME);
#endif
  chprintf((BaseSequentialStream *)&PORTAB_SD1, "*** Architecture: %s\r\n", PORT_ARCHITECTURE_NAME);
#ifdef PORT_CORE_VARIANT_NAME
  chprintf((BaseSequentialStream *)&PORTAB_SD1, "*** Core Variant: %s\r\n", PORT_CORE_VARIANT_NAME);
#endif
  chprintf((BaseSequentialStream *)&PORTAB_SD1, "*** System Clock: %d\r\n", SystemCoreClock);
#ifdef PORT_INFO
  chprintf((BaseSequentialStream *)&PORTAB_SD1, "*** Port Info:    %s\r\n", PORT_INFO);
#endif
#ifdef PLATFORM_NAME
  chprintf((BaseSequentialStream *)&PORTAB_SD1, "*** Platform:     %s\r\n", PLATFORM_NAME);
#endif
#ifdef BOARD_NAME
  chprintf((BaseSequentialStream *)&PORTAB_SD1, "*** Test Board:   %s\r\n", BOARD_NAME);
#endif
  chprintf((BaseSequentialStream *)&PORTAB_SD1, "***\r\n\r\n");
  chThdSleepMilliseconds(500U);

  /* Test loop.*/
  for (unsigned i = 0U; i < TEST_CYCLES; i++) {
    /* Triggering IRQ zero, it will happen on the unlock.*/
    chSysLock();
    nvicSetPending(0);
    chTMStartMeasurementX(&tm1);
    chSysUnlock();
    chThdSleepMilliseconds(1);
  }

  /* Printing results.*/
  chprintf((BaseSequentialStream *)&PORTAB_SD1, "ISR activation time latency\r\n\r\n");
  chprintf((BaseSequentialStream *)&PORTAB_SD1, "Iterations:        %8u\r\n", tm1.n);
  chprintf((BaseSequentialStream *)&PORTAB_SD1, "Last measurement:  %8u cycles\r\n", tm1.last);
  chprintf((BaseSequentialStream *)&PORTAB_SD1, "Best measurement:  %8u cycles\r\n", tm1.best);
  chprintf((BaseSequentialStream *)&PORTAB_SD1, "Worst measurement: %8u cycles\r\n", tm1.worst);
  chprintf((BaseSequentialStream *)&PORTAB_SD1, "Cumulative time:   %8u cycles\r\n\r\n", (uint32_t)tm1.cumulative);
  chprintf((BaseSequentialStream *)&PORTAB_SD1, "Thread fly-back latency\r\n\r\n");
  chprintf((BaseSequentialStream *)&PORTAB_SD1, "Iterations:        %8u\r\n", tm2.n);
  chprintf((BaseSequentialStream *)&PORTAB_SD1, "Last measurement:  %8u cycles\r\n", tm2.last);
  chprintf((BaseSequentialStream *)&PORTAB_SD1, "Best measurement:  %8u cycles\r\n", tm2.best);
  chprintf((BaseSequentialStream *)&PORTAB_SD1, "Worst measurement: %8u cycles\r\n", tm2.worst);
  chprintf((BaseSequentialStream *)&PORTAB_SD1, "Cumulative time:   %8u cycles\r\n\r\n", (uint32_t)tm2.cumulative);

  /*
   * Normal main() thread activity, does nothing but sleep.
   */
  while (true) {
    chThdSleepMilliseconds(500);
  }

  return 0;
}
