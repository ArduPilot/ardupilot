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

#include "shell.h"
#include "chprintf.h"

#define _offsetof(type, m) ((size_t)((char *)&((type)0)->m - (char *)0))

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)

static const ShellCommand commands[] = {
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
  (BaseSequentialStream *)&SIOD0,
  commands
};

/*
 * Green LED blinker thread, times are in milliseconds.
 */
static CH_SYS_CORE1_MEMORY THD_WORKING_AREA(waThreadTimer, 128);
static THD_FUNCTION(ThreadTimer, arg) {
  extern semaphore_t blinker_sem;

  (void)arg;
  chRegSetThreadName("timer");
  while (true) {
    chThdSleepMilliseconds(500);
    chSemSignal(&blinker_sem);
  }
}

/**
 * Core 1 entry point.
 */
void c1_main(void) {

  /*
   * Starting a new OS instance running on this core, we need to wait for
   * system initialization on the other side.
   */
  chSysWaitSystemState(ch_sys_running);
  chInstanceObjectInit(&ch1, &ch_core1_cfg);

  /* It is alive now.*/
  chSysUnlock();

  /*
   * Setting up GPIOs.
   */
  palSetLineMode(0U, PAL_MODE_ALTERNATE_UART);
  palSetLineMode(1U, PAL_MODE_ALTERNATE_UART);

  /*
   * Activates the UART0 SIO driver using the default configuration.
   */
  sioStart(&SIOD0, NULL);

  /*
   * Creates the timer thread.
   */
  chThdCreateStatic(waThreadTimer, sizeof(waThreadTimer),
                    NORMALPRIO + 10, ThreadTimer, NULL);

  /*
   * Shell manager initialization.
   */
  shellInit();

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop (re)spawning a shell.
   */
  while (true) {
    thread_t *shelltp = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
                                            "shell", NORMALPRIO + 1,
                                            shellThread, (void *)&shell_cfg1);
    chThdWait(shelltp);               /* Waiting termination.             */
    chThdSleepMilliseconds(500);
  }
}
