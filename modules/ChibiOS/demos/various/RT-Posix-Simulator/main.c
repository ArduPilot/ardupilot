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

#define SHELL_WA_SIZE       THD_WORKING_AREA_SIZE(4096)
#define CONSOLE_WA_SIZE     THD_WORKING_AREA_SIZE(4096)
#define TEST_WA_SIZE        THD_WORKING_AREA_SIZE(4096)

#define cputs(msg) chMsgSend(cdtp, (msg_t)msg)

static thread_t *cdtp;
static thread_t *shelltp1;
static thread_t *shelltp2;

static const ShellCommand commands[] = {
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
  (BaseSequentialStream *)&SD1,
  commands
};

static const ShellConfig shell_cfg2 = {
  (BaseSequentialStream *)&SD2,
  commands
};

/*
 * Console print server done using synchronous messages. This makes the access
 * to the C printf() thread safe and the print operation atomic among threads.
 * In this example the message is the zero terminated string itself.
 */
static THD_FUNCTION(console_thread, arg) {

  (void)arg;
  while (!chThdShouldTerminateX()) {
    thread_t *tp = chMsgWait();
    puts((char *)chMsgGet(tp));
    fflush(stdout);
    chMsgRelease(tp, MSG_OK);
  }
}

/**
 * @brief Shell termination handler.
 *
 * @param[in] id event id.
 */
static void termination_handler(eventid_t id) {

  (void)id;
  if (shelltp1 && chThdTerminatedX(shelltp1)) {
    chThdWait(shelltp1);
    shelltp1 = NULL;
    chThdSleepMilliseconds(10);
    cputs("Init: shell on SD1 terminated");
    chSysLock();
    oqResetI(&SD1.oqueue);
    chSchRescheduleS();
    chSysUnlock();
  }
  if (shelltp2 && chThdTerminatedX(shelltp2)) {
    chThdWait(shelltp2);
    shelltp2 = NULL;
    chThdSleepMilliseconds(10);
    cputs("Init: shell on SD2 terminated");
    chSysLock();
    oqResetI(&SD2.oqueue);
    chSchRescheduleS();
    chSysUnlock();
  }
}

static event_listener_t sd1fel, sd2fel;

/**
 * @brief SD1 status change handler.
 *
 * @param[in] id event id.
 */
static void sd1_handler(eventid_t id) {
  eventflags_t flags;

  (void)id;
  flags = chEvtGetAndClearFlags(&sd1fel);
  if ((flags & CHN_CONNECTED) && (shelltp1 == NULL)) {
    cputs("Init: connection on SD1");
    shelltp1 = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
                                   "shell1", NORMALPRIO + 10,
                                   shellThread, (void *)&shell_cfg1);
  }
  if (flags & CHN_DISCONNECTED) {
    cputs("Init: disconnection on SD1");
    chSysLock();
    iqResetI(&SD1.iqueue);
    chSchRescheduleS();
    chSysUnlock();
  }
}

/**
 * @brief SD2 status change handler.
 *
 * @param[in] id event id.
 */
static void sd2_handler(eventid_t id) {
  eventflags_t flags;

  (void)id;
  flags = chEvtGetAndClearFlags(&sd2fel);
  if ((flags & CHN_CONNECTED) && (shelltp2 == NULL)) {
    cputs("Init: connection on SD2");
    shelltp2 = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
                                   "shell2", NORMALPRIO + 10,
                                   shellThread, (void *)&shell_cfg2);
  }
  if (flags & CHN_DISCONNECTED) {
    cputs("Init: disconnection on SD2");
    chSysLock();
    iqResetI(&SD2.iqueue);
    chSchRescheduleS();
    chSysUnlock();
  }
}

static evhandler_t fhandlers[] = {
  termination_handler,
  sd1_handler,
  sd2_handler
};

/*------------------------------------------------------------------------*
 * Simulator main.                                                        *
 *------------------------------------------------------------------------*/
int main(void) {
  event_listener_t tel;

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * Serial ports (simulated) initialization.
   */
  sdStart(&SD1, NULL);
  sdStart(&SD2, NULL);

  /*
   * Shell manager initialization.
   */
  shellInit();
  chEvtRegister(&shell_terminated, &tel, 0);

  /*
   * Console thread started.
   */
  cdtp = chThdCreateFromHeap(NULL, CONSOLE_WA_SIZE, "console",
                             NORMALPRIO + 1, console_thread, NULL);

  /*
   * Initializing connection/disconnection events.
   */
  cputs("Shell service started on SD1, SD2");
  cputs("  - Listening for connections on SD1");
  chEvtRegister(chnGetEventSource(&SD1), &sd1fel, 1);
  cputs("  - Listening for connections on SD2");
  chEvtRegister(chnGetEventSource(&SD2), &sd2fel, 2);

  /*
   * Events servicing loop.
   */
  while (!chThdShouldTerminateX())
    chEvtDispatch(fhandlers, chEvtWaitOne(ALL_EVENTS));

  /*
   * Clean simulator exit.
   */
  chEvtUnregister(chnGetEventSource(&SD1), &sd1fel);
  chEvtUnregister(chnGetEventSource(&SD2), &sd2fel);
  return 0;
}
