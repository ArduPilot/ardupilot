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

#include "portab.h"

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)

static void cmd_date(BaseSequentialStream *chp, int argc, char *argv[]) {
  RTCDateTime timespec;

  (void)argv;

  if (argc > 0) {
    chprintf(chp, "Usage: date\r\n");
    return;
  }

  rtcGetTime(&RTCD1, &timespec);
  chprintf(chp, "%02d:%02d:%02d:%03d - %02d-%02d-%04d\r\n",
           timespec.millisecond / 3600000U,
           (timespec.millisecond % 3600000U) / 60000U,
           (timespec.millisecond % 60000U) / 1000U,
           timespec.millisecond % 1000U,
           timespec.month,
           timespec.day,
           timespec.year + 1980U);
}

#if RTC_HAS_STORAGE
static void cmd_storage(BaseSequentialStream *chp, int argc, char *argv[]) {
  size_t storage_size = psGetStorageSize(&RTCD1);
  ps_offset_t i;

  (void)argv;

  if (argc > 0) {
    chprintf(chp, "Usage: storage\r\n");
    return;
  }

  for (i = 0U; i < (ps_offset_t)storage_size; i++) {
    uint8_t val;
    psRead(&RTCD1, i, 1U, &val);
    chprintf(chp, "%02x ", val);
    if (((i + 1) & 15) == 0U) {
      chprintf(chp, "\r\n");
    }
  }
}
#endif

static const ShellCommand commands[] = {
  {"date", cmd_date},
#if RTC_HAS_STORAGE
  {"storage", cmd_storage},
#endif
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
  (BaseSequentialStream *)&PORTAB_SD1,
  commands
};

/*===========================================================================*/
/* Generic code.                                                             */
/*===========================================================================*/

static sysinterval_t interval = TIME_MS2I(500);

/*
 * LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    palToggleLine(PORTAB_LINE_LED1);
    chThdSleep(interval);
    palToggleLine(PORTAB_LINE_LED1);
    chThdSleep(interval);
  }
}

/*
 * RTC callback.
 */
static void alarmcb(RTCDriver *rtcp, rtcevent_t event) {

  (void)rtcp;

  if (event == RTC_EVENT_ALARM_A) {
    interval = TIME_MS2I(500);
  }
  else if (event == RTC_EVENT_ALARM_B) {
    interval = TIME_MS2I(50);
  }
}

/*
 * Application entry point.
 */
int main(void) {
  static const RTCAlarm alarm1 = {
    RTC_ALRM_MSK4  |    /* No month/week day match. */
    RTC_ALRM_MSK3  |    /* No hour match.           */
    RTC_ALRM_MSK2  |    /* No minutes match.        */
    RTC_ALRM_ST(0) |
    RTC_ALRM_SU(0)      /* Match minute start.      */
  };
  static const RTCAlarm alarm2 = {
    RTC_ALRM_MSK4  |    /* No month/week day match. */
    RTC_ALRM_MSK3  |    /* No hour match.           */
    RTC_ALRM_MSK2  |    /* No minutes match.        */
    RTC_ALRM_ST(3) |
    RTC_ALRM_SU(0)      /* Match minute half.       */
  };

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /* Board-dependent GPIO setup code.*/
  portab_setup();

  /* Starting a serial port for shell.*/
  sdStart(&PORTAB_SD1, NULL);

  /* Shell manager initialization.*/
  shellInit();

  /* Creates the blinker thread.*/
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  rtcSetAlarm(&RTCD1, 0, &alarm1);
  rtcSetAlarm(&RTCD1, 1, &alarm2);
  rtcSetCallback(&RTCD1, alarmcb);
#if RTC_HAS_STORAGE
  psWrite(&RTCD1, 0U, 12U, (const uint8_t *)"Hello World!");
#endif

  /* Normal main() thread activity, spawning shells.*/
  while (true) {
    thread_t *shelltp = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
                                            "shell", NORMALPRIO + 1,
                                            shellThread, (void *)&shell_cfg1);
    chThdWait(shelltp);               /* Waiting termination.             */
    chThdSleepMilliseconds(1000);
  }
}
