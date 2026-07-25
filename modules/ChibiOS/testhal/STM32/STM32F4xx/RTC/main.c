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
This structure is used to hold the values representing a calendar time.
It contains the following members, with the meanings as shown.

int tm_sec       seconds after minute [0-61] (61 allows for 2 leap-seconds)
int tm_min       minutes after hour [0-59]
int tm_hour      hours after midnight [0-23]
int tm_mday      day of the month [1-31]
int tm_mon       month of year [0-11]
int tm_year      current year-1900
int tm_wday      days since Sunday [0-6]
int tm_yday      days since January 1st [0-365]
int tm_isdst     daylight savings indicator (1 = yes, 0 = no, -1 = unknown)
*/
#define WAKEUP_TEST       FALSE

#include <string.h>
#include <stdlib.h>

#include "ch.h"
#include "hal.h"

#include "shell.h"
#include "chprintf.h"

#if WAKEUP_TEST
static RTCWakeup wakeupspec;
#endif
static RTCAlarm alarmspec;
static RTCDateTime timespec;
static time_t unix_time;

/*
 * Awake state indicator thread
 */
static THD_WORKING_AREA(blinkWA, 128);
static THD_FUNCTION(blink_thd, arg){
  (void)arg;
  while (true) {
    chThdSleepMilliseconds(100);
    palTogglePad(GPIOC, GPIOC_LED);
  }
}

/*
 * Helper functions putting MCU in sleep state
 */
static void anabiosis(void) {
  chSysLock();
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  PWR->CR  |= (PWR_CR_PDDS | PWR_CR_LPDS | PWR_CR_CSBF | PWR_CR_CWUF);
  RTC->ISR &= ~(RTC_ISR_ALRBF | RTC_ISR_ALRAF | RTC_ISR_WUTF | RTC_ISR_TAMP1F |
                RTC_ISR_TSOVF | RTC_ISR_TSF);
  __WFI();
}

/*
 * Console applet for sleep testing
 */
static void cmd_sleep(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: sleep\r\n");
    return;
  }
  chprintf(chp, "Going sleep...\r\n");

  chThdSleepMilliseconds(200);

  anabiosis();
}

/*
 * Console applet for periodic alaram testing
 */
static void cmd_alarm(BaseSequentialStream *chp, int argc, char *argv[]) {
  int i = 0;

  (void)argv;
  if (argc < 1) {
    goto ERROR;
  }

  if ((argc == 1) && (strcmp(argv[0], "get") == 0)) {
    rtcGetAlarm(&RTCD1, 0, &alarmspec);
    i = (alarmspec.alrmr & 0b1111) + ((alarmspec.alrmr >> 4) & 0b111) * 10;
    chprintf(chp, "%U%s", i," - alarm in seconds\r\n");
    return;
  }

  if ((argc == 2) && (strcmp(argv[0], "set") == 0)) {
    i = atol(argv[1]);
    if (i > 59)
      goto ERROR;

    /* first disable all alrams if any */
    rtcSetAlarm(&RTCD1, 0, NULL);
    rtcSetAlarm(&RTCD1, 1, NULL);

    /* now set alarm only A */
    alarmspec.alrmr = ((i / 10) << 4) | (i % 10) |
                        RTC_ALRMAR_MSK4 | RTC_ALRMAR_MSK3 | RTC_ALRMAR_MSK2;
    rtcSetAlarm(&RTCD1, 0, &alarmspec);
    return;
  }
  else {
    goto ERROR;
  }

ERROR:
  chprintf(chp, "Usage: alarm get\r\n");
  chprintf(chp, "       alarm set N\r\n");
  chprintf(chp, "where N is alarm second on every minute\r\n");
  chprintf(chp, "To test alarm functionality perform following steps:\r\n");
  chprintf(chp, "1) set alarm second using this command\r\n");
  chprintf(chp, "2) execute 'sleep' command\r\n");
  chprintf(chp, "3) wait until the red led starts blinking\r\n");
  chprintf(chp, "4) immediately execute 'date get' command\r\n");
  chprintf(chp, "5) check seconds's field in returned date.\r\n");
  chprintf(chp, "   It must be close to programmed alarm second\r\n");
}

/*
 * helper function
 */
static time_t GetTimeUnixSec(void) {
  struct tm tim;

  rtcGetTime(&RTCD1, &timespec);
  rtcConvertDateTimeToStructTm(&timespec, &tim, NULL);
  return mktime(&tim);
}

/*
 * helper function
 */
static void GetTimeTm(struct tm *timp) {
  rtcGetTime(&RTCD1, &timespec);
  rtcConvertDateTimeToStructTm(&timespec, timp, NULL);
}

/*
 * helper function
 */
static void SetTimeUnixSec(time_t unix_time) {
  struct tm tim;
  struct tm *canary;

  /* If the conversion is successful the function returns a pointer
     to the object the result was written into.*/
  canary = localtime_r(&unix_time, &tim);
  osalDbgCheck(&tim == canary);

  rtcConvertStructTmToDateTime(&tim, 0, &timespec);
  rtcSetTime(&RTCD1, &timespec);
}

/*
 * Console applet for date set and get
 */
static void cmd_date(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argv;
  struct tm timp = {0};

  if (argc == 0) {
    goto ERROR;
  }

  if ((argc == 1) && (strcmp(argv[0], "get") == 0)){
    unix_time = GetTimeUnixSec();

    if (unix_time == -1){
      chprintf(chp, "incorrect time in RTC cell\r\n");
    }
    else{
      chprintf(chp, "%D%s", unix_time, "\r\n");
      GetTimeTm(&timp);
      chprintf(chp, "%s", asctime(&timp));
    }
    return;
  }

  if ((argc == 2) && (strcmp(argv[0], "set") == 0)){
    unix_time = atol(argv[1]);
    if (unix_time > 0){
      SetTimeUnixSec(unix_time);
      return;
    }
    else{
      goto ERROR;
    }
  }
  else{
    goto ERROR;
  }

ERROR:
  chprintf(chp, "Usage: date get\r\n");
  chprintf(chp, "       date set N\r\n");
  chprintf(chp, "where N is time in seconds sins Unix epoch\r\n");
  chprintf(chp, "you can get current N value from unix console by the command\r\n");
  chprintf(chp, "%s", "date +\%s\r\n");
  return;
}

/*
 *
 */
static SerialConfig ser_cfg = {
    115200,
    0,
    0,
    0,
};

/*
 *
 */
static const ShellCommand commands[] = {
  {"alarm", cmd_alarm},
  {"date",  cmd_date},
  {"sleep", cmd_sleep},
  {NULL, NULL}
};

/*
 *
 */
static const ShellConfig shell_cfg1 = {
  (BaseSequentialStream  *)&SD6,
  commands
};

/*
 * working area for shell thread
 */
static THD_WORKING_AREA(waShell, 1024);

/**
 * Main function.
 */
int main(void){

  halInit();
  chSysInit();
  chThdCreateStatic(blinkWA, sizeof(blinkWA), NORMALPRIO, blink_thd, NULL);

#if WAKEUP_TEST
  /* set wakeup */
  wakeupspec.wutr = ((uint32_t)4) << 16; /* select 1 Hz clock source */
  wakeupspec.wutr |= 9; /* set counter value to 9. Period will be 9+1 seconds. */
  rtcSTM32SetPeriodicWakeup(&RTCD1, &wakeupspec);

  osalThreadSleepSeconds(3);
  anabiosis();

#else

  /* switch off wakeup */
  rtcSTM32SetPeriodicWakeup(&RTCD1, NULL);

  /* Shell initialization.*/
  sdStart(&SD6, &ser_cfg);
  shellInit();
  chThdCreateStatic(waShell, sizeof(waShell), NORMALPRIO,
                    shellThread, (void *)&shell_cfg1);

  /* wait until user do not want to test wakeup */
  while (true){
    osalThreadSleepMilliseconds(200);
  }
#endif /* WAKEUP_TEST */

  return 0;
}


