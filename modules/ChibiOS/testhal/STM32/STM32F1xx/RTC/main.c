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

RTCDateTime timespec;
RTCAlarm alarmspec;

#define TEST_ALARM_WAKEUP     FALSE

#if TEST_ALARM_WAKEUP

/*
 * Running indicator thread.
 */
static THD_WORKING_AREA(blinkWA, 128);
static THD_FUNCTION(blink_thd, arg) {
  (void)arg;
  while (true) {
    chThdSleepMilliseconds(100);
    palTogglePad(GPIOC, GPIOC_LED);
  }
  return 0;
}

/*
 *
 */
int main(void) {

  uint32_t tv_sec;

  halInit();
  chSysInit();

  chThdCreateStatic(blinkWA, sizeof(blinkWA), NORMALPRIO, blink_thd, NULL);

  /* compile ability test */
  rtcGetTime(&RTCD1, &timespec);

  /* set alarm in near future */
  rtcSTM32GetSecMsec(&RTCD1, &tv_sec, NULL);
  alarmspec.tv_sec = tv_sec + 20;
  rtcSetAlarm(&RTCD1, 0, &alarmspec);

  while (true){
    chThdSleepSeconds(10);

    /* going to anabiosis*/
    chSysLock();
    PWR->CR |= PWR_CR_CWUF | PWR_CR_CSBF;
    PWR->CR |= PWR_CR_PDDS | PWR_CR_LPDS;
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    __WFI();
  }
  return 0;
}

#else /* TEST_ALARM_WAKEUP */

/*
 * Test alarm period.
 */
#define RTC_ALARMPERIOD   10

binary_semaphore_t alarm_sem;

/*
 * Alarm callback.
 */
static void my_cb(RTCDriver *rtcp, rtcevent_t event) {

  (void)rtcp;

  switch (event) {
  case RTC_EVENT_OVERFLOW:
    palTogglePad(GPIOC, GPIOC_LED);
    break;
  case RTC_EVENT_SECOND:
    //palTogglePad(GPIOC, GPIOC_LED);
    break;
  case RTC_EVENT_ALARM:
    palTogglePad(GPIOC, GPIOC_LED);
    osalSysLockFromISR();
    chBSemSignalI(&alarm_sem);
    osalSysUnlockFromISR();
    break;
  }
}

static time_measurement_t sett, gett;

int main(void) {

  msg_t status = MSG_TIMEOUT;
  uint32_t tv_sec;

  halInit();
  chSysInit();
  chBSemObjectInit(&alarm_sem, TRUE);
  chTMObjectInit(&sett);
  chTMObjectInit(&gett);

  /* compile ability test */
  chTMStartMeasurementX(&gett);
  rtcGetTime(&RTCD1, &timespec);
  chTMStopMeasurementX(&gett);

  rtcSTM32SetSec(&RTCD1, 1414845464);
  osalThreadSleepMilliseconds(10);
  rtcGetTime(&RTCD1, &timespec);
  timespec.month -= 1;

  chTMStartMeasurementX(&sett);
  rtcSetTime(&RTCD1, &timespec);
  chTMStopMeasurementX(&sett);
  osalThreadSleepMilliseconds(10);

  rtcGetTime(&RTCD1, &timespec);

  rtcSTM32GetSecMsec(&RTCD1, &tv_sec, NULL);
  alarmspec.tv_sec = tv_sec + RTC_ALARMPERIOD;
  rtcSetAlarm(&RTCD1, 0, &alarmspec);

  rtcSetCallback(&RTCD1, my_cb);

  while (true){

    /* Wait until alarm callback signaled semaphore.*/
    status = chBSemWaitTimeout(&alarm_sem, TIME_S2I(RTC_ALARMPERIOD + 5));

    if (status == MSG_TIMEOUT){
      osalSysHalt("time is out");
    }
    else{
      rtcSTM32GetSecMsec(&RTCD1, &tv_sec, NULL);
      alarmspec.tv_sec = tv_sec + RTC_ALARMPERIOD;
      rtcSetAlarm(&RTCD1, 0, &alarmspec);
    }
  }
  return 0;
}

#endif /* TEST_ALARM_WAKEUP */


