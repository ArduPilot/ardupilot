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

#define LED  7

static icucnt_t width = 0;
static icucnt_t period = 0;

static thread_t *thread1 = NULL;
static thread_t *thread2 = NULL;
static thread_t *thread_main = NULL;

static const uint32_t ICU_CLK = F_CPU / 1024;

void width_cb(ICUDriver *icup) {
  width = icup->width;
}

void period_cb(ICUDriver *icup) {
  period = icup->period;
  chSysLockFromISR();
  chEvtSignalI(thread1, (eventmask_t) 1);
  chSysUnlockFromISR();
}

void overflow_cb(ICUDriver *icup) {
  chSysLockFromISR();
  chEvtSignalI(thread2, (eventmask_t) 1);
  chSysUnlockFromISR();
}

void output_single_cycle(const uint16_t low, const uint16_t high) {
  palClearPad(IOPORT2, LED);
  palClearPad(IOPORT4, 4);
  chThdSleepMilliseconds(low);
  palSetPad(IOPORT2, LED);
  palSetPad(IOPORT4, 4);
  chThdSleepMilliseconds(high);
  palClearPad(IOPORT2, LED);
  palClearPad(IOPORT4, 4);
}

static THD_WORKING_AREA(waThread1, 64);
static THD_FUNCTION(Thread1, arg) {

  BaseSequentialStream *serp = (BaseSequentialStream *) &SD1;
  thread1 = chThdGetSelfX();
  while (TRUE) {
    chEvtWaitAny((eventmask_t) 1);
    chprintf(serp, "WIDTH[%lu ms, %u ticks] PERIOD[%lu ms, %u ticks]\r\n",
                   ((uint32_t) width * 1000) / ICU_CLK,
                   width,
                   ((uint32_t) period * 1000) / ICU_CLK,
                   period);
    chEvtSignal(thread_main, (eventmask_t) 1);
  }
}

static THD_WORKING_AREA(waThread2, 64);
static THD_FUNCTION(Thread2, arg) {

  BaseSequentialStream *serp = (BaseSequentialStream *) &SD1;
  thread2 = chThdGetSelfX();
  while (TRUE) {
    chEvtWaitAny((eventmask_t) 1);
    chprintf(serp, "OVERFLOW\r\n");
    chEvtSignal(thread_main, (eventmask_t) 2);
  }
}

int main(void) {

  halInit();

  static ICUConfig icu3cfg = {
    ICU_INPUT_ACTIVE_HIGH,
    0, /* Bogus frequency */
    width_cb,
    period_cb,
    overflow_cb,
  };

  palSetPadMode(IOPORT2, LED, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(IOPORT4, 4, PAL_MODE_OUTPUT_PUSHPULL);
  palClearPad(IOPORT2, LED);
  palClearPad(IOPORT4, 4);

  sdStart(&SD1, NULL);
  icuStart(&ICUD3, &icu3cfg);

  chSysInit();
  thread_main = chThdGetSelfX();

  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
  chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, Thread2, NULL);

  while (1) {
    BaseSequentialStream *serp = (BaseSequentialStream *) &SD1;

    chprintf(serp, "Testing 50 duty cycle\r\n");
    icuStartCapture(&ICUD3);
    output_single_cycle(500, 500);
    icuStopCapture(&ICUD3);
    chEvtWaitAny((eventmask_t) 3);

    chprintf(serp, "Testing 25 duty cycle\r\n");
    icuStartCapture(&ICUD3);
    output_single_cycle(250, 750);
    icuStopCapture(&ICUD3);
    chEvtWaitAny((eventmask_t) 3);

    chprintf(serp, "Testing 75 duty cycle\r\n");
    icuStartCapture(&ICUD3);
    output_single_cycle(750, 250);
    icuStopCapture(&ICUD3);
    chEvtWaitAny((eventmask_t) 3);

    chprintf(serp, "Testing overflow\r\n");
    icuStartCapture(&ICUD3);
    chEvtWaitAny((eventmask_t) 3);
    icuStopCapture(&ICUD3);
  }
}
