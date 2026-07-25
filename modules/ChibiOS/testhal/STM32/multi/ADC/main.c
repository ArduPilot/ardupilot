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
#include "ccportab.h"

#include "portab.h"

/*===========================================================================*/
/* ADC driver related.                                                       */
/*===========================================================================*/

#define ADC_GRP1_BUF_DEPTH      1
#define ADC_GRP2_BUF_DEPTH      64

/* Buffers are allocated with size and address aligned to the cache
   line size.*/
#if CACHE_LINE_SIZE > 0
CC_ALIGN_DATA(CACHE_LINE_SIZE)
#endif
adcsample_t samples1[CACHE_SIZE_ALIGN(adcsample_t, ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH)];

#if CACHE_LINE_SIZE > 0
CC_ALIGN_DATA(CACHE_LINE_SIZE)
#endif
adcsample_t samples2[CACHE_SIZE_ALIGN(adcsample_t, ADC_GRP2_NUM_CHANNELS * ADC_GRP2_BUF_DEPTH)];

/*
 * ADC streaming callback.
 */
size_t n= 0, nx = 0, ny = 0;
void adccallback(ADCDriver *adcp) {

  /* Updating counters.*/
  n++;
  if (adcIsBufferComplete(adcp)) {
    nx += 1;
  }
  else {
    ny += 1;
  }

  if ((n % 200) == 0U) {
#if defined(PORTAB_LINE_LED2)
    palToggleLine(PORTAB_LINE_LED2);
#endif
  }
}

/*
 * ADC errors callback, should never happen.
 */
void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {

  (void)adcp;
  (void)err;

  chSysHalt("it happened");
}

/*===========================================================================*/
/* Application code.                                                         */
/*===========================================================================*/

/*
 * This is a periodic thread that does absolutely nothing except flashing
 * a LED attached to TP1.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    palSetLine(PORTAB_LINE_LED1);
    chThdSleepMilliseconds(500);
    palClearLine(PORTAB_LINE_LED1);
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

  /* Board-dependent GPIO setup code.*/
  portab_setup();

  /*
   * Creates the example thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /*
   * Starting PORTAB_ADC1 driver and the temperature sensor.
   */
  adcStart(&PORTAB_ADC1, &portab_adccfg1);
  adcSTM32EnableVREF(&PORTAB_ADC1);
  adcSTM32EnableTS(&PORTAB_ADC1);

  /* Performing a one-shot conversion on two channels.*/
  adcConvert(&PORTAB_ADC1, &portab_adcgrpcfg1, samples1, ADC_GRP1_BUF_DEPTH);
  cacheBufferInvalidate(samples1, sizeof (samples1) / sizeof (adcsample_t));

  /*
   * Starting PORTAB_GPT1 driver, it is used for triggering the ADC.
   */
  gptStart(&PORTAB_GPT1, &portab_gptcfg1);

  /*
   * Starting an ADC continuous conversion triggered with a period of
   * 1/10000 second.
   */
  adcStartConversion(&PORTAB_ADC1, &portab_adcgrpcfg2,
                     samples2, ADC_GRP2_BUF_DEPTH);
  gptStartContinuous(&PORTAB_GPT1, 100U);

  /*
   * Normal main() thread activity, if the button is pressed then the
   * conversion is stopped.
   */
  while (true) {
    if (palReadLine(PORTAB_LINE_BUTTON) == PORTAB_BUTTON_PRESSED) {
      gptStopTimer(&PORTAB_GPT1);
      adcStopConversion(&PORTAB_ADC1);
    }
    chThdSleepMilliseconds(500);
  }
  return 0;
}
