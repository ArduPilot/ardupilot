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

#define ADC_GRP1_NUM_CHANNELS   1
#define ADC_GRP1_BUF_DEPTH      8

#define ADC_GRP2_NUM_CHANNELS   1
#define ADC_GRP2_BUF_DEPTH      32

static adcsample_t samples1[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];
static adcsample_t samples2[ADC_GRP2_NUM_CHANNELS * ADC_GRP2_BUF_DEPTH];

/*
 * ADC streaming callback.
 */
size_t nx = 0, ny = 0;
static void adccallback(ADCDriver *adcp) {

  if (adcIsBufferComplete(adcp)) {
    nx += 1;
  }
  else {
    ny += 1;
  }
}

static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {

  (void)adcp;
  (void)err;
}

/*
 * SDADC configuration.
 */
static const ADCConfig sdadc_config = {
  0,
  {
     SDADC_CONFR_GAIN_1X | SDADC_CONFR_SE_ZERO_VOLT | SDADC_CONFR_COMMON_VSSSD,
     0,
     0
  }
};

/*
 * ADC conversion group.
 * Mode:        Linear buffer, 8 samples of 1 channel, SW triggered.
 * Channels:    ADC_IN5P.
 */
static const ADCConversionGroup adcgrpcfg1 = {
  FALSE,
  ADC_GRP1_NUM_CHANNELS,
  NULL,
  adcerrorcallback,
  .u.sdadc = {
    SDADC_CR2_JSWSTART,     /* CR2      */
    SDADC_JCHGR_CH(5),      /* JCHGR    */
    {                       /* CONFCHR[2]*/
      SDADC_CONFCHR1_CH5(0),
      0
    }
  }
};

/*
 * ADC conversion group.
 * Mode:        Continuous, 32 samples of 1 channel, SW triggered.
 * Channels:    ADC_IN5P.
 */
static const ADCConversionGroup adcgrpcfg2 = {
  TRUE,
  ADC_GRP2_NUM_CHANNELS,
  adccallback,
  adcerrorcallback,
  .u.sdadc = {
    SDADC_CR2_JSWSTART,     /* CR2      */
    SDADC_JCHGR_CH(5),      /* JCHGR    */
    {                       /* CONFCHR[2]*/
      SDADC_CONFCHR1_CH5(0),
      0
    }
  }
};

/*
 * Red LEDs blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;

  chRegSetThreadName("blinker");
  while (true) {
    palClearPad(GPIOC, GPIOC_LED1);
    chThdSleepMilliseconds(500);
    palSetPad(GPIOC, GPIOC_LED1);
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

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /*
   * Activates the SDADC1 driver.
   */
  adcStart(&SDADCD1, &sdadc_config);
  adcSTM32Calibrate(&SDADCD1);

  /*
   * Linear conversion.
   */
  adcConvert(&SDADCD1, &adcgrpcfg1, samples1, ADC_GRP1_BUF_DEPTH);

  /*
   * Starts an ADC continuous conversion.
   */
  adcStartConversion(&SDADCD1, &adcgrpcfg2, samples2, ADC_GRP2_BUF_DEPTH);

  /*
   * Normal main() thread activity, in this demo it does nothing.
   */
  while (true) {
    if (palReadPad(GPIOA, GPIOA_WKUP_BUTTON)) {
      adcStopConversion(&SDADCD1);
    }
    chThdSleepMilliseconds(500);
  }
}
