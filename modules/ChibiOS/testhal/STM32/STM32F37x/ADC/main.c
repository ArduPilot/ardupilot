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

#define ADC_GRP1_NUM_CHANNELS   2
#define ADC_GRP1_BUF_DEPTH      8

#define ADC_GRP2_NUM_CHANNELS   8
#define ADC_GRP2_BUF_DEPTH      16

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
 * ADC conversion group.
 * Mode:        Linear buffer, 8 samples of 2 channels, SW triggered.
 * Channels:    IN1, IN9.
 */
static const ADCConversionGroup adcgrpcfg1 = {
  FALSE,
  ADC_GRP1_NUM_CHANNELS,
  NULL,
  adcerrorcallback,
  .u.adc = {
    0,                      /* CR1      */
    ADC_CR2_SWSTART,        /* CR2      */
    0,                      /* LTR      */
    0,                      /* HTR      */
    {                       /* SMPR[2]  */
      0,
      ADC_SMPR2_SMP_AN9(ADC_SAMPLE_41P5) | ADC_SMPR2_SMP_AN1(ADC_SAMPLE_41P5),
    },
    {                       /* SQR[3]   */
      0,
      0,
      ADC_SQR3_SQ2_N(ADC_CHANNEL_IN9) | ADC_SQR3_SQ1_N(ADC_CHANNEL_IN1)
    }
  }
};

/*
 * ADC conversion group.
 * Mode:        Continuous, 16 samples of 8 channels, SW triggered.
 * Channels:    IN1, IN9, IN1, IN9, IN1, IN9, VBat, Sensor.
 */
static const ADCConversionGroup adcgrpcfg2 = {
  TRUE,
  ADC_GRP2_NUM_CHANNELS,
  adccallback,
  adcerrorcallback,
  .u.adc = {
    0,                      /* CR1      */
    ADC_CR2_SWSTART,        /* CR2      */
    0,                      /* LTR      */
    0,                      /* HTR      */
    {                       /* SMPR[2]  */
      ADC_SMPR1_SMP_SENSOR(ADC_SAMPLE_239P5) | ADC_SMPR1_SMP_VREF(ADC_SAMPLE_239P5),
      ADC_SMPR2_SMP_AN9(ADC_SAMPLE_41P5)     | ADC_SMPR2_SMP_AN1(ADC_SAMPLE_41P5)
    },
    {                       /* SQR[3]  */
      0,
      ADC_SQR2_SQ8_N(ADC_CHANNEL_SENSOR) | ADC_SQR2_SQ7_N(ADC_CHANNEL_VBAT),
      ADC_SQR3_SQ6_N(ADC_CHANNEL_IN9)    | ADC_SQR3_SQ5_N(ADC_CHANNEL_IN1) |
      ADC_SQR3_SQ4_N(ADC_CHANNEL_IN9)    | ADC_SQR3_SQ3_N(ADC_CHANNEL_IN1) |
      ADC_SQR3_SQ2_N(ADC_CHANNEL_IN9)    | ADC_SQR3_SQ1_N(ADC_CHANNEL_IN1)
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
   * Activates the ADC1 driver, the temperature sensor and the VBat
   * measurement.
   */
  adcStart(&ADCD1, NULL);
  adcSTM32Calibrate(&ADCD1);
  adcSTM32EnableTSVREFE();
  adcSTM32EnableVBATE();

  /*
   * Linear conversion.
   */
  adcConvert(&ADCD1, &adcgrpcfg1, samples1, ADC_GRP1_BUF_DEPTH);
  chThdSleepMilliseconds(1000);

  /*
   * Starts an ADC continuous conversion.
   */
  adcStartConversion(&ADCD1, &adcgrpcfg2, samples2, ADC_GRP2_BUF_DEPTH);

  /*
   * Normal main() thread activity, in this demo it does nothing.
   */
  while (true) {
    if (palReadPad(GPIOA, GPIOA_WKUP_BUTTON)) {
      adcStopConversion(&ADCD1);
    }
    chThdSleepMilliseconds(500);
  }
}
