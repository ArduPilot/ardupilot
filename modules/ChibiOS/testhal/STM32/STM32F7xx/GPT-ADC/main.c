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

/* TRUE means that DMA-accessible buffers are placed in a non-cached RAM
   area and that no cache management is required.*/
#define DMA_BUFFERS_COHERENCE TRUE

/*===========================================================================*/
/* GPT driver related.                                                       */
/*===========================================================================*/

/*
 * GPT4 configuration. This timer is used as trigger for the ADC.
 */
static const GPTConfig gpt4cfg1 = {
  .frequency =  1000000U,
  .callback  =  NULL,
  .cr2       =  TIM_CR2_MMS_1,  /* MMS = 010 = TRGO on Update Event.        */
  .dier      =  0U
};

/*===========================================================================*/
/* ADC driver related.                                                       */
/*===========================================================================*/

#define ADC_GRP1_NUM_CHANNELS   2
#define ADC_GRP1_BUF_DEPTH      64

#if !DMA_BUFFERS_COHERENCE
/* Note, the buffer is aligned to a 32 bytes boundary because limitations
   imposed by the data cache. Note, this is GNU specific, it must be
   handled differently for other compilers.
   Only required if the ADC buffer is placed in a cache-able area.*/
#if defined(__GNUC__)
__attribute__((aligned (32)))
#endif
#endif
static adcsample_t samples1[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];

/*
 * ADC streaming callback.
 */
size_t nx = 0, ny = 0;
static void adccallback(ADCDriver *adcp) {

#if !DMA_BUFFERS_COHERENCE
  /* DMA buffer invalidation because data cache, only invalidating the
     half buffer just filled.
     Only required if the ADC buffer is placed in a cache-able area.*/
  dmaBufferInvalidate(buffer,
                      n * adcp->grpp->num_channels * sizeof (adcsample_t));
#endif

  /* Updating counters.*/
  if (adcIsBufferComplete(adcp)) {
    nx += 1;
  }
  else {
    ny += 1;
  }
}

/*
 * ADC errors callback, should never happen.
 */
static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {

  (void)adcp;
  (void)err;
}

/*
 * ADC conversion group.
 * Mode:        Continuous, 16 samples of 2 channels, HS triggered by
 *              GPT4-TRGO.
 * Channels:    Sensor, VRef.
 */
static const ADCConversionGroup adcgrpcfg1 = {
  true,
  ADC_GRP1_NUM_CHANNELS,
  adccallback,
  adcerrorcallback,
  0,                                                    /* CR1   */
  ADC_CR2_EXTEN_RISING | ADC_CR2_EXTSEL_SRC(12),        /* CR2   */
  ADC_SMPR1_SMP_SENSOR(ADC_SAMPLE_144) | 
  ADC_SMPR1_SMP_VREF(ADC_SAMPLE_144),                   /* SMPR1 */
  0,                                                    /* SMPR2 */
  0,                                                    /* HTR */
  0,                                                    /* LTR */
  0,                                                    /* SQR1  */
  0,                                                    /* SQR2  */
  ADC_SQR3_SQ2_N(ADC_CHANNEL_SENSOR) | 
  ADC_SQR3_SQ1_N(ADC_CHANNEL_VREFINT)                   /* SQR3  */
};

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
  palSetLineMode(LINE_ARD_D13, PAL_MODE_OUTPUT_PUSHPULL);
  while (true) {
    palSetLine(LINE_ARD_D13);
    chThdSleepMilliseconds(500);
    palClearLine(LINE_ARD_D13);
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
   * Activates the serial driver 1 using the driver default configuration.
   */
  sdStart(&SD1, NULL);

  /*
   * Starting GPT4 driver, it is used for triggering the ADC.
   */
  gptStart(&GPTD4, &gpt4cfg1);

  /*
   * Fixed an errata on the STM32F7xx, the DAC clock is required for ADC
   * triggering.
   */
  rccEnableDAC1(false);

  /*
   * Activates the ADC1 driver and the temperature sensor.
   */
  adcStart(&ADCD1, NULL);
  adcSTM32EnableTSVREFE();

  /*
   * Starts an ADC continuous conversion triggered with a period of
   * 1/10000 second.
   */
  adcStartConversion(&ADCD1, &adcgrpcfg1, samples1, ADC_GRP1_BUF_DEPTH);
  gptStartContinuous(&GPTD4, 100);

  /*
   * Creates the example thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /*
   * Normal main() thread activity, in this demo it does nothing.
   */
  while (true) {
    chThdSleepMilliseconds(500);
  }
}
