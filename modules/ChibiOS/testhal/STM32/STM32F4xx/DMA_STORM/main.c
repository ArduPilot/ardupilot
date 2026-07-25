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

#include <string.h>

#include "ch.h"
#include "hal.h"

/**
 * DMA operation timeout failure handler.
 */
static void tmo(virtual_timer_t *vtp, void *p) {

  (void)vtp;

  chSysHalt((const char *)p);
}

/*--------------------------------------------------------------------------*/
/* ADC related code.                                                        */
/*--------------------------------------------------------------------------*/

#define ADC_GRP2_NUM_CHANNELS   8
#define ADC_GRP2_BUF_DEPTH      16

static virtual_timer_t adcvt;

static adcsample_t samples2[ADC_GRP2_NUM_CHANNELS * ADC_GRP2_BUF_DEPTH];

static void adccallback(ADCDriver *adcp) {

  (void)adcp;

  chSysLockFromISR();
  chVTSetI(&adcvt, TIME_MS2I(10), tmo, (void *)"ADC timeout");
  chSysUnlockFromISR();
}

static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {

  (void)adcp;
  (void)err;
  chSysHalt("ADC error");
}

/*
 * ADC conversion group.
 * Mode:        Continuous, 16 samples of 8 channels, SW triggered.
 * Channels:    IN11, IN12, IN11, IN12, IN11, IN12, Sensor, VRef.
 */
static const ADCConversionGroup adcgrpcfg2 = {
  true,
  ADC_GRP2_NUM_CHANNELS,
  adccallback,
  adcerrorcallback,
  0,                        /* CR1 */
  ADC_CR2_SWSTART,          /* CR2 */
  ADC_SMPR1_SMP_AN12(ADC_SAMPLE_56) | ADC_SMPR1_SMP_AN11(ADC_SAMPLE_56) |
  ADC_SMPR1_SMP_SENSOR(ADC_SAMPLE_144) | ADC_SMPR1_SMP_VREF(ADC_SAMPLE_144),
  0,                        /* SMPR2 */
  0,                        /* HTR */
  0,                        /* LTR */
  0,                        /* SQR1 */
  ADC_SQR2_SQ8_N(ADC_CHANNEL_SENSOR) | ADC_SQR2_SQ7_N(ADC_CHANNEL_VREFINT),
  ADC_SQR3_SQ6_N(ADC_CHANNEL_IN12)   | ADC_SQR3_SQ5_N(ADC_CHANNEL_IN11) |
  ADC_SQR3_SQ4_N(ADC_CHANNEL_IN12)   | ADC_SQR3_SQ3_N(ADC_CHANNEL_IN11) |
  ADC_SQR3_SQ2_N(ADC_CHANNEL_IN12)   | ADC_SQR3_SQ1_N(ADC_CHANNEL_IN11)
};

/*--------------------------------------------------------------------------*/
/* SPI related code.                                                        */
/*--------------------------------------------------------------------------*/

/*
 * Maximum speed SPI configuration (21MHz, CPHA=0, CPOL=0, MSb first).
 */
static const SPIConfig hs_spicfg = {
  .circular         = false,
  .slave            = false,
  .data_cb          = NULL,
  .error_cb         = NULL,
  .ssport           = GPIOB,
  .sspad            = 12,
  .cr1              = 0,
  .cr2              = 0
};

/*
 * SPI thread.
 */
static THD_WORKING_AREA(waSPI1, 1024);
static THD_WORKING_AREA(waSPI2, 1024);
static THD_WORKING_AREA(waSPI3, 1024);
static THD_FUNCTION(spi_thread, p) {
  unsigned i;
  SPIDriver *spip = (SPIDriver *)p;
  virtual_timer_t vt;
  uint8_t txbuf[256];
  uint8_t rxbuf[256];

  chRegSetThreadName("SPI overlord");

  chVTObjectInit(&vt);

  /* Prepare transmit pattern.*/
  for (i = 0; i < sizeof(txbuf); i++)
    txbuf[i] = (uint8_t)i;

  /* Continuous transmission.*/
  while (true) {
    /* Starts a VT working as watchdog to catch a malfunction in the SPI
       driver.*/
    chVTSet(&vt, TIME_MS2I(10), tmo, (void *)"SPI timeout");

    spiExchange(spip, sizeof(txbuf), txbuf, rxbuf);

    /* Stops the watchdog.*/
    chVTReset(&vt);
  }
}

/*--------------------------------------------------------------------------*/
/* Application code.                                                        */
/*--------------------------------------------------------------------------*/

/*
 * This is a periodic thread that does absolutely nothing except flashing
 * a LED. If the application fails then the LED stops flashing.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    palSetPad(GPIOD, GPIOD_LED3);       /* Orange.  */
    chThdSleepMilliseconds(500);
    palClearPad(GPIOD, GPIOD_LED3);     /* Orange.  */
    chThdSleepMilliseconds(500);
  }
}

/*
 * Application entry point.
 */
int main(void) {
  unsigned i;
  static uint8_t patterns1[4096], patterns2[4096], buf1[4096], buf2[4096];

  /* System initializations.
     - HAL initialization, this also initializes the configured device drivers
       and performs the board-specific initializations.
     - Kernel initialization, the main() function becomes a thread and the
       RTOS is active.*/
  halInit();
  chSysInit();

  /* Creates the blinker thread.*/
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 10,
                    Thread1, NULL);

  /* Activates the ADC1 driver and the temperature sensor.*/
  adcStart(&ADCD1, NULL);
  adcSTM32EnableTSVREFE();

  /* Starts an ADC continuous conversion and its watchdog virtual timer.*/
  chVTSet(&adcvt, TIME_MS2I(10), tmo, (void *)"ADC timeout");
  adcStartConversion(&ADCD1, &adcgrpcfg2, samples2, ADC_GRP2_BUF_DEPTH);

  /* Activating SPI drivers.*/
  spiStart(&SPID1, &hs_spicfg);
  spiStart(&SPID2, &hs_spicfg);
  spiStart(&SPID3, &hs_spicfg);

  /* Starting SPI threads instances.*/
  chThdCreateStatic(waSPI1, sizeof(waSPI1), NORMALPRIO + 1, spi_thread, &SPID1);
  chThdCreateStatic(waSPI2, sizeof(waSPI2), NORMALPRIO + 1, spi_thread, &SPID2);
  chThdCreateStatic(waSPI3, sizeof(waSPI3), NORMALPRIO + 1, spi_thread, &SPID3);

  /* Allocating two DMA2 streams for memory copy operations.*/
  if (dmaStreamAlloc(STM32_DMA_STREAM_ID(2, 6), 0, NULL, NULL) == NULL)
    chSysHalt("DMA already in use");
  if (dmaStreamAlloc(STM32_DMA_STREAM_ID(2, 7), 0, NULL, NULL) == NULL)
    chSysHalt("DMA already in use");
  for (i = 0; i < sizeof (patterns1); i++)
    patterns1[i] = (uint8_t)i;
  for (i = 0; i < sizeof (patterns2); i++)
    patterns2[i] = (uint8_t)(i ^ 0xAA);

  /* Normal main() thread activity, it does continues memory copy operations
     using 2 DMA streams at the lowest priority.*/
  while (true) {
    virtual_timer_t vt;

    chVTObjectInit(&vt);

    /* Starts a VT working as watchdog to catch a malfunction in the DMA
       driver.*/
    chVTSet(&vt, TIME_MS2I(10), tmo, (void *)"copy timeout");

    /* Copy pattern 1.*/
    dmaStartMemCopy(STM32_DMA2_STREAM6,
                    STM32_DMA_CR_PL(0) | STM32_DMA_CR_PSIZE_BYTE |
                                         STM32_DMA_CR_MSIZE_BYTE,
                    patterns1, buf1, sizeof (patterns1));
    dmaStartMemCopy(STM32_DMA2_STREAM7,
                    STM32_DMA_CR_PL(0) | STM32_DMA_CR_PSIZE_BYTE |
                                         STM32_DMA_CR_MSIZE_BYTE,
                    patterns1, buf2, sizeof (patterns1));
    dmaWaitCompletion(STM32_DMA2_STREAM6);
    dmaWaitCompletion(STM32_DMA2_STREAM7);
    if (memcmp(patterns1, buf1, sizeof (patterns1)))
      chSysHalt("pattern error");
    if (memcmp(patterns1, buf2, sizeof (patterns1)))
      chSysHalt("pattern error");

    /* Copy pattern 2.*/
    dmaStartMemCopy(STM32_DMA2_STREAM6,
                    STM32_DMA_CR_PL(0) | STM32_DMA_CR_PSIZE_BYTE |
                                         STM32_DMA_CR_MSIZE_BYTE,
                    patterns2, buf1, sizeof (patterns2));
    dmaStartMemCopy(STM32_DMA2_STREAM7,
                    STM32_DMA_CR_PL(0) | STM32_DMA_CR_PSIZE_BYTE |
                                         STM32_DMA_CR_MSIZE_BYTE,
                    patterns2, buf2, sizeof (patterns2));
    dmaWaitCompletion(STM32_DMA2_STREAM6);
    dmaWaitCompletion(STM32_DMA2_STREAM7);
    if (memcmp(patterns2, buf1, sizeof (patterns2)))
      chSysHalt("pattern error");
    if (memcmp(patterns2, buf2, sizeof (patterns2)))
      chSysHalt("pattern error");

    /* Stops the watchdog.*/
    chVTReset(&vt);

    chThdSleepMilliseconds(2);
  }
  return 0;
}
