/*
    ChibiOS - Copyright (C) 2016...2021 Theodore Ateba

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

/*
 * In this demo we use a single channel to sample voltage across
 * a potentiometer.
 */
#define MY_NUM_CH           1
#define MY_SAMPLING_NUMBER  10

/**
 * @brief Global variables
 */
BaseSequentialStream * chp = (BaseSequentialStream *) &SD1;
static int32_t  adcConvA0 = 0;      /**< A0 adc conversion. */
static float    voltageA0 = 0;      /**< A0 voltage.        */
static adcsample_t sample_buff[MY_NUM_CH * MY_SAMPLING_NUMBER];

/*
 * ADC conversion group.
 * Mode:        Linear buffer, 10 samples of 1 channel, SW triggered.
 * Channels:    IN0 (Arduino Pin A0).
 */
static const ADCConversionGroup my_conversion_group = {
  FALSE,      /* Not circular buffer.           */
  MY_NUM_CH,  /* Number of channels.            */
  NULL,       /* No ADC callback function.      */
  NULL,       /* No ADC Callback erro function. */
  1,          /* Channel mask.                  */
};

static const ADCConfig adcConfig = {
  ANALOG_REFERENCE_AVCC, /* Analog reference. */
};

static THD_WORKING_AREA(waThd1, 32);
static THD_FUNCTION(Thd1, arg) {
  (void) arg;
  chRegSetThreadName("LED Thread");

  while (true) {
    palTogglePad(IOPORT2, PORTB_LED1);
    chThdSleepMilliseconds(1000);
  }
}

static THD_WORKING_AREA(waThd2, 512);
static THD_FUNCTION(Thd2, arg) {

  unsigned i;
  (void) arg;
  chRegSetThreadName("ADC1 Thread");

  /* Activates the ADC1 driver. */
  adcStart(&ADCD1, &adcConfig);

  while (TRUE) {
    /* Make ADC conversion of the voltage on A0. */
    adcConvert(&ADCD1, &my_conversion_group, sample_buff, MY_SAMPLING_NUMBER);

    /* Making mean of sampled values.*/
    for (i = 0; i < MY_NUM_CH * MY_SAMPLING_NUMBER; i++) {
      adcConvA0 += sample_buff[i];
    }

    adcConvA0 /= (MY_NUM_CH * MY_SAMPLING_NUMBER);
    voltageA0 = (((float)adcConvA0 * 5) / 1024);
  }
}

/**
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
  * Initialized the serial driver an turn off the onboard debug led.
  */
  sdStart(&SD1, NULL);
  palClearPad(IOPORT2, PORTB_LED1);

  /*
   * Starts the LED blinker thread.
   */
  chThdCreateStatic(waThd1, sizeof(waThd1), NORMALPRIO + 1, Thd1, NULL);
  chThdCreateStatic(waThd2, sizeof(waThd2), NORMALPRIO + 1, Thd2, NULL);

  chprintf(chp, "AVR ADC program example... \r\n");

  while (TRUE) {
    chprintf(chp, "Measure the voltage on A0 pin: \r\n");
    chprintf(chp, "   %.3fv \r\n", voltageA0);
    chThdSleepMilliseconds(100);
    chprintf(chp, "\033[2J\033[1;1H");
  }
}

