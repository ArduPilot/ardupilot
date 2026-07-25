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

/**
 * @file    ADCv1/hal_adc_lld.c
 * @brief   AVR/MEGA ADC subsystem low level driver source.
 *
 * @addtogroup ADC
 * @{
 */

#include "hal.h"

#if HAL_USE_ADC || defined(__DOXYGEN__)

/*==========================================================================*/
/* Driver local definitions.                                                */
/*==========================================================================*/

/*==========================================================================*/
/* Driver exported variables.                                               */
/*==========================================================================*/
/** @brief ADC1 driver identifier.*/
#if AVR_ADC_USE_ADC1 || defined(__DOXYGEN__)
ADCDriver ADCD1;
#endif

/*==========================================================================*/
/* Driver local variables.                                                  */
/*==========================================================================*/

/*==========================================================================*/
/* Driver local functions.                                                  */
/*==========================================================================*/

/**
 * @brief   Get the ADC channel.
 *
 * @param[in] mask            the mask containing the channel number
 * @param[in] currentChannel  the current channel.
 *
 * @return                    the channel number.
 * @retval                    ADC channel number
 * @retval                    -1 in case of error.
 */
static size_t getAdcChannelNumberFromMask(uint8_t mask,
                                          uint8_t currentChannel) {

  for (uint8_t i = 0; mask > 0; i++) {
    if (mask & 0x01) {
      if (!currentChannel)
        return i;
      currentChannel--;
    }
    mask >>= 1;
  }

  /* Error, should never reach this line */
  return -1;
}

/**
 * @brief   Configure the ADC channel.
 *
 * @param[in] channelNum  the channel number to set.
 */
static void setAdcChannel(uint8_t channelNum) {

  ADMUX = (ADMUX & 0xf8) | (channelNum & 0x07);
}

/*==========================================================================*/
/* Driver interrupt handlers.                                               */
/*==========================================================================*/

#include <util/delay.h>

OSAL_IRQ_HANDLER(ADC_vect) {

  OSAL_IRQ_PROLOGUE();
  uint8_t low = ADCL;
  uint8_t high = ADCH;
  uint16_t result = (high << 8) | low;

  ADCD1.samples[ADCD1.currentBufferPosition] = result;
  ADCD1.currentBufferPosition++;

  size_t bufferSize = ADCD1.depth * ADCD1.grpp->num_channels;
  size_t currentChannel = ADCD1.currentBufferPosition % ADCD1.grpp->num_channels;
  size_t currentIteration = ADCD1.currentBufferPosition / ADCD1.grpp->num_channels;
  if (ADCD1.grpp->circular && currentChannel == 0 && currentIteration == ADCD1.depth/2) {
    _adc_isr_half_code(&ADCD1);
  }

  if (ADCD1.currentBufferPosition == bufferSize) {
    _adc_isr_full_code(&ADCD1);
  }
  else {
    setAdcChannel(getAdcChannelNumberFromMask(ADCD1.grpp->channelsMask, currentChannel));
    ADCSRA |= 1 << ADSC;
  }

  OSAL_IRQ_EPILOGUE();
}

/*==========================================================================*/
/* Driver exported functions.                                               */
/*==========================================================================*/

/**
 * @brief   Low level ADC driver initialization.
 *
 * @notapi
 */
void adc_lld_init(void) {

  adcObjectInit(&ADCD1);

  /* Prescaler 128, only value possible at 20Mhz, interrupt. */
  ADCSRA = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADIE);

  /* Uso aref, only valid for arduino. arduino ha aref collegato. */
  ADMUX = (0 << REFS1) | (0 << REFS0);
}

/**
 * @brief   Configures and activates the ADC peripheral.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_lld_start(ADCDriver *adcp) {

  if (adcp->state == ADC_STOP) {
    /* Clock activation. */
    ADCSRA |= (1 << ADEN);
  }

  if (adcp->config != NULL) {
    ADMUX = (adcp->config->analog_reference << REFS0);
  }
}

/**
 * @brief   Deactivates the ADC peripheral.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_lld_stop(ADCDriver *adcp) {

  if (adcp->state == ADC_READY) {
    /* Clock de-activation. */
    ADCSRA &= ~(1 << ADEN);
  }
}

/**
 * @brief   Starts an ADC conversion.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_lld_start_conversion(ADCDriver *adcp) {

  adcp->currentBufferPosition = 0;

  setAdcChannel(getAdcChannelNumberFromMask(adcp->grpp->channelsMask, 0));
  ADCSRA |= 1 << ADSC;
}

/**
 * @brief   Stops an ongoing conversion.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_lld_stop_conversion(ADCDriver *adcp) {

  ADCSRA &= ~(1 << ADSC);
}

#endif /* HAL_USE_ADC */

/** @} */
