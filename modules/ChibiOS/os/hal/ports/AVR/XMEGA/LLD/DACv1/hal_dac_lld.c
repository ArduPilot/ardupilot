/*
    ChibiOS - Copyright (C) 2016..2018 Theodore Ateba

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
 * @file    hal_dac_lld.c
 * @brief   AVR DAC subsystem low level driver source.
 *
 * @addtogroup DAC
 * @{
 */

#include "hal.h"

#if (HAL_USE_DAC == TRUE) || defined(__DOXYGEN__)

/*==========================================================================*/
/* Driver local definitions.                                                */
/*==========================================================================*/

/*==========================================================================*/
/* Driver exported variables.                                               */
/*==========================================================================*/

/** @brief DAC1 driver identifier.*/
#if (AVR_DAC_USE_DAC1 == TRUE) || defined(__DOXYGEN__)
DACDriver DACD1;
#endif

/*==========================================================================*/
/* Driver local variables.                                                  */
/*==========================================================================*/

/*==========================================================================*/
/* Driver local functions.                                                  */
/*==========================================================================*/

/*==========================================================================*/
/* Driver interrupt handlers.                                               */
/*==========================================================================*/

/*==========================================================================*/
/* Driver exported functions.                                               */
/*==========================================================================*/

/**
 * @brief   Check if channel data register is empty.
 *
 * @detail  This function return the status of the datt register empty flag
 *          for the selected channel in the given DAC module. This can be used
 *          to get the status of the register before writing a new value to it.
 *
 * @param[in]  dac     pointer to DAC module register section
 * @param[in]  channel pelected channel in the DAC module, either CH0 or CH1
 *
 * @retval dacStatus True if data register is empty
 * @retval dacStatus False if data register is not empty
 */
static bool dac_is_channel_data_empty(DACDriver *dacp) {

  bool dacStatus = (dacp->dacblock->STATUS &
  (dacp->config->ch ? DAC_CH1DRE_bm : DAC_CH0DRE_bm));

  return dacStatus;
}

/**
 * @brief   Configure the DAC trigger mode.
 *
 * @param[in] dacp    pointer to the DAC driver object
 * @param[in] channel channel on wich the trigger must be configure
 * @param[in] tm      trigger mode to use for the DAC configuration
 */
static void dac_set_trigger_mode(DACDriver *dacp) {

  if (dacp->config->ch == DAC_CHANNEL0) {
    if (dacp->config->tm == DAC_TRIG_ON_DATAREG)
      dacp->dacblock->CTRLB &= ~(DAC_CH0TRIG_bm);
    else
      dacp->dacblock->CTRLB |= (DAC_CH0TRIG_bm);
  }
  else {
    if (dacp->config->tm == DAC_TRIG_ON_DATAREG)
      dacp->dacblock->CTRLB &= ~(DAC_CH1TRIG_bm);
    else
      dacp->dacblock->CTRLB |= (DAC_CH1TRIG_bm);
  }
}

/**
 * @brief   Configure the DAC operation (Single/Dual).
 * @note    In single channel operation, the DAC conversion block is always
 *          connected the data register of the channel 0.
 *
 * @param[in] dacp  pointer to the DAC driver object
 * @param[in] om    dac operation mode
 */
static void dac_set_operation_mode(DACDriver *dacp) {

  if (dacp->config->om == DAC_OPMODE_SINGLE)
    dacp->dacblock->CTRLB = (dacp->dacblock->CTRLB & ~DAC_CHSEL_gm) | DAC_CHSEL_SINGLE_gc;
  else
    dacp->dacblock->CTRLB = (dacp->dacblock->CTRLB & ~DAC_CHSEL_gm) | DAC_CHSEL_DUAL_gc;
}

/**
 * @brief   Configure the DAC to accept Left or Right ajusted value.
 *
 * @param[in] dacp  pointer to the DAC driver object
 * @param[in] da    data ajustment.
 */
static void dac_set_ajusted_mode(DACDriver *dacp) {

  dacp->dacblock->CTRLC = (dacp->dacblock->CTRLC & ~(DAC_LEFTADJ_bm)) |
  (dacp->config->da ? DAC_LEFTADJ_bm : 0x00);
}

/**
 * @brief   Configure the DAC voltage reference.
 *
 * @param[in] dacp  pointer to the DAC driver object
 * @param[in] vr    voltage reference.
 */
static void dac_set_voltage_ref(DACDriver *dacp) {

  dacp->dacblock->CTRLC = (dacp->dacblock->CTRLC & ~(DAC_REFSEL_gm)) |
  dacp->config->vr;
}

/**
 * @brief   Low level DAC driver initialization.
 *
 * @notapi
 */
void dac_lld_init(void) {

#if AVR_DAC_USE_DAC1 == TRUE
  dacObjectInit(&DACD1);
#endif
}

/**
 * @brief   Configures and activates the DAC peripheral.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 *
 * @notapi
 */
void dac_lld_start(DACDriver *dacp) {

  /* If the driver is in DAC_STOP state then a full initialization is
     required. */
  if (dacp->state == DAC_STOP) {
    dac_set_trigger_mode(dacp);
    dac_set_operation_mode(dacp);
    dac_set_ajusted_mode(dacp);
    dac_set_voltage_ref(dacp);

    /* Enabling the DAC peripheral. */
#if AVR_DAC_USE_DAC1 == TRUE
    if (&DACD1 == dacp) {
      dacp->dacblock->CTRLA |= DAC_ENABLE_bm;
    }
#endif
  }
}

/**
 * @brief   Deactivates the DAC peripheral.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 *
 * @notapi
 */
void dac_lld_stop(DACDriver *dacp) {

  /* If in ready state then disables the DAC clock. */
  if (dacp->state == DAC_READY) {
    /* Disabling DAC.*/
#if AVR_DAC_USE_DAC1 == TRUE
    if (&DACD1 == dacp) {
      dacp->dacblock->CTRLA &= ~DAC_ENABLE_bm;
    }
#endif
  }
}

/**
 * @brief   Outputs a value directly on a DAC channel.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 * @param[in] channel   DAC channel number
 * @param[in] sample    value to be output
 *
 * @api
 */
void dac_lld_put_channel(DACDriver        *dacp,
                           dacchannel_t   channel,
                           dacsample_t    sample) {

  if (channel == DAC_CHANNEL0) {
    dacp->dacblock->CH0DATA = sample;
  }
  else {
    dacp->dacblock->CH1DATA = sample;
  }
}

/**
 * @brief   Starts a DAC conversion.
 * @details Starts an asynchronous conversion operation.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 *
 * @notapi
 */
void dac_lld_start_conversion(DACDriver *dacp) {

  /* FIXME: For the moment just the single mode is implemented.
   * FIXME: There is a buffer to output: All sample must be send to the DAC.
   */
  while (dacp->depth >= 0) {
    if (dac_is_channel_data_empty(dacp)) {
      dacp->dacblock->CH0DATA = *dacp->samples;
      dacp->depth--;
    }
  }
}

/**
 * @brief   Stops an ongoing conversion.
 * @details This function stops the currently ongoing conversion and returns
 *          the driver in the @p DAC_READY state. If there was no conversion
 *          being processed then the function does nothing.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 *
 * @iclass
 */
void dac_lld_stop_conversion(DACDriver *dacp) {

  (void)dacp;
  /* TODO: Must be implemented. */
}

#endif /* HAL_USE_DAC == TRUE */

/** @} */
