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
 * @file    ADCv7/hal_adc_lld.c
 * @brief   STM32 ADC subsystem low level driver source.
 *
 * @addtogroup ADC
 * @{
 */

#include "hal.h"

#if HAL_USE_ADC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#if STM32_ADC_DUAL_MODE
#if STM32_ADC_COMPACT_SAMPLES
/* Compact type dual mode.*/
#define ADC_DMA3_CTR1_SIZE   (STM32_DMA3_CTR1_DDW_HALF | STM32_DMA3_CTR1_SDW_HALF)
#define ADC_CCR_DAMDF_MODE   ADC_CCR_DAMDF_BYTE
#else /* !STM32_ADC_COMPACT_SAMPLES */
/* Large type dual mode.*/
#define ADC_DMA3_CTR1_SIZE   (STM32_DMA3_CTR1_DDW_WORD | STM32_DMA3_CTR1_SDW_WORD)
#define ADC_CCR_DAMDF_MODE   ADC_CCR_DAMDF_HWORD
#endif /* !STM32_ADC_COMPACT_SAMPLES */
#else /* !STM32_ADC_DUAL_MODE */
#if STM32_ADC_COMPACT_SAMPLES
/* Compact type single mode.*/
#define ADC_DMA3_CTR1_SIZE   (STM32_DMA3_CTR1_DDW_BYTE | STM32_DMA3_CTR1_SDW_BYTE)
#define ADC_CCR_DAMDF_MODE   ADC_CCR_DAMDF_DISABLED
#else /* !STM32_ADC_COMPACT_SAMPLES */
/* Large type single mode.*/
#define ADC_DMA3_CTR1_SIZE   (STM32_DMA3_CTR1_DDW_HALF | STM32_DMA3_CTR1_SDW_HALF)
#define ADC_CCR_DAMDF_MODE   ADC_CCR_DAMDF_DISABLED
#endif /* !STM32_ADC_COMPACT_SAMPLES */
#endif /* !STM32_ADC_DUAL_MODE */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief ADC1 driver identifier.*/
#if STM32_ADC_USE_ADC1 || defined(__DOXYGEN__)
ADCDriver ADCD1;
#endif

/** @brief ADC2 driver identifier.*/
#if STM32_ADC_USE_ADC2 || defined(__DOXYGEN__)
ADCDriver ADCD2;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

static const ADCConfig default_config = {
  .dmactr1     = 0U,
  .dmactr2     = 0U
};

static uint32_t clkmask;

#if STM32_ADC_USE_ADC1 || defined(__DOXYGEN__)
static adc_dmabuf_t __dma3_adc1;
#endif

#if STM32_ADC_USE_ADC2 || defined(__DOXYGEN__)
static adc_dmabuf_t __dma3_adc2;
#endif

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Enables the ADC voltage regulator.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 */
static void adc_lld_vreg_on(ADCDriver *adcp) {

  adcp->adcm->CR = 0U;
  adcp->adcm->CR = ADC_CR_ADVREGEN;
#if STM32_ADC_DUAL_MODE
  adcp->adcs->CR = 0U;
  adcp->adcs->CR = ADC_CR_ADVREGEN;
#endif
  osalSysPolledDelayX(OSAL_US2RTC(STM32_HCLK, 20U));
}

/**
 * @brief   Disables the ADC voltage regulator.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 */
static void adc_lld_vreg_off(ADCDriver *adcp) {

  adcp->adcm->CR = 0U;
  adcp->adcm->CR = ADC_CR_DEEPPWD;
#if STM32_ADC_DUAL_MODE
  adcp->adcs->CR = 0U;
  adcp->adcs->CR = ADC_CR_DEEPPWD;
#endif
}

/**
 * @brief   Calibrates an ADC unit.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 */
static void adc_lld_calibrate(ADCDriver *adcp) {

  osalDbgAssert(adcp->adcm->CR == ADC_CR_ADVREGEN, "invalid register state");

  /* Single-ended calibration for master ADC.*/
  adcp->adcm->CR = ADC_CR_ADVREGEN;
  adcp->adcm->CR = ADC_CR_ADVREGEN | ADC_CR_ADCAL;
  while ((adcp->adcm->CR & ADC_CR_ADCAL) != 0U) {
  }

  osalSysPolledDelayX(OSAL_US2RTC(STM32_HCLK, 20U));

#if STM32_ADC_DUAL_MODE
  osalDbgAssert(adcp->adcs->CR == ADC_CR_ADVREGEN, "invalid register state");

  /* Single-ended calibration for slave ADC.*/
  adcp->adcs->CR = ADC_CR_ADVREGEN;
  adcp->adcs->CR = ADC_CR_ADVREGEN | ADC_CR_ADCAL;
  while ((adcp->adcs->CR & ADC_CR_ADCAL) != 0U) {
  }

  osalSysPolledDelayX(OSAL_US2RTC(STM32_HCLK, 20U));
#endif
}

/**
 * @brief   Enables the ADC analog circuit.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 */
static void adc_lld_analog_on(ADCDriver *adcp) {

  adcp->adcm->ISR = ADC_ISR_ADRDY;
  adcp->adcm->CR |= ADC_CR_ADEN;
  while ((adcp->adcm->ISR & ADC_ISR_ADRDY) == 0U) {
  }
#if STM32_ADC_DUAL_MODE
  adcp->adcs->ISR = ADC_ISR_ADRDY;
  adcp->adcs->CR |= ADC_CR_ADEN;
  while ((adcp->adcs->ISR & ADC_ISR_ADRDY) == 0U) {
  }
#endif
}

/**
 * @brief   Disables the ADC analog circuit.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 */
static void adc_lld_analog_off(ADCDriver *adcp) {

  adcp->adcm->CR |= ADC_CR_ADDIS;
  while ((adcp->adcm->CR & ADC_CR_ADDIS) != 0U) {
  }
#if STM32_ADC_DUAL_MODE
  adcp->adcs->CR |= ADC_CR_ADDIS;
  while ((adcp->adcs->CR & ADC_CR_ADDIS) != 0U) {
  }
#endif
}

/**
 * @brief   Stops an ongoing conversion, if any.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 */
static void adc_lld_stop_adc(ADCDriver *adcp) {

  if ((adcp->adcm->CR & ADC_CR_ADSTART) != 0U) {
    adcp->adcm->CR |= ADC_CR_ADSTP;
    while ((adcp->adcm->CR & ADC_CR_ADSTP) != 0U) {
    }
    adcp->adcm->IER = 0U;
  }
  adcp->adcm->PCSEL = 0U;
}

/**
 * @brief   ADC DMA service routine.
 *
 * @param[in] p         parameter for the registered function
 * @param[in] csr       content of the CxSR register
 */
static void adc_lld_serve_dma_interrupt(void *p, uint32_t csr) {
  ADCDriver *adcp = (ADCDriver *)p;

  if ((csr & STM32_DMA3_CSR_ERRORS) != 0U) {
    _adc_isr_error_code(adcp, ADC_ERR_DMAFAILURE);
  }
  else if (adcp->grpp != NULL) {
    if ((csr & STM32_DMA3_CSR_TCF) != 0U) {
      _adc_isr_full_code(adcp);
    }
    else if ((csr & STM32_DMA3_CSR_HTF) != 0U) {
      _adc_isr_half_code(adcp);
    }
  }
}

/**
 * @brief   ADC ISR common service routine.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 * @param[in] isr       content of the ISR register
 */
static void adc_lld_serve_interrupt(ADCDriver *adcp, uint32_t isr) {

  if (adcp->grpp == NULL) {
    return;
  }

  adcerror_t emask = 0U;

  if ((isr & ADC_ISR_OVR) != 0U && adcp->state == ADC_ACTIVE) {
    emask |= ADC_ERR_OVERFLOW;
  }
  if ((isr & ADC_ISR_AWD1) != 0U) {
    emask |= ADC_ERR_AWD1;
  }
  if ((isr & ADC_ISR_AWD2) != 0U) {
    emask |= ADC_ERR_AWD2;
  }
  if ((isr & ADC_ISR_AWD3) != 0U) {
    emask |= ADC_ERR_AWD3;
  }
  if (emask != 0U) {
    _adc_isr_error_code(adcp, emask);
  }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if STM32_ADC_USE_ADC1 || defined(__DOXYGEN__)
/**
 * @brief   ADC1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_ADC1_HANDLER) {
  uint32_t isr;

  OSAL_IRQ_PROLOGUE();

  isr  = ADC1->ISR;
  ADC1->ISR = isr;
#if defined(STM32_ADC_ADC1_IRQ_HOOK)
  STM32_ADC_ADC1_IRQ_HOOK
#endif
  adc_lld_serve_interrupt(&ADCD1, isr);

  OSAL_IRQ_EPILOGUE();
}

#if STM32_ADC_DUAL_MODE
/**
 * @brief   ADC2 interrupt handler when operating as ADC1 slave.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_ADC2_HANDLER) {
  uint32_t isr;

  OSAL_IRQ_PROLOGUE();

  isr  = ADC2->ISR;
  ADC2->ISR = isr;

  adc_lld_serve_interrupt(&ADCD1, isr);

  OSAL_IRQ_EPILOGUE();
}
#endif /* STM32_ADC_DUAL_MODE */
#endif /* STM32_ADC_USE_ADC1 */

#if STM32_ADC_USE_ADC2 && !STM32_ADC_DUAL_MODE
/**
 * @brief   Standalone ADC2 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_ADC2_HANDLER) {
  uint32_t isr;

  OSAL_IRQ_PROLOGUE();

  isr  = ADC2->ISR;
  ADC2->ISR = isr;
#if defined(STM32_ADC_ADC2_IRQ_HOOK)
  STM32_ADC_ADC2_IRQ_HOOK
#endif
  adc_lld_serve_interrupt(&ADCD2, isr);

  OSAL_IRQ_EPILOGUE();
}
#endif /* STM32_ADC_USE_ADC2 && !STM32_ADC_DUAL_MODE */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level ADC driver initialization.
 *
 * @notapi
 */
void adc_lld_init(void) {

  clkmask = 0U;

#if STM32_ADC_USE_ADC1
  adcObjectInit(&ADCD1);
  ADCD1.adcm   = ADC1;
  ADCD1.adcc   = ADC12_COMMON;
#if STM32_ADC_DUAL_MODE
  ADCD1.adcs   = ADC2;
#endif
  ADCD1.dmachp = NULL;
  ADCD1.dprio  = STM32_ADC_ADC1_DMA_PRIORITY;
  ADCD1.dreq   = STM32_DMA3_REQ_ADC1;
  ADCD1.dbuf   = &__dma3_adc1;
#endif

#if STM32_ADC_USE_ADC2 && !STM32_ADC_DUAL_MODE
  adcObjectInit(&ADCD2);
  ADCD2.adcm   = ADC2;
  ADCD2.adcc   = ADC12_COMMON;
  ADCD2.dmachp = NULL;
  ADCD2.dprio  = STM32_ADC_ADC2_DMA_PRIORITY;
  ADCD2.dreq   = STM32_DMA3_REQ_ADC2;
  ADCD2.dbuf   = &__dma3_adc2;
#endif

#if STM32_ADC_USE_ADC1
  nvicEnableVector(STM32_ADC1_NUMBER, STM32_ADC_ADC1_IRQ_PRIORITY);
#if STM32_ADC_DUAL_MODE
  nvicEnableVector(STM32_ADC2_NUMBER, STM32_ADC_ADC1_IRQ_PRIORITY);
#endif
#endif

#if STM32_ADC_USE_ADC2 && !STM32_ADC_DUAL_MODE
  nvicEnableVector(STM32_ADC2_NUMBER, STM32_ADC_ADC2_IRQ_PRIORITY);
#endif

#if STM32_ADC_USE_ADC1 || STM32_ADC_USE_ADC2
  rccResetADC12();
#endif
}

/**
 * @brief   Configures and activates the ADC peripheral.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
msg_t adc_lld_start(ADCDriver *adcp) {

  if (adcp->config == NULL) {
    adcp->config = &default_config;
  }

  if (adcp->state == ADC_STOP) {
#if STM32_ADC_USE_ADC1
    if (&ADCD1 == adcp) {

      osalDbgAssert(STM32_ADC1_CLOCK <= STM32_ADCCLK_MAX,
                    "invalid clock frequency");

      adcp->dmachp = dma3ChannelAllocI(STM32_ADC_ADC1_DMA3_CHANNEL,
                                       STM32_ADC_ADC1_DMA_IRQ_PRIORITY,
                                       adc_lld_serve_dma_interrupt,
                                       (void *)adcp);
      osalDbgAssert(adcp->dmachp != NULL, "unable to allocate DMA channel");

      clkmask |= 1U;
      rccEnableADC12(true);
    }
#endif

#if STM32_ADC_USE_ADC2 && !STM32_ADC_DUAL_MODE
    if (&ADCD2 == adcp) {

      osalDbgAssert(STM32_ADC2_CLOCK <= STM32_ADCCLK_MAX,
                    "invalid clock frequency");

      adcp->dmachp = dma3ChannelAllocI(STM32_ADC_ADC2_DMA3_CHANNEL,
                                       STM32_ADC_ADC2_DMA_IRQ_PRIORITY,
                                       adc_lld_serve_dma_interrupt,
                                       (void *)adcp);
      osalDbgAssert(adcp->dmachp != NULL, "unable to allocate DMA channel");

      clkmask |= 2U;
      rccEnableADC12(true);
    }
#endif

    adc_lld_vreg_on(adcp);
    adc_lld_calibrate(adcp);
    adc_lld_analog_on(adcp);
  }

  return HAL_RET_SUCCESS;
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

    dma3ChannelDisable(adcp->dmachp);
    dma3ChannelFreeI(adcp->dmachp);
    adcp->dmachp = NULL;

    adc_lld_stop_adc(adcp);
    adc_lld_analog_off(adcp);
    adc_lld_vreg_off(adcp);

    if (&ADCD1 == adcp) {
      clkmask &= ~1U;
    }
#if STM32_ADC_USE_ADC2 && !STM32_ADC_DUAL_MODE
    if (&ADCD2 == adcp) {
      clkmask &= ~2U;
    }
#endif

    if ((clkmask & 0x3U) == 0U) {
      rccDisableADC12();
    }
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
  uint32_t cfgr1;
  uint32_t dmaccr;
  uint32_t dmallr;
  uint32_t ccr;
  const ADCConversionGroup *grpp = adcp->grpp;

#if STM32_ADC_DUAL_MODE
  osalDbgAssert((grpp->num_channels & 1U) == 0U, "odd number of channels");
#endif

  dmaccr = STM32_DMA3_CCR_PRIO((uint32_t)adcp->dprio) |
           STM32_DMA3_CCR_LAP_MEM                     |
           STM32_DMA3_CCR_TOIE                        |
           STM32_DMA3_CCR_USEIE                       |
           STM32_DMA3_CCR_ULEIE                       |
           STM32_DMA3_CCR_DTEIE                       |
           STM32_DMA3_CCR_TCIE;

  cfgr1 = grpp->cfgr & ~ADC_CFGR1_DMNGT_MASK;
  if (grpp->circular) {
    cfgr1 |= ADC_CFGR1_DMNGT_CIRCULAR;
    dmallr = STM32_DMA3_CLLR_UDA |
             (((uint32_t)&adcp->dbuf->cdar) & 0xFFFFU);
    adcp->dbuf->cdar = (uint32_t)adcp->samples;
    if (adcp->depth > 1U) {
      dmaccr |= STM32_DMA3_CCR_HTIE;
    }
  }
  else {
    cfgr1 |= ADC_CFGR1_DMNGT_ONESHOT;
    dmallr = 0U;
  }

  dma3ChannelSetDestination(adcp->dmachp, adcp->samples);
#if STM32_ADC_DUAL_MODE
  dma3ChannelSetSource(adcp->dmachp, &adcp->adcc->CDR);
  dma3ChannelSetTransactionSize(adcp->dmachp,
                                (((uint32_t)grpp->num_channels / 2U) *
                                 (uint32_t)adcp->depth) * ADC_SAMPLE_MULTIPLIER);
#else
  dma3ChannelSetSource(adcp->dmachp, &adcp->adcm->DR);
  dma3ChannelSetTransactionSize(adcp->dmachp,
                                ((uint32_t)grpp->num_channels *
                                 (uint32_t)adcp->depth) * ADC_SAMPLE_MULTIPLIER);
#endif
  dma3ChannelSetMode(adcp->dmachp,
                     dmaccr,
                     (adcp->config->dmactr1             |
                      STM32_DMA3_CTR1_DAP_MEM          |
                      STM32_DMA3_CTR1_DINC             |
                      STM32_DMA3_CTR1_SAP_PER          |
                      ADC_DMA3_CTR1_SIZE),
                     (adcp->config->dmactr2 |
                      STM32_DMA3_CTR2_REQSEL(adcp->dreq)),
                     dmallr);
  dma3ChannelEnable(adcp->dmachp);

  adcp->adcm->ISR = adcp->adcm->ISR;
  adcp->adcm->AWD1LTR = grpp->ltr1;
  adcp->adcm->AWD1HTR = grpp->htr1;
  adcp->adcm->AWD2LTR = grpp->ltr2;
  adcp->adcm->AWD2HTR = grpp->htr2;
  adcp->adcm->AWD3LTR = grpp->ltr3;
  adcp->adcm->AWD3HTR = grpp->htr3;
  adcp->adcm->AWD2CR  = grpp->awd2cr;
  adcp->adcm->AWD3CR  = grpp->awd3cr;

  if (grpp->error_cb != NULL) {
    adcp->adcm->IER = ADC_IER_OVRIE | ADC_IER_AWD1IE |
                      ADC_IER_AWD2IE | ADC_IER_AWD3IE;
  }
  else {
    adcp->adcm->IER = 0U;
  }

#if STM32_ADC_DUAL_MODE
  adcp->adcs->ISR = adcp->adcs->ISR;
  adcp->adcs->AWD1LTR = grpp->sltr1;
  adcp->adcs->AWD1HTR = grpp->shtr1;
  adcp->adcs->AWD2LTR = grpp->sltr2;
  adcp->adcs->AWD2HTR = grpp->shtr2;
  adcp->adcs->AWD3LTR = grpp->sltr3;
  adcp->adcs->AWD3HTR = grpp->shtr3;
  adcp->adcs->AWD2CR  = grpp->sawd2cr;
  adcp->adcs->AWD3CR  = grpp->sawd3cr;
  if (grpp->error_cb != NULL) {
    adcp->adcs->IER = ADC_IER_OVRIE | ADC_IER_AWD1IE |
                      ADC_IER_AWD2IE | ADC_IER_AWD3IE;
  }
  else {
    adcp->adcs->IER = 0U;
  }
#endif

#if STM32_ADC_DUAL_MODE
  ccr = adcp->adcc->CCR;
  ccr &= ~ADC_CCR_DAMDF_Msk;
  ccr |= (grpp->ccr & ~ADC_CCR_DAMDF_Msk) | ADC_CCR_DAMDF_MODE;
  adcp->adcc->CCR = ccr;
  adcp->adcm->PCSEL = grpp->pcsel;
  adcp->adcm->SMPR1 = grpp->smpr[0];
  adcp->adcm->SMPR2 = grpp->smpr[1];
  adcp->adcm->SQR1  = grpp->sqr[0] |
                      ADC_SQR1_NUM_CH(grpp->num_channels / 2U);
  adcp->adcm->SQR2  = grpp->sqr[1];
  adcp->adcm->SQR3  = grpp->sqr[2];
  adcp->adcm->SQR4  = grpp->sqr[3];
  adcp->adcs->PCSEL = grpp->pcsel;
  adcp->adcs->SMPR1 = grpp->ssmpr[0];
  adcp->adcs->SMPR2 = grpp->ssmpr[1];
  adcp->adcs->SQR1  = grpp->ssqr[0] |
                      ADC_SQR1_NUM_CH(grpp->num_channels / 2U);
  adcp->adcs->SQR2  = grpp->ssqr[1];
  adcp->adcs->SQR3  = grpp->ssqr[2];
  adcp->adcs->SQR4  = grpp->ssqr[3];
  adcp->adcs->CFGR1 = (grpp->cfgr & ~ADC_CFGR1_DMNGT_MASK) |
                      ADC_CFGR1_DMNGT_DR_ONLY;
  adcp->adcs->CFGR2 = grpp->cfgr2;
#else
  ccr = adcp->adcc->CCR;
  ccr &= ~ADC_CCR_DAMDF_Msk;
  ccr |= (grpp->ccr & ~ADC_CCR_DAMDF_Msk) | ADC_CCR_DAMDF_DISABLED;
  adcp->adcc->CCR = ccr;
  adcp->adcm->PCSEL = grpp->pcsel;
  adcp->adcm->SMPR1 = grpp->smpr[0];
  adcp->adcm->SMPR2 = grpp->smpr[1];
  adcp->adcm->SQR1  = grpp->sqr[0] |
                      ADC_SQR1_NUM_CH(grpp->num_channels);
  adcp->adcm->SQR2  = grpp->sqr[1];
  adcp->adcm->SQR3  = grpp->sqr[2];
  adcp->adcm->SQR4  = grpp->sqr[3];
#endif

  adcp->adcm->CFGR1 = cfgr1;
  adcp->adcm->CFGR2 = grpp->cfgr2;
  adcp->adcm->CR   |= ADC_CR_ADSTART;
}

/**
 * @brief   Stops an ongoing conversion.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_lld_stop_conversion(ADCDriver *adcp) {

  dma3ChannelDisable(adcp->dmachp);
  adc_lld_stop_adc(adcp);
}

/**
 * @brief   Enables the VREFEN bit.
 * @details The VREFEN bit is required in order to sample the VREF channel.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 */
void adcSTM32EnableVREF(ADCDriver *adcp) {

  adcp->adcc->CCR |= ADC_CCR_VREFEN;
}

/**
 * @brief   Disables the VREFEN bit.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 */
void adcSTM32DisableVREF(ADCDriver *adcp) {

  adcp->adcc->CCR &= ~ADC_CCR_VREFEN;
}

/**
 * @brief   Enables the TSEN bit.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 */
void adcSTM32EnableTS(ADCDriver *adcp) {

  adcp->adcc->CCR |= ADC_CCR_TSEN;
}

/**
 * @brief   Disables the TSEN bit.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 */
void adcSTM32DisableTS(ADCDriver *adcp) {

  adcp->adcc->CCR &= ~ADC_CCR_TSEN;
}

/**
 * @brief   Enables the VBATEN bit.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 */
void adcSTM32EnableVBAT(ADCDriver *adcp) {

  adcp->adcc->CCR |= ADC_CCR_VBATEN;
}

/**
 * @brief   Disables the VBATEN bit.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 */
void adcSTM32DisableVBAT(ADCDriver *adcp) {

  adcp->adcc->CCR &= ~ADC_CCR_VBATEN;
}

#endif /* HAL_USE_ADC */

/** @} */
