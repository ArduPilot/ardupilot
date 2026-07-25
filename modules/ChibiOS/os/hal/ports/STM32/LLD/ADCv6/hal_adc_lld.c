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
 * @file    ADCv6/hal_adc_lld.c
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
#define ADC_DMA3_CTR1_SIZE     (STM32_DMA3_CTR1_DDW_HALF |                \
                                 STM32_DMA3_CTR1_SDW_HALF)
#define ADC_CCR_MDMA_MODE       ADC_CCR_MDMA_HWORD
#define ADC_SAMPLE_MULTIPLIER   2U

#else /* !STM32_ADC_COMPACT_SAMPLES */
/* Large type dual mode.*/
#define ADC_DMA3_CTR1_SIZE     (STM32_DMA3_CTR1_DDW_WORD |                \
                                 STM32_DMA3_CTR1_SDW_WORD)
#define ADC_CCR_MDMA_MODE       ADC_CCR_MDMA_WORD
#define ADC_SAMPLE_MULTIPLIER   4U
#endif /* !STM32_ADC_COMPACT_SAMPLES */

#else /* !STM32_ADC_DUAL_MODE */
#if STM32_ADC_COMPACT_SAMPLES
/* Compact type single mode.*/
#define ADC_DMA3_CTR1_SIZE     (STM32_DMA3_CTR1_DDW_BYTE |                \
                                 STM32_DMA3_CTR1_SDW_BYTE)
#define ADC_CCR_MDMA_MODE       ADC_CCR_MDMA_DISABLED
#define ADC_SAMPLE_MULTIPLIER   1U

#else /* !STM32_ADC_COMPACT_SAMPLES */
/* Large type single mode.*/
#define ADC_DMA3_CTR1_SIZE     (STM32_DMA3_CTR1_DDW_HALF |                \
                                 STM32_DMA3_CTR1_SDW_HALF)
#define ADC_CCR_MDMA_MODE       ADC_CCR_MDMA_DISABLED
#define ADC_SAMPLE_MULTIPLIER   2U
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

/** @brief ADC3 driver identifier.*/
#if STM32_ADC_USE_ADC3 || defined(__DOXYGEN__)
ADCDriver ADCD3;
#endif

/** @brief ADC4 driver identifier.*/
#if STM32_ADC_USE_ADC4 || defined(__DOXYGEN__)
ADCDriver ADCD4;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

static const ADCConfig default_config = {
  .difsel = 0
};

static uint32_t clkmask;

#if STM32_ADC_USE_ADC1 || defined(__DOXYGEN__)
static adc_dmabuf_t __dma3_adc1;
#endif

#if STM32_ADC_USE_ADC2 || defined(__DOXYGEN__)
static adc_dmabuf_t __dma3_adc2;
#endif

#if STM32_ADC_USE_ADC3 || defined(__DOXYGEN__)
static adc_dmabuf_t __dma3_adc3;
#endif

#if STM32_ADC_USE_ADC4 || defined(__DOXYGEN__)
static adc_dmabuf_t __dma3_adc4;
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

  adcp->adcm->CR = 0;   /* See RM.*/
  adcp->adcm->CR = ADC_CR_ADVREGEN;
#if STM32_ADC_DUAL_MODE
  adcp->adcs->CR = ADC_CR_ADVREGEN;
#endif
  osalSysPolledDelayX(OSAL_US2RTC(STM32_HCLK, 20));
}

/**
 * @brief   Disables the ADC voltage regulator.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 */
static void adc_lld_vreg_off(ADCDriver *adcp) {

  adcp->adcm->CR = 0;   /* See RM.*/
  adcp->adcm->CR =ADC_CR_DEEPPWD;
#if STM32_ADC_DUAL_MODE
  adcp->adcs->CR = 0;
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

  /* Differential calibration for master ADC.*/
  adcp->adcm->CR = ADC_CR_ADVREGEN | ADC_CR_ADCALDIF;
  adcp->adcm->CR = ADC_CR_ADVREGEN | ADC_CR_ADCALDIF | ADC_CR_ADCAL;
  while ((adcp->adcm->CR & ADC_CR_ADCAL) != 0)
    ;

  osalSysPolledDelayX(OSAL_US2RTC(STM32_HCLK, 20));

  /* Single-ended calibration for master ADC.*/
  adcp->adcm->CR = ADC_CR_ADVREGEN;
  adcp->adcm->CR = ADC_CR_ADVREGEN | ADC_CR_ADCAL;
  while ((adcp->adcm->CR & ADC_CR_ADCAL) != 0)
    ;

  osalSysPolledDelayX(OSAL_US2RTC(STM32_HCLK, 20));

#if STM32_ADC_DUAL_MODE
  osalDbgAssert(adcp->adcs->CR == ADC_CR_ADVREGEN, "invalid register state");

  /* Differential calibration for slave ADC.*/
  adcp->adcs->CR = ADC_CR_ADVREGEN | ADC_CR_ADCALDIF;
  adcp->adcs->CR = ADC_CR_ADVREGEN | ADC_CR_ADCALDIF | ADC_CR_ADCAL;
  while ((adcp->adcs->CR & ADC_CR_ADCAL) != 0)
    ;

  osalSysPolledDelayX(OSAL_US2RTC(STM32_HCLK, 20));

  /* Single-ended calibration for slave ADC.*/
  adcp->adcs->CR = ADC_CR_ADVREGEN;
  adcp->adcs->CR = ADC_CR_ADVREGEN | ADC_CR_ADCAL;
  while ((adcp->adcs->CR & ADC_CR_ADCAL) != 0)
    ;

  osalSysPolledDelayX(OSAL_US2RTC(STM32_HCLK, 20));
#endif
}

/**
 * @brief   Enables the ADC analog circuit.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 */
static void adc_lld_analog_on(ADCDriver *adcp) {

  adcp->adcm->CR |= ADC_CR_ADEN;
  while ((adcp->adcm->ISR & ADC_ISR_ADRDY) == 0)
    ;
#if STM32_ADC_DUAL_MODE
  adcp->adcs->CR |= ADC_CR_ADEN;
  while ((adcp->adcs->ISR & ADC_ISR_ADRDY) == 0)
    ;
#endif
}

/**
 * @brief   Disables the ADC  analog circuit.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 */
static void adc_lld_analog_off(ADCDriver *adcp) {

  adcp->adcm->CR |= ADC_CR_ADDIS;
  while ((adcp->adcm->CR & ADC_CR_ADDIS) != 0)
    ;
#if STM32_ADC_DUAL_MODE
  adcp->adcs->CR |= ADC_CR_ADDIS;
  while ((adcp->adcs->CR & ADC_CR_ADDIS) != 0)
    ;
#endif
}

/**
 * @brief   Stops an ongoing conversion, if any.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 */
static void adc_lld_stop_adc(ADCDriver *adcp) {

  if (adcp->adcm->CR & ADC_CR_ADSTART) {
    adcp->adcm->CR |= ADC_CR_ADSTP;
    while (adcp->adcm->CR & ADC_CR_ADSTP)
      ;
    adcp->adcm->IER = 0;
  }
}

/**
 * @brief   ADC GPDMA service routine.
 *
 * @param[in] p         parameter for the registered function
 * @param[in] csr       content of the CxSR register
 */
static void adc_lld_serve_dma_interrupt(void *p, uint32_t csr) {
  ADCDriver *adcp = (ADCDriver *)p;

  /* DMA errors handling.*/
  if ((csr & STM32_DMA3_CSR_ERRORS) != 0U) {
    /* DMA, this could help only if the DMA tries to access an unmapped
       address space or violates alignment rules.*/
    _adc_isr_error_code(adcp, ADC_ERR_DMAFAILURE);
  }
  else {
    /* It is possible that the conversion group has already be reset by the
       ADC error handler, in this case this interrupt is spurious.*/
    if (adcp->grpp != NULL) {
      if ((csr & STM32_DMA3_CSR_TCF) != 0U) {
        /* Transfer complete processing.*/
        _adc_isr_full_code(adcp);
      }
      else if ((csr & STM32_DMA3_CSR_HTF) != 0U) {
        /* Half transfer processing.*/
        _adc_isr_half_code(adcp);
      }
    }
  }
}

/**
 * @brief   ADC IRQ service routine.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 * @param[in] isr       content of the ISR register
 */
static void adc_lld_serve_interrupt(ADCDriver *adcp, uint32_t isr) {

  /* It could be a spurious interrupt caused by overflows after DMA disabling,
     just ignore it in this case.*/
  if (adcp->grpp != NULL) {
    adcerror_t emask = 0U;

    /* Note, an overflow may occur after the conversion ended before the driver
       is able to stop the ADC, this is why the state is checked too.*/
    if ((isr & ADC_ISR_OVR) && (adcp->state == ADC_ACTIVE)) {
      /* ADC overflow condition, this could happen only if the DMA is unable
         to read data fast enough.*/
      emask |= ADC_ERR_OVERFLOW;
    }
    if (isr & ADC_ISR_AWD1) {
      /* Analog watchdog 1 error.*/
      emask |= ADC_ERR_AWD1;
    }
    if (isr & ADC_ISR_AWD2) {
      /* Analog watchdog 2 error.*/
      emask |= ADC_ERR_AWD2;
    }
    if (isr & ADC_ISR_AWD3) {
      /* Analog watchdog 3 error.*/
      emask |= ADC_ERR_AWD3;
    }
    if (emask != 0U) {
      _adc_isr_error_code(adcp, emask);
    }
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
 * @brief   ADC2 interrupt handler (as ADC1 slave).
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

#if STM32_ADC_USE_ADC2 || defined(__DOXYGEN__)
/**
 * @brief   ADC2 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_ADC2_HANDLER) {
  uint32_t isr;

  OSAL_IRQ_PROLOGUE();

  isr  = ADC2->ISR;
  ADC2->ISR = isr;

  adc_lld_serve_interrupt(&ADCD2, isr);

  OSAL_IRQ_EPILOGUE();
}
#endif /* STM32_ADC_USE_ADC4 */

#if STM32_ADC_USE_ADC3 || defined(__DOXYGEN__)
/**
 * @brief   ADC3 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_ADC3_HANDLER) {
  uint32_t isr;

  OSAL_IRQ_PROLOGUE();

  isr  = ADC3->ISR;
  ADC3->ISR = isr;
#if defined(STM32_ADC_ADC3_IRQ_HOOK)
  STM32_ADC_ADC3_IRQ_HOOK
#endif
  adc_lld_serve_interrupt(&ADCD3, isr);

  OSAL_IRQ_EPILOGUE();
}

#if STM32_ADC_DUAL_MODE
/**
 * @brief   ADC4 interrupt handler (as ADC3 slave).
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_ADC4_HANDLER) {
  uint32_t isr;

  OSAL_IRQ_PROLOGUE();

  isr  = ADC4->ISR;
  ADC4->ISR = isr;
#if defined(STM32_ADC_ADC4_IRQ_HOOK)
  STM32_ADC_ADC4_IRQ_HOOK
#endif

  adc_lld_serve_interrupt(&ADCD3, isr);

  OSAL_IRQ_EPILOGUE();
}
#endif /* STM32_ADC_DUAL_MODE */
#endif /* STM32_ADC_USE_ADC3 */

#if STM32_ADC_USE_ADC4 || defined(__DOXYGEN__)
/**
 * @brief   ADC4 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_ADC4_HANDLER) {
  uint32_t isr;

  OSAL_IRQ_PROLOGUE();

  isr  = ADC4->ISR;
  ADC4->ISR = isr;
#if defined(STM32_ADC_ADC4_IRQ_HOOK)
  STM32_ADC_ADC4_IRQ_HOOK
#endif

  adc_lld_serve_interrupt(&ADCD4, isr);

  OSAL_IRQ_EPILOGUE();
}
#endif /* STM32_ADC_USE_ADC4 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level ADC driver initialization.
 *
 * @notapi
 */
void adc_lld_init(void) {

  clkmask = 0;

#if STM32_ADC_USE_ADC1
  /* Driver initialization.*/
  adcObjectInit(&ADCD1);
  ADCD1.adcc    = ADC12_COMMON;
  ADCD1.adcm    = ADC1;
#if STM32_ADC_DUAL_MODE
  ADCD1.adcs    = ADC2;
#endif
  ADCD1.dmachp  = NULL;
  ADCD1.dprio   = STM32_ADC_ADC1_DMA_PRIORITY;
  ADCD1.dreq    = STM32_DMA3_REQ_ADC1;
  ADCD1.dbuf    = &__dma3_adc1;
#endif /* STM32_ADC_USE_ADC1 */

#if STM32_ADC_USE_ADC2
  /* Driver initialization.*/
  adcObjectInit(&ADCD2);
  ADCD2.adcc    = ADC12_COMMON;
  ADCD2.adcm    = ADC2;
  ADCD2.dmachp  = NULL;
  ADCD2.dprio   = STM32_ADC_ADC2_DMA_PRIORITY;
  ADCD2.dreq    = STM32_DMA3_REQ_ADC2;
  ADCD2.dbuf    = &__dma3_adc2;
#endif /* STM32_ADC_USE_ADC2 */

#if STM32_ADC_USE_ADC3
  /* Driver initialization.*/
  adcObjectInit(&ADCD3);
  ADCD3.adcc    = ADC34_COMMON;
  ADCD3.adcm    = ADC3;
#if STM32_ADC_DUAL_MODE
  ADCD3.adcs    = ADC4;
#endif
  ADCD3.dmachp  = NULL;
  ADCD3.dprio   = STM32_ADC_ADC3_DMA_PRIORITY;
  ADCD3.dreq    = STM32_DMA3_REQ_ADC3;
  ADCD3.dbuf    = &__dma3_adc3;
#endif /* STM32_ADC_USE_ADC3 */

#if STM32_ADC_USE_ADC4
  /* Driver initialization.*/
  adcObjectInit(&ADCD4);
  ADCD4.adcc    = ADC34_COMMON;
  ADCD4.adcm    = ADC4;
  ADCD4.dmachp  = NULL;
  ADCD4.dprio   = STM32_ADC_ADC4_DMA_PRIORITY;
  ADCD4.dreq    = STM32_DMA3_REQ_ADC4;
  ADCD4.dbuf    = &__dma3_adc4;
#endif /* STM32_ADC_USE_ADC4 */

  /* IRQs setup.*/
#if STM32_ADC_USE_ADC1
  nvicEnableVector(STM32_ADC1_NUMBER, STM32_ADC_ADC1_IRQ_PRIORITY);
#if STM32_ADC_DUAL_MODE
  nvicEnableVector(STM32_ADC2_NUMBER, STM32_ADC_ADC1_IRQ_PRIORITY);
#endif
#endif /* STM32_ADC_USE_ADC1 */

#if STM32_ADC_USE_ADC2
  nvicEnableVector(STM32_ADC2_NUMBER, STM32_ADC_ADC2_IRQ_PRIORITY);
#endif /* STM32_ADC_USE_ADC2 */

#if STM32_ADC_USE_ADC3
  nvicEnableVector(STM32_ADC3_NUMBER, STM32_ADC_ADC3_IRQ_PRIORITY);
#if STM32_ADC_DUAL_MODE
  nvicEnableVector(STM32_ADC4_NUMBER, STM32_ADC_ADC3_IRQ_PRIORITY);
#endif
#endif /* STM32_ADC_USE_ADC3 */

#if STM32_ADC_USE_ADC4
  nvicEnableVector(STM32_ADC4_NUMBER, STM32_ADC_ADC4_IRQ_PRIORITY);
#endif /* STM32_ADC_USE_ADC4 */

  /* ADC units pre-initializations.*/
#if defined(STM32H5XX)
#if STM32_ADC_USE_ADC1 || STM32_ADC_USE_ADC2
  rccResetADC12();
  rccEnableADC12(true);
  ADC12_COMMON->CCR = STM32_ADC_ADC12_PRESC | STM32_ADC_ADC12_CLOCK_MODE | ADC_CCR_MDMA_MODE;
  rccDisableADC12();
#endif
#endif
}

/**
 * @brief   Configures and activates the ADC peripheral.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_lld_start(ADCDriver *adcp) {

  /* Handling the default configuration.*/
  if (adcp->config == NULL) {
    adcp->config = &default_config;
  }

  /* If in stopped state then enables the ADC and DMA clocks.*/
  if (adcp->state == ADC_STOP) {
#if STM32_ADC_USE_ADC1
    if (&ADCD1 == adcp) {

      osalDbgAssert(STM32_ADC1_CLOCK <= STM32_ADCCLK_MAX,
                    "invalid clock frequency");

      adcp->dmachp = dma3ChannelAllocI(STM32_ADC_ADC1_DMA3_CHANNEL,
                                        STM32_ADC_ADC1_DMA_IRQ_PRIORITY,
                                        adc_lld_serve_dma_interrupt,
                                        (void *)adcp);
      osalDbgAssert(adcp->dmachp != NULL, "unable to allocate stream");

      clkmask |= (1U << 0);
#if defined(STM32H5XX)
      rccEnableADC12(true);
#endif
    }
#endif /* STM32_ADC_USE_ADC1 */

#if STM32_ADC_USE_ADC2
    if (&ADCD2 == adcp) {

      osalDbgAssert(STM32_ADC2_CLOCK <= STM32_ADCCLK_MAX,
                    "invalid clock frequency");

      adcp->dmachp = dma3ChannelAllocI(STM32_ADC_ADC2_DMA3_CHANNEL,
                                        STM32_ADC_ADC2_DMA_IRQ_PRIORITY,
                                        adc_lld_serve_dma_interrupt,
                                        (void *)adcp);
      osalDbgAssert(adcp->dmachp != NULL, "unable to allocate stream");

      clkmask |= (1U << 1);
#if defined(STM32H5XX)
      rccEnableADC12(true);
#endif
    }
#endif /* STM32_ADC_USE_ADC2 */

#if STM32_ADC_USE_ADC3
    if (&ADCD3 == adcp) {

      osalDbgAssert(STM32_ADC3_CLOCK <= STM32_ADCCLK_MAX,
                    "invalid clock frequency");

      adcp->dmachp = dma3ChannelAllocI(STM32_ADC_ADC3_DMA3_CHANNEL,
                                        STM32_ADC_ADC3_DMA_IRQ_PRIORITY,
                                        adc_lld_serve_dma_interrupt,
                                        (void *)adcp);
      osalDbgAssert(adcp->dmachp != NULL, "unable to allocate stream");

      clkmask |= (1U << 2);
#if defined(STM32H5XX)
      rccEnableADC34(true);
#endif
    }
#endif /* STM32_ADC_USE_ADC3 */

#if STM32_ADC_USE_ADC4
    if (&ADCD4 == adcp) {

      osalDbgAssert(STM32_ADC4_CLOCK <= STM32_ADCCLK_MAX,
                    "invalid clock frequency");

      adcp->dmachp = dma3ChannelAllocI(STM32_ADC_ADC4_DMA3_CHANNEL,
                                        STM32_ADC_ADC4_DMA_IRQ_PRIORITY,
                                        adc_lld_serve_dma_interrupt,
                                        (void *)adcp);
      osalDbgAssert(adcp->dmachp != NULL, "unable to allocate stream");

      clkmask |= (1U << 3);
#if defined(STM32H5XX)
      rccEnableADC34(true);
#endif
    }
#endif /* STM32_ADC_USE_ADC4 */

    /* Setting DMA peripheral-side pointer.*/
#if STM32_ADC_DUAL_MODE
//    dma3ChannelSetSource(adcp->dmachp, &adcp->adcc->CDR);
#else
//    dma3ChannelSetSource(adcp->dmachp, &adcp->adcm->DR);
#endif

    /* Differential channels setting.*/
#if STM32_ADC_DUAL_MODE
    adcp->adcm->DIFSEL = adcp->config->difsel;
    adcp->adcs->DIFSEL = adcp->config->difsel;
#else
    adcp->adcm->DIFSEL = adcp->config->difsel;
#endif

    /* Master ADC calibration.*/
    adc_lld_vreg_on(adcp);
    adc_lld_calibrate(adcp);

    /* Master ADC enabled here in order to reduce conversions latencies.*/
    adc_lld_analog_on(adcp);
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

  /* If in ready state then disables the ADC clock and analog part.*/
  if (adcp->state == ADC_READY) {

    /* Releasing the associated DMA channel.*/
    dma3ChannelFreeI(adcp->dmachp);
    adcp->dmachp = NULL;

    /* Stopping the ongoing conversion, if any.*/
    adc_lld_stop_adc(adcp);

    /* Disabling ADC analog circuit and regulator.*/
    adc_lld_analog_off(adcp);
    adc_lld_vreg_off(adcp);

#if STM32_ADC_USE_ADC1
    if (&ADCD1 == adcp) {
      /* Resetting CCR options except default ones.*/
      clkmask &= ~(1U << 0);
    }
#endif

#if STM32_ADC_USE_ADC2
    if (&ADCD2 == adcp) {
      clkmask &= ~(1U << 1);
    }
#endif

#if STM32_ADC_USE_ADC3
    if (&ADCD3 == adcp) {
      clkmask &= ~(1U << 2);
    }
#endif

#if STM32_ADC_USE_ADC4
    if (&ADCD4 == adcp) {
      clkmask &= ~(1U << 3);
    }
#endif

#if defined(STM32H5XX)
#if STM32_HAS_ADC1 || STM32_HAS_ADC2
    if ((clkmask & 0x3) == 0) {
      rccDisableADC12();
    }
#endif

#if STM32_HAS_ADC3 || STM32_HAS_ADC4
    if ((clkmask & 0xC) == 0) {
      rccDisableADC34();
    }
#endif
#endif
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
//  uint32_t dmamode, cfgr;
  uint32_t cfgr, dmaccr, dmallr;
  const ADCConversionGroup *grpp = adcp->grpp;
#if STM32_ADC_DUAL_MODE
  uint32_t ccr = grpp->ccr & ~(ADC_CCR_CKMODE_MASK | ADC_CCR_MDMA_MASK);
#endif

  osalDbgAssert(!STM32_ADC_DUAL_MODE || ((grpp->num_channels & 1) == 0),
                "odd number of channels in dual mode");

#if 0
  /* Calculating control registers values.*/
  dmamode = adcp->dmamode;
  cfgr    = grpp->cfgr | ADC_CFGR_DMAEN;
  if (grpp->circular) {
    dmamode |= STM32_DMA_CR_CIRC;
#if STM32_ADC_DUAL_MODE
    ccr  |= ADC_CCR_DMACFG_CIRCULAR;
#else
    cfgr |= ADC_CFGR_DMACFG_CIRCULAR;
#endif
    if (adcp->depth > 1) {
      /* If circular buffer depth > 1, then the half transfer interrupt
         is enabled in order to allow streaming processing.*/
      dmamode |= STM32_DMA_CR_HTIE;
    }
  }
#endif

  /* Calculating control registers values.*/
  dmaccr = STM32_DMA3_CCR_PRIO((uint32_t)adcp->dprio)   |
           STM32_DMA3_CCR_LAP_MEM                       |
           STM32_DMA3_CCR_TOIE                          |
           STM32_DMA3_CCR_USEIE                         |
           STM32_DMA3_CCR_ULEIE                         |
           STM32_DMA3_CCR_DTEIE                         |
           STM32_DMA3_CCR_TCIE;
  cfgr    = grpp->cfgr | ADC_CFGR_DMAEN;
  if (grpp->circular) {
#if STM32_ADC_DUAL_MODE
    ccr  |= ADC_CCR_DMACFG_CIRCULAR;
#else
    cfgr |= ADC_CFGR_DMACFG_CIRCULAR;
#endif
    /* It is a circular operation, using the linking mechanism to reload
       source/destination pointers.*/
    dmallr = STM32_DMA3_CLLR_UDA | (((uint32_t)&adcp->dbuf->cdar) & 0xFFFFU);
    adcp->dbuf->cdar = (uint32_t)adcp->samples;

    if (adcp->depth > 1U) {
      /* If circular buffer depth > 1, then the half transfer interrupt
         is enabled in order to allow streaming processing.*/
      dmaccr |= STM32_DMA3_CCR_HTIE;
    }
  }
  else {
    dmallr = 0U;
  }

  /* DMA setup.*/
  dma3ChannelSetDestination(adcp->dmachp, adcp->samples);
#if STM32_ADC_DUAL_MODE
  dma3ChannelSetSource(adcp->dmachp, &adcp->adcc->CDR);
  dma3ChannelSetTransactionSize(adcp->dmachp, (((uint32_t)grpp->num_channels / 2) *
                                                (uint32_t)adcp->depth) * ADC_SAMPLE_MULTIPLIER);
#else
  dma3ChannelSetSource(adcp->dmachp, &adcp->adcm->DR);
  dma3ChannelSetTransactionSize(adcp->dmachp, ((uint32_t)grpp->num_channels *
                                                (uint32_t)adcp->depth) * ADC_SAMPLE_MULTIPLIER);
#endif
  dma3ChannelSetMode(adcp->dmachp,
                      dmaccr,
                      (adcp->config->dmactr1                        |
                       STM32_DMA3_CTR1_DAP_MEM                      |
                       STM32_DMA3_CTR1_DINC                         |
                       STM32_DMA3_CTR1_SAP_PER                      |
                       ADC_DMA3_CTR1_SIZE),
                      (adcp->config->dmactr2                        |
                       STM32_DMA3_CTR2_REQSEL(adcp->dreq)),
                       dmallr);
  dma3ChannelEnable(adcp->dmachp);

  /* ADC setup, if it is defined a callback for the analog watch dog then it
     is enabled.*/
  adcp->adcm->ISR   = adcp->adcm->ISR;
  if (grpp->error_cb != NULL) {
    adcp->adcm->IER    = ADC_IER_OVRIE | ADC_IER_AWD1IE
                                       | ADC_IER_AWD2IE
                                       | ADC_IER_AWD3IE;
    adcp->adcm->TR1    = grpp->tr1;
    adcp->adcm->TR2    = grpp->tr2;
    adcp->adcm->TR3    = grpp->tr3;
    adcp->adcm->AWD2CR = grpp->awd2cr;
    adcp->adcm->AWD3CR = grpp->awd3cr;
  }

#if STM32_ADC_DUAL_MODE

  /* Configuring the CCR register with the user-specified settings
     in the conversion group configuration structure, static settings are
     preserved.*/
  adcp->adcc->CCR   = (adcp->adcc->CCR &
                       (ADC_CCR_CKMODE_MASK | ADC_CCR_MDMA_MASK)) | ccr;

  adcp->adcm->SMPR1 = grpp->smpr[0];
  adcp->adcm->SMPR2 = grpp->smpr[1];
  adcp->adcm->SQR1  = grpp->sqr[0] | ADC_SQR1_NUM_CH(grpp->num_channels / 2);
  adcp->adcm->SQR2  = grpp->sqr[1];
  adcp->adcm->SQR3  = grpp->sqr[2];
  adcp->adcm->SQR4  = grpp->sqr[3];
  adcp->adcs->SMPR1 = grpp->ssmpr[0];
  adcp->adcs->SMPR2 = grpp->ssmpr[1];
  adcp->adcs->SQR1  = grpp->ssqr[0] | ADC_SQR1_NUM_CH(grpp->num_channels / 2);
  adcp->adcs->SQR2  = grpp->ssqr[1];
  adcp->adcs->SQR3  = grpp->ssqr[2];
  adcp->adcs->SQR4  = grpp->ssqr[3];

#else /* !STM32_ADC_DUAL_MODE */
  adcp->adcm->SMPR1 = grpp->smpr[0];
  adcp->adcm->SMPR2 = grpp->smpr[1];
  adcp->adcm->SQR1  = grpp->sqr[0] | ADC_SQR1_NUM_CH(grpp->num_channels);
  adcp->adcm->SQR2  = grpp->sqr[1];
  adcp->adcm->SQR3  = grpp->sqr[2];
  adcp->adcm->SQR4  = grpp->sqr[3];
#endif /* !STM32_ADC_DUAL_MODE */

  /* ADC configuration.*/
  adcp->adcm->CFGR  = cfgr;
  adcp->adcm->CFGR2 = grpp->cfgr2;

  /* Starting conversion.*/
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
 * @note    This is an STM32-only functionality.
 * @note    This function is meant to be called after @p adcStart().
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adcSTM32EnableVREF(ADCDriver *adcp) {

  adcp->adcc->CCR |= ADC_CCR_VREFEN;
}

/**
 * @brief   Disables the VREFEN bit.
 * @details The VREFEN bit is required in order to sample the VREF channel.
 * @note    This is an STM32-only functionality.
 * @note    This function is meant to be called after @p adcStart().
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adcSTM32DisableVREF(ADCDriver *adcp) {

  adcp->adcc->CCR &= ~ADC_CCR_VREFEN;
}

/**
 * @brief   Enables the TSEN bit.
 * @details The TSEN bit is required in order to sample the internal
 *          temperature sensor and internal reference voltage.
 * @note    This is an STM32-only functionality.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adcSTM32EnableTS(ADCDriver *adcp) {

  adcp->adcc->CCR |= ADC_CCR_TSEN;
}

/**
 * @brief   Disables the TSEN bit.
 * @details The TSEN bit is required in order to sample the internal
 *          temperature sensor and internal reference voltage.
 * @note    This is an STM32-only functionality.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adcSTM32DisableTS(ADCDriver *adcp) {

  adcp->adcc->CCR &= ~ADC_CCR_TSEN;
}

/**
 * @brief   Enables the VBATEN bit.
 * @details The VBATEN bit is required in order to sample the VBAT channel.
 * @note    This is an STM32-only functionality.
 * @note    This function is meant to be called after @p adcStart().
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adcSTM32EnableVBAT(ADCDriver *adcp) {

  adcp->adcc->CCR |= ADC_CCR_VBATEN;
}

/**
 * @brief   Disables the VBATEN bit.
 * @details The VBATEN bit is required in order to sample the VBAT channel.
 * @note    This is an STM32-only functionality.
 * @note    This function is meant to be called after @p adcStart().
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adcSTM32DisableVBAT(ADCDriver *adcp) {

  adcp->adcc->CCR &= ~ADC_CCR_VBATEN;
}

/**
 * @brief   Enables the OP0 bit.
 * @details The OP0 bit is required in order to sample the VCORE channel.
 * @note    This is an STM32-only functionality.
 * @note    This function is meant to be called after @p adcStart().
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adcSTM32EnableVCORE(ADCDriver *adcp) {

  adcp->adcm->OR |= ADC_OR_OP0;
}

/**
 * @brief   Disables the OP0 bit.
 * @details The OP0 bit is required in order to sample the VCORE channel.
 * @note    This is an STM32-only functionality.
 * @note    This function is meant to be called after @p adcStart().
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adcSTM32DisableVCORE(ADCDriver *adcp) {

  adcp->adcm->OR &= ~ADC_OR_OP0;
}

#endif /* HAL_USE_ADC */

/** @} */
