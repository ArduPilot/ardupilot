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
 * @file    STM32F37x/hal_adc_lld.c
 * @brief   STM32F37x ADC subsystem low level driver source.
 *
 * @addtogroup ADC
 * @{
 */

#include "hal.h"

#if HAL_USE_ADC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define SDADC_FORBIDDEN_CR1_FLAGS   (SDADC_CR1_INIT   | SDADC_CR1_RDMAEN |  \
                                     SDADC_CR1_RSYNC  | SDADC_CR1_JSYNC  |  \
                                     SDADC_CR1_ROVRIE | SDADC_CR1_REOCIE |  \
                                     SDADC_CR1_JEOCIE | SDADC_CR1_EOCALIE)

#define SDADC_ENFORCED_CR1_FLAGS    (SDADC_CR1_JDMAEN  | SDADC_CR1_JOVRIE)

#define SDADC_FORBIDDEN_CR2_FLAGS   (SDADC_CR2_RSWSTART   |                 \
                                     SDADC_CR2_RCONT      |                 \
                                     SDADC_CR2_RCH        |                 \
                                     SDADC_CR2_JCONT      |                 \
                                     SDADC_CR2_STARTCALIB |                 \
                                     SDADC_CR2_CALIBCNT)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief ADC1 driver identifier.*/
#if STM32_ADC_USE_ADC1 || defined(__DOXYGEN__)
ADCDriver ADCD1;
#endif

/** @brief SDADC1 driver identifier.*/
#if STM32_ADC_USE_SDADC1 || defined(__DOXYGEN__)
ADCDriver SDADCD1;
#endif

/** @brief SDADC2 driver identifier.*/
#if STM32_ADC_USE_SDADC2 || defined(__DOXYGEN__)
ADCDriver SDADCD2;
#endif

/** @brief SDADC3 driver identifier.*/
#if STM32_ADC_USE_SDADC3 || defined(__DOXYGEN__)
ADCDriver SDADCD3;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

static const ADCConfig adc_lld_default_config = {
#if STM32_ADC_USE_SDADC
  0,
  {
     0,
     0,
     0
  }
#else /* !STM32_ADC_USE_SDADC */
  0
#endif /* !STM32_ADC_USE_SDADC */
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Stops, reconfigures and restarts an ADC/SDADC.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 */
static void adc_lld_reconfig(ADCDriver *adcp) {

#if STM32_ADC_USE_ADC && STM32_ADC_USE_SDADC
  if (adcp->adc != NULL)
#endif /* STM32_ADC_USE_ADC && STM32_ADC_USE_SDADC */
#if STM32_ADC_USE_ADC
  {
    /* ADC initial setup, starting the analog part here in order to reduce
       the latency when starting a conversion.*/
    uint32_t cr2 = adcp->adc->CR2 & ADC_CR2_TSVREFE;
    adcp->adc->CR2 = cr2;
    adcp->adc->CR1 = 0;
    adcp->adc->CR2 = cr2 | ADC_CR2_ADON;

  }
#endif /* STM32_ADC_USE_ADC */
#if STM32_ADC_USE_ADC && STM32_ADC_USE_SDADC
  else if (adcp->sdadc != NULL)
#endif /* STM32_ADC_USE_ADC && STM32_ADC_USE_SDADC */
#if STM32_ADC_USE_SDADC
  {
    /* SDADC initial setup, starting the analog part here in order to reduce
       the latency when starting a conversion.*/
    adcp->sdadc->CR2    = 0;
    adcp->sdadc->CR1    = (adcp->config->cr1 | SDADC_ENFORCED_CR1_FLAGS) &
                          ~SDADC_FORBIDDEN_CR1_FLAGS;
    adcp->sdadc->CONF0R = (adcp->sdadc->CONF0R & SDADC_CONFR_OFFSET_MASK) |
                          adcp->config->confxr[0];
    adcp->sdadc->CONF1R = (adcp->sdadc->CONF1R & SDADC_CONFR_OFFSET_MASK) |
                          adcp->config->confxr[1];
    adcp->sdadc->CONF2R = (adcp->sdadc->CONF2R & SDADC_CONFR_OFFSET_MASK) |
                          adcp->config->confxr[2];
    adcp->sdadc->CR2    = SDADC_CR2_ADON;
  }
#endif /* STM32_ADC_USE_SDADC */
#if STM32_ADC_USE_ADC && STM32_ADC_USE_SDADC
else {
    osalDbgAssert(FALSE, "invalid state");
  }
#endif /* STM32_ADC_USE_ADC && STM32_ADC_USE_SDADC */
}

/**
 * @brief   ADC DMA ISR service routine.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 * @param[in] flags     pre-shifted content of the ISR register
 *
 * @notapi
 */
static void adc_lld_serve_dma_interrupt(ADCDriver *adcp, uint32_t flags) {

  /* DMA errors handling.*/
  if ((flags & (STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF)) != 0) {
    /* DMA, this could help only if the DMA tries to access an unmapped
       address space or violates alignment rules.*/
    _adc_isr_error_code(adcp, ADC_ERR_DMAFAILURE);
  }
  else {
    /* It is possible that the conversion group has already be reset by the
       ADC error handler, in this case this interrupt is spurious.*/
    if (adcp->grpp != NULL) {
      if ((flags & STM32_DMA_ISR_TCIF) != 0) {
        /* Transfer complete processing.*/
        _adc_isr_full_code(adcp);
      }
      else if ((flags & STM32_DMA_ISR_HTIF) != 0) {
        /* Half transfer processing.*/
        _adc_isr_half_code(adcp);
      }
    }
  }
}

#if STM32_ADC_USE_ADC || defined(__DOXYGEN__)
/**
 * @brief   ADC ISR service routine.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 * @param[in] sr        content of the ISR register
 *
 * @notapi
 */
static void adc_lld_serve_interrupt(ADCDriver *adcp, uint32_t sr) {

  /* It could be a spurious interrupt caused by overflows after DMA disabling,
     just ignore it in this case.*/
  if (adcp->grpp != NULL) {
    if (sr & ADC_SR_AWD) {
      /* Analog watchdog error.*/
      _adc_isr_error_code(adcp, ADC_ERR_AWD1);
    }
  }
}
#endif /* STM32_ADC_USE_ADC */

#if STM32_ADC_USE_SDADC || defined(__DOXYGEN__)
/**
 * @brief   ADC ISR service routine.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 * @param[in] isr       content of the ISR register
 *
 * @notapi
 */
static void sdadc_lld_serve_interrupt(ADCDriver *adcp, uint32_t isr) {

  /* It could be a spurious interrupt caused by overflows after DMA disabling,
     just ignore it in this case.*/
  if (adcp->grpp != NULL) {
    /* Note, an overflow may occur after the conversion ended before the driver
       is able to stop the ADC, this is why the DMA channel is checked too.*/
    if ((isr & SDADC_ISR_JOVRF) &&
        (dmaStreamGetTransactionSize(adcp->dmastp) > 0)) {
      /* ADC overflow condition, this could happen only if the DMA is unable
         to read data fast enough.*/
      _adc_isr_error_code(adcp, ADC_ERR_OVERFLOW);
    }
  }
}
#endif /* STM32_ADC_USE_SDADC */

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if STM32_ADC_USE_ADC1 || defined(__DOXYGEN__)
/**
 * @brief   ADC1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(Vector88) {
  uint32_t sr;

  OSAL_IRQ_PROLOGUE();

  sr  = ADC1->SR;
  ADC1->SR = 0;
  adc_lld_serve_interrupt(&ADCD1, sr);

  OSAL_IRQ_EPILOGUE();
}
#endif /* STM32_ADC_USE_ADC1 */

#if STM32_ADC_USE_SDADC1 || defined(__DOXYGEN__)
/**
 * @brief   SDADC1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(Vector134) {
  uint32_t isr;

  OSAL_IRQ_PROLOGUE();

  isr  = SDADC1->ISR;
  SDADC1->CLRISR = isr;
  sdadc_lld_serve_interrupt(&SDADCD1, isr);

  OSAL_IRQ_EPILOGUE();
}
#endif /* STM32_ADC_USE_SDADC1 */

#if STM32_ADC_USE_SDADC2 || defined(__DOXYGEN__)
/**
 * @brief   SDADC2 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(Vector138) {
  uint32_t isr;

  OSAL_IRQ_PROLOGUE();

  isr  = SDADC2->ISR;
  SDADC2->CLRISR = isr;
  sdadc_lld_serve_interrupt(&SDADCD2, isr);

  OSAL_IRQ_EPILOGUE();
}
#endif /* STM32_ADC_USE_SDADC2 */

#if STM32_ADC_USE_SDADC3 || defined(__DOXYGEN__)
/**
 * @brief   SDADC3 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(Vector13C) {
  uint32_t isr;

  OSAL_IRQ_PROLOGUE();

  isr  = SDADC3->ISR;
  SDADC3->CLRISR = isr;
  sdadc_lld_serve_interrupt(&SDADCD3, isr);

  OSAL_IRQ_EPILOGUE();
}
#endif /* STM32_ADC_USE_SDADC3 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level ADC driver initialization.
 *
 * @notapi
 */
void adc_lld_init(void) {

#if STM32_ADC_USE_ADC1
  /* Driver initialization.*/
  adcObjectInit(&ADCD1);
  ADCD1.adc     = ADC1;
#if STM32_ADC_USE_SDADC
  ADCD1.sdadc   = NULL;
#endif
  ADCD1.dmastp  = NULL;
  ADCD1.dmamode = STM32_DMA_CR_CHSEL(ADC1_DMA_CHANNEL) |
                  STM32_DMA_CR_PL(STM32_ADC_ADC1_DMA_PRIORITY) |
                  STM32_DMA_CR_DIR_P2M |
                  STM32_DMA_CR_MSIZE_HWORD | STM32_DMA_CR_PSIZE_HWORD |
                  STM32_DMA_CR_MINC        | STM32_DMA_CR_TCIE        |
                  STM32_DMA_CR_DMEIE       | STM32_DMA_CR_TEIE;
  nvicEnableVector(ADC1_IRQn, STM32_ADC_ADC1_IRQ_PRIORITY);
#endif

#if STM32_ADC_USE_SDADC1
  /* Driver initialization.*/
  adcObjectInit(&SDADCD1);
#if STM32_ADC_USE_ADC
  SDADCD1.adc     = NULL;
#endif
  SDADCD1.sdadc   = SDADC1;
  SDADCD1.dmastp  = NULL;
  SDADCD1.dmamode = STM32_DMA_CR_CHSEL(SDADC1_DMA_CHANNEL) |
                  STM32_DMA_CR_PL(STM32_ADC_SDADC1_DMA_PRIORITY) |
                  STM32_DMA_CR_DIR_P2M |
                  STM32_DMA_CR_MSIZE_HWORD | STM32_DMA_CR_PSIZE_HWORD |
                  STM32_DMA_CR_MINC        | STM32_DMA_CR_TCIE        |
                  STM32_DMA_CR_DMEIE       | STM32_DMA_CR_TEIE;
  nvicEnableVector(SDADC1_IRQn, STM32_ADC_SDADC1_IRQ_PRIORITY);
#endif

#if STM32_ADC_USE_SDADC2
  /* Driver initialization.*/
  adcObjectInit(&SDADCD2);
#if STM32_ADC_USE_ADC
  SDADCD2.adc     = NULL;
#endif
  SDADCD2.sdadc   = SDADC2;
  SDADCD2.dmastp  = NULL;
  SDADCD2.dmamode = STM32_DMA_CR_CHSEL(SDADC2_DMA_CHANNEL) |
                  STM32_DMA_CR_PL(STM32_ADC_SDADC2_DMA_PRIORITY) |
                  STM32_DMA_CR_DIR_P2M |
                  STM32_DMA_CR_MSIZE_HWORD | STM32_DMA_CR_PSIZE_HWORD |
                  STM32_DMA_CR_MINC        | STM32_DMA_CR_TCIE        |
                  STM32_DMA_CR_DMEIE       | STM32_DMA_CR_TEIE;
  nvicEnableVector(SDADC2_IRQn, STM32_ADC_SDADC2_IRQ_PRIORITY);
#endif

#if STM32_ADC_USE_SDADC3
  /* Driver initialization.*/
  adcObjectInit(&SDADCD3);
#if STM32_ADC_USE_ADC
  SDADCD3.adc     = NULL;
#endif
  SDADCD3.sdadc   = SDADC3;
  SDADCD3.dmastp  = NULL;
  SDADCD3.dmamode = STM32_DMA_CR_CHSEL(SDADC3_DMA_CHANNEL) |
                  STM32_DMA_CR_PL(STM32_ADC_SDADC3_DMA_PRIORITY) |
                  STM32_DMA_CR_DIR_P2M |
                  STM32_DMA_CR_MSIZE_HWORD | STM32_DMA_CR_PSIZE_HWORD |
                  STM32_DMA_CR_MINC        | STM32_DMA_CR_TCIE        |
                  STM32_DMA_CR_DMEIE       | STM32_DMA_CR_TEIE;
  nvicEnableVector(SDADC3_IRQn, STM32_ADC_SDADC3_IRQ_PRIORITY);
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

  if (adcp->config == NULL)
    adcp->config = &adc_lld_default_config;

  /* If in stopped state then enables the ADC and DMA clocks.*/
  if (adcp->state == ADC_STOP) {
#if STM32_ADC_USE_ADC1
    if (&ADCD1 == adcp) {
      adcp->dmastp = dmaStreamAllocI(STM32_DMA_STREAM_ID(1, 1),
                                     STM32_ADC_ADC1_DMA_IRQ_PRIORITY,
                                     (stm32_dmaisr_t)adc_lld_serve_dma_interrupt,
                                     (void *)adcp);
      osalDbgAssert(adcp->dmastp != NULL, "unable to allocate stream");

      dmaStreamSetPeripheral(adcp->dmastp, &ADC1->DR);
      rccEnableADC1(true);
    }
#endif /* STM32_ADC_USE_ADC1 */

#if STM32_ADC_USE_SDADC1
    if (&SDADCD1 == adcp) {
      adcp->dmastp = dmaStreamAllocI(STM32_DMA_STREAM_ID(2, 3),
                                     STM32_ADC_SDADC1_DMA_IRQ_PRIORITY,
                                     (stm32_dmaisr_t)adc_lld_serve_dma_interrupt,
                                     (void *)adcp);
      osalDbgAssert(adcp->dmastp != NULL, "unable to allocate stream");

      dmaStreamSetPeripheral(adcp->dmastp, &SDADC1->JDATAR);
      rccEnableSDADC1(true);
      PWR->CR |= PWR_CR_SDADC1EN;
      adcp->sdadc->CR2 = 0;
      adcp->sdadc->CR1 = (adcp->config->cr1 | SDADC_ENFORCED_CR1_FLAGS) &
                         ~SDADC_FORBIDDEN_CR1_FLAGS;
      adcp->sdadc->CR2 = SDADC_CR2_ADON;
    }
#endif /* STM32_ADC_USE_SDADC1 */

#if STM32_ADC_USE_SDADC2
    if (&SDADCD2 == adcp) {
      adcp->dmastp = dmaStreamAllocI(STM32_DMA_STREAM_ID(2, 4),
                                     STM32_ADC_SDADC2_DMA_IRQ_PRIORITY,
                                     (stm32_dmaisr_t)adc_lld_serve_dma_interrupt,
                                     (void *)adcp);
      osalDbgAssert(adcp->dmastp != NULL, "unable to allocate stream");

      dmaStreamSetPeripheral(adcp->dmastp, &SDADC2->JDATAR);
      rccEnableSDADC2(true);
      PWR->CR |= PWR_CR_SDADC2EN;
      adcp->sdadc->CR2 = 0;
      adcp->sdadc->CR1 = (adcp->config->cr1 | SDADC_ENFORCED_CR1_FLAGS) &
                         ~SDADC_FORBIDDEN_CR1_FLAGS;
      adcp->sdadc->CR2 = SDADC_CR2_ADON;
    }
#endif /* STM32_ADC_USE_SDADC2 */

#if STM32_ADC_USE_SDADC3
    if (&SDADCD3 == adcp) {
      adcp->dmastp = dmaStreamAllocI(STM32_DMA_STREAM_ID(2, 5),
                                     STM32_ADC_SDADC3_DMA_IRQ_PRIORITY,
                                     (stm32_dmaisr_t)adc_lld_serve_dma_interrupt,
                                     (void *)adcp);
      osalDbgAssert(adcp->dmastp != NULL, "unable to allocate stream");

      dmaStreamSetPeripheral(adcp->dmastp, &SDADC3->JDATAR);
      rccEnableSDADC3(true);
      PWR->CR |= PWR_CR_SDADC3EN;
      adcp->sdadc->CR2 = 0;
      adcp->sdadc->CR1 = (adcp->config->cr1 | SDADC_ENFORCED_CR1_FLAGS) &
                         ~SDADC_FORBIDDEN_CR1_FLAGS;
      adcp->sdadc->CR2 = SDADC_CR2_ADON;
    }
#endif /* STM32_ADC_USE_SDADC3 */
  }

  adc_lld_reconfig(adcp);
}

/**
 * @brief   Deactivates the ADC peripheral.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_lld_stop(ADCDriver *adcp) {

  /* If in ready state then disables the ADC clock.*/
  if (adcp->state == ADC_READY) {
    dmaStreamFreeI(adcp->dmastp);
    adcp->dmastp = NULL;

#if STM32_ADC_USE_ADC1
    if (&ADCD1 == adcp) {
      adcp->adc->CR1 = 0;
      adcp->adc->CR2 = 0;
      rccDisableADC1();
    }
#endif

#if STM32_ADC_USE_SDADC1
    if (&SDADCD1 == adcp) {
      adcp->sdadc->CR1 = 0;
      adcp->sdadc->CR2 = 0;
      rccDisableSDADC1();
      PWR->CR &= ~PWR_CR_SDADC1EN;
    }
#endif

#if STM32_ADC_USE_SDADC2
    if (&SDADCD2 == adcp) {
      adcp->sdadc->CR1 = 0;
      adcp->sdadc->CR2 = 0;
      rccDisableSDADC2();
      PWR->CR &= ~PWR_CR_SDADC2EN;
    }
#endif

#if STM32_ADC_USE_SDADC3
    if (&SDADCD3 == adcp) {
      adcp->sdadc->CR1 = 0;
      adcp->sdadc->CR2 = 0;
      rccDisableSDADC3();
      PWR->CR &= ~PWR_CR_SDADC3EN;
    }
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
  uint32_t mode;
  const ADCConversionGroup* grpp = adcp->grpp;

  /* DMA setup.*/
  mode = adcp->dmamode;
  if (grpp->circular) {
    mode |= STM32_DMA_CR_CIRC;
    if (adcp->depth > 1) {
      /* If circular buffer depth > 1, then the half transfer interrupt
         is enabled in order to allow streaming processing.*/
      mode |= STM32_DMA_CR_HTIE;
    }
  }
  dmaStreamSetMemory0(adcp->dmastp, adcp->samples);
  dmaStreamSetTransactionSize(adcp->dmastp,
                              (uint32_t)grpp->num_channels *
                              (uint32_t)adcp->depth);
  dmaStreamSetMode(adcp->dmastp, mode);
  dmaStreamEnable(adcp->dmastp);

#if STM32_ADC_USE_ADC && STM32_ADC_USE_SDADC
  if (adcp->adc != NULL)
#endif /* STM32_ADC_USE_ADC && STM32_ADC_USE_SDADC */
#if STM32_ADC_USE_ADC
  {
    uint32_t cr2 = adcp->adc->CR2 & ADC_CR2_TSVREFE;
    cr2 |= grpp->u.adc.cr2 | ADC_CR2_DMA | ADC_CR2_ADON;
    if ((cr2 & ADC_CR2_SWSTART) != 0)
      cr2 |= ADC_CR2_CONT;
    adcp->adc->CR2   = cr2;

    /* ADC setup.*/
    adcp->adc->SR    = 0;
    adcp->adc->LTR   = grpp->u.adc.ltr;
    adcp->adc->HTR   = grpp->u.adc.htr;
    adcp->adc->SMPR1 = grpp->u.adc.smpr[0];
    adcp->adc->SMPR2 = grpp->u.adc.smpr[1];
    adcp->adc->SQR1  = grpp->u.adc.sqr[0] |
                       ADC_SQR1_NUM_CH(grpp->num_channels);
    adcp->adc->SQR2  = grpp->u.adc.sqr[1];
    adcp->adc->SQR3  = grpp->u.adc.sqr[2];

    /* ADC conversion start, the start is performed using the method
       specified in the CR2 configuration, usually ADC_CR2_SWSTART.*/
    adcp->adc->CR1   = grpp->u.adc.cr1 | ADC_CR1_AWDIE | ADC_CR1_SCAN;
    adcp->adc->CR2   = adcp->adc->CR2;  /* Triggers the conversion start.*/
  }
#endif /* STM32_ADC_USE_ADC */
#if STM32_ADC_USE_ADC && STM32_ADC_USE_SDADC
  else if (adcp->sdadc != NULL)
#endif /* STM32_ADC_USE_ADC && STM32_ADC_USE_SDADC */
#if STM32_ADC_USE_SDADC
  {
    uint32_t cr2 = (grpp->u.sdadc.cr2 & ~SDADC_FORBIDDEN_CR2_FLAGS) |
                   SDADC_CR2_ADON;
    if ((grpp->u.sdadc.cr2 & SDADC_CR2_JSWSTART) != 0)
      cr2 |= SDADC_CR2_JCONT;

    /* Entering initialization mode.*/
    adcp->sdadc->CR1 |= SDADC_CR1_INIT;
    while ((adcp->sdadc->ISR & SDADC_ISR_INITRDY) == 0)
      ;

    /* SDADC setup.*/
    adcp->sdadc->JCHGR    = grpp->u.sdadc.jchgr;
    adcp->sdadc->CONFCHR1 = grpp->u.sdadc.confchr[0];
    adcp->sdadc->CONFCHR2 = grpp->u.sdadc.confchr[1];

    /* SDADC trigger modes, this write must be performed when
       SDADC_CR1_INIT=1.*/
    adcp->sdadc->CR2 = cr2;

    /* Leaving initialization mode.*/
    adcp->sdadc->CR1 &= ~SDADC_CR1_INIT;

    /* Special case, if SDADC_CR2_JSWSTART is specified it has to be
       written after SDADC_CR1_INIT has been set to zero. Just a write is
       performed, any other bit is ingore if not in initialization mode.*/
    adcp->sdadc->CR2 = cr2;
  }
#endif /* STM32_ADC_USE_SDADC */
#if STM32_ADC_USE_ADC && STM32_ADC_USE_SDADC
  else {
    osalDbgAssert(FALSE, "invalid state");
  }
#endif /* STM32_ADC_USE_ADC && STM32_ADC_USE_SDADC */
}

/**
 * @brief   Stops an ongoing conversion.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_lld_stop_conversion(ADCDriver *adcp) {

  /* Disabling the associated DMA stream.*/
  dmaStreamDisable(adcp->dmastp);

  /* Stopping and restarting the whole ADC, apparently the only way to stop
     a conversion.*/
  adc_lld_reconfig(adcp);
}

/**
 * @brief   Calibrates an ADC unit.
 * @note    The calibration must be performed after calling @p adcStart().
 * @note    For SDADC units it is assumed that the field SDADC_CR2_CALIBCNT
 *          has been
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @api
 */
void adcSTM32Calibrate(ADCDriver *adcp) {

  osalDbgAssert((adcp->state == ADC_READY) ||
              (adcp->state == ADC_COMPLETE) ||
              (adcp->state == ADC_ERROR),
              "not ready");

#if STM32_ADC_USE_ADC && STM32_ADC_USE_SDADC
  if (adcp->adc != NULL)
#endif /* STM32_ADC_USE_ADC && STM32_ADC_USE_SDADC */
#if STM32_ADC_USE_ADC
  {
    /* Resetting calibration just to be safe.*/
    ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_RSTCAL;
    while ((ADC1->CR2 & ADC_CR2_RSTCAL) != 0)
      ;

    /* Calibration.*/
    ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_CAL;
    while ((ADC1->CR2 & ADC_CR2_CAL) != 0)
      ;
  }
#endif /* STM32_ADC_USE_ADC */
#if STM32_ADC_USE_ADC && STM32_ADC_USE_SDADC
  else if (adcp->sdadc != NULL)
#endif /* STM32_ADC_USE_ADC && STM32_ADC_USE_SDADC */
#if STM32_ADC_USE_SDADC
  {
    /* Selecting a full calibration in three steps.*/
    adcp->sdadc->CR2  = (adcp->sdadc->CR2 & ~SDADC_CR2_CALIBCNT) |
                        SDADC_CR2_CALIBCNT_1;

    /* Calibration.*/
    adcp->sdadc->CR2 |= SDADC_CR2_STARTCALIB;
    while ((adcp->sdadc->ISR & SDADC_ISR_EOCALF) == 0)
      ;

    /* Clearing the EOCALF flag.*/
    adcp->sdadc->CLRISR |= SDADC_ISR_CLREOCALF;
  }
#endif /* STM32_ADC_USE_SDADC */
#if STM32_ADC_USE_ADC && STM32_ADC_USE_SDADC
  else {
    osalDbgAssert(FALSE, "invalid state");
  }
#endif /* STM32_ADC_USE_ADC && STM32_ADC_USE_SDADC */
}

#if STM32_ADC_USE_ADC || defined(__DOXYGEN__)
/**
 * @brief   Enables the TSVREFE bit.
 * @details The TSVREFE bit is required in order to sample the internal
 *          temperature sensor and internal reference voltage.
 * @note    This is an STM32-only functionality.
 *
 * @api
 */
void adcSTM32EnableTSVREFE(void) {

  ADC1->CR2 |= ADC_CR2_TSVREFE;
}

/**
 * @brief   Disables the TSVREFE bit.
 * @details The TSVREFE bit is required in order to sample the internal
 *          temperature sensor and internal reference voltage.
 * @note    This is an STM32-only functionality.
 *
 * @api
 */
void adcSTM32DisableTSVREFE(void) {

  ADC1->CR2 &= ~ADC_CR2_TSVREFE;
}

/**
 * @brief   Enables the VBATE bit.
 * @details The VBATE bit is required in order to sample the VBAT channel.
 * @note    This is an STM32-only functionality.
 *
 * @api
 */
void adcSTM32EnableVBATE(void) {

  SYSCFG->CFGR1 |= SYSCFG_CFGR1_VBAT;
}

/**
 * @brief   Disables the VBATE bit.
 * @details The VBATE bit is required in order to sample the VBAT channel.
 * @note    This is an STM32-only functionality.
 *
 * @api
 */
void adcSTM32DisableVBATE(void) {

  SYSCFG->CFGR1 &= ~SYSCFG_CFGR1_VBAT;
}
#endif /* STM32_ADC_USE_ADC */

#endif /* HAL_USE_ADC */

/** @} */
