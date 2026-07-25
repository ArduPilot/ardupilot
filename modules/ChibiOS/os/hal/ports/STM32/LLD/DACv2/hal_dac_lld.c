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
 * @file    DACv2/hal_dac_lld.c
 * @brief   STM32 DAC subsystem low level driver source.
 *
 * @addtogroup DAC
 * @{
 */

#include "hal.h"

#if HAL_USE_DAC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define CHANNEL_DATA_OFFSET           (12U / 4U)
#define CHANNEL_REGISTER_SHIFT        16U
#define CHANNEL_REGISTER_MASK1        0xFFFF0000U
#define CHANNEL_REGISTER_MASK2        0x0000FFFFU
#define CONFIG_SINGLE_MASK            0x0000FFFFU

#define BYTE_SINGLE_SAMPLE_MULTIPLIER 1U
#define HALF_SINGLE_SAMPLE_MULTIPLIER 2U
#define WORD_SINGLE_SAMPLE_MULTIPLIER 4U
#define BYTE_DUAL_SAMPLE_MULTIPLIER   2U
#define HALF_DUAL_SAMPLE_MULTIPLIER   4U

#define HF_SEL_AHB_GT_80MHZ           80000000U
#define HF_SEL_AHB_GT_160MHZ          160000000U

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/* Fix ST headers naming inconsistencies.*/
#if !defined(DAC1)
#define DAC1 DAC
#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief DAC1 CH1 driver identifier.*/
#if STM32_DAC_USE_DAC1_CH1 || defined(__DOXYGEN__)
DACDriver DACD1;
#endif

/** @brief DAC1 CH2 driver identifier.*/
#if STM32_DAC_USE_DAC1_CH2 || defined(__DOXYGEN__)
DACDriver DACD2;
#endif

/** @brief DAC2 CH1 driver identifier.*/
#if STM32_DAC_USE_DAC2_CH1 || defined(__DOXYGEN__)
DACDriver DACD3;
#endif

/** @brief DAC2 CH2 driver identifier.*/
#if STM32_DAC_USE_DAC2_CH2 || defined(__DOXYGEN__)
DACDriver DACD4;
#endif

/** @brief DAC3 CH1 driver identifier.*/
#if STM32_DAC_USE_DAC3_CH1 || defined(__DOXYGEN__)
DACDriver DACD5;
#endif

/** @brief DAC3 CH2 driver identifier.*/
#if STM32_DAC_USE_DAC3_CH2 || defined(__DOXYGEN__)
DACDriver DACD6;
#endif

/** @brief DAC4 CH1 driver identifier.*/
#if STM32_DAC_USE_DAC4_CH1 || defined(__DOXYGEN__)
DACDriver DACD7;
#endif

/** @brief DAC4 CH2 driver identifier.*/
#if STM32_DAC_USE_DAC4_CH2 || defined(__DOXYGEN__)
DACDriver DACD8;
#endif

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

#if STM32_DAC_USE_DAC1_CH1 == TRUE
static const dacparams_t dac1_ch1_params = {
  .dac          = DAC1,
  .dataoffset   = 0U,
  .regshift     = 0U,
  .regmask      = CHANNEL_REGISTER_MASK1,
  .dmach        = STM32_DAC_DAC1_CH1_DMA3_CHANNEL,
  .dmaprio      = STM32_DAC_DAC1_CH1_DMA_PRIORITY,
  .dmareq       = STM32_DMA3_REQ_DAC1_CH1,
  .dmairqprio   = STM32_IRQ_DAC1_PRIORITY,
};
#endif

#if STM32_DAC_USE_DAC1_CH2 == TRUE
static const dacparams_t dac1_ch2_params = {
  .dac          = DAC1,
  .dataoffset   = CHANNEL_DATA_OFFSET,
  .regshift     = CHANNEL_REGISTER_SHIFT,
  .regmask      = CHANNEL_REGISTER_MASK2,
  .dmach        = STM32_DAC_DAC1_CH2_DMA3_CHANNEL,
  .dmaprio      = STM32_DAC_DAC1_CH2_DMA_PRIORITY,
  .dmareq       = STM32_DMA3_REQ_DAC1_CH2,
  .dmairqprio   = STM32_IRQ_DAC1_PRIORITY,
};
#endif

#if STM32_DAC_USE_DAC2_CH1 == TRUE
static const dacparams_t dac2_ch1_params = {
  .dac          = DAC2,
  .dataoffset   = 0U,
  .regshift     = 0U,
  .regmask      = CHANNEL_REGISTER_MASK1,
  .dmach        = STM32_DAC_DAC2_CH1_DMA3_CHANNEL,
  .dmaprio      = STM32_DAC_DAC2_CH1_DMA_PRIORITY,
  .dmareq       = STM32_DMA3_REQ_DAC2_CH1,
  .dmairqprio   = STM32_IRQ_DAC2_PRIORITY,
};
#endif

#if STM32_DAC_USE_DAC2_CH2 == TRUE
static const dacparams_t dac2_ch2_params = {
  .dac          = DAC2,
  .dataoffset   = CHANNEL_DATA_OFFSET,
  .regshift     = CHANNEL_REGISTER_SHIFT,
  .regmask      = CHANNEL_REGISTER_MASK2,
  .dmach        = STM32_DAC_DAC2_CH2_DMA3_CHANNEL,
  .dmaprio      = STM32_DAC_DAC2_CH2_DMA_PRIORITY,
  .dmareq       = STM32_DMA3_REQ_DAC2_CH2,
  .dmairqprio   = STM32_IRQ_DAC2_PRIORITY,
};
#endif

#if STM32_DAC_USE_DAC3_CH1 == TRUE
static const dacparams_t dac3_ch1_params = {
  .dac          = DAC3,
  .dataoffset   = 0U,
  .regshift     = 0U,
  .regmask      = CHANNEL_REGISTER_MASK1,
  .dmach        = STM32_DAC_DAC3_CH1_DMA3_CHANNEL,
  .dmaprio      = STM32_DAC_DAC3_CH1_DMA_PRIORITY,
  .dmareq       = STM32_DMA3_REQ_DAC3_CH1,
  .dmairqprio   = STM32_IRQ_DAC3_PRIORITY,
};
#endif

#if STM32_DAC_USE_DAC3_CH2 == TRUE
static const dacparams_t dac3_ch2_params = {
  .dac          = DAC3,
  .dataoffset   = CHANNEL_DATA_OFFSET,
  .regshift     = CHANNEL_REGISTER_SHIFT,
  .regmask      = CHANNEL_REGISTER_MASK2,
  .dmach        = STM32_DAC_DAC3_CH2_DMA3_CHANNEL,
  .dmaprio      = STM32_DAC_DAC3_CH2_DMA_PRIORITY,
  .dmareq       = STM32_DMA3_REQ_DAC3_CH2,
  .dmairqprio   = STM32_IRQ_DAC3_PRIORITY,
};
#endif

#if STM32_DAC_USE_DAC4_CH1 == TRUE
static const dacparams_t dac4_ch1_params = {
  .dac          = DAC4,
  .dataoffset   = 0U,
  .regshift     = 0U,
  .regmask      = CHANNEL_REGISTER_MASK1,
  .dmach        = STM32_DAC_DAC4_CH1_DMA3_CHANNEL,
  .dmaprio      = STM32_DAC_DAC4_CH1_DMA_PRIORITY,
  .dmareq       = STM32_DMA3_REQ_DAC4_CH1,
  .dmairqprio   = STM32_IRQ_DAC4_PRIORITY,
};
#endif

#if STM32_DAC_USE_DAC4_CH2 == TRUE
static const dacparams_t dac4_ch2_params = {
  .dac          = DAC4,
  .dataoffset   = CHANNEL_DATA_OFFSET,
  .regshift     = CHANNEL_REGISTER_SHIFT,
  .regmask      = CHANNEL_REGISTER_MASK2,
  .dmach        = STM32_DAC_DAC4_CH2_DMA3_CHANNEL,
  .dmaprio      = STM32_DAC_DAC4_CH2_DMA_PRIORITY,
  .dmareq       = STM32_DMA3_REQ_DAC4_CH2,
  .dmairqprio   = STM32_IRQ_DAC4_PRIORITY,
};
#endif

/* DMA circular link control.*/

#if STM32_DAC_USE_DAC1_CH1 || defined(__DOXYGEN__)
static dac_dmabuf_t __dma3_dac1_ch1;
#endif

#if STM32_DAC_USE_DAC1_CH2 || defined(__DOXYGEN__)
static dac_dmabuf_t __dma3_dac1_ch2;
#endif

#if STM32_DAC_USE_DAC2_CH1 || defined(__DOXYGEN__)
static dac_dmabuf_t __dma3_dac2_ch1;
#endif

#if STM32_DAC_USE_DAC2_CH2 || defined(__DOXYGEN__)
static dac_dmabuf_t __dma3_dac2_ch2;
#endif

#if STM32_DAC_USE_DAC3_CH1 || defined(__DOXYGEN__)
static dac_dmabuf_t __dma3_dac3_ch1;
#endif

#if STM32_DAC_USE_DAC3_CH2 || defined(__DOXYGEN__)
static dac_dmabuf_t __dma3_dac3_ch2;
#endif

#if STM32_DAC_USE_DAC4_CH1 || defined(__DOXYGEN__)
static dac_dmabuf_t __dma3_dac4_ch1;
#endif

#if STM32_DAC_USE_DAC4_CH2 || defined(__DOXYGEN__)
static dac_dmabuf_t __dma3_dac4_ch2;
#endif

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Shared DAC GPDMA service routine.
 *
 * @param[in] p         parameter for the registered function
 * @param[in] flags     content of the CxSR register
 *
 * @isr
 */
static void dac_lld_serve_dma_interrupt(void *p, uint32_t flags) {
  DACDriver *dacp = (DACDriver *)p;

  if ((flags & STM32_DMA3_CSR_ERRORS) != 0) {
    /* DMA errors handling.*/
    _dac_isr_error_code(dacp, DAC_ERR_DMAFAILURE);
  }
  else {
    /* It is possible that the conversion group has already been reset by a
       DAC error handler. In this case this interrupt is spurious.*/
    if (dacp->grpp != NULL) {
      if ((flags & STM32_DMA3_CSR_HTF) != 0) {
        /* Half transfer processing.*/
        _dac_isr_half_code(dacp);
      }
      if ((flags & STM32_DMA3_CSR_TCF) != 0) {
        /* Transfer complete processing.*/
        _dac_isr_full_code(dacp);
      }
    }
  }
}

/**
 * @brief   Outputs a value directly on a DAC channel.
 * @note    The value may be 8 bit or 12 bit for setting DOR.
 *          The value will contain the shifted value for DORB when
 *          double DMA mode is enabled.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 * @param[in] channel   DAC channel number
 * @param[in] value     value to be output to the DAC holding register
 *
 * @notapi
 */
static msg_t put_channel(DACDriver *dacp,
                         dacchannel_t channel,
                         uint32_t value) {

  switch (dacp->config->datamode) {
    case DAC_DHRM_12BIT_RIGHT:
#if STM32_DAC_DUAL_MODE
    case DAC_DHRM_12BIT_RIGHT_DUAL:
#endif
      if (channel == 0U) {
#if STM32_DAC_DUAL_MODE
        dacp->params->dac->DHR12R1 = value;
#else
        *(&dacp->params->dac->DHR12R1 + dacp->params->dataoffset) = value;
#endif
      }
#if (STM32_HAS_DAC1_CH2 || STM32_HAS_DAC2_CH2 ||                            \
    STM32_HAS_DAC3_CH2 || STM32_HAS_DAC4_CH2)
      else {
        dacp->params->dac->DHR12R2 = value;
      }
#endif
      break;
    case DAC_DHRM_12BIT_LEFT:
#if STM32_DAC_DUAL_MODE
    case DAC_DHRM_12BIT_LEFT_DUAL:
#endif
      if (channel == 0U) {
#if STM32_DAC_DUAL_MODE
        dacp->params->dac->DHR12L1 = value;
#else
        *(&dacp->params->dac->DHR12L1 + dacp->params->dataoffset) = value;
#endif
      }
#if (STM32_HAS_DAC1_CH2 || STM32_HAS_DAC2_CH2 ||                            \
    STM32_HAS_DAC3_CH2 || STM32_HAS_DAC4_CH2)
      else {
        dacp->params->dac->DHR12L2 = value;
      }
#endif
      break;
    case DAC_DHRM_8BIT_RIGHT:
#if STM32_DAC_DUAL_MODE
    case DAC_DHRM_8BIT_RIGHT_DUAL:
#endif
      if (channel == 0U) {
#if STM32_DAC_DUAL_MODE
        dacp->params->dac->DHR8R1 = value;
#else
        *(&dacp->params->dac->DHR8R1 + dacp->params->dataoffset) = (uint8_t)value;
#endif
      }
#if (STM32_HAS_DAC1_CH2 || STM32_HAS_DAC2_CH2 ||                            \
    STM32_HAS_DAC3_CH2 || STM32_HAS_DAC4_CH2)
      else {
        dacp->params->dac->DHR8R2 = (uint8_t)value;
      }
#endif
      break;
    default:
      osalDbgAssert(false, "unknown DAC mode");
      return HAL_RET_CONFIG_ERROR;
  }
  return HAL_RET_SUCCESS;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level DAC driver initialization.
 *
 * @notapi
 */
void dac_lld_init(void) {

  /* Initialise driver fields.*/
#if STM32_DAC_USE_DAC1_CH1
  dacObjectInit(&DACD1);
  DACD1.params  = &dac1_ch1_params;
  DACD1.dmachp = NULL;
  DACD1.dbuf    = &__dma3_dac1_ch1;
#endif

#if STM32_DAC_USE_DAC1_CH2
  dacObjectInit(&DACD2);
  DACD2.params  = &dac1_ch2_params;
  DACD2.dmachp = NULL;
  DACD2.dbuf    = &__dma3_dac1_ch2;
#endif

#if STM32_DAC_USE_DAC2_CH1
  dacObjectInit(&DACD3);
  DACD3.params  = &dac2_ch1_params;
  DACD3.dmachp = NULL;
  DACD3.dbuf    = &__dma3_dac2_ch1;
#endif

#if STM32_DAC_USE_DAC2_CH2
  dacObjectInit(&DACD4);
  DACD4.params  = &dac2_ch2_params;
  DACD4.dmachp = NULL;
  DACD4.dbuf    = &__dma3_dac2_ch2;
#endif

#if STM32_DAC_USE_DAC3_CH1
  dacObjectInit(&DACD5);
  DACD5.params  = &dac3_ch1_params;
  DACD5.dmachp = NULL;
  DACD5.dbuf    = &__dma3_dac3_ch1;
#endif

#if STM32_DAC_USE_DAC3_CH2
  dacObjectInit(&DACD6);
  DACD6.params  = &dac3_ch2_params;
  DACD6.dmachp = NULL;
  DACD6.dbuf    = &__dma3_dac3_ch2;
#endif

#if STM32_DAC_USE_DAC4_CH1
  dacObjectInit(&DACD7);
  DACD7.params  = &dac4_ch1_params;
  DACD7.dmachp = NULL;
  DACD7.dbuf    = &__dma3_dac4_ch1;
#endif

#if STM32_DAC_USE_DAC4_CH2
  dacObjectInit(&DACD8);
  DACD8.params  = &dac4_ch2_params;
  DACD8.dmachp = NULL;
  DACD8.dbuf    = &__dma3_dac4_ch2;
#endif

  /* Used DAC units reset on initialization, note, reset must occur with
     clock enabled.*/
#if STM32_DAC_USE_DAC1_CH1 || STM32_DAC_USE_DAC1_CH2
  rccEnableDAC1(false);
  rccResetDAC1();
  rccDisableDAC1();
#endif

#if STM32_DAC_USE_DAC2_CH1 || STM32_DAC_USE_DAC2_CH2
  rccEnableDAC1(false);
  rccResetDAC2();
  rccDisableDAC2();
#endif

#if STM32_DAC_USE_DAC3_CH1 || STM32_DAC_USE_DAC3_CH2
  rccEnableDAC3(false);
  rccResetDAC3();
  rccDisableDAC3();
#endif

#if STM32_DAC_USE_DAC4_CH1 || STM32_DAC_USE_DAC4_CH2
  rccEnableDAC4(false);
  rccResetDAC4();
  rccDisableDAC4();
#endif
}

/**
 * @brief   Configures and activates the DAC peripheral.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 *
 * @notapi
 */
msg_t dac_lld_start(DACDriver *dacp) {

  /* If the driver is in DAC_STOP state then a full initialization is
     required.*/
  if (dacp->state == DAC_STOP) {
    dacchannel_t channel = 0;

    /* Enable DAC clock. DMA channel allocation is deferred to conversion
       start and only allocated if a group conversion is used.*/

    if (false) {
    }
#if STM32_DAC_USE_DAC1_CH1
    else if (&DACD1 == dacp) {
      rccEnableDAC1(true);
    }
#endif

#if STM32_DAC_USE_DAC1_CH2
    else if (&DACD2 == dacp) {
      rccEnableDAC1(true);
      channel = 1;
    }
#endif

#if STM32_DAC_USE_DAC2_CH1
    else if (&DACD3 == dacp) {
      rccEnableDAC2(true);
    }
#endif

#if STM32_DAC_USE_DAC2_CH2
    else if (&DACD4 == dacp) {
      rccEnableDAC2(true);
      channel = 1;
    }
#endif

#if STM32_DAC_USE_DAC3_CH1
    else if (&DACD5 == dacp) {
      rccEnableDAC3(true);
    }
#endif

#if STM32_DAC_USE_DAC3_CH2
    else if (&DACD6 == dacp) {
      rccEnableDAC3(true);
      channel = 1;
    }
#endif

#if STM32_DAC_USE_DAC4_CH1
    else if (&DACD7 == dacp) {
      rccEnableDAC4(true);
    }
#endif

#if STM32_DAC_USE_DAC4_CH2
    else if (&DACD8 == dacp) {
      rccEnableDAC4(true);
      channel = 1;
    }
#endif

    else {
      osalDbgAssert(false, "unknown DAC instance");
      return HAL_RET_NO_RESOURCE;
    }

    /* Enabling DAC in SW triggering mode initially, initializing data to
       configuration default.*/
    uint32_t reg;

#if STM32_DAC_DUAL_MODE == FALSE
    /* Operating in SINGLE mode. Setup registers for specified channel from
       configuration. Lower half word of configuration specifies configuration
       for either channel 1 or 2.*/
    reg = (dacp->params->dac->MCR & dacp->params->regmask);

    /* Handle HFSEL setting based on DAC clock.*/
    reg &= ~(DAC_MCR_HFSEL_0 | DAC_MCR_HFSEL_1);
    if (STM32_ADCDACCLK > HF_SEL_AHB_GT_160MHZ) {
      reg |= DAC_MCR_HFSEL_1;
    }
    else if (STM32_ADCDACCLK > HF_SEL_AHB_GT_80MHZ) {
      reg |= DAC_MCR_HFSEL_0;
    }

    /* Disable double DMA setting so DOR is updated.*/
    reg &= ~(DAC_MCR_DMADOUBLE1 << dacp->params->regshift);
    dacp->params->dac->MCR = reg |
      ((dacp->config->mcr & CONFIG_SINGLE_MASK) << dacp->params->regshift);

    /* Enable and initialise the channel.*/
    reg = dacp->params->dac->CR;
    reg &= dacp->params->regmask;
    reg |= (DAC_CR_EN1 | (dacp->config->cr & CONFIG_SINGLE_MASK)) <<
                                            dacp->params->regshift;
    dacp->params->dac->CR = reg;
    (void) put_channel(dacp, channel, (dacsample_t)dacp->config->init);
#else /* STM32_DAC_DUAL_MODE != FALSE */
    /* Operating in DUAL mode with two channels to setup. Set registers for
       both channels from configuration. Lower and upper half words specify
       configuration for channels 1 & 2 respectively.*/
    (void)channel;
    reg = (dacp->params->dac->MCR & dacp->params->regmask);

    /* Handle HFSEL setting based on DAC clock.*/
    reg = (dacp->params->dac->MCR | dacp->config->mcr) &
                      ~(DAC_MCR_HFSEL_0 | DAC_MCR_HFSEL_1);
    if (STM32_ADCDACCLK > HF_SEL_AHB_GT_160MHZ) {
      reg |= DAC_MCR_HFSEL_1;
    }
    else if (STM32_ADCDACCLK > HF_SEL_AHB_GT_80MHZ) {
      reg |= DAC_MCR_HFSEL_0;
    }

    /* Disable double DMA setting so DOR is updated.*/
    reg &= ~(DAC_MCR_DMADOUBLE1 << dacp->params->regshift);
    dacp->params->dac->MCR = reg;

    /* Enable and initialise both channels 1 and 2. Mask out DMA enable.*/
    reg = dacp->config->cr;
    reg &= ~(DAC_CR_DMAEN1 | DAC_CR_DMAEN2);
    dacp->params->dac->CR = DAC_CR_EN2 | DAC_CR_EN1 | reg;
    (void) put_channel(dacp, 0U, (dacsample_t)dacp->config->init);
    (void) put_channel(dacp, 1U, (dacsample_t)(dacp->config->init >>
                                                CHANNEL_REGISTER_SHIFT));
#endif /* STM32_DAC_DUAL_MODE == FALSE */
  }
  return HAL_RET_SUCCESS;
}

/**
 * @brief   Deactivates the DAC peripheral.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 *
 * @notapi
 */
void dac_lld_stop(DACDriver *dacp) {

  /* If in ready state then disables the DAC clock.*/
  if (dacp->state == DAC_READY) {

    /* Disabling DAC conditionally.*/
    dacp->params->dac->CR &= dacp->params->regmask;

#if STM32_DAC_USE_DAC1_CH1
    if (&DACD1 == dacp) {
#if defined(DAC_CR_EN2)
      if ((dacp->params->dac->CR & DAC_CR_EN2) != 0U) {
        return;
      }
      rccDisableDAC1();
#endif
      return;
    }
#endif

#if STM32_DAC_USE_DAC1_CH2
    if (&DACD2 == dacp) {
      if ((dacp->params->dac->CR & DAC_CR_EN1) == 0U) {
        rccDisableDAC1();
      }
      return;
    }
#endif

#if STM32_DAC_USE_DAC2_CH1
    if (&DACD3 == dacp) {
#if defined(DAC_CR_EN2)
      if ((dacp->params->dac->CR & DAC_CR_EN2) != 0U) {
        return;
      }
      rccDisableDAC2();
#endif
      return;
    }
#endif

#if STM32_DAC_USE_DAC2_CH2
    if (&DACD4 == dacp) {
      if ((dacp->params->dac->CR & DAC_CR_EN1) == 0U) {
        rccDisableDAC2();
      }
      return;
    }
#endif

#if STM32_DAC_USE_DAC3_CH1
    if (&DACD5 == dacp) {
#if defined(DAC_CR_EN2)
      if ((dacp->params->dac->CR & DAC_CR_EN2) != 0U) {
        return;
      }
      rccDisableDAC3();
#endif
      return;
    }
#endif

#if STM32_DAC_USE_DAC3_CH2
    if (&DACD6 == dacp) {
      if ((dacp->params->dac->CR & DAC_CR_EN1) == 0U) {
        rccDisableDAC3();
      }
      return;
    }
#endif

#if STM32_DAC_USE_DAC4_CH1
    if (&DACD7 == dacp) {
#if defined(DAC_CR_EN2)
      if ((dacp->params->dac->CR & DAC_CR_EN2) != 0U) {
        return;
      }
      rccDisableDAC4();
#endif
      return;
    }
#endif

#if STM32_DAC_USE_DAC4_CH2
    if (&DACD8 == dacp) {
      if ((dacp->params->dac->CR & DAC_CR_EN1) == 0U) {
        rccDisableDAC4();
      }
      return;
    }
#endif
  }
}

/**
 * @brief   Outputs a value directly on a DAC channel.
 * @note    While a group is active in DUAL mode on CH1 only then CH2
 *          is available for normal output (put) operations.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 * @param[in] channel   DAC channel number
 * @param[in] sample    value to be output
 *
 * @return              The operation status.
 *
 * @notapi
 */
msg_t dac_lld_put_channel(DACDriver *dacp,
                         dacchannel_t channel,
                         dacsample_t sample) {

#if STM32_DAC_DUAL_MODE
  if ((dacp->state == DAC_ACTIVE || dacp->state == DAC_COMPLETE) &&
       dacp->grpp->num_channels == 2 && channel == 0) {
    osalDbgCheck(false);
    return HAL_RET_HW_BUSY;
  }
#endif /* STM32_DAC_DUAL_MODE */

  /* Check if channel enabled.*/
  if ((dacp->params->dac->SR &
                  (DAC_SR_DAC1RDY << dacp->params->regshift)) == 0) {
    return HAL_RET_NO_RESOURCE;
  }

  return put_channel(dacp, channel, (uint32_t)sample);

}

/**
 * @brief   Starts a DAC conversion.
 * @details Starts an asynchronous conversion operation.
 * @note    In @p DAC_DHRM_8BIT_RIGHT mode two samples are packed in a single
 *          dacsample_t element. DMA does byte read.
 * @note    In @p DAC_DHRM_8BIT_RIGHT_DUAL mode two samples are treated
 *          as a single 16 bits sample and packed into a single dacsample_t
 *          element. The num_channels must be set to one in the group
 *          conversion configuration structure. DMA does half word read.
 * @note    If using DUAL mode with a single channel conversion then channel 2
 *          is enabled for manual (put_channel) for non DMA triggered use. The
 *          the data format for put operations is specified in the upper half
 *          word of the 'datamode' field. The CR setting is in the upper half
 *          word of the 'cr' field of the configuration.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 *
 * @return              The operation status.
 *
 * @notapi
 */
msg_t dac_lld_start_conversion(DACDriver *dacp) {
  uint32_t n, ni, nch, cr, dmamode, dmaccr, dmallr, chx, ch2;
  volatile const void *dacreg;
  uint8_t *si;
  uint8_t mult;
  bool dacddma = false;

  if (dacp->grpp->num_channels < 1) {
    osalDbgAssert(false, "invalid number of channels");
    return HAL_RET_CONFIG_ERROR;
  }

  /* Determine double DMA mode.*/
  dacddma = ((((dacp->config->mcr & ~dacp->params->regmask)
      >> dacp->params->regshift) & DAC_MCR_DMADOUBLE1) != 0);

  /* DMA settings depend on the chosen DAC mode. If not in dual mode then each
     channel of a DAC operates independently. The DAC DMA update request of DHR
     happens after the DAC does DHR to DOR transfer. Thus the first sample must
     be loaded into DHR prior to conversion start. The initial DMA count and
     source address are adjusted accordingly for first DMA cycle and then set
     for subsequent conversion cycles using the GPDMA LLR. All DMA to DAC is
     32 bit since DAC is on AHB.*/
  switch (dacp->config->datamode) {

    case DAC_DHRM_12BIT_RIGHT:

      /* One channel where data is a 16 bit (dacsample_t). GPDMA count is 2 bytes
       per transfer.*/
      nch = 1U;
      dacreg = &dacp->params->dac->DHR12R1 + dacp->params->dataoffset;
      dmamode = STM32_DMA3_CTR1_DDW_WORD;
      dmamode |= dacddma ? STM32_DMA3_CTR1_SDW_WORD : STM32_DMA3_CTR1_SDW_HALF;
      mult = HALF_SINGLE_SAMPLE_MULTIPLIER;

      /* Set initial value of channel holding register(s).*/
      chx = *(uint32_t *)(void *)dacp->samples;
      break;

    case DAC_DHRM_12BIT_LEFT:

      /* One channel where data is a 16 bit (dacsample_t). GPDMA count is 2 bytes
       per transfer.*/
      nch = 1U;
      dacreg = &dacp->params->dac->DHR12L1 + dacp->params->dataoffset;
      dmamode = STM32_DMA3_CTR1_DDW_WORD;
      dmamode |= dacddma ? STM32_DMA3_CTR1_SDW_WORD : STM32_DMA3_CTR1_SDW_HALF;
      mult = HALF_SINGLE_SAMPLE_MULTIPLIER;

      /* Get initial value of channel holding register(s).*/
      chx = *(uint32_t *)(void *)dacp->samples;
      break;

    case DAC_DHRM_8BIT_RIGHT:

      /* One channel where data is in bytes. GPDMA count is 1 byte per transfer.*/
      nch = 1U;
      dacreg = &dacp->params->dac->DHR8R1 + dacp->params->dataoffset;
      dmamode = STM32_DMA3_CTR1_DDW_WORD;
      dmamode |= dacddma ? STM32_DMA3_CTR1_SDW_HALF : STM32_DMA3_CTR1_SDW_BYTE;
      mult = BYTE_SINGLE_SAMPLE_MULTIPLIER;

      /* Get initial value of channel holding register(s).*/
      chx = (uint32_t)*(uint16_t *)dacp->samples;
      break;

#if STM32_DAC_DUAL_MODE == TRUE
    case DAC_DHRM_12BIT_RIGHT_DUAL:

      /* Two channels as 2 x dacsample_t in a word. GPDMA count is 4 bytes per
       transfer.*/
      nch = 2U;
      dacreg = &dacp->params->dac->DHR12RD;
      dmamode = (STM32_DMA3_CTR1_DDW_WORD | STM32_DMA3_CTR1_SDW_HALF);
      mult = HALF_DUAL_SAMPLE_MULTIPLIER;

      /* Get initial value of channels.*/
      chx = (dacsample_t)*dacp->samples;
      ch2 = (dacsample_t)*(dacp->samples + 2);
      break;

    case DAC_DHRM_12BIT_LEFT_DUAL:

      /* Two channels as 2 x dacsample_t per word. GPDMA count is 4 bytes per
       transfer.*/
      nch = 2U;
      dacreg = &dacp->params->dac->DHR12LD;
      dmamode = (STM32_DMA3_CTR1_DDW_WORD | STM32_DMA3_CTR1_SDW_HALF);
      mult = HALF_DUAL_SAMPLE_MULTIPLIER;

      /* Get initial value of channels.*/
      chx = (dacsample_t)*dacp->samples;
      ch2 = (dacsample_t)*(dacp->samples + 2);
      break;

    case DAC_DHRM_8BIT_RIGHT_DUAL:

      /* Two channels packed as two bytes in a single dacsample_t. GPDMA count
       is 2 bytes per transfer.*/
      nch = 2U;
      dacreg = &dacp->params->dac->DHR8RD;
      dmamode = (STM32_DMA3_CTR1_DDW_WORD | STM32_DMA3_CTR1_SDW_BYTE);
      mult = BYTE_DUAL_SAMPLE_MULTIPLIER;

      /* Get initial value of channels.*/
      chx = (dacsample_t)(*dacp->samples & 0xFF);
      ch2 = (dacsample_t)(*(dacp->samples) >> 8);
      break;

#endif /* STM32_DAC_DUAL_MODE == TRUE */
    default:
      osalDbgAssert(false, "dual mode not enabled or invalid register identity");
      return HAL_RET_CONFIG_ERROR;
  } /* End switch.*/

  /* Check configuration and setup DMA.*/
  if (dacp->grpp->num_channels != nch) {
    osalDbgAssert(false, "invalid number of channels");
    return HAL_RET_CONFIG_ERROR;
  }

  /* Double DMA is supported on single channel only in dual mode.*/
  if (dacddma && nch == 2) {
    osalDbgAssert(false, "double DMA mode not supported in 2 channel dual mode");
    return HAL_RET_CONFIG_ERROR;
  }

  /* Calculate count of GPDMA byte transfers.*/
  n = dacp->depth * mult;

  /* Adjust multiplier for double DMA mode.*/
  mult *= dacddma ? 2 : 1;

  /* A depth of one just repeats one sample. Otherwise Adjust count and
     source address for first cycle of DMA.*/
  if (dacp->depth == 1) {
    ni = n;
    si = (uint8_t *)dacp->samples;
  }
  else {
    ni = n - mult;
    si = (uint8_t *)dacp->samples + mult;
  }

  if (n > STM32_DMA3_MAX_TRANSFER) {
    osalDbgAssert(false, "unsupported GPDMA transfer size");
    return HAL_RET_CONFIG_ERROR;
  }

  /* Allocate GPDMA channel.*/
  dacp->dmachp = dma3ChannelAllocI(dacp->params->dmach,
                                    dacp->params->dmairqprio,
                                    dac_lld_serve_dma_interrupt,
                                    (void *)dacp);

  if (dacp->dmachp == NULL) {
    return HAL_RET_NO_RESOURCE;
  }

  /* Set DAC target register for GPDMA.*/
  dma3ChannelSetDestination(dacp->dmachp, dacreg);

  /* Setup DMA control registers values.*/
  dmaccr = STM32_DMA3_CCR_PRIO((uint32_t)dacp->params->dmaprio)     |
           STM32_DMA3_CCR_LAP_MEM                                   |
           STM32_DMA3_CCR_TOIE                                      |
           STM32_DMA3_CCR_USEIE                                     |
           STM32_DMA3_CCR_ULEIE                                     |
           STM32_DMA3_CCR_DTEIE                                     |
           STM32_DMA3_CCR_TCIE;

  /* DAC uses a circular operation. Use the GPDMA linking mechanism to reload
     source pointer and count for subsequent cycles.*/
  dmallr = STM32_DMA3_CLLR_USA | STM32_DMA3_CLLR_UB1 |
              (((uint32_t)dacp->dbuf) & 0xFFFFU);
  dacp->dbuf->cb1r = n;
  dacp->dbuf->csar = (uint32_t)dacp->samples;

  if (n > 1U) {
    /* If circular buffer depth > 1, then the half transfer interrupt
       is enabled in order to allow streaming processing.*/
    dmaccr |= STM32_DMA3_CCR_HTIE;
  }

  /* Configure and enable GPDMA controller with initial transfer settings.*/
  dma3ChannelSetSource(dacp->dmachp, si);
  dma3ChannelSetTransactionSize(dacp->dmachp, ni);
  dma3ChannelSetMode(dacp->dmachp,
                      dmaccr,
                      (dmamode                                        |
                       STM32_DMA3_CTR1_SAP_MEM                        |
                       STM32_DMA3_CTR1_SINC                           |
                       STM32_DMA3_CTR1_DAP_PER),
                      (STM32_DMA3_CTR2_REQSEL(dacp->params->dmareq)   |
                       STM32_DMA3_CTR2_DREQ),
                       dmallr);

  dma3ChannelEnable(dacp->dmachp);

  /* DAC configuration.*/
  cr = dacp->params->dac->CR;


#if STM32_DAC_DUAL_MODE == FALSE
  (void) ch2;

  /* Operating in single mode. Disable channel.*/
  dacp->params->dac->CR &= ~(DAC_CR_EN1 << dacp->params->regshift);

  /* Wait for channel to disable.*/
  while ((dacp->params->dac->SR &
                  (DAC_SR_DAC1RDY << dacp->params->regshift)) != 0);

  if (dacddma) {
    /* Enable DMA double mode now so DORB values will be written.*/
    dacp->params->dac->MCR |= DAC_MCR_DMADOUBLE1 << dacp->params->regshift;
  }

  /* Set initial value of channel holding register(s).*/
  dacchannel_t ch_num = dacp->params->regshift == 0 ? 0U : 1U;
  (void) put_channel(dacp, ch_num, chx);

  /* Enable DMA and trigger on the specified channel. Clear under-run status.*/
  cr &= dacp->params->regmask;
  cr |= (DAC_CR_DMAEN1 | (dacp->grpp->trigger << DAC_CR_TSEL1_Pos) |
         DAC_CR_TEN1 | dacp->config->cr) << dacp->params->regshift;
  dacp->params->dac->SR = (DAC_SR_DMAUDR1 << dacp->params->regshift);

  /* Setup trigger and DMA.*/
  dacp->params->dac->CR = cr;
  cr |= DAC_CR_EN1 << dacp->params->regshift;

#else /* !STM32_DAC_DUAL_MODE == FALSE */

  /* Operating in dual mode. Disable CH1.*/
  dacp->params->dac->CR &= ~DAC_CR_EN1;

  /* Wait for channel to disable.*/
  while ((dacp->params->dac->SR & DAC_SR_DAC1RDY) != 0);

  if (dacddma) {
    /* Enable DMA double mode now so DORB values will be written.*/
    dacp->params->dac->MCR |= DAC_MCR_DMADOUBLE1;
  }

  /* Set initial value of DHR/DHRB register(s).*/
  (void) put_channel(dacp, 0U, chx);
  if (nch == 2) {
    (void) put_channel(dacp,1U, ch2);
  }

  /* Set trigger for CH1 group.*/
  cr = DAC_CR_DMAEN1 | (dacp->grpp->trigger << DAC_CR_TSEL1_Pos) |
       DAC_CR_TEN1 | dacp->config->cr;
  dacp->params->dac->SR = DAC_SR_DMAUDR1;

  /* Setup trigger and DMA.*/
  dacp->params->dac->CR = cr;
  cr |= DAC_CR_EN1 | DAC_CR_EN2;
#endif

  /* Start continuous conversion.*/
  dacp->params->dac->CR = cr;
  return HAL_RET_SUCCESS;

}

/**
 * @brief   Stops an ongoing conversion.
 * @details This function stops the currently ongoing conversion. The
 *          configuration is restored to start condition. The DOR values
 *          are not updated.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 *
 * @iclass
 */
void dac_lld_stop_conversion(DACDriver *dacp) {
  uint32_t cr, mcr;

  /* GPDMA channel disabled and released.*/
  dma3ChannelDisable(dacp->dmachp);
  dma3ChannelFreeI(dacp->dmachp);
  dacp->dmachp = NULL;

  /* Get current CR and SR.*/
  cr = dacp->params->dac->CR;
#if STM32_DAC_DUAL_MODE == FALSE

  /* Operating in single mode. Disable channel.*/
  dacp->params->dac->CR &= ~(DAC_CR_EN1 << dacp->params->regshift);

  /* Wait for channel to disable.*/
  while ((dacp->params->dac->SR &
                  (DAC_SR_DAC1RDY << dacp->params->regshift)) != 0);

  /* Reset MCR of this channel, restore config mode and retain HFSEL.*/
  mcr = dacp->params->dac->MCR &
              (dacp->params->regmask | DAC_MCR_HFSEL_0 | DAC_MCR_HFSEL_1);
  mcr |= ((dacp->config->mcr & dacp->params->regmask) << dacp->params->regshift);

  /* Disable double DMA setting so DOR is update target.*/
  mcr &= ~(DAC_MCR_DMADOUBLE1 << dacp->params->regshift);

  dacp->params->dac->MCR = mcr;

  /* Restore CR start settings before re-enabling channel.*/
  cr &= dacp->params->regmask;
  cr |= (dacp->config->cr & CONFIG_SINGLE_MASK) << dacp->params->regshift;

  /* Mask out enable then write CR start settings.*/
  cr &= ~(DAC_CR_EN1 << dacp->params->regshift);
  dacp->params->dac->CR = cr;

  /* Setup enable.*/
  cr |= DAC_CR_EN1 << dacp->params->regshift;

#else /* !STM32_DAC_DUAL_MODE == FALSE */

  /* Operating in dual mode. Disable CH1.*/
  dacp->params->dac->CR &= ~DAC_CR_EN1;

  /* Wait for channel to disable.*/
  while ((dacp->params->dac->SR & DAC_SR_DAC1RDY) != 0);

  /* Reset MCR of CH1, restore config mode and retain HFSEL.*/
  mcr = dacp->params->dac->MCR &
              (dacp->params->regmask | DAC_MCR_HFSEL_0 | DAC_MCR_HFSEL_1);
  mcr |= (dacp->config->mcr & CONFIG_SINGLE_MASK);

  /* Disable double DMA setting so DOR is update target.*/
  mcr &= ~DAC_MCR_DMADOUBLE1;
  dacp->params->dac->MCR = mcr;

  /* Restore CR start settings before enabling channel.*/
  cr &= dacp->params->regmask;
  cr |= (dacp->config->cr & CONFIG_SINGLE_MASK);

  /* Mask out enable then write CR start settings.*/
  cr &= ~(DAC_CR_EN1);
  dacp->params->dac->CR = cr;

  /* Setup enable.*/
  cr |= DAC_CR_EN1;

#endif /* STM32_DAC_DUAL_MODE == FALSE */

  /* Re-enable channel.*/
  dacp->params->dac->CR = cr;
}

/**
 * @brief   DAC IRQ service routine.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 * @param[in] isr       content of the ISR register
 *
 * @isr
 */
void dac_lld_serve_interrupt(DACDriver *dacp) {

  /* Check for DMA underrun, the error is only handled if the driver is in
     DAC_ACTIVE state.*/
  if (dacp->state == DAC_ACTIVE) {
    /* DAC DMA underrun condition. This can happen only if the DMA is
       unable to read data fast enough.*/
    _dac_isr_error_code(dacp, DAC_ERR_UNDERFLOW);
  }
}

#endif /* HAL_USE_DAC */

/** @} */
