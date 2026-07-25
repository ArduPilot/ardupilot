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
 * @file    DACv1/hal_dac_lld.c
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

/* Because ST headers naming inconsistencies.*/
#if !defined(DAC1)
#define DAC1 DAC
#endif

#define DAC1_CH1_DMA_CHANNEL                                                \
  STM32_DMA_GETCHANNEL(STM32_DAC_DAC1_CH1_DMA_STREAM,                       \
                       STM32_DAC1_CH1_DMA_CHN)

#define DAC1_CH2_DMA_CHANNEL                                                \
  STM32_DMA_GETCHANNEL(STM32_DAC_DAC1_CH2_DMA_STREAM,                       \
                       STM32_DAC1_CH2_DMA_CHN)

#define DAC2_CH1_DMA_CHANNEL                                                \
  STM32_DMA_GETCHANNEL(STM32_DAC_DAC2_CH1_DMA_STREAM,                       \
                       STM32_DAC2_CH1_DMA_CHN)

#define DAC2_CH2_DMA_CHANNEL                                                \
  STM32_DMA_GETCHANNEL(STM32_DAC_DAC2_CH2_DMA_STREAM,                       \
                       STM32_DAC2_CH2_DMA_CHN)

#define DAC3_CH1_DMA_CHANNEL                                                \
  STM32_DMA_GETCHANNEL(STM32_DAC_DAC3_CH1_DMA_STREAM,                       \
                       STM32_DAC3_CH1_DMA_CHN)

#define DAC3_CH2_DMA_CHANNEL                                                \
  STM32_DMA_GETCHANNEL(STM32_DAC_DAC3_CH2_DMA_STREAM,                       \
                       STM32_DAC3_CH2_DMA_CHN)

#define DAC4_CH1_DMA_CHANNEL                                                \
  STM32_DMA_GETCHANNEL(STM32_DAC_DAC4_CH1_DMA_STREAM,                       \
                       STM32_DAC4_CH1_DMA_CHN)

#define DAC4_CH2_DMA_CHANNEL                                                \
  STM32_DMA_GETCHANNEL(STM32_DAC_DAC4_CH2_DMA_STREAM,                       \
                       STM32_DAC4_CH2_DMA_CHN)

#define CHANNEL_DATA_OFFSET     3U
#define CHANNEL_REGISTER_SHIFT  16U
#define CHANNEL_REGISTER_MASK1  0xFFFF0000U
#define CHANNEL_REGISTER_MASK2  0x0000FFFFU
#define CONFIG_SINGLE_MASK      0x0000FFFFU

#define HF_SEL_AHB_GT_80MHZ     80000000U
#define HF_SEL_AHB_GT_160MHZ    160000000U

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief DAC1 CH1 driver identifier.*/
#if STM32_DAC_USE_DAC1_CH1 || defined(__DOXYGEN__)
DACDriver DACD1;
#endif

/** @brief DAC1 CH2 driver identifier.*/
#if (STM32_DAC_USE_DAC1_CH2 && !STM32_DAC_DUAL_MODE) || defined(__DOXYGEN__)
DACDriver DACD2;
#endif

/** @brief DAC2 CH1 driver identifier.*/
#if STM32_DAC_USE_DAC2_CH1 || defined(__DOXYGEN__)
DACDriver DACD3;
#endif

/** @brief DAC2 CH2 driver identifier.*/
#if (STM32_DAC_USE_DAC2_CH2 && !STM32_DAC_DUAL_MODE) || defined(__DOXYGEN__)
DACDriver DACD4;
#endif

/** @brief DAC3 CH1 driver identifier.*/
#if STM32_DAC_USE_DAC3_CH1 || defined(__DOXYGEN__)
DACDriver DACD5;
#endif

/** @brief DAC3 CH2 driver identifier.*/
#if (STM32_DAC_USE_DAC3_CH2 && !STM32_DAC_DUAL_MODE) || defined(__DOXYGEN__)
DACDriver DACD6;
#endif

/** @brief DAC4 CH1 driver identifier.*/
#if STM32_DAC_USE_DAC4_CH1 || defined(__DOXYGEN__)
DACDriver DACD7;
#endif

/** @brief DAC4 CH2 driver identifier.*/
#if (STM32_DAC_USE_DAC4_CH2 && !STM32_DAC_DUAL_MODE) || defined(__DOXYGEN__)
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
  .dmastream    = STM32_DAC_DAC1_CH1_DMA_STREAM,
#if STM32_DMA_SUPPORTS_DMAMUX
  .peripheral   = STM32_DMAMUX1_DAC1_CH1,
#endif
  .dmamode      = STM32_DMA_CR_CHSEL(DAC1_CH1_DMA_CHANNEL) |
                  STM32_DMA_CR_PL(STM32_DAC_DAC1_CH1_DMA_PRIORITY) |
                  STM32_DMA_CR_MINC | STM32_DMA_CR_CIRC | STM32_DMA_CR_DIR_M2P |
                  STM32_DMA_CR_DMEIE | STM32_DMA_CR_TEIE | STM32_DMA_CR_HTIE |
                  STM32_DMA_CR_TCIE,
  .dmairqprio   = STM32_DAC_DAC1_CH1_IRQ_PRIORITY
};
#endif

#if STM32_DAC_USE_DAC1_CH2 == TRUE
static const dacparams_t dac1_ch2_params = {
  .dac          = DAC1,
  .dataoffset   = CHANNEL_DATA_OFFSET,
  .regshift     = CHANNEL_REGISTER_SHIFT,
  .regmask      = CHANNEL_REGISTER_MASK2,
  .dmastream    = STM32_DAC_DAC1_CH2_DMA_STREAM,
#if STM32_DMA_SUPPORTS_DMAMUX
  .peripheral   = STM32_DMAMUX1_DAC1_CH2,
#endif
  .dmamode      = STM32_DMA_CR_CHSEL(DAC1_CH2_DMA_CHANNEL) |
                  STM32_DMA_CR_PL(STM32_DAC_DAC1_CH2_DMA_PRIORITY) |
                  STM32_DMA_CR_MINC | STM32_DMA_CR_CIRC | STM32_DMA_CR_DIR_M2P |
                  STM32_DMA_CR_DMEIE | STM32_DMA_CR_TEIE | STM32_DMA_CR_HTIE |
                  STM32_DMA_CR_TCIE,
  .dmairqprio   = STM32_DAC_DAC1_CH2_IRQ_PRIORITY
};
#endif

#if STM32_DAC_USE_DAC2_CH1 == TRUE
static const dacparams_t dac2_ch1_params = {
  .dac          = DAC2,
  .dataoffset   = 0U,
  .regshift     = 0U,
  .regmask      = CHANNEL_REGISTER_MASK1,
  .dmastream    = STM32_DAC_DAC2_CH1_DMA_STREAM,
#if STM32_DMA_SUPPORTS_DMAMUX
  .peripheral   = STM32_DMAMUX1_DAC2_CH1,
#endif
  .dmamode      = STM32_DMA_CR_CHSEL(DAC2_CH1_DMA_CHANNEL) |
                  STM32_DMA_CR_PL(STM32_DAC_DAC2_CH1_DMA_PRIORITY) |
                  STM32_DMA_CR_MINC | STM32_DMA_CR_CIRC | STM32_DMA_CR_DIR_M2P |
                  STM32_DMA_CR_DMEIE | STM32_DMA_CR_TEIE | STM32_DMA_CR_HTIE |
                  STM32_DMA_CR_TCIE,
  .dmairqprio   = STM32_DAC_DAC2_CH1_IRQ_PRIORITY
};
#endif

#if STM32_DAC_USE_DAC2_CH2 == TRUE
static const dacparams_t dac2_ch2_params = {
  .dac          = DAC2,
  .dataoffset   = CHANNEL_DATA_OFFSET,
  .regshift     = CHANNEL_REGISTER_SHIFT,
  .regmask      = CHANNEL_REGISTER_MASK2,
  .dmastream    = STM32_DAC_DAC2_CH2_DMA_STREAM,
#if STM32_DMA_SUPPORTS_DMAMUX
  .peripheral   = STM32_DMAMUX1_DAC2_CH2,
#endif
  .dmamode      = STM32_DMA_CR_CHSEL(DAC2_CH2_DMA_CHANNEL) |
                  STM32_DMA_CR_PL(STM32_DAC_DAC2_CH2_DMA_PRIORITY) |
                  STM32_DMA_CR_MINC | STM32_DMA_CR_CIRC | STM32_DMA_CR_DIR_M2P |
                  STM32_DMA_CR_DMEIE | STM32_DMA_CR_TEIE | STM32_DMA_CR_HTIE |
                  STM32_DMA_CR_TCIE,
  .dmairqprio   = STM32_DAC_DAC2_CH2_IRQ_PRIORITY
};
#endif

#if STM32_DAC_USE_DAC3_CH1 == TRUE
static const dacparams_t dac3_ch1_params = {
  .dac          = DAC3,
  .dataoffset   = 0U,
  .regshift     = 0U,
  .regmask      = CHANNEL_REGISTER_MASK1,
  .dmastream    = STM32_DAC_DAC3_CH1_DMA_STREAM,
#if STM32_DMA_SUPPORTS_DMAMUX
  .peripheral   = STM32_DMAMUX1_DAC3_CH1,
#endif
  .dmamode      = STM32_DMA_CR_CHSEL(DAC3_CH1_DMA_CHANNEL) |
                  STM32_DMA_CR_PL(STM32_DAC_DAC3_CH1_DMA_PRIORITY) |
                  STM32_DMA_CR_MINC | STM32_DMA_CR_CIRC | STM32_DMA_CR_DIR_M2P |
                  STM32_DMA_CR_DMEIE | STM32_DMA_CR_TEIE | STM32_DMA_CR_HTIE |
                  STM32_DMA_CR_TCIE,
  .dmairqprio   = STM32_DAC_DAC3_CH1_IRQ_PRIORITY
};
#endif

#if STM32_DAC_USE_DAC3_CH2 == TRUE
static const dacparams_t dac3_ch2_params = {
  .dac          = DAC3,
  .dataoffset   = CHANNEL_DATA_OFFSET,
  .regshift     = CHANNEL_REGISTER_SHIFT,
  .regmask      = CHANNEL_REGISTER_MASK2,
  .dmastream    = STM32_DAC_DAC3_CH2_DMA_STREAM,
#if STM32_DMA_SUPPORTS_DMAMUX
  .peripheral   = STM32_DMAMUX1_DAC3_CH2,
#endif
  .dmamode      = STM32_DMA_CR_CHSEL(DAC3_CH2_DMA_CHANNEL) |
                  STM32_DMA_CR_PL(STM32_DAC_DAC3_CH2_DMA_PRIORITY) |
                  STM32_DMA_CR_MINC | STM32_DMA_CR_CIRC | STM32_DMA_CR_DIR_M2P |
                  STM32_DMA_CR_DMEIE | STM32_DMA_CR_TEIE | STM32_DMA_CR_HTIE |
                  STM32_DMA_CR_TCIE,
  .dmairqprio   = STM32_DAC_DAC3_CH2_IRQ_PRIORITY
};
#endif

#if STM32_DAC_USE_DAC4_CH1 == TRUE
static const dacparams_t dac4_ch1_params = {
  .dac          = DAC4,
  .dataoffset   = 0U,
  .regshift     = 0U,
  .regmask      = CHANNEL_REGISTER_MASK1,
  .dmastream    = STM32_DAC_DAC4_CH1_DMA_STREAM,
#if STM32_DMA_SUPPORTS_DMAMUX
  .peripheral   = STM32_DMAMUX1_DAC4_CH1,
#endif
  .dmamode      = STM32_DMA_CR_CHSEL(DAC4_CH1_DMA_CHANNEL) |
                  STM32_DMA_CR_PL(STM32_DAC_DAC4_CH1_DMA_PRIORITY) |
                  STM32_DMA_CR_MINC | STM32_DMA_CR_CIRC | STM32_DMA_CR_DIR_M2P |
                  STM32_DMA_CR_DMEIE | STM32_DMA_CR_TEIE | STM32_DMA_CR_HTIE |
                  STM32_DMA_CR_TCIE,
  .dmairqprio   = STM32_DAC_DAC4_CH1_IRQ_PRIORITY
};
#endif

#if STM32_DAC_USE_DAC4_CH2 == TRUE
static const dacparams_t dac4_ch2_params = {
  .dac          = DAC4,
  .dataoffset   = CHANNEL_DATA_OFFSET,
  .regshift     = CHANNEL_REGISTER_SHIFT,
  .regmask      = CHANNEL_REGISTER_MASK2,
  .dmastream    = STM32_DAC_DAC4_CH2_DMA_STREAM,
#if STM32_DMA_SUPPORTS_DMAMUX
  .peripheral   = STM32_DMAMUX1_DAC4_CH2,
#endif
  .dmamode      = STM32_DMA_CR_CHSEL(DAC4_CH2_DMA_CHANNEL) |
                  STM32_DMA_CR_PL(STM32_DAC_DAC4_CH2_DMA_PRIORITY) |
                  STM32_DMA_CR_MINC | STM32_DMA_CR_CIRC | STM32_DMA_CR_DIR_M2P |
                  STM32_DMA_CR_DMEIE | STM32_DMA_CR_TEIE | STM32_DMA_CR_HTIE |
                  STM32_DMA_CR_TCIE,
  .dmairqprio   = STM32_DAC_DAC4_CH2_IRQ_PRIORITY
};
#endif

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Shared end/half-of-tx service routine.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 * @param[in] flags     pre-shifted content of the ISR register
 */
static void dac_lld_serve_tx_interrupt(DACDriver *dacp, uint32_t flags) {

  if ((flags & (STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF)) != 0) {
    /* DMA errors handling.*/
    dac_lld_stop_conversion(dacp);
    _dac_isr_error_code(dacp, DAC_ERR_DMAFAILURE);
  }
  else {
    if ((flags & STM32_DMA_ISR_HTIF) != 0) {
      /* Half transfer processing.*/
      _dac_isr_half_code(dacp);
    }
    if ((flags & STM32_DMA_ISR_TCIF) != 0) {
      /* Transfer complete processing.*/
      _dac_isr_full_code(dacp);
    }
  }
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

#if STM32_DAC_USE_DAC1_CH1
  dacObjectInit(&DACD1);
  DACD1.params  = &dac1_ch1_params;
  DACD1.dma = NULL;
#endif

#if STM32_DAC_USE_DAC1_CH2
  dacObjectInit(&DACD2);
  DACD2.params  = &dac1_ch2_params;
  DACD2.dma = NULL;
#endif

#if STM32_DAC_USE_DAC2_CH1
  dacObjectInit(&DACD3);
  DACD3.params  = &dac2_ch1_params;
  DACD3.dma = NULL;
#endif

#if STM32_DAC_USE_DAC2_CH2
  dacObjectInit(&DACD4);
  DACD4.params  = &dac2_ch2_params;
  DACD4.dma = NULL;
#endif

#if STM32_DAC_USE_DAC3_CH1
  dacObjectInit(&DACD5);
  DACD5.params  = &dac3_ch1_params;
  DACD5.dma = NULL;
#endif

#if STM32_DAC_USE_DAC3_CH2
  dacObjectInit(&DACD6);
  DACD6.params  = &dac3_ch2_params;
  DACD6.dma = NULL;
#endif

#if STM32_DAC_USE_DAC4_CH1
  dacObjectInit(&DACD7);
  DACD7.params  = &dac4_ch1_params;
  DACD7.dma = NULL;
#endif

#if STM32_DAC_USE_DAC4_CH2
  dacObjectInit(&DACD8);
  DACD8.params  = &dac4_ch2_params;
  DACD8.dma = NULL;
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
     required.*/
  if (dacp->state == DAC_STOP) {
    dacchannel_t channel = 0;

    /* Enabling the clock source.*/
#if STM32_DAC_USE_DAC1_CH1
    if (&DACD1 == dacp) {
      rccEnableDAC1(true);
    }
#endif

#if STM32_DAC_USE_DAC1_CH2
    if (&DACD2 == dacp) {
      rccEnableDAC1(true);
      channel = 1;
    }
#endif

#if STM32_DAC_USE_DAC2_CH1
    if (&DACD3 == dacp) {
      rccEnableDAC2(true);
    }
#endif

#if STM32_DAC_USE_DAC2_CH2
    if (&DACD4 == dacp) {
      rccEnableDAC2(true);
      channel = 1;
    }
#endif

#if STM32_DAC_USE_DAC3_CH1
    if (&DACD5 == dacp) {
      rccEnableDAC3(true);
    }
#endif

#if STM32_DAC_USE_DAC3_CH2
    if (&DACD6 == dacp) {
      rccEnableDAC3(true);
      channel = 1;
    }
#endif

#if STM32_DAC_USE_DAC4_CH1
    if (&DACD7 == dacp) {
      rccEnableDAC4(true);
    }
#endif

#if STM32_DAC_USE_DAC4_CH2
    if (&DACD8 == dacp) {
      rccEnableDAC4(true);
      channel = 1;
    }
#endif
    uint32_t reg;
#if STM32_DAC_DUAL_MODE == FALSE
    /* Enabling DAC in SW triggering mode initially, initializing data to
       configuration default.*/
    {

      /* Operating in SINGLE mode with one channel to set. Set registers for
         specified channel from configuration. Lower half word of
         configuration specifies configuration for any channel.*/
#if STM32_DAC_HAS_MCR == TRUE
      reg = dacp->params->dac->MCR & dacp->params->regmask;
#if defined(DAC_MCR_HFSEL)

      /* Handle HFSEL setting based on DAC clock.*/
      reg &= ~(DAC_MCR_HFSEL_0 | DAC_MCR_HFSEL_1);
      if (STM32_SYSCLK > HF_SEL_AHB_GT_160MHZ) {
        reg |= DAC_MCR_HFSEL_1;
      }
      else if (STM32_SYSCLK > HF_SEL_AHB_GT_80MHZ) {
        reg |= DAC_MCR_HFSEL_0;
      }
#endif
      dacp->params->dac->MCR = reg |
        ((dacp->config->mcr & CONFIG_SINGLE_MASK) << dacp->params->regshift);
#endif
      /* Enable and initialise the channel.*/
      reg = dacp->params->dac->CR;
      reg &= dacp->params->regmask;
      reg |= (DAC_CR_EN1 | (dacp->config->cr & CONFIG_SINGLE_MASK)) <<
                                                dacp->params->regshift;
      dacp->params->dac->CR = reg;
      dac_lld_put_channel(dacp, channel, (dacsample_t)dacp->config->init);
    }
#else /* STM32_DAC_DUAL_MODE != FALSE */
    /* Operating in DUAL mode with two channels to setup. Set registers for
       both channels from configuration. Lower and upper half words specify
       configuration for channels CH1 & CH2 respectively.*/
    (void)channel;
#if STM32_DAC_HAS_MCR == TRUE
    reg = (dacp->params->dac->MCR & dacp->params->regmask);
#if defined(DAC_MCR_HFSEL)
    /* Handle HFSEL setting based on DAC clock.*/
    reg = (dacp->params->dac->MCR | dacp->config->mcr) &
                      ~(DAC_MCR_HFSEL_0 | DAC_MCR_HFSEL_1);
    if (STM32_SYSCLK > HF_SEL_AHB_GT_160MHZ) {
      reg |= DAC_MCR_HFSEL_1;
    }
    else if (STM32_SYSCLK > HF_SEL_AHB_GT_80MHZ) {
      reg |= DAC_MCR_HFSEL_0;
    }
#endif
    dacp->params->dac->MCR = reg;
#endif
    /* Enable and initialise both CH1 and CH2. Mask out DMA enable.*/
    reg = dacp->config->cr;
    reg &= ~(DAC_CR_DMAEN1 | DAC_CR_DMAEN2);
    dacp->params->dac->CR = DAC_CR_EN2 | DAC_CR_EN1 | reg;
    dac_lld_put_channel(dacp, 0U, (dacsample_t)dacp->config->init);
    dac_lld_put_channel(dacp, 1U, (dacsample_t)(dacp->config->init >>
                                               (sizeof(dacsample_t) * 8)));
#endif /* STM32_DAC_DUAL_MODE == FALSE */
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

  /* If in ready state then disables the DAC clock.*/
  if (dacp->state == DAC_READY) {

    /* Disabling DAC.*/
    dacp->params->dac->CR &= dacp->params->regmask;

#if STM32_DAC_USE_DAC1_CH1
    if (&DACD1 == dacp) {
#if defined(DAC_CR_EN2)
      if ((dacp->params->dac->CR & DAC_CR_EN2) == 0U) {
        rccDisableDAC1();
      }
#else
      rccDisableDAC1();
#endif
    }
#endif

#if STM32_DAC_USE_DAC1_CH2
    if (&DACD2 == dacp) {
      if ((dacp->params->dac->CR & DAC_CR_EN1) == 0U) {
        rccDisableDAC1();
      }
    }
#endif

#if STM32_DAC_USE_DAC2_CH1
    if (&DACD3 == dacp) {
#if defined(DAC_CR_EN2)
      if ((dacp->params->dac->CR & DAC_CR_EN2) == 0U) {
        rccDisableDAC2();
      }
#else
      rccDisableDAC2();
#endif
    }
#endif

#if STM32_DAC_USE_DAC2_CH2
    if (&DACD4 == dacp) {
      if ((dacp->params->dac->CR & DAC_CR_EN1) == 0U) {
        rccDisableDAC2();
      }
    }
#endif

#if STM32_DAC_USE_DAC3_CH1
    if (&DACD5 == dacp) {
#if defined(DAC_CR_EN2)
      if ((dacp->params->dac->CR & DAC_CR_EN2) == 0U) {
        rccDisableDAC3();
      }
#else
      rccDisableDAC3();
#endif
    }
#endif

#if STM32_DAC_USE_DAC3_CH2
    if (&DACD6 == dacp) {
      if ((dacp->params->dac->CR & DAC_CR_EN1) == 0U) {
        rccDisableDAC3();
      }
    }
#endif

#if STM32_DAC_USE_DAC4_CH1
    if (&DACD7 == dacp) {
#if defined(DAC_CR_EN2)
      if ((dacp->params->dac->CR & DAC_CR_EN2) == 0U) {
        rccDisableDAC4();
      }
#else
      rccDisableDAC4();
#endif
    }
#endif

#if STM32_DAC_USE_DAC4_CH2
    if (&DACD8 == dacp) {
      if ((dacp->params->dac->CR & DAC_CR_EN1) == 0U) {
        rccDisableDAC4();
      }
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
 * @api
 */
void dac_lld_put_channel(DACDriver *dacp,
                         dacchannel_t channel,
                         dacsample_t sample) {

#if STM32_DAC_DUAL_MODE
  if (dacp->grpp != NULL) {
    osalDbgAssert(dacp->grpp->num_channels == 1 && channel == 1,
                                          "channel busy");
  }
#endif /* STM32_DAC_DUAL_MODE */

  switch (dacp->config->datamode) {
  case DAC_DHRM_12BIT_RIGHT:
#if STM32_DAC_DUAL_MODE
  case DAC_DHRM_12BIT_RIGHT_DUAL:
#endif
    if (channel == 0U) {
#if STM32_DAC_DUAL_MODE
      dacp->params->dac->DHR12R1 = (uint32_t)sample;
#else
      *(&dacp->params->dac->DHR12R1 + dacp->params->dataoffset) = (uint32_t)sample;
#endif
    }
#if (STM32_HAS_DAC1_CH2 || STM32_HAS_DAC2_CH2 ||                            \
     STM32_HAS_DAC3_CH2 || STM32_HAS_DAC4_CH2)
    else {
      dacp->params->dac->DHR12R2 = (uint32_t)sample;
    }
#endif
    break;
  case DAC_DHRM_12BIT_LEFT:
#if STM32_DAC_DUAL_MODE
  case DAC_DHRM_12BIT_LEFT_DUAL:
#endif
    if (channel == 0U) {
#if STM32_DAC_DUAL_MODE
      dacp->params->dac->DHR12L1 = (uint32_t)sample;
#else
      *(&dacp->params->dac->DHR12L1 + dacp->params->dataoffset) = (uint32_t)sample;
#endif
    }
#if (STM32_HAS_DAC1_CH2 || STM32_HAS_DAC2_CH2 ||                            \
     STM32_HAS_DAC3_CH2 || STM32_HAS_DAC4_CH2)
    else {
      dacp->params->dac->DHR12L2 = (uint32_t)sample;
    }
#endif
    break;
  case DAC_DHRM_8BIT_RIGHT:
#if STM32_DAC_DUAL_MODE
  case DAC_DHRM_8BIT_RIGHT_DUAL:
#endif
    if (channel == 0U) {
#if STM32_DAC_DUAL_MODE
      dacp->params->dac->DHR8R1 = (uint32_t)sample;
#else
      *(&dacp->params->dac->DHR8R1 + dacp->params->dataoffset) = (uint32_t)sample;
#endif
    }
#if (STM32_HAS_DAC1_CH2 || STM32_HAS_DAC2_CH2 ||                            \
     STM32_HAS_DAC3_CH2 || STM32_HAS_DAC4_CH2)
    else {
      dacp->params->dac->DHR8R2 = (uint32_t)sample;
    }
#endif
    break;
  default:
    osalDbgAssert(false, "unexpected DAC mode");
    break;
  }
}

/**
 * @brief   Starts a DAC conversion.
 * @details Starts an asynchronous conversion operation.
 * @note    In @p DAC_DHRM_8BIT_RIGHT mode the parameters passed to the
 *          callback are wrong because two samples are packed in a single
 *          dacsample_t element. This will not be corrected, do not rely
 *          on those parameters.
 * @note    In @p DAC_DHRM_8BIT_RIGHT_DUAL mode two samples are treated
 *          as a single 16 bits sample and packed into a single dacsample_t
 *          element. The num_channels must be set to one in the group
 *          conversion configuration structure.
 * @note    If using DUAL mode with a single channel conversion then CH2
 *          is enabled for manual (put_channel) for non DMA triggered use. The
 *          the data format for put operations is specified in the upper half
 *          word of the 'datamode' field. The CR setting is in the upper half
 *          word of the 'cr' field of the configuration.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 *
 * @notapi
 */
void dac_lld_start_conversion(DACDriver *dacp) {
  uint32_t n, cr, dmamode;

  /* Number of DMA operations per buffer.*/
  n = dacp->depth * dacp->grpp->num_channels;

  /* Allocating the DMA channel.*/
  dacp->dma = dmaStreamAllocI(dacp->params->dmastream,
                              dacp->params->dmairqprio,
                              (stm32_dmaisr_t)dac_lld_serve_tx_interrupt,
                              (void *)dacp);
  osalDbgAssert(dacp->dma != NULL, "unable to allocate stream");
#if STM32_DMA_SUPPORTS_DMAMUX
  dmaSetRequestSource(dacp->dma, dacp->params->peripheral);
#endif

  /* DMA settings depend on the chosen DAC mode.*/
  switch (dacp->config->datamode) {
  /* Sets the DAC data register */
  case DAC_DHRM_12BIT_RIGHT:
    osalDbgAssert(dacp->grpp->num_channels == 1, "invalid number of channels");

    dmaStreamSetPeripheral(dacp->dma, &dacp->params->dac->DHR12R1 +
                                      dacp->params->dataoffset);
    dmamode = dacp->params->dmamode |
#if STM32_DMA_ADVANCED == FALSE
              STM32_DMA_CR_PSIZE_WORD  | STM32_DMA_CR_MSIZE_HWORD;
#else
              STM32_DMA_CR_PSIZE_HWORD | STM32_DMA_CR_MSIZE_HWORD;
#endif
    break;
  case DAC_DHRM_12BIT_LEFT:
    osalDbgAssert(dacp->grpp->num_channels == 1, "invalid number of channels");

    dmaStreamSetPeripheral(dacp->dma, &dacp->params->dac->DHR12L1 +
                                      dacp->params->dataoffset);
    dmamode = dacp->params->dmamode |
#if STM32_DMA_ADVANCED == FALSE
              STM32_DMA_CR_PSIZE_WORD  | STM32_DMA_CR_MSIZE_HWORD;
#else
              STM32_DMA_CR_PSIZE_HWORD | STM32_DMA_CR_MSIZE_HWORD;
#endif
    break;
  case DAC_DHRM_8BIT_RIGHT:
    osalDbgAssert(dacp->grpp->num_channels == 1, "invalid number of channels");

    dmaStreamSetPeripheral(dacp->dma, &dacp->params->dac->DHR8R1 +
                                      dacp->params->dataoffset);
    dmamode = dacp->params->dmamode |
#if STM32_DMA_ADVANCED == FALSE
              STM32_DMA_CR_PSIZE_WORD  | STM32_DMA_CR_MSIZE_BYTE;
#else
              STM32_DMA_CR_MSIZE_BYTE  | STM32_DMA_CR_MSIZE_BYTE;
#endif

    /* In this mode the size of the buffer is halved because two samples
       packed in a single dacsample_t element.*/
    n = (n + 1) / 2;
    break;
#if STM32_DAC_DUAL_MODE == TRUE
  case DAC_DHRM_12BIT_RIGHT_DUAL:
    osalDbgAssert(dacp->grpp->num_channels == 2, "invalid number of channels");

    dmaStreamSetPeripheral(dacp->dma, &dacp->params->dac->DHR12RD);
    dmamode = dacp->params->dmamode |
              STM32_DMA_CR_PSIZE_WORD | STM32_DMA_CR_MSIZE_WORD;
    n /= 2;
    break;
  case DAC_DHRM_12BIT_LEFT_DUAL:
    osalDbgAssert(dacp->grpp->num_channels == 2, "invalid number of channels");

    dmaStreamSetPeripheral(dacp->dma, &dacp->params->dac->DHR12LD);
    dmamode = dacp->params->dmamode |
              STM32_DMA_CR_PSIZE_WORD | STM32_DMA_CR_MSIZE_WORD;
    n /= 2;
    break;
  case DAC_DHRM_8BIT_RIGHT_DUAL:
    osalDbgAssert(dacp->grpp->num_channels == 1, "invalid number of channels");

    dmaStreamSetPeripheral(dacp->dma, &dacp->params->dac->DHR8RD);
    dmamode = dacp->params->dmamode |
#if STM32_DMA_ADVANCED == FALSE
              STM32_DMA_CR_PSIZE_WORD  | STM32_DMA_CR_MSIZE_HWORD;
#else
              STM32_DMA_CR_PSIZE_HWORD | STM32_DMA_CR_MSIZE_HWORD;
#endif
    n /= 2;
    break;
#endif
  default:
    osalDbgAssert(false, "unexpected DAC mode");
    return;
  }

  dmaStreamSetMemory0(dacp->dma, dacp->samples);
  dmaStreamSetTransactionSize(dacp->dma, n);
  dmaStreamSetMode(dacp->dma, dmamode            |
                              STM32_DMA_CR_DMEIE | STM32_DMA_CR_TEIE |
                              STM32_DMA_CR_HTIE  | STM32_DMA_CR_TCIE);
  dmaStreamEnable(dacp->dma);

  /* DAC configuration.*/
  cr = dacp->params->dac->CR;
#if STM32_DAC_DUAL_MODE == FALSE
  /* Start the DMA on the single channel.*/
  cr &= dacp->params->regmask;
  cr |= (DAC_CR_DMAEN1 | (dacp->grpp->trigger << DAC_CR_TSEL1_Pos) |
         DAC_CR_TEN1 | DAC_CR_EN1 | (dacp->config->cr & CONFIG_SINGLE_MASK)) <<
                                               dacp->params->regshift;
#else
  /* Enable the DMA operation on CH1.*/
  cr = DAC_CR_DMAEN1 | (dacp->grpp->trigger << DAC_CR_TSEL1_Pos) |
       DAC_CR_TEN1 | DAC_CR_EN1 | dacp->config->cr;
#endif

  dacp->params->dac->CR = cr;
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
  uint32_t cr;

  /* DMA channel disabled and released.*/
  dmaStreamDisable(dacp->dma);
  dmaStreamFreeI(dacp->dma);
  dacp->dma = NULL;

  /* Restore start configuration but leave DORx at current values.*/
  cr = dacp->params->dac->CR;
#if STM32_DAC_DUAL_MODE == FALSE
#if STM32_DAC_HAS_MCR == TRUE
  uint32_t mcr;

  /* Restore MCR start config but retain HFSEL if existing.*/
#if defined(DAC_MCR_HFSEL)
  mcr = dacp->params->dac->MCR &
              (dacp->params->regmask | DAC_MCR_HFSEL_0 | DAC_MCR_HFSEL_1);
#else
  mcr = dacp->params->dac->MCR & dacp->params->regmask;
#endif
  dacp->params->dac->MCR = mcr |
    ((dacp->config->mcr & CONFIG_SINGLE_MASK) << dacp->params->regshift);
#endif
  cr &= dacp->params->regmask;
  cr |= (DAC_CR_EN1 | (dacp->config->cr & CONFIG_SINGLE_MASK) <<
                                      dacp->params->regshift);
#else
#if STM32_DAC_HAS_MCR == TRUE
  uint32_t mcr;

  /* Reset MCR of CH1 but retain HFSEL if existing.*/
#if defined(DAC_MCR_HFSEL)
  mcr = dacp->params->dac->MCR &
              (dacp->params->regmask | DAC_MCR_HFSEL_0 | DAC_MCR_HFSEL_1);
#else
  mcr = dacp->params->dac->MCR & dacp->params->regmask;
#endif

  /* Restore CH1 MCR to start configuration.*/
  mcr |= (dacp->config->mcr & CONFIG_SINGLE_MASK);
  dacp->params->dac->MCR = mcr;
#endif
  cr = dacp->config->cr | DAC_CR_EN1 | DAC_CR_EN2;
#endif /* STM32_DAC_DUAL_MODE == FALSE.*/

  /* Re-enable channel.*/
  dacp->params->dac->CR = cr;
}

#endif /* HAL_USE_DAC */

/** @} */
