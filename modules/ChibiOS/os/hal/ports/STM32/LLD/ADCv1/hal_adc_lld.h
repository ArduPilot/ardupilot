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
 * @file    ADCv1/hal_adc_lld.h
 * @brief   STM32 ADC subsystem low level driver header.
 *
 * @addtogroup ADC
 * @{
 */

#ifndef HAL_ADC_LLD_H
#define HAL_ADC_LLD_H

#if HAL_USE_ADC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Possible ADC errors mask bits.
 * @{
 */
#define ADC_ERR_DMAFAILURE      1U  /**< DMA operations failure.            */
#define ADC_ERR_OVERFLOW        2U  /**< ADC overflow condition.            */
#define ADC_ERR_AWD1            4U  /**< Watchdog triggered.                */
/** @} */

/**
 * @name    Sampling rates
 * @{
 */
#if defined(STM32F0XX) || defined(__DOXYGEN__)
#define ADC_SMPR_SMP_1P5        0U  /**< @brief 14 cycles conversion time   */
#define ADC_SMPR_SMP_7P5        1U  /**< @brief 21 cycles conversion time.  */
#define ADC_SMPR_SMP_13P5       2U  /**< @brief 28 cycles conversion time.  */
#define ADC_SMPR_SMP_28P5       3U  /**< @brief 41 cycles conversion time.  */
#define ADC_SMPR_SMP_41P5       4U  /**< @brief 54 cycles conversion time.  */
#define ADC_SMPR_SMP_55P5       5U  /**< @brief 68 cycles conversion time.  */
#define ADC_SMPR_SMP_71P5       6U  /**< @brief 84 cycles conversion time.  */
#define ADC_SMPR_SMP_239P5      7U  /**< @brief 252 cycles conversion time. */
#elif defined(STM32L0XX)
#define ADC_SMPR_SMP_1P5        0U  /**< @brief 14 cycles conversion time   */
#define ADC_SMPR_SMP_3P5        1U  /**< @brief 16 cycles conversion time.  */
#define ADC_SMPR_SMP_7P5        2U  /**< @brief 20 cycles conversion time.  */
#define ADC_SMPR_SMP_12P5       3U  /**< @brief 25 cycles conversion time.  */
#define ADC_SMPR_SMP_19P5       4U  /**< @brief 31 cycles conversion time.  */
#define ADC_SMPR_SMP_39P5       5U  /**< @brief 52 cycles conversion time.  */
#define ADC_SMPR_SMP_79P5       6U  /**< @brief 92 cycles conversion time.  */
#define ADC_SMPR_SMP_160P5      7U  /**< @brief 173 cycles conversion time. */
#endif
/** @} */

/**
 * @name    CFGR1 register configuration helpers
 * @{
 */
#define ADC_CFGR1_RES_12BIT             (0U << 3U)
#define ADC_CFGR1_RES_10BIT             (1U << 3U)
#define ADC_CFGR1_RES_8BIT              (2U << 3U)
#define ADC_CFGR1_RES_6BIT              (3U << 3U)

#define ADC_CFGR1_EXTSEL_MASK           (15U << 6U)
#define ADC_CFGR1_EXTSEL_SRC(n)         ((n) << 6U)

#define ADC_CFGR1_EXTEN_MASK            (3U << 10U)
#define ADC_CFGR1_EXTEN_DISABLED        (0U << 10U)
#define ADC_CFGR1_EXTEN_RISING          (1U << 10U)
#define ADC_CFGR1_EXTEN_FALLING         (2U << 10U)
#define ADC_CFGR1_EXTEN_BOTH            (3U << 10U)
/** @} */

/**
 * @name    CFGR2 register configuration helpers
 * @{
 */
#define ADC_CFGR2_CKMODE_MASK           (3U << 30U)
#define ADC_CFGR2_CKMODE_ADCCLK         (0U << 30U)
#define ADC_CFGR2_CKMODE_PCLK_DIV2      (1U << 30U)
#define ADC_CFGR2_CKMODE_PCLK_DIV4      (2U << 30U)
#define ADC_CFGR2_CKMODE_PCLK           (3U << 30U)

#if (STM32_ADC_SUPPORTS_OVERSAMPLING == TRUE) || defined(__DOXYGEN__)
#define ADC_CFGR2_OVSR_MASK             (7U << 2U)
#define ADC_CFGR2_OVSR_2X               (0U << 2U)
#define ADC_CFGR2_OVSR_4X               (1U << 2U)
#define ADC_CFGR2_OVSR_8X               (2U << 2U)
#define ADC_CFGR2_OVSR_16X              (3U << 2U)
#define ADC_CFGR2_OVSR_32X              (4U << 2U)
#define ADC_CFGR2_OVSR_64X              (5U << 2U)
#define ADC_CFGR2_OVSR_128X             (6U << 2U)
#define ADC_CFGR2_OVSR_256X             (7U << 2U)

#define ADC_CFGR2_OVSS_MASK             (15 << 5U)
#define ADC_CFGR2_OVSS_SHIFT(n)         ((n) << 5U)
#endif
/** @} */

/**
 * @name    Threashold register initializer
 * @{
 */
#define ADC_TR(low, high)               (((uint32_t)(high) << 16U) |        \
                                         (uint32_t)(low))
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   ADC1 driver enable switch.
 * @details If set to @p TRUE the support for ADC1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_ADC_USE_ADC1) || defined(__DOXYGEN__)
#define STM32_ADC_USE_ADC1                  FALSE
#endif

/**
 * @brief   ADC1 CRFG2 initialization.
 */
#if !defined(STM32_ADC_ADC1_CFGR2) || defined(__DOXYGEN__)
#define STM32_ADC_ADC1_CFGR2               ADC_CFGR2_CKMODE_ADCCLK
#endif

/**
 * @brief   ADC1 DMA priority (0..3|lowest..highest).
 */
#if !defined(STM32_ADC_ADC1_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ADC_ADC1_DMA_PRIORITY         2
#endif

/**
 * @brief   ADC interrupt priority level setting.
 */
#if !defined(STM32_ADC_ADC1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ADC_ADC1_IRQ_PRIORITY         2
#endif

/**
 * @brief   ADC1 DMA interrupt priority level setting.
 */
#if !defined(STM32_ADC_ADC1_DMA_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ADC_ADC1_DMA_IRQ_PRIORITY     2
#endif

#if (STM32_ADC_SUPPORTS_PRESCALER == TRUE) || defined(__DOXYGEN__)
/*
 * @brief   ADC prescaler setting.
 * @note    This setting has effect only in asynchronous clock mode (the
 *          default, @p STM32_ADC_CKMODE_ADCCLK).
 */
#if !defined(STM32_ADC_PRESCALER_VALUE) || defined(__DOXYGEN__)
#define STM32_ADC_PRESCALER_VALUE           2
#endif
#endif

/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Supported devices checks.*/
#if !defined(STM32F0XX) && !defined(STM32L0XX)
#error "ADCv1 only supports F0 and L0 STM32 devices"
#endif

#if defined(STM32L0XX) || defined(__DOXYGEN__)
#define STM32_ADCV1_OVERSAMPLING            TRUE
#else
#define STM32_ADCV1_OVERSAMPLING            FALSE
#endif

/* Registry checks.*/
#if !defined(STM32_HAS_ADC1)
#error "STM32_HAS_ADC1 not defined in registry"
#endif

#if !defined(STM32_ADC_SUPPORTS_PRESCALER)
#error "STM32_ADC_SUPPORTS_PRESCALER not defined in registry"
#endif

#if (STM32_ADC_USE_ADC1 && !defined(STM32_ADC1_HANDLER))
#error "STM32_ADC1_HANDLER not defined in registry"
#endif

#if (STM32_ADC_USE_ADC1 && !defined(STM32_ADC1_NUMBER))
#error "STM32_ADC1_NUMBER not defined in registry"
#endif

#if STM32_ADC_USE_ADC1 && !STM32_HAS_ADC1
#error "ADC1 not present in the selected device"
#endif

/* Units checks.*/
#if STM32_ADC_USE_ADC1 && !STM32_HAS_ADC1
#error "ADC1 not present in the selected device"
#endif

/* At least one ADC must be assigned.*/
#if !STM32_ADC_USE_ADC1
#error "ADC driver activated but no ADC peripheral assigned"
#endif

/* ADC IRQ priority tests.*/
#if STM32_ADC_USE_ADC1 &&                                                   \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_ADC_ADC1_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to ADC1"
#endif

/* DMA IRQ priority tests.*/
#if STM32_ADC_USE_ADC1 &&                                                   \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_ADC_ADC1_DMA_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to ADC1 DMA"
#endif

/* DMA priority tests.*/
#if STM32_ADC_USE_ADC1 &&                                                   \
    !STM32_DMA_IS_VALID_PRIORITY(STM32_ADC_ADC1_DMA_PRIORITY)
#error "Invalid DMA priority assigned to ADC1"
#endif

/* Check on the presence of the DMA streams settings in mcuconf.h.*/
#if STM32_ADC_USE_ADC1 && !defined(STM32_ADC_ADC1_DMA_STREAM)
#error "ADC DMA stream not defined"
#endif
#if STM32_DMA_SUPPORTS_DMAMUX

#else /* !STM32_DMA_SUPPORTS_DMAMUX */

/* Check on the validity of the assigned DMA channels.*/
#if STM32_ADC_USE_ADC1 &&                                                   \
    !STM32_DMA_IS_VALID_ID(STM32_ADC_ADC1_DMA_STREAM, STM32_ADC1_DMA_MSK)
#error "invalid DMA stream associated to ADC1"
#endif

#endif /* !STM32_DMA_SUPPORTS_DMAMUX */

/* ADC clock source checks.*/
#if STM32_ADC_SUPPORTS_PRESCALER == TRUE
#if STM32_ADC_PRESCALER_VALUE == 1
#define STM32_ADC_PRESC                     0U
#elif STM32_ADC_PRESCALER_VALUE == 2
#define STM32_ADC_PRESC                     1U
#elif STM32_ADC_PRESCALER_VALUE == 4
#define STM32_ADC_PRESC                     2U
#elif STM32_ADC_PRESCALER_VALUE == 6
#define STM32_ADC_PRESC                     3U
#elif STM32_ADC_PRESCALER_VALUE == 8
#define STM32_ADC_PRESC                     4U
#elif STM32_ADC_PRESCALER_VALUE == 10
#define STM32_ADC_PRESC                     5U
#elif STM32_ADC_PRESCALER_VALUE == 12
#define STM32_ADC_PRESC                     6U
#elif STM32_ADC_PRESCALER_VALUE == 16
#define STM32_ADC_PRESC                     7U
#elif STM32_ADC_PRESCALER_VALUE == 32
#define STM32_ADC_PRESC                     8U
#elif STM32_ADC_PRESCALER_VALUE == 64
#define STM32_ADC_PRESC                     9U
#elif STM32_ADC_PRESCALER_VALUE == 128
#define STM32_ADC_PRESC                     10U
#elif STM32_ADC_PRESCALER_VALUE == 256
#define STM32_ADC_PRESC                     11U
#else
#error "Invalid value assigned to STM32_ADC_PRESCALER_VALUE"
#endif
#endif

#if !defined(STM32_DMA_REQUIRED)
#define STM32_DMA_REQUIRED
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   ADC sample data type.
 */
typedef uint16_t adcsample_t;

/**
 * @brief   Channels number in a conversion group.
 */
typedef uint16_t adc_channels_num_t;

/**
 * @brief   Type of an ADC error mask.
 */
typedef uint32_t adcerror_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Low level fields of the ADC driver structure.
 */
#define adc_lld_driver_fields                                               \
  /* Pointer to the ADCx registers block.*/                                 \
  ADC_TypeDef               *adc;                                           \
  /* Pointer to associated DMA channel.*/                                   \
  const stm32_dma_stream_t  *dmastp;                                        \
  /* DMA mode bit mask.*/                                                   \
  uint32_t                  dmamode;

/**
 * @brief   Low level fields of the ADC configuration structure.
 */
#define adc_lld_config_fields                                               \
  /* Dummy configuration, it is not needed.*/                               \
  uint32_t                  dummy;

/**
 * @brief   Low level fields of the ADC configuration structure.
 */
#define adc_lld_configuration_group_fields                                  \
  /* ADC CFGR1 register initialization data.                                \
     NOTE: The bits DMAEN and DMACFG are enforced internally                \
           to the driver, keep them to zero.                                \
     NOTE: The bits @p ADC_CFGR1_CONT or @p ADC_CFGR1_DISCEN must be        \
           specified in continuous more or if the buffer depth is           \
           greater than one.*/                                              \
  uint32_t                  cfgr1;                                          \
  /* ADC TR register initialization data.*/                                 \
  uint32_t                  tr;                                             \
  /* ADC SMPR register initialization data.*/                               \
  uint32_t                  smpr;                                           \
  /* ADC CHSELR register initialization data.                               \
     NOTE: The number of bits at logic level one in this register must      \
           be equal to the number in the @p num_channels field.*/           \
  uint32_t                  chselr;

/**
 * @brief   Changes the value of the ADC CCR register.
 * @details Use this function in order to enable or disable the internal
 *          analog sources. See the documentation in the STM32 Reference
 *          Manual.
 * @note    PRESC bits must not be specified and left to zero.
 */
#define adcSTM32SetCCR(ccr) (ADC->CCR = (ccr))

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if STM32_ADC_USE_ADC1 && !defined(__DOXYGEN__)
extern ADCDriver ADCD1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void adc_lld_init(void);
  void adc_lld_start(ADCDriver *adcp);
  void adc_lld_stop(ADCDriver *adcp);
  void adc_lld_start_conversion(ADCDriver *adcp);
  void adc_lld_stop_conversion(ADCDriver *adcp);
  void adc_lld_serve_interrupt(ADCDriver *adcp);
  void adcSTM32EnableVREF(ADCDriver *adcp);
  void adcSTM32DisableVREF(ADCDriver *adcp);
  void adcSTM32EnableTS(ADCDriver *adcp);
  void adcSTM32DisableTS(ADCDriver *adcp);
#if defined(ADC_CCR_VBATEN)
  void adcSTM32EnableVBAT(ADCDriver *adcp);
  void adcSTM32DisableVBAT(ADCDriver *adcp);
#endif
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_ADC */

#endif /* HAL_ADC_LLD_H */

/** @} */
