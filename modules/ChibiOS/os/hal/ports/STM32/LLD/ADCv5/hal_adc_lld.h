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
#define ADC_ERR_DMAFAILURE              1U  /**< DMA operations failure.    */
#define ADC_ERR_OVERFLOW                2U  /**< ADC overflow condition.    */
#define ADC_ERR_AWD1                    4U  /**< Watchdog 1 triggered.      */
#define ADC_ERR_AWD2                    8U  /**< Watchdog 2 triggered.      */
#define ADC_ERR_AWD3                    16U /**< Watchdog 3 triggered.      */
/** @} */

/**
 * @name    SMPR register configuration helpers
 * @{
 */
#define ADC_SMPR_SMP1_1P5               (0U << 0)   /**< @brief 14 cycles.  */
#define ADC_SMPR_SMP1_3P5               (1U << 0)   /**< @brief 16 cycles.  */
#define ADC_SMPR_SMP1_7P5               (2U << 0)   /**< @brief 20 cycles.  */
#define ADC_SMPR_SMP1_12P5              (3U << 0)   /**< @brief 25 cycles.  */
#define ADC_SMPR_SMP1_19P5              (4U << 0)   /**< @brief 31 cycles.  */
#define ADC_SMPR_SMP1_39P5              (5U << 0)   /**< @brief 52 cycles.  */
#define ADC_SMPR_SMP1_79P5              (6U << 0)   /**< @brief 92 cycles.  */
#define ADC_SMPR_SMP1_160P5             (7U << 0)   /**< @brief 173 cycles. */

#define ADC_SMPR_SMP2_1P5               (0U << 4)   /**< @brief 14 cycles.  */
#define ADC_SMPR_SMP2_3P5               (1U << 4)   /**< @brief 16 cycles.  */
#define ADC_SMPR_SMP2_7P5               (2U << 4)   /**< @brief 20 cycles.  */
#define ADC_SMPR_SMP2_12P5              (3U << 4)   /**< @brief 25 cycles.  */
#define ADC_SMPR_SMP2_19P5              (4U << 4)   /**< @brief 31 cycles.  */
#define ADC_SMPR_SMP2_39P5              (5U << 4)   /**< @brief 52 cycles.  */
#define ADC_SMPR_SMP2_79P5              (6U << 4)   /**< @brief 92 cycles.  */
#define ADC_SMPR_SMP2_160P5             (7U << 4)   /**< @brief 173 cycles. */
/** @} */

/**
 * @name    CFGR1 register configuration helpers
 * @{
 */
#define ADC_CFGR1_RES_12BIT             (0U << 3)
#define ADC_CFGR1_RES_10BIT             (1U << 3)
#define ADC_CFGR1_RES_8BIT              (2U << 3)
#define ADC_CFGR1_RES_6BIT              (3U << 3)

#define ADC_CFGR1_EXTSEL_MASK           (15U << 6)
#define ADC_CFGR1_EXTSEL_SRC(n)         ((n) << 6)

#define ADC_CFGR1_EXTEN_MASK            (3U << 10)
#define ADC_CFGR1_EXTEN_DISABLED        (0U << 10)
#define ADC_CFGR1_EXTEN_RISING          (1U << 10)
#define ADC_CFGR1_EXTEN_FALLING         (2U << 10)
#define ADC_CFGR1_EXTEN_BOTH            (3U << 10)
/** @} */

/**
 * @name    CFGR2 register configuration helpers
 * @{
 */
#define ADC_CFGR2_CKMODE_MASK           (3U << 30)
#define ADC_CFGR2_CKMODE_ADCCLK         (0U << 30)
#define ADC_CFGR2_CKMODE_PCLK_DIV2      (1U << 30)
#define ADC_CFGR2_CKMODE_PCLK_DIV4      (2U << 30)
#define ADC_CFGR2_CKMODE_PCLK           (3U << 30)

#define ADC_CFGR2_OVSR_MASK             (7U << 2)
#define ADC_CFGR2_OVSR_2X               (0U << 2)
#define ADC_CFGR2_OVSR_4X               (1U << 2)
#define ADC_CFGR2_OVSR_8X               (2U << 2)
#define ADC_CFGR2_OVSR_16X              (3U << 2)
#define ADC_CFGR2_OVSR_32X              (4U << 2)
#define ADC_CFGR2_OVSR_64X              (5U << 2)
#define ADC_CFGR2_OVSR_128X             (6U << 2)
#define ADC_CFGR2_OVSR_256X             (7U << 2)

#define ADC_CFGR2_OVSS_MASK             (15 << 5)
#define ADC_CFGR2_OVSS_SHIFT(n)         ((n) << 5)
/** @} */

/**
 * @name    CHSELR register initializers for CHSELRMOD=0
 * @{
 */
#define ADC_CHSELR_CHSEL_N(n)           (1U << (n))
/** @} */

/**
 * @name    CHSELR register initializers for CHSELRMOD=1
 * @{
 */
#define ADC_CHSELR_SQ1_N(n)             ((uint32_t)(n) << 0U)
#define ADC_CHSELR_SQ2_N(n)             ((uint32_t)(n) << 4U)
#define ADC_CHSELR_SQ3_N(n)             ((uint32_t)(n) << 8U)
#define ADC_CHSELR_SQ4_N(n)             ((uint32_t)(n) << 12U)
#define ADC_CHSELR_SQ5_N(n)             ((uint32_t)(n) << 16U)
#define ADC_CHSELR_SQ6_N(n)             ((uint32_t)(n) << 20U)
#define ADC_CHSELR_SQ7_N(n)             ((uint32_t)(n) << 24U)
#define ADC_CHSELR_SQ8_N(n)             ((uint32_t)(n) << 28U)

#define ADC_CHSELR_SQ1_END              (15U << 0U)
#define ADC_CHSELR_SQ2_END              (15U << 4U)
#define ADC_CHSELR_SQ3_END              (15U << 8U)
#define ADC_CHSELR_SQ4_END              (15U << 12U)
#define ADC_CHSELR_SQ5_END              (15U << 16U)
#define ADC_CHSELR_SQ6_END              (15U << 20U)
#define ADC_CHSELR_SQ7_END              (15U << 24U)
#define ADC_CHSELR_SQ8_END              (15U << 28U)
/** @} */

/**
 * @name    Threshold registers initializers
 * @{
 */
#define ADC_TR(low, high)               (((uint32_t)(high) << 16U) |        \
                                         (uint32_t)(low))
#define ADC_TR_DISABLED                 ADC_TR(0U, 0x0FFFU)
#define ADC_AWDCR_ENABLE(n)             (1U << (n))
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

/*
 * @brief   ADC prescaler setting.
 * @note    This setting has effect only in asynchronous clock mode (the
 *          default, @p STM32_ADC_CKMODE_ADCCLK).
 */
#if !defined(STM32_ADC_PRESCALER_VALUE) || defined(__DOXYGEN__)
#define STM32_ADC_PRESCALER_VALUE           2
#endif

/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Supported devices checks.*/
#if !defined(STM32C0XX) && !defined(STM32G0XX) && !defined(STM32WLXX)
#error "ADCv5 only supports C0, G0 and WL STM32 devices"
#endif

/* Registry checks.*/
#if !defined(STM32_HAS_ADC1)
#error "STM32_HAS_ADC1 not defined in registry"
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

/* ADC clock source checks.*/
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
  /* ADC TR1 register initialization data.*/                                \
  uint32_t                  tr1;                                            \
  /* ADC TR2 register initialization data.*/                                \
  uint32_t                  tr2;                                            \
  /* ADC TR3 register initialization data.*/                                \
  uint32_t                  tr3;                                            \
  /* ADC AWD2CR register initialization data.*/                             \
  uint32_t                  awd2cr;                                         \
  /* ADC AWD3CR register initialization data.*/                             \
  uint32_t                  awd3cr;                                         \
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
#define adcSTM32SetCCR(ccr) (ADC1_COMMON->CCR = (ccr))

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
