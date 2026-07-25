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
 * @file    ADCv7/hal_adc_lld.h
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

#define ADC_LLD_ENHANCED_API

/**
 * @name    Possible ADC errors mask bits.
 * @{
 */
#define ADC_ERR_DMAFAILURE      1U  /**< DMA operations failure.            */
#define ADC_ERR_OVERFLOW        2U  /**< ADC overflow condition.            */
#define ADC_ERR_AWD1            4U  /**< Watchdog 1 triggered.              */
#define ADC_ERR_AWD2            8U  /**< Watchdog 2 triggered.              */
#define ADC_ERR_AWD3            16U /**< Watchdog 3 triggered.              */
/** @} */

/**
 * @name    Available analog channels
 * @{
 */
#define ADC_CHANNEL_IN0         0U  /**< @brief External analog input 0.    */
#define ADC_CHANNEL_IN1         1U  /**< @brief External analog input 1.    */
#define ADC_CHANNEL_IN2         2U  /**< @brief External analog input 2.    */
#define ADC_CHANNEL_IN3         3U  /**< @brief External analog input 3.    */
#define ADC_CHANNEL_IN4         4U  /**< @brief External analog input 4.    */
#define ADC_CHANNEL_IN5         5U  /**< @brief External analog input 5.    */
#define ADC_CHANNEL_IN6         6U  /**< @brief External analog input 6.    */
#define ADC_CHANNEL_IN7         7U  /**< @brief External analog input 7.    */
#define ADC_CHANNEL_IN8         8U  /**< @brief External analog input 8.    */
#define ADC_CHANNEL_IN9         9U  /**< @brief External analog input 9.    */
#define ADC_CHANNEL_IN10        10U /**< @brief External analog input 10.   */
#define ADC_CHANNEL_IN11        11U /**< @brief External analog input 11.   */
#define ADC_CHANNEL_IN12        12U /**< @brief External analog input 12.   */
#define ADC_CHANNEL_IN13        13U /**< @brief External analog input 13.   */
#define ADC_CHANNEL_IN14        14U /**< @brief External analog input 14.   */
#define ADC_CHANNEL_IN15        15U /**< @brief External analog input 15.   */
#define ADC_CHANNEL_IN16        16U /**< @brief External analog input 16.   */
#define ADC_CHANNEL_IN17        17U /**< @brief External analog input 17.   */
#define ADC_CHANNEL_IN18        18U /**< @brief External analog input 18.   */
/** @} */

/**
 * @name    ADC channels selection masks
 * @{
 */
#define ADC_SELMASK_IN0         (1U << ADC_CHANNEL_IN0)
#define ADC_SELMASK_IN1         (1U << ADC_CHANNEL_IN1)
#define ADC_SELMASK_IN2         (1U << ADC_CHANNEL_IN2)
#define ADC_SELMASK_IN3         (1U << ADC_CHANNEL_IN3)
#define ADC_SELMASK_IN4         (1U << ADC_CHANNEL_IN4)
#define ADC_SELMASK_IN5         (1U << ADC_CHANNEL_IN5)
#define ADC_SELMASK_IN6         (1U << ADC_CHANNEL_IN6)
#define ADC_SELMASK_IN7         (1U << ADC_CHANNEL_IN7)
#define ADC_SELMASK_IN8         (1U << ADC_CHANNEL_IN8)
#define ADC_SELMASK_IN9         (1U << ADC_CHANNEL_IN9)
#define ADC_SELMASK_IN10        (1U << ADC_CHANNEL_IN10)
#define ADC_SELMASK_IN11        (1U << ADC_CHANNEL_IN11)
#define ADC_SELMASK_IN12        (1U << ADC_CHANNEL_IN12)
#define ADC_SELMASK_IN13        (1U << ADC_CHANNEL_IN13)
#define ADC_SELMASK_IN14        (1U << ADC_CHANNEL_IN14)
#define ADC_SELMASK_IN15        (1U << ADC_CHANNEL_IN15)
#define ADC_SELMASK_IN16        (1U << ADC_CHANNEL_IN16)
#define ADC_SELMASK_IN17        (1U << ADC_CHANNEL_IN17)
#define ADC_SELMASK_IN18        (1U << ADC_CHANNEL_IN18)
/** @} */

/**
 * @name    Sampling rates
 * @{
 */
#define ADC_SMPR_SMP_1P5        0U  /**< @brief 14 cycles conversion time.  */
#define ADC_SMPR_SMP_2P5        1U  /**< @brief 15 cycles conversion time.  */
#define ADC_SMPR_SMP_6P5        2U  /**< @brief 19 cycles conversion time.  */
#define ADC_SMPR_SMP_11P5       3U  /**< @brief 24 cycles conversion time.  */
#define ADC_SMPR_SMP_23P5       4U  /**< @brief 36 cycles conversion time.  */
#define ADC_SMPR_SMP_46P5       5U  /**< @brief 59 cycles conversion time.  */
#define ADC_SMPR_SMP_246P5      6U  /**< @brief 259 cycles conversion time. */
#define ADC_SMPR_SMP_1499P5     7U  /**< @brief 1512 cycles conversion time.*/
/** @} */

/**
 * @name    CFGR1 register configuration helpers
 * @{
 */
#define ADC_CFGR1_DMNGT_MASK            ADC_CFGR1_DMNGT_Msk
#define ADC_CFGR1_DMNGT_NODMA           (0U << ADC_CFGR1_DMNGT_Pos)
#define ADC_CFGR1_DMNGT_ONESHOT         (1U << ADC_CFGR1_DMNGT_Pos)
#define ADC_CFGR1_DMNGT_DFSDM           (2U << ADC_CFGR1_DMNGT_Pos)
#define ADC_CFGR1_DMNGT_CIRCULAR        (3U << ADC_CFGR1_DMNGT_Pos)
#define ADC_CFGR1_DMNGT_DR_ONLY         ADC_CFGR1_DMNGT_NODMA

#define ADC_CFGR1_RES_MASK              ADC_CFGR1_RES_Msk
#define ADC_CFGR1_RES_12BITS            (0U << ADC_CFGR1_RES_Pos)
#define ADC_CFGR1_RES_10BITS            (1U << ADC_CFGR1_RES_Pos)
#define ADC_CFGR1_RES_8BITS             (2U << ADC_CFGR1_RES_Pos)
#define ADC_CFGR1_RES_6BITS             (3U << ADC_CFGR1_RES_Pos)

#define ADC_CFGR1_EXTSEL_MASK           ADC_CFGR1_EXTSEL_Msk
#define ADC_CFGR1_EXTSEL_SRC(n)         ((uint32_t)(n) << ADC_CFGR1_EXTSEL_Pos)

#define ADC_CFGR1_EXTEN_MASK            ADC_CFGR1_EXTEN_Msk
#define ADC_CFGR1_EXTEN_DISABLED        (0U << ADC_CFGR1_EXTEN_Pos)
#define ADC_CFGR1_EXTEN_RISING          (1U << ADC_CFGR1_EXTEN_Pos)
#define ADC_CFGR1_EXTEN_FALLING         (2U << ADC_CFGR1_EXTEN_Pos)
#define ADC_CFGR1_EXTEN_BOTH            (3U << ADC_CFGR1_EXTEN_Pos)

#define ADC_CFGR1_OVRMOD_PRESERVE       (0U << ADC_CFGR1_OVRMOD_Pos)
#define ADC_CFGR1_OVRMOD_OVERWRITE      (1U << ADC_CFGR1_OVRMOD_Pos)

#define ADC_CFGR1_CONT_DISABLED         (0U << ADC_CFGR1_CONT_Pos)
#define ADC_CFGR1_CONT_ENABLED          (1U << ADC_CFGR1_CONT_Pos)

#define ADC_CFGR1_DISCEN_DISABLED       (0U << ADC_CFGR1_DISCEN_Pos)
#define ADC_CFGR1_DISCEN_ENABLED        (1U << ADC_CFGR1_DISCEN_Pos)

#define ADC_CFGR1_DISCNUM_MASK          ADC_CFGR1_DISCNUM_Msk
#define ADC_CFGR1_DISCNUM_VAL(n)        ((uint32_t)(n) << ADC_CFGR1_DISCNUM_Pos)

#define ADC_CFGR1_AWD1_DISABLED         0U
#define ADC_CFGR1_AWD1_ALL              (1U << ADC_CFGR1_AWD1EN_Pos)
#define ADC_CFGR1_AWD1_SINGLE(n)        (((uint32_t)(n) << ADC_CFGR1_AWD1CH_Pos) | \
                                         ADC_CFGR1_AWD1EN |                 \
                                         ADC_CFGR1_AWD1SGL)
/** @} */

/**
 * @name    CCR register configuration helpers
 * @{
 */
#define ADC_CCR_DUAL_FIELD(n)           ((uint32_t)(n) << ADC_CCR_DUAL_Pos)
#define ADC_CCR_DELAY_FIELD(n)          ((uint32_t)(n) << ADC_CCR_DELAY_Pos)

#define ADC_CCR_DAMDF_DISABLED          (0U << ADC_CCR_DAMDF_Pos)
#define ADC_CCR_DAMDF_HWORD             (2U << ADC_CCR_DAMDF_Pos)
#define ADC_CCR_DAMDF_BYTE              (3U << ADC_CCR_DAMDF_Pos)
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   Enables the ADC master/slave mode.
 * @note    In dual mode only ADCD1 acts on both master/slave peripherals.
 */
#if !defined(STM32_ADC_DUAL_MODE) || defined(__DOXYGEN__)
#define STM32_ADC_DUAL_MODE                 FALSE
#endif

/**
 * @brief   Makes the ADC samples type an 8bits one.
 * @note    10 and 12 bits sampling mode must not be used when this option
 *          is enabled.
 */
#if !defined(STM32_ADC_COMPACT_SAMPLES) || defined(__DOXYGEN__)
#define STM32_ADC_COMPACT_SAMPLES           FALSE
#endif

/**
 * @brief   ADC1 driver enable switch.
 * @details If set to @p TRUE the support for ADC1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_ADC_USE_ADC1) || defined(__DOXYGEN__)
#define STM32_ADC_USE_ADC1                  FALSE
#endif

/**
 * @brief   ADC2 driver enable switch.
 * @details If set to @p TRUE the support for ADC2 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_ADC_USE_ADC2) || defined(__DOXYGEN__)
#define STM32_ADC_USE_ADC2                  FALSE
#endif

/**
 * @brief   ADC1 DMA priority (0..7|lowest..highest).
 */
#if !defined(STM32_ADC_ADC1_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ADC_ADC1_DMA_PRIORITY         2
#endif

/**
 * @brief   ADC2 DMA priority (0..7|lowest..highest).
 */
#if !defined(STM32_ADC_ADC2_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ADC_ADC2_DMA_PRIORITY         2
#endif

/**
 * @brief   ADC1 DMA interrupt priority level setting.
 */
#if !defined(STM32_ADC_ADC1_DMA_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ADC_ADC1_DMA_IRQ_PRIORITY     5
#endif

/**
 * @brief   ADC2 DMA interrupt priority level setting.
 */
#if !defined(STM32_ADC_ADC2_DMA_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ADC_ADC2_DMA_IRQ_PRIORITY     5
#endif

/**
 * @brief   ADC1 interrupt priority level setting.
 */
#if !defined(STM32_ADC_ADC1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ADC_ADC1_IRQ_PRIORITY         5
#endif

/**
 * @brief   ADC2 interrupt priority level setting.
 */
#if !defined(STM32_ADC_ADC2_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ADC_ADC2_IRQ_PRIORITY         5
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Supported devices checks.*/
#if !defined(STM32U3XX)
#error "ADCv7 only supports STM32U3 devices"
#endif

/* Registry checks.*/
#if !defined(STM32_HAS_ADC1) || !defined(STM32_HAS_ADC2)
#error "STM32_HAS_ADCx not defined in registry"
#endif

/* Units checks.*/
#if STM32_ADC_USE_ADC1 && !STM32_HAS_ADC1
#error "ADC1 not present in the selected device"
#endif

#if STM32_ADC_USE_ADC2 && !STM32_HAS_ADC2
#error "ADC2 not present in the selected device"
#endif

#if STM32_ADC_DUAL_MODE && (!STM32_ADC_USE_ADC1 || !STM32_HAS_ADC2)
#error "STM32_ADC_DUAL_MODE requires ADC1 and ADC2"
#endif

/* IRQ handlers checks.*/
#if STM32_ADC_USE_ADC1 && !defined(STM32_ADC1_HANDLER)
#error "STM32_ADC1_HANDLER not defined in registry"
#endif

#if STM32_ADC_USE_ADC2 && !defined(STM32_ADC2_HANDLER)
#error "STM32_ADC2_HANDLER not defined in registry"
#endif

/* IRQ vector numbers checks.*/
#if STM32_ADC_USE_ADC1 && !defined(STM32_ADC1_NUMBER)
#error "STM32_ADC1_NUMBER not defined in registry"
#endif

#if STM32_ADC_USE_ADC2 && !defined(STM32_ADC2_NUMBER)
#error "STM32_ADC2_NUMBER not defined in registry"
#endif

/* At least one ADC must be assigned.*/
#if !STM32_ADC_USE_ADC1 && !STM32_ADC_USE_ADC2
#error "ADC driver activated but no ADC peripheral assigned"
#endif

/* DMA channel configuration checks.*/
#if STM32_ADC_USE_ADC1 && !defined(STM32_ADC_ADC1_DMA3_CHANNEL)
#error "ADC1 DMA3 channel not defined"
#endif

#if STM32_ADC_USE_ADC2 && !defined(STM32_ADC_ADC2_DMA3_CHANNEL)
#error "ADC2 DMA3 channel not defined"
#endif

/* DMA priority checks.*/
#if STM32_ADC_USE_ADC1 &&                                                   \
    !STM32_DMA3_IS_VALID_PRIORITY(STM32_ADC_ADC1_DMA_PRIORITY)
#error "Invalid DMA priority assigned to ADC1"
#endif

#if STM32_ADC_USE_ADC2 &&                                                   \
    !STM32_DMA3_IS_VALID_PRIORITY(STM32_ADC_ADC2_DMA_PRIORITY)
#error "Invalid DMA priority assigned to ADC2"
#endif

/* DMA IRQ priority checks.*/
#if STM32_ADC_USE_ADC1 &&                                                   \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_ADC_ADC1_DMA_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to ADC1 DMA"
#endif

#if STM32_ADC_USE_ADC2 &&                                                   \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_ADC_ADC2_DMA_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to ADC2 DMA"
#endif

/* ADC IRQ priority tests.*/
#if STM32_ADC_USE_ADC1 &&                                                   \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_ADC_ADC1_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to ADC1"
#endif

#if STM32_ADC_USE_ADC2 &&                                                   \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_ADC_ADC2_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to ADC2"
#endif

/* Samples size option check.*/
#if (STM32_ADC_COMPACT_SAMPLES == TRUE)
#define STM32_ADC_SAMPLES_SIZE            8U
#else
#define STM32_ADC_SAMPLES_SIZE            16U
#endif

#if STM32_ADC_DUAL_MODE
#if STM32_ADC_COMPACT_SAMPLES
#define ADC_SAMPLE_MULTIPLIER             2U
#else
#define ADC_SAMPLE_MULTIPLIER             4U
#endif
#else
#if STM32_ADC_COMPACT_SAMPLES
#define ADC_SAMPLE_MULTIPLIER             1U
#else
#define ADC_SAMPLE_MULTIPLIER             2U
#endif
#endif

#if !defined(STM32_DMA3_REQUIRED)
#define STM32_DMA3_REQUIRED
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   ADC sample data type.
 */
#if STM32_ADC_COMPACT_SAMPLES
typedef uint8_t adcsample_t;
#else
typedef uint16_t adcsample_t;
#endif

/**
 * @brief   Channels number in a conversion group.
 */
typedef uint16_t adc_channels_num_t;

/**
 * @brief   Type of an ADC error mask.
 */
typedef uint32_t adcerror_t;

/**
 * @brief   Type of a structure containing DMA-accessible driver fields.
 */
typedef struct adc_dmabuf {
  uint32_t                          cdar;
} adc_dmabuf_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Low level fields of the ADC driver structure.
 */
#if (STM32_ADC_DUAL_MODE == TRUE) || defined(__DOXYGEN__)
#define adc_lld_driver_fields                                               \
  /* Pointer to the master ADCx registers block.*/                          \
  ADC_TypeDef                       *adcm;                                  \
  /* Pointer to the slave ADCx registers block.*/                           \
  ADC_TypeDef                       *adcs;                                  \
  /* Pointer to the common ADC registers block.*/                           \
  ADC_Common_TypeDef                *adcc;                                  \
  /* Pointer to associated DMA channel.*/                                   \
  const stm32_dma3_channel_t        *dmachp;                                \
  /* DMA priority.*/                                                        \
  uint8_t                           dprio;                                  \
  /* DMA request number.*/                                                  \
  uint8_t                           dreq;                                   \
  /* DMA buffers.*/                                                         \
  adc_dmabuf_t                      *dbuf;
#else
#define adc_lld_driver_fields                                               \
  /* Pointer to the master ADCx registers block.*/                          \
  ADC_TypeDef                       *adcm;                                  \
  /* Pointer to the common ADC registers block.*/                           \
  ADC_Common_TypeDef                *adcc;                                  \
  /* Pointer to associated DMA channel.*/                                   \
  const stm32_dma3_channel_t        *dmachp;                                \
  /* DMA priority.*/                                                        \
  uint8_t                           dprio;                                  \
  /* DMA request number.*/                                                  \
  uint8_t                           dreq;                                   \
  /* DMA buffers.*/                                                         \
  adc_dmabuf_t                      *dbuf;
#endif

/**
 * @brief   Low level fields of the ADC configuration structure.
 */
#define adc_lld_config_fields                                               \
  /* DMA extra CTR1 settings.*/                                             \
  uint32_t                          dmactr1;                                \
  /* DMA extra CTR2 settings.*/                                             \
  uint32_t                          dmactr2;

/**
 * @brief   Low level fields of the ADC group configuration structure.
 */
#if (STM32_ADC_DUAL_MODE == TRUE) || defined(__DOXYGEN__)
#define adc_lld_configuration_group_fields                                  \
  /* ADC CFGR1 register initialization data.*/                              \
  uint32_t                          cfgr;                                   \
  /* ADC CFGR2 register initialization data.*/                              \
  uint32_t                          cfgr2;                                  \
  /* ADC CCR register initialization data.*/                                \
  uint32_t                          ccr;                                    \
  /* ADC PCSEL register initialization data.*/                              \
  uint32_t                          pcsel;                                  \
  /* ADC AWD1 threshold registers initialization data.*/                    \
  uint32_t                          ltr1;                                   \
  uint32_t                          htr1;                                   \
  /* ADC AWD2 threshold registers initialization data.*/                    \
  uint32_t                          ltr2;                                   \
  uint32_t                          htr2;                                   \
  /* ADC AWD3 threshold registers initialization data.*/                    \
  uint32_t                          ltr3;                                   \
  uint32_t                          htr3;                                   \
  /* ADC AWDxCR registers initialization data.*/                            \
  uint32_t                          awd2cr;                                 \
  uint32_t                          awd3cr;                                 \
  /* ADC SMPRx registers initialization data.*/                             \
  uint32_t                          smpr[2];                                \
  /* ADC SQRx registers initialization data.*/                              \
  uint32_t                          sqr[4];                                 \
  /* Slave ADC threshold registers initialization data.*/                   \
  uint32_t                          sltr1;                                  \
  uint32_t                          shtr1;                                  \
  uint32_t                          sltr2;                                  \
  uint32_t                          shtr2;                                  \
  uint32_t                          sltr3;                                  \
  uint32_t                          shtr3;                                  \
  /* Slave ADC AWDxCR registers initialization data.*/                      \
  uint32_t                          sawd2cr;                                \
  uint32_t                          sawd3cr;                                \
  /* Slave ADC SMPRx registers initialization data.*/                       \
  uint32_t                          ssmpr[2];                               \
  /* Slave ADC SQRx registers initialization data.*/                        \
  uint32_t                          ssqr[4]
#else
#define adc_lld_configuration_group_fields                                  \
  uint32_t                          cfgr;                                   \
  uint32_t                          cfgr2;                                  \
  uint32_t                          ccr;                                    \
  uint32_t                          pcsel;                                  \
  uint32_t                          ltr1;                                   \
  uint32_t                          htr1;                                   \
  uint32_t                          ltr2;                                   \
  uint32_t                          htr2;                                   \
  uint32_t                          ltr3;                                   \
  uint32_t                          htr3;                                   \
  uint32_t                          awd2cr;                                 \
  uint32_t                          awd3cr;                                 \
  uint32_t                          smpr[2];                                \
  uint32_t                          sqr[4];
#endif

/**
 * @name    Sequences building helper macros
 * @{
 */
#define ADC_SQR1_NUM_CH(n)      (((n) - 1U) << 0U)
#define ADC_SQR1_SQ1_N(n)       ((uint32_t)(n) << 6U)
#define ADC_SQR1_SQ2_N(n)       ((uint32_t)(n) << 12U)
#define ADC_SQR1_SQ3_N(n)       ((uint32_t)(n) << 18U)
#define ADC_SQR1_SQ4_N(n)       ((uint32_t)(n) << 24U)

#define ADC_SQR2_SQ5_N(n)       ((uint32_t)(n) << 0U)
#define ADC_SQR2_SQ6_N(n)       ((uint32_t)(n) << 6U)
#define ADC_SQR2_SQ7_N(n)       ((uint32_t)(n) << 12U)
#define ADC_SQR2_SQ8_N(n)       ((uint32_t)(n) << 18U)
#define ADC_SQR2_SQ9_N(n)       ((uint32_t)(n) << 24U)

#define ADC_SQR3_SQ10_N(n)      ((uint32_t)(n) << 0U)
#define ADC_SQR3_SQ11_N(n)      ((uint32_t)(n) << 6U)
#define ADC_SQR3_SQ12_N(n)      ((uint32_t)(n) << 12U)
#define ADC_SQR3_SQ13_N(n)      ((uint32_t)(n) << 18U)
#define ADC_SQR3_SQ14_N(n)      ((uint32_t)(n) << 24U)

#define ADC_SQR4_SQ15_N(n)      ((uint32_t)(n) << 0U)
#define ADC_SQR4_SQ16_N(n)      ((uint32_t)(n) << 6U)
#define ADC_SQR4_SQ17_N(n)      ((uint32_t)(n) << 12U)
#define ADC_SQR4_SQ18_N(n)      ((uint32_t)(n) << 18U)
/** @} */

/**
 * @name    Sampling rate settings helper macros
 * @{
 */
#define ADC_SMPR1_SMP_AN0(n)    ((n) << 0U) /**< @brief AN0 sampling time.  */
#define ADC_SMPR1_SMP_AN1(n)    ((n) << 3U) /**< @brief AN1 sampling time.  */
#define ADC_SMPR1_SMP_AN2(n)    ((n) << 6U) /**< @brief AN2 sampling time.  */
#define ADC_SMPR1_SMP_AN3(n)    ((n) << 9U) /**< @brief AN3 sampling time.  */
#define ADC_SMPR1_SMP_AN4(n)    ((n) << 12U)/**< @brief AN4 sampling time.  */
#define ADC_SMPR1_SMP_AN5(n)    ((n) << 15U)/**< @brief AN5 sampling time.  */
#define ADC_SMPR1_SMP_AN6(n)    ((n) << 18U)/**< @brief AN6 sampling time.  */
#define ADC_SMPR1_SMP_AN7(n)    ((n) << 21U)/**< @brief AN7 sampling time.  */
#define ADC_SMPR1_SMP_AN8(n)    ((n) << 24U)/**< @brief AN8 sampling time.  */
#define ADC_SMPR1_SMP_AN9(n)    ((n) << 27U)/**< @brief AN9 sampling time.  */

#define ADC_SMPR2_SMP_AN10(n)   ((n) << 0U) /**< @brief AN10 sampling time. */
#define ADC_SMPR2_SMP_AN11(n)   ((n) << 3U) /**< @brief AN11 sampling time. */
#define ADC_SMPR2_SMP_AN12(n)   ((n) << 6U) /**< @brief AN12 sampling time. */
#define ADC_SMPR2_SMP_AN13(n)   ((n) << 9U) /**< @brief AN13 sampling time. */
#define ADC_SMPR2_SMP_AN14(n)   ((n) << 12U)/**< @brief AN14 sampling time. */
#define ADC_SMPR2_SMP_AN15(n)   ((n) << 15U)/**< @brief AN15 sampling time. */
#define ADC_SMPR2_SMP_AN16(n)   ((n) << 18U)/**< @brief AN16 sampling time. */
#define ADC_SMPR2_SMP_AN17(n)   ((n) << 21U)/**< @brief AN17 sampling time. */
#define ADC_SMPR2_SMP_AN18(n)   ((n) << 24U)/**< @brief AN18 sampling time. */
#define ADC_SMPR2_SMP_AN19(n)   ((n) << 27U)/**< @brief AN19 sampling time. */
/** @} */

/**
 * @name    Analog watchdog settings helper macros
 * @{
 */
#define ADC_CFGR_AWD1_N(n)      ((n) << 26U)/**< @brief AWD1 channel number */
#define ADC_AWD23_MASK(n)       (1U << (n)) /**< @brief AWD2/3 channels mask*/
/** @} */

/**
 * @name    Oversampling settings helper macros
 * @{
 */
#define ADC_CFGR2_OVSS_N(n)     ((n) << 5U)/**< @brief ovsr right shift */
#define ADC_CFGR2_OVSR_N(n)     ((n) << 16U)/**< @brief oversampling ratio */
#define ADC_CFGR2_LSHIFT_N(n)   ((n) << 28U)/**< @brief ovsr left shift */
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if STM32_ADC_USE_ADC1 && !defined(__DOXYGEN__)
extern ADCDriver ADCD1;
#endif

#if STM32_ADC_USE_ADC2 && !defined(__DOXYGEN__)
extern ADCDriver ADCD2;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void adc_lld_init(void);
  msg_t adc_lld_start(ADCDriver *adcp);
  void adc_lld_stop(ADCDriver *adcp);
  void adc_lld_start_conversion(ADCDriver *adcp);
  void adc_lld_stop_conversion(ADCDriver *adcp);
  void adcSTM32EnableVREF(ADCDriver *adcp);
  void adcSTM32DisableVREF(ADCDriver *adcp);
  void adcSTM32EnableTS(ADCDriver *adcp);
  void adcSTM32DisableTS(ADCDriver *adcp);
  void adcSTM32EnableVBAT(ADCDriver *adcp);
  void adcSTM32DisableVBAT(ADCDriver *adcp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_ADC */

#endif /* HAL_ADC_LLD_H */

/** @} */
