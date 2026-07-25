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
 * @file    ADCv2/hal_adc_lld.h
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
#define ADC_ERR_AWD             4U  /**< Watchdog triggered.                */
/** @} */

/**
 * @name    Absolute Maximum Ratings
 * @{
 */
/**
 * @brief   Minimum ADC clock frequency.
 */
#define STM32_ADCCLK_MIN        600000

/**
 * @brief   Maximum ADC clock frequency.
 */
#if defined(STM32F4XX) || defined(__DOXYGEN__)
#define STM32_ADCCLK_MAX        36000000
#else
#define STM32_ADCCLK_MAX        30000000
#endif
/** @} */

/**
 * @name    Triggers selection
 * @{
 */
#define ADC_CR2_EXTEN_MASK      (3U << 28U)
#define ADC_CR2_EXTEN_DISABLED  (0U << 28U)
#define ADC_CR2_EXTEN_RISING    (1U << 28U)
#define ADC_CR2_EXTEN_FALLING   (2U << 28U)
#define ADC_CR2_EXTEN_BOTH      (3U << 28U)

#define ADC_CR2_EXTSEL_MASK     (15U << 24U)
#define ADC_CR2_EXTSEL_SRC(n)   ((n) << 24U)
/** @} */

/**
 * @name    ADC clock divider settings
 * @{
 */
#define ADC_CCR_ADCPRE_DIV2     0
#define ADC_CCR_ADCPRE_DIV4     1
#define ADC_CCR_ADCPRE_DIV6     2
#define ADC_CCR_ADCPRE_DIV8     3
/** @} */

/**
 * @name    Available analog channels
 * @{
 */
#define ADC_CHANNEL_IN0         0   /**< @brief External analog input 0.    */
#define ADC_CHANNEL_IN1         1   /**< @brief External analog input 1.    */
#define ADC_CHANNEL_IN2         2   /**< @brief External analog input 2.    */
#define ADC_CHANNEL_IN3         3   /**< @brief External analog input 3.    */
#define ADC_CHANNEL_IN4         4   /**< @brief External analog input 4.    */
#define ADC_CHANNEL_IN5         5   /**< @brief External analog input 5.    */
#define ADC_CHANNEL_IN6         6   /**< @brief External analog input 6.    */
#define ADC_CHANNEL_IN7         7   /**< @brief External analog input 7.    */
#define ADC_CHANNEL_IN8         8   /**< @brief External analog input 8.    */
#define ADC_CHANNEL_IN9         9   /**< @brief External analog input 9.    */
#define ADC_CHANNEL_IN10        10  /**< @brief External analog input 10.   */
#define ADC_CHANNEL_IN11        11  /**< @brief External analog input 11.   */
#define ADC_CHANNEL_IN12        12  /**< @brief External analog input 12.   */
#define ADC_CHANNEL_IN13        13  /**< @brief External analog input 13.   */
#define ADC_CHANNEL_IN14        14  /**< @brief External analog input 14.   */
#define ADC_CHANNEL_IN15        15  /**< @brief External analog input 15.   */
#define ADC_CHANNEL_SENSOR      16  /**< @brief Internal temperature sensor.
                                         @note Available onADC1 only.       */
#define ADC_CHANNEL_VREFINT     17  /**< @brief Internal reference.
                                         @note Available onADC1 only.       */
#define ADC_CHANNEL_VBAT        18  /**< @brief VBAT.
                                         @note Available onADC1 only.       */
/** @} */

/**
 * @name    Sampling rates
 * @{
 */
#define ADC_SAMPLE_3            0   /**< @brief 3 cycles sampling time.     */
#define ADC_SAMPLE_15           1   /**< @brief 15 cycles sampling time.    */
#define ADC_SAMPLE_28           2   /**< @brief 28 cycles sampling time.    */
#define ADC_SAMPLE_56           3   /**< @brief 56 cycles sampling time.    */
#define ADC_SAMPLE_84           4   /**< @brief 84 cycles sampling time.    */
#define ADC_SAMPLE_112          5   /**< @brief 112 cycles sampling time.   */
#define ADC_SAMPLE_144          6   /**< @brief 144 cycles sampling time.   */
#define ADC_SAMPLE_480          7   /**< @brief 480 cycles sampling time.   */
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   ADC common clock divider.
 * @note    This setting is influenced by the VDDA voltage and other
 *          external conditions, please refer to the datasheet for more
 *          info.<br>
 *          See section 5.3.20 "12-bit ADC characteristics".
 */
#if !defined(STM32_ADC_ADCPRE) || defined(__DOXYGEN__)
#define STM32_ADC_ADCPRE                    ADC_CCR_ADCPRE_DIV2
#endif

/**
 * @brief   ADC1 driver enable switch.
 * @details If set to @p TRUE the support for ADC1 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(STM32_ADC_USE_ADC1) || defined(__DOXYGEN__)
#define STM32_ADC_USE_ADC1                  FALSE
#endif

/**
 * @brief   ADC2 driver enable switch.
 * @details If set to @p TRUE the support for ADC2 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(STM32_ADC_USE_ADC2) || defined(__DOXYGEN__)
#define STM32_ADC_USE_ADC2                  FALSE
#endif

/**
 * @brief   ADC3 driver enable switch.
 * @details If set to @p TRUE the support for ADC3 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(STM32_ADC_USE_ADC3) || defined(__DOXYGEN__)
#define STM32_ADC_USE_ADC3                  FALSE
#endif

#if !defined(_ARDUPILOT_)

/**
 * @brief   DMA stream used for ADC1 operations.
 */
#if !defined(STM32_ADC_ADC1_DMA_STREAM) || defined(__DOXYGEN__)
#define STM32_ADC_ADC1_DMA_STREAM           STM32_DMA_STREAM_ID(2, 4)
#endif

/**
 * @brief   DMA stream used for ADC2 operations.
 */
#if !defined(STM32_ADC_ADC2_DMA_STREAM) || defined(__DOXYGEN__)
#define STM32_ADC_ADC2_DMA_STREAM           STM32_DMA_STREAM_ID(2, 2)
#endif

/**
 * @brief   DMA stream used for ADC3 operations.
 */
#if !defined(STM32_ADC_ADC3_DMA_STREAM) || defined(__DOXYGEN__)
#define STM32_ADC_ADC3_DMA_STREAM           STM32_DMA_STREAM_ID(2, 1)
#endif

#endif // !_ARDUPILOT_

/**
 * @brief   ADC1 DMA priority (0..3|lowest..highest).
 */
#if !defined(STM32_ADC_ADC1_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ADC_ADC1_DMA_PRIORITY         2
#endif

/**
 * @brief   ADC2 DMA priority (0..3|lowest..highest).
 */
#if !defined(STM32_ADC_ADC2_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ADC_ADC2_DMA_PRIORITY         2
#endif

/**
 * @brief   ADC3 DMA priority (0..3|lowest..highest).
 */
#if !defined(STM32_ADC_ADC3_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ADC_ADC3_DMA_PRIORITY         2
#endif

/**
 * @brief   ADC interrupt priority level setting.
 * @note    This setting is shared among ADC1, ADC2 and ADC3 because
 *          all ADCs share the same vector.
 */
#if !defined(STM32_ADC_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ADC_IRQ_PRIORITY              5
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
 * @brief   ADC3 DMA interrupt priority level setting.
 */
#if !defined(STM32_ADC_ADC3_DMA_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ADC_ADC3_DMA_IRQ_PRIORITY     5
#endif

/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if STM32_ADC_USE_ADC1 && !STM32_HAS_ADC1
#error "ADC1 not present in the selected device"
#endif

#if STM32_ADC_USE_ADC2 && !STM32_HAS_ADC2
#error "ADC2 not present in the selected device"
#endif

#if STM32_ADC_USE_ADC3 && !STM32_HAS_ADC3
#error "ADC3 not present in the selected device"
#endif

#if !STM32_ADC_USE_ADC1 && !STM32_ADC_USE_ADC2 && !STM32_ADC_USE_ADC3
#error "ADC driver activated but no ADC peripheral assigned"
#endif

#if STM32_ADC_USE_ADC1 &&                                                   \
    !STM32_DMA_IS_VALID_ID(STM32_ADC_ADC1_DMA_STREAM, STM32_ADC1_DMA_MSK)
#error "invalid DMA stream associated to ADC1"
#endif

#if STM32_ADC_USE_ADC2 &&                                                   \
    !STM32_DMA_IS_VALID_ID(STM32_ADC_ADC2_DMA_STREAM, STM32_ADC2_DMA_MSK)
#error "invalid DMA stream associated to ADC2"
#endif

#if STM32_ADC_USE_ADC3 &&                                                   \
    !STM32_DMA_IS_VALID_ID(STM32_ADC_ADC3_DMA_STREAM, STM32_ADC3_DMA_MSK)
#error "invalid DMA stream associated to ADC3"
#endif

/* ADC clock related settings and checks.*/
#if STM32_ADC_ADCPRE == ADC_CCR_ADCPRE_DIV2
#define STM32_ADCCLK                        (STM32_PCLK2 / 2)
#elif STM32_ADC_ADCPRE == ADC_CCR_ADCPRE_DIV4
#define STM32_ADCCLK                        (STM32_PCLK2 / 4)
#elif STM32_ADC_ADCPRE == ADC_CCR_ADCPRE_DIV6
#define STM32_ADCCLK                        (STM32_PCLK2 / 6)
#elif STM32_ADC_ADCPRE == ADC_CCR_ADCPRE_DIV8
#define STM32_ADCCLK                        (STM32_PCLK2 / 8)
#else
#error "invalid STM32_ADC_ADCPRE value specified"
#endif

#if (STM32_ADCCLK < STM32_ADCCLK_MIN) || (STM32_ADCCLK > STM32_ADCCLK_MAX)
#error "STM32_ADCCLK outside acceptable range (STM32_ADCCLK_MIN...STM32_ADCCLK_MAX)"
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
  /* ADC CR1 register initialization data.                                  \
     NOTE: All the required bits must be defined into this field except     \
           @p ADC_CR1_SCAN that is enforced inside the driver.*/            \
  uint32_t                  cr1;                                            \
  /* ADC CR2 register initialization data.                                  \
     NOTE: All the required bits must be defined into this field except     \
           @p ADC_CR2_DMA, @p ADC_CR2_CONT and @p ADC_CR2_ADON that are     \
           enforced inside the driver.*/                                    \
  uint32_t                  cr2;                                            \
  /* ADC SMPR1 register initialization data.                                \
     NOTE: In this field must be specified the sample times for channels    \
           10...18.*/                                                       \
  uint32_t                  smpr1;                                          \
  /* ADC SMPR2 register initialization data.                                \
     NOTE: In this field must be specified the sample times for channels    \
           0...9.*/                                                         \
  uint32_t                  smpr2;                                          \
  /* ADC watchdog high threshold register.                                  \
     NOTE: This field defines the high threshold of the analog watchdog.*/  \
  uint16_t                  htr;                                            \
  /* ADC watchdog low threshold register.                                   \
     NOTE: This field defines the low threshold of the analog watchdog.*/   \
  uint16_t                  ltr;                                            \
  /* ADC SQR1 register initialization data.                                 \
     NOTE: Conversion group sequence 13...16 + sequence length.*/           \
  uint32_t                  sqr1;                                           \
  /* ADC SQR2 register initialization data.                                 \
     NOTE: Conversion group sequence 7...12.*/                              \
  uint32_t                  sqr2;                                           \
  /* ADC SQR3 register initialization data.                                 \
     NOTE: Conversion group sequence 1...6.*/                               \
  uint32_t                  sqr3;

/**
 * @name    Sequences building helper macros
 * @{
 */
/**
 * @brief   Number of channels in a conversion sequence.
 */
#define ADC_SQR1_NUM_CH(n)      (((n) - 1) << 20)

#define ADC_SQR3_SQ1_N(n)       ((n) << 0)  /**< @brief 1st channel in seq. */
#define ADC_SQR3_SQ2_N(n)       ((n) << 5)  /**< @brief 2nd channel in seq. */
#define ADC_SQR3_SQ3_N(n)       ((n) << 10) /**< @brief 3rd channel in seq. */
#define ADC_SQR3_SQ4_N(n)       ((n) << 15) /**< @brief 4th channel in seq. */
#define ADC_SQR3_SQ5_N(n)       ((n) << 20) /**< @brief 5th channel in seq. */
#define ADC_SQR3_SQ6_N(n)       ((n) << 25) /**< @brief 6th channel in seq. */

#define ADC_SQR2_SQ7_N(n)       ((n) << 0)  /**< @brief 7th channel in seq. */
#define ADC_SQR2_SQ8_N(n)       ((n) << 5)  /**< @brief 8th channel in seq. */
#define ADC_SQR2_SQ9_N(n)       ((n) << 10) /**< @brief 9th channel in seq. */
#define ADC_SQR2_SQ10_N(n)      ((n) << 15) /**< @brief 10th channel in seq.*/
#define ADC_SQR2_SQ11_N(n)      ((n) << 20) /**< @brief 11th channel in seq.*/
#define ADC_SQR2_SQ12_N(n)      ((n) << 25) /**< @brief 12th channel in seq.*/

#define ADC_SQR1_SQ13_N(n)      ((n) << 0)  /**< @brief 13th channel in seq.*/
#define ADC_SQR1_SQ14_N(n)      ((n) << 5)  /**< @brief 14th channel in seq.*/
#define ADC_SQR1_SQ15_N(n)      ((n) << 10) /**< @brief 15th channel in seq.*/
#define ADC_SQR1_SQ16_N(n)      ((n) << 15) /**< @brief 16th channel in seq.*/
/** @} */

/**
 * @name    Sampling rate settings helper macros
 * @{
 */
#define ADC_SMPR2_SMP_AN0(n)    ((n) << 0)  /**< @brief AN0 sampling time.  */
#define ADC_SMPR2_SMP_AN1(n)    ((n) << 3)  /**< @brief AN1 sampling time.  */
#define ADC_SMPR2_SMP_AN2(n)    ((n) << 6)  /**< @brief AN2 sampling time.  */
#define ADC_SMPR2_SMP_AN3(n)    ((n) << 9)  /**< @brief AN3 sampling time.  */
#define ADC_SMPR2_SMP_AN4(n)    ((n) << 12) /**< @brief AN4 sampling time.  */
#define ADC_SMPR2_SMP_AN5(n)    ((n) << 15) /**< @brief AN5 sampling time.  */
#define ADC_SMPR2_SMP_AN6(n)    ((n) << 18) /**< @brief AN6 sampling time.  */
#define ADC_SMPR2_SMP_AN7(n)    ((n) << 21) /**< @brief AN7 sampling time.  */
#define ADC_SMPR2_SMP_AN8(n)    ((n) << 24) /**< @brief AN8 sampling time.  */
#define ADC_SMPR2_SMP_AN9(n)    ((n) << 27) /**< @brief AN9 sampling time.  */

#define ADC_SMPR1_SMP_AN10(n)   ((n) << 0)  /**< @brief AN10 sampling time. */
#define ADC_SMPR1_SMP_AN11(n)   ((n) << 3)  /**< @brief AN11 sampling time. */
#define ADC_SMPR1_SMP_AN12(n)   ((n) << 6)  /**< @brief AN12 sampling time. */
#define ADC_SMPR1_SMP_AN13(n)   ((n) << 9)  /**< @brief AN13 sampling time. */
#define ADC_SMPR1_SMP_AN14(n)   ((n) << 12) /**< @brief AN14 sampling time. */
#define ADC_SMPR1_SMP_AN15(n)   ((n) << 15) /**< @brief AN15 sampling time. */
#define ADC_SMPR1_SMP_SENSOR(n) ((n) << 18) /**< @brief Temperature Sensor
                                                 sampling time.             */
#define ADC_SMPR1_SMP_VREF(n)   ((n) << 21) /**< @brief Voltage Reference
                                                 sampling time.             */
#define ADC_SMPR1_SMP_VBAT(n)   ((n) << 24) /**< @brief VBAT sampling time. */
/** @} */

/**
 * @name    Threshold settings helper macros
 * @{
 */
/**
 * @brief   High threshold limitation.
 */
#define ADC_HTR(n)              ((n > ADC_HTR_HT) ? ADC_HTR_HT : n)
/**
 * @brief   Low threshold limitation.
 */
#define ADC_LTR(n)              ((n > ADC_LTR_LT) ? ADC_LTR_LT : n)
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

#if STM32_ADC_USE_ADC3 && !defined(__DOXYGEN__)
extern ADCDriver ADCD3;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void adc_lld_init(void);
  void adc_lld_start(ADCDriver *adcp);
  void adc_lld_stop(ADCDriver *adcp);
  void adc_lld_start_conversion(ADCDriver *adcp);
  void adc_lld_stop_conversion(ADCDriver *adcp);
  void adcSTM32EnableTSVREFE(void);
  void adcSTM32DisableTSVREFE(void);
  void adcSTM32EnableVBATE(void);
  void adcSTM32DisableVBATE(void);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_ADC */

#endif /* HAL_ADC_LLD_H */

/** @} */
