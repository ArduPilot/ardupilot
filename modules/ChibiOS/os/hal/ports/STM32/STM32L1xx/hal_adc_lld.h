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
 * @file    STM32L1xx/hal_adc_lld.h
 * @brief   STM32L1xx ADC subsystem low level driver header.
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
 * @name    Triggers selection
 * @{
 */
#define ADC_CR2_EXTSEL_SRC(n)   ((n) << 24) /**< @brief Trigger source.     */
/** @} */

/**
 * @name    ADC clock divider settings
 * @{
 */
#define ADC_CCR_ADCPRE_DIV1     0
#define ADC_CCR_ADCPRE_DIV2     1
#define ADC_CCR_ADCPRE_DIV4     2
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
#define ADC_CHANNEL_SENSOR      16  /**< @brief Internal temperature sensor.*/
#define ADC_CHANNEL_VREFINT     17  /**< @brief Internal reference.         */
#define ADC_CHANNEL_IN18        18  /**< @brief External analog input 18.   */
#define ADC_CHANNEL_IN19        19  /**< @brief External analog input 19.   */
#define ADC_CHANNEL_IN20        20  /**< @brief External analog input 20.   */
#define ADC_CHANNEL_IN21        21  /**< @brief External analog input 21.   */
#define ADC_CHANNEL_IN22        22  /**< @brief External analog input 22.   */
#define ADC_CHANNEL_IN23        23  /**< @brief External analog input 23.   */
#define ADC_CHANNEL_IN24        24  /**< @brief External analog input 24.   */
#define ADC_CHANNEL_IN25        25  /**< @brief External analog input 25.   */
/** @} */

/**
 * @name    Sampling rates
 * @{
 */
#define ADC_SAMPLE_4            0   /**< @brief 4 cycles sampling time.     */
#define ADC_SAMPLE_9            1   /**< @brief 9 cycles sampling time.     */
#define ADC_SAMPLE_16           2   /**< @brief 16 cycles sampling time.    */
#define ADC_SAMPLE_24           3   /**< @brief 24 cycles sampling time.    */
#define ADC_SAMPLE_48           4   /**< @brief 48 cycles sampling time.    */
#define ADC_SAMPLE_96           5   /**< @brief 96 cycles sampling time.    */
#define ADC_SAMPLE_192          6   /**< @brief 192 cycles sampling time.   */
#define ADC_SAMPLE_384          7   /**< @brief 384 cycles sampling time.   */
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
 * @note    The default is @p TRUE.
 */
#if !defined(STM32_ADC_USE_ADC1) || defined(__DOXYGEN__)
#define STM32_ADC_USE_ADC1                  FALSE
#endif

/**
 * @brief   ADC common clock divider.
 * @note    This setting is influenced by the VDDA voltage and other
 *          external conditions, please refer to the STM32L15x datasheet
 *          for more info.<br>
 *          See section 6.3.15 "12-bit ADC characteristics".
 */
#if !defined(STM32_ADC_ADCPRE) || defined(__DOXYGEN__)
#define STM32_ADC_ADCPRE                    ADC_CCR_ADCPRE_DIV1
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
#if !defined(STM32_ADC_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ADC_IRQ_PRIORITY              5
#endif

/**
 * @brief   ADC1 DMA interrupt priority level setting.
 */
#if !defined(STM32_ADC_ADC1_DMA_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ADC_ADC1_DMA_IRQ_PRIORITY     5
#endif

/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if STM32_ADC_USE_ADC1 && !STM32_HAS_ADC1
#error "ADC1 not present in the selected device"
#endif

#if !STM32_ADC_USE_ADC1
#error "ADC driver activated but no ADC peripheral assigned"
#endif

#if STM32_ADC_USE_ADC1 &&                                                   \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_ADC_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to ADC1"
#endif

#if STM32_ADC_USE_ADC1 &&                                                   \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_ADC_ADC1_DMA_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to ADC1 DMA"
#endif

#if STM32_ADC_USE_ADC1 &&                                                   \
    !STM32_DMA_IS_VALID_PRIORITY(STM32_ADC_ADC1_DMA_PRIORITY)
#error "Invalid DMA priority assigned to ADC1"
#endif

#if !defined(STM32_DMA_REQUIRED)
#define STM32_DMA_REQUIRED
#endif

/**
 *  * @brief   ADC frequency.
 *   */
/* ADC clock related settings and checks.*/
#if STM32_ADC_ADCPRE == ADC_CCR_ADCPRE_DIV1
#define STM32_ADCCLK                        STM32_HSICLK
#elif STM32_ADC_ADCPRE == ADC_CCR_ADCPRE_DIV2
#define STM32_ADCCLK                        (STM32_HSICLK / 2)
#elif STM32_ADC_ADCPRE == ADC_CCR_ADCPRE_DIV4
#define STM32_ADCCLK                        (STM32_HSICLK / 4)
#else
#error "invalid STM32_ADC_ADCPRE value specified"
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
 * @brief   Possible ADC failure causes.
 * @note    Error codes are architecture dependent and should not relied
 *          upon.
 */
typedef enum {
  ADC_ERR_DMAFAILURE = 0,                   /**< DMA operations failure.    */
  ADC_ERR_OVERFLOW = 1                      /**< ADC overflow condition.    */
} adcerror_t;

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
           20...25.*/                                                       \
  uint32_t                  smpr1;                                          \
  /* ADC SMPR2 register initialization data.                                \
     NOTE: In this field must be specified the sample times for channels    \
           10...19.*/                                                       \
  uint32_t                  smpr2;                                          \
  /* ADC SMPR3 register initialization data.                                \
     NOTE: In this field must be specified the sample times for channels    \
           0...9.*/                                                         \
  uint32_t                  smpr3;                                          \
  /* ADC SQR1 register initialization data.                                 \
     NOTE: Conversion group sequence 25...27 + sequence length.*/           \
  uint32_t                  sqr1;                                           \
  /* ADC SQR2 register initialization data.                                 \
     NOTE: Conversion group sequence 19...24.*/                             \
  uint32_t                  sqr2;                                           \
  /* ADC SQR3 register initialization data.                                 \
     NOTE: Conversion group sequence 13...18.*/                             \
  uint32_t                  sqr3;                                           \
  /* ADC SQR3 register initialization data.                                 \
     NOTE: Conversion group sequence 7...12.*/                              \
  uint32_t                  sqr4;                                           \
  /* ADC SQR3 register initialization data.                                 \
     NOTE: Conversion group sequence 1...6.*/                               \
  uint32_t                  sqr5;

/**
 * @name    Sequences building helper macros
 * @{
 */
/**
 * @brief   Number of channels in a conversion sequence.
 */
#define ADC_SQR1_NUM_CH(n)      (((n) - 1) << 20)

#define ADC_SQR5_SQ1_N(n)       ((n) << 0)  /**< @brief 1st channel in seq. */
#define ADC_SQR5_SQ2_N(n)       ((n) << 5)  /**< @brief 2nd channel in seq. */
#define ADC_SQR5_SQ3_N(n)       ((n) << 10) /**< @brief 3rd channel in seq. */
#define ADC_SQR5_SQ4_N(n)       ((n) << 15) /**< @brief 4th channel in seq. */
#define ADC_SQR5_SQ5_N(n)       ((n) << 20) /**< @brief 5th channel in seq. */
#define ADC_SQR5_SQ6_N(n)       ((n) << 25) /**< @brief 6th channel in seq. */

#define ADC_SQR4_SQ7_N(n)       ((n) << 0)  /**< @brief 7th channel in seq. */
#define ADC_SQR4_SQ8_N(n)       ((n) << 5)  /**< @brief 8th channel in seq. */
#define ADC_SQR4_SQ9_N(n)       ((n) << 10) /**< @brief 9th channel in seq. */
#define ADC_SQR4_SQ10_N(n)      ((n) << 15) /**< @brief 10th channel in seq.*/
#define ADC_SQR4_SQ11_N(n)      ((n) << 20) /**< @brief 11th channel in seq.*/
#define ADC_SQR4_SQ12_N(n)      ((n) << 25) /**< @brief 12th channel in seq.*/

#define ADC_SQR3_SQ13_N(n)      ((n) << 0)  /**< @brief 13th channel in seq.*/
#define ADC_SQR3_SQ14_N(n)      ((n) << 5)  /**< @brief 14th channel in seq.*/
#define ADC_SQR3_SQ15_N(n)      ((n) << 10) /**< @brief 15th channel in seq.*/
#define ADC_SQR3_SQ16_N(n)      ((n) << 15) /**< @brief 16th channel in seq.*/
#define ADC_SQR3_SQ17_N(n)      ((n) << 20) /**< @brief 17th channel in seq.*/
#define ADC_SQR3_SQ18_N(n)      ((n) << 25) /**< @brief 18th channel in seq.*/

#define ADC_SQR2_SQ19_N(n)      ((n) << 0)  /**< @brief 19th channel in seq.*/
#define ADC_SQR2_SQ20_N(n)      ((n) << 5)  /**< @brief 20th channel in seq.*/
#define ADC_SQR2_SQ21_N(n)      ((n) << 10) /**< @brief 21th channel in seq.*/
#define ADC_SQR2_SQ22_N(n)      ((n) << 15) /**< @brief 22th channel in seq.*/
#define ADC_SQR2_SQ23_N(n)      ((n) << 20) /**< @brief 23th channel in seq.*/
#define ADC_SQR2_SQ24_N(n)      ((n) << 25) /**< @brief 24th channel in seq.*/

#define ADC_SQR1_SQ25_N(n)      ((n) << 0)  /**< @brief 25th channel in seq.*/
#define ADC_SQR1_SQ26_N(n)      ((n) << 5)  /**< @brief 26th channel in seq.*/
#define ADC_SQR1_SQ27_N(n)      ((n) << 10) /**< @brief 27th channel in seq.*/
/** @} */

/**
 * @name    Sampling rate settings helper macros
 * @{
 */
#define ADC_SMPR3_SMP_AN0(n)    ((n) << 0)  /**< @brief AN0 sampling time.  */
#define ADC_SMPR3_SMP_AN1(n)    ((n) << 3)  /**< @brief AN1 sampling time.  */
#define ADC_SMPR3_SMP_AN2(n)    ((n) << 6)  /**< @brief AN2 sampling time.  */
#define ADC_SMPR3_SMP_AN3(n)    ((n) << 9)  /**< @brief AN3 sampling time.  */
#define ADC_SMPR3_SMP_AN4(n)    ((n) << 12) /**< @brief AN4 sampling time.  */
#define ADC_SMPR3_SMP_AN5(n)    ((n) << 15) /**< @brief AN5 sampling time.  */
#define ADC_SMPR3_SMP_AN6(n)    ((n) << 18) /**< @brief AN6 sampling time.  */
#define ADC_SMPR3_SMP_AN7(n)    ((n) << 21) /**< @brief AN7 sampling time.  */
#define ADC_SMPR3_SMP_AN8(n)    ((n) << 24) /**< @brief AN8 sampling time.  */
#define ADC_SMPR3_SMP_AN9(n)    ((n) << 27) /**< @brief AN9 sampling time.  */

#define ADC_SMPR2_SMP_AN10(n)   ((n) << 0)  /**< @brief AN10 sampling time. */
#define ADC_SMPR2_SMP_AN11(n)   ((n) << 3)  /**< @brief AN11 sampling time. */
#define ADC_SMPR2_SMP_AN12(n)   ((n) << 6)  /**< @brief AN12 sampling time. */
#define ADC_SMPR2_SMP_AN13(n)   ((n) << 9)  /**< @brief AN13 sampling time. */
#define ADC_SMPR2_SMP_AN14(n)   ((n) << 12) /**< @brief AN14 sampling time. */
#define ADC_SMPR2_SMP_AN15(n)   ((n) << 15) /**< @brief AN15 sampling time. */
#define ADC_SMPR2_SMP_SENSOR(n) ((n) << 18) /**< @brief Temperature Sensor
                                                 sampling time.             */
#define ADC_SMPR2_SMP_VREF(n)   ((n) << 21) /**< @brief Voltage Reference
                                                 sampling time.             */
#define ADC_SMPR2_SMP_AN18(n)   ((n) << 24) /**< @brief AN18 sampling time. */
#define ADC_SMPR2_SMP_AN19(n)   ((n) << 27) /**< @brief AN19 sampling time. */

#define ADC_SMPR1_SMP_AN20(n)   ((n) << 0)  /**< @brief AN20 sampling time. */
#define ADC_SMPR1_SMP_AN21(n)   ((n) << 3)  /**< @brief AN21 sampling time. */
#define ADC_SMPR1_SMP_AN22(n)   ((n) << 6)  /**< @brief AN22 sampling time. */
#define ADC_SMPR1_SMP_AN23(n)   ((n) << 9)  /**< @brief AN23 sampling time. */
#define ADC_SMPR1_SMP_AN24(n)   ((n) << 12) /**< @brief AN24 sampling time. */
#define ADC_SMPR1_SMP_AN25(n)   ((n) << 15) /**< @brief AN25 sampling time. */
/** @} */

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
  void adcSTM32EnableTSVREFE(void);
  void adcSTM32DisableTSVREFE(void);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_ADC */

#endif /* HAL_ADC_LLD_H */

/** @} */
