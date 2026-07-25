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
 * @file    STM32F37x/hal_adc_lld.h
 * @brief   STM32F37x ADC subsystem low level driver header.
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
#define ADC_CR2_EXTSEL_SRC(n)   ((n) << 17) /**< @brief Trigger source.     */
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
#define ADC_CHANNEL_SENSOR      16  /**< @brief Internal temperature sensor.*/
#define ADC_CHANNEL_VREFINT     17  /**< @brief Internal reference.         */
#define ADC_CHANNEL_VBAT        18  /**< @brief VBAT.                       */
/** @} */

/**
 * @name    Sampling rates
 * @{
 */
#define ADC_SAMPLE_1P5          0   /**< @brief 1.5 cycles sampling time.   */
#define ADC_SAMPLE_7P5          1   /**< @brief 7.5 cycles sampling time.   */
#define ADC_SAMPLE_13P5         2   /**< @brief 13.5 cycles sampling time.  */
#define ADC_SAMPLE_28P5         3   /**< @brief 28.5 cycles sampling time.  */
#define ADC_SAMPLE_41P5         4   /**< @brief 41.5 cycles sampling time.  */
#define ADC_SAMPLE_55P5         5   /**< @brief 55.5 cycles sampling time.  */
#define ADC_SAMPLE_71P5         6   /**< @brief 71.5 cycles sampling time.  */
#define ADC_SAMPLE_239P5        7   /**< @brief 239.5 cycles sampling time. */
/** @} */

/**
 * @name    SDADC JCHGR bit definitions
 * @{
 */
#define SDADC_JCHG_MASK         (511U << 0)
#define SDADC_JCHG(n)           (1U << (n))
/** @} */

/**
 * @name    SDADC channels definitions
 * @{
 */
#define SDADC_CHANNEL_0         SDADC_JCHG(0)
#define SDADC_CHANNEL_1         SDADC_JCHG(1)
#define SDADC_CHANNEL_2         SDADC_JCHG(2)
#define SDADC_CHANNEL_3         SDADC_JCHG(3)
#define SDADC_CHANNEL_4         SDADC_JCHG(4)
#define SDADC_CHANNEL_5         SDADC_JCHG(5)
#define SDADC_CHANNEL_6         SDADC_JCHG(6)
#define SDADC_CHANNEL_7         SDADC_JCHG(7)
#define SDADC_CHANNEL_8         SDADC_JCHG(8)
#define SDADC_CHANNEL_9         SDADC_JCHG(9)
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
 */
#if !defined(STM32_ADC_USE_ADC1) || defined(__DOXYGEN__)
#define STM32_ADC_USE_ADC1                  FALSE
#endif

/**
 * @brief   SDADC1 driver enable switch.
 * @details If set to @p TRUE the support for SDADC1 is included.
 */
#if !defined(STM32_ADC_USE_SDADC1) || defined(__DOXYGEN__)
#define STM32_ADC_USE_SDADC1                FALSE
#endif

/**
 * @brief   SDADC2 driver enable switch.
 * @details If set to @p TRUE the support for SDADC2 is included.
 */
#if !defined(STM32_ADC_USE_SDADC2) || defined(__DOXYGEN__)
#define STM32_ADC_USE_SDADC2                FALSE
#endif

/**
 * @brief   SDADC3 driver enable switch.
 * @details If set to @p TRUE the support for SDADC3 is included.
 */
#if !defined(STM32_ADC_USE_SDADC3) || defined(__DOXYGEN__)
#define STM32_ADC_USE_SDADC3                FALSE
#endif

/**
 * @brief   ADC1 DMA priority (0..3|lowest..highest).
 */
#if !defined(STM32_ADC_ADC1_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ADC_ADC1_DMA_PRIORITY         2
#endif

/**
 * @brief   SDADC1 DMA priority (0..3|lowest..highest).
 */
#if !defined(STM32_ADC_SDADC1_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ADC_SDADC1_DMA_PRIORITY       2
#endif

/**
 * @brief   SDADC2 DMA priority (0..3|lowest..highest).
 */
#if !defined(STM32_ADC_SDADC2_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ADC_SDADC2_DMA_PRIORITY       2
#endif

/**
 * @brief   SDADC3 DMA priority (0..3|lowest..highest).
 */
#if !defined(STM32_ADC_SDADC3_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ADC_SDADC3_DMA_PRIORITY       2
#endif

/**
 * @brief   ADC interrupt priority level setting.
 */
#if !defined(STM32_ADC_ADC1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ADC_ADC1_IRQ_PRIORITY         5
#endif

/**
 * @brief   ADC DMA interrupt priority level setting.
 */
#if !defined(STM32_ADC_ADC1_DMA_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ADC_ADC1_DMA_IRQ_PRIORITY     5
#endif

/**
 * @brief   SDADC1 interrupt priority level setting.
 */
#if !defined(STM32_ADC_SDADC1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ADC_SDADC1_IRQ_PRIORITY       5
#endif

/**
 * @brief   SDADC2 interrupt priority level setting.
 */
#if !defined(STM32_ADC_SDADC2_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ADC_SDADC2_IRQ_PRIORITY       5
#endif

/**
 * @brief   SDADC3 interrupt priority level setting.
 */
#if !defined(STM32_ADC_SDADC3_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ADC_SDADC3_IRQ_PRIORITY       5
#endif

/**
 * @brief   SDADC1 DMA interrupt priority level setting.
 */
#if !defined(STM32_ADC_SDADC1_DMA_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ADC_SDADC1_DMA_IRQ_PRIORITY   5
#endif

/**
 * @brief   SDADC2 DMA interrupt priority level setting.
 */
#if !defined(STM32_ADC_SDADC2_DMA_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ADC_SDADC2_DMA_IRQ_PRIORITY   5
#endif

/**
 * @brief   SDADC3 DMA interrupt priority level setting.
 */
#if !defined(STM32_ADC_SDADC3_DMA_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ADC_SDADC3_DMA_IRQ_PRIORITY   5
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/**
 * @brief   At least an ADC unit is in use.
 */
#define STM32_ADC_USE_ADC   STM32_ADC_USE_ADC1

/**
 * @brief   At least an SDADC unit is in use.
 */
#define STM32_ADC_USE_SDADC (STM32_ADC_USE_SDADC1 ||                        \
                             STM32_ADC_USE_SDADC2 ||                        \
                             STM32_ADC_USE_SDADC3)

#if STM32_ADC_USE_ADC1 && !STM32_HAS_ADC1
#error "ADC1 not present in the selected device"
#endif

#if STM32_ADC_USE_SDADC1 && !STM32_HAS_SDADC1
#error "SDADC1 not present in the selected device"
#endif

#if STM32_ADC_USE_SDADC2 && !STM32_HAS_SDADC2
#error "SDADC2 not present in the selected device"
#endif

#if STM32_ADC_USE_SDADC3 && !STM32_HAS_SDADC3
#error "SDADC3 not present in the selected device"
#endif

#if !STM32_ADC_USE_ADC && !STM32_ADC_USE_SDADC
#error "ADC driver activated but no ADC/SDADC peripheral assigned"
#endif

#if STM32_ADC_USE_ADC1 &&                                                   \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_ADC_ADC1_IRQ_PRIORITY)
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

#if STM32_ADC_USE_SDADC1 &&                                                 \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_ADC_SDADC1_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to SDADC1"
#endif

#if STM32_ADC_USE_SDADC1 &&                                                 \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_ADC_SDADC1_DMA_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to SDADC1 DMA"
#endif

#if STM32_ADC_USE_SDADC1 &&                                                 \
    !STM32_DMA_IS_VALID_PRIORITY(STM32_ADC_SDADC1_DMA_PRIORITY)
#error "Invalid DMA priority assigned to SDADC1"
#endif

#if STM32_ADC_USE_SDADC2 &&                                                 \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_ADC_SDADC2_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to SDADC2"
#endif

#if STM32_ADC_USE_SDADC2 &&                                                 \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_ADC_SDADC2_DMA_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to SDADC2 DMA"
#endif

#if STM32_ADC_USE_SDADC2 &&                                                 \
    !STM32_DMA_IS_VALID_PRIORITY(STM32_ADC_SDADC2_DMA_PRIORITY)
#error "Invalid DMA priority assigned to SDADC2"
#endif

#if STM32_ADC_USE_SDADC3 &&                                                 \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_ADC_SDADC3_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to SDADC3"
#endif

#if STM32_ADC_USE_SDADC3 &&                                                 \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_ADC_SDADC3_DMA_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to SDADC3 DMA"
#endif

#if STM32_ADC_USE_SDADC3 &&                                                 \
    !STM32_DMA_IS_VALID_PRIORITY(STM32_ADC_SDADC3_DMA_PRIORITY)
#error "Invalid DMA priority assigned to SDADC3"
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
 * @brief   Possible ADC failure causes.
 * @note    Error codes are architecture dependent and should not relied
 *          upon.
 */
typedef enum {
  ADC_ERR_DMAFAILURE = 0,                   /**< DMA operations failure.    */
  ADC_ERR_OVERFLOW = 1,                     /**< ADC overflow condition.    */
  ADC_ERR_AWD1 = 2                          /**< Watchdog 1 triggered.      */
} adcerror_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

#if (STM32_ADC_USE_ADC && STM32_ADC_USE_SDADC) || defined(__DOXYGEN__)
/**
 * @brief   Low level fields of the ADC driver structure.
 */
#define adc_lld_driver_fields                                               \
  /* Pointer to the ADCx registers block.*/                                 \
  ADC_TypeDef               *adc;                                           \
  /* Pointer to the SDADCx registers block.*/                               \
  SDADC_TypeDef             *sdadc;                                         \
  /* Pointer to associated DMA channel.*/                                   \
  const stm32_dma_stream_t  *dmastp;                                        \
  /* DMA mode bit mask.*/                                                   \
  uint32_t                  dmamode;

/**
 * @brief   Low level fields of the ADC configuration structure.
 */
#define adc_lld_config_fields                                               \
  /* SDADC CR1 register initialization data.*/                              \
  uint32_t                  cr1;                                            \
  /* SDADC CONFxR registers initialization data.*/                          \
  uint32_t                  confxr[3];

/**
 * @brief   Low level fields of the ADC configuration structure.
 */
#define adc_lld_configuration_group_fields                                  \
  union {                                                                   \
    struct {                                                                \
      /* ADC CR1 register initialization data.                              \
         NOTE: All the required bits must be defined into this field except \
               @p ADC_CR1_SCAN that is enforced inside the driver.*/        \
      uint32_t                  cr1;                                        \
      /* ADC CR2 register initialization data.                              \
         NOTE: All the required bits must be defined into this field except \
               @p ADC_CR2_DMA, @p ADC_CR2_CONT and @p ADC_CR2_ADON that are \
               enforced inside the driver.*/                                \
      uint32_t                  cr2;                                        \
      /* ADC LTR register initialization data.*/                            \
      uint32_t                  ltr;                                        \
      /* ADC HTR register initialization data.*/                            \
      uint32_t                  htr;                                        \
      /* ADC SMPRx registers initialization data.*/                         \
      uint32_t                  smpr[2];                                    \
      /* ADC SQRx register initialization data.*/                           \
      uint32_t                  sqr[3];                                     \
    } adc;                                                                  \
    struct {                                                                \
      /* SDADC CR2 register initialization data.                            \
         NOTE: Only the @p SDADC_CR2_JSWSTART, @p SDADC_CR2_JEXTSEL         \
               and @p SDADC_CR2_JEXTEN can be specified in this field.*/    \
      uint32_t                  cr2;                                        \
      /* SDADC JCHGR register initialization data.*/                        \
      uint32_t                  jchgr;                                      \
      /* SDADC CONFCHxR registers initialization data.*/                    \
      uint32_t                  confchr[2];                                 \
    } sdadc;                                                                \
  } u;

#elif STM32_ADC_USE_ADC
#define adc_lld_driver_fields                                               \
  /* Pointer to the ADCx registers block.*/                                 \
  ADC_TypeDef               *adc;                                           \
  /* Pointer to associated DMA channel.*/                                   \
  const stm32_dma_stream_t  *dmastp;                                        \
  /* DMA mode bit mask.*/                                                   \
  uint32_t                  dmamode;

#define adc_lld_config_fields                                               \
  /* Dummy configuration, it is not needed.*/                               \
  uint32_t                  dummy;

#define adc_lld_configuration_group_fields                                  \
  union {                                                                   \
    struct {                                                                \
      /* ADC CR1 register initialization data.                              \
         NOTE: All the required bits must be defined into this field except \
               @p ADC_CR1_SCAN that is enforced inside the driver.*/        \
      uint32_t                  cr1;                                        \
      /* ADC CR2 register initialization data.                              \
         NOTE: All the required bits must be defined into this field except \
               @p ADC_CR2_DMA, @p ADC_CR2_CONT and @p ADC_CR2_ADON that are \
               enforced inside the driver.*/                                \
      uint32_t                  cr2;                                        \
      /* ADC LTR register initialization data.*/                            \
      uint32_t                  ltr;                                        \
      /* ADC HTR register initialization data.*/                            \
      uint32_t                  htr;                                        \
      /* ADC SMPRx registers initialization data.*/                         \
      uint32_t                  smpr[2];                                    \
      /* ADC SQRx register initialization data.*/                           \
      uint32_t                  sqr[3];                                     \
    } adc;                                                                  \
  } u;

#elif STM32_ADC_USE_SDADC
#define adc_lld_driver_fields                                               \
  /* Pointer to the SDADCx registers block.*/                               \
  SDADC_TypeDef             *sdadc;                                         \
  /* Pointer to associated DMA channel.*/                                   \
  const stm32_dma_stream_t  *dmastp;                                        \
  /* DMA mode bit mask.*/                                                   \
  uint32_t                  dmamode;

#define adc_lld_config_fields                                               \
  /* SDADC CR1 register initialization data.*/                              \
  uint32_t                  cr1;                                            \
  /* SDADC CONFxR registers initialization data.*/                          \
  uint32_t                  confxr[3];

#define adc_lld_configuration_group_fields                                  \
  union {                                                                   \
    struct {                                                                \
      /* SDADC CR2 register initialization data.                            \
         NOTE: Only the @p SDADC_CR2_JSWSTART, @p SDADC_CR2_JEXTSEL         \
               and @p SDADC_CR2_JEXTEN can be specified in this field.*/    \
      uint32_t                  cr2;                                        \
      /* SDADC JCHGR register initialization data.*/                        \
      uint32_t                  jchgr;                                      \
      /* SDADC CONFCHxR registers initialization data.*/                    \
      uint32_t                  confchr[2];                                 \
    } sdadc;                                                                \
  } u;

#endif

/**
 * @name    Sequences building helper macros for ADC
 * @{
 */
/**
 * @brief   Number of channels in a conversion sequence.
 */
#define ADC_SQR1_NUM_CH(n)      (((n) - 1) << 20)
#define ADC_SQR1_SQ13_N(n)      ((n) << 0)  /**< @brief 13th channel in seq.*/
#define ADC_SQR1_SQ14_N(n)      ((n) << 5)  /**< @brief 14th channel in seq.*/
#define ADC_SQR1_SQ15_N(n)      ((n) << 10) /**< @brief 15th channel in seq.*/
#define ADC_SQR1_SQ16_N(n)      ((n) << 15) /**< @brief 16th channel in seq.*/

#define ADC_SQR2_SQ7_N(n)       ((n) << 0)  /**< @brief 7th channel in seq. */
#define ADC_SQR2_SQ8_N(n)       ((n) << 5)  /**< @brief 8th channel in seq. */
#define ADC_SQR2_SQ9_N(n)       ((n) << 10) /**< @brief 9th channel in seq. */
#define ADC_SQR2_SQ10_N(n)      ((n) << 15) /**< @brief 10th channel in seq.*/
#define ADC_SQR2_SQ11_N(n)      ((n) << 20) /**< @brief 11th channel in seq.*/
#define ADC_SQR2_SQ12_N(n)      ((n) << 25) /**< @brief 12th channel in seq.*/

#define ADC_SQR3_SQ1_N(n)       ((n) << 0)  /**< @brief 1st channel in seq. */
#define ADC_SQR3_SQ2_N(n)       ((n) << 5)  /**< @brief 2nd channel in seq. */
#define ADC_SQR3_SQ3_N(n)       ((n) << 10) /**< @brief 3rd channel in seq. */
#define ADC_SQR3_SQ4_N(n)       ((n) << 15) /**< @brief 4th channel in seq. */
#define ADC_SQR3_SQ5_N(n)       ((n) << 20) /**< @brief 5th channel in seq. */
#define ADC_SQR3_SQ6_N(n)       ((n) << 25) /**< @brief 6th channel in seq. */
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
 * @name    Sequences building helper macros for SDADC
 * @{
 */
#define SDADC_JCHGR_CH(n)           (1U << (n))
/** @} */

/**
 * @name    Channel configuration number helper macros for SDADC
 * @{
 */
#define SDADC_CONFCHR1_CH0(n)       ((n) << 0)
#define SDADC_CONFCHR1_CH1(n)       ((n) << 4)
#define SDADC_CONFCHR1_CH2(n)       ((n) << 8)
#define SDADC_CONFCHR1_CH3(n)       ((n) << 12)
#define SDADC_CONFCHR1_CH4(n)       ((n) << 16)
#define SDADC_CONFCHR1_CH5(n)       ((n) << 20)
#define SDADC_CONFCHR1_CH6(n)       ((n) << 24)
#define SDADC_CONFCHR1_CH7(n)       ((n) << 28)
#define SDADC_CONFCHR2_CH8(n)       ((n) << 0)
/** @} */

/**
 * @name    Configuration registers helper macros for SDADC
 * @{
 */
#define SDADC_CONFR_OFFSET_MASK     (0xFFFU << 0)
#define SDADC_CONFR_OFFSET(n)       ((n) << 0)
#define SDADC_CONFR_GAIN_MASK       (7U << 20)
#define SDADC_CONFR_GAIN_1X         (0U << 20)
#define SDADC_CONFR_GAIN_2X         (1U << 20)
#define SDADC_CONFR_GAIN_4X         (2U << 20)
#define SDADC_CONFR_GAIN_8X         (3U << 20)
#define SDADC_CONFR_GAIN_16X        (4U << 20)
#define SDADC_CONFR_GAIN_32X        (5U << 20)
#define SDADC_CONFR_GAIN_0P5X       (7U << 20)
#define SDADC_CONFR_SE_MASK         (3U << 26)
#define SDADC_CONFR_SE_DIFF         (0U << 26)
#define SDADC_CONFR_SE_OFFSET       (1U << 26)
#define SDADC_CONFR_SE_ZERO_VOLT    (3U << 26)
#define SDADC_CONFR_COMMON_MASK     (3U << 30)
#define SDADC_CONFR_COMMON_VSSSD    (0U << 30)
#define SDADC_CONFR_COMMON_VDDSD2   (1U << 30)
#define SDADC_CONFR_COMMON_VDDSD    (2U << 30)
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if STM32_ADC_USE_ADC1 && !defined(__DOXYGEN__)
extern ADCDriver ADCD1;
#endif

#if STM32_ADC_USE_SDADC1 && !defined(__DOXYGEN__)
extern ADCDriver SDADCD1;
#endif

#if STM32_ADC_USE_SDADC2 && !defined(__DOXYGEN__)
extern ADCDriver SDADCD2;
#endif

#if STM32_ADC_USE_SDADC3 && !defined(__DOXYGEN__)
extern ADCDriver SDADCD3;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void adc_lld_init(void);
  void adc_lld_start(ADCDriver *adcp);
  void adc_lld_stop(ADCDriver *adcp);
  void adc_lld_start_conversion(ADCDriver *adcp);
  void adc_lld_stop_conversion(ADCDriver *adcp);
  void adcSTM32Calibrate(ADCDriver *adcdp);
#if STM32_ADC_USE_ADC
  void adcSTM32EnableTSVREFE(void);
  void adcSTM32DisableTSVREFE(void);
  void adcSTM32EnableVBATE(void);
  void adcSTM32DisableVBATE(void);
#endif /* STM32_ADC_USE_ADC */
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_ADC */

#endif /* HAL_ADC_LLD_H */

/** @} */
