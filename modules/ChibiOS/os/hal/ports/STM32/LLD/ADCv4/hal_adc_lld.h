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
 * @file    ADCv4/hal_adc_lld.h
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
#define ADC_CHANNEL_IN19        19U /**< @brief External analog input 19.   */
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
#define ADC_SELMASK_IN19        (1U << ADC_CHANNEL_IN19)
/** @} */

/**
 * @name    Sampling rates
 * @{
 */
#if defined(STM32H7XX)
#define ADC_SMPR_SMP_1P5        0U  /**< @brief 9 cycles conversion time    */
#define ADC_SMPR_SMP_2P5        1U  /**< @brief 10 cycles conversion time.  */
#define ADC_SMPR_SMP_8P5        2U  /**< @brief 16 cycles conversion time.  */
#define ADC_SMPR_SMP_16P5       3U  /**< @brief 24 cycles conversion time.  */
#define ADC_SMPR_SMP_32P5       4U  /**< @brief 40 cycles conversion time.  */
#define ADC_SMPR_SMP_64P5       5U  /**< @brief 72 cycles conversion time.  */
#define ADC_SMPR_SMP_384P5      6U  /**< @brief 392 cycles conversion time. */
#define ADC_SMPR_SMP_810P5      7U  /**< @brief 818 cycles conversion time. */
#endif
/** @} */

/**
 * @name    CFGR register configuration helpers
 * @{
 */
#define ADC_CFGR_DMNGT_MASK             (3U << 0U)
#define ADC_CFGR_DMNGT_NODMA            (0U << 0U)
#define ADC_CFGR_DMNGT_ONESHOT          (1U << 0U)
#define ADC_CFGR_DMNGT_DFSDM            (2U << 0U)
#define ADC_CFGR_DMNGT_CIRCULAR         (3U << 0U)

#define ADC_CFGR_RES_MASK               (7U << 2U)
#define ADC_CFGR_RES_16BITS             (0U << 2U)
#define ADC_CFGR_RES_10BITS             (3U << 2U)
#if !defined(STM32_ENFORCE_H7_REV_XY)
#define ADC_CFGR_RES_14BITS             (5U << 2U)
#define ADC_CFGR_RES_12BITS             (6U << 2U)
#define ADC_CFGR_RES_8BITS              (7U << 2U)
#else
#define ADC_CFGR_RES_14BITS             (1U << 2U)
#define ADC_CFGR_RES_12BITS             (2U << 2U)
#define ADC_CFGR_RES_8BITS              (4U << 2U)
#endif

#define ADC_CFGR_EXTSEL_MASK            (15U << 5U)
#define ADC_CFGR_EXTSEL_SRC(n)          ((n) << 5U)

#define ADC_CFGR_EXTEN_MASK             (3U << 10U)
#define ADC_CFGR_EXTEN_DISABLED         (0U << 10U)
#define ADC_CFGR_EXTEN_RISING           (1U << 10U)
#define ADC_CFGR_EXTEN_FALLING          (2U << 10U)
#define ADC_CFGR_EXTEN_BOTH             (3U << 10U)

#define ADC_CFGR_CONT_MASK            (1U << 13U)
#define ADC_CFGR_CONT_DISABLED        (0U << 13U)
#define ADC_CFGR_CONT_ENABLED         (1U << 13U)

#define ADC_CFGR_DISCEN_MASK            (1U << 16U)
#define ADC_CFGR_DISCEN_DISABLED        (0U << 16U)
#define ADC_CFGR_DISCEN_ENABLED         (1U << 16U)

#define ADC_CFGR_DISCNUM_MASK           (7U << 17U)
#define ADC_CFGR_DISCNUM_VAL(n)         ((n) << 17U)
/** @} */

/**
 * @name    CCR register configuration helpers
 * @{
 */
#define ADC_CCR_DUAL_MASK               (31U << 0U)
#define ADC_CCR_DUAL_FIELD(n)           ((n) << 0U)
#define ADC_CCR_DUAL_INDEPENDENT        (0U << 0U)  /**< @brief Independent, dual mode disabled.                             */
#define ADC_CCR_DUAL_REG_SIMULT         (6U << 0U)  /**< @brief Regular simultaneous.                                        */
#define ADC_CCR_DUAL_REG_INTERL         (7U << 0U)  /**< @brief Regular interleaved.                                         */
#define ADC_CCR_DUAL_INJ_SIMULT         (5U << 0U)  /**< @brief Injected simultaneous.                                       */
#define ADC_CCR_DUAL_INJ_ALTERNATE      (9U << 0U)  /**< @brief Injected alternate trigger.                                  */
#define ADC_CCR_DUAL_REG_SIM_INJ_SIM    (1U << 0U)  /**< @brief Combined regular simultaneous + injected simultaneous.       */
#define ADC_CCR_DUAL_REG_SIM_INJ_ALT    (2U << 0U)  /**< @brief Combined regular simultaneous + injected alternate trigger.  */
#define ADC_CCR_DUAL_REG_INT_INJ_SIM    (3U << 0U)  /**< @brief Combined regular interleaved  + injected simultaneous.       */
#define ADC_CCR_DELAY_MASK              (15U << 8U)
#define ADC_CCR_DELAY_FIELD(n)          ((n) << 8U)
#define ADC_CCR_DAMDF_MASK              (3U << 14U)
#define ADC_CCR_DAMDF_DISABLED          (0U << 14U)
#define ADC_CCR_DAMDF_HWORD             (2U << 14U)
#define ADC_CCR_DAMDF_BYTE              (3U << 14U)
#define ADC_CCR_CKMODE_MASK             (3U << 16U)
#define ADC_CCR_CKMODE_ADCCK            (0U << 16U)
#define ADC_CCR_CKMODE_AHB_DIV1         (1U << 16U)
#define ADC_CCR_CKMODE_AHB_DIV2         (2U << 16U)
#define ADC_CCR_CKMODE_AHB_DIV4         (3U << 16U)
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   Enables the ADC1 and ADC2 master/slave mode.
 */
#if !defined(STM32_ADC_DUAL_MODE) || defined(__DOXYGEN__)
#define STM32_ADC_DUAL_MODE                 FALSE
#endif

/**
 * @brief   Specifies the ADC samples width.
 * @note    Must be 8, 16 or 32.
 * @note    10, 12, 14 and 16 bits sampling modes must not be used when
 *          this option is set to 8.
 * @note    32 is useful when oversampling is activated.
 */
#if !defined(STM32_ADC_SAMPLES_SIZE) || defined(__DOXYGEN__)
#define STM32_ADC_SAMPLES_SIZE               16
#endif

/**
 * @brief   ADC1/ADC2 driver enable switch.
 * @details If set to @p TRUE the support for ADC1/ADC2 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_ADC_USE_ADC12) || defined(__DOXYGEN__)
#define STM32_ADC_USE_ADC12                 FALSE
#endif

/**
 * @brief   ADC3 driver enable switch.
 * @details If set to @p TRUE the support for ADC3 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_ADC_USE_ADC3) || defined(__DOXYGEN__)
#define STM32_ADC_USE_ADC3                  FALSE
#endif

/**
 * @brief   ADC3 BDMA enable switch.
 * @details If set to @p TRUE the ADC3 uses BDMA.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_ADC_ADC3_USE_BDMA) || defined(__DOXYGEN__)
#define STM32_ADC_ADC3_USE_BDMA             FALSE
#endif

/**
 * @brief   ADC1/ADC2 DMA priority (0..3|lowest..highest).
 */
#if !defined(STM32_ADC_ADC12_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ADC_ADC12_DMA_PRIORITY        2
#endif

/**
 * @brief   ADC3 DMA priority (0..3|lowest..highest).
 */
#if !defined(STM32_ADC_ADC3_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ADC_ADC3_DMA_PRIORITY         2
#endif

/**
 * @brief   ADC1/ADC2 interrupt priority level setting.
 */
#if !defined(STM32_ADC_ADC12_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ADC_ADC12_IRQ_PRIORITY        5
#endif

/**
 * @brief   ADC3 interrupt priority level setting.
 */
#if !defined(STM32_ADC_ADC3_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ADC_ADC3_IRQ_PRIORITY         5
#endif

/**
 * @brief   ADC1/ADC2 clock source and mode.
 */
#if !defined(STM32_ADC_ADC12_CLOCK_MODE) || defined(__DOXYGEN__)
#define STM32_ADC_ADC12_CLOCK_MODE          ADC_CCR_CKMODE_AHB_DIV4
#endif

/**
 * @brief   ADC3 clock source and mode.
 */
#if !defined(STM32_ADC_ADC3_CLOCK_MODE) || defined(__DOXYGEN__)
#define STM32_ADC_ADC3_CLOCK_MODE           ADC_CCR_CKMODE_AHB_DIV4
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Supported devices checks.*/
#if !defined(STM32H7XX)
#error "ADCv4 only supports H7 STM32 devices"
#endif

/* Registry checks.*/
#if !defined(STM32_HAS_ADC1) || !defined(STM32_HAS_ADC2) ||                 \
    !defined(STM32_HAS_ADC3)
#error "STM32_HAS_ADCx not defined in registry"
#endif

/* Units checks.*/
#if STM32_ADC_USE_ADC12 && !STM32_HAS_ADC1
#error "ADC1 not present in the selected device"
#endif

#if STM32_ADC_DUAL_MODE && !STM32_HAS_ADC2
#error "ADC2 not present in the selected device"
#endif

#if STM32_ADC_USE_ADC3 && !STM32_HAS_ADC3
#error "ADC3 not present in the selected device"
#endif

/* IRQ handlers checks.*/
#if STM32_HAS_ADC1 && !defined(STM32_ADC12_HANDLER)
#error "STM32_ADC12_HANDLER not defined in registry"
#endif

#if STM32_HAS_ADC2 && !defined(STM32_ADC12_HANDLER)
#error "STM32_ADC12_HANDLER not defined in registry"
#endif

#if STM32_HAS_ADC3 && !defined(STM32_ADC3_HANDLER)
#error "STM32_ADC3_HANDLER not defined in registry"
#endif

/* IRQ vector numbers checks.*/
#if STM32_HAS_ADC1 && !defined(STM32_ADC12_NUMBER)
#error "STM32_ADC12_NUMBER not defined in registry"
#endif

#if STM32_HAS_ADC2 && !defined(STM32_ADC12_NUMBER)
#error "STM32_ADC12_NUMBER not defined in registry"
#endif

#if STM32_HAS_ADC3 && !defined(STM32_ADC3_NUMBER)
#error "STM32_ADC3_NUMBER not defined in registry"
#endif

/* At least one ADC must be assigned.*/
#if !STM32_ADC_USE_ADC12 && !STM32_ADC_USE_ADC3
#error "ADC driver activated but no ADC peripheral assigned"
#endif

/* Dual mode is only supported with ADC12.*/
#if !STM32_ADC_USE_ADC12 && STM32_ADC_DUAL_MODE
#error "STM32_ADC_DUAL_MODE only supported with ADC12"
#endif

/* Check on the presence of the DMA streams settings in mcuconf.h.*/
#if STM32_ADC_USE_ADC12 && !defined(STM32_ADC_ADC12_DMA_STREAM)
#error "STM32_ADC_ADC12_DMA_STREAM not defined"
#endif

/* Check ADC3 DMA stream settings in mcuconf.h.*/
#if STM32_ADC_USE_ADC3 && !defined(STM32_ADC_ADC3_BDMA_STREAM) &&           \
                          !defined(STM32_ADC_ADC3_DMA_STREAM)
#error "STM32_ADC_ADC3_DMA_STREAM or STM32_ADC_ADC3_BDMA_STREAM must be defined"
#endif

/* Check the assignment of BDMA stream for ADC3.*/
#if STM32_ADC_USE_ADC3 && STM32_ADC_ADC3_USE_BDMA
#if !STM32_BDMA_IS_VALID_STREAM(STM32_ADC_ADC3_BDMA_STREAM)
#error "Invalid BDMA channel assigned to ADC3"
#endif
#endif

/* Check the assignment of DMA stream for ADC3.*/
#if STM32_ADC_USE_ADC3 && !STM32_ADC_ADC3_USE_BDMA
#if !STM32_DMA_IS_VALID_STREAM(STM32_ADC_ADC3_DMA_STREAM)
#error "Invalid DMA channel assigned to ADC3"
#endif
#endif

/* DMA channel range tests.*/
#if STM32_ADC_USE_ADC12 &&                                                  \
    !STM32_DMA_IS_VALID_STREAM(STM32_ADC_ADC12_DMA_STREAM)
#error "Invalid DMA channel assigned to ADC12"
#endif

/* DMA priority tests.*/
#if STM32_ADC_USE_ADC12 &&                                                  \
    !STM32_DMA_IS_VALID_PRIORITY(STM32_ADC_ADC12_DMA_PRIORITY)
#error "Invalid DMA priority assigned to ADC1"
#endif

#if STM32_ADC_USE_ADC3 &&                                                   \
    !STM32_DMA_IS_VALID_PRIORITY(STM32_ADC_ADC3_DMA_PRIORITY)
#error "Invalid DMA priority assigned to ADC3"
#endif

/* ADC IRQ priority tests.*/
#if STM32_ADC_USE_ADC12 &&                                                  \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_ADC_ADC12_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to ADC1"
#endif

#if STM32_ADC_USE_ADC3 &&                                                   \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_ADC_ADC3_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to ADC3"
#endif

#if ((STM32_ADC_SAMPLES_SIZE != 8)  &&                                      \
     (STM32_ADC_SAMPLES_SIZE != 16) &&                                      \
     (STM32_ADC_SAMPLES_SIZE != 32))
#error "STM32_ADC_SAMPLES_SIZE must be 8, 16 or 32"
#endif

#if (STM32_ADC_SAMPLES_SIZE != 32) && STM32_ADC_DUAL_MODE
#error "STM32_ADC_SAMPLES_SIZE != 32 not compatible with STM32_ADC_DUAL_MODE"
#endif

/* ADC clock source checks.*/
#define STM32_ADC_SCLK                  STM32_HCLK

#if STM32_ADC_ADC12_CLOCK_MODE == ADC_CCR_CKMODE_ADCCK
/* CHTODO: also check ADC_CCR_PRESC.*/
#define STM32_ADC12_CLOCK               STM32_ADCCLK
#elif STM32_ADC_ADC12_CLOCK_MODE == ADC_CCR_CKMODE_AHB_DIV1
#define STM32_ADC12_CLOCK               (STM32_ADC_SCLK / 1)
#elif STM32_ADC_ADC12_CLOCK_MODE == ADC_CCR_CKMODE_AHB_DIV2
#define STM32_ADC12_CLOCK               (STM32_ADC_SCLK / 2)
#elif STM32_ADC_ADC12_CLOCK_MODE == ADC_CCR_CKMODE_AHB_DIV4
#define STM32_ADC12_CLOCK               (STM32_ADC_SCLK / 4)
#else
#error "invalid clock mode selected for STM32_ADC_ADC12_CLOCK_MODE"
#endif

#if STM32_ADC_ADC3_CLOCK_MODE == ADC_CCR_CKMODE_ADCCK
/* CHTODO: also check ADC_CCR_PRESC.*/
#define STM32_ADC3_CLOCK                STM32_ADCCLK
#elif STM32_ADC_ADC3_CLOCK_MODE == ADC_CCR_CKMODE_AHB_DIV1
#define STM32_ADC3_CLOCK                (STM32_ADC_SCLK / 1)
#elif STM32_ADC_ADC3_CLOCK_MODE == ADC_CCR_CKMODE_AHB_DIV2
#define STM32_ADC3_CLOCK                (STM32_ADC_SCLK / 2)
#elif STM32_ADC_ADC3_CLOCK_MODE == ADC_CCR_CKMODE_AHB_DIV4
#define STM32_ADC3_CLOCK                (STM32_ADC_SCLK / 4)
#else
#error "invalid clock mode selected for STM32_ADC_ADC3_CLOCK_MODE"
#endif

#if STM32_ADC12_CLOCK > STM32_ADCCLK_MAX
#error "STM32_ADC12_CLOCK exceeding maximum frequency (STM32_ADCCLK_MAX)"
#endif

#if STM32_ADC3_CLOCK > STM32_ADCCLK_MAX
#error "STM32_ADC3_CLOCK exceeding maximum frequency (STM32_ADCCLK_MAX)"
#endif

#if !defined(STM32_ENFORCE_H7_REV_XY)
/* ADC boost checks.*/
#if   STM32_ADC12_CLOCK >  6250000
#define STM32_ADC12_BOOST               (1U << 8U)
#elif STM32_ADC12_CLOCK > 12500000
#define STM32_ADC12_BOOST               (2U << 8U)
#elif STM32_ADC12_CLOCK > 25000000
#define STM32_ADC12_BOOST               (3U << 8U)
#else
#define STM32_ADC12_BOOST               (0U << 8U)
#endif

#if   STM32_ADC3_CLOCK >  6250000
#define STM32_ADC3_BOOST                (1U << 8U)
#elif STM32_ADC3_CLOCK > 12500000
#define STM32_ADC3_BOOST                (2U << 8U)
#elif STM32_ADC3_CLOCK > 25000000
#define STM32_ADC3_BOOST                (3U << 8U)
#else
#define STM32_ADC3_BOOST                (0U << 8U)
#endif

#else /* defined(STM32_ENFORCE_H7_REV_XY) */

#if STM32_ADC12_CLOCK > 20000000
#define STM32_ADC12_BOOST               (1U << 8U)
#else
#define STM32_ADC12_BOOST               (0U << 8U)
#endif

#if STM32_ADC3_CLOCK > 20000000
#define STM32_ADC3_BOOST                (1U << 8U)
#else
#define STM32_ADC3_BOOST                (0U << 8U)
#endif

#endif /* defined(STM32_ENFORCE_H7_REV_XY) */

#if !defined(STM32_DMA_REQUIRED)
#define STM32_DMA_REQUIRED
#endif

#if STM32_ADC_USE_ADC12
#define STM32_ADC_DMA_REQUIRED
#if !defined(STM32_DMA_REQUIRED)
#define STM32_DMA_REQUIRED
#endif
#endif

#if STM32_ADC_USE_ADC3 && STM32_ADC_ADC3_USE_BDMA
#define STM32_ADC_BDMA_REQUIRED
#if !defined(STM32_BDMA_REQUIRED)
#define STM32_BDMA_REQUIRED
#elif !defined(STM32_ADC_DMA_REQUIRED)
#define STM32_ADC_DMA_REQUIRED
#if !defined(STM32_DMA_REQUIRED)
#define STM32_DMA_REQUIRED
#endif /* !defined(STM32_DMA_REQUIRED) */
#endif /* !defined(STM32_BDMA_REQUIRED) */
#endif /* STM32_ADC_USE_ADC3 && STM32_ADC_ADC3_USE_BDMA */

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   ADC sample data type.
 */
#if (STM32_ADC_SAMPLES_SIZE == 16) || defined(__DOXYGEN__)
typedef uint16_t adcsample_t;
#elif (STM32_ADC_SAMPLES_SIZE == 8)
typedef uint8_t adcsample_t;
#elif (STM32_ADC_SAMPLES_SIZE == 32)
typedef uint32_t adcsample_t;
#endif

/**
 * @brief   Channels number in a conversion group.
 */
typedef uint32_t adc_channels_num_t;

/**
 * @brief   Type of an ADC error mask.
 */
typedef uint32_t adcerror_t;

/**
 * @brief   Type of a DMA channel pointer choice.
 */
typedef union {
#if defined(STM32_ADC_DMA_REQUIRED) || defined(__DOXYGEN__)
    /**
     * @brief   DMA stream.
     */
    const stm32_dma_stream_t  *dma;
#endif
#if defined(STM32_ADC_BDMA_REQUIRED) || defined(__DOXYGEN__)
    /**
     * @brief   BDMA stream.
     */
    const stm32_bdma_stream_t  *bdma;
#endif
} adc_ldd_dma_reference_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Low level fields of the ADC driver structure.
 */
#if (STM32_ADC_DUAL_MODE == TRUE) || defined(__DOXYGEN__)
#define adc_lld_driver_fields                                               \
  /* Pointer to the master ADCx registers block.*/                          \
  ADC_TypeDef               *adcm;                                          \
  /* Pointer to the slave ADCx registers block.*/                           \
  ADC_TypeDef               *adcs;                                          \
   /* Pointer to the common ADCx_y registers block.*/                       \
  ADC_Common_TypeDef        *adcc;                                          \
  /* Pointer to associated DMA channel.*/                                   \
  adc_ldd_dma_reference_t   data;                                           \
  /* DMA mode bit mask.*/                                                   \
  uint32_t                  dmamode;
#else
#define adc_lld_driver_fields                                               \
  /* Pointer to the master ADCx registers block.*/                          \
  ADC_TypeDef               *adcm;                                          \
  /* Pointer to the slave ADCx registers block.*/                           \
  ADC_Common_TypeDef        *adcc;                                          \
  /* Pointer to associated DMA channel.*/                                   \
  adc_ldd_dma_reference_t   data;                                           \
  /* DMA mode bit mask.*/                                                   \
  uint32_t                  dmamode;
#endif

/**
 * @brief   Low level fields of the ADC configuration structure.
 */
#define adc_lld_config_fields                                               \
  /* ADC DIFSEL register initialization data.*/                             \
  uint32_t                  difsel;                                         \
  /* Calibration mode, specify ADCCALIN and/or ADCCALDIF bits in here.*/    \
  uint32_t                  calibration;

#if (STM32_ADC_DUAL_MODE == TRUE) || defined(__DOXYGEN__)
#define adc_lld_configuration_group_fields                                  \
  /* ADC CFGR register initialization data.                                 \
     NOTE: The bits DMAEN and DMACFG are enforced internally                \
           to the driver, keep them to zero.                                \
     NOTE: The bits @p ADC_CFGR_CONT or @p ADC_CFGR_DISCEN must be          \
           specified in continuous mode or if the buffer depth is           \
           greater than one.*/                                              \
  uint32_t                  cfgr;                                           \
  /* ADC CFGR2 register initialization data.                                \
     NOTE: Put this field to zero if not using oversampling.*/              \
  uint32_t                  cfgr2;                                          \
  /* ADC CCR register initialization data*/                                 \
  uint32_t                  ccr;                                            \
  /* ADC PCSEL register initialization data.*/                              \
  uint32_t                  pcsel;                                          \
  /* ADC LTR1 register initialization data.*/                               \
  uint32_t                  ltr1;                                           \
  /* ADC HTR1 register initialization data.*/                               \
  uint32_t                  htr1;                                           \
  /* ADC LTR2 register initialization data.*/                               \
  uint32_t                  ltr2;                                           \
  /* ADC HTR2 register initialization data.*/                               \
  uint32_t                  htr2;                                           \
  /* ADC LTR3 register initialization data.*/                               \
  uint32_t                  ltr3;                                           \
  /* ADC HTR3 register initialization data.*/                               \
  uint32_t                  htr3;                                           \
  /* ADC AWD2CR register initialization data.*/                             \
  uint32_t                  awd2cr;                                         \
  /* ADC AWD3CR register initialization data.*/                             \
  uint32_t                  awd3cr;                                         \
  /* ADC SMPRx registers initialization data.*/                             \
  uint32_t                  smpr[2];                                        \
  /* ADC SQRx register initialization data.*/                               \
  uint32_t                  sqr[4];                                         \
  /* Slave ADC LTR/HTRx registers initialization data.                      \
     NOTE: This field is only present in dual mode.*/                       \
  uint32_t                  sltr1;                                          \
  uint32_t                  shtr1;                                          \
  uint32_t                  sltr2;                                          \
  uint32_t                  shtr2;                                          \
  uint32_t                  sltr3;                                          \
  uint32_t                  shtr3;                                          \
  /* Slave ADC AWDxCR registers initialization data.                        \
     NOTE: This field is only present in dual mode.*/                       \
  uint32_t                  sawd2cr;                                        \
  uint32_t                  sawd3cr;                                        \
  /* Slave ADC SMPRx registers initialization data.                         \
     NOTE: This field is only present in dual mode.*/                       \
  uint32_t                  ssmpr[2];                                       \
  /* Slave ADC SQRx register initialization data.                           \
     NOTE: This field is only present in dual mode.*/                       \
  uint32_t                  ssqr[4];
#else /* STM32_ADC_DUAL_MODE == FALSE */
#define adc_lld_configuration_group_fields                                  \
  uint32_t                  cfgr;                                           \
  uint32_t                  cfgr2;                                          \
  uint32_t                  ccr;                                            \
  uint32_t                  pcsel;                                          \
  uint32_t                  ltr1;                                           \
  uint32_t                  htr1;                                           \
  uint32_t                  ltr2;                                           \
  uint32_t                  htr2;                                           \
  uint32_t                  ltr3;                                           \
  uint32_t                  htr3;                                           \
  uint32_t                  awd2cr;                                         \
  uint32_t                  awd3cr;                                         \
  uint32_t                  smpr[2];                                        \
  uint32_t                  sqr[4];
#endif /* STM32_ADC_DUAL_MODE == FALSE */

/**
 * @name    Sequences building helper macros
 * @{
 */
/**
 * @brief   Number of channels in a conversion sequence.
 */
#define ADC_SQR1_NUM_CH(n)      (((n) - 1U) << 0U)

#define ADC_SQR1_SQ1_N(n)       ((n) << 6U) /**< @brief 1st channel in seq. */
#define ADC_SQR1_SQ2_N(n)       ((n) << 12U)/**< @brief 2nd channel in seq. */
#define ADC_SQR1_SQ3_N(n)       ((n) << 18U)/**< @brief 3rd channel in seq. */
#define ADC_SQR1_SQ4_N(n)       ((n) << 24U)/**< @brief 4th channel in seq. */

#define ADC_SQR2_SQ5_N(n)       ((n) << 0U) /**< @brief 5th channel in seq. */
#define ADC_SQR2_SQ6_N(n)       ((n) << 6U) /**< @brief 6th channel in seq. */
#define ADC_SQR2_SQ7_N(n)       ((n) << 12U)/**< @brief 7th channel in seq. */
#define ADC_SQR2_SQ8_N(n)       ((n) << 18U)/**< @brief 8th channel in seq. */
#define ADC_SQR2_SQ9_N(n)       ((n) << 24U)/**< @brief 9th channel in seq. */

#define ADC_SQR3_SQ10_N(n)      ((n) << 0U) /**< @brief 10th channel in seq.*/
#define ADC_SQR3_SQ11_N(n)      ((n) << 6U) /**< @brief 11th channel in seq.*/
#define ADC_SQR3_SQ12_N(n)      ((n) << 12U)/**< @brief 12th channel in seq.*/
#define ADC_SQR3_SQ13_N(n)      ((n) << 18U)/**< @brief 13th channel in seq.*/
#define ADC_SQR3_SQ14_N(n)      ((n) << 24U)/**< @brief 14th channel in seq.*/

#define ADC_SQR4_SQ15_N(n)      ((n) << 0U) /**< @brief 15th channel in seq.*/
#define ADC_SQR4_SQ16_N(n)      ((n) << 6U) /**< @brief 16th channel in seq.*/
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

#if STM32_ADC_USE_ADC12 && !defined(__DOXYGEN__)
extern ADCDriver ADCD1;
#endif

#if STM32_ADC_USE_ADC3 && !defined(__DOXYGEN__)
extern ADCDriver ADCD3;
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
