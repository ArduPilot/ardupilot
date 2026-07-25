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
 * @file    DACv1/hal_dac_lld.h
 * @brief   STM32 DAC subsystem low level driver header.
 *
 * @addtogroup DAC
 * @{
 */

#ifndef HAL_DAC_LLD_H
#define HAL_DAC_LLD_H

#if HAL_USE_DAC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */

/**
 * @brief   Enables the DAC dual mode.
 * @note    In dual mode DAC second channels cannot be accessed individually.
 */
#if !defined(STM32_DAC_DUAL_MODE) || defined(__DOXYGEN__)
#define STM32_DAC_DUAL_MODE                 FALSE
#endif

/**
 * @brief   DAC1 CH1 driver enable switch.
 * @details If set to @p TRUE the support for DAC1 channel 1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_DAC_USE_DAC1_CH1) || defined(__DOXYGEN__)
#define STM32_DAC_USE_DAC1_CH1              FALSE
#endif

/**
 * @brief   DAC1 CH2 driver enable switch.
 * @details If set to @p TRUE the support for DAC1 channel 2 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_DAC_USE_DAC1_CH2) || defined(__DOXYGEN__)
#define STM32_DAC_USE_DAC1_CH2              FALSE
#endif

/**
 * @brief   DAC2 CH1 driver enable switch.
 * @details If set to @p TRUE the support for DAC2 channel 1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_DAC_USE_DAC2_CH1) || defined(__DOXYGEN__)
#define STM32_DAC_USE_DAC2_CH1              FALSE
#endif

/**
 * @brief   DAC2 CH2 driver enable switch.
 * @details If set to @p TRUE the support for DAC2 channel 2 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_DAC_USE_DAC2_CH2) || defined(__DOXYGEN__)
#define STM32_DAC_USE_DAC2_CH2              FALSE
#endif

/**
 * @brief   DAC3 CH1 driver enable switch.
 * @details If set to @p TRUE the support for DAC3 channel 1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_DAC_USE_DAC3_CH1) || defined(__DOXYGEN__)
#define STM32_DAC_USE_DAC3_CH1              FALSE
#endif

/**
 * @brief   DAC3 CH2 driver enable switch.
 * @details If set to @p TRUE the support for DAC3 channel 2 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_DAC_USE_DAC3_CH2) || defined(__DOXYGEN__)
#define STM32_DAC_USE_DAC3_CH2              FALSE
#endif

/**
 * @brief   DAC4 CH1 driver enable switch.
 * @details If set to @p TRUE the support for DAC4 channel 1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_DAC_USE_DAC4_CH1) || defined(__DOXYGEN__)
#define STM32_DAC_USE_DAC4_CH1              FALSE
#endif

/**
 * @brief   DAC4 CH2 driver enable switch.
 * @details If set to @p TRUE the support for DAC4 channel 2 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_DAC_USE_DAC4_CH2) || defined(__DOXYGEN__)
#define STM32_DAC_USE_DAC4_CH2              FALSE
#endif

/**
 * @brief   DAC1 CH1 interrupt priority level setting.
 */
#if !defined(STM32_DAC_DAC1_CH1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_DAC_DAC1_CH1_IRQ_PRIORITY     10
#endif

/**
 * @brief   DAC1 CH2 interrupt priority level setting.
 */
#if !defined(STM32_DAC_DAC1_CH2_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_DAC_DAC1_CH2_IRQ_PRIORITY     10
#endif

/**
 * @brief   DAC2 CH1 interrupt priority level setting.
 */
#if !defined(STM32_DAC_DAC2_CH1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_DAC_DAC2_CH1_IRQ_PRIORITY     10
#endif

/**
 * @brief   DAC2 CH2 interrupt priority level setting.
 */
#if !defined(STM32_DAC_DAC2_CH2_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_DAC_DAC2_CH2_IRQ_PRIORITY     10
#endif

/**
 * @brief   DAC3 CH1 interrupt priority level setting.
 */
#if !defined(STM32_DAC_DAC3_CH1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_DAC_DAC3_CH1_IRQ_PRIORITY     10
#endif

/**
 * @brief   DAC3 CH2 interrupt priority level setting.
 */
#if !defined(STM32_DAC_DAC3_CH2_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_DAC_DAC3_CH2_IRQ_PRIORITY     10
#endif

/**
 * @brief   DAC4 CH1 interrupt priority level setting.
 */
#if !defined(STM32_DAC_DAC4_CH1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_DAC_DAC4_CH1_IRQ_PRIORITY     10
#endif

/**
 * @brief   DAC4 CH2 interrupt priority level setting.
 */
#if !defined(STM32_DAC_DAC4_CH2_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_DAC_DAC4_CH2_IRQ_PRIORITY     10
#endif

/**
 * @brief   DAC1 CH1 DMA priority (0..3|lowest..highest).
 */
#if !defined(STM32_DAC_DAC1_CH1_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_DAC_DAC1_CH1_DMA_PRIORITY     2
#endif

/**
 * @brief   DAC1 CH2 DMA priority (0..3|lowest..highest).
 */
#if !defined(STM32_DAC_DAC1_CH2_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_DAC_DAC1_CH2_DMA_PRIORITY     2
#endif

/**
 * @brief   DAC2 CH1 DMA priority (0..3|lowest..highest).
 */
#if !defined(STM32_DAC_DAC2_CH1_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_DAC_DAC2_CH1_DMA_PRIORITY     2
#endif

/**
 * @brief   DAC2 CH2 DMA priority (0..3|lowest..highest).
 */
#if !defined(STM32_DAC_DAC2_CH2_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_DAC_DAC2_CH2_DMA_PRIORITY     2
#endif

/**
 * @brief   DAC3 CH1 DMA priority (0..3|lowest..highest).
 */
#if !defined(STM32_DAC_DAC3_CH1_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_DAC_DAC3_CH1_DMA_PRIORITY     2
#endif

/**
 * @brief   DAC3 CH2 DMA priority (0..3|lowest..highest).
 */
#if !defined(STM32_DAC_DAC3_CH2_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_DAC_DAC3_CH2_DMA_PRIORITY     2
#endif

/**
 * @brief   DAC4 CH1 DMA priority (0..3|lowest..highest).
 */
#if !defined(STM32_DAC_DAC4_CH1_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_DAC_DAC4_CH1_DMA_PRIORITY     2
#endif

/**
 * @brief   DAC4 CH2 DMA priority (0..3|lowest..highest).
 */
#if !defined(STM32_DAC_DAC4_CH2_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_DAC_DAC4_CH2_DMA_PRIORITY     2
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Handling missing registry keys.*/
#if !defined(STM32_HAS_DAC1_CH1)
#define STM32_HAS_DAC1_CH1                  FALSE
#endif
#if !defined(STM32_HAS_DAC1_CH2)
#define STM32_HAS_DAC1_CH2                  FALSE
#endif
#if !defined(STM32_HAS_DAC2_CH1)
#define STM32_HAS_DAC2_CH1                  FALSE
#endif
#if !defined(STM32_HAS_DAC2_CH2)
#define STM32_HAS_DAC2_CH2                  FALSE
#endif
#if !defined(STM32_HAS_DAC3_CH1)
#define STM32_HAS_DAC3_CH1                  FALSE
#endif
#if !defined(STM32_HAS_DAC3_CH2)
#define STM32_HAS_DAC3_CH2                  FALSE
#endif
#if !defined(STM32_HAS_DAC4_CH1)
#define STM32_HAS_DAC4_CH1                  FALSE
#endif
#if !defined(STM32_HAS_DAC4_CH2)
#define STM32_HAS_DAC4_CH2                  FALSE
#endif

#if !defined(STM32_DAC_HAS_MCR)
#define STM32_DAC_HAS_MCR                   FALSE
#endif

#if STM32_DAC_USE_DAC1_CH1 && !STM32_HAS_DAC1_CH1
#error "DAC1 CH1 not present in the selected device"
#endif

#if STM32_DAC_USE_DAC1_CH2 && !STM32_HAS_DAC1_CH2
#error "DAC1 CH2 not present in the selected device"
#endif

#if STM32_DAC_USE_DAC2_CH1 && !STM32_HAS_DAC2_CH1
#error "DAC2 CH1 not present in the selected device"
#endif

#if STM32_DAC_USE_DAC2_CH2 && !STM32_HAS_DAC2_CH2
#error "DAC2 CH2 not present in the selected device"
#endif

#if STM32_DAC_USE_DAC3_CH1 && !STM32_HAS_DAC3_CH1
#error "DAC3 CH1 not present in the selected device"
#endif

#if STM32_DAC_USE_DAC3_CH2 && !STM32_HAS_DAC3_CH2
#error "DAC3 CH2 not present in the selected device"
#endif

#if STM32_DAC_USE_DAC4_CH1 && !STM32_HAS_DAC4_CH1
#error "DAC4 CH1 not present in the selected device"
#endif

#if STM32_DAC_USE_DAC4_CH2 && !STM32_HAS_DAC4_CH2
#error "DAC4 CH2 not present in the selected device"
#endif

#if (STM32_DAC_USE_DAC1_CH2 || STM32_DAC_USE_DAC2_CH2 ||                    \
     STM32_DAC_USE_DAC3_CH2 || STM32_DAC_USE_DAC4_CH2) && STM32_DAC_DUAL_MODE
#error "DACx CH2 cannot be used independently in dual mode"
#endif

#if !STM32_DAC_USE_DAC1_CH1 && !STM32_DAC_USE_DAC1_CH2 &&                   \
    !STM32_DAC_USE_DAC2_CH1 && !STM32_DAC_USE_DAC2_CH2 &&                   \
    !STM32_DAC_USE_DAC3_CH1 && !STM32_DAC_USE_DAC3_CH2 &&                   \
    !STM32_DAC_USE_DAC4_CH1 && !STM32_DAC_USE_DAC4_CH2
#error "DAC driver activated but no DAC peripheral assigned"
#endif

#if STM32_DAC_USE_DAC1_CH1 &&                                               \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_DAC_DAC1_CH1_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to DAC1 CH1"
#endif

#if STM32_DAC_USE_DAC1_CH2 &&                                               \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_DAC_DAC1_CH2_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to DAC1 CH2"
#endif

#if STM32_DAC_USE_DAC2_CH1 &&                                               \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_DAC_DAC2_CH1_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to DAC2 CH1"
#endif

#if STM32_DAC_USE_DAC2_CH2 &&                                               \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_DAC_DAC2_CH2_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to DAC2 CH2"
#endif

#if STM32_DAC_USE_DAC3_CH1 &&                                               \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_DAC_DAC3_CH1_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to DAC3 CH1"
#endif

#if STM32_DAC_USE_DAC3_CH2 &&                                               \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_DAC_DAC3_CH2_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to DAC3 CH2"
#endif

#if STM32_DAC_USE_DAC4_CH1 &&                                               \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_DAC_DAC4_CH1_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to DAC4 CH1"
#endif

#if STM32_DAC_USE_DAC4_CH2 &&                                               \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_DAC_DAC4_CH2_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to DAC4 CH2"
#endif

/* The following checks are only required when there is a DMA able to
   reassign streams to different channels.*/
#if STM32_ADVANCED_DMA

/* Check on the presence of the DMA streams settings in mcuconf.h.*/
#if STM32_DAC_USE_DAC1_CH1 && !defined(STM32_DAC_DAC1_CH1_DMA_STREAM)
#error "DAC1 CH1 DMA stream not defined"
#endif

#if STM32_DAC_USE_DAC1_CH2 && !defined(STM32_DAC_DAC1_CH2_DMA_STREAM)
#error "DAC1 CH2 DMA stream not defined"
#endif

#if STM32_DAC_USE_DAC2_CH1 && !defined(STM32_DAC_DAC2_CH1_DMA_STREAM)
#error "DAC2 CH1 DMA stream not defined"
#endif

#if STM32_DAC_USE_DAC2_CH2 && !defined(STM32_DAC_DAC2_CH2_DMA_STREAM)
#error "DAC2 CH2 DMA stream not defined"
#endif

#if STM32_DAC_USE_DAC3_CH1 && !defined(STM32_DAC_DAC3_CH1_DMA_STREAM)
#error "DAC3 CH1 DMA stream not defined"
#endif

#if STM32_DAC_USE_DAC3_CH2 && !defined(STM32_DAC_DAC3_CH2_DMA_STREAM)
#error "DAC3 CH2 DMA stream not defined"
#endif

#if STM32_DAC_USE_DAC4_CH1 && !defined(STM32_DAC_DAC4_CH1_DMA_STREAM)
#error "DAC4 CH1 DMA stream not defined"
#endif

#if STM32_DAC_USE_DAC4_CH2 && !defined(STM32_DAC_DAC4_CH2_DMA_STREAM)
#error "DAC4 CH2 DMA stream not defined"
#endif

#if STM32_DMA_SUPPORTS_DMAMUX

#else /* !STM32_DMA_SUPPORTS_DMAMUX */

/* Check on the validity of the assigned DMA streams.*/
#if STM32_DAC_USE_DAC1_CH1 &&                                               \
    !STM32_DMA_IS_VALID_ID(STM32_DAC_DAC1_CH1_DMA_STREAM, STM32_DAC1_CH1_DMA_MSK)
#error "invalid DMA stream associated to DAC1 CH1"
#endif

#if STM32_DAC_USE_DAC1_CH2 &&                                               \
    !STM32_DMA_IS_VALID_ID(STM32_DAC_DAC1_CH2_DMA_STREAM, STM32_DAC1_CH2_DMA_MSK)
#error "invalid DMA stream associated to DAC1 CH2"
#endif

#if STM32_DAC_USE_DAC2_CH1 &&                                               \
    !STM32_DMA_IS_VALID_ID(STM32_DAC_DAC2_CH1_DMA_STREAM, STM32_DAC2_CH1_DMA_MSK)
#error "invalid DMA stream associated to DAC2 CH1"
#endif

#if STM32_DAC_USE_DAC2_CH2 &&                                               \
    !STM32_DMA_IS_VALID_ID(STM32_DAC_DAC2_CH2_DMA_STREAM, STM32_DAC2_CH2_DMA_MSK)
#error "invalid DMA stream associated to DAC2 CH2"
#endif

#if STM32_DAC_USE_DAC3_CH1 &&                                               \
    !STM32_DMA_IS_VALID_ID(STM32_DAC_DAC3_CH1_DMA_STREAM, STM32_DAC3_CH1_DMA_MSK)
#error "invalid DMA stream associated to DAC1 CH1"
#endif

#if STM32_DAC_USE_DAC3_CH2 &&                                               \
    !STM32_DMA_IS_VALID_ID(STM32_DAC_DAC3_CH2_DMA_STREAM, STM32_DAC3_CH2_DMA_MSK)
#error "invalid DMA stream associated to DAC1 CH2"
#endif

#if STM32_DAC_USE_DAC4_CH1 &&                                               \
    !STM32_DMA_IS_VALID_ID(STM32_DAC_DAC4_CH1_DMA_STREAM, STM32_DAC4_CH1_DMA_MSK)
#error "invalid DMA stream associated to DAC2 CH1"
#endif

#if STM32_DAC_USE_DAC4_CH2 &&                                               \
    !STM32_DMA_IS_VALID_ID(STM32_DAC_DAC4_CH2_DMA_STREAM, STM32_DAC4_CH2_DMA_MSK)
#error "invalid DMA stream associated to DAC2 CH2"
#endif

#endif /* !STM32_DMA_SUPPORTS_DMAMUX */

#endif /* STM32_ADVANCED_DMA */

#if STM32_DAC_USE_DAC1_CH1 &&                                               \
    !STM32_DMA_IS_VALID_PRIORITY(STM32_DAC_DAC1_CH1_DMA_PRIORITY)
#error "Invalid DMA priority assigned to DAC1 CH1"
#endif

#if STM32_DAC_USE_DAC1_CH2 &&                                               \
    !STM32_DMA_IS_VALID_PRIORITY(STM32_DAC_DAC1_CH2_DMA_PRIORITY)
#error "Invalid DMA priority assigned to DAC1 CH2"
#endif

#if STM32_DAC_USE_DAC2_CH1 &&                                               \
    !STM32_DMA_IS_VALID_PRIORITY(STM32_DAC_DAC2_CH1_DMA_PRIORITY)
#error "Invalid DMA priority assigned to DAC2 CH1"
#endif

#if STM32_DAC_USE_DAC2_CH2 &&                                               \
    !STM32_DMA_IS_VALID_PRIORITY(STM32_DAC_DAC2_CH2_DMA_PRIORITY)
#error "Invalid DMA priority assigned to DAC2 CH2"
#endif

#if STM32_DAC_USE_DAC3_CH1 &&                                               \
    !STM32_DMA_IS_VALID_PRIORITY(STM32_DAC_DAC3_CH1_DMA_PRIORITY)
#error "Invalid DMA priority assigned to DAC3 CH1"
#endif

#if STM32_DAC_USE_DAC3_CH2 &&                                               \
    !STM32_DMA_IS_VALID_PRIORITY(STM32_DAC_DAC3_CH2_DMA_PRIORITY)
#error "Invalid DMA priority assigned to DAC3 CH2"
#endif

#if STM32_DAC_USE_DAC4_CH1 &&                                               \
    !STM32_DMA_IS_VALID_PRIORITY(STM32_DAC_DAC4_CH1_DMA_PRIORITY)
#error "Invalid DMA priority assigned to DAC4 CH1"
#endif

#if STM32_DAC_USE_DAC4_CH2 &&                                               \
    !STM32_DMA_IS_VALID_PRIORITY(STM32_DAC_DAC4_CH2_DMA_PRIORITY)
#error "Invalid DMA priority assigned to DAC4 CH2"
#endif

#if !defined(STM32_DMA_REQUIRED)
#define STM32_DMA_REQUIRED
#endif

/**
 * @brief   Max DAC channels.
 */
#if STM32_DAC_DUAL_MODE == TRUE
#define DAC_MAX_CHANNELS                    2
#else
#define DAC_MAX_CHANNELS                    1
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a DAC channel index.
 */
typedef uint32_t dacchannel_t;

/**
 * @brief   Type representing a DAC sample.
 */
typedef uint16_t dacsample_t;

/**
 * @brief   DAC channel parameters type.
 */
typedef struct {
  /**
   * @brief   Pointer to the DAC registers block.
   */
  DAC_TypeDef               *dac;
  /**
   * @brief   DAC data registers offset.
   */
  uint32_t                  dataoffset;
  /**
   * @brief   DAC CR register bit offset.
   */
  uint32_t                  regshift;
  /**
   * @brief   DAC CR register mask.
   */
  uint32_t                  regmask;
  /**
   * @brief   Associated DMA stream.
   */
  uint32_t                  dmastream;
  /**
   * @brief   Mode bits for the DMA.
   */
  uint32_t                  dmamode;
  /**
   * @brief   DMA channel IRQ priority.
   */
  uint32_t                  dmairqprio;
#if (STM32_DMA_SUPPORTS_DMAMUX == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief   DMAMUX peripheral selector.
   */
  uint32_t                  peripheral;
#endif
} dacparams_t;

/**
 * @brief   Possible DAC failure causes.
 * @note    Error codes are architecture dependent and should not relied
 *          upon.
 */
typedef enum {
  DAC_ERR_DMAFAILURE = 0,                   /**< DMA operations failure.    */
  DAC_ERR_UNDERFLOW = 1                     /**< DAC overflow condition.    */
} dacerror_t;

/**
 * @brief   Samples alignment and size mode.
 */
typedef enum {
  DAC_DHRM_12BIT_RIGHT = 0,
  DAC_DHRM_12BIT_LEFT = 1,
  DAC_DHRM_8BIT_RIGHT = 2,
#if STM32_DAC_DUAL_MODE && !defined(__DOXYGEN__)
  DAC_DHRM_12BIT_RIGHT_DUAL = 3,
  DAC_DHRM_12BIT_LEFT_DUAL = 4,
  DAC_DHRM_8BIT_RIGHT_DUAL = 5
#endif
} dacdhrmode_t;

/**
 * @brief   Low level fields of the DAC driver structure.
 */
#define dac_lld_driver_fields                                               \
  /* DAC channel parameters.*/                                              \
  const dacparams_t         *params;                                        \
  /* Associated DMA.*/                                                      \
  const stm32_dma_stream_t  *dma;

/**
 * @brief   Low level fields of the DAC configuration structure.
 * @note    In DUAL mode init, cr and mcr (if available) fields hold CH1
 *          settings in their lower 16 bits and CH2 settings in the upper
 *          16 bits.
 */
#if STM32_DAC_HAS_MCR == TRUE
#define dac_lld_config_fields                                               \
  /* Initial output on DAC channel.*/                                       \
  uint32_t                  init;                                           \
  /* DAC data holding register mode.*/                                      \
  dacdhrmode_t              datamode;                                       \
  /* DAC control register.*/                                                \
  uint32_t                  cr;                                             \
  /* DAC mode control register.*/                                           \
  uint32_t                  mcr;
#else
#define dac_lld_config_fields                                               \
  /* Initial output on DAC channel.*/                                       \
  uint32_t                  init;                                           \
  /* DAC data holding register mode.*/                                      \
  dacdhrmode_t              datamode;                                       \
  /* DAC control register.*/                                                \
  uint32_t                  cr;
#endif /* STM32_DAC_HAS_MCR == TRUE */

/**
 * @brief   Low level fields of the DAC group configuration structure.
 */
#define dac_lld_conversion_group_fields                                     \
  /* DAC initialization data. This field contains the (not shifted) value   \
     to be put into the TSEL field of the DAC CR register during            \
     initialization. All other fields are handled internally.*/             \
  uint32_t                  trigger;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    DAC trigger modes
 * @{
 */
#define DAC_TRG_MASK                    7U
#define DAC_TRG(n)                      (n)
/** @} */

/**
 * @brief   Shift of initialisation value for channel 2 in dual mode.
 */
#define DAC_VALUE_DUAL(n) ((n) << (sizeof(dacsample_t) * 8))

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if STM32_DAC_USE_DAC1_CH1 && !defined(__DOXYGEN__)
extern DACDriver DACD1;
#endif

#if STM32_DAC_USE_DAC1_CH2 && !STM32_DAC_DUAL_MODE && !defined(__DOXYGEN__)
extern DACDriver DACD2;
#endif

#if STM32_DAC_USE_DAC2_CH1 && !defined(__DOXYGEN__)
extern DACDriver DACD3;
#endif

#if STM32_DAC_USE_DAC2_CH2 && !STM32_DAC_DUAL_MODE && !defined(__DOXYGEN__)
extern DACDriver DACD4;
#endif

#if STM32_DAC_USE_DAC3_CH1 && !defined(__DOXYGEN__)
extern DACDriver DACD5;
#endif

#if STM32_DAC_USE_DAC3_CH2 && !STM32_DAC_DUAL_MODE && !defined(__DOXYGEN__)
extern DACDriver DACD6;
#endif

#if STM32_DAC_USE_DAC4_CH1 && !defined(__DOXYGEN__)
extern DACDriver DACD7;
#endif

#if STM32_DAC_USE_DAC4_CH2 && !STM32_DAC_DUAL_MODE && !defined(__DOXYGEN__)
extern DACDriver DACD8;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void dac_lld_init(void);
  void dac_lld_start(DACDriver *dacp);
  void dac_lld_stop(DACDriver *dacp);
  void dac_lld_put_channel(DACDriver *dacp,
                           dacchannel_t channel,
                           dacsample_t sample);
  void dac_lld_start_conversion(DACDriver *dacp);
  void dac_lld_stop_conversion(DACDriver *dacp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_DAC */

#endif /* HAL_DAC_LLD_H */

/** @} */
