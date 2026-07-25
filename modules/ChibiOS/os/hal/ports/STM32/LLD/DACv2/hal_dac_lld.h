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
 * @file    DACv2/hal_dac_lld.h
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

#define DAC_LLD_ENHANCED_API

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

/* Supported devices checks.*/
#if !defined(STM32H5XX) && !defined(STM32U3XX) && !defined(STM32U5XX)
#error "DACv2 supports STM32 H5, U3 and U5 devices only"
#endif

/* Registry checks, keys not present are defaulted to FALSE.*/
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

/* IP instances check.*/
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

#if !STM32_DAC_USE_DAC1_CH1 && !STM32_DAC_USE_DAC1_CH2 &&                   \
    !STM32_DAC_USE_DAC2_CH1 && !STM32_DAC_USE_DAC2_CH2 &&                   \
    !STM32_DAC_USE_DAC3_CH1 && !STM32_DAC_USE_DAC3_CH2 &&                   \
    !STM32_DAC_USE_DAC4_CH1 && !STM32_DAC_USE_DAC4_CH2
#error "DAC driver activated but no DAC peripheral assigned"
#endif

#if STM32_DAC_USE_DAC1_CH1 || STM32_DAC_USE_DAC1_CH2
#if defined(STM32_DAC1_IS_USED)
#error "DACD1/DACD2 require DAC1 but it is already used"
#else
#define STM32_DAC1_IS_USED
#endif
#endif

#if STM32_DAC_USE_DAC2_CH1 || STM32_DAC_USE_DAC2_CH2
#if defined(STM32_DAC2_IS_USED)
#error "DACD3/DACD4 require DAC2 but it is already used"
#else
#define STM32_DAC2_IS_USED
#endif
#endif

#if STM32_DAC_USE_DAC3_CH1 || STM32_DAC_USE_DAC3_CH2
#if defined(STM32_DAC3_IS_USED)
#error "DACD5/DACD6 require DAC3 but it is already used"
#else
#define STM32_DAC3_IS_USED
#endif
#endif

#if STM32_DAC_USE_DAC4_CH1 || STM32_DAC_USE_DAC4_CH2
#if defined(STM32_DAC4_IS_USED)
#error "DACD7/DACD8 require DAC4 but it is already used"
#else
#define STM32_DAC4_IS_USED
#endif
#endif

/* Check on the presence of the DMA channel settings in mcuconf.h.*/
#if STM32_DAC_USE_DAC1_CH1 && !defined(STM32_DAC_DAC1_CH1_DMA3_CHANNEL)
#error "DAC1 STM32_DAC_DAC1_CH1_DMA3_CHANNEL not defined"
#endif

#if STM32_DAC_USE_DAC1_CH2 && !defined(STM32_DAC_DAC1_CH2_DMA3_CHANNEL)
#error "STM32_DAC_DAC1_CH2_DMA3_CHANNEL not defined"
#endif

#if STM32_DAC_USE_DAC2_CH1 && !defined(STM32_DAC_DAC2_CH1_DMA3_CHANNEL)
#error "STM32_DAC_DAC2_CH1_DMA3_CHANNEL not defined"
#endif

#if STM32_DAC_USE_DAC2_CH2 && !defined(STM32_DAC_DAC2_CH2_DMA3_CHANNEL)
#error "STM32_DAC_DAC2_CH2_DMA3_CHANNEL not defined"
#endif

#if STM32_DAC_USE_DAC3_CH1 && !defined(STM32_DAC_DAC3_CH1_DMA3_CHANNEL)
#error "STM32_DAC_DAC3_CH1_DMA3_CHANNEL not defined"
#endif

#if STM32_DAC_USE_DAC3_CH2 && !defined(STM32_DAC_DAC3_CH2_DMA3_CHANNEL)
#error "STM32_DAC_DAC3_CH2_DMA3_CHANNEL not defined"
#endif

#if STM32_DAC_USE_DAC4_CH1 && !defined(STM32_DAC_DAC4_CH1_DMA3_CHANNEL)
#error "STM32_DAC_DAC4_CH1_DMA3_CHANNEL not defined"
#endif

#if STM32_DAC_USE_DAC4_CH2 && !defined(STM32_DAC_DAC4_CH2_DMA3_CHANNEL)
#error "STM32_DAC_DAC4_CH2_DMA3_CHANNEL not defined"
#endif

/* Check on the validity of the assigned GPDMA channels.*/
#if STM32_DAC_USE_DAC1_CH1 &&                                               \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_DAC_DAC1_CH1_DMA_PRIORITY)
#error "Invalid DMA priority assigned to DAC1 CH1"
#endif

#if STM32_DAC_USE_DAC1_CH2 &&                                               \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_DAC_DAC1_CH2_DMA_PRIORITY)
#error "Invalid DMA priority assigned to DAC1 CH2"
#endif

#if STM32_DAC_USE_DAC2_CH1 &&                                               \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_DAC_DAC2_CH1_DMA_PRIORITY)
#error "Invalid DMA priority assigned to DAC2 CH1"
#endif

#if STM32_DAC_USE_DAC2_CH2 &&                                               \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_DAC_DAC2_CH2_DMA_PRIORITY)
#error "Invalid DMA priority assigned to DAC2 CH2"
#endif

#if STM32_DAC_USE_DAC3_CH1 &&                                               \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_DAC_DAC3_CH1_DMA_PRIORITY)
#error "Invalid DMA priority assigned to DAC3 CH1"
#endif

#if STM32_DAC_USE_DAC3_CH2 &&                                               \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_DAC_DAC3_CH2_DMA_PRIORITY)
#error "Invalid DMA priority assigned to DAC3 CH2"
#endif

#if STM32_DAC_USE_DAC4_CH1 &&                                               \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_DAC_DAC4_CH1_DMA_PRIORITY)
#error "Invalid DMA priority assigned to DAC4 CH1"
#endif

#if STM32_DAC_USE_DAC4_CH2 &&                                               \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_DAC_DAC4_CH2_DMA_PRIORITY)
#error "Invalid DMA priority assigned to DAC4 CH2"
#endif

#if STM32_DAC_USE_DAC1_CH1 || STM32_DAC_USE_DAC1_CH2 ||                     \
    STM32_DAC_USE_DAC2_CH1 || STM32_DAC_USE_DAC2_CH2 ||                     \
    STM32_DAC_USE_DAC3_CH1 || STM32_DAC_USE_DAC3_CH2 ||                     \
    STM32_DAC_USE_DAC4_CH1 || STM32_DAC_USE_DAC4_CH2
#if !defined(STM32_DMA3_REQUIRED)
#define STM32_DMA3_REQUIRED
#endif
#endif

/* Consistency checks.*/
#if (STM32_DAC_USE_DAC1_CH2 || STM32_DAC_USE_DAC2_CH2 ||                    \
     STM32_DAC_USE_DAC3_CH2 || STM32_DAC_USE_DAC4_CH2) && STM32_DAC_DUAL_MODE
#error "DACx CH2 not available as an independent driver in dual mode"
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
 * @brief   Type of a structure containing DMA-accessible driver fields.
 */
typedef struct dac_dmabuf {
  /**
   * @brief   GPDMA link structure for circular mode.
   */
  uint32_t                  cb1r;
  uint32_t                  csar;
} dac_dmabuf_t;

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
   * @brief DMA channel request type.
   * */
  uint32_t                  dmach;
  /**
   * @brief DMA priority.
   * */
  uint8_t                   dmaprio;
  /**
   * @brief DMA request line.
   * */
  uint8_t                   dmareq;
  /**
   * @brief   DMA channel IRQ priority.
   */
  uint32_t                  dmairqprio;
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
  const dacparams_t                 *params;                                \
  /* Pointer to associated DMA.*/                                           \
  const stm32_dma3_channel_t        *dmachp;                                \
  /* DMA buffers.*/                                                         \
  dac_dmabuf_t                      *dbuf;


/**
 * @brief   Low level fields of the DAC configuration structure.
 * @note    In DUAL mode init, cr and mcr (if available) fields hold CH1
 *          settings in their lower 16 bits and CH2 settings in the upper
 *          16 bits.
 */
#define dac_lld_config_fields                                               \
  /* Initial output on DAC channel.*/                                       \
  uint32_t                  init;                                           \
  /* DAC data holding register mode.*/                                      \
  dacdhrmode_t              datamode;                                       \
  /* DAC control register.*/                                                \
  uint32_t                  cr;                                             \
  /* DAC mode control register.*/                                           \
  uint32_t                  mcr;

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
#define DAC_TRG_MASK                    0xFU
#define DAC_TRG(n)                      (n)
/** @} */

/**
 * @brief   Shift of initialization value for channel 2 in dual mode.
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
  void  dac_lld_init(void);
  msg_t dac_lld_start(DACDriver *dacp);
  void  dac_lld_stop(DACDriver *dacp);
  msg_t dac_lld_put_channel(DACDriver *dacp,
                           dacchannel_t channel,
                           dacsample_t sample);
  msg_t dac_lld_start_conversion(DACDriver *dacp);
  void  dac_lld_stop_conversion(DACDriver *dacp);
  void dac_lld_serve_interrupt(DACDriver *dacp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_DAC */

#endif /* HAL_DAC_LLD_H */

/** @} */
