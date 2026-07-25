/*
    ChibiOS - Copyright (C) 2016..2018 Theodore Ateba

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
 * @file    hal_dac_lld.h
 * @brief   AVR DAC subsystem low level driver header.
 *
 * @addtogroup DAC
 * @{
 */

#ifndef HAL_DAC_LLD_H
#define HAL_DAC_LLD_H

#if HAL_USE_DAC || defined(__DOXYGEN__)

/*==========================================================================*/
/* Driver constants.                                                        */
/*==========================================================================*/

/**
 * @brief   Maximum number of DAC channels per unit.
 */
#define DAC_MAX_CHANNELS                    2

/*==========================================================================*/
/* Driver pre-compile time settings.                                        */
/*==========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   DAC1 CH1 driver enable switch.
 * @details If set to @p TRUE the support for DAC1 channel 1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(AVR_DAC_USE_DAC1) || defined(__DOXYGEN__)
#define AVR_DAC_USE_DAC1               FALSE
#endif
/** @} */

/*==========================================================================*/
/* Derived constants and error checks.                                      */
/*==========================================================================*/

/*==========================================================================*/
/* Driver data structures and types.                                        */
/*==========================================================================*/

/**
 * @brief   DAC channel identifier.
 */
typedef enum {
  DAC_CHANNEL0  = 0, /**< DAC channel 0.                                    */
  DAC_CHANNEL1  = 1, /**< DAC channel 1.                                    */
} dacchan_t;

/**
 * @brief   DAC channel selection.
 */
typedef enum {
  DAC_CH_SEL0 = 0, /**< DAC single operation on channel 0.                  */
  DAC_CH_SEL1 = 1, /**< DAC single operation on channel 1.                  */
  DAC_CH_DUAL = 2, /**< DAC dual channel opration.                          */
} dacchansel_t;

/**
 * @brief   DAC voltage reference selection.
 */
typedef enum {
  DAC_REFSEL_INT1V  = 0, /**< DAC reference is the internal 1V.             */
  DAC_REFSEL_AVCC   = 1, /**< DAC reference is the analog voltage.          */
  DAC_REFSEL_AREFA  = 2, /**< DAC reference is the analog voltage on PORTA. */
  DAC_REFSEL_AREFB  = 3, /**< DAC reference is the analog voltage on PORTB. */
} dacrefsel_t;

/**
 * @brief   DAC operation mode (Single/Dual).
 */
typedef enum {
  DAC_OPMODE_SINGLE = 0, /**< DAC is use in single operation mode.          */
  DAC_OPMODE_DUAL   = 1, /**< DAC is use in dual operation mode.            */
} dacopmode_t;

/**
 * @brief   DAC alignment mode(Right/Left).
 */
typedef enum {
  DAC_AJUST_12BIT_RIGHT = 0, /**< DAC right ajusted value.                  */
  DAC_AJUST_12BIT_LEFT  = 1, /**< DAC left ajusted value.                   */
} dacajustmode_t;

/**
 * @brief   Possible DAC trigger mode during conversion.
 */
typedef enum {
  DAC_TRIG_ON_DATAREG  = 0, /**< DAC triggered by data register.            */
  DAC_TRIG_ON_EVENT    = 1, /**< DAC triggered by incoming event system.    */
} dactrigmode_t;

/**
 * @brief   Type of a DAC channel index.
 */
typedef uint8_t dacchannel_t;

/**
 * @brief   Type of a structure representing an DAC driver.
 */
typedef struct DACDriver DACDriver;

/**
 * @brief   Type representing a DAC sample.
 */
typedef uint16_t dacsample_t;

/**
 * @brief   Possible DAC failure causes.
 * @note    Error codes are architecture dependent and should not relied
 *          upon.
 */
typedef enum {
  DAC_ERR_DMAFAILURE  = 0,  /**< DMA operations failure.                    */
  DAC_ERR_UNDERFLOW   = 1   /**< DAC overflow condition.                    */
} dacerror_t;

/**
 * @brief   DAC notification callback type.
 *
 * @param[in] dacp      pointer to the @p DACDriver object triggering the
 * @param[in] buffer    pointer to the next semi-buffer to be filled
 * @param[in] n         number of buffer rows available starting from @p buffer
 *                      callback
 */
typedef void (*daccallback_t)(DACDriver *dacp, dacsample_t *buffer, size_t n);

/**
 * @brief   DAC error callback type.
 *
 * @param[in] dacp      pointer to the @p DACDriver object triggering the
 *                      callback
 * @param[in] err       DAC error code
 */
typedef void (*dacerrorcallback_t)(DACDriver *dacp, dacerror_t err);

/**
 * @brief   DAC Conversion group structure.
 */
typedef struct {
  /**
   * @brief   Number of DAC channels.
   */
  uint32_t                  num_channels;
  /**
   * @brief   Operation complete callback or @p NULL.
   */
  daccallback_t             end_cb;
  /**
   * @brief   Error handling callback or @p NULL.
   */
  dacerrorcallback_t        error_cb;
  /* End of the mandatory fields. */
} DACConversionGroup;

/**
 * @brief   Driver configuration structure.
 */
typedef struct {
  dacchan_t                 ch; /* DAC channel id.                          */
  dactrigmode_t             tm; /* Register write/event trigger.            */
  dacopmode_t               om; /* DAC operation mode.                      */
  dacajustmode_t            da; /* DAC (left/rigth) ajustement.             */
  dacrefsel_t               vr; /* DAC voltage reference.                   */
  /* End of the mandatory fields. */
} DACConfig;

/**
 * @brief   Structure representing a DAC driver.
 */
struct DACDriver {
  /**
   * @brief   Driver state.
   */
  dacstate_t                state;
  /**
   * @brief   Conversion group.
   */
  const DACConversionGroup  *grpp;
  /**
   * @brief   Samples buffer pointer.
   */
  dacsample_t               *samples;
  /**
   * @brief   Samples buffer size.
   */
  uint16_t                  depth;
  /**
   * @brief   Current configuration data.
   */
  const DACConfig           *config;
#if DAC_USE_WAIT || defined(__DOXYGEN__)
  /**
   * @brief   Waiting thread.
   */
  thread_reference_t        thread;
#endif /* DAC_USE_WAIT */
#if DAC_USE_MUTUAL_EXCLUSION || defined(__DOXYGEN__)
  /**
   * @brief   Mutex protecting the bus.
   */
  mutex_t                   mutex;
#endif /* DAC_USE_MUTUAL_EXCLUSION */
#if defined(DAC_DRIVER_EXT_FIELDS)
  DAC_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields. */
  /**
   * @brief   Pointer to the DAC module block.
   */
  DAC_t                     *dacblock;
};

/*==========================================================================*/
/* Driver macros.                                                           */
/*==========================================================================*/

/*==========================================================================*/
/* External declarations.                                                   */
/*==========================================================================*/

#if AVR_DAC_USE_DAC1 && !defined(__DOXYGEN__)
extern DACDriver DACD1;
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
