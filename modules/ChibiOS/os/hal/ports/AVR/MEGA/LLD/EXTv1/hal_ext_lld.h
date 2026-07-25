/*
  ChibiOS - Copyright (C) 2016 Theodore Ateba

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
 * @file  EXTv1/hal_ext_lld.h
 * @brief AVR/MEGA EXT subsystem low level driver header.
 *
 * @addtogroup EXT
 * @{
 */

#ifndef HAL_EXT_LLD_H
#define HAL_EXT_LLD_H

#if HAL_USE_EXT || defined(__DOXYGEN__)

/*==========================================================================*/
/* Driver constants.                                                        */
/*==========================================================================*/

/**
 * @brief Maximum number of EXT channels.
 */
#define AVR_INT_NUM_LINES 6 /**< INT0 to INT5. */

/**
 * @brief Available number of EXT channels.
 */
#define EXT_MAX_CHANNELS  AVR_INT_NUM_LINES

/*==========================================================================*/
/* Driver pre-compile time settings.                                        */
/*==========================================================================*/

/*==========================================================================*/
/* Derived constants and error checks.                                      */
/*==========================================================================*/

/*==========================================================================*/
/* Driver data structures and types.                                        */
/*==========================================================================*/

/**
 * @brief EXT channel identifier.
 */
typedef uint16_t expchannel_t;

/**
 * @brief Type of an EXT generic notification callback.
 *
 * @param[in] extp  pointer to the @p EXPDriver object triggering the
 *                  callback
 */
typedef void (*extcallback_t)(EXTDriver *extp, expchannel_t channel);

/**
 * @brief Channel configuration structure.
 */
typedef struct {
  /**
   * @brief Channel mode.
   */
  uint32_t      mode;
  /**
   * @brief Channel callback.
   */
  extcallback_t cb;
} EXTChannelConfig;

/**
 * @brief Driver configuration structure.
 * @note  It could be empty on some architectures.
 */
typedef struct {
  /**
   * @brief Channel configurations.
   */
  EXTChannelConfig  channels[EXT_MAX_CHANNELS];
  /* End of the mandatory fields. */
} EXTConfig;

/**
 * @brief Structure representing an EXT driver.
 */
struct EXTDriver {
  /**
   * @brief Driver state.
   */
  extstate_t      state;

  /**
   * @brief Current configuration data.
   */
  const EXTConfig *config;
  /* End of the mandatory fields. */
};

/*==========================================================================*/
/* Driver macros.                                                           */
/*==========================================================================*/

/*==========================================================================*/
/* External declarations.                                                   */
/*==========================================================================*/
extern EXTDriver EXTD1;

#ifdef __cplusplus
extern "C" {
#endif
  void ext_lld_init(void);
  void ext_lld_start(EXTDriver *extp);
  void ext_lld_stop(EXTDriver *extp);
  void ext_lld_channel_enable(EXTDriver *extp, expchannel_t channel);
  void ext_lld_channel_disable(EXTDriver *extp, expchannel_t channel);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_EXT */

#endif /* HAL_EXT_LLD_H */

