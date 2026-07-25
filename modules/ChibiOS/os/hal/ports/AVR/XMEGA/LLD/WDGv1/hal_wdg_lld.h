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
 * @file    WDGv1/hal_wdg_lld.h
 * @brief   WDG Driver subsystem low level driver header.
 *
 * @addtogroup WDG
 * @{
 */

#ifndef HAL_WDG_LLD_H
#define HAL_WDG_LLD_H

#if (HAL_USE_WDG == TRUE) || defined(__DOXYGEN__)

/*==========================================================================*/
/* Driver constants.                                                        */
/*==========================================================================*/

/*==========================================================================*/
/* Driver pre-compile time settings.                                        */
/*==========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   WDT driver enable switch.
 * @note    The default is @p FALSE.
 */
#if !defined(AVR_WDG_USE_WDT) || defined(__DOXYGEN__)
#define AVR_WDG_USE_WDT                     FALSE
#endif
/** @} */

/*==========================================================================*/
/* Derived constants and error checks.                                      */
/*==========================================================================*/

#define AVR_HAS_WDG   TRUE

#if AVR_WDG_USE_WDT && !AVR_HAS_WDG
#error "WDT not present in the selected device"
#endif

#if !AVR_WDG_USE_WDT
#error "WDT driver activated but no WDT peripheral assigned"
#endif

/*==========================================================================*/
/* Driver data structures and types.                                        */
/*==========================================================================*/

/**
 * @brief   Type of a structure representing an WDG driver.
 */
typedef struct WDGDriver WDGDriver;

/**
 * @brief   Driver configuration structure.
 * @note    It could be empty on some architectures.
 */
typedef struct {
  /**
   * @brief   Configuration of the WDT driver mode.
   */
  bool        mode;
  /**
   * @brief   Configuration of the WDT driver Nomal timeout period.
   */
  WDT_PER_t   ntp;
  /**
   * @brief   Configuration of the WDT driver Closed timeout period.
   */
  WDT_WPER_t  ctp;
} WDGConfig;

/**
 * @brief   Structure representing an WDG driver.
 */
struct WDGDriver {
  /**
   * @brief   Driver state.
   */
  wdgstate_t                state;
  /**
   * @brief   Current configuration data.
   */
  const WDGConfig           *config;
  /* End of the mandatory fields. */
  /**
   * @brief   Pointer to the WDG*T registers block.
   */
  WDT_t                     *wdg;
};

/*==========================================================================*/
/* Driver macros.                                                           */
/*==========================================================================*/

/*==========================================================================*/
/* External declarations.                                                   */
/*==========================================================================*/

#if (AVR_WDG_USE_WDT == TRUE) && !defined(__DOXYGEN__)
extern WDGDriver WDTD1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void wdg_lld_init(void);
  void wdg_lld_start(WDGDriver *wdgp);
  void wdg_lld_stop(WDGDriver *wdgp);
  void wdg_lld_reset(WDGDriver *wdgp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_WDG == TRUE */

#endif /* HAL_WDG_LLD_H */

/** @} */
