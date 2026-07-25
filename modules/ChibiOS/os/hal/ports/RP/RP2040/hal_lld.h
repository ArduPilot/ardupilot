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
 * @file    RP2040/hal_lld.h
 * @brief   RP2040 HAL subsystem low level driver header.
 *
 * @addtogroup HAL
 * @{
 */

#ifndef HAL_LLD_H
#define HAL_LLD_H

/*
 * Registry definitions.
 */
#include "rp_registry.h"

/* From Pico-SDK */
#include "hardware/clocks.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Platform identification macros
 * @{
 */
#if defined(RP2040) || defined(__DOXYGEN__)
#define PLATFORM_NAME           "RP2040"

#else
#error "RP2040 device not specified"
#endif
/** @} */

/**
 * @name    Internal clock sources
 * @{
 */
#define RP_ROSCCLK              6500000     /**< 6.5MHz internal clock.     */
/** @} */

/**
 * @brief   Dynamic clock supported.
 */
#define HAL_LLD_USE_CLOCK_MANAGEMENT

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   Disables the clocks initialization in the HAL.
 */
#if !defined(RP_NO_INIT) || defined(__DOXYGEN__)
#define RP_NO_INIT                          FALSE
#endif

/**
 * @brief   Starts core 1 after initialization.
 */
#if !defined(RP_CORE1_START) || defined(__DOXYGEN__)
#define RP_CORE1_START                      FALSE
#endif

/**
 * @brief   Symbol for core 1 vectors table.
 */
#if !defined(RP_CORE1_VECTORS_TABLE) || defined(__DOXYGEN__)
#define RP_CORE1_VECTORS_TABLE              _vectors
#endif

/**
 * @brief   Symbol for core 1 entry point.
 */
#if !defined(RP_CORE1_ENTRY_POINT) || defined(__DOXYGEN__)
#define RP_CORE1_ENTRY_POINT                _crt0_c1_entry
#endif

/**
 * @brief   Symbol for core 1 initial MSP position.
 */
#if !defined(RP_CORE1_STACK_END) || defined(__DOXYGEN__)
#define RP_CORE1_STACK_END                  __c1_main_stack_end__
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*
 * Configuration-related checks.
 */
#if !defined(RP2040_MCUCONF)
#error "Using a wrong mcuconf.h file, RP2040_MCUCONF not defined"
#endif

/*
 * Board files sanity checks.
 */
#if !defined(RP_XOSCCLK)
#error "RP_XOSCCLK not defined in board.h"
#endif

/**
 * @name    Various clock points.
 * @{
 */
#define RP_GPOUT0_CLK           hal_lld_get_clock_point(clk_gpout0)
#define RP_GPOUT1_CLK           hal_lld_get_clock_point(clk_gpout1)
#define RP_GPOUT2_CLK           hal_lld_get_clock_point(clk_gpout2)
#define RP_GPOUT3_CLK           hal_lld_get_clock_point(clk_gpout3)
#define RP_REF_CLK              hal_lld_get_clock_point(clk_ref)
#define RP_CORE_CLK             hal_lld_get_clock_point(clk_sys)
#define RP_PERI_CLK             hal_lld_get_clock_point(clk_peri)
#define RP_USB_CLK              hal_lld_get_clock_point(clk_usb)
#define RP_ADC_CLK              hal_lld_get_clock_point(clk_adc)
#define RP_RTC_CLK              hal_lld_get_clock_point(clk_rtc)
/** @} */

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

#if defined(HAL_LLD_USE_CLOCK_MANAGEMENT) || defined(__DOXYGEN__)
/**
 * @brief   Type of a clock configuration structure.
 */
typedef struct {
  uint32_t          dummy;
} halclkcfg_t;
#endif /* defined(HAL_LLD_USE_CLOCK_MANAGEMENT) */

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/* Various helpers.*/
#include "nvic.h"
#include "cache.h"
#include "rp_isr.h"
#include "rp_fifo.h"
#include "rp_dma.h"

#ifdef __cplusplus
extern "C" {
#endif
  void hal_lld_init(void);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Driver inline functions.                                                  */
/*===========================================================================*/

__STATIC_INLINE void hal_lld_peripheral_reset(uint32_t mask) {

  RESETS->RESET |=  mask;
}

__STATIC_INLINE void hal_lld_peripheral_unreset(uint32_t mask) {

  RESETS->RESET &= ~mask;
  while ((RESETS->RESET_DONE & mask) == 0U) {
    /* Waiting for reset.*/
  }
}

#if defined(HAL_LLD_USE_CLOCK_MANAGEMENT) || defined(__DOXYGEN__)
/**
 * @brief   Switches to a different clock configuration
 *
 * @param[in] ccp       pointer to clock a @p halclkcfg_t structure
 * @return              The clock switch result.
 * @retval false        if the clock switch succeeded
 * @retval true         if the clock switch failed
 *
 * @notapi
 */
__STATIC_INLINE bool hal_lld_clock_switch_mode(const halclkcfg_t *ccp) {

  (void)ccp;

  return false;
}

/**
 * @brief   Returns the frequency of a clock point in Hz.
 *
 * @param[in] clkpt     clock point to be returned
 * @return              The clock point frequency in Hz or zero if the
 *                      frequency is unknown.
 *
 * @notapi
 */
__STATIC_INLINE halfreq_t hal_lld_get_clock_point(halclkpt_t clkpt) {

  osalDbgAssert(clkpt < CLK_COUNT, "invalid clock point");

  return clock_get_hz(clkpt);
}
#endif /* defined(HAL_LLD_USE_CLOCK_MANAGEMENT) */

#endif /* HAL_LLD_H */

/** @} */
