#pragma once

#include "hal.h"

/*
  define for controlling how long the watchdog is set for.
*/
#ifndef STM32_WDG_TIMEOUT_MS
#define STM32_WDG_TIMEOUT_MS 2048
#endif
#if STM32_WDG_TIMEOUT_MS < 1000
#error "STM32_WDG_TIMEOUT_MS must be at least 1000"
#endif

/*
  SysTick based lockup detection, needs crash dump support and a
  free-running (TIM based) system timer so SysTick is free
 */
#ifndef AP_WATCHDOG_LOCKUP_DETECT_ENABLED
#if AP_CRASHDUMP_ENABLED && OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING && \
    !defined(HAL_BOOTLOADER_BUILD) && !defined(IOMCU_FW)
#define AP_WATCHDOG_LOCKUP_DETECT_ENABLED 1
#else
#define AP_WATCHDOG_LOCKUP_DETECT_ENABLED 0
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*
  setup the watchdog
 */
void stm32_watchdog_init(void);

/*
  return true if the watchdog has been started
 */
bool stm32_watchdog_enabled(void);

/*
  pat the dog, to prevent a reset. If not called for STM32_WDG_TIMEOUT_MS
  after stm32_watchdog_init() then MCU will reset
 */
void stm32_watchdog_pat(void);

/*
  start SysTick based lockup detection, triggering a crash dump
  before the watchdog fires
 */
#if AP_WATCHDOG_LOCKUP_DETECT_ENABLED
void stm32_lockup_detect_start(void);
void stm32_lockup_detect_pause(bool pause);
#else
static inline void stm32_lockup_detect_start(void) {}
static inline void stm32_lockup_detect_pause(bool pause) { (void)pause; }
#endif

/*
  return true if reboot was from a watchdog reset
 */
bool stm32_was_watchdog_reset(void);

/*
  return true if reboot was from a software reset
 */
bool stm32_was_software_reset(void);

/*
  causes of the last reset, as returned by stm32_reset_cause(). More
  than one can be set, e.g. a power-on reset also sets the pin flag on
  some MCUs. These values are sent by the IOMCU, so don't renumber them
 */
#define STM32_RESET_CAUSE_POWER_ON  (1U<<0) // power-on/power-down reset
#define STM32_RESET_CAUSE_PIN       (1U<<1) // NRST pin
#define STM32_RESET_CAUSE_SOFTWARE  (1U<<2)
#define STM32_RESET_CAUSE_IWDG      (1U<<3) // independent watchdog
#define STM32_RESET_CAUSE_WWDG      (1U<<4) // window watchdog
#define STM32_RESET_CAUSE_LOW_POWER (1U<<5)
#define STM32_RESET_CAUSE_BROWNOUT  (1U<<6) // brownout reset above the power-on threshold

/*
  return the causes of the last reset as STM32_RESET_CAUSE_* flags
 */
uint8_t stm32_reset_cause(void);

/*
  save the reset reason code
 */
void stm32_watchdog_save_reason(void);

/*
  clear reset reason code
 */
void stm32_watchdog_clear_reason(void);

/*
  save persistent watchdog data
 */
void stm32_watchdog_save(const uint32_t *data, uint32_t nwords);

/*
  load persistent watchdog data
 */
void stm32_watchdog_load(uint32_t *data, uint32_t nwords);
    
#ifdef __cplusplus
}
#endif
    
