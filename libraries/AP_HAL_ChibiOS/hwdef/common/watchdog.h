#pragma once

#include "hal.h"

#if defined(STM32_HW)

    #ifdef __cplusplus
    extern "C" {
    #endif

    /*
      setup the watchdog
    */
    void stm32_watchdog_init(void);

    /*
      pat the dog, to prevent a reset. If not called for STM32_WDG_TIMEOUT_MS
      after stm32_watchdog_init() then MCU will reset
    */
    void stm32_watchdog_pat(void);

    /*
      return true if reboot was from a watchdog reset
    */
    bool stm32_was_watchdog_reset(void);

    /*
      return true if reboot was from a software reset
    */
    bool stm32_was_software_reset(void);
        
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

#endif // STM32_HW

#if defined(RP2350)

/*
  Shared SCRATCH register constants used for reset-cause breadcrumbs.
  These magic sentinels survive PSM-level resets (WD reset, software reset)
  and are used to identify the cause of the last reboot.
  They are stored in WATCHDOG->SCRATCH[RP2350_RESET_DIAG_SCRATCH_IDX].
  Also referenced in board.c (unhandled-exception trap) and Scheduler.cpp
  (explicit reboot path) — keep these values consistent across all files.
*/
#define RP2350_RESET_DIAG_SCRATCH_IDX          7U
#define RP2350_RESET_DIAG_UNHANDLED_EXCEPTION  0x55484E44U  /* 'UHND' */
#define RP2350_RESET_DIAG_SCHEDULER_REBOOT     0x53434852U  /* 'SCHR' */

#ifdef __cplusplus
extern "C" {
#endif

/*
  initialise the RP2350 watchdog with a 2 second timeout
*/
void rp2350_watchdog_init(void);

/*
  pat the RP2350 watchdog to prevent a reset
*/
void rp2350_watchdog_pat(void);

/*
  return true if the last reboot was caused by the watchdog timer.
  Uses SCRATCH[6] canary (not WATCHDOG->REASON, which is cleared by the
  WD-triggered PSM reset on RP2350).  Returns false once
  rp2350_watchdog_clear_reason() has been called.
*/
bool rp2350_was_watchdog_reset(void);

/* return true if the last reboot was caused by a software (forced) reset */
bool rp2350_was_software_reset(void);

/* mark the watchdog reset reason as consumed */
void rp2350_watchdog_clear_reason(void);

/* no-op on RP2350: REASON is cleared by WD-triggered PSM reset; detection
   uses SCRATCH[6] canary written by rp2350_watchdog_pat() instead */
void rp2350_watchdog_save_reason(void);

/* persistent data save/load across resets (stub; SCRATCH registers not yet used) */
void rp2350_watchdog_save(const uint32_t *data, uint32_t nwords);
void rp2350_watchdog_load(uint32_t *data, uint32_t nwords);

#ifdef __cplusplus
}
#endif

/*
  Redirect the stm32_-prefixed watchdog API used throughout the ChibiOS HAL
  and bootloader to the correctly named RP2350 implementations above.
  These are #define macros so no stm32_-named symbols are emitted for RP2350.
*/
#define stm32_watchdog_init()          rp2350_watchdog_init()
#define stm32_watchdog_pat()           rp2350_watchdog_pat()
#define stm32_was_watchdog_reset()     rp2350_was_watchdog_reset()
#define stm32_was_software_reset()     rp2350_was_software_reset()
#define stm32_watchdog_clear_reason()  rp2350_watchdog_clear_reason()
#define stm32_watchdog_save_reason()   rp2350_watchdog_save_reason()
#define stm32_watchdog_save(d, n)      rp2350_watchdog_save((d), (n))
#define stm32_watchdog_load(d, n)      rp2350_watchdog_load((d), (n))

#endif // RP2350
    
