#pragma once

/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
  RP2350 watchdog API, included from hwdef/common/watchdog.h
 */

#include <stdbool.h>
#include <stdint.h>

#if defined(RP2350)

/*
  Shared SCRATCH register constants used for reset-cause breadcrumbs.
  These magic sentinels survive PSM-level resets (WD reset, software reset)
  and are used to identify the cause of the last reboot.
  They are stored in WATCHDOG->SCRATCH[RP2350_RESET_DIAG_SCRATCH_IDX].
  Also referenced in board_rp2350.c (unhandled-exception trap) and Scheduler.cpp
  (explicit reboot path) -- keep these values consistent across all files.
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
  WD-triggered PSM reset on RP2350).
*/
bool rp2350_was_watchdog_reset(void);

/* return true if the last reboot was caused by a software (forced) reset */
bool rp2350_was_software_reset(void);

/* clear the reset markers so the next boot starts fresh */
void rp2350_watchdog_clear_reason(void);

/* save the reset reason before anything can pat the watchdog */
void rp2350_watchdog_save_reason(void);

/* persistent data save/load across watchdog resets, kept in no-init SRAM */
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
