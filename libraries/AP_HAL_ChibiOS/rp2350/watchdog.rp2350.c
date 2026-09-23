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
  RP2350 watchdog support, standing in for the STM32 IWDG code in
  hwdef/common/watchdog.c, which chibios_board_rp2350.mk does not build
 */

#include "hal.h"
#include "watchdog.h"

#if defined(RP2350)

/*
  default watchdog timeout in milliseconds for RP2350
*/
#ifndef RP2350_WDG_TIMEOUT_MS
#define RP2350_WDG_TIMEOUT_MS 2000U
#endif

static const WDGConfig rp2350_wdg_cfg = {
    .rlr = RP2350_WDG_TIMEOUT_MS,
};

static bool rp2350_watchdog_enabled;

/*
  SCRATCH[6] dual-purpose register for WD detection on RP2350.
  On RP2350, the WD PSM reset (ChibiOS WDSEL=ALL_BITS) resets the WATCHDOG
  peripheral itself, so WATCHDOG->REASON is always 0 after any WD-triggered
  reset.  SCRATCH registers survive PSM-level resets (confirmed by hardware
  test: SCRATCH preserved across WD-triggered PSM reset, 2026-04-10).
  We therefore use SCRATCH[6] as the sole WD detection mechanism:
    RP2350_WDG_ARMED_CANARY ('WDOG'): written on every rp2350_watchdog_pat()
      call (AP_Bootloader only pats on its CAN path, which no RP2350
      bootloader builds).  When WD fires and PSM-resets the board,
      SCRATCH[6] still holds the canary, allowing detection at next boot.
    RP2350_WDG_REASON_CLEARED: written by rp2350_watchdog_clear_reason() to
      prevent re-detection after the reason has been consumed.
  Detection is cached in RAM by rp2350_watchdog_save_reason(), which board.c
  calls at boot before anything can pat, and returned by
  rp2350_was_watchdog_reset().
  Explicit Scheduler::reboot() writes RP2350_RESET_DIAG_SCHEDULER_REBOOT to
  SCRATCH[7], enabling false-positive suppression for software reboots.
*/
#define RP2350_WDG_ARMED_CANARY   0x57444F47U  /* 'WDOG' - app is petting WD */
#define RP2350_WDG_REASON_CLEARED 0xDEADC0DEU  /* reason consumed, do not re-report */

static bool rp2350_wd_reason_saved;
static bool rp2350_wd_reset_detected;

/*
  initialise and start the RP2350 watchdog.
  Must be called before rp2350_watchdog_pat()
*/
void rp2350_watchdog_init(void)
{
    wdgStart(&WDGD1, &rp2350_wdg_cfg);
    rp2350_watchdog_enabled = true;
}

/*
  reload the watchdog counter to prevent a reset.
  Also writes the armed canary to SCRATCH[6] on every call so that any
  subsequent WD-triggered PSM reset can be detected at next boot.
*/
void rp2350_watchdog_pat(void)
{
    if (rp2350_watchdog_enabled) {
        WATCHDOG->SCRATCH[6] = RP2350_WDG_ARMED_CANARY;
        wdgReset(&WDGD1);
    }
}

/*
  return true if the last reboot was caused by the watchdog timer.
  This is on the core1 hot path and relocated to SRAM, so it must not call
  into flash; board.c saves the reason before anything can ask.
*/
bool rp2350_was_watchdog_reset(void)
{
    return rp2350_wd_reset_detected;
}

/*
  clear the reset markers so the next boot starts fresh. As on STM32 this
  leaves the reason already saved for this boot in place.
  The next rp2350_watchdog_pat() will re-arm the canary for future WD resets.
*/
void rp2350_watchdog_clear_reason(void)
{
    WATCHDOG->SCRATCH[6] = RP2350_WDG_REASON_CLEARED;
    // otherwise every later watchdog reset in this power cycle would be
    // taken for a software reboot
    if (WATCHDOG->SCRATCH[RP2350_RESET_DIAG_SCRATCH_IDX] == RP2350_RESET_DIAG_SCHEDULER_REBOOT) {
        WATCHDOG->SCRATCH[RP2350_RESET_DIAG_SCRATCH_IDX] = 0U;
    }
}

/*
  save the reset reason before any pat can overwrite SCRATCH[6]
*/
void rp2350_watchdog_save_reason(void)
{
    if (rp2350_wd_reason_saved) {
        return;
    }
    rp2350_wd_reason_saved = true;
    rp2350_wd_reset_detected =
        (WATCHDOG->SCRATCH[6] == RP2350_WDG_ARMED_CANARY) &&
        (WATCHDOG->SCRATCH[RP2350_RESET_DIAG_SCRATCH_IDX] != RP2350_RESET_DIAG_SCHEDULER_REBOOT);
}

/*
 * RP2350 persistent data save/load across WD resets using a no-init SRAM buffer.
 * SRAM on RP2350 is in the always-on power domain: NOT reset by the PSM watchdog reset.
 * The buffer is in ".ram0" (after __ram0_noinit__) so ChibiOS CRT0 doesn't zero it.
 * AP_Bootloader only zeros its own BSS, leaving the app noinit region intact.
 * 32 words gives comfortable headroom for future growth.
 */
#define RP2350_WD_PERSIST_MAGIC     0x5750444fU  /* 'WPDO' little-endian */
#define RP2350_WD_PERSIST_MAX_WORDS 32U

typedef struct {
    uint32_t magic;                              /* RP2350_WD_PERSIST_MAGIC when valid */
    uint32_t nwords;                             /* number of valid words in data[] */
    uint32_t data[RP2350_WD_PERSIST_MAX_WORDS];  /* copy of HAL::Util::PersistentData */
} rp2350_wd_persist_t;

/*
 * Placed in ".ram0" (no-init section): CRT0 doesn't touch anything past __ram0_noinit__,
 * so the struct retains its value across WD-triggered PSM resets.
 */
static rp2350_wd_persist_t wd_persist_buf __attribute__((section(".ram0")));

void rp2350_watchdog_save(const uint32_t *data, uint32_t nwords)
{
    /* Cap at our buffer limit to prevent overflow. */
    if (nwords > RP2350_WD_PERSIST_MAX_WORDS) {
        nwords = RP2350_WD_PERSIST_MAX_WORDS;
    }

    wd_persist_buf.nwords = nwords;
    for (uint32_t i = 0U; i < nwords; i++) {
        wd_persist_buf.data[i] = data[i];
    }
    /* Write magic last so a partial write leaves an invalid header. */
    wd_persist_buf.magic = RP2350_WD_PERSIST_MAGIC;
}

void rp2350_watchdog_load(uint32_t *data, uint32_t nwords)
{
    /* Reject invalid header (power-on-reset, no prior save, or partial write). */
    if (wd_persist_buf.magic != RP2350_WD_PERSIST_MAGIC) {
        return;
    }

    uint32_t saved = wd_persist_buf.nwords;
    if (saved > RP2350_WD_PERSIST_MAX_WORDS) {
        saved = RP2350_WD_PERSIST_MAX_WORDS;  /* guard against corrupt nwords */
    }
    const uint32_t copy = (saved < nwords) ? saved : nwords;

    for (uint32_t i = 0U; i < copy; i++) {
        data[i] = wd_persist_buf.data[i];
    }
    /* Zero any words the caller wants that we did not save. */
    for (uint32_t i = copy; i < nwords; i++) {
        data[i] = 0U;
    }

    /* Invalidate the buffer so a cold POR (even without re-arm by save())
     * does not replay stale data on the next boot. */
    wd_persist_buf.magic = 0U;
}

bool rp2350_was_software_reset(void)
{
    /* RP2350 WATCHDOG REASON.FORCE bit indicates a forced (software) reset */
    return (WATCHDOG->REASON & WATCHDOG_REASON_FORCE) != 0U;
}

#endif // RP2350
