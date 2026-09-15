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

/* SNVS LP SRTC backend for hal.util->get_hw_rtc()/set_hw_rtc(): it is in the
 * always-on domain, so it survives a warm reset. */
#include <zephyr/init.h>
#include <stdint.h>

#include "rt1176_snvs_rtc.h"

#define SNVS_LPCR     (*(volatile uint32_t *)(0x40C90000u + 0x38u))
#define SNVS_LPSRTCMR (*(volatile uint32_t *)(0x40C90000u + 0x50u))
#define SNVS_LPSRTCLR (*(volatile uint32_t *)(0x40C90000u + 0x54u))
#define SNVS_LPCR_SRTC_ENV 0x1u

/* SNVS_LP General Purpose Register 3, offset 0x100 + 3*4 (PERI_SNVS.h: LPGPR[]
   array offset 0x100, step 0x4, LPGPR0..3). Holds the fast-reboot signature;
   LPGPR3 chosen to match PX4's PX4_IMXRT_RTC_REBOOT_REG so the two bootloaders
   agree on where the "reboot to bootloader" word lives. */
#define SNVS_LPGPR3   (*(volatile uint32_t *)(0x40C90000u + 0x10Cu))
/* LPCR[GPR_Z_DIS] bit 24 (PERI_SNVS.h SNVS_LPCR_GPR_Z_DIS_MASK): when set, the
   LP GPRs are NOT zeroised by a tamper/security event, so the signature is
   retained across the reset. Same bit PX4's imxrt_common sets. */
#define SNVS_LPCR_GPR_Z_DIS 0x1000000u

uint32_t rt1176_snvs_srtc_get_seconds(void)
{
    if ((SNVS_LPCR & SNVS_LPCR_SRTC_ENV) == 0u) {
        return 0;
    }
    /* Double-read until stable: MR/LR are two halves of one live counter
       (same technique as fsl_snvs_lp.c's SNVS_LP_SRTC_GetSeconds). */
    uint32_t s1, s2;
    s2 = (SNVS_LPSRTCMR << 17) | (SNVS_LPSRTCLR >> 15);
    do {
        s1 = s2;
        s2 = (SNVS_LPSRTCMR << 17) | (SNVS_LPSRTCLR >> 15);
    } while (s1 != s2);
    return s2;
}

void rt1176_snvs_srtc_set_seconds(uint32_t seconds)
{
    /* The counter must be stopped to load MR/LR (fsl_snvs_lp.c does the
       same disable/write/enable dance). Each enable-bit change takes a few
       32 kHz cycles to synchronise into the LP domain - poll it. */
    SNVS_LPCR &= ~SNVS_LPCR_SRTC_ENV;
    while ((SNVS_LPCR & SNVS_LPCR_SRTC_ENV) != 0u) {
    }
    SNVS_LPSRTCMR = seconds >> 17;
    SNVS_LPSRTCLR = seconds << 15;
    SNVS_LPCR |= SNVS_LPCR_SRTC_ENV;
    while ((SNVS_LPCR & SNVS_LPCR_SRTC_ENV) == 0u) {
    }
}

void rt1176_snvs_set_boot_signature(uint32_t sig)
{
    /* Keep the GPRs retention-enabled before writing, or a subsequent
       security event would clear the signature we just stored. */
    SNVS_LPCR |= SNVS_LPCR_GPR_Z_DIS;
    SNVS_LPGPR3 = sig;
}

uint32_t rt1176_snvs_get_boot_signature(void)
{
    return SNVS_LPGPR3;
}

static int rt1176_snvs_rtc_init(void)
{
    /* Start the SRTC if it isn't already running (first boot / LP domain
       lost power). Deliberately does NOT reset the count when it's already
       live - that continuity across reboots is the whole point. */
    if ((SNVS_LPCR & SNVS_LPCR_SRTC_ENV) == 0u) {
        SNVS_LPCR |= SNVS_LPCR_SRTC_ENV;
        while ((SNVS_LPCR & SNVS_LPCR_SRTC_ENV) == 0u) {
        }
    }
    return 0;
}

SYS_INIT(rt1176_snvs_rtc_init, APPLICATION, 42);
