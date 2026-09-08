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
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* SNVS LP Secure Real Time Counter (SRTC) backend for Util::get/set_hw_rtc.
   Seconds since Unix epoch; 0 = counter not running / never set. */
uint32_t rt1176_snvs_srtc_get_seconds(void);
void rt1176_snvs_srtc_set_seconds(uint32_t seconds);

/* Fast-reboot signature in an SNVS LP General Purpose Register, which survives a
 * warm reset. */
void rt1176_snvs_set_boot_signature(uint32_t sig);
uint32_t rt1176_snvs_get_boot_signature(void);

#ifdef __cplusplus
}
#endif
