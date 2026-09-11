/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 * Periodic RP2350 performance report: main loop rate, rate thread rate, the
 * per-core loads and the XIP cache behaviour. Driven from the HAL's own
 * monitor thread so the vehicle code carries none of it.
 */
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#if defined(RP2350) && AP_RP2350_DEBUG_REPORT_ENABLED
// called at 0.1 Hz from the monitor thread
void rp2350_perf_report(void);
#endif
