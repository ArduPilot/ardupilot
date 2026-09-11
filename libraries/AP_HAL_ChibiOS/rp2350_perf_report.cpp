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

// ch.h/hal.h bring in the generated hwdef.h, which is where RP2350 is defined;
// the guard below is only correct once they have been seen.
#include <ch.h>
#include "hal.h"

#include "rp2350_perf_report.h"

#if defined(RP2350) && AP_RP2350_DEBUG_REPORT_ENABLED && !defined(HAL_BOOTLOADER_BUILD)

#include <AP_HAL/AP_HAL.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include "rp2350_pc_sampler.h"

extern const AP_HAL::HAL& hal;

extern "C" void rp2350_xip_cache_stats(uint32_t *hit, uint32_t *acc);
extern "C" void rp2350_xip_park_stats(uint32_t *count, uint32_t *max_us);

void rp2350_perf_report(void)
{
    // the numbers are not meaningful until the loop has settled
    if (AP_HAL::millis() < 5000) {
        return;
    }

    const float c1_pct = hal.scheduler->get_core1_load_pct();
    // c1_pct sentinels: -2.0 = SMP active but core1 thread not yet started
    // (suppress to avoid bogus single-core output); -1.0 = non-SMP target
    // (print single-core format); >= 0.0 = SMP ready (print dual-core format).
    if (c1_pct < -1.5f) {
        return;
    }

    const float main_hz  = AP::scheduler().get_filtered_loop_rate_hz();
    const float load_pct = AP::scheduler().load_average() * 100.0f;
    auto &ins = AP::ins();
    const uint32_t rate_hz = ins.get_raw_gyro_rate_hz() / ins.get_rate_decimation();

    // XIP cache hit rate over the interval since the last report
    char xip[16] = "";
    uint32_t xip_hit = 0, xip_acc = 0;
    rp2350_xip_cache_stats(&xip_hit, &xip_acc);
    if (xip_acc > 0) {
        hal.util->snprintf(xip, sizeof(xip), " xip=%.0f%%",
                           (double)xip_hit * 100.0 / (double)xip_acc);
    }

    if (c1_pct >= 0.0f) {
        hal.console->printf("Perf: main=%.0fHz rate=%uHz core0load:%.0f%% core1load:%.0f%%%s\n",
                            main_hz, (unsigned)rate_hz, load_pct, c1_pct, xip);
        gcs().send_text(MAV_SEVERITY_INFO,
                        "Perf: main=%.0fHz rate=%uHz core0load:%.0f%% core1load:%.0f%%%s",
                        main_hz, (unsigned)rate_hz, load_pct, c1_pct, xip);
    } else {
        hal.console->printf("Perf: main=%.0fHz rate=%uHz core0load:%.0f%%%s\n",
                            main_hz, (unsigned)rate_hz, load_pct, xip);
        gcs().send_text(MAV_SEVERITY_INFO,
                        "Perf: main=%.0fHz rate=%uHz core0load:%.0f%%%s",
                        main_hz, (unsigned)rate_hz, load_pct, xip);
    }

    // Core1 park diagnostic: flash-op XIP lockouts freeze core1, which is
    // where the rate loop runs, so a large park max explains rate loop jitter.
    uint32_t park_n = 0, park_max = 0;
    rp2350_xip_park_stats(&park_n, &park_max);
    if (park_n > 0) {
        gcs().send_text(MAV_SEVERITY_INFO, "XIPpark: n=%lu max=%luus",
                        (unsigned long)park_n, (unsigned long)park_max);
    }

#if AP_RP2350_PC_SAMPLER_ENABLED
    // Arm core0's sampler on the first report (this runs on core0); core1 is
    // armed by the HAL when the pinned thread starts. Emit the core1 histogram
    // top-N; addresses are attributed to functions offline.
    rp2350_pc_sampler_init_core0();
    {
        // send_text uses AP's cut-down vsnprintf, which has no %.*s; terminate
        // each line in place and send it with plain %s.
        char pbuf[320];
        rp2350_pc_sampler_dump(1, 16, pbuf, sizeof(pbuf));
        char *p = pbuf;
        while (*p != '\0') {
            char *nl = strchr(p, '\n');
            if (nl != nullptr) {
                *nl = '\0';
            }
            if (*p != '\0') {
                gcs().send_text(MAV_SEVERITY_INFO, "PROFc1 %s", p);
            }
            if (nl == nullptr) {
                break;
            }
            p = nl + 1;
        }
    }
#endif
}

#endif  // RP2350 && AP_RP2350_DEBUG_REPORT_ENABLED && !HAL_BOOTLOADER_BUILD
