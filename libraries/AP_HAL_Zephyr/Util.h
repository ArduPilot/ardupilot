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
 *
 * Code by @davidbuzz and Claude
 */
#pragma once

/* <AP_HAL/AP_HAL.h> FIRST, exactly as AP_HAL_ChibiOS/Util.h does, and NOT
 * zephyr/kernel.h first - the kernel headers define macros that collide. */
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Util.h>
#ifdef __ZEPHYR__
#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
/* No <sys_malloc.h> here: it lives in Zephyr's common-libc include dir, which
   vanishes when a board uses EXTERNAL_LIBC (native_sim = host glibc). Its one
   prototype, malloc_runtime_stats_get(), is forward-declared by Util.cpp. */
#include <zephyr/sys/mem_stats.h>
#else
#include <stdio.h>
#endif

namespace Zephyr {

class Util : public AP_HAL::Util {
public:
    /* Was the last boot caused by a watchdog reset? Reads the SRC reset-cause
       via Zephyr's hwinfo driver. Mirrors AP_HAL_ChibiOS::Util. */
    bool was_watchdog_reset() const override;

    /* Factory board unique ID via Zephyr's portable hwinfo_get_device_id(). */
    bool get_system_id(char buf[50]) override;
    bool get_system_id_unformatted(uint8_t buf[], uint8_t &len) override;

#if HAL_ENABLE_THREAD_STATISTICS
    // request information on running threads (@SYS/threads.txt).
    // Mirrors AP_HAL_ChibiOS/Util.h; enabled by ./waf configure --enable-stats.
    void thread_info(ExpandingString &str) override;
#endif
    /* Wall clock. On RT1176 these are backed by the SNVS LP SRTC - a
       32768 Hz counter in the always-on/battery domain that survives warm,
       watchdog, and (with VBAT fitted) power-off resets, matching the
       STM32 backup-domain RTC ChibiOS uses. Other boards keep the RAM-only
       fallback (GPS re-syncs it each boot). */
    void set_hw_rtc(uint64_t time_utc_usec) override;
    uint64_t get_hw_rtc() const override;

    /* @SYS/mem.txt - heap/pool report, ChibiOS MemInfoV1 format. */
    void mem_info(ExpandingString &str) override;

    /* DMA-safe allocation. THIS IS THE PREREQUISITE FOR SPI DMA: a buffer the engine
     * cannot reach, or that shares a cache line, corrupts silently. */
    void *malloc_type(size_t size, AP_HAL::Util::Memory_Type mem_type) override;
    void free_type(void *ptr, size_t size, AP_HAL::Util::Memory_Type mem_type) override;

    uint32_t available_memory(void) override;

#if AP_CRASHDUMP_ENABLED
    /* On-board crash dump retrieval, backing AP_Filesystem_Sys.cpp. */
    size_t last_crash_dump_size() const override;
    void *last_crash_dump_ptr() const override;
#endif

#if AP_BOOTLOADER_FLASHING_ENABLED
    /* In-app bootloader update (MAV_CMD_FLASH_BOOTLOADER), ChibiOS parity -
       flashes the ROMFS-embedded "bootloader.bin" over the resident
       bootloader via the BootROM flash API. RT1176-only today (needs
       CONFIG_AP_RT1176_ROMAPI_FLASH; boards.py sets the enable define only
       for mr_vmu_rt1176 app builds). */
    FlashBootloader flash_bootloader() override;
#endif

    bool get_random_vals(uint8_t *data, size_t size) override
    {
#ifdef __ZEPHYR__
        sys_rand_get(data, size);
        return true;
#else
        static FILE *urandom;
        if (urandom == nullptr) {
            urandom = fopen("/dev/urandom", "rb");
        }
        if (urandom == nullptr) {
            return false;
        }
        return fread(data, size, 1, urandom) == 1;
#endif
    }

private:
    uint64_t _rtc_usec = 0;
};

}  // namespace Zephyr
