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
 * Copyright (C) 2021 Siddharth Bharat Purohit, CubePilot Pty Ltd
 */

#include <AP_HAL/AP_HAL.h>

#if AP_CRASHDUMP_ENABLED

#include "CrashCatcher.h"
#include <ch.h>
#include <hal.h>
#include <string.h>

#include "CrashDump.h"
#include "hwdef/common/stm32_util.h"
#include "hwdef/common/watchdog.h"

#if HAL_USE_SDC || (HAL_USE_MMC_SPI && CRASHDUMP_SD_SPI_SUPPORTED_MCU)
#define CRASHDUMP_SD_AVAILABLE 1
#else
#define CRASHDUMP_SD_AVAILABLE 0
#endif

#define CRASHDUMP_SD_ENABLED (AP_CRASHDUMP_FATFS_ENABLED && CRASHDUMP_SD_AVAILABLE)

#if AP_CRASHDUMP_FLASH_ENABLED || CRASHDUMP_SD_ENABLED
static CrashCatcherInfo g_info;
#endif

#if AP_CRASHDUMP_FLASH_ENABLED
static bool do_flash_crash_dump = true;
#endif
#if CRASHDUMP_SD_ENABLED
static bool do_sd_crash_dump = false;
static uint32_t sd_dump_size = 0;
#endif

static void wait_for_watchdog_or_reset() NORETURN;
static void wait_for_watchdog_or_reset()
{
    if (!stm32_watchdog_enabled()) {
        NVIC_SystemReset();
    }
    while (true) {}
}

const CrashCatcherMemoryRegion* CrashCatcher_GetMemoryRegions(void)
{
#if CRASHDUMP_SD_ENABLED || !AP_CRASHDUMP_FLASH_ENABLED
    static const CrashCatcherMemoryRegion no_regions[] = {
        {0xFFFFFFFF, 0xFFFFFFFF, CRASH_CATCHER_BYTE}
    };
#endif
#if CRASHDUMP_SD_ENABLED
    if (crashdump_sd_ready()) {
        if (!do_sd_crash_dump) {
            return no_regions;
        }

        // dump all memory regions exactly once, no duplication.
        // thread stacks, BSS, heap are all inside these regions already.
        static const CrashCatcherMemoryRegion sd_regions[] = {
            HAL_CC_MEMORY_REGIONS,
            {0xFFFFFFFF, 0xFFFFFFFF, CRASH_CATCHER_BYTE}
        };
        return sd_regions;
    }
#endif

#if AP_CRASHDUMP_FLASH_ENABLED
    return crashdump_flash_memory_regions(do_flash_crash_dump);
#else
    return no_regions;
#endif
}

void CrashCatcher_DumpMemory(const void* pvMemory, CrashCatcherElementSizes elementSize, size_t elementCount)
{
    (void)pvMemory;
    (void)elementSize;
    (void)elementCount;
#if CRASHDUMP_SD_ENABLED
    if (do_sd_crash_dump) {
        uint32_t byte_count = elementCount * (elementSize == CRASH_CATCHER_BYTE ? 1 :
                                              elementSize == CRASH_CATCHER_HALFWORD ? 2 : 4);
        if (crashdump_sd_write(pvMemory, elementSize, elementCount)) {
            sd_dump_size += byte_count;
        } else {
            do_sd_crash_dump = false;
        }
    }
#endif
#if AP_CRASHDUMP_FLASH_ENABLED
    if (do_flash_crash_dump) {
        crashdump_flash_write(pvMemory, elementSize, elementCount);
    }
#endif
}


void CrashCatcher_DumpStart(const CrashCatcherInfo* pInfo)
{
    // Record the fault info for watchdog
    struct port_extctx* ctx = (struct port_extctx*)pInfo->sp;
    FaultType faultType = (FaultType)__get_IPSR();
    save_fault_watchdog(__LINE__, faultType, pInfo->sp, ctx->lr_thd);
#if AP_CRASHDUMP_FLASH_ENABLED || CRASHDUMP_SD_ENABLED
    g_info = *pInfo;
#endif
#if CRASHDUMP_SD_ENABLED
    // when SD crash dump is configured, use it exclusively - no flash fallback
    if (pInfo->isBKPT) {
        return;
    }
    if (crashdump_sd_ready()) {
#if AP_CRASHDUMP_FLASH_ENABLED
        do_flash_crash_dump = false;
#endif
        if (crashdump_sd_start()) {
            do_sd_crash_dump = true;
            sd_dump_size = 0;
        }
        // if SD start fails, we don't fall back to flash
        return;
    }
#endif
#if AP_CRASHDUMP_FLASH_ENABLED
    if (do_flash_crash_dump) {
        do_flash_crash_dump = crashdump_flash_start(pInfo);
    }
#endif
}

CrashCatcherReturnCodes CrashCatcher_DumpEnd(void)
{
#if CRASHDUMP_SD_ENABLED
    if (g_info.isBKPT) {
        return CRASH_CATCHER_EXIT;
    }
    if (crashdump_sd_ready()) {
        if (do_sd_crash_dump) {
            crashdump_sd_end(sd_dump_size);
        }
        do_sd_crash_dump = false;
    }
#endif
#if AP_CRASHDUMP_FLASH_ENABLED
    if (do_flash_crash_dump) {
        crashdump_flash_end();
        do_flash_crash_dump = false;
    }
    if (g_info.isBKPT) {
        return CRASH_CATCHER_EXIT;
    }
#endif
    // Preserve watchdog-reset semantics when active; otherwise reset now.
    wait_for_watchdog_or_reset();
}

#endif // AP_CRASHDUMP_ENABLED
