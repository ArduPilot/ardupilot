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

#include <CrashCatcher.h>
#include <ch.h>
#include <hal.h>

#include "CrashDump.h"
#include "hwdef/common/stm32_util.h"
#include "hwdef/common/watchdog.h"

CRASH_CATCHER_TEST_WRITEABLE CrashCatcherReturnCodes g_crashCatcherDumpEndReturn = CRASH_CATCHER_TRY_AGAIN;
static CrashCatcherInfo g_info;
static bool do_flash_crash_dump = true;

const CrashCatcherMemoryRegion *CrashCatcher_GetMemoryRegions(void)
{
    return crashdump_flash_memory_regions(do_flash_crash_dump);
}

void CrashCatcher_DumpMemory(const void *memory,
                             CrashCatcherElementSizes element_size,
                             size_t element_count)
{
    if (do_flash_crash_dump) {
        crashdump_flash_write(memory, element_size, element_count);
    }
}

void CrashCatcher_DumpStart(const CrashCatcherInfo *info)
{
    // Record the fault info for watchdog
    struct port_extctx *ctx = reinterpret_cast<struct port_extctx *>(info->sp);
    FaultType fault_type = static_cast<FaultType>(__get_IPSR());
    save_fault_watchdog(__LINE__, fault_type, info->sp, ctx->lr_thd);
    g_info = *info;
    if (do_flash_crash_dump) {
        do_flash_crash_dump = crashdump_flash_start(info);
    }
}

CrashCatcherReturnCodes CrashCatcher_DumpEnd(void)
{
    if (do_flash_crash_dump) {
        return crashdump_flash_end(g_crashCatcherDumpEndReturn, g_info.isBKPT);
    }
    do_flash_crash_dump = false;
    return CRASH_CATCHER_TRY_AGAIN;
}

#endif // AP_CRASHDUMP_ENABLED
