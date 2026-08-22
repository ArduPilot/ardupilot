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

#if AP_CRASHDUMP_ENABLED && AP_CRASHDUMP_FLASH_ENABLED

#include "CrashCatcher.h"
#include <ch.h>
#include <hal.h>
#include <string.h>

#include "CrashDump.h"
#include "hwdef/common/flash.h"
#include "hwdef/common/stm32_util.h"
#include "hwdef/common/watchdog.h"

#define REMAINDER_MEM_REGION_SIZE 15000U

// Preserve memory below the fault-time stack pointer for post-mortem stack
// unwinding. This margin is independent of the compiler stack-frame warning.
#ifndef AP_CRASHDUMP_FLASH_STACK_MARGIN
#define AP_CRASHDUMP_FLASH_STACK_MARGIN 1300U
#endif

extern "C" {
    extern uint32_t __crash_log_base__;
    extern uint32_t __crash_log_end__;
    extern uint32_t __ram0_start__;
    extern uint32_t __ram0_end__;
    extern uint32_t __heap_base__;
    extern uint32_t __heap_end__;
    extern uint32_t __bss_base__;
    extern uint32_t __bss_end__;
}

static void *dump_start_address;
static void *dump_end_address;
static uint32_t dump_size;
static uint8_t dump_buffer[32]; // H7 flash writes must be 32-byte aligned
static uint8_t buf_off;

uint32_t stm32_crash_dump_size(void)
{
    const uint32_t *page_addr = &__crash_log_base__;
    const uint32_t page_size = stm32_crash_dump_max_size();
    return page_addr[(page_size / sizeof(uint32_t)) - 1U];
}

uint32_t stm32_crash_dump_max_size(void)
{
    return uint32_t(reinterpret_cast<uintptr_t>(&__crash_log_end__) -
                    reinterpret_cast<uintptr_t>(&__crash_log_base__));
}

uint32_t stm32_crash_dump_addr(void)
{
    return uint32_t(reinterpret_cast<uintptr_t>(&__crash_log_base__));
}

static bool crashdump_flash_region_erased(void)
{
    const uint32_t *page = reinterpret_cast<const uint32_t *>(stm32_crash_dump_addr());
    for (uint32_t i = 0; i < stm32_crash_dump_max_size() / sizeof(uint32_t); i++) {
        if (page[i] != 0xFFFFFFFFU) {
            return false;
        }
    }
    return true;
}

bool crashdump_flash_start(const CrashCatcherInfo *info)
{
    uint8_t *sp = reinterpret_cast<uint8_t *>(info->sp);
    if (sp == nullptr || !is_address_in_memory(sp)) {
        return false;
    }

    uint8_t *region_start = static_cast<uint8_t *>(get_addr_mem_region_start_addr(sp));
    uint8_t *region_end = static_cast<uint8_t *>(get_addr_mem_region_end_addr(sp));
    if (region_start + AP_CRASHDUMP_FLASH_STACK_MARGIN > sp) {
        dump_start_address = region_start;
    } else {
        dump_start_address = sp - AP_CRASHDUMP_FLASH_STACK_MARGIN;
    }

    if (region_end < sp + HAL_PROCESS_STACK_SIZE) {
        dump_end_address = region_end;
    } else {
        dump_end_address = sp + HAL_PROCESS_STACK_SIZE;
    }

    dump_size = 0;
    buf_off = 0;
    if (!crashdump_flash_region_erased()) {
        return false;
    }
    stm32_watchdog_pat();
    stm32_flash_keep_unlocked(true);
    return true;
}

const CrashCatcherMemoryRegion *crashdump_flash_memory_regions(bool active)
{
    const uint32_t ram_start = uint32_t(reinterpret_cast<uintptr_t>(&__ram0_start__));
    const uint32_t ram_end = uint32_t(reinterpret_cast<uintptr_t>(&__ram0_end__));
    static CrashCatcherMemoryRegion regions[80] = {
        {0, 0, CRASH_CATCHER_BYTE},
        {
            uint32_t(reinterpret_cast<uintptr_t>(&ch_system)),
            uint32_t(reinterpret_cast<uintptr_t>(&ch_system)) + sizeof(ch_system),
            CRASH_CATCHER_BYTE
        }
    };

    regions[0].startAddress = active ?
                              uint32_t(reinterpret_cast<uintptr_t>(dump_start_address)) : ram_start;
    regions[0].endAddress = active ?
                            uint32_t(reinterpret_cast<uintptr_t>(dump_end_address)) : ram_end;
    regions[0].elementSize = CRASH_CATCHER_BYTE;

    const uint32_t max_dump = stm32_crash_dump_max_size();
    uint32_t total_dump_size = dump_size + buf_off + REMAINDER_MEM_REGION_SIZE;
    uint8_t curr_region = 2;

    for (thread_t *tp = chRegFirstThread(); tp != nullptr; tp = chRegNextThread(tp)) {
        const bool add_name = tp->name != nullptr &&
                              is_address_in_memory(const_cast<char *>(tp->name));
        const uint8_t required_regions = add_name ? 3U : 2U;
        // Leave one entry for the terminating sentinel.
        if (curr_region + required_regions >= ARRAY_SIZE(regions)) {
            goto finalise;
        }

        uint32_t total_stack;
        if (tp->wabase == static_cast<void *>(&__main_thread_stack_base__)) {
            total_stack = uint32_t(reinterpret_cast<const uint8_t *>(&__main_thread_stack_end__) -
                                   reinterpret_cast<const uint8_t *>(&__main_thread_stack_base__));
        } else {
            total_stack = uint32_t(reinterpret_cast<uintptr_t>(tp) -
                                   reinterpret_cast<uintptr_t>(tp->wabase));
        }

        if (add_name) {
            regions[curr_region].elementSize = CRASH_CATCHER_BYTE;
            regions[curr_region].startAddress = uint32_t(reinterpret_cast<uintptr_t>(tp->name));
            regions[curr_region++].endAddress = uint32_t(reinterpret_cast<uintptr_t>(tp->name)) + 13U;
        }

        regions[curr_region].elementSize = CRASH_CATCHER_BYTE;
        regions[curr_region].startAddress = uint32_t(reinterpret_cast<uintptr_t>(tp));
        regions[curr_region++].endAddress = uint32_t(reinterpret_cast<uintptr_t>(tp)) + sizeof(thread_t);
        regions[curr_region].elementSize = CRASH_CATCHER_BYTE;
        regions[curr_region].startAddress = uint32_t(reinterpret_cast<uintptr_t>(tp->wabase));
        regions[curr_region++].endAddress = uint32_t(reinterpret_cast<uintptr_t>(tp->wabase)) + total_stack;

        total_dump_size += total_stack;
        if (total_dump_size >= max_dump) {
            goto finalise;
        }
    }

    {
        const int32_t bss_size = int32_t(reinterpret_cast<uintptr_t>(&__bss_end__) -
                                         reinterpret_cast<uintptr_t>(&__bss_base__));
        int32_t available_space = int32_t(max_dump - total_dump_size);
        if (available_space < 0 || curr_region >= ARRAY_SIZE(regions) - 1U) {
            goto finalise;
        }
        regions[curr_region].elementSize = CRASH_CATCHER_BYTE;
        regions[curr_region].startAddress = uint32_t(reinterpret_cast<uintptr_t>(&__bss_base__));
        if (bss_size > available_space) {
            regions[curr_region++].endAddress =
                uint32_t(reinterpret_cast<uintptr_t>(&__bss_base__)) + available_space;
            total_dump_size += available_space;
        } else {
            regions[curr_region++].endAddress = uint32_t(reinterpret_cast<uintptr_t>(&__bss_end__));
            total_dump_size += bss_size;
        }

        const int32_t heap_size = int32_t(reinterpret_cast<uintptr_t>(&__heap_end__) -
                                          reinterpret_cast<uintptr_t>(&__heap_base__));
        available_space = int32_t(max_dump - total_dump_size);
        if (available_space < 0 || curr_region >= ARRAY_SIZE(regions) - 1U) {
            goto finalise;
        }
        regions[curr_region].elementSize = CRASH_CATCHER_BYTE;
        regions[curr_region].startAddress = uint32_t(reinterpret_cast<uintptr_t>(&__heap_base__));
        if (heap_size > available_space) {
            regions[curr_region++].endAddress =
                uint32_t(reinterpret_cast<uintptr_t>(&__heap_base__)) + available_space;
        } else {
            regions[curr_region++].endAddress = uint32_t(reinterpret_cast<uintptr_t>(&__heap_end__));
        }
    }

finalise:
    if (curr_region >= ARRAY_SIZE(regions)) {
        curr_region = ARRAY_SIZE(regions) - 1U;
    }
    regions[curr_region] = {0xFFFFFFFFU, 0xFFFFFFFFU, CRASH_CATCHER_BYTE};
    return regions;
}

static void flush_dump_buffer(void)
{
    if (buf_off != sizeof(dump_buffer)) {
        return;
    }
    stm32_flash_write(stm32_crash_dump_addr() + dump_size,
                      dump_buffer, sizeof(dump_buffer));
    dump_size += sizeof(dump_buffer);
    buf_off = 0;
    memset(dump_buffer, 0, sizeof(dump_buffer));
    stm32_watchdog_pat();
}

void crashdump_flash_write(const void *memory,
                           CrashCatcherElementSizes element_size,
                           size_t element_count)
{
    const uint8_t *bytes = static_cast<const uint8_t *>(memory);
    size_t count = 0;
    while (count < element_count) {
        if (dump_size + buf_off + sizeof(dump_size) >= stm32_crash_dump_max_size()) {
            memset(&dump_buffer[sizeof(dump_buffer) - sizeof(dump_size)],
                   0xFF, sizeof(dump_size));
            buf_off = sizeof(dump_buffer);
            return;
        }
        flush_dump_buffer();
        switch (element_size) {
        case CRASH_CATCHER_BYTE:
            dump_buffer[buf_off++] = bytes[count++];
            break;
        case CRASH_CATCHER_HALFWORD: {
            const uint16_t value = reinterpret_cast<const uint16_t *>(memory)[count++];
            dump_buffer[buf_off++] = value & 0xFFU;
            flush_dump_buffer();
            dump_buffer[buf_off++] = value >> 8U;
            break;
        }
        case CRASH_CATCHER_WORD: {
            const uint32_t value = reinterpret_cast<const uint32_t *>(memory)[count++];
            for (uint8_t i = 0; i < sizeof(value); i++) {
                dump_buffer[buf_off++] = (value >> (8U * i)) & 0xFFU;
                flush_dump_buffer();
            }
            break;
        }
        }
    }
}

CrashCatcherReturnCodes crashdump_flash_end(CrashCatcherReturnCodes return_code,
        bool is_breakpoint)
{
    if (dump_size + buf_off + sizeof(dump_size) >= stm32_crash_dump_max_size()) {
        memset(&dump_buffer[sizeof(dump_buffer) - sizeof(dump_size)],
               0xFF, sizeof(dump_size));
        buf_off = sizeof(dump_buffer);
    }
    if (buf_off > 0) {
        if (dump_size + sizeof(dump_buffer) >= stm32_crash_dump_max_size() &&
            buf_off < sizeof(dump_buffer)) {
            memcpy(&dump_buffer[sizeof(dump_buffer) - sizeof(dump_size)],
                   &dump_size, sizeof(dump_size));
            buf_off = sizeof(dump_buffer);
        }
        stm32_flash_write(stm32_crash_dump_addr() + dump_size,
                          dump_buffer, sizeof(dump_buffer));
        dump_size += buf_off;
        buf_off = 0;
        memset(dump_buffer, 0, sizeof(dump_buffer));
        stm32_watchdog_pat();
    }

    if (dump_size < stm32_crash_dump_max_size()) {
        memcpy(&dump_buffer[sizeof(dump_buffer) - sizeof(dump_size)],
               &dump_size, sizeof(dump_size));
        stm32_flash_write(stm32_crash_dump_addr() + stm32_crash_dump_max_size() -
                          sizeof(dump_buffer),
                          dump_buffer, sizeof(dump_buffer));
        stm32_watchdog_pat();
    }

    stm32_flash_keep_unlocked(false);
    if (return_code == CRASH_CATCHER_TRY_AGAIN && is_breakpoint) {
        return CRASH_CATCHER_EXIT;
    }
    return return_code;
}

#endif // AP_CRASHDUMP_ENABLED && AP_CRASHDUMP_FLASH_ENABLED
