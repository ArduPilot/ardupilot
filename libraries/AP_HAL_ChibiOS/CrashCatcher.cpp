/*
 * Copyright (C) 2018 Adam Green (https://github.com/adamgreen)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Adapted for ArduPilot.
 */

#include <AP_HAL/AP_HAL.h>

#if AP_CRASHDUMP_ENABLED

#include "CrashCatcher.h"

#include <string.h>

#define LR_PSP (1U << 2)
#define LR_FLOAT (1U << 4)
#define PSR_STACK_ALIGN (1U << 9)

static constexpr uintptr_t COPROCESSOR_ACCESS_CONTROL_REGISTER = 0xE000ED88U;
static constexpr uintptr_t FAULT_STATUS_REGISTERS = 0xE000ED28U;

alignas(8) uint32_t g_crashCatcherStack[CRASH_CATCHER_STACK_WORD_COUNT];

struct CrashCatcherObject {
    const CrashCatcherExceptionRegisters *exception_registers;
    CrashCatcherStackedRegisters *stacked_registers;
    uint32_t flags;
    CrashCatcherInfo info;
};

static_assert(sizeof(CrashCatcherExceptionRegisters) == 12U * sizeof(uint32_t),
              "CrashCatcher exception register layout changed");
static_assert(offsetof(CrashCatcherStackedRegisters, floats) == 8U * sizeof(uint32_t),
              "CrashCatcher stacked register layout changed");

static void *address_to_pointer(uint32_t address)
{
    return reinterpret_cast<void *>(static_cast<uintptr_t>(address));
}

static uint32_t exception_stack_address(const CrashCatcherExceptionRegisters *exception_registers)
{
    if ((exception_registers->exceptionLR & LR_PSP) != 0) {
        return exception_registers->psp;
    }
    return exception_registers->msp;
}

static CrashCatcherObject initialise(const CrashCatcherExceptionRegisters *exception_registers)
{
    CrashCatcherObject object {};
    object.exception_registers = exception_registers;
    object.info.sp = exception_stack_address(exception_registers);
    object.stacked_registers = static_cast<CrashCatcherStackedRegisters *>(address_to_pointer(object.info.sp));

    // Cortex-M always stacks eight integer registers on exception entry.
    object.info.sp += 8U * sizeof(uint32_t);
    // An extended exception frame also contains S0-S15, FPSCR and a reserved word.
    if ((exception_registers->exceptionLR & LR_FLOAT) == 0) {
        object.info.sp += 18U * sizeof(uint32_t);
    }
    // xPSR records the alignment word inserted before the exception frame.
    if ((object.stacked_registers->psr & PSR_STACK_ALIGN) != 0) {
        object.info.sp |= 4U;
    }

    const volatile uint32_t *const cpacr =
        reinterpret_cast<const volatile uint32_t *>(COPROCESSOR_ACCESS_CONTROL_REGISTER);
    constexpr uint32_t CP10_CP11_ENABLED = 5U << 20;
    if ((*cpacr & CP10_CP11_ENABLED) == CP10_CP11_ENABLED) {
        object.flags |= CRASH_CATCHER_FLAGS_FLOATING_POINT;
    }

    const uint16_t *const instruction =
        static_cast<const uint16_t *>(address_to_pointer(object.stacked_registers->pc));
    object.info.isBKPT = ((*instruction & 0xFF00U) == 0xBE00U);
    return object;
}

static void dump_signature()
{
    static const uint8_t signature[] = {
        CRASH_CATCHER_SIGNATURE_BYTE0,
        CRASH_CATCHER_SIGNATURE_BYTE1,
        CRASH_CATCHER_VERSION_MAJOR,
        CRASH_CATCHER_VERSION_MINOR
    };
    CrashCatcher_DumpMemory(signature, CRASH_CATCHER_BYTE, sizeof(signature));
}

static void dump_core_registers(const CrashCatcherObject &object)
{
    CrashCatcher_DumpMemory(&object.stacked_registers->r0, CRASH_CATCHER_BYTE, 4U * sizeof(uint32_t));
    CrashCatcher_DumpMemory(&object.exception_registers->r4, CRASH_CATCHER_BYTE, 8U * sizeof(uint32_t));
    CrashCatcher_DumpMemory(&object.stacked_registers->r12, CRASH_CATCHER_BYTE, sizeof(uint32_t));
    CrashCatcher_DumpMemory(&object.info.sp, CRASH_CATCHER_BYTE, sizeof(uint32_t));
    CrashCatcher_DumpMemory(&object.stacked_registers->lr, CRASH_CATCHER_BYTE, 3U * sizeof(uint32_t));
    CrashCatcher_DumpMemory(&object.exception_registers->msp, CRASH_CATCHER_BYTE, 3U * sizeof(uint32_t));
}

static void dump_floating_point_registers(const CrashCatcherObject &object)
{
    uint32_t registers[33];
    if ((object.exception_registers->exceptionLR & LR_FLOAT) == 0) {
        // Accessing S16-S31 first completes any pending lazy exception stacking.
        CrashCatcher_CopyUpperFloatingPointRegisters(&registers[16]);
        memcpy(&registers[0], &object.stacked_registers->floats, sizeof(object.stacked_registers->floats));
        registers[32] = object.stacked_registers->fpscr;
    } else {
        CrashCatcher_CopyAllFloatingPointRegisters(registers);
    }
    CrashCatcher_DumpMemory(registers, CRASH_CATCHER_BYTE, sizeof(registers));
}

static void dump_memory_regions(const CrashCatcherMemoryRegion *region)
{
    while (region != nullptr && region->startAddress != UINT32_MAX) {
        // The dump format stores only the start and end addresses, not elementSize.
        CrashCatcher_DumpMemory(region, CRASH_CATCHER_BYTE, 2U * sizeof(uint32_t));
        CrashCatcher_DumpMemory(address_to_pointer(region->startAddress), region->elementSize,
                                (region->endAddress - region->startAddress) / region->elementSize);
        region++;
    }
}

static void dump_fault_status_registers()
{
    const CrashCatcherMemoryRegion regions[] = {
        {
            static_cast<uint32_t>(FAULT_STATUS_REGISTERS),
            static_cast<uint32_t>(FAULT_STATUS_REGISTERS + 5U * sizeof(uint32_t)),
            CRASH_CATCHER_WORD
        },
        {UINT32_MAX, UINT32_MAX, CRASH_CATCHER_BYTE}
    };
    dump_memory_regions(regions);
}

void CrashCatcher_Entry(const CrashCatcherExceptionRegisters *exception_registers)
{
    CrashCatcherObject object = initialise(exception_registers);

    do {
        g_crashCatcherStack[0] = CRASH_CATCHER_STACK_SENTINEL;
        CrashCatcher_DumpStart(&object.info);
        dump_signature();
        CrashCatcher_DumpMemory(&object.flags, CRASH_CATCHER_BYTE, sizeof(object.flags));
        dump_core_registers(object);
        if ((object.flags & CRASH_CATCHER_FLAGS_FLOATING_POINT) != 0) {
            dump_floating_point_registers(object);
        }
        dump_memory_regions(CrashCatcher_GetMemoryRegions());
        dump_fault_status_registers();
        if (g_crashCatcherStack[0] != CRASH_CATCHER_STACK_SENTINEL) {
            static const uint8_t overflow_marker[] = {0xAC, 0xCE, 0x55, 0xED};
            CrashCatcher_DumpMemory(overflow_marker, CRASH_CATCHER_BYTE, sizeof(overflow_marker));
        }
    } while (CrashCatcher_DumpEnd() == CRASH_CATCHER_TRY_AGAIN);

    if (object.info.isBKPT) {
        object.stacked_registers->pc += 2U;
    }
}

#endif // AP_CRASHDUMP_ENABLED
