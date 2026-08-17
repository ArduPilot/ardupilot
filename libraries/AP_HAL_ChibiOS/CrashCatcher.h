/*
 * Copyright (C) 2017-2019 Adam Green (https://github.com/adamgreen)
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

#pragma once

#define CRASH_CATCHER_STACK_WORD_COUNT 192

#ifdef __ARM_ARCH_7EM__
#define CRASH_CATCHER_WITH_FPU 1
#else
#define CRASH_CATCHER_WITH_FPU 0
#endif

#if !defined(__ASSEMBLER__) || (!__ASSEMBLER__)

#include <stddef.h>
#include <stdint.h>

#define CRASH_CATCHER_SIGNATURE_BYTE0 'c'
#define CRASH_CATCHER_SIGNATURE_BYTE1 'C'
#define CRASH_CATCHER_VERSION_MAJOR 3
#define CRASH_CATCHER_VERSION_MINOR 0

#define CRASH_CATCHER_FLAGS_FLOATING_POINT (1U << 0)
#define CRASH_CATCHER_STACK_SENTINEL 0xACCE55EDU

typedef struct {
    uint32_t sp;
    int isBKPT;
} CrashCatcherInfo;

typedef struct {
    uint32_t startAddress;
    uint32_t endAddress;
} CrashCatcherMemoryRegionInfo;

typedef enum {
    CRASH_CATCHER_BYTE = 1,
    CRASH_CATCHER_HALFWORD = 2,
    CRASH_CATCHER_WORD = 4
} CrashCatcherElementSizes;

typedef enum {
    CRASH_CATCHER_TRY_AGAIN = 0,
    CRASH_CATCHER_EXIT
} CrashCatcherReturnCodes;

typedef struct {
    uint32_t startAddress;
    uint32_t endAddress;
    CrashCatcherElementSizes elementSize;
} CrashCatcherMemoryRegion;

typedef struct {
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t pc;
    uint32_t psr;
    uint32_t floats[16];
    uint32_t fpscr;
    uint32_t reserved;
} CrashCatcherStackedRegisters;

typedef struct {
    uint32_t msp;
    uint32_t psp;
    uint32_t exceptionPSR;
    uint32_t r4;
    uint32_t r5;
    uint32_t r6;
    uint32_t r7;
    uint32_t r8;
    uint32_t r9;
    uint32_t r10;
    uint32_t r11;
    uint32_t exceptionLR;
} CrashCatcherExceptionRegisters;

#ifdef RUNNING_HOST_TESTS
#define CRASH_CATCHER_TEST_WRITEABLE
#else
#define CRASH_CATCHER_TEST_WRITEABLE static const
#endif

#ifdef __cplusplus
extern "C" {
#endif

void CrashCatcher_DumpStart(const CrashCatcherInfo *info);
const CrashCatcherMemoryRegion *CrashCatcher_GetMemoryRegions(void);
void CrashCatcher_DumpMemory(const void *memory, CrashCatcherElementSizes element_size, size_t element_count);
CrashCatcherReturnCodes CrashCatcher_DumpEnd(void);

int CrashCatcher_getc(void);
void CrashCatcher_putc(int character);

extern uint32_t g_crashCatcherStack[CRASH_CATCHER_STACK_WORD_COUNT];
void CrashCatcher_Entry(const CrashCatcherExceptionRegisters *exception_registers);
void CrashCatcher_CopyAllFloatingPointRegisters(uint32_t *buffer);
void CrashCatcher_CopyUpperFloatingPointRegisters(uint32_t *buffer);

#ifdef __cplusplus
}
#endif

#endif // !__ASSEMBLER__
