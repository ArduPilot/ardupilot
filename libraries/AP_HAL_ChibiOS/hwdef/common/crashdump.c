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

#include <CrashCatcher.h>
#include <ch.h>
#include "hal.h"
#include "string.h"
#include "watchdog.h"
#include "stm32_util.h"
#include "flash.h"

CRASH_CATCHER_TEST_WRITEABLE CrashCatcherReturnCodes g_crashCatcherDumpEndReturn = CRASH_CATCHER_TRY_AGAIN;
static                       CrashCatcherInfo        g_info;

static bool do_flash_crash_dump = true;
static void* dump_start_address;
static void* dump_end_address;

static void CrashCatcher_DumpStartFlash(const CrashCatcherInfo* pInfo);
static CrashCatcherReturnCodes CrashCatcher_DumpEndFlash(void);
static void CrashCatcher_DumpMemoryFlash(const void* pvMemory, CrashCatcherElementSizes elementSize, size_t elementCount);

#if defined(HAL_CRASH_SERIAL_PORT)
static bool uart_initialised = false;
static bool do_serial_crash_dump = false;

#ifndef HAL_CRASH_SERIAL_PORT_BAUD
#define HAL_CRASH_SERIAL_PORT_BAUD 921600
#endif // HAL_CRASH_SERIAL_PORT_BAUD

#if !defined(USART_ISR_RXNE)
#define USART_ISR_RXNE                      USART_ISR_RXNE_RXFNE
#endif
static void printString(const char* pString);
static void waitForUserInput(void);
static void dumpBytes(const uint8_t* pMemory, size_t elementCount);
static void dumpByteAsHex(uint8_t byte);
static void dumpHexDigit(uint8_t nibble);
static void dumpHalfwords(const uint16_t* pMemory, size_t elementCount);
static void dumpWords(const uint32_t* pMemory, size_t elementCount);
static void CrashCatcher_DumpStartHex(const CrashCatcherInfo* pInfo);
static CrashCatcherReturnCodes CrashCatcher_DumpEndHex(void);
static void CrashCatcher_DumpMemoryHex(const void* pvMemory, CrashCatcherElementSizes elementSize, size_t elementCount);
#endif // HAL_CRASH_SERIAL_PORT

extern uint32_t __crash_log_base__, __crash_log_end__;

uint32_t stm32_crash_dump_size(void)
{
    uint32_t* page_addr = (uint32_t*)&__crash_log_base__;
    uint32_t page_size = stm32_crash_dump_max_size();
    return page_addr[(page_size / sizeof(uint32_t)) - 1];
}

uint32_t stm32_crash_dump_max_size(void)
{
    return (uint32_t)&__crash_log_end__ - (uint32_t)&__crash_log_base__;
}

uint32_t stm32_crash_dump_addr(void)
{
    return (uint32_t)&__crash_log_base__;
}

bool stm32_crash_dump_region_erased(void)
{
    for (uint32_t i = 0; i < stm32_crash_dump_max_size(); i += 4) {
        if (((uint32_t*)stm32_crash_dump_addr())[i / 4] != 0xFFFFFFFF) {
            return false;
        }
    }
    return true;
}

#define ARRAY_SIZE(X) (sizeof(X)/sizeof(X[0]))
extern uint32_t __ram0_start__, __ram0_end__, __heap_base__, __heap_end__, __bss_base__, __bss_end__;
#define REMAINDER_MEM_REGION_SIZE (15000) // remainder memory for crashcatcher internal regions
static uint32_t dump_size = 0;
static uint8_t dump_buffer[32]; // we need to maintain a dump buffer of 32bytes for H7
static uint8_t buf_off = 0;

const CrashCatcherMemoryRegion* CrashCatcher_GetMemoryRegions(void);
const CrashCatcherMemoryRegion* CrashCatcher_GetMemoryRegions(void)
{
    // do a full dump if on serial
    static CrashCatcherMemoryRegion regions[60] = {
    {(uint32_t)&__ram0_start__, (uint32_t)&__ram0_end__, CRASH_CATCHER_BYTE},
    {(uint32_t)&ch_system, (uint32_t)&ch_system + sizeof(ch_system), CRASH_CATCHER_BYTE}};
    uint32_t total_dump_size = dump_size + buf_off + REMAINDER_MEM_REGION_SIZE;
    // loop through chibios threads and add their stack info
    uint8_t curr_region = 2;
    for (thread_t *tp = chRegFirstThread(); tp && (curr_region < (ARRAY_SIZE(regions) - 1)); tp = chRegNextThread(tp)) {
        uint32_t total_stack;
        if (tp->wabase == (void*)&__main_thread_stack_base__) {
            // main thread has its stack separated from the thread context
            total_stack = (uint32_t)((const uint8_t *)&__main_thread_stack_end__ - (const uint8_t *)&__main_thread_stack_base__);
        } else {
            // all other threads have their thread context pointer
            // above the stack top
            total_stack = (uint32_t)(tp) - (uint32_t)(tp->wabase);
        }
        // log names if in RAM
        if (tp->name != NULL && is_address_in_memory((void*)tp->name)) {
            regions[curr_region].elementSize = CRASH_CATCHER_BYTE;
            regions[curr_region].startAddress = (uint32_t)(tp->name);
            regions[curr_region++].endAddress = (uint32_t)(tp->name) + 13;
        }
        // log thread info
        regions[curr_region].elementSize = CRASH_CATCHER_BYTE;
        regions[curr_region].startAddress = (uint32_t)(tp);
        regions[curr_region++].endAddress = (uint32_t)(tp) + sizeof(thread_t);
        // log thread stacks
        regions[curr_region].elementSize = CRASH_CATCHER_BYTE;
        regions[curr_region].startAddress = (uint32_t)(tp->wabase);
        regions[curr_region++].endAddress = (uint32_t)(tp->wabase) + total_stack;

        total_dump_size += total_stack;
        if ((total_dump_size) >= stm32_crash_dump_max_size()) {
            // we can't log anymore than this
            goto finalise;
        }
    }

    // log statically alocated memory
    int32_t bss_size = ((uint32_t)&__bss_end__) - ((uint32_t)&__bss_base__);
    int32_t available_space = stm32_crash_dump_max_size() - total_dump_size;
    if (available_space < 0) {
        // we can't log anymore than this
        goto finalise;
    }
    if (bss_size > available_space) { // dump however much we can
        regions[curr_region].elementSize = CRASH_CATCHER_BYTE;
        regions[curr_region].startAddress = (uint32_t)&__bss_base__;
        regions[curr_region++].endAddress = (uint32_t)&__bss_base__ + available_space;
        total_dump_size += available_space;
    } else { // dump the entire bss
        regions[curr_region].elementSize = CRASH_CATCHER_BYTE;
        regions[curr_region].startAddress = (uint32_t)&__bss_base__;
        regions[curr_region++].endAddress = (uint32_t)&__bss_end__;
        total_dump_size += bss_size;
    }

    // dump the Heap as well as much as we can
    int32_t heap_size = ((uint32_t)&__heap_end__) - ((uint32_t)&__heap_base__);
    available_space = stm32_crash_dump_max_size() - total_dump_size;
    if (available_space < 0) {
        // we can't log anymore than this
        goto finalise;
    }
    if (heap_size > available_space) { // dump however much we can
        regions[curr_region].elementSize = CRASH_CATCHER_BYTE;
        regions[curr_region].startAddress = (uint32_t)&__heap_base__;
        regions[curr_region++].endAddress = (uint32_t)&__heap_base__ + available_space;
        total_dump_size += available_space;
    } else { // dump the entire heap
        regions[curr_region].elementSize = CRASH_CATCHER_BYTE;
        regions[curr_region].startAddress = (uint32_t)&__heap_base__;
        regions[curr_region++].endAddress = (uint32_t)&__heap_end__;
        total_dump_size += heap_size;
    }

finalise:
    // ensure that last is filled with 0xFFFFFFFF
    if (curr_region < ARRAY_SIZE(regions)) {
        regions[curr_region].elementSize = CRASH_CATCHER_BYTE;
        regions[curr_region].startAddress = 0xFFFFFFFF;
        regions[curr_region].endAddress = 0xFFFFFFFF;
    } else {
        regions[ARRAY_SIZE(regions) - 1].elementSize = CRASH_CATCHER_BYTE;
        regions[ARRAY_SIZE(regions) - 1].startAddress = 0xFFFFFFFF;
        regions[ARRAY_SIZE(regions) - 1].endAddress = 0xFFFFFFFF;
    }

    if (do_flash_crash_dump) {
        // smaller dump if on flash
        regions[0].startAddress = (uint32_t)dump_start_address;
        regions[0].endAddress = (uint32_t)dump_end_address;
    }
    return regions;
}

void CrashCatcher_DumpMemory(const void* pvMemory, CrashCatcherElementSizes elementSize, size_t elementCount)
{
    (void)pvMemory;
    (void)elementSize;
    (void)elementCount;
#if defined(HAL_CRASH_SERIAL_PORT)
    if (do_serial_crash_dump) {
        CrashCatcher_DumpMemoryHex(pvMemory, elementSize, elementCount);
    }
#endif
    if (do_flash_crash_dump) {
        CrashCatcher_DumpMemoryFlash(pvMemory, elementSize, elementCount);
    }
}


void CrashCatcher_DumpStart(const CrashCatcherInfo* pInfo)
{
    // Record the fault info for watchdog
    struct port_extctx* ctx = (struct port_extctx*)pInfo->sp;
    FaultType faultType = (FaultType)__get_IPSR();
    save_fault_watchdog(__LINE__, faultType, pInfo->sp, ctx->lr_thd);
#if defined(HAL_CRASH_SERIAL_PORT)
    if (do_serial_crash_dump) {
        CrashCatcher_DumpStartHex(pInfo);
    }
#endif
    if (do_flash_crash_dump) {
        CrashCatcher_DumpStartFlash(pInfo);
    }
}

CrashCatcherReturnCodes CrashCatcher_DumpEnd(void)
{
#if defined(HAL_CRASH_SERIAL_PORT)
    if (do_serial_crash_dump) {
        return CrashCatcher_DumpEndHex();
    }
#endif
    if (do_flash_crash_dump) {
        return CrashCatcher_DumpEndFlash();
    }
    do_flash_crash_dump = false;
#if defined(HAL_CRASH_SERIAL_PORT)
    do_serial_crash_dump = true;
#endif
    return CRASH_CATCHER_TRY_AGAIN;
}

// -------------- FlashDump Code --------------------
static void CrashCatcher_DumpStartFlash(const CrashCatcherInfo* pInfo)
{
    // initialise for dumping
    void *sp = (void*)pInfo->sp;
    if (sp == NULL || !is_address_in_memory(sp)) {
        do_flash_crash_dump = false;
        return;
    }
    // let's set the memory range to be dumped
    if (get_addr_mem_region_start_addr(sp) + HAL_MAX_STACK_FRAME_SIZE > sp) {
        // only go until the start of the region
        dump_start_address = get_addr_mem_region_start_addr(sp);
    } else {
        // go back as far as we need to
        dump_start_address = sp - HAL_MAX_STACK_FRAME_SIZE;
    }

    if (get_addr_mem_region_end_addr(sp) < sp + HAL_PROCESS_STACK_SIZE) {
        // only go until the end of the region
        dump_end_address = get_addr_mem_region_end_addr(sp);
    } else {
        // go ahead as far as we need to
        dump_end_address = sp + HAL_PROCESS_STACK_SIZE;
    }

    dump_size = 0;
    buf_off = 0;
    // we expect crash dump flash page to already be empty
    if (!stm32_crash_dump_region_erased()) {
        // stuff is already there, maybe last dump
        // so just do nothing
        do_flash_crash_dump = false;
        return;
    }
    stm32_watchdog_pat();
    // unlock flash page for write
    stm32_flash_keep_unlocked(true);
}
// only flushes if we have a full buffer
static void flush_dump_buffer(void)
{
    if (buf_off == sizeof(dump_buffer)) {
        uint32_t page_start = (uint32_t)stm32_crash_dump_addr();
        // write dump buffer to flash
        stm32_flash_write(page_start + dump_size, dump_buffer, sizeof(dump_buffer));
        dump_size += sizeof(dump_buffer);
        buf_off = 0;
        memset(dump_buffer, 0, sizeof(dump_buffer));
        stm32_watchdog_pat();
    }
}

// Does the requested Dump to Flash
static void CrashCatcher_DumpMemoryFlash(const void* pvMemory, CrashCatcherElementSizes elementSize, size_t elementCount)
{
    const uint8_t* pv = (const uint8_t*)pvMemory;
    size_t cnt = 0;
    while (cnt < elementCount) {
        if (dump_size + buf_off + sizeof(dump_size) >= stm32_crash_dump_max_size()) {
            // when this happens, buf_off will be sizeof(buffer)-sizeof(dump_size)
            // 0xFF will be used to detect that we were in the middle of taking a dump
            memset(&dump_buffer[sizeof(dump_buffer)-sizeof(dump_size)], 0xFF, sizeof(dump_size));
            buf_off = sizeof(dump_buffer);
            return;
        }
        flush_dump_buffer();
        switch (elementSize)
        {
            case CRASH_CATCHER_BYTE:
                dump_buffer[buf_off++] = pv[cnt++];
                break;
            case CRASH_CATCHER_HALFWORD:
                // we need to split the half word into two parts
                dump_buffer[buf_off++] = (uint16_t)*pv & 0xFF;
                flush_dump_buffer();
                dump_buffer[buf_off++] = ((uint16_t)*pv >> 8) & 0xFF;
                cnt++;
                break;
            case CRASH_CATCHER_WORD:
                // we need to split the word and then write
                for (size_t i = 0; i < sizeof(uint32_t); i++) {
                    dump_buffer[buf_off++] = ((uint32_t)*pv >> (8*i)) & 0xFF;
                    flush_dump_buffer();
                }
                cnt++;
                break;
        }
    }
}

static CrashCatcherReturnCodes CrashCatcher_DumpEndFlash(void)
{
    // flush the buffer
    if (dump_size + buf_off + sizeof(dump_size) >= stm32_crash_dump_max_size()) {
        // when this happens, buf_off will be sizeof(buffer)-sizeof(dump_size)
        // 0xFF will be used to detect that we were in the middle of taking a dump
        memset(&dump_buffer[sizeof(dump_buffer)-sizeof(dump_size)], 0xFF, sizeof(dump_size));
        buf_off = sizeof(dump_buffer);
    }
    if (buf_off > 0) {
        if (dump_size + sizeof(dump_buffer) >= stm32_crash_dump_max_size() && buf_off < sizeof(dump_buffer)) {
            // we have a partially full buffer towards the end, so we need to write the buffer
            // and then write the size of the buffer
            memcpy(&dump_buffer[sizeof(dump_buffer)-sizeof(dump_size)], &dump_size, sizeof(dump_size));
            buf_off = 32;
        }
        stm32_flash_write(stm32_crash_dump_addr() + dump_size, dump_buffer, 32);
        dump_size += buf_off;
        buf_off = 0;
        memset(dump_buffer, 0, sizeof(dump_buffer));
        stm32_watchdog_pat();
    }

    // write size to buffer if not already written
    if (dump_size < stm32_crash_dump_max_size()) {
        memcpy(&dump_buffer[sizeof(dump_buffer)-sizeof(dump_size)], &dump_size, sizeof(dump_size));
        stm32_flash_write(stm32_crash_dump_addr() + stm32_crash_dump_max_size() - sizeof(dump_buffer), dump_buffer, sizeof(dump_buffer));
        stm32_watchdog_pat();
    }

    stm32_flash_keep_unlocked(false);
    // How big of a dump did we take, record that at the end of flash sector
    if (g_crashCatcherDumpEndReturn == CRASH_CATCHER_TRY_AGAIN && g_info.isBKPT)
        return CRASH_CATCHER_EXIT;
    else
        return g_crashCatcherDumpEndReturn;
}

// -------------- HexDump Code --------------------
/* Copyright (C) 2018  Adam Green (https://github.com/adamgreen)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
#if defined(HAL_CRASH_SERIAL_PORT)

static void CrashCatcher_DumpStartHex(const CrashCatcherInfo* pInfo)
{
    g_info = *pInfo;
    
    printString("\r\n\r\n");
    if (pInfo->isBKPT)
        printString("BREAKPOINT");
    else
        printString("CRASH");
    printString(" ENCOUNTERED\r\n"
                 "Enable logging and then press any key to start dump.\r\n");
    
    waitForUserInput();
    printString("\r\n");
}

static void printString(const char* pString)
{
    while (*pString)
        CrashCatcher_putc(*pString++);
}

static void waitForUserInput(void)
{
    CrashCatcher_getc();
}

static void CrashCatcher_DumpMemoryHex(const void* pvMemory, CrashCatcherElementSizes elementSize, size_t elementCount)
{
    switch (elementSize)
    {
        case CRASH_CATCHER_BYTE:
            dumpBytes(pvMemory, elementCount);
            break;
        case CRASH_CATCHER_HALFWORD:
            dumpHalfwords(pvMemory, elementCount);
            break;
        case CRASH_CATCHER_WORD:
            dumpWords(pvMemory, elementCount);
            break;
    }
    printString("\r\n");
}

static void dumpBytes(const uint8_t* pMemory, size_t elementCount)
{
    size_t i;
    for (i = 0 ; i < elementCount ; i++)
    {
        /* Only dump 16 bytes to a single line before introducing a line break. */
        if (i != 0 && (i & 0xF) == 0)
            printString("\r\n");
        dumpByteAsHex(*pMemory++);
    }
}

static void dumpByteAsHex(uint8_t byte)
{
    dumpHexDigit(byte >> 4);
    dumpHexDigit(byte & 0xF);
}

static void dumpHexDigit(uint8_t nibble)
{
    static const char hexToASCII[] = "0123456789ABCDEF";

    // assert( nibble < 16 );
    CrashCatcher_putc(hexToASCII[nibble]);
}

static void dumpHalfwords(const uint16_t* pMemory, size_t elementCount)
{
    size_t i;
    for (i = 0 ; i < elementCount ; i++)
    {
        uint16_t val = *pMemory++;
        /* Only dump 8 halfwords to a single line before introducing a line break. */
        if (i != 0 && (i & 0x7) == 0)
            printString("\r\n");
        dumpBytes((uint8_t*)&val, sizeof(val));
    }
}

static void dumpWords(const uint32_t* pMemory, size_t elementCount)
{
    size_t i;
    for (i = 0 ; i < elementCount ; i++)
    {
        uint32_t val = *pMemory++;
        /* Only dump 4 words to a single line before introducing a line break. */
        if (i != 0 && (i & 0x3) == 0)
            printString("\r\n");
        dumpBytes((uint8_t*)&val, sizeof(val));
    }
}


static CrashCatcherReturnCodes CrashCatcher_DumpEndHex(void)
{
    printString("\r\nEnd of dump\r\n");
    if (g_crashCatcherDumpEndReturn == CRASH_CATCHER_TRY_AGAIN && g_info.isBKPT)
        return CRASH_CATCHER_EXIT;
    else
        return g_crashCatcherDumpEndReturn;
}

/*
  initialise serial ports
 */
static void init_uarts(void)
{
    USART_TypeDef *u = HAL_CRASH_SERIAL_PORT;
    IRQ_DISABLE_HAL_CRASH_SERIAL_PORT();
    RCC_RESET_HAL_CRASH_SERIAL_PORT();

    /* Baud rate setting.*/
    uint32_t fck;
#if defined(STM32F7) || defined(STM32H7) || defined(STM32F3) || defined(STM32G4) || defined(STM32L4) || defined(STM32L4PLUS)
    fck = (uint32_t)(((HAL_CRASH_SERIAL_PORT_CLOCK + ((HAL_CRASH_SERIAL_PORT_BAUD)/2)) / HAL_CRASH_SERIAL_PORT_BAUD));
#else
#if STM32_HAS_USART6
    if ((u == USART1) || (u == USART6))
#else
    if (u == USART1)
#endif
        fck = (STM32_PCLK2+((HAL_CRASH_SERIAL_PORT_BAUD)/2)) / HAL_CRASH_SERIAL_PORT_BAUD;
    else
        fck = (STM32_PCLK1+((HAL_CRASH_SERIAL_PORT_BAUD)/2)) / HAL_CRASH_SERIAL_PORT_BAUD;
#endif //defined(STM32F7) || defined(STM32H7) || defined(STM32F3) || defined(STM32G4) || defined(STM32L4) || defined(STM32L4PLUS)

    u->BRR = fck;

#if defined(STM32F7) || defined(STM32H7) || defined(STM32F3) || defined(STM32G4) || defined(STM32L4) || defined(STM32L4PLUS)
    /* Resetting eventual pending status flags.*/
    u->ICR = 0xFFFFFFFFU;
#else
  u->SR = 0;
  (void)u->SR;  /* SR reset step 1.*/
  (void)u->DR;  /* SR reset step 2.*/
#endif

    u->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

    uart_initialised = true;
}

int CrashCatcher_getc(void);
int CrashCatcher_getc(void)
{
    if (!uart_initialised) {
        init_uarts();
    }
    USART_TypeDef *u = HAL_CRASH_SERIAL_PORT;
    // wait for a follwing string, only then do we start dumping
    static const char* wait_for_string = "dump_crash_log";
    uint8_t curr_off = 0;
    while (true) {
#if defined(STM32F7) || defined(STM32H7) || defined(STM32F3) || defined(STM32G4) || defined(STM32L4) || defined(STM32L4PLUS)
        while (!(USART_ISR_RXNE & u->ISR)) {}
        uint8_t c = u->RDR;
#else
        while (!(USART_SR_RXNE & u->SR)) {}
        uint8_t c = u->DR;
#endif
        if (c == wait_for_string[curr_off]) {
            curr_off++;
            if (curr_off == strlen(wait_for_string)) {
                return 0;
            }
        } else {
            curr_off = 0;
        }
    }
    return -1;
}

void CrashCatcher_putc(int c);
void CrashCatcher_putc(int c)
{
    if (!uart_initialised) {
        init_uarts();
    }
    USART_TypeDef *u = HAL_CRASH_SERIAL_PORT;
#if defined(STM32F7) || defined(STM32H7) || defined(STM32F3) || defined(STM32G4) || defined(STM32L4) || defined(STM32L4PLUS)
    u->TDR = c & 0xFF;
#else
    u->DR = c & 0xFF;
#endif
#if defined(STM32F7) || defined(STM32H7) || defined(STM32F3) || defined(STM32G4) || defined(STM32L4) || defined(STM32L4PLUS)
    while (!(USART_ISR_TC & u->ISR)) {
#else
    while (!(USART_SR_TC & u->SR)) {
#endif
        // keep alive while dump is happening
        stm32_watchdog_pat();
    }
}
#endif // #if defined(HAL_CRASH_SERIAL_PORT)
