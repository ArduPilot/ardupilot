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

static bool initialised = false;
static bool do_flash_crash_dump = true;
static bool do_serial_crash_dump = false;
static void* dump_start_address;
static void* dump_end_address;

#ifndef HAL_CRASH_SERIAL_PORT_BAUD
#define HAL_CRASH_SERIAL_PORT_BAUD 921600
#endif

#if !defined(USART_ISR_RXNE)
#define USART_ISR_RXNE                      USART_ISR_RXNE_RXFNE
#endif

CRASH_CATCHER_TEST_WRITEABLE CrashCatcherReturnCodes g_crashCatcherDumpEndReturn = CRASH_CATCHER_TRY_AGAIN;
static                       CrashCatcherInfo        g_info;

static void CrashCatcher_DumpStartFlash(const CrashCatcherInfo* pInfo);
static CrashCatcherReturnCodes CrashCatcher_DumpEndFlash(void);
static void CrashCatcher_DumpMemoryFlash(const void* pvMemory, CrashCatcherElementSizes elementSize, size_t elementCount);


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

uint32_t stm32_crash_dump_size(void)
{
    uint32_t* page_addr = (uint32_t*)stm32_flash_getpageaddr(HAL_CRASH_DUMP_FLASHPAGE);
    uint32_t page_size = stm32_flash_getpagesize(HAL_CRASH_DUMP_FLASHPAGE);
    return page_addr[(page_size / sizeof(uint32_t)) - 1];
}

extern uint32_t __ram0_start__, __ram0_end__;
const CrashCatcherMemoryRegion* CrashCatcher_GetMemoryRegions(void);
const CrashCatcherMemoryRegion* CrashCatcher_GetMemoryRegions(void)
{
    // do a full dump if on serial
    static CrashCatcherMemoryRegion regions[] = {
        {(uint32_t)&__ram0_start__, (uint32_t)&__ram0_end__, CRASH_CATCHER_BYTE},
        {0xFFFFFFFF, 0xFFFFFFFF, CRASH_CATCHER_BYTE}
    };
    if (do_flash_crash_dump) {
        // smaller dump if on flash
        regions[0].startAddress = (uint32_t)dump_start_address;
        regions[0].endAddress = (uint32_t)dump_end_address;
    }
    return regions;
}

void CrashCatcher_DumpMemory(const void* pvMemory, CrashCatcherElementSizes elementSize, size_t elementCount)
{
    if (do_serial_crash_dump) {
        CrashCatcher_DumpMemoryHex(pvMemory, elementSize, elementCount);
    } else if (do_flash_crash_dump) {
        CrashCatcher_DumpMemoryFlash(pvMemory, elementSize, elementCount);
    }
}


void CrashCatcher_DumpStart(const CrashCatcherInfo* pInfo)
{
    if (do_serial_crash_dump) {
        CrashCatcher_DumpStartHex(pInfo);
    } else if (do_flash_crash_dump) {
        CrashCatcher_DumpStartFlash(pInfo);
    }
}

CrashCatcherReturnCodes CrashCatcher_DumpEnd(void)
{
    if (do_serial_crash_dump) {
        return CrashCatcher_DumpEndHex();
    } else if (do_flash_crash_dump) {
        return CrashCatcher_DumpEndFlash();
    }
    do_flash_crash_dump = false;
    do_serial_crash_dump = true;
    return CRASH_CATCHER_TRY_AGAIN;
}

// -------------- FlashDump Code --------------------
static uint32_t dump_size;
static uint8_t dump_buffer[32]; // we need to maintain a dump buffer of 32bytes for H7
static uint8_t buf_off;

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
    if (!stm32_flash_ispageerased(HAL_CRASH_DUMP_FLASHPAGE)) {
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
        uint32_t page_start = stm32_flash_getpageaddr(HAL_CRASH_DUMP_FLASHPAGE);
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
        if (dump_size + buf_off + sizeof(dump_size) >= stm32_flash_getpagesize(HAL_CRASH_DUMP_FLASHPAGE)) {
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
    if (dump_size + buf_off + sizeof(dump_size) >= stm32_flash_getpagesize(HAL_CRASH_DUMP_FLASHPAGE)) {
        // when this happens, buf_off will be sizeof(buffer)-sizeof(dump_size)
        // 0xFF will be used to detect that we were in the middle of taking a dump
        memset(&dump_buffer[sizeof(dump_buffer)-sizeof(dump_size)], 0xFF, sizeof(dump_size));
        buf_off = sizeof(dump_buffer);
    }
    if (buf_off > 0) {
        if (dump_size + sizeof(dump_buffer) >= stm32_flash_getpagesize(HAL_CRASH_DUMP_FLASHPAGE) && buf_off < sizeof(dump_buffer)) {
            // we have a partially full buffer towards the end, so we need to write the buffer
            // and then write the size of the buffer
            memcpy(&dump_buffer[sizeof(dump_buffer)-sizeof(dump_size)], &dump_size, sizeof(dump_size));
            buf_off = 32;
        }
        stm32_flash_write(stm32_flash_getpageaddr(HAL_CRASH_DUMP_FLASHPAGE) + dump_size, dump_buffer, buf_off);
        dump_size += buf_off;
        buf_off = 0;
        memset(dump_buffer, 0, sizeof(dump_buffer));
        stm32_watchdog_pat();
    }

    // write size to buffer if not already written
    if (dump_size < stm32_flash_getpagesize(HAL_CRASH_DUMP_FLASHPAGE)) {
        memcpy(&dump_buffer[sizeof(dump_buffer)-sizeof(dump_size)], &dump_size, sizeof(dump_size));
        stm32_flash_write(stm32_flash_getpageaddr(HAL_CRASH_DUMP_FLASHPAGE) + stm32_flash_getpagesize(HAL_CRASH_DUMP_FLASHPAGE) - sizeof(dump_buffer), dump_buffer, sizeof(dump_buffer));
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
    uint32_t fck = (uint32_t)(((HAL_CRASH_SERIAL_PORT_CLOCK + ((HAL_CRASH_SERIAL_PORT_BAUD)/2)) / HAL_CRASH_SERIAL_PORT_BAUD));

    u->BRR = fck;

    /* Resetting eventual pending status flags.*/
    u->ICR = 0xFFFFFFFFU;

    u->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

    initialised = true;
}

int CrashCatcher_getc(void);
int CrashCatcher_getc(void)
{
    if (!initialised) {
        init_uarts();
    }
    USART_TypeDef *u = HAL_CRASH_SERIAL_PORT;
    // wait for a follwing string, only then do we start dumping
    static const char* wait_for_string = "dump_crash_log";
    uint8_t curr_off = 0;
    while (true) {
        while (!(USART_ISR_RXNE & u->ISR)) {}
        uint8_t c = u->RDR;
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
    if (!initialised) {
        init_uarts();
    }
    USART_TypeDef *u = HAL_CRASH_SERIAL_PORT;
    u->TDR = c & 0xFF;
    while (!(USART_ISR_TC & u->ISR)) {
        // keep alive while dump is happening
        stm32_watchdog_pat();
    }
}
