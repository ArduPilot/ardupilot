/************************************************************************************
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

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
 * Modified for use in AP_HAL by Andrew Tridgell and Siddharth Bharat Purohit
 */

#include "flash.h"
#include "hal.h"
#include <string.h>
#include "stm32_util.h"
#include "hrt.h"

#include <assert.h>

// #pragma GCC optimize("O0")

/*
  this driver has been tested with STM32F427 and STM32F412
 */

#ifndef HAL_NO_FLASH_SUPPORT

#ifndef BOARD_FLASH_SIZE
#error "You must define BOARD_FLASH_SIZE in kbyte"
#endif

#define KB(x)   ((x*1024))
// Refer Flash memory map in the User Manual to fill the following fields per microcontroller
#define STM32_FLASH_BASE    0x08000000
#define STM32_FLASH_SIZE    KB(BOARD_FLASH_SIZE)

// optionally disable interrupts during flash writes
#ifndef STM32_FLASH_DISABLE_ISR
#define STM32_FLASH_DISABLE_ISR 0
#endif

// the 2nd bank of flash needs to be handled differently
#define STM32_FLASH_BANK2_START (STM32_FLASH_BASE+0x00080000)

#if defined(STM32F4)
#if BOARD_FLASH_SIZE == 512
#define STM32_FLASH_NPAGES  8
static const uint32_t flash_memmap[STM32_FLASH_NPAGES] = { KB(16), KB(16), KB(16), KB(16), KB(64),
                                                           KB(128), KB(128), KB(128) };

#elif BOARD_FLASH_SIZE == 1024
#define STM32_FLASH_NPAGES  12
static const uint32_t flash_memmap[STM32_FLASH_NPAGES] = { KB(16), KB(16), KB(16), KB(16), KB(64),
                                                           KB(128), KB(128), KB(128), KB(128), KB(128), KB(128), KB(128) };

#elif BOARD_FLASH_SIZE == 2048
#define STM32_FLASH_NPAGES  24
static const uint32_t flash_memmap[STM32_FLASH_NPAGES] = { KB(16), KB(16), KB(16), KB(16), KB(64),
                                                           KB(128), KB(128), KB(128), KB(128), KB(128), KB(128), KB(128),
                                                           KB(16), KB(16), KB(16), KB(16), KB(64),
                                                           KB(128), KB(128), KB(128), KB(128), KB(128), KB(128), KB(128)};
#else
#error "BOARD_FLASH_SIZE invalid"
#endif

#elif defined(STM32F7)
#if BOARD_FLASH_SIZE == 512
#define STM32_FLASH_NPAGES  8
static const uint32_t flash_memmap[STM32_FLASH_NPAGES] = { KB(16), KB(16), KB(16), KB(16), KB(64), KB(128), KB(128), KB(128) };

#elif BOARD_FLASH_SIZE == 1024
#define STM32_FLASH_NPAGES  8
static const uint32_t flash_memmap[STM32_FLASH_NPAGES] = { KB(32), KB(32), KB(32), KB(32), KB(128), KB(256), KB(256), KB(256) };

#elif BOARD_FLASH_SIZE == 2048
#define STM32_FLASH_NPAGES  12
static const uint32_t flash_memmap[STM32_FLASH_NPAGES] = { KB(32), KB(32), KB(32), KB(32), KB(128), KB(256), KB(256), KB(256),
                                                           KB(256), KB(256), KB(256), KB(256) };
#else
#error "BOARD_FLASH_SIZE invalid"
#endif
#elif defined(STM32H7)
#define STM32_FLASH_NPAGES  (BOARD_FLASH_SIZE / 128)
#define STM32_FLASH_FIXED_PAGE_SIZE 128
#elif defined(STM32F100_MCUCONF) || defined(STM32F103_MCUCONF)
#define STM32_FLASH_NPAGES BOARD_FLASH_SIZE
#define STM32_FLASH_FIXED_PAGE_SIZE 1
#elif defined(STM32F105_MCUCONF)
#define STM32_FLASH_NPAGES (BOARD_FLASH_SIZE/2)
#define STM32_FLASH_FIXED_PAGE_SIZE 2
#elif defined(STM32F303_MCUCONF)
#define STM32_FLASH_NPAGES (BOARD_FLASH_SIZE/2)
#define STM32_FLASH_FIXED_PAGE_SIZE 2
#elif defined(STM32G4)
#define STM32_FLASH_NPAGES (BOARD_FLASH_SIZE/2)
#define STM32_FLASH_FIXED_PAGE_SIZE 2
#else
#error "Unsupported processor for flash.c"
#endif

#if defined(__GNUC__) && __GNUC__ >= 6
#ifdef STORAGE_FLASH_PAGE
static_assert(STORAGE_FLASH_PAGE < STM32_FLASH_NPAGES,
              "STORAGE_FLASH_PAGE out of range");
#endif
#endif

// keep a cache of the page addresses
#ifndef STM32_FLASH_FIXED_PAGE_SIZE
static uint32_t flash_pageaddr[STM32_FLASH_NPAGES];
static bool flash_pageaddr_initialised;
#endif

static bool flash_keep_unlocked;

#ifndef FLASH_KEY1
#define FLASH_KEY1      0x45670123
#endif
#ifndef FLASH_KEY2
#define FLASH_KEY2      0xCDEF89AB
#endif

/* Some compiler options will convert short loads and stores into byte loads
 * and stores.  We don't want this to happen for IO reads and writes!
 */
/* # define getreg16(a)       (*(volatile uint16_t *)(a)) */
static inline uint16_t getreg16(unsigned int addr)
{
    uint16_t retval;
    __asm__ __volatile__("\tldrh %0, [%1]\n\t" : "=r"(retval) : "r"(addr));
    return retval;
}

/* define putreg16(v,a)       (*(volatile uint16_t *)(a) = (v)) */
static inline void putreg16(uint16_t val, unsigned int addr)
{
    __asm__ __volatile__("\tstrh %0, [%1]\n\t": : "r"(val), "r"(addr));
}

/* # define getreg32(a)       (*(volatile uint32_t *)(a)) */
static inline uint32_t getreg32(unsigned int addr)
{
    uint32_t retval;
    __asm__ __volatile__("\tldr %0, [%1]\n\t" : "=r"(retval) : "r"(addr));
    return retval;
}

/* define putreg32(v,a)        */
static inline void putreg32(uint32_t val, unsigned int addr)
{
    *(volatile uint32_t *)(addr) = val;
}

static void stm32_flash_wait_idle(void)
{
    __DSB();
#if defined(STM32H7)
    while ((FLASH->SR1 & (FLASH_SR_BSY|FLASH_SR_QW|FLASH_SR_WBNE)) ||
           (FLASH->SR2 & (FLASH_SR_BSY|FLASH_SR_QW|FLASH_SR_WBNE))) {
        // nop
    }
#else
	while (FLASH->SR & FLASH_SR_BSY) {
        // nop
    }
#endif
}

static void stm32_flash_clear_errors(void)
{
#if defined(STM32H7)
    FLASH->CCR1 = ~0;
    FLASH->CCR2 = ~0;
#else
    FLASH->SR = 0xF3;
#endif
}

static void stm32_flash_unlock(void)
{
    if (flash_keep_unlocked) {
        return;
    }
    stm32_flash_wait_idle();

#if defined(STM32H7)
    if (FLASH->CR1 & FLASH_CR_LOCK) {
        /* Unlock sequence */
        FLASH->KEYR1 = FLASH_KEY1;
        FLASH->KEYR1 = FLASH_KEY2;
    }
    if (FLASH->CR2 & FLASH_CR_LOCK) {
        /* Unlock sequence */
        FLASH->KEYR2 = FLASH_KEY1;
        FLASH->KEYR2 = FLASH_KEY2;
    }
#else
    if (FLASH->CR & FLASH_CR_LOCK) {
        /* Unlock sequence */
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
    }
#endif

#ifdef FLASH_ACR_DCEN
    // disable the data cache - see stm32 errata 2.1.11
    FLASH->ACR &= ~FLASH_ACR_DCEN;
#endif
}

void stm32_flash_lock(void)
{
    if (flash_keep_unlocked) {
        return;
    }
#if defined(STM32H7)
    if (FLASH->SR1 & FLASH_SR_QW) {
        FLASH->CR1 |= FLASH_CR_FW;
    }
    if (FLASH->SR2 & FLASH_SR_QW) {
        FLASH->CR2 |= FLASH_CR_FW;
    }
    stm32_flash_wait_idle();
    FLASH->CR1 |= FLASH_CR_LOCK;
    FLASH->CR2 |= FLASH_CR_LOCK;
#else
    stm32_flash_wait_idle();
    FLASH->CR |= FLASH_CR_LOCK;
#endif

#ifdef FLASH_ACR_DCEN
    // reset and re-enable the data cache - see stm32 errata 2.1.11
    FLASH->ACR |= FLASH_ACR_DCRST;
    FLASH->ACR &= ~FLASH_ACR_DCRST;
    FLASH->ACR |= FLASH_ACR_DCEN;
#endif
}



/*
  get the memory address of a page
 */
uint32_t stm32_flash_getpageaddr(uint32_t page)
{
    if (page >= STM32_FLASH_NPAGES) {
        return 0;
    }
#if defined(STM32_FLASH_FIXED_PAGE_SIZE)
    return STM32_FLASH_BASE + page * STM32_FLASH_FIXED_PAGE_SIZE * 1024;
#else
    if (!flash_pageaddr_initialised) {
        uint32_t address = STM32_FLASH_BASE;
        uint8_t i;

        for (i = 0; i < STM32_FLASH_NPAGES; i++) {
            flash_pageaddr[i] = address;
            address += stm32_flash_getpagesize(i);
        }
        flash_pageaddr_initialised = true;
    }

    return flash_pageaddr[page];
#endif
}

/*
  get size in bytes of a page
 */
uint32_t stm32_flash_getpagesize(uint32_t page)
{
#if defined(STM32_FLASH_FIXED_PAGE_SIZE)
    (void)page;
    return STM32_FLASH_FIXED_PAGE_SIZE * 1024;
#else
    return flash_memmap[page];
#endif
}

/*
  return total number of pages
 */
uint32_t stm32_flash_getnumpages()
{
    return STM32_FLASH_NPAGES;
}

bool stm32_flash_ispageerased(uint32_t page)
{
    uint32_t addr;
    uint32_t count;

    if (page >= STM32_FLASH_NPAGES) {
        return false;
    }

    for (addr = stm32_flash_getpageaddr(page), count = stm32_flash_getpagesize(page);
        count; count -= 4, addr += 4) {
        uint32_t v = getreg32(addr);
        if (v != 0xffffffff) {
            return false;
        }
    }

    return true;
}

#ifndef HAL_BOOTLOADER_BUILD
static uint32_t last_erase_ms;
#endif

/*
  erase a page
 */
bool stm32_flash_erasepage(uint32_t page)
{
    if (page >= STM32_FLASH_NPAGES) {
        return false;
    }

#ifndef HAL_BOOTLOADER_BUILD
    last_erase_ms = hrt_millis32();
#endif

#if STM32_FLASH_DISABLE_ISR
    syssts_t sts = chSysGetStatusAndLockX();
#endif
    stm32_flash_wait_idle();
    stm32_flash_unlock();

    // clear any previous errors
    stm32_flash_clear_errors();

#if defined(STM32H7)
    if (page < 8) {
        // first bank
        FLASH->SR1 = ~0;

        stm32_flash_wait_idle();

        uint32_t snb = page << 8;

        // use 32 bit operations
        FLASH->CR1 = FLASH_CR_PSIZE_1 | snb | FLASH_CR_SER;
        FLASH->CR1 |= FLASH_CR_START;
        while (FLASH->SR1 & FLASH_SR_QW) ;
    } else {
        // second bank
        FLASH->SR2 = ~0;

        stm32_flash_wait_idle();

        uint32_t snb = (page-8) << 8;

        // use 32 bit operations
        FLASH->CR2 = FLASH_CR_PSIZE_1 | snb | FLASH_CR_SER;
        FLASH->CR2 |= FLASH_CR_START;
        while (FLASH->SR2 & FLASH_SR_QW) ;
    }
#elif defined(STM32F1) || defined(STM32F3)
    FLASH->CR = FLASH_CR_PER;
    FLASH->AR = stm32_flash_getpageaddr(page);
    FLASH->CR |= FLASH_CR_STRT;
#elif defined(STM32F4) || defined(STM32F7)
    // the snb mask is not contiguous, calculate the mask for the page
    uint8_t snb = (((page % 12) << 3) | ((page / 12) << 7));

    // use 32 bit operations
    FLASH->CR = FLASH_CR_PSIZE_1 | snb | FLASH_CR_SER;
    FLASH->CR |= FLASH_CR_STRT;
#elif defined(STM32G4)
    FLASH->CR = FLASH_CR_PER;
    // rather oddly, PNB is a 7 bit field that the ref manual says can
    // contain 8 bits we assume that for 512k single bank devices
    // there is an 8th bit
    FLASH->CR |= page<<FLASH_CR_PNB_Pos;
    FLASH->CR |= FLASH_CR_STRT;
#else
#error "Unsupported MCU"
#endif

    stm32_flash_wait_idle();

    stm32_cacheBufferInvalidate((void*)stm32_flash_getpageaddr(page), stm32_flash_getpagesize(page));

    stm32_flash_lock();
#if STM32_FLASH_DISABLE_ISR
    chSysRestoreStatusX(sts);
#endif

#ifndef HAL_BOOTLOADER_BUILD
    last_erase_ms = hrt_millis32();
#endif

    return stm32_flash_ispageerased(page);
}


#if defined(STM32H7)
// Check that the flash line is erased as writing to an un-erased line causes flash corruption
static bool stm32h7_check_all_ones(uint32_t addr, uint32_t words)
{
    for (uint32_t i = 0; i < words; i++) {
        // check that the byte was erased
        if (getreg32(addr) != 0xFFFFFFFF) {
            return false;
        }
        addr += 4;
    }
    return true;
}

/*
  the H7 needs special handling, and only writes 32 bytes at a time
 */
static bool stm32h7_flash_write32(uint32_t addr, const void *buf)
{
    volatile uint32_t *CR, *CCR, *SR;
    if (addr - STM32_FLASH_BASE < 8 * STM32_FLASH_FIXED_PAGE_SIZE * 1024) {
        CR = &FLASH->CR1;
        CCR = &FLASH->CCR1;
        SR = &FLASH->SR1;
    } else {
        CR = &FLASH->CR2;
        CCR = &FLASH->CCR2;
        SR = &FLASH->SR2;
    }
    stm32_flash_wait_idle();

    *CCR = ~0;
    *CR |= FLASH_CR_PG;

    const uint32_t *v = (const uint32_t *)buf;
    for (uint8_t i=0; i<8; i++) {
        while (*SR & (FLASH_SR_BSY|FLASH_SR_QW)) ;
        putreg32(*v, addr);
        v++;
        addr += 4;
    }
    __DSB();
    stm32_flash_wait_idle();
    *CCR = ~0;
    *CR &= ~FLASH_CR_PG;
    return true;
}

static bool stm32_flash_write_h7(uint32_t addr, const void *buf, uint32_t count)
{
    uint8_t *b = (uint8_t *)buf;

    if ((count & 0x1F) || (addr & 0x1F)) {
        // only allow 256 bit aligned writes
        return false;
    }

    // check for erasure
    if (!stm32h7_check_all_ones(addr, count >> 2)) {
        return false;
    }

#if STM32_FLASH_DISABLE_ISR
    syssts_t sts = chSysGetStatusAndLockX();
#endif

    stm32_flash_unlock();
    bool success = true;

    while (count >= 32) {
        if (!stm32h7_flash_write32(addr, b)) {
            success = false;
            goto failed;
        }

        // check contents
        if (memcmp((void*)addr, b, 32) != 0) {
            success = false;
            goto failed;
        }

        addr += 32;
        count -= 32;
        b += 32;
    }

failed:
    stm32_flash_lock();
#if STM32_FLASH_DISABLE_ISR
    chSysRestoreStatusX(sts);
#endif
    return success;
}

#endif // STM32H7

#if defined(STM32F4) || defined(STM32F7)
static bool stm32_flash_write_f4f7(uint32_t addr, const void *buf, uint32_t count)
{
    uint8_t *b = (uint8_t *)buf;

    /* STM32 requires half-word access */
    if (count & 1) {
        return false;
    }

    if ((addr+count) >= STM32_FLASH_BASE+STM32_FLASH_SIZE) {
        return false;
    }

    /* Get flash ready and begin flashing */

    if (!(RCC->CR & RCC_CR_HSION)) {
        return false;
    }

#if STM32_FLASH_DISABLE_ISR
    syssts_t sts = chSysGetStatusAndLockX();
#endif
    
    stm32_flash_unlock();

    // clear previous errors
    stm32_flash_clear_errors();

    stm32_flash_wait_idle();

    // do as much as possible with 32 bit writes
    while (count >= 4 && (addr & 3) == 0) {
        FLASH->CR &= ~(FLASH_CR_PSIZE);
        FLASH->CR |= FLASH_CR_PSIZE_1 | FLASH_CR_PG;
        const uint32_t v1 = *(uint32_t *)b;

        putreg32(v1, addr);

        // ensure write ordering with cache
        __DSB();
        
        stm32_flash_wait_idle();

        const uint32_t v2 = getreg32(addr);
        if (v2 != v1) {
            FLASH->CR &= ~(FLASH_CR_PG);
            goto failed;
        }

        count -= 4;
        b += 4;
        addr += 4;
    }

    // the rest as 16 bit
    while (count >= 2) {
        FLASH->CR &= ~(FLASH_CR_PSIZE);
        FLASH->CR |= FLASH_CR_PSIZE_0 | FLASH_CR_PG;

        putreg16(*(uint16_t *)b, addr);

        // ensure write ordering with cache
        __DSB();
        
        stm32_flash_wait_idle();

        if (getreg16(addr) != *(uint16_t *)b) {
            FLASH->CR &= ~(FLASH_CR_PG);
            goto failed;
        }

        count -= 2;
        b += 2;
        addr += 2;
    }

    FLASH->CR &= ~(FLASH_CR_PG);

    stm32_flash_lock();
#if STM32_FLASH_DISABLE_ISR
    chSysRestoreStatusX(sts);
#endif
    return true;

failed:
    stm32_flash_lock();
#if STM32_FLASH_DISABLE_ISR
    chSysRestoreStatusX(sts);
#endif
    return false;
}

#endif // STM32F4 || STM32F7

uint32_t _flash_fail_line;
uint32_t _flash_fail_addr;
uint32_t _flash_fail_count;
uint8_t *_flash_fail_buf;

#if defined(STM32F1) || defined(STM32F3)
static bool stm32_flash_write_f1(uint32_t addr, const void *buf, uint32_t count)
{
    uint8_t *b = (uint8_t *)buf;

    /* STM32 requires half-word access */
    if (count & 1) {
        _flash_fail_line = __LINE__;
        return false;
    }

    if ((addr+count) >= STM32_FLASH_BASE+STM32_FLASH_SIZE) {
        _flash_fail_line = __LINE__;
        return false;
    }

#if STM32_FLASH_DISABLE_ISR
    syssts_t sts = chSysGetStatusAndLockX();
#endif
    
    stm32_flash_unlock();

    stm32_flash_wait_idle();

    // program in 16 bit steps
    while (count >= 2) {
        FLASH->CR = FLASH_CR_PG;

        putreg16(*(uint16_t *)b, addr);

        stm32_flash_wait_idle();

        FLASH->CR = 0;

        if (getreg16(addr) != *(uint16_t *)b) {
            _flash_fail_line = __LINE__;
            _flash_fail_addr = addr;
            _flash_fail_count = count;
            _flash_fail_buf = b;
            goto failed;
        }

        count -= 2;
        b += 2;
        addr += 2;
    }

    stm32_flash_lock();
#if STM32_FLASH_DISABLE_ISR
    chSysRestoreStatusX(sts);
#endif
    return true;

failed:
    stm32_flash_lock();
#if STM32_FLASH_DISABLE_ISR
    chSysRestoreStatusX(sts);
#endif
    return false;
}
#endif // STM32F1 or STM32F3

#if defined(STM32G4)
static bool stm32_flash_write_g4(uint32_t addr, const void *buf, uint32_t count)
{
    uint32_t *b = (uint32_t *)buf;

    /* STM32G4 requires double-word access */
    if ((count & 7) || (addr & 7)) {
        _flash_fail_line = __LINE__;
        return false;
    }

    if ((addr+count) >= STM32_FLASH_BASE+STM32_FLASH_SIZE) {
        _flash_fail_line = __LINE__;
        return false;
    }

    // skip already programmed word pairs
    while (count >= 8 &&
           getreg32(addr+0) == b[0] &&
           getreg32(addr+4) == b[1]) {
        count -= 8;
        addr += 8;
        b += 2;
    }
    if (count == 0) {
        return true;
    }


#if STM32_FLASH_DISABLE_ISR
    syssts_t sts = chSysGetStatusAndLockX();
#endif
    
    stm32_flash_unlock();

    stm32_flash_wait_idle();

    // program in 16 bit steps
    while (count >= 2) {
        FLASH->CR = FLASH_CR_PG;

        putreg32(b[0], addr+0);
        putreg32(b[1], addr+4);

        stm32_flash_wait_idle();

        FLASH->CR = 0;

        if (getreg32(addr+0) != b[0] ||
            getreg32(addr+4) != b[1]) {
            _flash_fail_line = __LINE__;
            _flash_fail_addr = addr;
            _flash_fail_count = count;
            _flash_fail_buf = (uint8_t *)b;
            goto failed;
        }

        count -= 8;
        b += 2;
        addr += 8;
    }

    stm32_flash_lock();
#if STM32_FLASH_DISABLE_ISR
    chSysRestoreStatusX(sts);
#endif
    return true;

failed:
    stm32_flash_lock();
#if STM32_FLASH_DISABLE_ISR
    chSysRestoreStatusX(sts);
#endif
    return false;
}
#endif // STM32G4

bool stm32_flash_write(uint32_t addr, const void *buf, uint32_t count)
{
#if defined(STM32F1) || defined(STM32F3)
    return stm32_flash_write_f1(addr, buf, count);
#elif defined(STM32F4) || defined(STM32F7)
    return stm32_flash_write_f4f7(addr, buf, count);
#elif defined(STM32H7)
    return stm32_flash_write_h7(addr, buf, count);
#elif defined(STM32G4)
    return stm32_flash_write_g4(addr, buf, count);
#else
#error "Unsupported MCU"
#endif
}

void stm32_flash_keep_unlocked(bool set)
{
    if (set && !flash_keep_unlocked) {
        stm32_flash_unlock();
        flash_keep_unlocked = true;
    } else if (!set && flash_keep_unlocked) {
        flash_keep_unlocked = false;
        stm32_flash_lock();        
    }
}

#ifndef HAL_BOOTLOADER_BUILD
/*
  return true if we had a recent erase
 */
bool stm32_flash_recent_erase(void)
{
    return hrt_millis32() - last_erase_ms < 3000U;
}
#endif

#endif // HAL_NO_FLASH_SUPPORT

