

/*
 * Copyright (C) Siddharth Bharat Purohit 2017
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
 */
#include "hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(RP2350)
/*
 * Rename all stm32_flash_* symbols to rp2350_flash_* for RP2350 builds.
 * These macros must be defined BEFORE the function declarations below so the preprocessor renames both the declarations (introducing rp2350_flash_* prototypes) and every call site that includes this header.
 */
#define stm32_flash_getpageaddr       rp2350_flash_getpageaddr
#define stm32_flash_getpagesize       rp2350_flash_getpagesize
#define stm32_flash_getnumpages       rp2350_flash_getnumpages
#define stm32_flash_erasepage         rp2350_flash_erasepage
#define stm32_flash_write             rp2350_flash_write
#define stm32_flash_keep_unlocked     rp2350_flash_keep_unlocked
#define stm32_flash_ispageerased      rp2350_flash_ispageerased
#define stm32_flash_protect_flash     rp2350_flash_protect_flash
#define stm32_flash_unprotect_flash   rp2350_flash_unprotect_flash
#define stm32_flash_recent_erase      rp2350_flash_recent_erase
#endif // RP2350

uint32_t stm32_flash_getpageaddr(uint32_t page);
uint32_t stm32_flash_getpagesize(uint32_t page);
uint32_t stm32_flash_getnumpages(void);
bool stm32_flash_erasepage(uint32_t page);
bool stm32_flash_write(uint32_t addr, const void *buf, uint32_t count);
void stm32_flash_keep_unlocked(bool set);
bool stm32_flash_ispageerased(uint32_t page);
void stm32_flash_protect_flash(bool bootloader, bool protect);
void stm32_flash_unprotect_flash(void);
void stm32_flash_set_NRST_MODE(uint8_t nrst_mode);
#if defined(STM32H7)
void stm32_flash_corrupt(uint32_t addr, bool double_bit);
#endif
#ifndef HAL_BOOTLOADER_BUILD
bool stm32_flash_recent_erase(void);
#endif
#ifdef __cplusplus
}
#endif
