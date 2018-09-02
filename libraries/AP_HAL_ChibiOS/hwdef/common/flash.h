

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
uint32_t stm32_flash_getpageaddr(uint32_t page);
uint32_t stm32_flash_getpagesize(uint32_t page);
uint32_t stm32_flash_getnumpages(void);
bool stm32_flash_erasepage(uint32_t page);
int32_t stm32_flash_write(uint32_t addr, const void *buf, uint32_t count);
void stm32_flash_keep_unlocked(bool set);
bool stm32_flash_ispageerased(uint32_t page);
#ifdef __cplusplus
}
#endif
