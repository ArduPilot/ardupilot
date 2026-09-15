/*
 * ROM-API flash primitives for mr_vmu_rt1176 parameter storage.
 *
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
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Geometry of the parameter-storage area. */
#define RT1176_FLASH_STORAGE_OFFSET  0x620000U    // storage_partition, flash-relative
#define RT1176_FLASH_SECTOR_SIZE     65536U       // MX25UM51345G erase block
#define RT1176_FLASH_PAGE_SIZE       256U         // NOR program page, 1 per ROM call
#define RT1176_FLASH_ERASE_CHUNK     4096U        // erase step; bounds irq-off time
#define RT1176_FLASH_MEMMAP_BASE     0x30000000U  // FlexSPI1 XIP window

/* Initialise the ROM API and its NOR driver. Safe to call repeatedly. */
int rt1176_flash_init(void);

/* Erase a byte range. Offsets are flash-relative. Returns 0 on success. */
int rt1176_flash_erase(uint32_t offset, uint32_t size);

/* Program a byte range, one NOR page per ROM call. Flash-relative offset.
 * The target range must already be erased. Returns 0 on success. */
int rt1176_flash_program(uint32_t offset, const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif
