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
 */

/* Crash dump backend interfaces. */

#pragma once

#ifndef AP_CRASHDUMP_FATFS_ENABLED
#define AP_CRASHDUMP_FATFS_ENABLED 0
#endif

#ifndef AP_CRASHDUMP_FLASH_ENABLED
#define AP_CRASHDUMP_FLASH_ENABLED 0
#endif

#if AP_CRASHDUMP_FATFS_ENABLED && AP_CRASHDUMP_FLASH_ENABLED
#error "SD and flash crash dump backends cannot be enabled together"
#endif

#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || \
    defined(STM32L4) || defined(STM32L4PLUS)
#define CRASHDUMP_SD_SPI_SUPPORTED_MCU 1
#else
#define CRASHDUMP_SD_SPI_SUPPORTED_MCU 0
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#if AP_CRASHDUMP_FATFS_ENABLED || AP_CRASHDUMP_FLASH_ENABLED
#include <CrashCatcher.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if AP_CRASHDUMP_FATFS_ENABLED
/*
  initialise crash dump to SD card support. Called after SD card is
  mounted. Returns true if SD crash dump is available.
 */
bool crashdump_sd_init(void);

/*
  invalidate SD crash dump state before stopping or remounting the card
 */
void crashdump_sd_invalidate(void);

/*
  return true if SD crash dump is ready to use
 */
bool crashdump_sd_ready(void);

/*
  update cached SD crash dump state from a non-main thread
 */
void crashdump_sd_update(void);

/*
  return the maximum dump size in bytes
 */
uint32_t crashdump_sd_max_size(void);

/*
  called from CrashCatcher to start a dump to SD. Returns true if
  dump can proceed.
 */
bool crashdump_sd_start(void);

/*
  write data to the SD crash dump. Called from fault handler context.
  Returns true on success.
 */
bool crashdump_sd_write(const void *data,
                        CrashCatcherElementSizes element_size,
                        size_t element_count);

/*
  finish the SD crash dump, writing any remaining buffered data and
  the dump size marker. Returns true on success.
 */
bool crashdump_sd_end(uint32_t dump_size);

/*
  get the size of an existing crash dump on SD, 0 if none
 */
uint32_t crashdump_sd_dump_size(void);
#endif

#if AP_CRASHDUMP_FLASH_ENABLED
/* Flash backend calls made by the common CrashCatcher callbacks. */
bool crashdump_flash_start(const CrashCatcherInfo *info);
const CrashCatcherMemoryRegion *crashdump_flash_memory_regions(bool active);
void crashdump_flash_write(const void *memory,
                           CrashCatcherElementSizes element_size,
                           size_t element_count);
CrashCatcherReturnCodes crashdump_flash_end(CrashCatcherReturnCodes return_code,
        bool is_breakpoint);
#endif

#ifdef __cplusplus
}
#endif
