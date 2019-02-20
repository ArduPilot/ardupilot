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
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include "HAL_ESP32_Namespace.h"
#include <AP_Common/Bitmask.h>
#include <AP_FlashStorage/AP_FlashStorage.h>

#include "esp_partition.h"

#define STORAGE_SIZE HAL_STORAGE_SIZE
#define STORAGE_SECTOR_SIZE (128*1024)

#define STORAGE_LINE_SHIFT 3

#define STORAGE_LINE_SIZE (1<<STORAGE_LINE_SHIFT)
#define STORAGE_NUM_LINES (STORAGE_SIZE/STORAGE_LINE_SIZE)

class ESP32::Storage : public AP_HAL::Storage {
public:
    void init() override {}
    void read_block(void *dst, uint16_t src, size_t n) override;
    void write_block(uint16_t dst, const void* src, size_t n) override;

    void _timer_tick(void) override;
    bool healthy(void) override;

private:
    volatile bool _initialised;
    const esp_partition_t *p;
    void _storage_open(void);
    void _mark_dirty(uint16_t loc, uint16_t length);
    uint8_t _buffer[STORAGE_SIZE] __attribute__((aligned(4)));
    Bitmask _dirty_mask{STORAGE_NUM_LINES};

    bool _flash_write_data(uint8_t sector, uint32_t offset, const uint8_t *data, uint16_t length);
    bool _flash_read_data(uint8_t sector, uint32_t offset, uint8_t *data, uint16_t length);
    bool _flash_erase_sector(uint8_t sector);
    bool _flash_erase_ok(void);
    bool _flash_failed;
    uint32_t _last_re_init_ms;
    uint32_t _last_empty_ms;

    AP_FlashStorage _flash{_buffer,
                           STORAGE_SECTOR_SIZE,
                           FUNCTOR_BIND_MEMBER(&Storage::_flash_write_data, bool, uint8_t, uint32_t, const uint8_t *, uint16_t),
                           FUNCTOR_BIND_MEMBER(&Storage::_flash_read_data, bool, uint8_t, uint32_t, uint8_t *, uint16_t),
                           FUNCTOR_BIND_MEMBER(&Storage::_flash_erase_sector, bool, uint8_t),
                           FUNCTOR_BIND_MEMBER(&Storage::_flash_erase_ok, bool)};

    void _flash_load(void);
    void _flash_write(uint16_t line);
};
