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
#pragma once

#include <array>

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/Bitmask.h>
#include "AP_HAL_Zephyr_Namespace.h"

#ifdef __ZEPHYR__
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#endif

#ifdef CONFIG_ZMS
#include <zephyr/kvss/zms.h>
#endif

#ifdef CONFIG_AP_RT1176_ROMAPI_FLASH
#include <AP_FlashStorage/AP_FlashStorage.h>
#include "zephyr/src/rt1176_romapi_flash.h"
#endif

#ifndef CONFIG_AP_HAL_STORAGE_SECTOR_SIZE
#define CONFIG_AP_HAL_STORAGE_SECTOR_SIZE 4096  // ZMS sector, bytes
#endif
#ifndef CONFIG_AP_HAL_STORAGE_SECTOR_COUNT
#define CONFIG_AP_HAL_STORAGE_SECTOR_COUNT 16
#endif

namespace Zephyr {

class Storage : public AP_HAL::Storage {
public:
    Storage();

    void init() override;
    bool erase() override;
    void read_block(void *dst, uint16_t src, size_t n) override;
    void write_block(uint16_t dst, const void *src, size_t n) override;

    /* Deferred write-behind, ChibiOS parity: storage writes must not block the
     * caller, so they are staged and flushed by the storage thread. */
    void _timer_tick(void) override;

    bool healthy() override;
    bool is_flash_backed() const {
#ifdef CONFIG_AP_RT1176_ROMAPI_FLASH
        if (_flash_ok) {
            return true;
        }
#endif
        return _zms_ok || _fram_ok || _file_ok;
    }
    bool get_storage_ptr(void *&ptr, size_t &size) override {
        ptr  = _storage.data();
        size = _storage.size();
        return true;
    }

private:
    static constexpr uint16_t CHUNK_SIZE  = 256;
    static constexpr uint16_t NUM_CHUNKS  = HAL_STORAGE_SIZE / CHUNK_SIZE;

    static constexpr uint32_t ZMS_SECTOR_SIZE  = CONFIG_AP_HAL_STORAGE_SECTOR_SIZE;
    static constexpr uint32_t ZMS_SECTOR_COUNT = CONFIG_AP_HAL_STORAGE_SECTOR_COUNT;

    std::array<uint8_t, HAL_STORAGE_SIZE> _storage;

    bool _healthy;
    bool _zms_ok;
    bool _fram_ok;
    bool _file_ok;

    // last time _timer_tick() found the dirty queue fully drained; healthy()
    // fails when this goes >2s stale (ChibiOS's _last_empty_ms pattern)
    uint32_t _last_empty_ms;

    // One bit per CHUNK_SIZE-byte chunk of _storage awaiting a backend
    // write. Set by write_block(), cleared by _timer_tick() once that
    // chunk's write actually lands.
    Bitmask<NUM_CHUNKS> _dirty_mask;

#ifdef CONFIG_AP_RT1176_ROMAPI_FLASH
    /* Flash backend via the BootROM API, with AP_FlashStorage above it as on ChibiOS. */
    bool _flash_ok;
    bool _flash_write_failed;   // latch, so a failure is reported once not every write
    uint32_t _last_re_init_ms;  // rate-limits re_initialise() to once per 5s (ChibiOS parity)
    bool _flash_write_data(uint8_t sector, uint32_t offset, const uint8_t *data, uint16_t length);
    bool _flash_read_data(uint8_t sector, uint32_t offset, uint8_t *data, uint16_t length);
    bool _flash_erase_sector(uint8_t sector);
    bool _flash_erase_ok(void);

    AP_FlashStorage _flash{_storage.data(),
                           RT1176_FLASH_SECTOR_SIZE,
                           FUNCTOR_BIND_MEMBER(&Storage::_flash_write_data, bool, uint8_t, uint32_t,
                                               const uint8_t *, uint16_t),
                           FUNCTOR_BIND_MEMBER(&Storage::_flash_read_data, bool, uint8_t, uint32_t,
                                               uint8_t *, uint16_t),
                           FUNCTOR_BIND_MEMBER(&Storage::_flash_erase_sector, bool, uint8_t),
                           FUNCTOR_BIND_MEMBER(&Storage::_flash_erase_ok, bool)};
#endif

#ifdef CONFIG_ZMS
    struct zms_fs _zms;
    void _write_chunk_zms(uint16_t chunk_idx);
#endif

#ifdef __ZEPHYR__
    const struct spi_dt_spec *_fram = nullptr;

    static constexpr uint8_t CMD_WREN  = 0x06;
    static constexpr uint8_t CMD_WRITE = 0x02;
    static constexpr uint8_t CMD_READ  = 0x03;

    bool _fram_wren();
    bool _fram_read(uint32_t addr, uint8_t *dst, size_t n);
    bool _fram_write(uint32_t addr, const uint8_t *src, size_t n);
#endif

    void _try_file_mount();
    void _write_file();
    void _write_file_chunk(uint16_t chunk_idx);
};

}  // namespace Zephyr
