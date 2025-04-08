#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/Bitmask.h>
#include "AP_HAL_SITL_Namespace.h"
#include <AP_FlashStorage/AP_FlashStorage.h>
#include <AP_RAMTRON/AP_RAMTRON.h>

#ifndef STORAGE_USE_FLASH
#define STORAGE_USE_FLASH 1
#endif

#ifndef STORAGE_USE_POSIX
#define STORAGE_USE_POSIX 1
#endif

#ifndef STORAGE_USE_FRAM
#define STORAGE_USE_FRAM HAL_WITH_RAMTRON
#endif

#define STORAGE_LINE_SHIFT 3

#define STORAGE_LINE_SIZE (1<<STORAGE_LINE_SHIFT)
#define STORAGE_NUM_LINES (HAL_STORAGE_SIZE/STORAGE_LINE_SIZE)

class HALSITL::Storage : public AP_HAL::Storage {
public:
    void init() override {}
    void read_block(void *dst, uint16_t src, size_t n) override;
    void write_block(uint16_t dst, const void* src, size_t n) override;
    bool get_storage_ptr(void *&ptr, size_t &size) override;

    void _timer_tick(void) override;
    bool healthy(void) override;

private:
    enum class StorageBackend: uint8_t {
        None,
        FRAM,
        Flash,
        SDCard,  // AKA POSIX
    };
    StorageBackend _initialisedType = StorageBackend::None;

    void _storage_create(void);
    void _storage_open(void);
    void _save_backup(void);
    void _mark_dirty(uint16_t loc, uint16_t length);
    uint8_t _buffer[HAL_STORAGE_SIZE] __attribute__((aligned(4)));
    Bitmask<STORAGE_NUM_LINES> _dirty_mask;

    uint32_t _last_empty_ms;

#if STORAGE_USE_FLASH
    bool _flash_write_data(uint8_t sector, uint32_t offset, const uint8_t *data, uint16_t length);
    bool _flash_read_data(uint8_t sector, uint32_t offset, uint8_t *data, uint16_t length);
    bool _flash_erase_sector(uint8_t sector);
    bool _flash_erase_ok(void);

    bool _flash_failed;
    uint32_t _last_re_init_ms;

    AP_FlashStorage _flash{_buffer,
            HAL_FLASH_SECTOR_SIZE,
            FUNCTOR_BIND_MEMBER(&Storage::_flash_write_data, bool, uint8_t, uint32_t, const uint8_t *, uint16_t),
            FUNCTOR_BIND_MEMBER(&Storage::_flash_read_data, bool, uint8_t, uint32_t, uint8_t *, uint16_t),
            FUNCTOR_BIND_MEMBER(&Storage::_flash_erase_sector, bool, uint8_t),
            FUNCTOR_BIND_MEMBER(&Storage::_flash_erase_ok, bool)};

    void _flash_load(void);
    void _flash_write(uint16_t line);
#endif

#if STORAGE_USE_POSIX
    int log_fd;
#endif

#if STORAGE_USE_FRAM
    AP_RAMTRON fram;
#endif
};
