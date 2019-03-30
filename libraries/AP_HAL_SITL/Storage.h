#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/Bitmask.h>
#include "AP_HAL_SITL_Namespace.h"
#include <AP_FlashStorage/AP_FlashStorage.h>

#ifndef HAL_STORAGE_FILE
#define HAL_STORAGE_FILE "eeprom.bin"
#endif

// define which storage system to use. This allows us to test flash storage with --sitl-flash-storage
// configure option
#ifndef STORAGE_USE_FLASH
#define STORAGE_USE_FLASH 0
#endif

#define STORAGE_USE_POSIX 1

#define STORAGE_LINE_SHIFT 3

#define STORAGE_LINE_SIZE (1<<STORAGE_LINE_SHIFT)
#define STORAGE_NUM_LINES (HAL_STORAGE_SIZE/STORAGE_LINE_SIZE)

class HALSITL::Storage : public AP_HAL::Storage {
public:
    void init() override {}
    void read_block(void *dst, uint16_t src, size_t n) override;
    void write_block(uint16_t dst, const void* src, size_t n) override;

    void _timer_tick(void) override;
    bool healthy(void) override;

private:
    volatile bool _initialised;
    void _storage_create(void);
    void _storage_open(void);
    void _save_backup(void);
    void _mark_dirty(uint16_t loc, uint16_t length);
    uint8_t _buffer[HAL_STORAGE_SIZE] __attribute__((aligned(4)));
    Bitmask _dirty_mask{STORAGE_NUM_LINES};

#if STORAGE_USE_FLASH
    bool _flash_write_data(uint8_t sector, uint32_t offset, const uint8_t *data, uint16_t length);
    bool _flash_read_data(uint8_t sector, uint32_t offset, uint8_t *data, uint16_t length);
    bool _flash_erase_sector(uint8_t sector);
    bool _flash_erase_ok(void);
#endif

    bool _flash_failed;
    uint32_t _last_re_init_ms;
    uint32_t _last_empty_ms;

#if STORAGE_USE_FLASH
    AP_FlashStorage _flash{_buffer,
            HAL_STORAGE_SIZE,
            FUNCTOR_BIND_MEMBER(&Storage::_flash_write_data, bool, uint8_t, uint32_t, const uint8_t *, uint16_t),
            FUNCTOR_BIND_MEMBER(&Storage::_flash_read_data, bool, uint8_t, uint32_t, uint8_t *, uint16_t),
            FUNCTOR_BIND_MEMBER(&Storage::_flash_erase_sector, bool, uint8_t),
            FUNCTOR_BIND_MEMBER(&Storage::_flash_erase_ok, bool)};
#endif
    
    void _flash_load(void);
    void _flash_write(uint16_t line);

#if STORAGE_USE_POSIX
    bool using_filesystem;
    int log_fd;
#endif
};
