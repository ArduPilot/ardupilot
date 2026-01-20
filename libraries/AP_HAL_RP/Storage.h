#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_FlashStorage/AP_FlashStorage.h>
#include "AP_HAL_RP_Namespace.h"

class RP::Storage : public AP_HAL::Storage {
public:
    Storage();

    void init() override;
    bool erase() override;
    void read_block(void *dst, uint16_t src, size_t n) override;
    void write_block(uint16_t dst, const void* src, size_t n) override;
    void _timer_tick(void) override {}

private:
    // Object that implements wear leveling logic on top of Flash
    AP_FlashStorage _flash_storage;
    bool _initialised = false;

    // RAM storage for caching parameters (size from AP_HAL/board/rp2350.h)
    static uint8_t _buffer[HAL_STORAGE_SIZE];

    // Callbacks for AP_FlashStorage
    bool _flash_write(uint8_t sector, uint32_t offset, const uint8_t *data, uint16_t length);
    bool _flash_read(uint8_t sector, uint32_t offset, uint8_t *data, uint16_t length);
    bool _flash_erase(uint8_t sector);
    bool _flash_erase_ok();
};
