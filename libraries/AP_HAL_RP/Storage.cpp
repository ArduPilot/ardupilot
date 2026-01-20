#include "Storage.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

using namespace RP;

uint8_t Storage::_buffer[HAL_STORAGE_SIZE];

Storage::Storage() :
    _flash_storage(
        _buffer,
        4096, // flash_sector_size (W25Q16 sector)
        FUNCTOR_BIND_MEMBER(&Storage::_flash_write, bool, uint8_t, uint32_t, const uint8_t *, uint16_t),
        FUNCTOR_BIND_MEMBER(&Storage::_flash_read, bool, uint8_t, uint32_t, uint8_t *, uint16_t),
        FUNCTOR_BIND_MEMBER(&Storage::_flash_erase, bool, uint8_t),
        FUNCTOR_BIND_MEMBER(&Storage::_flash_erase_ok, bool)
    )
{}

void Storage::init() {
    if (_initialised) return;

    // AP_FlashStorage::init() loads data from Flash into _buffer
    if (_flash_storage.init()) {
        _initialised = true;
    }
}

void Storage::read_block(void *dst, uint16_t src, size_t n) {
    if (src + n <= sizeof(_buffer)) {
        memcpy(dst, &_buffer[src], n);
    }
}

void Storage::write_block(uint16_t dst, const void* src, size_t n) {
    if (dst + n <= sizeof(_buffer)) {
        if (memcmp(&_buffer[dst], src, n) != 0) {
            memcpy(&_buffer[dst], src, n);
            if (!_flash_storage.write(dst, n)) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Storage write failed at %u", dst);
            }
        }
    }
}

bool Storage::erase() {
    return _flash_storage.erase();
}

/////////////////////////////////////////////
// Private proxy methods for AP_FlashStorage
/////////////////////////////////////////////

bool Storage::_flash_write(uint8_t sector, uint32_t offset, const uint8_t *data, uint16_t length) {
    // sector is an index from 0 to HAL_FLASH_STORAGE_NUM_PAGES
    uint32_t addr = hal.flash->getpageaddr(sector) + offset;
    return hal.flash->write(addr, data, length);
}

bool Storage::_flash_read(uint8_t sector, uint32_t offset, uint8_t *data, uint16_t length) {
    uint32_t addr = hal.flash->getpageaddr(sector) + offset;
    // Read directly from memory (XIP)
    memcpy(data, (void*)(XIP_BASE + addr), length);
    return true;
}

bool Storage::_flash_erase(uint8_t sector) {
    return hal.flash->erasepage(sector);
}

bool Storage::_flash_erase_ok() {
    // In ArduPilot, you can add an "if not in flight" check here
    return true;
}
