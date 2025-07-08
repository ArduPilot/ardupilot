#pragma once

#include <AP_HAL/AP_HAL.h>

#define QURT_STORAGE_SIZE HAL_STORAGE_SIZE

namespace QURT
{

class Storage : public AP_HAL::Storage
{
public:
    static Storage *from(AP_HAL::Storage *storage)
    {
        return static_cast<Storage*>(storage);
    }


    void init() override;

    uint8_t  read_byte(uint16_t loc);
    uint16_t read_word(uint16_t loc);
    uint32_t read_dword(uint16_t loc);
    void     read_block(void *dst, uint16_t src, size_t n) override;

    void write_byte(uint16_t loc, uint8_t value);
    void write_word(uint16_t loc, uint16_t value);
    void write_dword(uint16_t loc, uint32_t value);
    void write_block(uint16_t dst, const void* src, size_t n) override;

    bool get_storage_ptr(void *&ptr, size_t &size) override;

    virtual void _timer_tick(void) override;

protected:
    void _mark_dirty(uint16_t loc, uint16_t length);
    bool _storage_create(void);

    int _fd = -1;
    volatile bool _initialised;
    volatile uint32_t _dirty_mask;
    uint8_t _buffer[QURT_STORAGE_SIZE];
};

}
