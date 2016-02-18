#pragma once

#define LINUX_STORAGE_USE_FRAM 0

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_Linux_Namespace.h"

#define LINUX_STORAGE_SIZE HAL_STORAGE_SIZE
#define LINUX_STORAGE_MAX_WRITE 512
#define LINUX_STORAGE_LINE_SHIFT 9
#define LINUX_STORAGE_LINE_SIZE (1<<LINUX_STORAGE_LINE_SHIFT)
#define LINUX_STORAGE_NUM_LINES (LINUX_STORAGE_SIZE/LINUX_STORAGE_LINE_SIZE)

class Linux::Storage : public AP_HAL::Storage
{
public:
    Storage() : _fd(-1),_dirty_mask(0) { }

    static Storage *from(AP_HAL::Storage *storage) {
        return static_cast<Storage*>(storage);
    }

    void init() {}
    uint8_t  read_byte(uint16_t loc);
    uint16_t read_word(uint16_t loc);
    uint32_t read_dword(uint16_t loc);
    void     read_block(void *dst, uint16_t src, size_t n);

    void write_byte(uint16_t loc, uint8_t value);
    void write_word(uint16_t loc, uint16_t value);
    void write_dword(uint16_t loc, uint32_t value);
    void write_block(uint16_t dst, const void* src, size_t n);

    virtual void _timer_tick(void);
protected:
    void _mark_dirty(uint16_t loc, uint16_t length);
    virtual void _storage_create(void);
    virtual void _storage_open(void);
    int _fd;
    volatile bool _initialised;
    uint8_t _buffer[LINUX_STORAGE_SIZE];
    volatile uint32_t _dirty_mask;
};

#include "Storage_FRAM.h"
