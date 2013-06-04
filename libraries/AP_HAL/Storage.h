
#ifndef __AP_HAL_STORAGE_H__
#define __AP_HAL_STORAGE_H__

#include <stdint.h>
#include "AP_HAL_Namespace.h"

class AP_HAL::Storage {
public:
    virtual void init(void *) = 0;
    virtual uint8_t  read_byte(uint16_t loc) = 0;
    virtual uint16_t read_word(uint16_t loc) = 0;
    virtual uint32_t read_dword(uint16_t loc) = 0;
    virtual void     read_block(void *dst, uint16_t src, size_t n) = 0;

    virtual void write_byte(uint16_t loc, uint8_t value) = 0;
    virtual void write_word(uint16_t loc, uint16_t value) = 0;
    virtual void write_dword(uint16_t loc, uint32_t value) = 0;
    virtual void write_block(uint16_t dst, const void* src, size_t n) = 0;
};

#endif // __AP_HAL_STORAGE_H__

