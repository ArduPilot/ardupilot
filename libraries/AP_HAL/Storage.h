
#ifndef __AP_HAL_STORAGE_H__
#define __AP_HAL_STORAGE_H__

#include <stdint.h>
#include "AP_HAL_Namespace.h"

class AP_HAL::Storage {
public:
    virtual void init(void *) = 0;
    virtual uint8_t  read_byte(const uint8_t   *p) = 0;
    virtual uint16_t read_word(const uint16_t  *p) = 0;
    virtual uint32_t read_dword(const uint32_t *p) = 0;
    virtual void     read_block(void *dst, const void *src, size_t n) = 0;

    virtual void write_byte(uint8_t   *p, uint8_t value) = 0;
    virtual void write_word(uint16_t  *p, uint16_t value) = 0;
    virtual void write_dword(uint32_t *p, uint32_t value) = 0;
    virtual void write_block(void *src, void *dst, size_t n) = 0;
};

#endif // __AP_HAL_STORAGE_H__

