

#ifndef __AP_HAL_AVR_STORAGE_H__
#define __AP_HAL_AVR_STORAGE_H__

#include <AP_HAL.h>
#include "AP_HAL_AVR_Namespace.h"

class AP_HAL_AVR::AVREEPROMStorage : public AP_HAL::Storage {
public:
    AVREEPROMStorage() {}
    void init(void* machtnicht) {}
    uint8_t  read_byte(const uint8_t   *p);
    uint16_t read_word(const uint16_t  *p);
    uint32_t read_dword(const uint32_t *p);
    void     read_block(void *dst, const void *src, size_t n);

    void write_byte(uint8_t   *p, uint8_t value);
    void write_word(uint16_t  *p, uint16_t value);
    void write_dword(uint32_t *p, uint32_t value);
    void write_block(void *src, void *dst, size_t n);
};

#endif // __AP_HAL_AVR_STORAGE_H__
