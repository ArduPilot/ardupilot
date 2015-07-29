

#ifndef __AP_HAL_AVR_STORAGE_H__
#define __AP_HAL_AVR_STORAGE_H__

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_AVR_Namespace.h"

class AP_HAL_AVR::AVREEPROMStorage : public AP_HAL::Storage {
public:
    AVREEPROMStorage() {}
    void init(void* machtnichts) {}
    void read_block(void *dst, uint16_t src, size_t n);
    void write_block(uint16_t dst, const void* src, size_t n);
};

#endif // __AP_HAL_AVR_STORAGE_H__
