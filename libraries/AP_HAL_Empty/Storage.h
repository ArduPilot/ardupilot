
#ifndef __AP_HAL_EMPTY_STORAGE_H__
#define __AP_HAL_EMPTY_STORAGE_H__

#include <AP_HAL_Empty.h>

class Empty::EmptyStorage : public AP_HAL::Storage {
public:
    EmptyStorage();
    void init(void *);
    uint8_t  read_byte(uint16_t loc);
    uint16_t read_word(uint16_t loc);
    uint32_t read_dword(uint16_t loc);
    void     read_block(void *dst, uint16_t src, size_t n);

    void write_byte(uint16_t loc, uint8_t value);
    void write_word(uint16_t loc, uint16_t value);
    void write_dword(uint16_t loc, uint32_t value);
    void write_block(uint16_t dst, void* src, size_t n);
};

#endif // __AP_HAL_EMPTY_STORAGE_H__
