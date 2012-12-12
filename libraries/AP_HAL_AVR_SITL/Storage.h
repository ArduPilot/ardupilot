

#ifndef __AP_HAL_AVR_SITL_STORAGE_H__
#define __AP_HAL_AVR_SITL_STORAGE_H__

#include <AP_HAL.h>
#include "AP_HAL_AVR_SITL_Namespace.h"

class SITLEEPROMStorage : public AP_HAL::Storage {
public:
    SITLEEPROMStorage() {
	    _eeprom_fd = -1;
    }
    void init(void* machtnichts) {}
    uint8_t  read_byte(uint16_t loc);
    uint16_t read_word(uint16_t loc);
    uint32_t read_dword(uint16_t loc);
    void     read_block(void *dst, uint16_t src, size_t n);

    void write_byte(uint16_t loc, uint8_t value);
    void write_word(uint16_t loc, uint16_t value);
    void write_dword(uint16_t loc, uint32_t value);
    void write_block(uint16_t dst, void* src, size_t n);

private:
    int _eeprom_fd;
    void _eeprom_open(void);
};

#endif // __AP_HAL_AVR_SITL_STORAGE_H__
