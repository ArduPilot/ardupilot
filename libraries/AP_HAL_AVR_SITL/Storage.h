

#ifndef __AP_HAL_AVR_SITL_STORAGE_H__
#define __AP_HAL_AVR_SITL_STORAGE_H__

#include <AP_HAL.h>
#include "AP_HAL_AVR_SITL_Namespace.h"

class AVR_SITL::SITLEEPROMStorage : public AP_HAL::Storage {
public:
    SITLEEPROMStorage() {
	    _eeprom_fd = -1;
    }
    void init(void* machtnichts) {}
    void read_block(void *dst, uint16_t src, size_t n);
    void write_block(uint16_t dst, const void* src, size_t n);

private:
    int _eeprom_fd;
    void _eeprom_open(void);
};

#endif // __AP_HAL_AVR_SITL_STORAGE_H__
