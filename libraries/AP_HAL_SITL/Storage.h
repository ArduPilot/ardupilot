

#ifndef __AP_HAL_SITL_STORAGE_H__
#define __AP_HAL_SITL_STORAGE_H__

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_SITL_Namespace.h"

class HALSITL::EEPROMStorage : public AP_HAL::Storage {
public:
    EEPROMStorage() {
        _eeprom_fd = -1;
    }
    void init() {}
    void read_block(void *dst, uint16_t src, size_t n);
    void write_block(uint16_t dst, const void* src, size_t n);

private:
    int _eeprom_fd;
    void _eeprom_open(void);
};

#endif // __AP_HAL_SITL_STORAGE_H__
