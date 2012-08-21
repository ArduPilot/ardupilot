

#ifndef __AP_HAL_AVR_STORAGE_H__
#define __AP_HAL_AVR_STORAGE_H__

#include <AP_HAL.h>

class AP_HAL_AVR::AVREEPROMStorage : public AP_HAL::Storage {
public:
    AVREEPROMStorage() : _init(0) {}
    void init(int machtnicht) { _init = 1; }
private:
    int _init;
};

#endif // __AP_HAL_AVR_STORAGE_H__
