
#ifndef __AP_HAL_AVR_LOG_H__
#define __AP_HAL_AVR_LOG_H__

#include <AP_HAL.h>
#include "AP_HAL_AVR_Namespace.h"

class AP_HAL_AVR::DataFlashAPM1Log : public AP_HAL::Log {
public:
    DataFlashAPM1Log() : _init(0) {}
    void init(int machtnicht) { _init = 1; }
private:
    int _init;
};

class AP_HAL_AVR::DataFlashAPM2Log : public AP_HAL::Log {
public:
    DataFlashAPM2Log() : _init(0) {}
    void init(int machtnicht) { _init = 2; }
private:
    int _init;
};

#endif // __AP_HAL_AVR_LOG_H__

