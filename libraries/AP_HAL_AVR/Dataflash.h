
#ifndef __AP_HAL_AVR_DATAFLASH_H__
#define __AP_HAL_AVR_DATAFLASH_H__

#include <AP_HAL.h>
#include "AP_HAL_AVR_Namespace.h"

class AP_HAL_AVR::APM1Dataflash : public AP_HAL::Dataflash {
public:
    APM1Dataflash() : _init(0) {}
    void init(int machtnicht) { _init = 1; }
private:
    int _init;
};

class AP_HAL_AVR::APM2Dataflash : public AP_HAL::Dataflash {
public:
    APM2Dataflash() : _init(0) {}
    void init(int machtnicht) { _init = 2; }
private:
    int _init;
};

#endif // __AP_HAL_AVR_DATAFLASH_H__

