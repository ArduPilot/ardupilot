
#ifndef __AP_HAL_AVR_RC_OUTPUT_H__
#define __AP_HAL_AVR_RC_OUTPUT_H__

#include <AP_HAL.h>
#include "AP_HAL_AVR_Namespace.h"

class AP_HAL_AVR::APM1RCOutput : public AP_HAL::RCOutput {
public:
    APM1RCOutput() : _init(0) {}
    void init(int machtnicht) { _init = 1; }
private:
    int _init;
};

class AP_HAL_AVR::APM2RCOutput : public AP_HAL::RCOutput {
public:
    APM2RCOutput() : _init(0) {}
    void init(int machtnicht) { _init = 2; }
private:
    int _init;
};

#endif // __AP_HAL_AVR_RC_OUTPUT_H__

