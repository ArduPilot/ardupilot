
#ifndef __AP_HAL_AVR_PPM_INPUT_H__
#define __AP_HAL_AVR_PPM_INPUT_H__

#include <AP_HAL.h>
#include "AP_HAL_AVR_Namespace.h"

class AP_HAL_AVR::APM1PPMInput : public AP_HAL::PPMInput {
public:
    APM1PPMInput() : _init(0) {}
    void init(int machtnicht) { _init = 1; }
private:
    int _init;
};

class AP_HAL_AVR::APM2PPMInput : public AP_HAL::PPMInput {
public:
    APM2PPMInput() : _init(0) {}
    void init(int machtnicht) { _init = 2; }
private:
    int _init;
};

#endif // __AP_HAL_AVR_PPM_INPUT_H__

