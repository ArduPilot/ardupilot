
#ifndef __AP_HAL_ANALOG_IN_H__
#define __AP_HAL_ANALOG_IN_H__

#include "AP_HAL_Namespace.h"

class AP_HAL::AnalogIn {
public:
    AnalogIn() {}
    virtual void init(int machtnicht) = 0;
};

#endif // __AP_HAL_ANALOG_IN_H__

