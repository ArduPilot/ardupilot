
#ifndef __AP_HAL_ANALOG_IN_H__
#define __AP_HAL_ANALOG_IN_H__

#include "AP_HAL_Namespace.h"

class AP_HAL::AnalogSource {
public:
    virtual float read() = 0;
};

class AP_HAL::AnalogIn {
public:
    virtual void init(void* implspecific) = 0;
    virtual AP_HAL::AnalogSource* channel(int n) = 0;
};

#define ANALOG_INPUT_BOARD_VCC 254
#define ANALOG_INPUT_NONE 255

#endif // __AP_HAL_ANALOG_IN_H__

