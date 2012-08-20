
#ifndef __AP_HAL_GPIO_H__
#define __AP_HAL_GPIO_H__

#include "AP_HAL_Namespace.h"

class AP_HAL::GPIO {
public:
    GPIO() {}
    virtual void init(int machtnicht) = 0;
};

#endif // __AP_HAL_GPIO_H__
