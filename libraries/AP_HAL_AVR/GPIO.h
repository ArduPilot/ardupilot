
#ifndef __AP_HAL_AVR_GPIO_H__
#define __AP_HAL_AVR_GPIO_H__

#include <AP_HAL.h>
#include "AP_HAL_AVR_Namespace.h"

class AP_HAL_AVR::ArduinoGPIO : public AP_HAL::GPIO {
public:
    ArduinoGPIO() : _init(0) {}
    void init(int machtnicht) { _init = machtnicht; }
private:
    int _init;
};

#endif // __AP_HAL_AVR_GPIO_H__

