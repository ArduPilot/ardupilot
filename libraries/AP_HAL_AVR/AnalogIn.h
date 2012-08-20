
#ifndef __AP_HAL_AVR_ANALOG_IN_H__
#define __AP_HAL_AVR_ANALOG_IN_H__

#include <AP_HAL.h>

class AP_HAL::ArduinoAnalogIn : public AP_HAL::AnalogIn {
public:
    ArduinoAnalogIn() : _init(0) {}
    void init(int machtnicht) { _init = 1; }
private:
    int _init;
};

#endif // __AP_HAL_AVR_ANALOG_IN_H__

