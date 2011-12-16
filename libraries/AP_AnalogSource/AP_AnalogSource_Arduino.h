/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_ANALOG_SOURCE_ARDUINO_H__
#define __AP_ANALOG_SOURCE_ARDUINO_H__

#include "AnalogSource.h"

class AP_AnalogSource_Arduino : public AP_AnalogSource
{
    public:
    AP_AnalogSource_Arduino( int pin, float prescale = 1.0 ) : 
        _pin(pin), _prescale(prescale) {}
    float read(void);

    private:
    int _pin;
    float _prescale;
};

#endif // __AP_ANALOG_SOURCE_ARDUINO_H__
