
#ifndef __AP_ANALOG_SOURCE_ARDUINO_H__
#define __AP_ANALOG_SOURCE_ARDUINO_H__

#include "AnalogSource.h"

class AP_AnalogSource_Arduino : public AP_AnalogSource
{
    public:
    AP_AnalogSource_Arduino( int pin ) : _pin(pin) {}
    int read(void);

    private:
    int _pin;
};

#endif // __AP_ANALOG_SOURCE_ARDUINO_H__
