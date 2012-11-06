/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_ANALOG_SOURCE_ARDUINO_H__
#define __AP_ANALOG_SOURCE_ARDUINO_H__

#include "AnalogSource.h"
#include "AP_PeriodicProcess.h"

// special pin number which is interpreted as a
// internal Vcc voltage read
#define ANALOG_PIN_VCC 254
#define ANALOG_PIN_NONE 255

class AP_AnalogSource_Arduino : public AP_AnalogSource
{
public:
    AP_AnalogSource_Arduino( uint8_t pin, float prescale = 1.0 ) :
        _prescale(prescale) {
        _assign_pin_index(pin);
    }

    // setup the timer callback
    static void         init_timer(AP_PeriodicProcess * scheduler);

    // read a value with a prescale
    float               read(void);

    // read the raw 16 bit ADC value
    uint16_t            read_raw(void);

    // read a Vcc value in millivolts
    uint16_t            read_vcc(void);

    // read the average 16 bit ADC value since
    // we last called read_average().
    float           read_average(void);

    // set the pin to be used for this source. This allows for the pin
    // to be changed at runtime
    void set_pin(uint8_t pin);

private:
    uint8_t         _pin_index;
    float           _prescale;

    uint8_t         _remap_pin(uint8_t pin);
    void            _assign_pin_index(uint8_t pin);
};

#endif // __AP_ANALOG_SOURCE_ARDUINO_H__
