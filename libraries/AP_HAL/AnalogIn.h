#pragma once

#include <inttypes.h>

#include "AP_HAL_Namespace.h"

class AP_HAL::AnalogSource {
public:
    virtual float read_average() = 0;
    virtual float read_latest() = 0;
    virtual void set_pin(uint8_t p) = 0;

    // return a voltage from 0.0 to 5.0V, scaled
    // against a reference voltage
    virtual float voltage_average() = 0;

    // return a voltage from 0.0 to 5.0V, scaled
    // against a reference voltage
    virtual float voltage_latest() = 0;

    // return a voltage from 0.0 to 5.0V, assuming a ratiometric
    // sensor
    virtual float voltage_average_ratiometric() = 0;
};

class AP_HAL::AnalogIn {
public:
    virtual void init() = 0;
    virtual AP_HAL::AnalogSource* channel(int16_t n) = 0;

    // board 5V rail voltage in volts
    virtual float board_voltage(void) = 0;

    // servo rail voltage in volts, or 0 if unknown
    virtual float servorail_voltage(void) { return 0; }

    // power supply status flags, see MAV_POWER_STATUS
    virtual uint16_t power_status_flags(void) { return 0; }
};

#define ANALOG_INPUT_BOARD_VCC 254
#define ANALOG_INPUT_NONE 255
