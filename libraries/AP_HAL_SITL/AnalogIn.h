#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_SITL_Namespace.h"

#define SITL_INPUT_MAX_CHANNELS 12

class HALSITL::ADCSource : public AP_HAL::AnalogSource {
public:
    friend class HALSITL::AnalogIn;
    /* pin designates the ADC input number */
    ADCSource(SITL_State *sitlState, int16_t pin);

    /* implement AnalogSource virtual api: */
    float read_average();
    float read_latest();
    void set_pin(uint8_t p);
    float voltage_average();
    float voltage_latest();
    float voltage_average_ratiometric() {
        return voltage_average();
    }
    void set_stop_pin(uint8_t pin) {}
    void set_settle_time(uint16_t settle_time_ms) {}

private:
    SITL_State *_sitlState;
    int16_t _pin;
};

/* AnalogIn : a concrete class providing the implementations of the
 * timer event and the AP_HAL::AnalogIn interface */
class HALSITL::AnalogIn : public AP_HAL::AnalogIn {
public:
    explicit AnalogIn(SITL_State *sitlState): _sitlState(sitlState) {}
    void init();
    AP_HAL::AnalogSource* channel(int16_t n);
    float board_voltage(void) {
        return 5.0f;
    }
private:
    static ADCSource* _channels[SITL_INPUT_MAX_CHANNELS];
    SITL_State *_sitlState;
};
