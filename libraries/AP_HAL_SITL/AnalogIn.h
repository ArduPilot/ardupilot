#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_SITL_Namespace.h"

#define SITL_ADC_RESOLUTION 16 // bits of resolution
#define SITL_ADC_MAX_PIN_VALUE ((1<<SITL_ADC_RESOLUTION)-1)
#define SITL_ADC_FULL_SCALE_VOLTAGE 5.0f
#define SITL_INPUT_MAX_CHANNELS 12

class HALSITL::ADCSource : public AP_HAL::AnalogSource {
public:
    friend class HALSITL::AnalogIn;
    /* pin designates the ADC input number */
    ADCSource(SITL_State *sitlState, int16_t pin);

    /* implement AnalogSource virtual api: */
    float read_average() override;
    float read_latest() override;
    bool set_pin(uint8_t p) override;
    float voltage_average() override;
    float voltage_latest() override;
    float voltage_average_ratiometric() override {
        return voltage_average();
    }

private:
    SITL_State *_sitlState;
    int16_t _pin;
};

/* AnalogIn : a concrete class providing the implementations of the
 * timer event and the AP_HAL::AnalogIn interface */
class HALSITL::AnalogIn : public AP_HAL::AnalogIn {
public:
    explicit AnalogIn(SITL_State *sitlState): _sitlState(sitlState) {}
    void init() override;
    AP_HAL::AnalogSource* channel(int16_t n) override;
    float board_voltage(void) override {
        return 5.0f;
    }
private:
    static ADCSource* _channels[SITL_INPUT_MAX_CHANNELS];
    SITL_State *_sitlState;
};
