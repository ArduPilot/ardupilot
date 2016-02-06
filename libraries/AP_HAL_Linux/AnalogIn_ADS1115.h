#pragma once

#include "AP_HAL_Linux.h"
#include <AP_ADC/AP_ADC.h>

#define ADS1115_ADC_MAX_CHANNELS 6

class AnalogSource_ADS1115: public AP_HAL::AnalogSource {
public:
    friend class AnalogIn_ADS1115;
    AnalogSource_ADS1115(int16_t pin);
    float read_average();
    float read_latest();
    void set_pin(uint8_t p);
    void set_stop_pin(uint8_t p) {}
    void set_settle_time(uint16_t settle_time_ms){}
    float voltage_average();
    float voltage_latest();
    float voltage_average_ratiometric();
private:
    int16_t _pin;
    float _value;
};

class AnalogIn_ADS1115: public AP_HAL::AnalogIn {
public:
    AnalogIn_ADS1115();
    void init();
    AP_HAL::AnalogSource* channel(int16_t n);

    /* Board voltage is not available */
    float board_voltage(void)
    {
        return 0.0f;
    }
private:
    uint8_t _channels_number;
    void _update();

    AP_ADC_ADS1115 *_adc;
    AnalogSource_ADS1115 *_channels[ADS1115_ADC_MAX_CHANNELS];
    uint32_t _last_update_timestamp;
};
