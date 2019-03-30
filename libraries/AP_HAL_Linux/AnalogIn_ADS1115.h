#pragma once

#include "AP_HAL_Linux.h"
#include <AP_ADC/AP_ADC_ADS1115.h>

#define ADS1115_ADC_MAX_CHANNELS 6

class AnalogSource_ADS1115: public AP_HAL::AnalogSource {
public:
    friend class AnalogIn_ADS1115;
    AnalogSource_ADS1115(int16_t pin);
    float read_average() override;
    float read_latest() override;
    void set_pin(uint8_t p) override;
    void set_stop_pin(uint8_t p) override {}
    void set_settle_time(uint16_t settle_time_ms) override {}
    float voltage_average() override;
    float voltage_latest() override;
    float voltage_average_ratiometric() override;
private:
    int16_t _pin;
    float _value;
};

class AnalogIn_ADS1115: public AP_HAL::AnalogIn {
public:
    AnalogIn_ADS1115();

    void init() override;
    AP_HAL::AnalogSource *channel(int16_t n) override;

    /* Board voltage is not available */
    float board_voltage() override { return 5.0f; }

private:
    uint8_t _channels_number;
    void _update();

    AP_ADC_ADS1115 *_adc;
    AnalogSource_ADS1115 *_channels[ADS1115_ADC_MAX_CHANNELS];
    uint32_t _last_update_timestamp;
};
