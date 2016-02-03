#pragma once

#include <AP_ADC/AP_ADC.h>

#include "AP_HAL_Linux.h"

#define RASPILOT_ADC_MAX_CHANNELS 8

class AnalogSource_Raspilot: public AP_HAL::AnalogSource {
public:
    friend class AnalogIn_Raspilot;
    AnalogSource_Raspilot(int16_t pin);
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

class AnalogIn_Raspilot: public AP_HAL::AnalogIn {
public:
    AnalogIn_Raspilot();
    void init();
    AP_HAL::AnalogSource* channel(int16_t n);

    /* Board voltage is not available */
    float board_voltage(void);

protected:
    AP_HAL::AnalogSource *_vcc_pin_analog_source;

private:
    AP_HAL::SPIDeviceDriver *_spi;
    AP_HAL::Semaphore *_spi_sem;

    AnalogSource_Raspilot *_channels[RASPILOT_ADC_MAX_CHANNELS];

    uint8_t _channels_number;

    void _update();

    uint32_t            _last_update_timestamp;
};
