#ifndef __ADS1115AnalogIn_H__
#define __ADS1115AnalogIn_H__

#include "AP_HAL_Linux.h"
#include <AP_ADC/AP_ADC.h>

#define ADS1115_ADC_MAX_CHANNELS 6

class ADS1115AnalogSource: public AP_HAL::AnalogSource {
public:
    friend class ADS1115AnalogIn;
    ADS1115AnalogSource(int16_t pin);
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

class ADS1115AnalogIn: public AP_HAL::AnalogIn {
public:
    ADS1115AnalogIn();
    void init();
    AP_HAL::AnalogSource* channel(int16_t n);

    /* Board voltage is not available */
    float board_voltage(void) 
    { 
        return 0.0f; 
    }
private:
    AP_ADC_ADS1115 *_adc;
    ADS1115AnalogSource *_channels[ADS1115_ADC_MAX_CHANNELS];

    uint8_t _channels_number;

    void _update();

    uint32_t            _last_update_timestamp;
};

#endif
