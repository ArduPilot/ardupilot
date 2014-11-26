#ifndef __NavioAnalogIn_H__
#define __NavioAnalogIn_H__

#include <AP_HAL.h>
#include <AP_ADC.h>


#define NAVIO_ADC_MAX_CHANNELS 8

class NavioAnalogSource: public AP_HAL::AnalogSource {
public:
    friend class NavioAnalogIn;
    NavioAnalogSource(int16_t pin);
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

class NavioAnalogIn: public AP_HAL::AnalogIn {
public:
    NavioAnalogIn();
    void init(void* implspecific);
    AP_HAL::AnalogSource* channel(int16_t n);

    // we don't yet know how to get the board voltage
    float board_voltage(void) { return 0.0f; }
private:
    AP_ADC *_adc;
    NavioAnalogSource *_channels[NAVIO_ADC_MAX_CHANNELS];

    void _update();

    uint32_t            _last_update_timestamp;
};

#endif
