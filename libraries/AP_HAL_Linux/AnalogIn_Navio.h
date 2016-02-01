#ifndef __NavioAnalogIn_H__
#define __NavioAnalogIn_H__

#include "AP_HAL_Linux.h"

#include <fcntl.h>

#define NAVIO_ADC_MAX_CHANNELS 6

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
    void set_channel(int16_t pin);
    int16_t _pin;
    int _fd = -1;
    float _value = 0.0f;
};

class NavioAnalogIn: public AP_HAL::AnalogIn {
public:
    NavioAnalogIn();
    void init();
    AP_HAL::AnalogSource* channel(int16_t n);
    float board_voltage(void);
    float servorail_voltage(void) override;

private:
    NavioAnalogSource *_channels[NAVIO_ADC_MAX_CHANNELS];

    uint8_t _channels_number;

    void _update();

    uint32_t            _last_update_timestamp;
    AP_HAL::AnalogSource *_board_voltage_pin = nullptr;
    AP_HAL::AnalogSource *_servorail_pin = nullptr;
};

#endif
