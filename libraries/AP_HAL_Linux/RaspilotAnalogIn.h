#ifndef __RaspilotAnalogIn_H__
#define __RaspilotAnalogIn_H__

#include "AP_HAL_Linux.h"
#include <AP_ADC/AP_ADC.h>

#define RASPILOT_ADC_MAX_CHANNELS 8

class RaspilotAnalogSource: public AP_HAL::AnalogSource {
public:
    friend class RaspilotAnalogIn;
    RaspilotAnalogSource(int16_t pin);
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

class RaspilotAnalogIn: public AP_HAL::AnalogIn {
public:
    RaspilotAnalogIn();
    void init(void* implspecific);
    AP_HAL::AnalogSource* channel(int16_t n);

    /* Board voltage is not available */
    float board_voltage(void);
    
protected:
    AP_HAL::AnalogSource *_vcc_pin_analog_source;
    
private:
    AP_HAL::SPIDeviceDriver *_spi;
    AP_HAL::Semaphore *_spi_sem;
    
    RaspilotAnalogSource *_channels[RASPILOT_ADC_MAX_CHANNELS];

    uint8_t _channels_number;

    void _update();

    uint32_t            _last_update_timestamp;
};

#endif
