/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_HAL_PX4_ANALOGIN_H__
#define __AP_HAL_PX4_ANALOGIN_H__

#include <AP_HAL_PX4.h>
#include <pthread.h>

#define PX4_ANALOG_MAX_CHANNELS 8

class PX4::PX4AnalogSource : public AP_HAL::AnalogSource {
public:
    friend class PX4::PX4AnalogIn;
    PX4AnalogSource(int16_t pin, float initial_value, float scale);
    float read_average();
    float read_latest();
    void set_pin(uint8_t p);
private:
    // what pin it is attached to
    int16_t _pin;

    // what value it has
    float _value;
    float _latest_value;
    uint8_t _sum_count;
    float _sum_value;
    float _scale;
};

class PX4::PX4AnalogIn : public AP_HAL::AnalogIn {
public:
    PX4AnalogIn();
    void init(void* implspecific);
    AP_HAL::AnalogSource* channel(int16_t pin);
    AP_HAL::AnalogSource* channel(int16_t pin, float scale);

private:
    static int _adc_fd;
    static PX4::PX4AnalogSource* _channels[PX4_ANALOG_MAX_CHANNELS];
    static void _analogin_timer(uint32_t now);
    static uint32_t _last_run;
};
#endif // __AP_HAL_PX4_ANALOGIN_H__
