#pragma once

#include "AP_HAL_Empty.h"

class Empty::AnalogSource : public AP_HAL::AnalogSource {
public:
    AnalogSource(float v);
    float read_average();
    float read_latest();
    void set_pin(uint8_t p);
    void set_stop_pin(uint8_t p);
    void set_settle_time(uint16_t settle_time_ms);
    float voltage_average();
    float voltage_latest();
    float voltage_average_ratiometric() { return voltage_average(); }
private:
    float _v;
};

class Empty::AnalogIn : public AP_HAL::AnalogIn {
public:
    AnalogIn();
    void init();
    AP_HAL::AnalogSource* channel(int16_t n);
    float board_voltage(void);
};
