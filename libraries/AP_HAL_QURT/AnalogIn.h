#pragma once

#include "AP_HAL_QURT.h"

class QURT::AnalogSource : public AP_HAL::AnalogSource
{
public:
    float read_average() override;
    float read_latest() override;
    bool set_pin(uint8_t p) override;
    float voltage_average() override;
    float voltage_latest() override;
    float voltage_average_ratiometric() override
    {
        return voltage_average();
    }
    uint8_t pin;
};

class QURT::AnalogIn : public AP_HAL::AnalogIn
{
public:
    AnalogIn();
    void init() override;
    AP_HAL::AnalogSource* channel(int16_t n) override;
    float board_voltage(void) override;
private:
    uint8_t next_chan;
};
