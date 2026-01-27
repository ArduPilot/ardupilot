#pragma once

#include "AP_HAL_RP.h"
#include "hardware/adc.h"

namespace RP {

class AnalogSource : public AP_HAL::AnalogSource {
public:
    friend class AnalogIn;
    AnalogSource(uint8_t pin);

    float read_average() override;
    float read_latest() override;
    bool set_pin(uint8_t p) override;
    float voltage_average() override;
    float voltage_latest() override;
    float voltage_average_ratiometric() override;

private:
    uint8_t _pin;
    float _latest_value;
    float _sum;
    uint32_t _count;

    // Semaphore to protect access to _sum and _count
    HAL_Semaphore _semaphore;

    void add_sample(uint16_t sample);
};

class AnalogIn : public AP_HAL::AnalogIn {
public:
    void init() override;
    AP_HAL::AnalogSource* channel(int16_t n) override;
    float board_voltage(void) override { return 3.3f; }
    bool valid_analog_pin(uint16_t pin) const override;

private:
    // Support 5 channels for RP2350 (ADC0-3 external + ADC4 internal thermometer)
    static AnalogSource* _channels[5];
    void _update();
};

} // namespace RP
