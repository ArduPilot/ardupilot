#pragma once

#include "AP_HAL_Linux.h"

#include <fcntl.h>

#define NAVIO_ADC_MAX_CHANNELS 6

class AnalogSource_Navio2: public AP_HAL::AnalogSource {
public:
    friend class AnalogIn_Navio2;
    AnalogSource_Navio2(uint8_t pin);
    float read_average() override;
    float read_latest() override;
    void set_pin(uint8_t p) override;
    float voltage_average() override;
    float voltage_latest() override;
    float voltage_average_ratiometric() override;
private:
    void set_channel(uint8_t pin);
    uint8_t _pin;
    int _fd = -1;
    float _value = 0.0f;
};

class AnalogIn_Navio2: public AP_HAL::AnalogIn {
public:
    AnalogIn_Navio2();
    void init() override;
    AP_HAL::AnalogSource *channel(int16_t n) override;
    float board_voltage(void) override;
    float servorail_voltage(void) override;

private:
    AnalogSource_Navio2 *_channels[NAVIO_ADC_MAX_CHANNELS];
    uint8_t _channels_number = NAVIO_ADC_MAX_CHANNELS;
    AP_HAL::AnalogSource *_board_voltage_pin = nullptr;
    AP_HAL::AnalogSource *_servorail_pin = nullptr;
};
