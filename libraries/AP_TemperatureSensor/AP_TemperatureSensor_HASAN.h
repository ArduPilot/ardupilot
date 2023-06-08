#pragma once
#include "AP_TemperatureSensor_Backend.h"

#ifndef AP_TEMPERATURE_SENSOR_HASAN_ENABLED
#define AP_TEMPERATURE_SENSOR_HASAN_ENABLED AP_TEMPERATURE_SENSOR_ENABLED
#endif

#if AP_TEMPERATURE_SENSOR_HASAN_ENABLED

#define HASAN_ADDR 0x49 // hASAN's address

class AP_TemperatureSensor_HASAN : public AP_TemperatureSensor_Backend {
public:
    using AP_TemperatureSensor_Backend::AP_TemperatureSensor_Backend;

    void init(void) override;
    void update() override {};
private:
    uint8_t _buffer[4]; // Buffer for incoming data
    //AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

    void start_next_sample();
    uint32_t read_adc(void) const;
    void _timer(void);
    float calculate(const uint32_t adc) const;
    float read_temperature();
};

#endif // AP_TEMPERATURE_SENSOR_HASAN_ENABLED
