/*
 * I2C driver for Measurement Specialties MEAS TSYS01 digital temperature sensor
 */

#pragma once
#include "AP_TemperatureSensor_Backend.h"

#ifndef AP_TEMPERATURE_SENSOR_TSYS01_ENABLE
#define AP_TEMPERATURE_SENSOR_TSYS01_ENABLE AP_TEMPERATURE_SENSOR_ENABLED
#endif

#if AP_TEMPERATURE_SENSOR_TSYS01_ENABLE
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>

#define TSYS01_ADDR_CSB0 0x77
#define TSYS01_ADDR_CSB1 0x76

class AP_TemperatureSensor_TSYS01 : public AP_TemperatureSensor_Backend {
    using AP_TemperatureSensor_Backend::AP_TemperatureSensor_Backend;

public:
    void init(void) override;

    void update() override {};

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

private:
    uint16_t _k[5];         // internal calibration for temperature calculation

    // reset device
    bool _reset(void) const;

    // read (relevant) internal calibration registers into _k
    bool _read_prom(void);
    uint16_t _read_prom_word(uint8_t word) const;

    // begin an ADC conversion (min:7.40ms typ:8.22ms max:9.04ms)
    bool _convert(void) const;
    uint32_t _read_adc(void) const;

    // update the temperature, called at 20Hz
    void _timer(void);

    // calculate temperature using adc reading and internal calibration
    void _calculate(uint32_t adc);
};
#endif // AP_TEMPERATURE_SENSOR_TSYS01_ENABLE
