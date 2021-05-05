/*
 * I2C driver for Measurement Specialties MEAS TSYS01 digital temperature sensor
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>

#define TSYS01_ADDR 0x77

class TSYS01 {
public:

    bool init(uint8_t bus);
    float temperature(void) const { return _temperature; } // temperature in degrees C
    bool healthy(void) const { // do we have a valid temperature reading?
        return _healthy;
    }

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

private:
    float _temperature; // degrees C
    bool _healthy; // we have a valid temperature reading to report
    uint16_t _k[5]; // internal calibration for temperature calculation
    bool _reset(void) const; // reset device
    bool _read_prom(void); // read (relevant) internal calibration registers into _k
    bool _convert(void) const; // begin an ADC conversion (min:7.40ms typ:8.22ms max:9.04ms)
    uint32_t _read_adc(void) const;
    uint16_t _read_prom_word(uint8_t word) const;
    void _timer(void); // update the temperature, called at 20Hz
    void _calculate(uint32_t adc); // calculate temperature using adc reading and internal calibration
};
