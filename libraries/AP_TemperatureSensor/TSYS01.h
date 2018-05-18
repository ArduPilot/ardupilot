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

    bool init(void);
    float temperature(void) { return _temperature; } // temperature in degrees C
    bool healthy(void) { // do we have a valid temperature reading?
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        return true;
#endif
        return _healthy;
    }

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

private:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    float _temperature = 42.42; // degrees C
#else
    float _temperature; // degrees C
#endif
    bool _healthy; // we have a valid temperature reading to report
    uint16_t _k[5]; // internal calibration for temperature calculation
    bool _reset(void); // reset device
    bool _read_prom(void); // read (relevant) internal calibration registers into _k
    bool _convert(void); // begin an ADC conversion (min:7.40ms typ:8.22ms max:9.04ms)
    uint32_t _read_adc(void);
    uint16_t _read_prom_word(uint8_t word);
    void _timer(void); // update the temperature, called at 20Hz
    void _calculate(uint32_t adc); // calculate temperature using adc reading and internal calibration
};
