/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "Compass.h"
#include "AP_Compass_Backend.h"

class AP_Compass_LSM303D : public AP_Compass_Backend
{
private:
    AP_HAL::DigitalSource *_drdy_pin_m;
    float   _scaling[3] = { 0 };
    bool    _initialised;
    bool    _read_raw(void);
    uint8_t _register_read( uint8_t reg );
    void    _register_write( uint8_t reg, uint8_t val );
    void    _register_modify(uint8_t reg, uint8_t clearbits, uint8_t setbits);
    bool    _data_ready();
    bool    _hardware_init(void);
    void    _collect_samples();
    void    _update();
    void    _disable_i2c(void);
    uint8_t _mag_set_range(uint8_t max_ga);
    uint8_t _mag_set_samplerate(uint16_t frequency);

    AP_HAL::SPIDeviceDriver *_spi;
    AP_HAL::Semaphore *_spi_sem;

    int16_t _mag_x;
    int16_t _mag_y;
    int16_t _mag_z;
    int16_t _mag_x_accum;
    int16_t _mag_y_accum;
    int16_t _mag_z_accum;
    uint32_t _last_accum_time;
    uint32_t _last_update_timestamp;
    uint8_t _accum_count;

    uint8_t _compass_instance;
    uint8_t _product_id;

    uint8_t _mag_range_ga;
    uint8_t _mag_samplerate;
    uint8_t _reg7_expected;
    float _mag_range_scale;

public:
    AP_Compass_LSM303D(Compass &compass);
    bool        init(void);
    void        read(void);

    // detect the sensor
    static AP_Compass_Backend *detect_spi(Compass &compass);
};
