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

    AP_HAL::SPIDeviceDriver *_spi = nullptr;
    AP_HAL::Semaphore *_spi_sem = nullptr;
    AP_HAL::DigitalSource *_drdy_pin_m = nullptr;
    float _scaling[3] = { 0 };
    bool _initialised = false;

    int16_t _mag_x = 0;
    int16_t _mag_y = 0;
    int16_t _mag_z = 0;
    float _mag_x_accum = 0.0f;
    float _mag_y_accum = 0.0f;
    float _mag_z_accum = 0.0f;
    uint32_t _last_accum_time = 0;
    uint32_t _last_update_timestamp = 0;
    uint8_t _accum_count = 0;

    uint8_t _compass_instance = 0;
    uint8_t _product_id = 0;

    uint8_t _mag_range_ga = 0;
    uint8_t _mag_samplerate = 0;
    uint8_t _reg7_expected = 0;
    float _mag_range_scale = 0.0f;

public:
    AP_Compass_LSM303D(Compass &compass);
    bool        init(void);
    void        read(void);
    uint32_t    get_dev_id();

    // detect the sensor
    static AP_Compass_Backend *detect_spi(Compass &compass);
};
