#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_LSM303D_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_Math/AP_Math.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

class AP_Compass_LSM303D : public AP_Compass_Backend
{
public:
    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                     enum Rotation rotation);

    static constexpr const char *name = "LSM303D";

    void read() override;

    virtual ~AP_Compass_LSM303D() { }

private:
    AP_Compass_LSM303D(AP_HAL::OwnPtr<AP_HAL::Device> dev);

    bool init(enum Rotation rotation);
    uint8_t _register_read(uint8_t reg);
    void _register_write(uint8_t reg, uint8_t val);
    void _register_modify(uint8_t reg, uint8_t clearbits, uint8_t setbits);
    bool _block_read(uint8_t reg, uint8_t *buf, uint32_t size);

    bool _read_sample();

    bool _data_ready();
    bool _hardware_init();
    void _update();
    void _disable_i2c();
    bool _mag_set_range(uint8_t max_ga);
    bool _mag_set_samplerate(uint16_t frequency);

    AP_HAL::DigitalSource *_drdy_pin_m;
    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    float _mag_range_scale;
    int16_t _mag_x;
    int16_t _mag_y;
    int16_t _mag_z;

    bool _initialised;

    uint8_t _mag_range_ga;
    uint8_t _mag_samplerate;
    uint8_t _reg7_expected;
};

#endif  // AP_COMPASS_LSM303D_ENABLED
