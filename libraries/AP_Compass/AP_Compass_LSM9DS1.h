#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_Math/AP_Math.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

class AP_Compass_LSM9DS1 : public AP_Compass_Backend
{
public:
    static AP_Compass_Backend *probe(Compass &compass,
                                     AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                     enum Rotation rotation = ROTATION_NONE);

    static constexpr const char *name = "LSM9DS1";

    void read() override;

    virtual ~AP_Compass_LSM9DS1() {}

private:
    AP_Compass_LSM9DS1(Compass &compass, AP_HAL::OwnPtr<AP_HAL::Device> dev,
                       enum Rotation rotation = ROTATION_NONE);
    bool init();
    bool _check_id(void);
    bool _configure(void);
    bool _set_scale(void);
    void _update(void);

    uint8_t _register_read(uint8_t reg);
    void _register_write(uint8_t reg, uint8_t val);
    void _register_modify(uint8_t reg, uint8_t clearbits, uint8_t setbits);
    bool _block_read(uint8_t reg, uint8_t *buf, uint32_t size);
    void _dump_registers();

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;
    uint8_t _compass_instance;
    float _scaling;
    float _mag_x_accum;
    float _mag_y_accum;
    float _mag_z_accum;
    uint32_t _accum_count;
    enum Rotation _rotation;
};
