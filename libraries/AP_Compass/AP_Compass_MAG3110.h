#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_Math/AP_Math.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"


#ifndef HAL_MAG3110_I2C_ADDR 
 #define HAL_MAG3110_I2C_ADDR     0x0E
#endif

class AP_Compass_MAG3110 : public AP_Compass_Backend
{
public:
    static AP_Compass_Backend *probe(Compass &compass,
                                     AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                     enum Rotation = ROTATION_NONE);

    static constexpr const char *name = "MAG3110";

    void read() override;

    ~AP_Compass_MAG3110() { }

private:
    AP_Compass_MAG3110(Compass &compass, AP_HAL::OwnPtr<AP_HAL::Device> dev);

    bool init(enum Rotation rotation);

    bool _read_sample();

    bool _hardware_init();
    void _update();

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    int32_t _mag_x;
    int32_t _mag_y;
    int32_t _mag_z;

    float _mag_x_accum;
    float _mag_y_accum;
    float _mag_z_accum;
    uint8_t _accum_count;

    uint8_t _compass_instance;
    bool _initialised;
};
