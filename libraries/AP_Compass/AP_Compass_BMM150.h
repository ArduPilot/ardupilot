#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/SPIDevice.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

class AP_Compass_BMM150 : public AP_Compass_Backend
{
public:
    static AP_Compass_Backend *probe(Compass &compass,
                                     AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    bool init() override;
    void read() override;

    static constexpr const char *name = "BMM150";

private:

    AP_Compass_BMM150(Compass &compass, AP_HAL::OwnPtr<AP_HAL::Device> dev);

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    Vector3f _mag_accum = Vector3f();
    uint32_t _accum_count = 0;
    uint32_t _last_update_timestamp = 0;

    uint8_t _compass_instance;

    void _update();

    int8_t _dig_x1;
    int8_t _dig_y1;
    int8_t _dig_x2;
    int8_t _dig_y2;
    uint16_t _dig_z1;
    int16_t _dig_z2;
    int16_t _dig_z3;
    int16_t _dig_z4;
    uint8_t _dig_xy1;
    int8_t _dig_xy2;
    uint16_t _dig_xyz1;
    bool _load_trim_values();

    int16_t _compensate_xy(int16_t xy, uint32_t rhall, int32_t txy1, int32_t txy2);
    int16_t _compensate_z(int16_t z, uint32_t rhall);
};
