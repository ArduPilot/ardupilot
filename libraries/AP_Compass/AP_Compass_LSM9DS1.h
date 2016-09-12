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
                                     AP_HAL::OwnPtr<AP_HAL::Device> dev);

    static constexpr const char *name = "LSM9DS1";

    bool init() override;
    void read() override;

    virtual ~AP_Compass_LSM9DS1() {}

private:
    AP_Compass_LSM9DS1(Compass &compass, AP_HAL::OwnPtr<AP_HAL::Device> dev, uint32_t dev_id);
    bool _check_id(void);
    bool _configure(void);
    bool _set_scale(void);
    void _update(void);
    void _reset_filter(void);
    Vector3f _get_filtered_field() const;

    uint8_t _register_read(uint8_t reg);
    void _register_write(uint8_t reg, uint8_t val);
    void _register_modify(uint8_t reg, uint8_t clearbits, uint8_t setbits);
    bool _block_read(uint8_t reg, uint8_t *buf, uint32_t size);
    void _dump_registers();

    AP_HAL::OwnPtr<AP_HAL::Device> _dev = nullptr;
    uint32_t _dev_id = 0;
    uint8_t _compass_instance = 0;
    float _scaling = 0.0f;
    float _mag_x_accum = 0.0f;
    float _mag_y_accum = 0.0f;
    float _mag_z_accum = 0.0f;
    uint32_t _accum_count = 0;
    uint32_t _last_update_timestamp;
    uint32_t _last_accum_time;
};
