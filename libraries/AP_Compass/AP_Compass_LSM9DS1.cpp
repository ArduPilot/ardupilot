
#include <assert.h>
#include <utility>

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>

#include "AP_Compass_LSM9DS1.h"


#define LSM9DS1M_OFFSET_X_REG_L_M   0x05
#define LSM9DS1M_OFFSET_X_REG_H_M   0x06
#define LSM9DS1M_OFFSET_Y_REG_L_M   0x07
#define LSM9DS1M_OFFSET_Y_REG_H_M   0x08
#define LSM9DS1M_OFFSET_Z_REG_L_M   0x09
#define LSM9DS1M_OFFSET_Z_REG_H_M   0x0A

#define LSM9DS1M_WHO_AM_I           0x0F
    #define WHO_AM_I_MAG            0x3D

#define LSM9DS1M_CTRL_REG1_M        0x20
    #define LSM9DS1M_TEMP_COMP      (0x1 << 7)
    #define LSM9DS1M_XY_ULTRA_HIGH  (0x3 << 5)
    #define LSM9DS1M_80HZ           (0x7 << 2)
    #define LSM9DS1M_FAST_ODR       (0x1 << 1)

#define LSM9DS1M_CTRL_REG2_M        0x21
    #define LSM9DS1M_FS_16G         (0x3 << 5)

#define LSM9DS1M_CTRL_REG3_M        0x22
    #define LSM9DS1M_SPI_ENABLE      (0x01 << 2)
    #define LSM9DS1M_CONTINUOUS_MODE 0x0

#define LSM9DS1M_CTRL_REG4_M        0x23
    #define LSM9DS1M_Z_ULTRA_HIGH  (0x3 << 2)

#define LSM9DS1M_CTRL_REG5_M        0x24
    #define LSM9DS1M_BDU            (0x0 << 6)

#define LSM9DS1M_STATUS_REG_M       0x27

#define LSM9DS1M_OUT_X_L_M          0x28
#define LSM9DS1M_OUT_X_H_M          0x29
#define LSM9DS1M_OUT_Y_L_M          0x2A
#define LSM9DS1M_OUT_Y_H_M          0x2B
#define LSM9DS1M_OUT_Z_L_M          0x2C
#define LSM9DS1M_OUT_Z_H_M          0x2D
#define LSM9DS1M_INT_CFG_M          0x30
#define LSM9DS1M_INT_SRC_M          0x31
#define LSM9DS1M_INT_THS_L_M        0x32
#define LSM9DS1M_INT_THS_H_M        0x33

struct PACKED sample_regs {
    uint8_t status;
    int16_t val[3];
};

extern const AP_HAL::HAL &hal;

AP_Compass_Backend *AP_Compass_LSM9DS1::probe(Compass &compass,
                                              AP_HAL::OwnPtr<AP_HAL::Device> dev)
{
    AP_Compass_LSM9DS1 *sensor = new AP_Compass_LSM9DS1(compass, std::move(dev), AP_COMPASS_TYPE_LSM9DS1);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

bool AP_Compass_LSM9DS1::init()
{
    _last_update_timestamp = AP_HAL::micros();

    hal.scheduler->suspend_timer_procs();
    AP_HAL::Semaphore *bus_sem = _dev->get_semaphore();

    if (!bus_sem || !bus_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        hal.console->printf("LSM9DS1: Unable to get bus semaphore\n");
        goto fail_sem;
    }

    if (!_check_id()) {
        hal.console->printf("LSM9DS1: Could not check id\n");
        goto errout;
    }

    if (!_configure()) {
        hal.console->printf("LSM9DS1: Could not check id\n");
        goto errout;
    }

    if (!_set_scale()) {
        hal.console->printf("LSM9DS1: Could not set scale\n");
        goto errout;
    }

    _compass_instance = register_compass();
    set_dev_id(_compass_instance, _dev_id);

    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Compass_LSM9DS1::_update, void));

    hal.scheduler->resume_timer_procs();
    bus_sem->give();

    return true;

errout:
    bus_sem->give();

fail_sem:
    hal.scheduler->resume_timer_procs();

    return false;
}

void AP_Compass_LSM9DS1::_dump_registers()
{
    hal.console->println("LSMDS1 registers");
    for (uint8_t reg = LSM9DS1M_OFFSET_X_REG_L_M; reg <= LSM9DS1M_INT_THS_H_M; reg++) {
        uint8_t v = _register_read(reg);
        hal.console->printf("%02x:%02x ", (unsigned)reg, (unsigned)v);
        if ((reg - (LSM9DS1M_OFFSET_X_REG_L_M-1)) % 16 == 0) {
            hal.console->println();
        }
    }
    hal.console->println();
}

void AP_Compass_LSM9DS1::_update(void)
{
    struct sample_regs regs;
    Vector3f raw_field;
    uint32_t time_us = AP_HAL::micros();

    if (AP_HAL::micros() - _last_update_timestamp < 10000) {
        goto end;
    }

    if (!_dev->get_semaphore()->take_nonblocking()) {
        goto end;
    }

    if (!_block_read(LSM9DS1M_STATUS_REG_M, (uint8_t *) &regs, sizeof(regs))) {
        goto fail;
    }

    if (regs.status & 0x80) {
        goto fail;
    }

    raw_field = Vector3f(regs.val[0], regs.val[1], regs.val[2]);

    if (is_zero(raw_field.x) && is_zero(raw_field.y) && is_zero(raw_field.z)) {
        goto fail;
    }

    raw_field *= _scaling;

    // rotate raw_field from sensor frame to body frame
    rotate_field(raw_field, _compass_instance);

    // publish raw_field (uncorrected point sample) for calibration use
    publish_raw_field(raw_field, time_us, _compass_instance);

    // correct raw_field for known errors
    correct_field(raw_field, _compass_instance);

    _mag_x_accum += raw_field.x;
    _mag_y_accum += raw_field.y;
    _mag_z_accum += raw_field.z;
    _accum_count++;
    if (_accum_count == 10) {
        _mag_x_accum /= 2;
        _mag_y_accum /= 2;
        _mag_z_accum /= 2;
        _accum_count = 5;
    }

    _last_update_timestamp = AP_HAL::micros();

fail:
    _dev->get_semaphore()->give();
end:
    return;
}

void AP_Compass_LSM9DS1::read()
{
    if (_accum_count == 0) {
        /* We're not ready to publish*/
        return;
    }

    hal.scheduler->suspend_timer_procs();

    auto field = _get_filtered_field();
    _reset_filter();

    hal.scheduler->resume_timer_procs();

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO2
    field.rotate(ROTATION_ROLL_180);
#endif

    publish_filtered_field(field, _compass_instance);

}

void AP_Compass_LSM9DS1::_reset_filter()
{
    _mag_x_accum = _mag_y_accum = _mag_z_accum = 0;
    _accum_count = 0;
}

Vector3f AP_Compass_LSM9DS1::_get_filtered_field() const
{
    Vector3f field(_mag_x_accum, _mag_y_accum, _mag_z_accum);
    field /= _accum_count;

    return field;
}


bool AP_Compass_LSM9DS1::_check_id(void)
{
    // initially run the bus at low speed
    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    uint8_t value = _register_read(LSM9DS1M_WHO_AM_I);
    if (value != WHO_AM_I_MAG) {
        hal.console->printf("LSM9DS1: unexpected WHOAMI 0x%x\n", (unsigned)value);
        return false;
    }

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    return true;
}

bool AP_Compass_LSM9DS1::_configure(void)
{
    _register_write(LSM9DS1M_CTRL_REG1_M, LSM9DS1M_TEMP_COMP | LSM9DS1M_XY_ULTRA_HIGH | LSM9DS1M_80HZ | LSM9DS1M_FAST_ODR);
    _register_write(LSM9DS1M_CTRL_REG2_M, LSM9DS1M_FS_16G);
    _register_write(LSM9DS1M_CTRL_REG3_M, LSM9DS1M_CONTINUOUS_MODE);
    _register_write(LSM9DS1M_CTRL_REG4_M, LSM9DS1M_Z_ULTRA_HIGH);
    _register_write(LSM9DS1M_CTRL_REG5_M, LSM9DS1M_BDU);

    return true;
}

bool AP_Compass_LSM9DS1::_set_scale(void)
{
    static const uint8_t FS_M_MASK = 0xc;
    _register_modify(LSM9DS1M_CTRL_REG2_M, FS_M_MASK, LSM9DS1M_FS_16G);
    _scaling = 0.58f;

    return true;
}

AP_Compass_LSM9DS1::AP_Compass_LSM9DS1(Compass &compass, AP_HAL::OwnPtr<AP_HAL::Device> dev, uint32_t dev_id)
    : AP_Compass_Backend(compass)
    , _dev(std::move(dev))
    , _dev_id(dev_id)
{
    _reset_filter();
}

uint8_t AP_Compass_LSM9DS1::_register_read(uint8_t reg)
{
    uint8_t val = 0;

    /* set READ bit */
    reg |= 0x80;
    _dev->read_registers(reg, &val, 1);

    return val;
}

bool AP_Compass_LSM9DS1::_block_read(uint8_t reg, uint8_t *buf, uint32_t size)
{
    /* set !MS bit */
    reg |= 0xc0;
    return _dev->read_registers(reg, buf, size);
}

void AP_Compass_LSM9DS1::_register_write(uint8_t reg, uint8_t val)
{
    _dev->write_register(reg, val);
}

void AP_Compass_LSM9DS1::_register_modify(uint8_t reg, uint8_t clearbits, uint8_t setbits)
{
    uint8_t val;

    val = _register_read(reg);
    val &= ~clearbits;
    val |= setbits;
    _register_write(reg, val);
}
