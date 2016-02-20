/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>

#include "Compass.h"
#include "AP_Compass_Backend.h"

class AuxiliaryBus;
class AuxiliaryBusSlave;
class AP_InertialSensor;

class AP_AK8963_SerialBus
{
public:
    struct PACKED raw_value {
        int16_t val[3];
        uint8_t st2;
    };

    virtual ~AP_AK8963_SerialBus() { }
    virtual void register_read(uint8_t reg, uint8_t *value, uint8_t count) = 0;
    uint8_t register_read(uint8_t reg) {
        uint8_t value;
        register_read(reg, &value, 1);
        return value;
    }
    virtual void register_write(uint8_t reg, uint8_t value) = 0;
    virtual AP_HAL::Semaphore* get_semaphore() = 0;
    virtual bool start_measurements() = 0;
    virtual void read_raw(struct raw_value *rv) = 0;
    virtual uint32_t get_dev_id() = 0;
};

class AP_Compass_AK8963 : public AP_Compass_Backend
{
public:
    static AP_Compass_Backend *detect_mpu9250(Compass &compass, uint8_t mpu9250_instance);
    static AP_Compass_Backend *detect_mpu9250_i2c(Compass &compass,
                                                  AP_HAL::I2CDriver *i2c,
                                                  uint8_t addr);
    static AP_Compass_Backend *detect_i2c(Compass &compass,
                                          AP_HAL::I2CDriver *i2c,
                                          uint8_t addr);

    AP_Compass_AK8963(Compass &compass, AP_AK8963_SerialBus *bus);
    ~AP_Compass_AK8963();

    bool        init(void);
    void        read(void);
    void        accumulate(void);

private:
    static AP_Compass_Backend *_detect(Compass &compass, AP_AK8963_SerialBus *bus);

    void _make_factory_sensitivity_adjustment(Vector3f& field) const;
    void _make_adc_sensitivity_adjustment(Vector3f& field) const;
    Vector3f _get_filtered_field() const;
    void _reset_filter();

    bool _reset();
    bool _setup_mode();
    bool _check_id();
    bool _calibrate();

    void _update();
    void _dump_registers();

    float               _magnetometer_ASA[3] {0, 0, 0};
    uint8_t             _compass_instance;

    float               _mag_x_accum;
    float               _mag_y_accum;
    float               _mag_z_accum;
    uint32_t            _accum_count;

    bool                _initialized;
    uint32_t            _last_update_timestamp;
    uint32_t            _last_accum_time;
    bool                _timesliced;

    AP_AK8963_SerialBus *_bus = nullptr;
    AP_HAL::Semaphore *_bus_sem;
};

class AP_AK8963_SerialBus_MPU9250: public AP_AK8963_SerialBus
{
public:
    AP_AK8963_SerialBus_MPU9250(AP_InertialSensor &ins, uint8_t addr, uint8_t mpu9250_instance);
    ~AP_AK8963_SerialBus_MPU9250();
    void register_read(uint8_t reg, uint8_t *value, uint8_t count);
    void register_write(uint8_t reg, uint8_t value);
    AP_HAL::Semaphore* get_semaphore();
    bool start_measurements();
    void read_raw(struct raw_value *rv);
    uint32_t get_dev_id();
private:
    AuxiliaryBus *_bus = nullptr;
    AuxiliaryBusSlave *_slave = nullptr;
    bool _started;
};

class AP_AK8963_SerialBus_I2C: public AP_AK8963_SerialBus
{
public:
    AP_AK8963_SerialBus_I2C(AP_HAL::I2CDriver *i2c, uint8_t addr);
    void register_read(uint8_t reg, uint8_t *value, uint8_t count);
    void register_write(uint8_t reg, uint8_t value);
    AP_HAL::Semaphore* get_semaphore();
    bool start_measurements() { return true; }
    void read_raw(struct raw_value *rv);
    uint32_t get_dev_id();
private:
    void _read(uint8_t reg, uint8_t *value, uint32_t count);
    void _write(uint8_t reg, const uint8_t *value,  uint32_t count);
    void _write(uint8_t reg, const uint8_t value) {
        _write(reg, &value, 1);
    }
    AP_HAL::I2CDriver *_i2c;
    uint8_t _addr;
};
