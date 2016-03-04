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
class AP_HMC5843_SerialBus;

class AP_Compass_HMC5843 : public AP_Compass_Backend
{
private:
    static AP_Compass_Backend *_detect(Compass &compass,
                                       AP_HMC5843_SerialBus *bus);

    AP_HMC5843_SerialBus *_bus;
    AP_HAL::Semaphore *_bus_sem = nullptr;

    float               _scaling[3] = {0};
    bool                _initialised;
    bool                read_raw(void);
    uint8_t             _base_config;
    bool                re_initialise(void);
    bool                read_register(uint8_t address, uint8_t *value);
    bool                write_register(uint8_t address, uint8_t value);

    bool                _calibrate(uint8_t calibration_gain,
                                   uint16_t expected_x,
                                   uint16_t expected_yz);
    bool                _detect_version();

    uint32_t            _retry_time; // when unhealthy the millis() value to retry at

    int16_t			    _mag_x;
    int16_t			    _mag_y;
    int16_t			    _mag_z;
    int16_t             _mag_x_accum;
    int16_t             _mag_y_accum;
    int16_t             _mag_z_accum;
    uint8_t			    _accum_count;
    uint32_t            _last_accum_time;

    uint8_t             _compass_instance;
    uint8_t             _product_id;

    float               _gain_multiple;
public:
    // detect the sensor
    static AP_Compass_Backend *detect_i2c(Compass &compass,
                                          AP_HAL::I2CDriver *i2c);
    static AP_Compass_Backend *detect_mpu6000(Compass &compass);

    AP_Compass_HMC5843(Compass &compass, AP_HMC5843_SerialBus *bus);
    ~AP_Compass_HMC5843();

    bool        init(void);
    void        read(void);
    void        accumulate(void);
};

class AP_HMC5843_SerialBus
{
public:
    struct PACKED raw_value {
        uint8_t val[6];
    };

    virtual ~AP_HMC5843_SerialBus() { };
    virtual void set_high_speed(bool val) = 0;
    virtual uint8_t register_read(uint8_t reg, uint8_t *buf, uint8_t size) = 0;
    uint8_t register_read(uint8_t reg, uint8_t *val)
    {
        return register_read(reg, val, 1);
    }

    virtual uint8_t register_write(uint8_t reg, uint8_t val) = 0;
    virtual AP_HAL::Semaphore* get_semaphore() = 0;
    virtual uint8_t read_raw(struct raw_value *rv) = 0;
    virtual bool configure() { return true; }
    virtual bool start_measurements() { return true; }
};

class AP_HMC5843_SerialBus_I2C : public AP_HMC5843_SerialBus
{
public:
    AP_HMC5843_SerialBus_I2C(AP_HAL::I2CDriver *i2c, uint8_t addr);
    void set_high_speed(bool val) override;
    uint8_t register_read(uint8_t reg, uint8_t *buf, uint8_t size) override;
    uint8_t register_write(uint8_t reg, uint8_t val) override;
    AP_HAL::Semaphore* get_semaphore() override;
    uint8_t read_raw(struct raw_value *rv) override;

private:
    AP_HAL::I2CDriver *_i2c;
    uint8_t _addr;
};

class AP_HMC5843_SerialBus_MPU6000 : public AP_HMC5843_SerialBus
{
public:
    AP_HMC5843_SerialBus_MPU6000(AP_InertialSensor &ins, uint8_t addr);
    ~AP_HMC5843_SerialBus_MPU6000();
    void set_high_speed(bool val) override;
    uint8_t register_read(uint8_t reg, uint8_t *buf, uint8_t size) override;
    uint8_t register_write(uint8_t reg, uint8_t val) override;
    AP_HAL::Semaphore* get_semaphore() override;
    uint8_t read_raw(struct raw_value *rv) override;
    bool configure() override;
    bool start_measurements() override;

private:
    AuxiliaryBus *_bus = nullptr;
    AuxiliaryBusSlave *_slave = nullptr;
    bool _started = false;
};
