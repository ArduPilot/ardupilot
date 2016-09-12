/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

class AuxiliaryBus;
class AuxiliaryBusSlave;
class AP_InertialSensor;
class AP_HMC5843_BusDriver;

class AP_Compass_HMC5843 : public AP_Compass_Backend
{
public:
    static AP_Compass_Backend *probe(Compass &compass,
                                     AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                     bool force_external = false);

    static AP_Compass_Backend *probe_mpu6000(Compass &compass);

    static constexpr const char *name = "HMC5843";

    virtual ~AP_Compass_HMC5843();

    bool init() override;
    void read() override;
    void accumulate() override;

private:
    AP_Compass_HMC5843(Compass &compass, AP_HMC5843_BusDriver *bus,
                       bool force_external);

    bool _detect_version();
    bool _calibrate();
    bool _setup_sampling_mode();

    /* Read a single sample */
    bool _read_sample();

    AP_HMC5843_BusDriver *_bus;

    float _scaling[3];
    float _gain_scale;
    uint32_t _last_accum_time;

    // when unhealthy the millis() value to retry at
    uint32_t _retry_time;

    int16_t _mag_x;
    int16_t _mag_y;
    int16_t _mag_z;
    int16_t _mag_x_accum;
    int16_t _mag_y_accum;
    int16_t _mag_z_accum;
    uint8_t _accum_count;

    uint8_t _base_config;
    uint8_t _compass_instance;
    uint8_t _gain_config;
    uint8_t _product_id;

    bool _initialised;
    bool _force_external;
};

class AP_HMC5843_BusDriver
{
public:
    virtual ~AP_HMC5843_BusDriver() { }

    virtual bool block_read(uint8_t reg, uint8_t *buf, uint32_t size) = 0;
    virtual bool register_read(uint8_t reg, uint8_t *val) = 0;
    virtual bool register_write(uint8_t reg, uint8_t val) = 0;

    virtual AP_HAL::Semaphore *get_semaphore() = 0;

    virtual bool configure() { return true; }
    virtual bool start_measurements() { return true; }
};

class AP_HMC5843_BusDriver_HALDevice : public AP_HMC5843_BusDriver
{
public:
    AP_HMC5843_BusDriver_HALDevice(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    bool block_read(uint8_t reg, uint8_t *buf, uint32_t size) override;
    bool register_read(uint8_t reg, uint8_t *val) override;
    bool register_write(uint8_t reg, uint8_t val) override;

    AP_HAL::Semaphore *get_semaphore() override;

private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
};

class AP_HMC5843_BusDriver_Auxiliary : public AP_HMC5843_BusDriver
{
public:
    AP_HMC5843_BusDriver_Auxiliary(AP_InertialSensor &ins, uint8_t backend_id,
                                   uint8_t addr);
    virtual ~AP_HMC5843_BusDriver_Auxiliary();

    bool block_read(uint8_t reg, uint8_t *buf, uint32_t size) override;
    bool register_read(uint8_t reg, uint8_t *val) override;
    bool register_write(uint8_t reg, uint8_t val) override;

    AP_HAL::Semaphore *get_semaphore() override;

    bool configure() override;
    bool start_measurements() override;

private:
    AuxiliaryBus *_bus;
    AuxiliaryBusSlave *_slave;
    bool _started;
};
