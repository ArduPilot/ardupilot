/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/SPIDevice.h>
#include <AP_Math/AP_Math.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

class AuxiliaryBus;
class AuxiliaryBusSlave;
class AP_InertialSensor;
class AP_AK8963_BusDriver;

class AP_Compass_AK8963 : public AP_Compass_Backend
{
public:
    /* Probe for AK8963 standalone on I2C bus */
    static AP_Compass_Backend *probe(Compass &compass,
                                     AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    /* Probe for AK8963 on auxiliary bus of MPU9250, connected through I2C */
    static AP_Compass_Backend *probe_mpu9250(Compass &compass,
                                             AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    /* Probe for AK8963 on auxiliary bus of MPU9250, connected through SPI */
    static AP_Compass_Backend *probe_mpu9250(Compass &compass, uint8_t mpu9250_instance);

    static constexpr const char *name = "AK8963";

    virtual ~AP_Compass_AK8963();

    bool init() override;
    void read() override;

private:
    AP_Compass_AK8963(Compass &compass, AP_AK8963_BusDriver *bus,
                      uint32_t dev_id);

    void _make_factory_sensitivity_adjustment(Vector3f &field) const;
    void _make_adc_sensitivity_adjustment(Vector3f &field) const;
    Vector3f _get_filtered_field() const;

    void _reset_filter();
    bool _reset();
    bool _setup_mode();
    bool _check_id();
    bool _calibrate();

    void _update();

    AP_AK8963_BusDriver *_bus;

    float _magnetometer_ASA[3] {0, 0, 0};
    float _mag_x_accum;
    float _mag_y_accum;
    float _mag_z_accum;
    uint32_t _accum_count;
    uint32_t _last_update_timestamp;
    uint32_t _dev_id;

    uint8_t _compass_instance;
    bool _initialized;
    bool _timesliced;
};

class AP_AK8963_BusDriver
{
public:
    virtual ~AP_AK8963_BusDriver() { }

    virtual bool block_read(uint8_t reg, uint8_t *buf, uint32_t size) = 0;
    virtual bool register_read(uint8_t reg, uint8_t *val) = 0;
    virtual bool register_write(uint8_t reg, uint8_t val) = 0;

    virtual AP_HAL::Semaphore  *get_semaphore() = 0;

    virtual bool configure() { return true; }
    virtual bool start_measurements() { return true; }
};

class AP_AK8963_BusDriver_HALDevice: public AP_AK8963_BusDriver
{
public:
    AP_AK8963_BusDriver_HALDevice(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    virtual bool block_read(uint8_t reg, uint8_t *buf, uint32_t size) override;
    virtual bool register_read(uint8_t reg, uint8_t *val) override;
    virtual bool register_write(uint8_t reg, uint8_t val) override;

    virtual AP_HAL::Semaphore  *get_semaphore() override;

private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
};

class AP_AK8963_BusDriver_Auxiliary : public AP_AK8963_BusDriver
{
public:
    AP_AK8963_BusDriver_Auxiliary(AP_InertialSensor &ins, uint8_t backend_id,
                                  uint8_t backend_instance, uint8_t addr);
    ~AP_AK8963_BusDriver_Auxiliary();

    bool block_read(uint8_t reg, uint8_t *buf, uint32_t size) override;
    bool register_read(uint8_t reg, uint8_t *val) override;
    bool register_write(uint8_t reg, uint8_t val) override;

    AP_HAL::Semaphore  *get_semaphore() override;

    bool configure();
    bool start_measurements();

private:
    AuxiliaryBus *_bus;
    AuxiliaryBusSlave *_slave;
    bool _started;
};
