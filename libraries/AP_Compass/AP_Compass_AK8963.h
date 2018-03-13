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
                                     AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                     enum Rotation rotation = ROTATION_NONE);

    /* Probe for AK8963 on auxiliary bus of MPU9250, connected through I2C */
    static AP_Compass_Backend *probe_mpu9250(Compass &compass,
                                             AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                             enum Rotation rotation = ROTATION_NONE);

    /* Probe for AK8963 on auxiliary bus of MPU9250, connected through SPI */
    static AP_Compass_Backend *probe_mpu9250(Compass &compass, uint8_t mpu9250_instance,
                                             enum Rotation rotation = ROTATION_NONE);

    static constexpr const char *name = "AK8963";

    virtual ~AP_Compass_AK8963();

    void read() override;

private:
    AP_Compass_AK8963(Compass &compass, AP_AK8963_BusDriver *bus,
                      enum Rotation rotation = ROTATION_NONE);

    bool init();
    void _make_factory_sensitivity_adjustment(Vector3f &field) const;
    void _make_adc_sensitivity_adjustment(Vector3f &field) const;

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

    uint8_t _compass_instance;
    bool _initialized;
    enum Rotation _rotation;
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
    virtual AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t, AP_HAL::Device::PeriodicCb) = 0;

    // set device type within a device class
    virtual void set_device_type(uint8_t devtype) = 0;

    // return 24 bit bus identifier
    virtual uint32_t get_bus_id(void) const = 0;
};

class AP_AK8963_BusDriver_HALDevice: public AP_AK8963_BusDriver
{
public:
    AP_AK8963_BusDriver_HALDevice(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    virtual bool block_read(uint8_t reg, uint8_t *buf, uint32_t size) override;
    virtual bool register_read(uint8_t reg, uint8_t *val) override;
    virtual bool register_write(uint8_t reg, uint8_t val) override;

    virtual AP_HAL::Semaphore  *get_semaphore() override;
    AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb) override;

    // set device type within a device class
    void set_device_type(uint8_t devtype) override {
        _dev->set_device_type(devtype);
    }

    // return 24 bit bus identifier
    uint32_t get_bus_id(void) const override {
        return _dev->get_bus_id();
    }
    
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

    AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb) override;
    
    AP_HAL::Semaphore  *get_semaphore() override;

    bool configure();
    bool start_measurements();

    // set device type within a device class
    void set_device_type(uint8_t devtype) override;

    // return 24 bit bus identifier
    uint32_t get_bus_id(void) const override;
    
private:
    AuxiliaryBus *_bus;
    AuxiliaryBusSlave *_slave;
    bool _started;
};
