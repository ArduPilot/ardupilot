#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_HMC5843_ENABLED

#ifndef HAL_COMPASS_HMC5843_I2C_ADDR
#define HAL_COMPASS_HMC5843_I2C_ADDR 0x1E
#endif

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>

#include "AP_Compass_Backend.h"
#include <AP_InertialSensor/AP_InertialSensor_config.h>

class AuxiliaryBus;
class AuxiliaryBusSlave;
class AP_InertialSensor;
class AP_HMC5843_BusDriver;

class AP_Compass_HMC5843 : public AP_Compass_Backend
{
public:
    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                     bool force_external,
                                     enum Rotation rotation);

#if AP_INERTIALSENSOR_ENABLED
    static AP_Compass_Backend *probe_mpu6000(enum Rotation rotation);
#endif

    static constexpr const char *name = "HMC5843";

    virtual ~AP_Compass_HMC5843();

    void read() override;

private:
    AP_Compass_HMC5843(AP_HMC5843_BusDriver *bus,
                       bool force_external, enum Rotation rotation);

    bool init();
    bool _check_whoami();
    bool _calibrate();
    bool _setup_sampling_mode();

    void _timer();

    /* Read a single sample */
    bool _read_sample();

    // ask for a new sample
    void _take_sample();

    AP_HMC5843_BusDriver *_bus;

    Vector3f _scaling;
    float _gain_scale;

    int16_t _mag_x;
    int16_t _mag_y;
    int16_t _mag_z;

    uint8_t _compass_instance;

    enum Rotation _rotation;
    
    bool _initialised:1;
    bool _force_external:1;
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

    virtual AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t, AP_HAL::Device::PeriodicCb) = 0;

    // set device type within a device class
    virtual void set_device_type(uint8_t devtype) = 0;

    // return 24 bit bus identifier
    virtual uint32_t get_bus_id(void) const = 0;

    virtual void set_retries(uint8_t retries) {}
};

class AP_HMC5843_BusDriver_HALDevice : public AP_HMC5843_BusDriver
{
public:
    AP_HMC5843_BusDriver_HALDevice(AP_HAL::OwnPtr<AP_HAL::Device> dev);

    bool block_read(uint8_t reg, uint8_t *buf, uint32_t size) override;
    bool register_read(uint8_t reg, uint8_t *val) override;
    bool register_write(uint8_t reg, uint8_t val) override;

    AP_HAL::Semaphore *get_semaphore() override;

    AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb) override;

    // set device type within a device class
    void set_device_type(uint8_t devtype) override {
        _dev->set_device_type(devtype);
    }

    // return 24 bit bus identifier
    uint32_t get_bus_id(void) const override {
        return _dev->get_bus_id();
    }

    void set_retries(uint8_t retries) override {
        return _dev->set_retries(retries);
    }
    
private:
    AP_HAL::OwnPtr<AP_HAL::Device> _dev;
};

#if AP_INERTIALSENSOR_ENABLED
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

    AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb) override;

    // set device type within a device class
    void set_device_type(uint8_t devtype) override;

    // return 24 bit bus identifier
    uint32_t get_bus_id(void) const override;
    
private:
    AuxiliaryBus *_bus;
    AuxiliaryBusSlave *_slave;
    bool _started;
};
#endif  // AP_INERTIALSENSOR_ENABLED

#endif // AP_COMPASS_HMC5843_ENABLED
