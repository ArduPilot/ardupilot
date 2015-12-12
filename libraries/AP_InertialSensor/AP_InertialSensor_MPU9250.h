/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_MPU9250_H__
#define __AP_INERTIAL_SENSOR_MPU9250_H__

#include <stdint.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <Filter/Filter.h>
#include <Filter/LowPassFilter2p.h>
#include "AP_InertialSensor.h"
#include "AuxiliaryBus.h"

class AP_MPU9250_AuxiliaryBus;
class AP_MPU9250_AuxiliaryBusSlave;

// enable debug to see a register dump on startup
#define MPU9250_DEBUG 0

class AP_MPU9250_BusDriver
{
public:
    virtual ~AP_MPU9250_BusDriver() { };
    virtual void init() = 0;
    virtual void read8(uint8_t reg, uint8_t *val) = 0;
    virtual void write8(uint8_t reg, uint8_t val) = 0;
    virtual void read_block(uint8_t reg, uint8_t *val, uint8_t count) = 0;
    virtual void set_bus_speed(AP_HAL::SPIDeviceDriver::bus_speed speed) = 0;
    virtual bool read_data_transaction(uint8_t* samples,
                                       uint8_t &n_samples) = 0;
    virtual AP_HAL::Semaphore* get_semaphore() = 0;
    virtual bool has_auxiliary_bus() = 0;
};

class AP_InertialSensor_MPU9250 : public AP_InertialSensor_Backend
{
    friend AP_MPU9250_AuxiliaryBus;
    friend AP_MPU9250_AuxiliaryBusSlave;

public:

    AP_InertialSensor_MPU9250(AP_InertialSensor &imu, AP_MPU9250_BusDriver *bus);

    /* update accel and gyro state */
    bool update();

    AuxiliaryBus *get_auxiliary_bus();

    static AP_InertialSensor_MPU9250 &from(AP_InertialSensor_Backend &backend) {
        return static_cast<AP_InertialSensor_MPU9250&>(backend);
    }

    // detect the sensor
    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu, AP_HAL::SPIDeviceDriver *spi);
    static AP_InertialSensor_Backend *detect_i2c(AP_InertialSensor &imu,
                                                 AP_HAL::I2CDriver *i2c,
                                                 uint8_t addr);

private:
    static AP_InertialSensor_Backend *_detect(AP_InertialSensor &_imu,
                                              AP_MPU9250_BusDriver *bus,
                                              int16_t id);

    bool                 _init_sensor();
    void                 _read_data_transaction();
    bool                 _data_ready();
    void                 _poll_data(void);
    uint8_t              _register_read( uint8_t reg );
    void                 _register_write( uint8_t reg, uint8_t val );
    bool                 _hardware_init(void);

    AP_MPU9250_BusDriver *_bus;
    AP_HAL::Semaphore *_bus_sem;
    AP_MPU9250_AuxiliaryBus *_auxiliar_bus = nullptr;

    // gyro and accel instances
    uint8_t _gyro_instance;
    uint8_t _accel_instance;

    // The default rotation for the IMU, its value depends on how the IMU is
    // placed by default on the system
    enum Rotation _default_rotation;

#if MPU9250_DEBUG
    static void _dump_registers();
#endif
};

class AP_MPU9250_BusDriver_SPI : public AP_MPU9250_BusDriver
{
public:
    AP_MPU9250_BusDriver_SPI(AP_HAL::SPIDeviceDriver *spi);
    void init();
    void read8(uint8_t reg, uint8_t *val);
    void write8(uint8_t reg, uint8_t val);
    void read_block(uint8_t reg, uint8_t *val, uint8_t count);
    void set_bus_speed(AP_HAL::SPIDeviceDriver::bus_speed speed);
    bool read_data_transaction(uint8_t* samples, uint8_t &n_samples);
    AP_HAL::Semaphore* get_semaphore();
    bool has_auxiliary_bus();

private:
    AP_HAL::SPIDeviceDriver *_spi;
};

class AP_MPU9250_BusDriver_I2C : public AP_MPU9250_BusDriver
{
public:
    AP_MPU9250_BusDriver_I2C(AP_HAL::I2CDriver *i2c, uint8_t addr);
    void init();
    void read8(uint8_t reg, uint8_t *val);
    void write8(uint8_t reg, uint8_t val);
    void read_block(uint8_t reg, uint8_t *val, uint8_t count);
    void set_bus_speed(AP_HAL::SPIDeviceDriver::bus_speed speed) {};
    bool read_data_transaction(uint8_t* samples, uint8_t &n_samples);
    AP_HAL::Semaphore* get_semaphore();
    bool has_auxiliary_bus();

private:
    uint8_t _addr;
    AP_HAL::I2CDriver *_i2c;
};

class AP_MPU9250_AuxiliaryBus : public AuxiliaryBus
{
    friend class AP_InertialSensor_MPU9250;

public:
    AP_HAL::Semaphore *get_semaphore() override;

protected:
    AP_MPU9250_AuxiliaryBus(AP_InertialSensor_MPU9250 &backend);

    AuxiliaryBusSlave *_instantiate_slave(uint8_t addr, uint8_t instance);
    int _configure_periodic_read(AuxiliaryBusSlave *slave, uint8_t reg,
                                 uint8_t size);

private:
    void _configure_slaves();

    static const uint8_t MAX_EXT_SENS_DATA = 24;
    uint8_t _ext_sens_data = 0;
};

class AP_MPU9250_AuxiliaryBusSlave : public AuxiliaryBusSlave
{
    friend class AP_MPU9250_AuxiliaryBus;

public:
    int passthrough_read(uint8_t reg, uint8_t *buf, uint8_t size) override;
    int passthrough_write(uint8_t reg, uint8_t val) override;

    int read(uint8_t *buf) override;

protected:
    AP_MPU9250_AuxiliaryBusSlave(AuxiliaryBus &bus, uint8_t addr, uint8_t instance);
    int _set_passthrough(uint8_t reg, uint8_t size, uint8_t *out = nullptr);

private:
    const uint8_t _mpu9250_addr;
    const uint8_t _mpu9250_reg;
    const uint8_t _mpu9250_ctrl;
    const uint8_t _mpu9250_do;

    uint8_t _ext_sens_data = 0;
};

#endif // __AP_INERTIAL_SENSOR_MPU9250_H__
