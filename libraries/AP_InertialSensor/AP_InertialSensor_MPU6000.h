/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include <stdint.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/SPIDevice.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <AP_Math/AP_Math.h>
#include <Filter/Filter.h>
#include <Filter/LowPassFilter.h>
#include <Filter/LowPassFilter2p.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"
#include "AuxiliaryBus.h"

// enable debug to see a register dump on startup
#define MPU6000_DEBUG 0

class AP_MPU6000_AuxiliaryBus;
class AP_MPU6000_AuxiliaryBusSlave;

class AP_InertialSensor_MPU6000 : public AP_InertialSensor_Backend
{
    friend AP_MPU6000_AuxiliaryBus;
    friend AP_MPU6000_AuxiliaryBusSlave;

public:
    virtual ~AP_InertialSensor_MPU6000();

    static AP_InertialSensor_MPU6000 &from(AP_InertialSensor_Backend &backend) {
        return static_cast<AP_InertialSensor_MPU6000&>(backend);
    }

    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev);

    /* update accel and gyro state */
    bool update();

    /*
     * Return an AuxiliaryBus if the bus driver allows it
     */
    AuxiliaryBus *get_auxiliary_bus() override;

    void start() override;

private:
    AP_InertialSensor_MPU6000(AP_InertialSensor &imu,
                              AP_HAL::OwnPtr<AP_HAL::Device> dev,
                              bool use_fifo);

#if MPU6000_DEBUG
    void _dump_registers();
#endif

    /* Initialize sensor*/
    bool _init();
    bool _hardware_init();

    void _set_filter_register(uint16_t filter_hz);
    void _fifo_reset();
    void _fifo_enable();
    bool _has_auxiliary_bus();

    /* Read samples from FIFO (FIFO enabled) */
    void _read_fifo();

    /* Read a single sample (FIFO disabled) */
    void _read_sample();

    /* Check if there's data available by either reading DRDY pin or register */
    bool _data_ready();

    /* Poll for new data (non-blocking) */
    void _poll_data();

    /* Read and write functions taking the differences between buses into
     * account */
    bool _block_read(uint8_t reg, uint8_t *buf, uint32_t size);
    uint8_t _register_read(uint8_t reg);
    void _register_write(uint8_t reg, uint8_t val );

    void _accumulate(uint8_t *samples, uint8_t n_samples);

    // instance numbers of accel and gyro data
    uint8_t _gyro_instance;
    uint8_t _accel_instance;

    const bool _use_fifo;

    uint16_t _error_count;

    float _temp_filtered;
    LowPassFilter2pFloat _temp_filter;

    AP_HAL::DigitalSource *_drdy_pin;
    AP_HAL::OwnPtr<AP_HAL::Device> _dev;
    AP_MPU6000_AuxiliaryBus *_auxiliary_bus;
};

class AP_MPU6000_AuxiliaryBusSlave : public AuxiliaryBusSlave
{
    friend class AP_MPU6000_AuxiliaryBus;

public:
    int passthrough_read(uint8_t reg, uint8_t *buf, uint8_t size) override;
    int passthrough_write(uint8_t reg, uint8_t val) override;

    int read(uint8_t *buf) override;

protected:
    AP_MPU6000_AuxiliaryBusSlave(AuxiliaryBus &bus, uint8_t addr, uint8_t instance);
    int _set_passthrough(uint8_t reg, uint8_t size, uint8_t *out = nullptr);

private:
    const uint8_t _mpu6000_addr;
    const uint8_t _mpu6000_reg;
    const uint8_t _mpu6000_ctrl;
    const uint8_t _mpu6000_do;

    uint8_t _ext_sens_data = 0;
};

class AP_MPU6000_AuxiliaryBus : public AuxiliaryBus
{
    friend class AP_InertialSensor_MPU6000;

public:
    AP_HAL::Semaphore *get_semaphore() override;

protected:
    AP_MPU6000_AuxiliaryBus(AP_InertialSensor_MPU6000 &backend);

    AuxiliaryBusSlave *_instantiate_slave(uint8_t addr, uint8_t instance) override;
    int _configure_periodic_read(AuxiliaryBusSlave *slave, uint8_t reg,
                                 uint8_t size) override;

private:
    void _configure_slaves();

    static const uint8_t MAX_EXT_SENS_DATA = 24;
    uint8_t _ext_sens_data = 0;
};
