#pragma once
/*
  driver for the invensensev2 range of IMUs
 */

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

class AP_Invensensev2_AuxiliaryBus;
class AP_Invensensev2_AuxiliaryBusSlave;

class AP_InertialSensor_Invensensev2 : public AP_InertialSensor_Backend
{
    friend AP_Invensensev2_AuxiliaryBus;
    friend AP_Invensensev2_AuxiliaryBusSlave;

public:
    virtual ~AP_InertialSensor_Invensensev2();

    static AP_InertialSensor_Invensensev2 &from(AP_InertialSensor_Backend &backend) {
        return static_cast<AP_InertialSensor_Invensensev2&>(backend);
    }

    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                            enum Rotation rotation = ROTATION_NONE);

    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                            enum Rotation rotation = ROTATION_NONE);

    /* update accel and gyro state */
    bool update() override;
    void accumulate() override;

    /*
     * Return an AuxiliaryBus if the bus driver allows it
     */
    AuxiliaryBus *get_auxiliary_bus() override;

    void start() override;

    enum Invensensev2_Type {
        Invensensev2_ICM20948 = 0,
        Invensensev2_ICM20648,
        Invensensev2_ICM20649
    };

    // acclerometers on Invensense sensors will return values up to
    // 24G, but they are not guaranteed to be remotely linear past
    // 16G
    const uint16_t multiplier_accel = INT16_MAX/(26*GRAVITY_MSS);

private:
    AP_InertialSensor_Invensensev2(AP_InertialSensor &imu,
                              AP_HAL::OwnPtr<AP_HAL::Device> dev,
                              enum Rotation rotation);

    /* Initialize sensor*/
    bool _init();
    bool _hardware_init();
    bool _check_whoami();

    void _set_filter_and_scaling(void);
    void _fifo_reset();
    bool _has_auxiliary_bus();

    /* Read samples from FIFO (FIFO enabled) */
    void _read_fifo();

    /* Check if there's data available by either reading DRDY pin or register */
    bool _data_ready();

    /* Poll for new data (non-blocking) */
    void _poll_data();

    /* Read and write functions taking the differences between buses into
     * account */
    bool _block_read(uint16_t reg, uint8_t *buf, uint32_t size);
    uint8_t _register_read(uint16_t reg);
    void _register_write(uint16_t reg, uint8_t val, bool checked=false);
    void _select_bank(uint8_t bank);

    bool _accumulate(uint8_t *samples, uint8_t n_samples);
    bool _accumulate_sensor_rate_sampling(uint8_t *samples, uint8_t n_samples);

    bool _check_raw_temp(int16_t t2);

    int16_t _raw_temp;
    
    // instance numbers of accel and gyro data
    uint8_t _gyro_instance;
    uint8_t _accel_instance;

    float temp_sensitivity = 1.0f/333.87f; // degC/LSB
    float temp_zero = 21; // degC
    
    float _temp_filtered;
    float _accel_scale;
    float _fifo_accel_scale;
    float _fifo_gyro_scale;
    LowPassFilter2pFloat _temp_filter;

    enum Rotation _rotation;

    AP_HAL::DigitalSource *_drdy_pin;
    AP_HAL::OwnPtr<AP_HAL::Device> _dev;
    AP_Invensensev2_AuxiliaryBus *_auxiliary_bus;

    // which sensor type this is
    enum Invensensev2_Type _inv2_type;

    // are we doing more than 1kHz sampling?
    bool _fast_sampling;

    // what downsampling rate are we using from the FIFO?
    uint8_t _fifo_downsample_rate;

    // what rate are we generating samples into the backend?
    uint16_t _backend_rate_hz;

    // Last status from register user control
    uint8_t _last_stat_user_ctrl;    

    // buffer for fifo read
    uint8_t *_fifo_buffer;

    uint8_t _current_bank = 0xFF;
    /*
      accumulators for sensor_rate sampling
      See description in _accumulate_sensor_rate_sampling()
    */
    struct {
        Vector3f accel;
        Vector3f gyro;
        uint8_t count;
        LowPassFilterVector3f accel_filter{4500, 188};
        LowPassFilterVector3f gyro_filter{9000, 188};
    } _accum;
};

class AP_Invensensev2_AuxiliaryBusSlave : public AuxiliaryBusSlave
{
    friend class AP_Invensensev2_AuxiliaryBus;

public:
    int passthrough_read(uint8_t reg, uint8_t *buf, uint8_t size) override;
    int passthrough_write(uint8_t reg, uint8_t val) override;

    int read(uint8_t *buf) override;

protected:
    AP_Invensensev2_AuxiliaryBusSlave(AuxiliaryBus &bus, uint8_t addr, uint8_t instance);
    int _set_passthrough(uint8_t reg, uint8_t size, uint8_t *out = nullptr);

private:
    const uint16_t _inv2_addr;
    const uint16_t _inv2_reg;
    const uint16_t _inv2_ctrl;
    const uint16_t _inv2_do;

    uint8_t _ext_sens_data = 0;
};

class AP_Invensensev2_AuxiliaryBus : public AuxiliaryBus
{
    friend class AP_InertialSensor_Invensensev2;

public:
    AP_HAL::Semaphore *get_semaphore() override;
    AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb) override;

protected:
    AP_Invensensev2_AuxiliaryBus(AP_InertialSensor_Invensensev2 &backend, uint32_t devid);

    AuxiliaryBusSlave *_instantiate_slave(uint8_t addr, uint8_t instance) override;
    int _configure_periodic_read(AuxiliaryBusSlave *slave, uint8_t reg,
                                 uint8_t size) override;

private:
    void _configure_slaves();

    static const uint8_t MAX_EXT_SENS_DATA = 24;
    uint8_t _ext_sens_data = 0;
};

