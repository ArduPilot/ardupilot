#pragma once
/*
  driver for the invensense range of IMUs, including:

  MPU6000
  MPU9250
  ICM-20608
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

class AP_Invensense_AuxiliaryBus;
class AP_Invensense_AuxiliaryBusSlave;

class AP_InertialSensor_Invensense : public AP_InertialSensor_Backend
{
    friend AP_Invensense_AuxiliaryBus;
    friend AP_Invensense_AuxiliaryBusSlave;

public:
    virtual ~AP_InertialSensor_Invensense();
    static AP_InertialSensor_Invensense &from(AP_InertialSensor_Backend &backend) {
        return static_cast<AP_InertialSensor_Invensense&>(backend);
    }

    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                            enum Rotation rotation);

    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                            enum Rotation rotation);

    /* update accel and gyro state */
    bool update() override __RAMFUNC__; /* front end */
    void accumulate() override; /* front end */

    /*
     * Return an AuxiliaryBus if the bus driver allows it
     */
    AuxiliaryBus *get_auxiliary_bus() override;

    void start() override;

    // get a startup banner to output to the GCS
    bool get_output_banner(char* banner, uint8_t banner_len) override;

    enum Invensense_Type {
        Invensense_MPU6000=0,
        Invensense_MPU6500,
        Invensense_MPU9250,
        Invensense_ICM20608,
        Invensense_ICM20602,
        Invensense_ICM20601,
        Invensense_ICM20789,
        Invensense_ICM20689,
    };

    // acclerometers on Invensense sensors will return values up to
    // 24G, but they are not guaranteed to be remotely linear past
    // 16G
    const uint16_t multiplier_accel = INT16_MAX/(26*GRAVITY_MSS);

protected:
#if AP_INERTIALSENSOR_DYNAMIC_FIFO
    void set_primary_gyro(bool is_primary) override;
#endif

private:
    AP_InertialSensor_Invensense(AP_InertialSensor &imu,
                              AP_HAL::OwnPtr<AP_HAL::Device> dev,
                              enum Rotation rotation);

    /* Initialize sensor*/
    bool _init();
    bool _hardware_init();
    bool _check_whoami();

    void _set_filter_register(void);
    void _fifo_reset(bool log_error) __RAMFUNC__;
    void _fast_fifo_reset() __RAMFUNC__;

    bool _has_auxiliary_bus();

    /* Read samples from FIFO (FIFO enabled) */
    void _read_fifo() __RAMFUNC__;

    /* Check if there's data available by either reading DRDY pin or register */
    bool _data_ready() __RAMFUNC__;

    /* Poll for new data (non-blocking) */
    void _poll_data() __RAMFUNC__;

    // debug function to watch for register changes
    void _check_register_change(void) __RAMFUNC__;

    /* Read and write functions taking the differences between buses into
     * account */
    bool _block_read(uint8_t reg, uint8_t *buf, uint32_t size) __RAMFUNC__;
    uint8_t _register_read(uint8_t reg) __RAMFUNC__;
    void _register_write(uint8_t reg, uint8_t val, bool checked=false) __RAMFUNC__;

    bool _accumulate(uint8_t *samples, uint8_t n_samples) __RAMFUNC__;
    bool _accumulate_sensor_rate_sampling(uint8_t *samples, uint8_t n_samples) __RAMFUNC__;

    bool _check_raw_temp(int16_t t2) __RAMFUNC__;

    int16_t _raw_temp;
    
    float temp_sensitivity = 1.0f/340; // degC/LSB
    float temp_zero = 36.53f; // degC
    
    float _temp_filtered;
    float _accel_scale;
    float _gyro_scale;

    float _fifo_accel_scale;
    float _fifo_gyro_scale;
    LowPassFilter2pFloat _temp_filter;
    uint32_t last_reset_ms;
    uint8_t reset_count;
    uint8_t fast_reset_count;
    uint8_t last_fast_reset_count;
    uint32_t last_fast_reset_count_report_ms;

    enum Rotation _rotation;

    // enable checking of unexpected resets of offsets
    bool _enable_offset_checking;

    // enable fast fifo reset instead of full fifo reset
    bool _enable_fast_fifo_reset;

    // ICM-20602 y offset register. See usage for explanation
    uint8_t _saved_y_ofs_high;

    AP_HAL::DigitalSource *_drdy_pin;
    AP_HAL::OwnPtr<AP_HAL::Device> _dev;
    AP_HAL::Device::PeriodicHandle periodic_handle;
    AP_Invensense_AuxiliaryBus *_auxiliary_bus;

    // which sensor type this is
    enum Invensense_Type _mpu_type;

    // are we doing more than 1kHz sampling?
    bool _fast_sampling;

    // what downsampling rate are we using from the FIFO for gyros?
    uint8_t _gyro_fifo_downsample_rate;

    // what downsampling rate are we using from the FIFO for accels?
    uint8_t _accel_fifo_downsample_rate;

    // ratio of raw gyro to accel sample rate
    uint8_t _gyro_to_accel_sample_ratio;

    // what rate are we generating samples into the backend for gyros?
    uint16_t _gyro_backend_rate_hz;

    // what rate are we generating samples into the backend for accels?
    uint16_t _accel_backend_rate_hz;

    // Last status from register user control
    uint8_t _last_stat_user_ctrl;    

    // buffer for fifo read
    uint8_t *_fifo_buffer;

    /*
      accumulators for sensor_rate sampling
      See description in _accumulate_sensor_rate_sampling()
    */
    struct {
        Vector3f accel;
        Vector3f gyro;
        uint8_t accel_count;
        uint8_t gyro_count;
        LowPassFilterConstDtVector3f accel_filter{4000, 188};
    } _accum;
};

class AP_Invensense_AuxiliaryBusSlave : public AuxiliaryBusSlave
{
    friend class AP_Invensense_AuxiliaryBus;

public:
    int passthrough_read(uint8_t reg, uint8_t *buf, uint8_t size) override;
    int passthrough_write(uint8_t reg, uint8_t val) override;

    int read(uint8_t *buf) override;

protected:
    AP_Invensense_AuxiliaryBusSlave(AuxiliaryBus &bus, uint8_t addr, uint8_t instance);
    int _set_passthrough(uint8_t reg, uint8_t size, uint8_t *out = nullptr);

private:
    const uint8_t _mpu_addr;
    const uint8_t _mpu_reg;
    const uint8_t _mpu_ctrl;
    const uint8_t _mpu_do;

    uint8_t _ext_sens_data = 0;
};

class AP_Invensense_AuxiliaryBus : public AuxiliaryBus
{
    friend class AP_InertialSensor_Invensense;

public:
    AP_HAL::Semaphore *get_semaphore() override;
    AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb) override;

protected:
    AP_Invensense_AuxiliaryBus(AP_InertialSensor_Invensense &backend, uint32_t devid);

    AuxiliaryBusSlave *_instantiate_slave(uint8_t addr, uint8_t instance) override;
    int _configure_periodic_read(AuxiliaryBusSlave *slave, uint8_t reg,
                                 uint8_t size) override;

private:
    void _configure_slaves();

    static const uint8_t MAX_EXT_SENS_DATA = 24;
    uint8_t _ext_sens_data = 0;
};

#ifndef INS_INVENSENSE_20789_I2C_ADDR
#define INS_INVENSENSE_20789_I2C_ADDR 0x68
#endif
