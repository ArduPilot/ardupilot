/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_MPU6000_H__
#define __AP_INERTIAL_SENSOR_MPU6000_H__

#include <stdint.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Progmem/AP_Progmem.h>

#include "AP_InertialSensor.h"
#include "AuxiliaryBus.h"

// enable debug to see a register dump on startup
#define MPU6000_DEBUG 0

// on fast CPUs we sample at 1kHz and use a software filter
#if HAL_CPU_CLASS >= HAL_CPU_CLASS_75
#define MPU6000_FAST_SAMPLING 1
#else
#define MPU6000_FAST_SAMPLING 0
#endif

#if MPU6000_FAST_SAMPLING
#include <Filter/Filter.h>
#include <Filter/LowPassFilter2p.h>
#include <Filter/LowPassFilter.h>
#endif

#define MPU6000_SAMPLE_SIZE 14
#define MPU6000_MAX_FIFO_SAMPLES 3
#define MAX_DATA_READ (MPU6000_MAX_FIFO_SAMPLES * MPU6000_SAMPLE_SIZE)

class AP_MPU6000_AuxiliaryBus;
class AP_MPU6000_AuxiliaryBusSlave;

class AP_MPU6000_BusDriver
{
public:
    virtual ~AP_MPU6000_BusDriver() { };
    virtual void init() = 0;
    virtual void start(bool &fifo_mode, uint8_t &max_samples) = 0;
    virtual void read8(uint8_t reg, uint8_t *val) = 0;

    /// Copy data from the device to @p buf starting at @p reg with @size
    /// length.
    virtual void read_block(uint8_t reg, uint8_t *buf, uint32_t size) = 0;
    virtual void write8(uint8_t reg, uint8_t val) = 0;
    virtual void set_bus_speed(AP_HAL::SPIDeviceDriver::bus_speed speed) = 0;
    virtual void read_data_transaction(uint8_t* samples,
                            AP_HAL::DigitalSource *_drdy_pin,
                            uint8_t &n_samples) = 0;
    virtual AP_HAL::Semaphore* get_semaphore() = 0;
    virtual bool has_auxiliar_bus() = 0;
};

class AP_InertialSensor_MPU6000 : public AP_InertialSensor_Backend
{
    friend AP_MPU6000_AuxiliaryBus;
    friend AP_MPU6000_AuxiliaryBusSlave;

public:
    AP_InertialSensor_MPU6000(AP_InertialSensor &imu, AP_MPU6000_BusDriver *bus);
    ~AP_InertialSensor_MPU6000();
    static AP_InertialSensor_Backend *detect_i2c(AP_InertialSensor &_imu,
                                                 AP_HAL::I2CDriver *i2c,
                                                 uint8_t addr);
    static AP_InertialSensor_Backend *detect_spi(AP_InertialSensor &_imu);
    static AP_InertialSensor_MPU6000 &from(AP_InertialSensor_Backend &backend) {
        return static_cast<AP_InertialSensor_MPU6000&>(backend);
    }

    /* update accel and gyro state */
    bool update();

    bool gyro_sample_available(void) { return _sum_count >= _sample_count; }
    bool accel_sample_available(void) { return _sum_count >= _sample_count; }

    /*
     * Return an AuxiliaryBus if the bus driver allows it
     */
    AuxiliaryBus *get_auxiliar_bus() override;

    void start() override;

private:
    static AP_InertialSensor_Backend *_detect(AP_InertialSensor &_imu,
                                              AP_MPU6000_BusDriver *bus,
                                              int16_t id = -1);

#if MPU6000_DEBUG
    void _dump_registers(void);
#endif

    // instance numbers of accel and gyro data
    uint8_t _gyro_instance;
    uint8_t _accel_instance;

    AP_HAL::DigitalSource *_drdy_pin;
    bool    _init_sensor(void);
    bool    _sample_available();
    void    _read_data_transaction();
    bool    _data_ready();
    void    _poll_data(void);
    void    _read_block(uint8_t reg, uint8_t *buf, uint32_t size);
    uint8_t _register_read( uint8_t reg);
    void    _register_write( uint8_t reg, uint8_t val );
    void    _register_write_check(uint8_t reg, uint8_t val);
    bool    _hardware_init(void);
    void    _accumulate(uint8_t *samples, uint8_t n_samples);

    AP_MPU6000_BusDriver *_bus;
    AP_HAL::Semaphore    *_bus_sem;

    AP_MPU6000_AuxiliaryBus *_auxiliar_bus = nullptr;

    static const float   _gyro_scale;

    // support for updating filter at runtime
    int8_t _last_accel_filter_hz;
    int8_t _last_gyro_filter_hz;

    void _set_filter_register(uint16_t filter_hz);

    // how many hardware samples before we report a sample to the caller
    uint8_t _sample_count;

#if MPU6000_FAST_SAMPLING
    Vector3f _accel_filtered;
    Vector3f _gyro_filtered;
    float    _temp_filtered;

    // Low Pass filters for gyro and accel
    LowPassFilter2pVector3f _accel_filter;
    LowPassFilter2pVector3f _gyro_filter;
    LowPassFilter2pFloat    _temp_filter;
#else
    // accumulation in timer - must be read with timer disabled
    // the sum of the values since last read
    Vector3l _accel_sum;
    Vector3l _gyro_sum;
    int32_t  _temp_sum;

#endif
    volatile uint16_t _sum_count;
    bool _fifo_mode;
    uint8_t *_samples = nullptr;
};

class AP_MPU6000_BusDriver_SPI : public AP_MPU6000_BusDriver
{
public:
    AP_MPU6000_BusDriver_SPI(void);
    void init();
    void start(bool &fifo_mode, uint8_t &max_samples);
    void read8(uint8_t reg, uint8_t *val);
    void read_block(uint8_t reg, uint8_t *buf, uint32_t size) override;
    void write8(uint8_t reg, uint8_t val);
    void set_bus_speed(AP_HAL::SPIDeviceDriver::bus_speed speed);
    void read_data_transaction(uint8_t* samples,
                               AP_HAL::DigitalSource *_drdy_pin,
                               uint8_t &n_samples);
    AP_HAL::Semaphore* get_semaphore();
    bool has_auxiliar_bus() override;

private:
    AP_HAL::SPIDeviceDriver *_spi;
    AP_HAL::Semaphore *_spi_sem;
    // count of bus errors
    uint16_t _error_count;
};

class AP_MPU6000_BusDriver_I2C : public AP_MPU6000_BusDriver
{
public:
    AP_MPU6000_BusDriver_I2C(AP_HAL::I2CDriver *i2c, uint8_t addr);
    void init();
    void start(bool &fifo_mode, uint8_t &max_samples);
    void read8(uint8_t reg, uint8_t *val);
    void read_block(uint8_t reg, uint8_t *buf, uint32_t size) override;
    void write8(uint8_t reg, uint8_t val);
    void set_bus_speed(AP_HAL::SPIDeviceDriver::bus_speed speed);
    void read_data_transaction(uint8_t* samples,
                               AP_HAL::DigitalSource *_drdy_pin,
                               uint8_t &n_samples);
    AP_HAL::Semaphore* get_semaphore();
    bool has_auxiliar_bus() override;

private:
    uint8_t _addr;
    AP_HAL::I2CDriver *_i2c;
    AP_HAL::Semaphore *_i2c_sem;
    uint8_t _rx[MAX_DATA_READ];
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

#endif // __AP_INERTIAL_SENSOR_MPU6000_H__
