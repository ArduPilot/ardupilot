/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_MPU9250_H__
#define __AP_INERTIAL_SENSOR_MPU9250_H__

#include <stdint.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <Filter/Filter.h>
#include <Filter/LowPassFilter2p.h>
#include "AP_InertialSensor.h"

// enable debug to see a register dump on startup
#define MPU9250_DEBUG 0

class AP_MPU9250_BusDriver
{
public:
    virtual ~AP_MPU9250_BusDriver() { };
    virtual void init() = 0;
    virtual void read8(uint8_t reg, uint8_t *val) = 0;
    virtual void write8(uint8_t reg, uint8_t val) = 0;
    virtual void set_bus_speed(AP_HAL::SPIDeviceDriver::bus_speed speed) = 0;
    virtual bool read_data_transaction(uint8_t* samples,
                                       uint8_t &n_samples) = 0;
    virtual AP_HAL::Semaphore* get_semaphore() = 0;
    virtual void get_driver_state(bool &initialized, bool &working) = 0;
    virtual void set_driver_state(bool working) = 0;
};

class AP_InertialSensor_MPU9250 : public AP_InertialSensor_Backend
{
public:

    AP_InertialSensor_MPU9250(AP_InertialSensor &imu, AP_MPU9250_BusDriver *bus);

    /* update accel and gyro state */
    bool update();

    bool gyro_sample_available(void) { return _have_sample_available; }
    bool accel_sample_available(void) { return _have_sample_available; }

    /* Put the MPU9250 in a known state so it can be
     * used both for the InertialSensor and as for backend of other drivers.
     *
     * The bus semaphore must be taken and timer_procs suspended.
     *
     * This method puts the bus in low speed. If the initialization is
     * successful the bus is left on low speed so the caller can finish the
     * initialization of its driver.
     */
    static bool initialize_driver_state(AP_HAL::SPIDeviceDriver *spi);

    // detect the sensor
    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu, AP_HAL::SPIDeviceDriver *spi);

private:
    static AP_InertialSensor_Backend *_detect(AP_InertialSensor &_imu,
                                              AP_MPU9250_BusDriver *bus,
                                              int16_t id);


    static bool initialize_driver_state(AP_MPU9250_BusDriver *bus);

    static uint8_t _register_read(AP_MPU9250_BusDriver *bus, uint8_t reg);
    static void _register_write(AP_MPU9250_BusDriver *bus, uint8_t reg,
                                uint8_t val);

    bool                 _init_sensor();
    void                 _read_data_transaction();
    bool                 _data_ready();
    void                 _poll_data(void);
    uint8_t              _register_read( uint8_t reg );
    void                 _register_write( uint8_t reg, uint8_t val );
    bool                 _hardware_init(void);
    bool                 _sample_available();

    AP_MPU9250_BusDriver *_bus;
    AP_HAL::Semaphore *_bus_sem;

    // support for updating filter at runtime
    int16_t _last_gyro_filter_hz;
    int16_t _last_accel_filter_hz;

    // change the filter frequency
    void _set_accel_filter(uint8_t filter_hz);
    void _set_gyro_filter(uint8_t filter_hz);

    // This structure is used to pass data from the timer which reads
    // the sensor to the main thread. The _shared_data_idx is used to
    // prevent race conditions by ensuring the data is fully updated
    // before being used by the consumer
    struct {
        Vector3f _accel_filtered;
        Vector3f _gyro_filtered;
    } _shared_data[2];
    volatile uint8_t _shared_data_idx;

    // Low Pass filters for gyro and accel 
    LowPassFilter2pVector3f _accel_filter;
    LowPassFilter2pVector3f _gyro_filter;

    // do we currently have a sample pending?
    bool _have_sample_available;

    // gyro and accel instances
    uint8_t _gyro_instance;
    uint8_t _accel_instance;

    // The default rotation for the IMU, its value depends on how the IMU is
    // placed by default on the system
    enum Rotation _default_rotation;

#if MPU9250_DEBUG
    static void _dump_registers(AP_MPU9250_BusDriver *bus);
#endif
};

class AP_MPU9250_BusDriver_SPI : public AP_MPU9250_BusDriver
{
public:
    AP_MPU9250_BusDriver_SPI(AP_HAL::SPIDeviceDriver *spi);
    void init();
    void read8(uint8_t reg, uint8_t *val);
    void write8(uint8_t reg, uint8_t val);
    void set_bus_speed(AP_HAL::SPIDeviceDriver::bus_speed speed);
    bool read_data_transaction(uint8_t* samples, uint8_t &n_samples);
    AP_HAL::Semaphore* get_semaphore();
    void get_driver_state(bool &initialized, bool &working);
    void set_driver_state(bool working);

private:
    AP_HAL::SPIDeviceDriver *_spi;
};

#endif // __AP_INERTIAL_SENSOR_MPU9250_H__
