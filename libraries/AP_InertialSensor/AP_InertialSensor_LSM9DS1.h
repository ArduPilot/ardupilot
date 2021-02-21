#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

/* enable debug to see a register dump on startup */
#define LSM9DS1_DEBUG 0

class AP_InertialSensor_LSM9DS1 : public AP_InertialSensor_Backend
{
public:
    virtual ~AP_InertialSensor_LSM9DS1() { }
    void start(void) override;
    bool update() override;

    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                            enum Rotation rotation);
private:
    AP_InertialSensor_LSM9DS1(AP_InertialSensor &imu,
                              AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                              int drdy_pin_num_xg,
                              enum Rotation rotation);

    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu);

    struct PACKED sensor_raw_data {
        int16_t x;
        int16_t y;
        int16_t z;
    };

    enum accel_scale {
        A_SCALE_2G = 0,
        A_SCALE_4G,
        A_SCALE_8G,
        A_SCALE_16G
    };

    void _poll_data();
    void _fifo_reset();

    bool _init_sensor();
    bool _hardware_init();

    void _gyro_init();
    void _accel_init();

    void _set_gyro_scale();
    void _set_accel_scale(accel_scale scale);

    uint8_t _register_read(uint8_t reg);
    void _register_write(uint8_t reg, uint8_t val, bool checked=false);

    void _read_data_transaction_x(uint16_t samples);
    void _read_data_transaction_g(uint16_t samples);

    #if LSM9DS1_DEBUG
    void        _dump_registers();
    #endif

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;
    AP_HAL::Semaphore *_spi_sem;
    AP_HAL::DigitalSource * _drdy_pin_xg;
    float _gyro_scale;
    float _accel_scale;
    int _drdy_pin_num_xg;
    uint8_t _gyro_instance;
    uint8_t _accel_instance;
    enum Rotation _rotation;
};
