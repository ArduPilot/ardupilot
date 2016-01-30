#pragma once

#define LSM9DS0_DEBUG 0

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_LSM9DS0 : public AP_InertialSensor_Backend
{
public:
    virtual ~AP_InertialSensor_LSM9DS0() { }
    bool update();

    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_gyro,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_accel);

private:
    AP_InertialSensor_LSM9DS0(AP_InertialSensor &imu,
                              AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_gyro,
                              AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_accel,
                              int drdy_pin_num_a, int drdy_pin_num_b);

    struct PACKED sensor_raw_data {
        int16_t x;
        int16_t y;
        int16_t z;
    };

    enum gyro_scale {
        G_SCALE_245DPS = 0,
        G_SCALE_500DPS,
        G_SCALE_2000DPS,
    };

    enum accel_scale {
        A_SCALE_2G = 0,
        A_SCALE_4G,
        A_SCALE_6G,
        A_SCALE_8G,
        A_SCALE_16G,
    };

    bool _accel_data_ready();
    bool _gyro_data_ready();

    void _poll_data();

    bool _init_sensor();
    bool _hardware_init();

    void _gyro_init();
    void _accel_init();

    void _set_gyro_scale(gyro_scale scale);
    void _set_accel_scale(accel_scale scale);

    uint8_t _register_read_xm(uint8_t reg);
    uint8_t _register_read_g(uint8_t reg);
    void _register_write_xm(uint8_t reg, uint8_t val);
    void _register_write_g(uint8_t reg, uint8_t val);

    void _read_data_transaction_a();
    void _read_data_transaction_g();

#if LSM9DS0_DEBUG
    void        _dump_registers();
#endif

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev_gyro;
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev_accel;
    AP_HAL::Semaphore *_spi_sem;

    /*
     * If data-ready GPIO pins numbers are not defined (i.e. any negative
     * value), the fallback approach used is to check if there's new data ready
     * by reading the status register. It is *strongly* recommended to use
     * data-ready GPIO pins for performance reasons.
     */
    AP_HAL::DigitalSource * _drdy_pin_a;
    AP_HAL::DigitalSource * _drdy_pin_g;
    float _gyro_scale;
    float _accel_scale;
    int _drdy_pin_num_a;
    int _drdy_pin_num_g;
    uint8_t _gyro_instance;
    uint8_t _accel_instance;
};
