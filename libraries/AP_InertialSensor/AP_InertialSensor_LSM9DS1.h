#ifndef __AP_INERTIAL_SENSOR_LSM9DS1_H__
#define __AP_INERTIAL_SENSOR_LSM9DS1_H__

#include <cstdio>


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
                                            enum Rotation rotation = ROTATION_NONE);

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

    enum gyro_scale
    {
        G_SCALE_245DPS = 0,
        G_SCALE_500DPS,
        G_SCALE_2000DPS,
    };

    enum accel_scale
    {
        A_SCALE_2G = 0,
        A_SCALE_4G,
        A_SCALE_8G,
        A_SCALE_16G
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

    uint8_t _register_read_xg(uint8_t reg);
    void _register_write_xg(uint8_t reg, uint8_t val, bool checked=false);

    void _read_data_transaction_x();
    void _read_data_transaction_g();
    //

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


#endif /* __AP_INERTIAL_SENSOR_LSM9DS1_H__ */
