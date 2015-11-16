
#ifndef __AP_INERTIAL_SENSOR_LSM9DS0_H__
#define __AP_INERTIAL_SENSOR_LSM9DS0_H__

#define LSM9DS0_DEBUG 0

#include <AP_HAL/AP_HAL.h>
#include "AP_InertialSensor.h"

class AP_InertialSensor_LSM9DS0 : public AP_InertialSensor_Backend
{
public:

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
        A_SCALE_6G,
        A_SCALE_8G,
        A_SCALE_16G
    };

    AP_InertialSensor_LSM9DS0(AP_InertialSensor &imu,
                              int drdy_pin_num_a, int drdy_pin_num_b);

    bool        update();

    static AP_InertialSensor_Backend *      detect(AP_InertialSensor &imu);

private:
    struct PACKED        sensor_raw_data {
        int16_t x;
        int16_t y;
        int16_t z;
    };

    AP_HAL::SPIDeviceDriver *       _accel_spi;
    AP_HAL::SPIDeviceDriver *       _gyro_spi;
    AP_HAL::Semaphore *             _spi_sem;

    /*
     * If data-ready GPIO pins numbers are not defined (i.e. any negative
     * value), the fallback approach used is to check if there's new data ready
     * by reading the status register. It is *strongly* recommended to use
     * data-ready GPIO pins for performance reasons.
     */
    int                             _drdy_pin_num_a;
    AP_HAL::DigitalSource *         _drdy_pin_a;
    int                             _drdy_pin_num_g;
    AP_HAL::DigitalSource *         _drdy_pin_g;

    bool                            _accel_data_ready();
    bool                            _gyro_data_ready();

    void                            _poll_data();

    bool                            _init_sensor();
    bool                            _hardware_init();

    uint8_t                         _gyro_instance;
    uint8_t                         _accel_instance;

    void                            _gyro_init();
    void                            _accel_init();

    float                           _gyro_scale, _accel_scale;
    void                            _set_gyro_scale(gyro_scale scale);
    void                            _set_accel_scale(accel_scale scale);

    uint8_t                         _register_read_xm( uint8_t reg );
    uint8_t                         _register_read_g( uint8_t reg );

    void                            _register_write_xm( uint8_t reg, uint8_t val );
    void                            _register_write_g( uint8_t reg, uint8_t val );

    void                            _accel_raw_data(struct sensor_raw_data *raw_data);
    void                            _gyro_raw_data(struct sensor_raw_data *raw_data);

    void                            _read_data_transaction_a();
    void                            _read_data_transaction_g();

#if LSM9DS0_DEBUG
    void        _dump_registers();
#endif
};

#endif /* __AP_INERTIAL_SENSOR_LSM9DS0_H__ */
