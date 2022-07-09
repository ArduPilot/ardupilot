#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"
#include <Filter/LowPassFilter2p.h>

class AP_InertialSensor_LSM6DS3 : public AP_InertialSensor_Backend
{
public:
    virtual ~AP_InertialSensor_LSM6DS3() { }
    void start(void) override;
    bool update() override;

    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                            enum Rotation rotation);
private:
    AP_InertialSensor_LSM6DS3(AP_InertialSensor &imu,
                              AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                              enum Rotation rotation);

    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu);
    
    struct PACKED sensor_data {
        int16_t x;
        int16_t y;
        int16_t z;
        
    };    

    struct PACKED sensor_fifo_data {
        int16_t gx;
        int16_t gy;
        int16_t gz;
        int16_t ax;
        int16_t ay;
        int16_t az;
        
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

/*    void _read_data_transaction_x(uint16_t samples);*/
/*    void _read_data_transaction_g();*/

	void _read_data_transaction(uint16_t samples);

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;
    AP_HAL::Semaphore *_spi_sem;
    float _gyro_scale;
    float _accel_scale;
    uint8_t _gyro_instance;
    uint8_t _accel_instance;
    enum Rotation _rotation;

    float _temperature;
    uint8_t _temp_counter;
    LowPassFilter2pFloat _temp_filter;
};
