#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

/* enable debug to see a register dump on startup */
#define ASM330_DEBUG 0

class AP_InertialSensor_ASM330 : public AP_InertialSensor_Backend
{
public:
    virtual ~AP_InertialSensor_ASM330() { }
    void start(void) override;
    bool update() override;

    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                            enum Rotation rotation);
private:
    AP_InertialSensor_ASM330(AP_InertialSensor &imu,
                             AP_HAL::OwnPtr<AP_HAL::Device> dev,
                             enum Rotation rotation);

    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu);

    struct PACKED sensor_raw_data {
        int16_t x;
        int16_t y;
        int16_t z;
    };

    enum gyro_scale {
        G_SCALE_125DPS = 0,
        G_SCALE_250DPS,
        G_SCALE_500DPS,
        G_SCALE_1000DPS,
        G_SCALE_2000DPS,
    };

    enum accel_scale {
        A_SCALE_2G = 0,
        A_SCALE_4G,
        A_SCALE_8G,
        A_SCALE_16G
    };

    bool _init_sensor();
    bool _hardware_init();

    uint8_t _register_read(uint8_t reg);
    void _register_write(uint8_t reg, uint8_t val, bool checked=false);

    bool _chip_reset();
    void _fifo_reset();

    void _common_init();
    void _fifo_init();
    void _gyro_init(gyro_scale scale);
    void _accel_init(accel_scale scale);

    uint16_t _get_count_fifo_unread_data();

    void _set_gyro_scale(gyro_scale scale);
    void _set_accel_scale(accel_scale scale);

    void _poll_data();

    void _update_transaction_g(struct sensor_raw_data raw_data);
    void _update_transaction_x(struct sensor_raw_data raw_data);

    #if ASM330_DEBUG
    void _dump_registers();
    #endif

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;
    AP_HAL::Semaphore *_spi_sem;
    float _gyro_scale;
    float _accel_scale;
    enum Rotation _rotation;

    float _temp_degc;
    LowPassFilter2pFloat _temp_filter;
};
