#pragma once

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/SPIDevice.h>
#include <Filter/Filter.h>
#include <Filter/LowPassFilter2p.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_RST : public AP_InertialSensor_Backend
{
public:
    AP_InertialSensor_RST(AP_InertialSensor &imu,
                              AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_gyro,
                              AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_accel,
                              enum Rotation rotation_a,
                              enum Rotation rotation_g);

    virtual ~AP_InertialSensor_RST();

    // probe the sensor on SPI bus
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                              AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_gyro,
                                              AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_accel,
                                              enum Rotation rotation_a,
                                              enum Rotation rotation_g);

    /* update accel and gyro state */
    bool update() override;

    void start(void) override;

private:
    bool _init_sensor();
    bool _init_gyro();
    bool _init_accel();
    void gyro_measure();
    void accel_measure();

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev_gyro;//i3g4250d
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev_accel;//iis328dq

    float _gyro_scale;
    float _accel_scale;

    // gyro and accel instances
    uint8_t _gyro_instance;
    uint8_t _accel_instance;
    enum Rotation _rotation_g;
    enum Rotation _rotation_a;
};
#endif
