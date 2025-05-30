
#pragma once

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include <AP_HAL/I2CDevice.h>
#include <Filter/Filter.h>
#include <Filter/LowPassFilter2p.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_L3G4200D : public AP_InertialSensor_Backend
{
public:
    AP_InertialSensor_L3G4200D(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev_gyro,
                                            AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev_accel);
    
    
    
    virtual ~AP_InertialSensor_L3G4200D();

    // probe the sensor on I2C bus
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev_gyro,
                                            AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev_accel);

   
   
    /* update accel and gyro state */
    bool update() override;

    void start(void) override;

    
private:
    bool _accel_init();
    bool _gyro_init();
    bool _init_sensor();
    void _accumulate_gyro();
    void _accumulate_accel();
    
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev_gyro;
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev_accel;

    void _set_filter_frequency(uint8_t filter_hz);

    // Low Pass filters for gyro and accel 
    LowPassFilter2pVector3f _accel_filter;
    LowPassFilter2pVector3f _gyro_filter;
};
#endif // __AP_INERTIAL_SENSOR_L3G4200D2_H__
