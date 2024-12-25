
#pragma once

#include <AP_HAL/AP_HAL.h>

#include <AP_HAL/I2CDevice.h>
#include <Filter/Filter.h>
#include <Filter/LowPassFilter2p.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_ISM330DHCX : public AP_InertialSensor_Backend
{
public:
    AP_InertialSensor_ISM330DHCX(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                            enum Rotation rotation);
    
    
    
    virtual ~AP_InertialSensor_ISM330DHCX();

    // probe the sensor on I2C bus
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                            enum Rotation rotation);

   
   
    /* update accel and gyro state */
    bool update() override;

    void start(void) override;

    
private:
    bool _accel_init();
    bool _gyro_init();
    bool _init_sensor();
    void _accumulate_gyro();
    void _accumulate_accel();
    
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

    void _set_filter_frequency(uint8_t filter_hz);

#ifdef ISM330DHCX_DEBUG
    void        _dump_registers();
#endif

    // Low Pass filters for gyro and accel 
    LowPassFilter2pVector3f _accel_filter;
    LowPassFilter2pVector3f _gyro_filter;

    enum Rotation _rotation; 
};
