#include <AP_HAL/AP_HAL.h>
#include "AP_InertialSensor_HIL.h"

const extern AP_HAL::HAL& hal;

AP_InertialSensor_HIL::AP_InertialSensor_HIL(AP_InertialSensor &imu) :
    AP_InertialSensor_Backend(imu)
{
}

/*
  detect the sensor
 */
AP_InertialSensor_Backend *AP_InertialSensor_HIL::detect(AP_InertialSensor &_imu)
{
    AP_InertialSensor_HIL *sensor = new AP_InertialSensor_HIL(_imu);
    if (sensor == nullptr) {
        return nullptr;
    }
    if (!sensor->_init_sensor()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_InertialSensor_HIL::_init_sensor(void) 
{
    // grab the used instances
    uint8_t instance;
    _imu.register_gyro(instance, 1200, 1);
    _imu.register_accel(instance, 1200, 1);

    _imu.set_hil_mode();

    return true;
}

bool AP_InertialSensor_HIL::update(void) 
{
    // the data is stored directly in the frontend, so update()
    // doesn't need to do anything
    return true;
}
