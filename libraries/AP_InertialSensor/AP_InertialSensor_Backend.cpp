/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

AP_InertialSensor_Backend::AP_InertialSensor_Backend(AP_InertialSensor &imu) :
    _imu(imu),
    _product_id(AP_PRODUCT_ID_NONE)
{}

/*
  rotate gyro vector and add the gyro offset
 */
void AP_InertialSensor_Backend::_rotate_and_offset_gyro(uint8_t instance, const Vector3f &gyro, uint32_t now)
{
    _imu._gyro[instance] = gyro;
    _imu._gyro[instance].rotate(_imu._board_orientation);
    _imu._gyro[instance] -= _imu._gyro_offset[instance];
    _imu._last_gyro_sample_time_usec[instance] = now;
}

/*
  rotate accel vector, scale and add the accel offset
 */
void AP_InertialSensor_Backend::_rotate_and_offset_accel(uint8_t instance, const Vector3f &accel, uint32_t now)
{
    _imu._accel[instance] = accel;
    _imu._accel[instance].rotate(_imu._board_orientation);

    const Vector3f &accel_scale = _imu._accel_scale[instance].get();
    _imu._accel[instance].x *= accel_scale.x;
    _imu._accel[instance].y *= accel_scale.y;
    _imu._accel[instance].z *= accel_scale.z;
    _imu._accel[instance] -= _imu._accel_offset[instance];
    _imu._last_accel_sample_time_usec[instance] = now;
}
