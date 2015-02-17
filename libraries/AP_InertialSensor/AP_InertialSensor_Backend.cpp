/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

AP_InertialSensor_Backend::AP_InertialSensor_Backend(AP_InertialSensor &imu) :
    _imu(imu),
    _product_id(AP_PRODUCT_ID_NONE)
{}

void AP_InertialSensor_Backend::_rotate_and_correct_accel(uint8_t instance, Vector3f &accel) {
    const Vector3f &accel_scale = _imu._accel_scale[instance].get();
    accel.rotate(_imu._board_orientation);
    accel.x *= accel_scale.x;
    accel.y *= accel_scale.y;
    accel.z *= accel_scale.z;
    accel -= _imu._accel_offset[instance];
}

void AP_InertialSensor_Backend::_rotate_and_correct_gyro(uint8_t instance, Vector3f &gyro) {
    gyro.rotate(_imu._board_orientation);
    gyro -= _imu._gyro_offset[instance];
}

void AP_InertialSensor_Backend::_publish_delta_angle(uint8_t instance, const Vector3f &delta_angle)
{
    // publish delta angle
    _imu._delta_angle[instance] = delta_angle;
    _imu._delta_angle_valid[instance] = true;
}

/*
  rotate gyro vector and add the gyro offset
 */
void AP_InertialSensor_Backend::_publish_gyro(uint8_t instance, const Vector3f &gyro, bool rotate_and_correct)
{
    _imu._gyro[instance] = gyro;
    _imu._gyro_healthy[instance] = true;

    if (rotate_and_correct) {
        _rotate_and_correct_gyro(instance, _imu._gyro[instance]);
    }
}

void AP_InertialSensor_Backend::_publish_delta_velocity(uint8_t instance, const Vector3f &delta_velocity)
{
    // publish delta velocity
    _imu._delta_velocity[instance] = delta_velocity;
    _imu._delta_velocity_valid[instance] = true;
}

/*
  rotate accel vector, scale and add the accel offset
 */
void AP_InertialSensor_Backend::_publish_accel(uint8_t instance, const Vector3f &accel, bool rotate_and_correct)
{
    _imu._accel[instance] = accel;
    _imu._accel_healthy[instance] = true;

    if (rotate_and_correct) {
        _rotate_and_correct_accel(instance, _imu._accel[instance]);
    }
}

// set accelerometer error_count
void AP_InertialSensor_Backend::_set_accel_error_count(uint8_t instance, uint32_t error_count)
{
    _imu._accel_error_count[instance] = error_count;
}

// set gyro error_count
void AP_InertialSensor_Backend::_set_gyro_error_count(uint8_t instance, uint32_t error_count)
{
    _imu._gyro_error_count[instance] = error_count;
}

/*
  return the default filter frequency in Hz for the sample rate
  
  This uses the sample_rate as a proxy for what type of vehicle it is
  (ie. plane and rover run at 50Hz). Copters need a bit more filter
  bandwidth
 */
uint8_t AP_InertialSensor_Backend::_default_filter(void) const
{
    switch (_imu.get_sample_rate()) {
    case AP_InertialSensor::RATE_50HZ:
        // on Rover and plane use a lower filter rate
        return 15;
    case AP_InertialSensor::RATE_100HZ:
        return 30;
    case AP_InertialSensor::RATE_200HZ:
        return 30;
    case AP_InertialSensor::RATE_400HZ:
        return 30;
    }
    return 30;
}
