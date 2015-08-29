/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>
#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

AP_InertialSensor_Backend::AP_InertialSensor_Backend(AP_InertialSensor &imu) :
    _imu(imu),
    _product_id(AP_PRODUCT_ID_NONE)
{}

void AP_InertialSensor_Backend::_rotate_and_correct_accel(uint8_t instance, Vector3f &accel) 
{
    /*
      accel calibration is always done in sensor frame with this
      version of the code. That means we apply the rotation after the
      offsets and scaling.
     */

    // apply scaling
    const Vector3f &accel_scale = _imu._accel_scale[instance].get();
    accel.x *= accel_scale.x;
    accel.y *= accel_scale.y;
    accel.z *= accel_scale.z;

    // apply offsets
    accel -= _imu._accel_offset[instance];

    // rotate to body frame
    accel.rotate(_imu._board_orientation);
}

void AP_InertialSensor_Backend::_rotate_and_correct_gyro(uint8_t instance, Vector3f &gyro) 
{
    // gyro calibration is always assumed to have been done in sensor frame
    gyro -= _imu._gyro_offset[instance];
    gyro.rotate(_imu._board_orientation);
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

void AP_InertialSensor_Backend::_publish_delta_velocity(uint8_t instance, const Vector3f &delta_velocity, float dt)
{
    // publish delta velocity
    _imu._delta_velocity[instance] = delta_velocity;
    _imu._delta_velocity_dt[instance] = dt;
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

void AP_InertialSensor_Backend::_set_accel_max_abs_offset(uint8_t instance,
                                                          float max_offset)
{
    _imu._accel_max_abs_offsets[instance] = max_offset;
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

// return the requested sample rate in Hz
uint16_t AP_InertialSensor_Backend::get_sample_rate_hz(void) const
{
    // enum can be directly cast to Hz
    return (uint16_t)_imu._sample_rate;
}

/*
  publish a temperature value for an instance
 */
void AP_InertialSensor_Backend::_publish_temperature(uint8_t instance, float temperature)
{
    _imu._temperature[instance] = temperature;
}
