#include <AP_HAL/AP_HAL.h>
#include <AP_Baro/AP_Baro.h>
#include "AP_InertialNav.h"

/*
  A wrapper around the AP_InertialNav class which uses the NavEKF
  filter if available, and falls back to the AP_InertialNav filter
  when EKF is not available
 */

/**
   update internal state
*/
void AP_InertialNav::update(bool high_vibes)
{
    // get the NE position relative to the local earth frame origin
    Vector2f posNE;
    if (_ahrs_ekf.get_relative_position_NE_origin(posNE)) {
        _relpos_cm.x = posNE.x * 100; // convert from m to cm
        _relpos_cm.y = posNE.y * 100; // convert from m to cm
    }

    // get the D position relative to the local earth frame origin
    float posD;
    if (_ahrs_ekf.get_relative_position_D_origin(posD)) {
        _relpos_cm.z = - posD * 100; // convert from m in NED to cm in NEU
    }

    // get the velocity relative to the local earth frame
    Vector3f velNED;
    if (_ahrs_ekf.get_velocity_NED(velNED)) {
        // during high vibration events use vertical position change
        if (high_vibes) {
            float rate_z;
            if (_ahrs_ekf.get_vert_pos_rate_D(rate_z)) {
                velNED.z = rate_z;
            }
        }
        _velocity_cm = velNED * 100; // convert to cm/s
        _velocity_cm.z = -_velocity_cm.z; // convert from NED to NEU
    }
}

/**
 * get_filter_status : returns filter status as a series of flags
 */
nav_filter_status AP_InertialNav::get_filter_status() const
{
    nav_filter_status status;
    _ahrs_ekf.get_filter_status(status);
    return status;
}

/**
 * get_position_neu_cm - returns the current position relative to the EKF origin in cm.
 *
 * @return
 */
const Vector3f &AP_InertialNav::get_position_neu_cm(void) const 
{
    return _relpos_cm;
}

/**
 * get_position_xy_cm - returns the current x-y position relative to the EKF origin in cm.
 *
 * @return
 */
const Vector2f &AP_InertialNav::get_position_xy_cm() const
{
    return _relpos_cm.xy();
}

/**
 * get_position_z_up_cm - returns the current z position relative to the EKF origin, frame z-axis up, in cm.
 * @return
 */
float AP_InertialNav::get_position_z_up_cm() const
{
    return _relpos_cm.z;
}

/**
 * get_velocity_neu_cms - returns the current velocity in cm/s
 *
 * @return velocity vector:
 *      		.x : latitude  velocity in cm/s
 * 				.y : longitude velocity in cm/s
 * 				.z : vertical  velocity in cm/s
 */
const Vector3f &AP_InertialNav::get_velocity_neu_cms() const
{
    return _velocity_cm;
}

/**
 * get_velocity_xy_cms - returns the current x-y velocity relative to the EKF origin in cm.
 *
 * @return
 */
const Vector2f &AP_InertialNav::get_velocity_xy_cms() const
{
    return _velocity_cm.xy();
}

/**
 * get_speed_xy_cms - returns the current horizontal speed in cm/s
 *
 * @returns the current horizontal speed in cm/s
 */
float AP_InertialNav::get_speed_xy_cms() const
{
    return _velocity_cm.xy().length();
}

/**
 * get_velocity_z_up_cms - returns the current z-axis velocity, frame z-axis up, in cm/s
 *
 * @return z-axis velocity, frame z-axis up, in cm/s
 */
float AP_InertialNav::get_velocity_z_up_cms() const
{
    return _velocity_cm.z;
}
