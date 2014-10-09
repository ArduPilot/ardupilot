/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#include "AP_InertialNav.h"

#if AP_AHRS_NAVEKF_AVAILABLE

/*
  A wrapper around the AP_InertialNav class which uses the NavEKF
  filter if available, and falls back to the AP_InertialNav filter
  when EKF is not available
 */

/**
 * initializes the object.
 */
void AP_InertialNav_NavEKF::init()
{
    AP_InertialNav::init();
}

/**
   update internal state
*/
void AP_InertialNav_NavEKF::update(float dt)
{
    AP_InertialNav::update(dt);
    _ahrs_ekf.get_NavEKF().getPosNED(_relpos_cm);
    _relpos_cm *= 100; // convert to cm

    _haveabspos = _ahrs.get_position(_abspos);

    _ahrs_ekf.get_NavEKF().getVelNED(_velocity_cm);
    _velocity_cm *= 100; // convert to cm/s

    // InertialNav is NEU
    _relpos_cm.z = - _relpos_cm.z;
    _velocity_cm.z = -_velocity_cm.z;
}

/**
 * position_ok - true if inertial based altitude and position can be trusted
 * @return
 */
bool AP_InertialNav_NavEKF::position_ok() const
{
    if (_ahrs.have_inertial_nav() && _haveabspos) {
        return true;
    }
    return AP_InertialNav::position_ok();
}

/**
 * get_position - returns the current position relative to the home location in cm.
 *
 * the home location was set with AP_InertialNav::set_home_position(int32_t, int32_t)
 *
 * @return
 */
const Vector3f &AP_InertialNav_NavEKF::get_position(void) const 
{
    if (_ahrs.have_inertial_nav()) {
        return _relpos_cm;
    }
    return AP_InertialNav::get_position();
}

/**
 * get_latitude - returns the latitude of the current position estimation in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
 */
int32_t AP_InertialNav_NavEKF::get_latitude() const
{
    if (_ahrs.have_inertial_nav() && _haveabspos) {
        return _abspos.lat;
    }
    return AP_InertialNav::get_latitude();
}

/**
 * get_longitude - returns the longitude of the current position estimation in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
 * @return
 */
int32_t AP_InertialNav_NavEKF::get_longitude() const
{
    if (_ahrs.have_inertial_nav() && _haveabspos) {
        return _abspos.lng;
    }
    return AP_InertialNav::get_longitude();
}

/**
 * get_latitude_diff - returns the current latitude difference from the home location.
 *
 * @return difference in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
 */
float AP_InertialNav_NavEKF::get_latitude_diff() const
{
    if (_ahrs.have_inertial_nav()) {
        return _relpos_cm.x / LATLON_TO_CM;
    }
    return AP_InertialNav::get_latitude_diff();
}

/**
 * get_longitude_diff - returns the current longitude difference from the home location.
 *
 * @return difference in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
 */
float AP_InertialNav_NavEKF::get_longitude_diff() const
{
    if (_ahrs.have_inertial_nav()) {
        return _relpos_cm.y / _lon_to_cm_scaling;
    }
    return AP_InertialNav::get_longitude_diff();
}

/**
 * get_velocity - returns the current velocity in cm/s
 *
 * @return velocity vector:
 *      		.x : latitude  velocity in cm/s
 * 				.y : longitude velocity in cm/s
 * 				.z : vertical  velocity in cm/s
 */
const Vector3f &AP_InertialNav_NavEKF::get_velocity() const
{
    if (_ahrs.have_inertial_nav()) {
        return _velocity_cm;
    }
    return AP_InertialNav::get_velocity();
}

/**
 * get_velocity_xy - returns the current horizontal velocity in cm/s
 *
 * @returns the current horizontal velocity in cm/s
 */
float AP_InertialNav_NavEKF::get_velocity_xy() const
{
    if (_ahrs.have_inertial_nav()) {
        return pythagorous2(_velocity_cm.x, _velocity_cm.y);
    }
    return AP_InertialNav::get_velocity_xy();
}

/**
 * altitude_ok - returns true if inertial based altitude and position can be trusted
 * @return
 */
bool AP_InertialNav_NavEKF::altitude_ok() const
{
    if (_ahrs.have_inertial_nav() && _haveabspos) {
        return true;
    }
    return AP_InertialNav::altitude_ok();
}

/**
 * get_altitude - get latest altitude estimate in cm
 * @return
 */
float AP_InertialNav_NavEKF::get_altitude() const
{
    if (_ahrs.have_inertial_nav()) {
        return _relpos_cm.z;
    }
    return AP_InertialNav::get_altitude();
}

/**
 * get_velocity_z - returns the current climbrate.
 *
 * @see get_velocity().z
 *
 * @return climbrate in cm/s
 */
float AP_InertialNav_NavEKF::get_velocity_z() const
{
    if (_ahrs.have_inertial_nav()) {
        return _velocity_cm.z;
    }
    return AP_InertialNav::get_velocity_z();
}

#endif // AP_AHRS_NAVEKF_AVAILABLE
