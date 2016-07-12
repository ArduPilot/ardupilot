/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>
#include "AP_InertialNav.h"

#if AP_AHRS_NAVEKF_AVAILABLE

/*
  A wrapper around the AP_InertialNav class which uses the NavEKF
  filter if available, and falls back to the AP_InertialNav filter
  when EKF is not available
 */

/**
   update internal state
*/
void AP_InertialNav_NavEKF::update(float dt)
{
    // get the position relative to the local earth frame origin
    Vector3f relpos_m = Vector3f(0.0f,0.0f,0.0f);
    _ahrs_ekf.get_relative_position_NED(relpos_m);
    _relpos_cm = relpos_m*100;
    _relpos_cm.z = - _relpos_cm.z; // InertialNav is NEU

    // get the absolute WGS-84 position
    _ahrs_ekf.get_position(_abspos);

    // get the velocity relative to the local earth frame
    Vector3f velocity_m = Vector3f(0.0f,0.0f,0.0f);
    _ahrs_ekf.get_velocity_NED(velocity_m);
    _velocity_cm = velocity_m*100;
    _velocity_cm.z = - _velocity_cm.z; // InertialNav is NEU

    // Get a derivative of the vertical position which is kinematically consistent with the vertical position is required by some control loops.
    // This is different to the vertical velocity from the EKF which is not always consistent with the verical position due to the various errors that are being corrected for.
    float pos_z_rate_m;
    _ahrs_ekf.get_vert_pos_rate(pos_z_rate_m);
    _pos_z_rate_cm = -pos_z_rate_m*100; // convert from meters down to centimeters up
}

/**
 * get_filter_status : returns filter status as a series of flags
 */
nav_filter_status AP_InertialNav_NavEKF::get_filter_status() const
{
    nav_filter_status status;
    _ahrs_ekf.get_filter_status(status);
    return status;
}

/**
 * get_origin - returns the inertial navigation origin in lat/lon/alt
 */
struct Location AP_InertialNav_NavEKF::get_origin() const
{
    struct Location ret;
     if (!_ahrs_ekf.get_origin(ret)) {
         // initialise location to all zeros if EKF1 origin not yet set
         memset(&ret, 0, sizeof(ret));
     }
    return ret;
}

/**
 * get_position - returns the current position relative to the home location in cm.
 *
 * @return
 */
const Vector3f &AP_InertialNav_NavEKF::get_position(void) const 
{
    return _relpos_cm;
}

/**
 * get_location - updates the provided location with the latest calculated location
 *  returns true on success (i.e. the EKF knows it's latest position), false on failure
 */
bool AP_InertialNav_NavEKF::get_location(struct Location &loc) const
{
    return _ahrs_ekf.get_location(loc);
}

/**
 * get_latitude - returns the latitude of the current position estimation in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
 */
int32_t AP_InertialNav_NavEKF::get_latitude() const
{
    return _abspos.lat;
}

/**
 * get_longitude - returns the longitude of the current position estimation in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
 * @return
 */
int32_t AP_InertialNav_NavEKF::get_longitude() const
{
    return _abspos.lng;
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
    return _velocity_cm;
}

/**
 * get_velocity_xy - returns the current horizontal velocity in cm/s
 *
 * @returns the current horizontal velocity in cm/s
 */
float AP_InertialNav_NavEKF::get_velocity_xy() const
{
    return norm(_velocity_cm.x, _velocity_cm.y);
}

/**
 * get_pos_z_derivative - returns the derivative of the z position in cm/s
*/
float AP_InertialNav_NavEKF::get_pos_z_derivative() const
{
    return _pos_z_rate_cm;
}

/**
 * get_altitude - get latest altitude estimate in cm
 * @return
 */
float AP_InertialNav_NavEKF::get_altitude() const
{
    return _relpos_cm.z;
}

/**
 * getHgtAboveGnd - get latest height above ground level estimate in cm and a validity flag
 *
 * @return
 */
bool AP_InertialNav_NavEKF::get_hagl(float &height) const
{
    // true when estimate is valid
    bool valid = _ahrs_ekf.get_hagl(height);
    // convert height from m to cm
    height *= 100.0f;
    return valid;
}

/**
 * get_hgt_ctrl_limit - get maximum height to be observed by the control loops in cm and a validity flag
 * this is used to limit height during optical flow navigation
 * it will return invalid when no limiting is required
 * @return
 */
bool AP_InertialNav_NavEKF::get_hgt_ctrl_limit(float& limit) const
{
    // true when estimate is valid
    if (_ahrs_ekf.get_hgt_ctrl_limit(limit)) {
        // convert height from m to cm
        limit *= 100.0f;
        return true;
    }
    return false;
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
    return _velocity_cm.z;
}

#endif // AP_AHRS_NAVEKF_AVAILABLE
