/*
  A wrapper around the AP_InertialNav class which uses the NavEKF
  filter if available, and falls back to the AP_InertialNav filter
  when EKF is not available
 */
#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_NavEKF/AP_Nav_Common.h>              // definitions shared by inertial and ekf nav filters

class AP_InertialNav
{
public:
    // Constructor
    AP_InertialNav(AP_AHRS &ahrs) :
        _ahrs_ekf(ahrs)
        {}

    /**
       update internal state
    */
    void        update(bool high_vibes = false);

    /**
     * get_filter_status - returns filter status as a series of flags
     */
    nav_filter_status get_filter_status() const;

    /**
     * get_position - returns the current position relative to the home location in cm.
     *
     * the home location was set with AP_InertialNav::set_home_position(int32_t, int32_t)
     *
     * @return
     */
    const Vector3f&    get_position() const;

    /**
     * get_position_xy - returns the current x-y position relative to the home location in cm.
     *
     * @return
     */
    const Vector2f&    get_position_xy() const;

    /**
     * get_velocity - returns the current velocity in cm/s
     *
     * @return velocity vector:
     *      		.x : latitude  velocity in cm/s
     * 				.y : longitude velocity in cm/s
     * 				.z : vertical  velocity in cm/s
     */
    const Vector3f&    get_velocity() const;

    /**
     * get_velocity_xy - returns the current x-y velocity relative to the home location in cm.
     *
     * @return
     */
    const Vector2f& get_velocity_xy() const;

    /**
     * get_speed_xy - returns the current horizontal speed in cm/s
     *
     * @returns the current horizontal speed in cm/s
     */
    float        get_speed_xy() const;

    /**
     * get_altitude - get latest altitude estimate in cm
     * @return
     */
    float       get_altitude() const;

    /**
     * get_velocity_z - returns the current climbrate.
     *
     * @see get_velocity().z
     *
     * @return climbrate in cm/s
     */
    float       get_velocity_z() const;

private:
    Vector3f _relpos_cm;   // NEU
    Vector3f _velocity_cm; // NEU
    AP_AHRS &_ahrs_ekf;
};
