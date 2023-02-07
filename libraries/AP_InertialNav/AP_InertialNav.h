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
     * get_position_neu_cm - returns the current position relative to the EKF origin in cm.
     *
     * @return
     */
    const Vector3f&    get_position_neu_cm() const;

    /**
     * get_position_xy_cm - returns the current x-y position relative to the EKF origin in cm.
     *
     * @return
     */
    const Vector2f&    get_position_xy_cm() const;

    /**
     * get_position_z_up_cm - returns the current z position relative to the EKF origin, frame z up, in cm.
     * @return
     */
    float              get_position_z_up_cm() const;

    /**
     * get_velocity_neu_cms - returns the current velocity in cm/s
     *
     * @return velocity vector:
     *      		.x : latitude  velocity in cm/s
     * 				.y : longitude velocity in cm/s
     * 				.z : vertical  velocity in cm/s
     */
    const Vector3f&    get_velocity_neu_cms() const;

    /**
     * get_velocity_xy_cms - returns the current x-y velocity relative to the EKF origin in cm.
     *
     * @return
     */
    const Vector2f& get_velocity_xy_cms() const;

    /**
     * get_speed_xy_cms - returns the current horizontal speed in cm/s
     *
     * @returns the current horizontal speed in cm/s
     */
    float        get_speed_xy_cms() const;

    /**
     * get_velocity_z_up_cms - returns the current z-axis velocity, frame z-axis up, in cm/s
     *
     * @return z-axis velocity, frame z-axis up, in cm/s
     */
    float       get_velocity_z_up_cms() const;

private:
    Vector3f _relpos_cm;   // NEU
    Vector3f _velocity_cm; // NEU
    AP_AHRS &_ahrs_ekf;
};
