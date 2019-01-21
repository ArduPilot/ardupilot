/*
  A wrapper around the AP_InertialNav class which uses the NavEKF
  filter if available, and falls back to the AP_InertialNav filter
  when EKF is not available
 */
#pragma once

#include <AP_NavEKF/AP_Nav_Common.h>              // definitions shared by inertial and ekf nav filters

class AP_InertialNav_NavEKF : public AP_InertialNav
{
public:
    // Constructor
    AP_InertialNav_NavEKF(AP_AHRS_NavEKF &ahrs) :
        AP_InertialNav(),
        _haveabspos(false),
        _ahrs_ekf(ahrs)
        {}

    /**
       update internal state
    */
    void        update(float dt) override;

    /**
     * get_filter_status - returns filter status as a series of flags
     */
    nav_filter_status get_filter_status() const override;

    /**
     * get_origin - returns the inertial navigation origin in lat/lon/alt
     *
     * @return origin Location
     */
    struct Location get_origin() const override;

    /**
     * get_position - returns the current position relative to the home location in cm.
     *
     * the home location was set with AP_InertialNav::set_home_position(int32_t, int32_t)
     *
     * @return
     */
    const Vector3f&    get_position() const override;

    /**
     * get_llh - updates the provided location with the latest calculated location including absolute altitude
     *  returns true on success (i.e. the EKF knows it's latest position), false on failure
     */
    bool get_location(struct Location &loc) const override;

    /**
     * get_latitude - returns the latitude of the current position estimation in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
     */
    int32_t     get_latitude() const override;

    /**
     * get_longitude - returns the longitude of the current position estimation in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
     * @return
     */
    int32_t     get_longitude() const override;

    /**
     * get_velocity - returns the current velocity in cm/s
     *
     * @return velocity vector:
     *      		.x : latitude  velocity in cm/s
     * 				.y : longitude velocity in cm/s
     * 				.z : vertical  velocity in cm/s
     */
    const Vector3f&    get_velocity() const override;

    /**
     * get_pos_z_derivative - returns the derivative of the z position in cm/s
    */
    float    get_pos_z_derivative() const;

    /**
     * get_velocity_xy - returns the current horizontal velocity in cm/s
     *
     * @returns the current horizontal velocity in cm/s
     */
    float        get_velocity_xy() const override;

    /**
     * get_altitude - get latest altitude estimate in cm
     * @return
     */
    float       get_altitude() const override;

    /**
     * get_velocity_z - returns the current climbrate.
     *
     * @see get_velocity().z
     *
     * @return climbrate in cm/s
     */
    float       get_velocity_z() const override;

private:
    Vector3f _relpos_cm;   // NEU
    Vector3f _velocity_cm; // NEU
    float _pos_z_rate;
    struct Location _abspos;
    bool _haveabspos;
    AP_AHRS_NavEKF &_ahrs_ekf;
};
