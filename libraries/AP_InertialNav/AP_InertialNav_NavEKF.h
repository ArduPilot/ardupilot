/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  A wrapper around the AP_InertialNav class which uses the NavEKF
  filter if available, and falls back to the AP_InertialNav filter
  when EKF is not available
 */


#ifndef __AP_INERTIALNAV_NAVEKF_H__
#define __AP_INERTIALNAV_NAVEKF_H__

class AP_InertialNav_NavEKF : public AP_InertialNav
{
public:
    // Constructor
    AP_InertialNav_NavEKF(AP_AHRS_NavEKF &ahrs, AP_Baro &baro, GPS_Glitch& gps_glitch, Baro_Glitch& baro_glitch) :
        AP_InertialNav(ahrs, baro, gps_glitch, baro_glitch),
        _haveabspos(false),
        _ahrs_ekf(ahrs)
        {
        }

    /**
     * initializes the object.
     */
    void        init();

    /**
       update internal state
    */
    void        update(float dt);

    /**
     * position_ok - true if inertial based altitude and position can be trusted
     * @return
     */
    bool        position_ok() const;

    /**
     * get_position - returns the current position relative to the home location in cm.
     *
     * the home location was set with AP_InertialNav::set_home_position(int32_t, int32_t)
     *
     * @return
     */
    const Vector3f&    get_position() const;

    /**
     * get_latitude - returns the latitude of the current position estimation in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
     */
    int32_t     get_latitude() const;

    /**
     * get_longitude - returns the longitude of the current position estimation in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
     * @return
     */
    int32_t     get_longitude() const;

    /**
     * get_latitude_diff - returns the current latitude difference from the home location.
     *
     * @return difference in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
     */
    float       get_latitude_diff() const;

    /**
     * get_longitude_diff - returns the current longitude difference from the home location.
     *
     * @return difference in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
     */
    float       get_longitude_diff() const;

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
     * get_velocity_xy - returns the current horizontal velocity in cm/s
     *
     * @returns the current horizontal velocity in cm/s
     */
    float        get_velocity_xy() const;

    /**
     * altitude_ok - returns true if inertial based altitude and position can be trusted
     * @return
     */
    bool        altitude_ok() const;

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
    struct Location _abspos;
    bool _haveabspos;
    AP_AHRS_NavEKF &_ahrs_ekf;
};

#endif // __AP_INERTIALNAV_NAVEKF_H__
