/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIALNAV_H__
#define __AP_INERTIALNAV_H__

#include <AP_AHRS.h>
#include <AP_InertialSensor.h>          // ArduPilot Mega IMU Library
#include <AP_Baro.h>                    // ArduPilot Mega Barometer Library
#include <AP_Buffer.h>                  // FIFO buffer library
#include <AP_GPS_Glitch.h>              // GPS Glitch detection library

#define AP_INTERTIALNAV_TC_XY   2.5f // default time constant for complementary filter's X & Y axis
#define AP_INTERTIALNAV_TC_Z    5.0f // default time constant for complementary filter's Z axis

// #defines to control how often historical accel based positions are saved
// so they can later be compared to laggy gps readings
#define AP_INTERTIALNAV_SAVE_POS_AFTER_ITERATIONS   10
#define AP_INTERTIALNAV_GPS_LAG_IN_10HZ_INCREMENTS  4       // must not be larger than size of _hist_position_estimate_x and _hist_position_estimate_y
#define AP_INTERTIALNAV_GPS_TIMEOUT_MS              300     // timeout after which position error from GPS will fall to zero

/*
 * AP_InertialNav blends accelerometer data with gps and barometer data to improve altitude and position hold.
 *
 * Most of the functions have to be called at 100Hz. (see defines above)
 *
 * The accelerometer values are integrated over time to approximate velocity and position.
 * The inaccurcy of these estimates grows over time due to noisy sensor data.
 * To improve the accuracy, baro and gps readings are used:
 *      An error value is calculated as the difference between the sensor's measurement and the last position estimation.
 *   	This value is weighted with a gain factor and incorporated into the new estimation
 */
class AP_InertialNav
{
public:

    // Constructor
    AP_InertialNav( const AP_AHRS* ahrs, AP_Baro* baro, GPS*& gps, GPS_Glitch& gps_glitch ) :
        _ahrs(ahrs),
        _baro(baro),
        _gps(gps),
        _xy_enabled(false),
        _gps_last_update(0),
        _gps_last_time(0),
        _historic_xy_counter(0),
        _baro_last_update(0),
        _glitch_detector(gps_glitch),
        _error_count(0)
        {
            AP_Param::setup_object_defaults(this, var_info);
        }

    /**
     * initializes the object.
     *
     * AP_InertialNav::set_home_position(int32_t, int32_t) should be called later,
     * to enable all "horizontal related" getter-methods.
     */
    void        init();

    /**
     * update - updates velocity and position estimates using latest info from accelerometers
     * augmented with gps and baro readings
     *
     * @param dt : time since last update in seconds
     */
    void        update(float dt);

    //
    // XY Axis specific methods
    //

    /**
     * set_time_constant_xy - sets time constant used by complementary filter for horizontal position estimate
     *
     * smaller values means higher influence of gps on position estimation
     * bigger values favor the integrated accelerometer data for position estimation
     *
     * @param time_constant_in_seconds : constant in seconds; 0 < constant < 30 must hold
     */
    void        set_time_constant_xy( float time_constant_in_seconds );

    /**
     * position_ok - true if inertial based altitude and position can be trusted
     * @return
     */
    bool        position_ok() const;

    /**
     * check_gps - checks if new gps readings have arrived and calls correct_with_gps to
     * calculate the horizontal position error
     * @see correct_with_gps(int32_t lon, int32_t lat, float dt);
     */
    void        check_gps();

    /**
     * correct_with_gps - calculates horizontal position error using gps
     *
     * @param now : current time since boot in milliseconds
     * @param lon : longitude in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
     * @param lat : latitude  in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
     */
    void        correct_with_gps(uint32_t now, int32_t lon, int32_t lat);

    /**
     * get_position - returns the current position relative to the home location in cm.
     *
     * the home location was set with AP_InertialNav::set_home_position(int32_t, int32_t)
     *
     * @return
     */
    const Vector3f&    get_position() const { return _position; }

    /**
     * get_latitude - returns the latitude of the current position estimation in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
     * @return
     */
    int32_t     get_latitude() const;

    /**
     * get_longitude - returns the longitude of the current position estimation in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
     * @return
     */
    int32_t     get_longitude() const;

    /**
     * set_home_position - sets home position
     *
     * all internal calculations are recorded as the distances from this point.
     *
     * @param lon : longitude in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
     * @param lat : latitude  in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
     */
    void        set_home_position(int32_t lon, int32_t lat);

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
    const Vector3f&    get_velocity() const { return _velocity; }

    /**
     * get_velocity_xy - returns the current horizontal velocity in cm/s
     *
     * @returns the current horizontal velocity in cm/s
     */
    float        get_velocity_xy();

    /**
     * set_velocity_xy - overwrites the current horizontal velocity in cm/s
     *
     * @param x : latitude  velocity in cm/s
     * @param y : longitude velocity in cm/s
     */
    void        set_velocity_xy(float x, float y);

    //
    // Z Axis methods
    //

    /**
     * set_time_constant_z - sets timeconstant used by complementary filter for vertical position estimation
     *
     * smaller values means higher influence of barometer in altitude estimation
     * bigger values favor the integrated accelerometer data for altitude estimation
     *
     * @param time_constant_in_seconds : constant in s; 0 < constant < 30 must hold
     */
    void        set_time_constant_z( float time_constant_in_seconds );

    /**
     * altitude_ok - returns true if inertial based altitude and position can be trusted
     * @return
     */
    bool        altitude_ok() const { return true; }

    /**
     * check_baro - checks if new baro readings have arrived and calls correct_with_baro to
     * calculate the vertical position error
     *
     * @see correct_with_baro(float baro_alt, float dt);
     */
    void        check_baro();

    /**
     * correct_with_baro - calculates vertical position error using barometer.
     *
     * @param baro_alt : altitude in cm
     * @param dt : time since last baro reading in s
     */
    void        correct_with_baro(float baro_alt, float dt);

    /**
     * get_altitude - get latest altitude estimate in cm
     * @return
     */
    float       get_altitude() const { return _position.z; }

    /**
     * set_altitude - overwrites the current altitude value.
     *
     * @param new_altitude : altitude in cm
     */
    void        set_altitude( float new_altitude);

    /**
     * get_velocity_z - returns the current climbrate.
     *
     * @see get_velocity().z
     *
     * @return climbrate in cm/s
     */
    float       get_velocity_z() const { return _velocity.z; }

    /**
     * set_velocity_z - overwrites the current climbrate.
     *
     * @param new_velocity : climbrate in cm/s
     */
    void        set_velocity_z( float new_velocity );

    /**
     * error_count - returns number of missed updates from GPS
     */
    uint8_t     error_count() const { return _error_count; }

    /**
     * ignore_next_error - the next error (if it occurs immediately) will not be added to the error count
     */
    void        ignore_next_error() { _flags.ignore_error = 7; }

    // class level parameters
    static const struct AP_Param::GroupInfo var_info[];

    // public variables
    Vector3f                accel_correction_ef;        // earth frame accelerometer corrections. here for logging purposes only

protected:

    /**
     * update gains from time constant.
     *
     * The time constants (in s) can be set with the following methods:
     *
     * @see: AP_InertialNav::set_time_constant_xy(float)
     * @see: AP_InertialNav::set_time_constant_z(float)
     */
    void                    update_gains();

    /**
     * set_position_xy - overwrites the current position relative to the home location in cm
     *
     * the home location was set with AP_InertialNav::set_home_location(int32_t, int32_t)
     *
     * @param x : relative latitude  position in cm
     * @param y : relative longitude position in cm
     */
    void set_position_xy(float x, float y);

    // structure for holding flags
    struct InertialNav_flags {
        uint8_t gps_glitching       : 1;                // 1 if glitch detector was previously indicating a gps glitch
        uint8_t ignore_error        : 3;                // the number of iterations for which we should ignore errors
    } _flags;

    const AP_AHRS*    const _ahrs;                      // pointer to ahrs object
    AP_Baro*                _baro;                      // pointer to barometer
    GPS*&                   _gps;                       // pointer to gps

    // XY Axis specific variables
    bool                    _xy_enabled;                // xy position estimates enabled
    AP_Float                _time_constant_xy;          // time constant for horizontal corrections in s
    float                   _k1_xy;                     // gain for horizontal position correction
    float                   _k2_xy;                     // gain for horizontal velocity correction
    float                   _k3_xy;                     // gain for horizontal accelerometer offset correction
    uint32_t                _gps_last_update;           // system time of last gps update in ms
    uint32_t                _gps_last_time;             // time of last gps update according to the gps itself in ms
    uint8_t                 _historic_xy_counter;       // counter used to slow saving of position estimates for later comparison to gps
    AP_BufferFloat_Size5    _hist_position_estimate_x;  // buffer of historic accel based position to account for gpslag
    AP_BufferFloat_Size5    _hist_position_estimate_y;  // buffer of historic accel based position to account for gps lag
    int32_t                 _base_lat;                  // base latitude  (home location) in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
    int32_t                 _base_lon;                  // base longitude (home location) in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
    float                   _lon_to_cm_scaling;         // conversion of longitude to centimeters

    // Z Axis specific variables
    AP_Float                _time_constant_z;           // time constant for vertical corrections in s
    float                   _k1_z;                      // gain for vertical position correction
    float                   _k2_z;                      // gain for vertical velocity correction
    float                   _k3_z;                      // gain for vertical accelerometer offset correction
    uint32_t                _baro_last_update;          // time of last barometer update in ms
    AP_BufferFloat_Size15   _hist_position_estimate_z;  // buffer of historic accel based altitudes to account for barometer lag

    // general variables
    Vector3f                _position_base;             // (uncorrected) position estimate in cm - relative to the home location (_base_lat, _base_lon, 0)
    Vector3f                _position_correction;       // sum of corrections to _position_base from delayed 1st order samples in cm
    Vector3f                _velocity;                  // latest velocity estimate (integrated from accelerometer values) in cm/s
    Vector3f                _position_error;            // current position error in cm - is set by the check_* methods and used by update method to calculate the correction terms
    Vector3f                _position;                  // sum(_position_base, _position_correction) - corrected position estimate in cm - relative to the home location (_base_lat, _base_lon, 0)

    // error handling
    GPS_Glitch&             _glitch_detector;           // GPS Glitch detector
    uint8_t                 _error_count;               // number of missed GPS updates

};

#endif // __AP_INERTIALNAV_H__
