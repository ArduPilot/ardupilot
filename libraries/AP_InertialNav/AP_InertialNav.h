/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIALNAV_H__
#define __AP_INERTIALNAV_H__

#include <AP_AHRS.h>
#include <AP_InertialSensor.h>          // ArduPilot Mega IMU Library
#include <AP_Baro.h>                    // ArduPilot Mega Barometer Library
#include <AP_Buffer.h>                  // FIFO buffer library

#define AP_INAV_TC_XY   2.5f // default time constant for complementary filter's X & Y axis
#define AP_INAV_TC_Z    5.0f // default time constant for complementary filter's Z axis


/* #defines to control how often historical accel based positions are saved
 * so they can later be compared to laggy gps and baro readings
 ********************************************************************************/

#define AP_INAV_GPS_DELAY_MS   400
#define AP_INAV_BARO_DELAY_MS  150

// 100Hz --> 10ms period
#define AP_INAV_MAIN_LOOP_PERIOD_MS 10

// 150ms delay --> 15 iterations at 100Hz
#define AP_INAV_BARO_DELAY_QUEUE_SIZE (AP_INAV_BARO_DELAY_MS / AP_INAV_MAIN_LOOP_PERIOD_MS)

/* since the GPS update rate is pretty low (<= 5Hz), we don't store the accel based positions at 100Hz (main loop freq.)
 * but at a lower rate determined by this divider.
 * --> 10 Hz (=100Hz / 10)
 */
#define AP_INAV_SAVE_POS_AFTER_ITERATIONS   10


/* GPS has 400ms delay --> we need to store 4 historical accel based position values
 * (values are written at 10Hz <=> every 10th iteration at 100Hz)
 */
#define AP_INAV_GPS_DELAY_QUEUE_SIZE (  AP_INAV_GPS_DELAY_MS            \
		                              / AP_INAV_MAIN_LOOP_PERIOD_MS     \
		                              / AP_INAV_SAVE_POS_AFTER_ITERATIONS )


// timeout after which position error from GPS will fall to zero
#define AP_INAV_GPS_TIMEOUT_MS              300

/**
 *
 * AP_InertialNav is an attempt to use accelerometers to augment other sensors
 * to improve altitude and position hold.
 *
 * Most of the functions have to be called at 100Hz. (see defines above)
 *
 * Basic idea:
 * -----------
 * - The accelerometer values are integrated over time to approximate velocity and position.
 * - The inaccurcy of these estimates grows over time due to noisy sensor data.
 * - To improve the accuracy, baro and gps readings are used:
 * 		- An error value is calculated as the difference between the sensor's measurement and
 * 		  the last position estimation.
 *   	- This value is weighted with a gain factor and incorporated into the new estimation
 *
 */
class AP_InertialNav
{
public:

    // Constructor
    AP_InertialNav( const AP_AHRS* ahrs, AP_InertialSensor* ins, AP_Baro* baro, GPS** gps_ptr ) :
        _ahrs(ahrs),
        _baro(baro),
        _gps_ptr(gps_ptr),
        _xy_enabled(false),
        _gps_last_update(0),
        _gps_last_time(0),
        _historic_xy_counter(0),
        _baro_last_update(0)
		// all other members (_k1_xy, _k2_xy, _k3_xy; _k1_z, _k2_z, _k3_z; _base_lat, _base_lon, _lon_to_cm_scaling)
		// are initialized by the update method that is called by init();
        {
            AP_Param::setup_object_defaults(this, var_info);
        }

    /**
     * initializes the object.
     *
     * AP_InertialNav::set_home_location(int32_t, int32_t) should be called later,
     * to enable all "horizontal related" getter-methods.
     */
    void        init();

    /**
     * updates velocity and position estimations using latest info from accelerometers
     * augmented with gps and baro readings
     *
     * @param dt : time since last update in s
     */
    void        update(float dt);

    //
    // XY Axis specific methods
    //

    /**
     * sets time constant used by complementary filter for horizontal position estimation.
     *
     * smaller values means higher influence of gps in position estimation
     * bigger values favor the integrated accelerometer data for position estimation
     *
     * @param time_constant_in_seconds : constant in s; 0 < constant < 30 must hold
     */
    void        set_time_constant_xy( float time_constant_in_seconds );

    /**
     * true if inertial based altitude and position can be trusted
     * @return
     */
    bool        position_ok() const;

    /**
     * checks if new gps readings have arrived and calls correct_with_gps to
     * calculate the horizontal position error
     * @see correct_with_gps(int32_t lon, int32_t lat, float dt);
     */
    void        check_gps();

    /**
     * calculates horizontal position error using gps
     *
     * @param lon : longitude in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
     * @param lat : latitude  in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
     * @param dt  : time since last gps update in seconds
     */
    void        correct_with_gps(int32_t lon, int32_t lat, float dt);

    /**
     * returns the current position relative to the home location in cm.
     *
     * the home location was set with AP_InertialNav::set_home_location(int32_t, int32_t)
     *
     * @return
     */
    const Vector3f&    get_position() const { return _position; }

    /**
     * returns the latitude of the current position estimation in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
     * @return
     */
    int32_t     get_latitude() const;

    /**
     * returns the longitude of the current position estimation in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
     * @return
     */
    int32_t     get_longitude() const;

    /**
     * sets home location.
     *
     * all internal calculations are recorded as the distances from this point.
     *
     * @param lon : longitude in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
     * @param lat : latitude  in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
     */
    void        set_home_location(int32_t lon, int32_t lat);

    /**
     * returns the current latitude difference from the home location.
     *
     * @return difference in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
     */
    float       get_latitude_diff() const;

    /**
     * returns the current longitude difference from the home location.
     *
     * @return difference in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
     */
    float       get_longitude_diff() const;
    
    /**
     * returns the current velocity in latitude direction.
     *
     * @see get_velocity().x
     *
     * @return : latitude velocity in cm/s
     */
    float       get_latitude_velocity() const;

    /**
     * returns the current velocity in longitude direction.
     *
     * @see get_velocity().y
     *
     * @return : longitude velocity in cm/s
     */
    float       get_longitude_velocity() const;

    /**
     * returns the current velocity.
     *
     * @return velocity vector:
     *      		.x : latitude  velocity in cm/s
     * 				.y : longitude velocity in cm/s
     * 				.z : vertical  velocity in cm/s
     */
    const Vector3f&    get_velocity() const { return _velocity; }

    /**
     * overwrites the current horizontal velocity values.
     *
     * @param x : latitude  velocity in cm/s
     * @param y : longitude velocity in cm/s
     */
    void        set_velocity_xy(float x, float y);

    /**
     * overwrites the current position relative to the home location in cm.
     *
     * the home location was set with AP_InertialNav::set_home_location(int32_t, int32_t)
     *
     * @param x : relative latitude  position in cm
     * @param y : relative longitude position in cm
     */
    void set_position_xy(float x, float y);

    //
    // Z Axis methods
    //

    /**
     * sets timeconstant used by complementary filter for vertical position estimation.
     *
     * smaller values means higher influence of barometer in altitude estimation
     * bigger values favor the integrated accelerometer data for altitude estimation
     *
     * @param time_constant_in_seconds : constant in s; 0 < constant < 30 must hold
     */
    void        set_time_constant_z( float time_constant_in_seconds );

    /**
     * returns true if inertial based altitude and position can be trusted.
     * @return
     */
    bool        altitude_ok() const { return true; }

    /**
     * checks if new baro readings have arrived and calls correct_with_baro to
     * calculate the vertical position error
     *
     * @see correct_with_baro(float baro_alt, float dt);
     */
    void        check_baro();

    /**
     * calculates vertical position error using barometer.
     *
     * @param baro_alt : altitude in cm
     * @param dt : time since last baro reading in s
     */
    void        correct_with_baro(float baro_alt, float dt);

    /**
     * get latest altitude estimate in cm
     * @return
     */
    float       get_altitude() const { return _position.z; }

    /**
     * overwrites the current altitude value.
     *
     * @param new_altitude : altitude in cm
     */
    void        set_altitude( float new_altitude);

    /**
     * returns the current climbrate.
     *
     * @see get_velocity().z
     *
     * @return climbrate in cm/s
     */
    float       get_velocity_z() const { return _velocity.z; }

    /**
     * overwrites the current climbrate.
     *
     * @param new_velocity : climbrate in cm/s
     */
    void        set_velocity_z( float new_velocity );

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

    const AP_AHRS*    const _ahrs;                      // pointer to ahrs object
    const AP_Baro*    const _baro;                      // pointer to barometer
    const GPS* const *const _gps_ptr;                   // pointer to pointer to gps

    // XY Axis specific variables
    bool                    _xy_enabled;                // xy position estimates enabled
    AP_Float                _time_constant_xy;          // time constant for horizontal corrections in s
    float                   _k1_xy;                     // gain for horizontal position correction
    float                   _k2_xy;                     // gain for horizontal velocity correction
    float                   _k3_xy;                     // gain for horizontal accelerometer offset correction
    uint32_t                _gps_last_update;           // system time of last gps update in ms
    uint32_t                _gps_last_time;             // time of last gps update according to the gps itself in ms
    uint8_t                 _historic_xy_counter;       // counter used to slow saving of position estimates for later comparison to gps
    typedef AP_Buffer<float,AP_INAV_GPS_DELAY_QUEUE_SIZE> GPS_Queue_t;
    GPS_Queue_t             _hist_position_estimate_x;  // buffer of historic accel based position to account for gps lag
    GPS_Queue_t             _hist_position_estimate_y;  // buffer of historic accel based position to account for gps lag
    int32_t                 _base_lat;                  // base latitude  (home location) in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
    int32_t                 _base_lon;                  // base longitude (home location) in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
    float                   _lon_to_cm_scaling;         // conversion of longitude to centimeters
    
    // Z Axis specific variables
    AP_Float                _time_constant_z;           // time constant for vertical corrections in s
    float                   _k1_z;                      // gain for vertical position correction
    float                   _k2_z;                      // gain for vertical velocity correction
    float                   _k3_z;                      // gain for vertical accelerometer offset correction
    uint32_t                _baro_last_update;          // time of last barometer update in ms
    typedef AP_Buffer<float, AP_INAV_BARO_DELAY_QUEUE_SIZE> Baro_Queue_t;
    Baro_Queue_t            _hist_position_estimate_z;  // buffer of historic accel based altitudes to account for barometer lag

    // general variables
    Vector3f                _position_base;             // (uncorrected) position estimate in cm - relative to the home location (_base_lat, _base_lon, 0)
    Vector3f                _position_correction;       // sum of corrections to _position_base from delayed 1st order samples in cm
    Vector3f                _velocity;                  // latest velocity estimate (integrated from accelerometer values) in cm/s
    Vector3f                _position_error;            // current position error in cm - is set by the check_* methods and used by update method to calculate the correction terms
    Vector3f                _position;                  // sum(_position_base, _position_correction) - corrected position estimate in cm - relative to the home location (_base_lat, _base_lon, 0)
};

#endif // __AP_INERTIALNAV_H__
