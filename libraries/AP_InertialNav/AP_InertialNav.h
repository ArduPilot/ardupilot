/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIALNAV_H__
#define __AP_INERTIALNAV_H__

#include <AP_AHRS.h>
#include <AP_InertialSensor.h>          // ArduPilot Mega IMU Library
#include <AP_Baro.h>                    // ArduPilot Mega Barometer Library
#include <AP_Buffer.h>                  // FIFO buffer library

#define AP_INTERTIALNAV_GRAVITY 9.80665
#define AP_INTERTIALNAV_TC_XY   3.0 // default time constant for complementary filter's X & Y axis
#define AP_INTERTIALNAV_TC_Z    3.0 // default time constant for complementary filter's Z axis

// #defines to control how often historical accel based positions are saved
// so they can later be compared to laggy gps readings
#define AP_INTERTIALNAV_SAVE_POS_AFTER_ITERATIONS   10
#define AP_INTERTIALNAV_GPS_LAG_IN_10HZ_INCREMENTS  4       // must not be larger than size of _hist_position_estimate_x and _hist_position_estimate_y

/*
 * AP_InertialNav is an attempt to use accelerometers to augment other sensors to improve altitud e position hold
 */
class AP_InertialNav
{
public:

    // Constructor
    AP_InertialNav( AP_AHRS* ahrs, AP_InertialSensor* ins, AP_Baro* baro, GPS** gps_ptr ) :
        _ahrs(ahrs),
        _ins(ins),
        _baro(baro),
        _gps_ptr(gps_ptr),
        _xy_enabled(false),
        _gps_last_update(0),
        _baro_last_update(0)
        {}

    // Initialisation
    virtual void        init();

    // save_params - save all parameters to eeprom
    virtual void        save_params();

    // update - updates velocities and positions using latest info from accelerometers;
    virtual void        update(float dt);

    //
    // XY Axis specific methods
    //

    // set time constant - set timeconstant used by complementary filter
    virtual void        set_time_constant_xy( float time_constant_in_seconds );

    // altitude_ok, position_ok - true if inertial based altitude and position can be trusted
    virtual bool        position_ok();

    // check_gps - check if new gps readings have arrived and use them to correct position estimates
    virtual void        check_gps();

    // correct_with_gps - modifies accelerometer offsets using gps.  dt is time since last gps update
    virtual void        correct_with_gps(int32_t lon, int32_t lat, float dt);

    // get latitude & longitude positions
    virtual int32_t     get_latitude();
    virtual int32_t     get_longitude();

    // set_current_position - all internal calculations are recorded as the distances from this point
    virtual void        set_current_position(int32_t lon, int32_t lat);

    // get latitude & longitude positions from base location (in cm)
    virtual float       get_latitude_diff();
    virtual float       get_longitude_diff();
    
    // get velocity in latitude & longitude directions (in cm/s)
    virtual float       get_latitude_velocity();
    virtual float       get_longitude_velocity();

    // set velocity in latitude & longitude directions (in cm/s)
    virtual void        set_velocity_xy(float x, float y);

    //
    // Z Axis methods
    //

    // set time constant - set timeconstant used by complementary filter
    virtual void        set_time_constant_z( float time_constant_in_seconds );

    // altitude_ok, position_ok - true if inertial based altitude and position can be trusted
    virtual bool        altitude_ok() { return true; }

    // check_baro - check if new baro readings have arrived and use them to correct vertical accelerometer offsets
    virtual void        check_baro();

    // correct_with_baro - modifies accelerometer offsets using barometer.  dt is time since last baro reading
    virtual void        correct_with_baro(float baro_alt, float dt);

    // get_altitude - get latest altitude estimate in cm
    virtual float       get_altitude() { return _position_base.z + _position_correction.z; }
    virtual void        set_altitude( float new_altitude);

    // get_velocity_z - get latest climb rate (in cm/s)
    virtual float       get_velocity_z() { return _velocity.z; }
    virtual void        set_velocity_z( float new_velocity );

    // class level parameters
    static const struct AP_Param::GroupInfo var_info[];

    // public variables
    AP_Vector3f             accel_correction;          // acceleration corrections
    Vector3f                accel_correction_ef;        // earth frame accelerometer corrections. here for logging purposes only

protected:

    virtual void            update_gains();             // update_gains - update gains from time constant (given in seconds)

    AP_AHRS*                _ahrs;                      // pointer to ahrs object
    AP_InertialSensor*      _ins;                       // pointer to inertial sensor
    AP_Baro*                _baro;                      // pointer to barometer
    GPS**                   _gps_ptr;                   // pointer to pointer to gps

    // XY Axis specific variables
    bool                    _xy_enabled;                // xy position estimates enabled
    AP_Float                _time_constant_xy;          // time constant for horizontal corrections
    float                   _k1_xy;                     // gain for horizontal position correction
    float                   _k2_xy;                     // gain for horizontal velocity correction
    float                   _k3_xy;                     // gain for horizontal accelerometer offset correction
    uint32_t                _gps_last_update;           // system time of last gps update
    uint32_t                _gps_last_time;             // time of last gps update according to the gps itself
    uint8_t                 _historic_xy_counter;       // counter used to slow saving of position estimates for later comparison to gps
    AP_BufferFloat_Size5    _hist_position_estimate_x;  // buffer of historic accel based position to account for lag
    AP_BufferFloat_Size5    _hist_position_estimate_y;  // buffer of historic accel based position to account for lag
    int32_t                 _base_lat;                  // base latitude
    int32_t                 _base_lon;                  // base longitude
    float                   _lon_to_m_scaling;          // conversion of longitude to meters
    
    // Z Axis specific variables
    AP_Float                _time_constant_z;           // time constant for vertical corrections
    float                   _k1_z;                      // gain for vertical position correction
    float                   _k2_z;                      // gain for vertical velocity correction
    float                   _k3_z;                      // gain for vertical accelerometer offset correction
    uint32_t                _baro_last_update;           // time of last barometer update
    AP_BufferFloat_Size15   _hist_position_estimate_z;  // buffer of historic accel based altitudes to account for lag

    // general variables
    Vector3f                _position_base;             // position estimate
    Vector3f                _position_correction;       // sum of correction to _comp_h from delayed 1st order samples    
    Vector3f                _velocity;                  // latest velocity estimate (integrated from accelerometer values)
};

#endif // __AP_INERTIALNAV_H__
