/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIALNAV_H__
#define __AP_INERTIALNAV_H__

#include <AP_AHRS.h>
#include <AP_InertialSensor.h>          // ArduPilot Mega IMU Library
#include <AP_Baro.h>                    // ArduPilot Mega Barometer Library
#include <ThirdOrderCompFilter.h>     // Complementary filter for combining barometer altitude with accelerometers

#define AP_INTERTIALNAV_GRAVITY 9.80665
#define AP_INTERTIALNAV_TC_XY   3.0 // default time constant for complementary filter's X & Y axis
#define AP_INTERTIALNAV_TC_Z    3.0 // default time constant for complementary filter's Z axis

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
        _baro_last_update(0),
        _gps_last_update(0),
        _xy_enabled(false),
        _comp_filter(AP_INTERTIALNAV_TC_XY, AP_INTERTIALNAV_TC_Z)
        {}

    // Initialisation
    virtual void        init() { 
                            set_time_constant_xy(_time_constant_xy);
                            set_time_constant_z(_time_constant_z); 
        }

    // save_params - save all parameters to eeprom
    virtual void        save_params();

    // set time constant - set timeconstant used by complementary filter
    virtual void        set_time_constant_xy( float time_constant_in_seconds );

    // set time constant - set timeconstant used by complementary filter
    virtual void        set_time_constant_z( float time_constant_in_seconds );

    // check_baro - check if new baro readings have arrived and use them to correct vertical accelerometer offsets
    virtual void        check_baro();

    // correct_with_baro - modifies accelerometer offsets using barometer.  dt is time since last baro reading
    virtual void        correct_with_baro(float baro_alt, float dt);

    // set_current_position - all internal calculations are recorded as the distances from this point
    virtual void        set_current_position(int32_t lon, int32_t lat);

    // check_gps - check if new gps readings have arrived and use them to correct position estimates
    virtual void        check_gps();

    // correct_with_gps - modifies accelerometer offsets using gps.  dt is time since last gps update
    virtual void        correct_with_gps(int32_t lon, int32_t lat, float dt);

    // update - updates velocities and positions using latest info from accelerometers;
    virtual void        update(float dt);

    // altitude_ok, position_ok - true if inertial based altitude and position can be trusted
    virtual bool        altitude_ok() { return true; }
    virtual bool        position_ok();

    // get_altitude - get latest altitude estimate in cm
    virtual float       get_altitude() { return _position.z; }
    virtual void        set_altitude( int32_t new_altitude) { _comp_filter.set_3rd_order_z(new_altitude); }

    // get_velocity_z - get latest climb rate (in cm/s)
    virtual float       get_velocity_z() { return _velocity.z; }
    virtual void        set_velocity_z( int32_t new_velocity ) { _comp_filter.set_2nd_order_z(new_velocity); }

    // get latitude & longitude positions
    virtual int32_t     get_latitude();
    virtual int32_t     get_longitude();

        // get latitude & longitude positions from base location
    virtual float       get_latitude_diff();
    virtual float       get_longitude_diff();
    
    // get velocity in latitude & longitude directions
    virtual float       get_latitude_velocity();
    virtual float       get_longitude_velocity();

    // class level parameters
    static const struct AP_Param::GroupInfo var_info[];

//protected:
    AP_AHRS*        _ahrs;
    AP_InertialSensor*  _ins;
    AP_Baro*        _baro;
    GPS**           _gps_ptr;

    uint32_t        _baro_last_update;
    uint32_t        _gps_last_update;       // system time of last gps update
    uint32_t        _gps_last_time;         // time of last gps update according to the gps itself

    bool            _xy_enabled;

    AP_Float        _time_constant_xy;      // time constant for complementary filter's horizontal corrections
    AP_Float        _time_constant_z;       // time constant for complementary filter's vertical corrections
    Vector3f        _accel_bf;              // latest accelerometer values
    Vector3f        _accel_ef;              // accelerometer values converted from body to earth frame
    AP_Vector3f     _accel_correction;      // accelerometer corrections calculated by complementary filter
    Vector3f        _velocity;              // latest velocity estimate (integrated from accelerometer values)
    Vector3f        _position;              // latest position estimate
    int32_t         _base_lat;              // base latitude
    int32_t         _base_lon;              // base longitude
    float           _lon_to_m_scaling;      // conversion of longitude to meters

    ThirdOrderCompFilter   _comp_filter;   // 3rd order complementary filter for combining baro readings with accelerometers

};

#endif // __AP_INERTIALNAV_H__
