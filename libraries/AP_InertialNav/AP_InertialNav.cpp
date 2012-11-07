/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <FastSerial.h>
#include <AP_InertialNav.h>

#if defined(ARDUINO) && ARDUINO >= 100
 #include "Arduino.h"
#else
 #include <wiring.h>
#endif

// table of user settable parameters
const AP_Param::GroupInfo AP_InertialNav::var_info[] PROGMEM = {
	// index 0 and 1 are for old parameters that are no longer used

    // @Param: ACORR
    // @DisplayName: Inertial Nav calculated accelerometer corrections
    // @Description: calculated accelerometer corrections
    // @Increment: 1
    AP_GROUPINFO("ACORR",   0, AP_InertialNav, _accel_correction, 0),

    // @Param: TC_XY
    // @DisplayName: Horizontal Time Constant
    // @Description: Time constnat for GPS and accel mixing. Higher TC increases GPS impact on position
    // @Range: 0 10
    // @Increment: 0.1
    AP_GROUPINFO("TC_XY",   1, AP_InertialNav, _time_constant_xy, AP_INTERTIALNAV_TC_XY),

    // @Param: TC_Z
    // @DisplayName: Vertical Time Constant
    // @Description: Time constnat for baro and accel mixing. Higher TC increases barometers impact on altitude
    // @Range: 0 10
    // @Increment: 0.1
    AP_GROUPINFO("TC_Z",    2, AP_InertialNav, _time_constant_z, AP_INTERTIALNAV_TC_Z),

    AP_GROUPEND
};

// save_params - save all parameters to eeprom
void AP_InertialNav::save_params()
{
    Vector3f accel_corr = _comp_filter.get_1st_order_correction();
    accel_corr.x = constrain(accel_corr.x,-200,200);
    accel_corr.y = constrain(accel_corr.y,-200,200);
    accel_corr.z = constrain(accel_corr.z,-200,200);
    _accel_correction.set_and_save(accel_corr);
}

// set time constant - set timeconstant used by complementary filter
void AP_InertialNav::set_time_constant_xy( float time_constant_in_seconds )
{
    // ensure it's a reasonable value
    if( time_constant_in_seconds > 0 && time_constant_in_seconds < 30 ) {
        _time_constant_xy = time_constant_in_seconds;
        _comp_filter.update_gains(_time_constant_xy, _time_constant_z);
    }
}

// set time constant - set timeconstant used by complementary filter
void AP_InertialNav::set_time_constant_z( float time_constant_in_seconds )
{
    // ensure it's a reasonable value
    if( time_constant_in_seconds > 0 && time_constant_in_seconds < 30 ) {
        _time_constant_z = time_constant_in_seconds;
        _comp_filter.update_gains(_time_constant_xy, _time_constant_z);
    }
}

// check_baro - check if new baro readings have arrived and use them to correct vertical accelerometer offsets
void AP_InertialNav::check_baro()
{
    uint32_t baro_update_time;

    if( _baro == NULL )
        return;

    // calculate time since last baro reading
    baro_update_time = _baro->get_last_update();
    if( baro_update_time != _baro_last_update ) {
        float dt = (float)(baro_update_time - _baro_last_update) / 1000.0;
        // call correction method
        correct_with_baro(_baro->get_altitude()*100, dt);
        _baro_last_update = baro_update_time;
    }
}


// correct_with_baro - modifies accelerometer offsets using barometer.  dt is time since last baro reading
void AP_InertialNav::correct_with_baro(float baro_alt, float dt)
{
    static uint8_t first_reads = 0;

    // discard samples where dt is too large
    if( dt > 0.2 ) {
        return;
    }

    // discard first 10 reads but perform some initialisation
    if( first_reads <= 10 ) {
        _comp_filter.set_3rd_order_z(baro_alt);
        //_comp_filter.set_2nd_order_z(climb_rate);
        first_reads++;
    }

    // get dcm matrix
    Matrix3f dcm = _ahrs->get_dcm_matrix();

    // provide baro alt to filter
    _comp_filter.correct_3rd_order_z(baro_alt, dcm, dt);
}

// set_current_position - all internal calculations are recorded as the distances from this point
void AP_InertialNav::set_current_position(int32_t lon, int32_t lat)
{
    // set base location
    _base_lon = lon;
    _base_lat = lat;

    // set longitude->meters scaling
    // this is used to offset the shrinking longitude as we go towards the poles
    _lon_to_m_scaling = cos((fabs((float)lat)/10000000.0) * 0.0174532925);

    // set estimated position to this position
    _comp_filter.set_3rd_order_xy(0,0);

    // set xy as enabled
    _xy_enabled = true;
}

// check_gps - check if new gps readings have arrived and use them to correct position estimates
void AP_InertialNav::check_gps()
{
    uint32_t gps_time;
    uint32_t now;

    if( _gps_ptr == NULL || *_gps_ptr == NULL )
        return;

    // get time according to the gps
    gps_time = (*_gps_ptr)->time;

    // compare gps time to previous reading
    if( gps_time != _gps_last_time ) {

        // calculate time since last gps reading
        now = millis();
        float dt = (float)(now - _gps_last_update) / 1000.0;

        // call position correction method
        correct_with_gps((*_gps_ptr)->longitude, (*_gps_ptr)->latitude, dt);

        // record gps time and system time of this update
        _gps_last_time = gps_time;
        _gps_last_update = now;
    }
}

// correct_with_gps - modifies accelerometer offsets using gps.  dt is time since last gps update
void AP_InertialNav::correct_with_gps(int32_t lon, int32_t lat, float dt)
{
    float x,y;

    // discard samples where dt is too large
    if( dt > 1.0 || dt == 0 || !_xy_enabled) {
        return;
    }

    // calculate distance from home
    //x = (float)(lat - _base_lat) * 1.113195;
    //y = (float)(lon - _base_lon) * _lon_to_m_scaling * 1.113195;
    x = (float)(lat - _base_lat);
    y = (float)(lon - _base_lon) * _lon_to_m_scaling;

    // convert accelerometer readings to earth frame
    Matrix3f dcm = _ahrs->get_dcm_matrix();

    // call comp filter's correct xy
    _comp_filter.correct_3rd_order_xy(-x, -y, dcm, dt);
    //Notes: with +x above, accel lat comes out reversed
}

// update - updates velocities and positions using latest info from ahrs, ins and barometer if new data is available;
void AP_InertialNav::update(float dt)
{
    // discard samples where dt is too large
    if( dt > 0.1 ) {
        return;
    }

    // check barometer
    check_baro();

    // check gps
    check_gps();

    // read acclerometer values
    _accel_bf = _ins->get_accel();

    // convert accelerometer readings to earth frame
    Matrix3f dcm = _ahrs->get_dcm_matrix();
    _accel_ef = dcm * _accel_bf;

    // remove influence of gravity
    _accel_ef.z += AP_INTERTIALNAV_GRAVITY;
    _accel_ef *= 100;

    // remove xy if not enabled
    if( !_xy_enabled ) {
        _accel_ef.x = 0;
        _accel_ef.y = 0;
    }

    // provide accelerometer values to filter
    _comp_filter.add_1st_order_sample(_accel_ef);

    // recalculate estimates
    _comp_filter.calculate(dt, dcm);
    
    // get position and velocity estimates
    _position = _comp_filter.get_3rd_order_estimate();
    _velocity = _comp_filter.get_2nd_order_estimate();
}

// position_ok - return true if position has been initialised and have received gps data within 3 seconds
bool AP_InertialNav::position_ok()
{
    return _xy_enabled && (millis() - _gps_last_update < 3000);
}

// get accel based latitude
int32_t AP_InertialNav::get_latitude()
{
    // make sure we've been initialised
    if( !_xy_enabled ) {
        return 0;
    }

    //return _base_lat - (int32_t)(_position.x / 1.113195);
    return _base_lat - (int32_t)_position.x;
}

// get accel based longitude
int32_t AP_InertialNav::get_longitude()
{
    // make sure we've been initialised
    if( !_xy_enabled ) {
        return 0;
    }

    //return _base_lon - (int32_t)(_position.y / (_lon_to_m_scaling * 1.113195) );
    return _base_lon - (int32_t)(_position.y / _lon_to_m_scaling );
}

// get accel based latitude
float AP_InertialNav::get_latitude_diff()
{
    // make sure we've been initialised
    if( !_xy_enabled ) {
        return 0;
    }

    //return _base_lat + (int32_t)_position.x;
    //return -_position.x / 1.113195;
    return -_position.x;
}

// get accel based longitude
float AP_InertialNav::get_longitude_diff()
{
    // make sure we've been initialised
    if( !_xy_enabled ) {
        return 0;
    }

    //return _base_lon - (int32_t)(_position.x / _lon_to_m_scaling);
    //return -_position.y / (_lon_to_m_scaling * 1.113195);
    return -_position.y / _lon_to_m_scaling;
}

// get velocity in latitude & longitude directions
float AP_InertialNav::get_latitude_velocity()
{
    // make sure we've been initialised
    if( !_xy_enabled ) {
        return 0;
    }

    return -_velocity.x;
    // Note: is +_velocity.x the output velocity in logs is in reverse direction from accel lat
}

float AP_InertialNav::get_longitude_velocity()
{
    // make sure we've been initialised
    if( !_xy_enabled ) {
        return 0;
    }

    return -_velocity.y;
}