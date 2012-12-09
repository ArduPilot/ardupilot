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
    // @Param: ACORR
    // @DisplayName: Inertial Nav calculated accelerometer corrections
    // @Description: calculated accelerometer corrections
    // @Increment: 1
    AP_GROUPINFO("ACORR",   0, AP_InertialNav, accel_correction, 0),

    // @Param: TC_XY
    // @DisplayName: Horizontal Time Constant
    // @Description: Time constant for GPS and accel mixing. Higher TC decreases GPS impact on position estimate
    // @Range: 0 10
    // @Increment: 0.1
    AP_GROUPINFO("TC_XY",   1, AP_InertialNav, _time_constant_xy, AP_INTERTIALNAV_TC_XY),

    // @Param: TC_Z
    // @DisplayName: Vertical Time Constant
    // @Description: Time constant for baro and accel mixing. Higher TC decreases barometers impact on altitude estimate
    // @Range: 0 10
    // @Increment: 0.1
    AP_GROUPINFO("TC_Z",    2, AP_InertialNav, _time_constant_z, AP_INTERTIALNAV_TC_Z),

    AP_GROUPEND
};

// init - initialise library
void AP_InertialNav::init()
{
    // recalculate the gains
    update_gains();
}

// save_params - save all parameters to eeprom
void AP_InertialNav::save_params()
{
    Vector3f accel_corr = accel_correction.get();
    accel_corr.x = constrain(accel_corr.x,-100,100);
    accel_corr.y = constrain(accel_corr.y,-100,100);
    accel_corr.z = constrain(accel_corr.z,-100,100);
    accel_correction.set_and_save(accel_corr);
}

// update - updates velocities and positions using latest info from ahrs, ins and barometer if new data is available;
void AP_InertialNav::update(float dt)
{
    Vector3f acc_corr = accel_correction.get();
    Vector3f accel_ef;

    // discard samples where dt is too large
    if( dt > 0.1 ) {
        return;
    }

    // check barometer
    check_baro();

    // check gps
    check_gps();

    // convert accelerometer readings to earth frame
    Matrix3f dcm = _ahrs->get_dcm_matrix();
    accel_ef = dcm * _ins->get_accel();

    // remove influence of gravity
    accel_ef.z += AP_INTERTIALNAV_GRAVITY;
    accel_ef *= 100;

    // remove xy if not enabled
    if( !_xy_enabled ) {
        accel_ef.x = 0;
        accel_ef.y = 0;
    }

    // get earth frame accelerometer correction
    accel_correction_ef = dcm * acc_corr;

    // calculate velocity by adding new acceleration from accelerometers
    _velocity += (-accel_ef + accel_correction_ef) * dt;

    // calculate new estimate of position
    _position_base += _velocity * dt;

    // store 3rd order estimate (i.e. estimated vertical position) for future use
    _hist_position_estimate_z.add(_position_base.z);

    // store 3rd order estimate (i.e. horizontal position) for future use at 10hz
    _historic_xy_counter++;
    if( _historic_xy_counter >= AP_INTERTIALNAV_SAVE_POS_AFTER_ITERATIONS ) {
        _historic_xy_counter = 0;
        _hist_position_estimate_x.add(_position_base.x);
        _hist_position_estimate_y.add(_position_base.y);
    }
}

//
// XY Axis specific methods
//

// set time constant - set timeconstant used by complementary filter
void AP_InertialNav::set_time_constant_xy( float time_constant_in_seconds )
{
    // ensure it's a reasonable value
    if( time_constant_in_seconds > 0 && time_constant_in_seconds < 30 ) {
        _time_constant_xy = time_constant_in_seconds;
        update_gains();
    }
}

// position_ok - return true if position has been initialised and have received gps data within 3 seconds
bool AP_InertialNav::position_ok()
{
    return _xy_enabled && (millis() - _gps_last_update < 3000);
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
    float hist_position_base_x, hist_position_base_y;

    // discard samples where dt is too large
    if( dt > 1.0 || dt == 0 || !_xy_enabled) {
        return;
    }

    // calculate distance from base location
    //x = (float)(lat - _base_lat) * 1.113195;
    //y = (float)(lon - _base_lon) * _lon_to_m_scaling * 1.113195;
    x = (float)(lat - _base_lat);
    y = (float)(lon - _base_lon) * _lon_to_m_scaling;

    // convert accelerometer readings to earth frame
    Matrix3f dcm = _ahrs->get_dcm_matrix();

    // correct accelerometer offsets using gps

    // ublox gps positions are delayed by 400ms
    // we store historical position at 10hz so 4 iterations ago
    if( _hist_position_estimate_x.num_items() >= AP_INTERTIALNAV_GPS_LAG_IN_10HZ_INCREMENTS ) {
        hist_position_base_x = _hist_position_estimate_x.peek(AP_INTERTIALNAV_GPS_LAG_IN_10HZ_INCREMENTS-1);
        hist_position_base_y = _hist_position_estimate_y.peek(AP_INTERTIALNAV_GPS_LAG_IN_10HZ_INCREMENTS-1);
    }else{
        hist_position_base_x = _position_base.x;
        hist_position_base_y = _position_base.y;
    }

    // calculate error in position from gps with our historical estimate
    // To-Do: check why x and y are reversed
    float err_x = -x - (hist_position_base_x + _position_correction.x);
    float err_y = -y - (hist_position_base_y + _position_correction.y);

    // calculate correction to accelerometers and apply in the body frame
    Vector3f accel_corr = accel_correction.get();
    accel_corr += dcm.mul_transpose(Vector3f((err_x*_k3_xy)*dt,(err_y*_k3_xy)*dt,0));
    accel_correction.set(accel_corr);

    // correct velocity
    _velocity.x += (err_x*_k2_xy) * dt;
    _velocity.y += (err_y*_k2_xy) * dt;

    // correct position
    _position_correction.x += err_x*_k1_xy * dt;
    _position_correction.y += err_y*_k1_xy * dt;
}

// get accel based latitude
int32_t AP_InertialNav::get_latitude()
{
    // make sure we've been initialised
    if( !_xy_enabled ) {
        return 0;
    }

    //return _base_lat - (int32_t)(_position.x / 1.113195);
    return _base_lat - (int32_t)(_position_base.x + _position_correction.x);
}

// get accel based longitude
int32_t AP_InertialNav::get_longitude()
{
    // make sure we've been initialised
    if( !_xy_enabled ) {
        return 0;
    }

    //return _base_lon - (int32_t)(_position.y / (_lon_to_m_scaling * 1.113195) );
    return _base_lon - (int32_t)((_position_base.y+_position_correction.y) / _lon_to_m_scaling );
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

    // reset corrections to base position to zero
    _position_base.x = 0;
    _position_base.y = 0;
    _position_correction.x = 0;
    _position_correction.y = 0;

    // clear historic estimates
    _hist_position_estimate_x.clear();
    _hist_position_estimate_y.clear();

    // set xy as enabled
    _xy_enabled = true;
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
    return -(_position_base.x+_position_correction.x);
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
    return -(_position_base.y+_position_correction.y) / _lon_to_m_scaling;
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

// set_velocity_xy - set velocity in latitude & longitude directions (in cm/s)
void AP_InertialNav::set_velocity_xy(float x, float y)
{
    _velocity.x = x;
    _velocity.y = y;
}

//
// Z Axis methods
//

// set time constant - set timeconstant used by complementary filter
void AP_InertialNav::set_time_constant_z( float time_constant_in_seconds )
{
    // ensure it's a reasonable value
    if( time_constant_in_seconds > 0 && time_constant_in_seconds < 30 ) {
        _time_constant_z = time_constant_in_seconds;
        update_gains();
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
    float hist_position_base_z;

    // discard samples where dt is too large
    if( dt > 0.2 ) {
        return;
    }

    // discard first 10 reads but perform some initialisation
    if( first_reads <= 10 ) {
        set_altitude(baro_alt);
        first_reads++;
    }

    // get dcm matrix
    Matrix3f dcm = _ahrs->get_dcm_matrix();

    // 3rd order samples (i.e. position from baro) are delayed by 150ms (15 iterations at 100hz)
    // so we should calculate error using historical estimates
    if( _hist_position_estimate_z.num_items() >= 15 ) {
        hist_position_base_z = _hist_position_estimate_z.peek(14);
    }else{
        hist_position_base_z = _position_base.z;
    }

    // calculate error in position from baro with our estimate
    float err = baro_alt - (hist_position_base_z + _position_correction.z);

    // calculate correction to accelerometers and apply in the body frame
    Vector3f accel_corr = accel_correction.get();
    accel_corr += dcm.mul_transpose(Vector3f(0,0,(err*_k3_z) * dt));
    accel_correction.set(accel_corr);

    // correct velocity
    _velocity.z += (err*_k2_z) * dt;

    // correct position
    _position_correction.z += err*_k1_z * dt;
}

// set_altitude - set base altitude estimate in cm
void AP_InertialNav::set_altitude( float new_altitude)
{
    _position_base.z = new_altitude;
    _position_correction.z = 0;
}

//
// Private methods
//

// update_gains - update gains from time constant (given in seconds)
void AP_InertialNav::update_gains()
{
    // X & Y axis time constant
    if( _time_constant_xy == 0 ) {
        _k1_xy = _k2_xy = _k3_xy = 0;
    }else{
        _k1_xy = 3 / _time_constant_xy;
        _k2_xy = 3 / (_time_constant_xy*_time_constant_xy);
        _k3_xy = 1 / (_time_constant_xy*_time_constant_xy*_time_constant_xy);
    }

    // Z axis time constant
    if( _time_constant_z == 0 ) {
        _k1_z = _k2_z = _k3_z = 0;
    }else{
        _k1_z = 3 / _time_constant_z;
        _k2_z = 3 / (_time_constant_z*_time_constant_z);
        _k3_z = 1 / (_time_constant_z*_time_constant_z*_time_constant_z);
    }
}

// set_velocity_z - get latest climb rate (in cm/s)
void AP_InertialNav::set_velocity_z(float z )
{
    _velocity.z = z;
}
