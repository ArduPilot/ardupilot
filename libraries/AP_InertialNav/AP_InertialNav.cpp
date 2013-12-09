/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>
#include <AP_InertialNav.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_InertialNav::var_info[] PROGMEM = {
    // start numbering at 1 because 0 was previous used for body frame accel offsets
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

// update - updates velocities and positions using latest info from ahrs and barometer if new data is available;
void AP_InertialNav::update(float dt)
{
    // discard samples where dt is too large
    if( dt > 0.1f ) {
        return;
    }

    // decrement ignore error count if required
    if (_flags.ignore_error > 0) {
        _flags.ignore_error--;
    }

    // check if new baro readings have arrived and use them to correct vertical accelerometer offsets.
    check_baro();

    // check if new gps readings have arrived and use them to correct position estimates
    check_gps();

    Vector3f accel_ef = _ahrs->get_accel_ef();

    // remove influence of gravity
    accel_ef.z += GRAVITY_MSS;
    accel_ef *= 100;

    // remove xy if not enabled
    if( !_xy_enabled ) {
        accel_ef.x = 0;
        accel_ef.y = 0;
    }

    //Convert North-East-Down to North-East-Up
    accel_ef.z = -accel_ef.z;

    float tmp = _k3_xy * dt;
    accel_correction_ef.x += _position_error.x * tmp;
    accel_correction_ef.y += _position_error.y * tmp;
    accel_correction_ef.z += _position_error.z * _k3_z  * dt;

    tmp = _k2_xy * dt;
    _velocity.x += _position_error.x * tmp;
    _velocity.y += _position_error.y * tmp;
    _velocity.z += _position_error.z * _k2_z  * dt;

    tmp = _k1_xy * dt;
    _position_correction.x += _position_error.x * tmp;
    _position_correction.y += _position_error.y * tmp;
    _position_correction.z += _position_error.z * _k1_z  * dt;

    // calculate velocity increase adding new acceleration from accelerometers
    const Vector3f &velocity_increase = (accel_ef + accel_correction_ef) * dt;

    // calculate new estimate of position
    _position_base += (_velocity + velocity_increase*0.5) * dt;

    // update the corrected position estimate
    _position = _position_base + _position_correction;

    // calculate new velocity
    _velocity += velocity_increase;

    // store 3rd order estimate (i.e. estimated vertical position) for future use
    _hist_position_estimate_z.push_back(_position_base.z);

    // store 3rd order estimate (i.e. horizontal position) for future use at 10hz
    _historic_xy_counter++;
    if( _historic_xy_counter >= AP_INTERTIALNAV_SAVE_POS_AFTER_ITERATIONS ) {
        _historic_xy_counter = 0;
        _hist_position_estimate_x.push_back(_position_base.x);
        _hist_position_estimate_y.push_back(_position_base.y);
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
bool AP_InertialNav::position_ok() const
{
    return _xy_enabled;
}

// check_gps - check if new gps readings have arrived and use them to correct position estimates
void AP_InertialNav::check_gps()
{
    const uint32_t now = hal.scheduler->millis();

    // compare gps time to previous reading
    if( _gps != NULL && _gps->last_fix_time != _gps_last_time ) {

        // call position correction method
        correct_with_gps(now, _gps->longitude, _gps->latitude);

        // record gps time and system time of this update
        _gps_last_time = _gps->last_fix_time;
    }else{
        // if GPS updates stop arriving degrade position error to 10% over 2 seconds (assumes 100hz update rate)
        if (now - _gps_last_update > AP_INTERTIALNAV_GPS_TIMEOUT_MS) {
            _position_error.x *= 0.9886;
            _position_error.y *= 0.9886;
            // increment error count
            if (_flags.ignore_error == 0 && _error_count < 255 && _xy_enabled) {
                _error_count++;
            }
        }
    }
}

// correct_with_gps - modifies accelerometer offsets using gps
void AP_InertialNav::correct_with_gps(uint32_t now, int32_t lon, int32_t lat)
{
    float dt,x,y;
    float hist_position_base_x, hist_position_base_y;

    // calculate time since last gps reading
    dt = (float)(now - _gps_last_update) * 0.001f;

    // update last gps update time
    _gps_last_update = now;

    // discard samples where dt is too large
    if( dt > 1.0f || dt == 0 || !_xy_enabled) {
        return;
    }

    // calculate distance from base location
    x = (float)(lat - _base_lat) * LATLON_TO_CM;
    y = (float)(lon - _base_lon) * _lon_to_cm_scaling;

    // sanity check the gps position.  Relies on the main code calling GPS_Glitch::check_position() immediatley after a GPS update
    if (_glitch_detector.glitching()) {
        // failed sanity check so degrate position_error to 10% over 2 seconds (assumes 5hz update rate)
        _position_error.x *= 0.7934;
        _position_error.y *= 0.7934;
    }else{
        // if our internal glitching flag (from previous iteration) is true we have just recovered from a glitch
        // reset the inertial nav position and velocity to gps values
        if (_flags.gps_glitching) {
            set_position_xy(x,y);
            set_velocity_xy(_gps->velocity_north() * 100.0f,_gps->velocity_east() * 100.0f);
            _position_error.x = 0;
            _position_error.y = 0;
        }else{
            // ublox gps positions are delayed by 400ms
            // we store historical position at 10hz so 4 iterations ago
            if( _hist_position_estimate_x.is_full()) {
                hist_position_base_x = _hist_position_estimate_x.front();
                hist_position_base_y = _hist_position_estimate_y.front();
            }else{
                hist_position_base_x = _position_base.x;
                hist_position_base_y = _position_base.y;
            }

            // calculate error in position from gps with our historical estimate
            _position_error.x = x - (hist_position_base_x + _position_correction.x);
            _position_error.y = y - (hist_position_base_y + _position_correction.y);
        }
    }

    // update our internal record of glitching flag so that we can notice a change
    _flags.gps_glitching = _glitch_detector.glitching();
}

// get accel based latitude
int32_t AP_InertialNav::get_latitude() const
{
    // make sure we've been initialised
    if( !_xy_enabled ) {
        return 0;
    }

    return _base_lat + (int32_t)(_position.x/LATLON_TO_CM);
}

// get accel based longitude
int32_t AP_InertialNav::get_longitude() const
{
    // make sure we've been initialised
    if( !_xy_enabled ) {
        return 0;
    }

    return _base_lon + (int32_t)(_position.y / _lon_to_cm_scaling);
}

// set_home_position - all internal calculations are recorded as the distances from this point
void AP_InertialNav::set_home_position(int32_t lon, int32_t lat)
{
    // set base location
    _base_lon = lon;
    _base_lat = lat;

    // set longitude to meters scaling to offset the shrinking longitude as we go towards the poles
    Location temp_loc;
    temp_loc.lat = lat;
    temp_loc.lng = lon;
    _lon_to_cm_scaling = longitude_scale(temp_loc) * LATLON_TO_CM;

    // reset corrections to base position to zero
    _position_base.x = 0;
    _position_base.y = 0;
    _position_correction.x = 0;
    _position_correction.y = 0;
    _position.x = 0;
    _position.y = 0;

    // clear historic estimates
    _hist_position_estimate_x.clear();
    _hist_position_estimate_y.clear();

    // set xy as enabled
    _xy_enabled = true;
}

// get accel based latitude
float AP_InertialNav::get_latitude_diff() const
{
    // make sure we've been initialised
    if( !_xy_enabled ) {
        return 0;
    }

    return (_position.x/LATLON_TO_CM);
}

// get accel based longitude
float AP_InertialNav::get_longitude_diff() const
{
    // make sure we've been initialised
    if( !_xy_enabled ) {
        return 0;
    }

    return (_position.y / _lon_to_cm_scaling);
}

// set_velocity_xy - set velocity in latitude & longitude directions (in cm/s)
void AP_InertialNav::set_velocity_xy(float x, float y)
{
    _velocity.x = x;
    _velocity.y = y;
}

// set_velocity_xy - set velocity in latitude & longitude directions (in cm/s)
float AP_InertialNav::get_velocity_xy()
{
	return safe_sqrt(_velocity.x * _velocity.x + _velocity.y * _velocity.y);
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

    // calculate time since last baro reading (in ms)
    baro_update_time = _baro->get_last_update();
    if( baro_update_time != _baro_last_update ) {
        const float dt = (float)(baro_update_time - _baro_last_update) * 0.001f; // in seconds
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
    if( dt > 0.5f ) {
        return;
    }

    // discard first 10 reads but perform some initialisation
    if( first_reads <= 10 ) {
        set_altitude(baro_alt);
        first_reads++;
    }

    // 3rd order samples (i.e. position from baro) are delayed by 150ms (15 iterations at 100hz)
    // so we should calculate error using historical estimates
    float hist_position_base_z;
    if( _hist_position_estimate_z.is_full() ) {
        hist_position_base_z = _hist_position_estimate_z.front();
    }else{
        hist_position_base_z = _position_base.z;
    }

    // calculate error in position from baro with our estimate
    _position_error.z = baro_alt - (hist_position_base_z + _position_correction.z);
}

// set_altitude - set base altitude estimate in cm
void AP_InertialNav::set_altitude( float new_altitude)
{
    _position_base.z = new_altitude;
    _position_correction.z = 0;
    _position.z = new_altitude; // _position = _position_base + _position_correction
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

// set_position_xy - sets inertial navigation position to given xy coordinates from home
void AP_InertialNav::set_position_xy(float x, float y)
{
    // reset position from home
    _position_base.x = x;
    _position_base.y = y;
    _position_correction.x = 0;
    _position_correction.y = 0;

    // clear historic estimates
    _hist_position_estimate_x.clear();
    _hist_position_estimate_y.clear();

    // add new position for future use
    _historic_xy_counter = 0;
    _hist_position_estimate_x.push_back(_position_base.x);
    _hist_position_estimate_y.push_back(_position_base.y);
}
