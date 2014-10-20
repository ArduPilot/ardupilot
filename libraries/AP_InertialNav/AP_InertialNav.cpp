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

    // check if home has moved and update
    check_home();

    // check if new gps readings have arrived and use them to correct position estimates
    check_gps();

    Vector3f accel_ef = _ahrs.get_accel_ef();

    // remove influence of gravity
    accel_ef.z += GRAVITY_MSS;
    accel_ef *= 100.0f;

    // remove xy if not enabled
    if( !_xy_enabled ) {
        accel_ef.x = 0.0f;
        accel_ef.y = 0.0f;
    }

    //Convert North-East-Down to North-East-Up
    accel_ef.z = -accel_ef.z;

    // convert ef position error to horizontal body frame
    Vector2f position_error_hbf;
    position_error_hbf.x = _position_error.x * _ahrs.cos_yaw() + _position_error.y * _ahrs.sin_yaw();
    position_error_hbf.y = -_position_error.x * _ahrs.sin_yaw() + _position_error.y * _ahrs.cos_yaw();

    float tmp = _k3_xy * dt;
    accel_correction_hbf.x += position_error_hbf.x * tmp;
    accel_correction_hbf.y += position_error_hbf.y * tmp;
    accel_correction_hbf.z += _position_error.z * _k3_z  * dt;

    tmp = _k2_xy * dt;
    _velocity.x += _position_error.x * tmp;
    _velocity.y += _position_error.y * tmp;
    _velocity.z += _position_error.z * _k2_z  * dt;

    tmp = _k1_xy * dt;
    _position_correction.x += _position_error.x * tmp;
    _position_correction.y += _position_error.y * tmp;
    _position_correction.z += _position_error.z * _k1_z  * dt;

    // convert horizontal body frame accel correction to earth frame
    Vector2f accel_correction_ef;
    accel_correction_ef.x = accel_correction_hbf.x * _ahrs.cos_yaw() - accel_correction_hbf.y * _ahrs.sin_yaw();
    accel_correction_ef.y = accel_correction_hbf.x * _ahrs.sin_yaw() + accel_correction_hbf.y * _ahrs.cos_yaw();

    // calculate velocity increase adding new acceleration from accelerometers
    Vector3f velocity_increase;
    velocity_increase.x = (accel_ef.x + accel_correction_ef.x) * dt;
    velocity_increase.y = (accel_ef.y + accel_correction_ef.y) * dt;
    velocity_increase.z = (accel_ef.z + accel_correction_hbf.z) * dt;

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

// position_ok - return true if position has been initialised and have received gps data within 3 seconds
bool AP_InertialNav::position_ok() const
{
    return _xy_enabled;
}

// check_home - checks if the home position has moved and offsets everything so it still lines up
void AP_InertialNav::check_home() {
    if (!_xy_enabled) {
        return;
    }

    // get position move in lat, lon coordinates
    int32_t lat_offset = _ahrs.get_home().lat - _last_home_lat;
    int32_t lng_offset = _ahrs.get_home().lng - _last_home_lng;

    if (lat_offset != 0) {
        // calculate the position move in cm
        float x_offset_cm = lat_offset * LATLON_TO_CM;

        // move position
        _position_base.x -= x_offset_cm;
        _position.x -= x_offset_cm;

        // update historic positions
        for (uint8_t i = 0; i < _hist_position_estimate_x.size(); i++) {
            float &x = _hist_position_estimate_x.peek_mutable(i);
            x -= x_offset_cm;
        }

        // update lon scaling
        _lon_to_cm_scaling = longitude_scale(_ahrs.get_home()) * LATLON_TO_CM;
    }

    if (lng_offset != 0) {
        // calculate the position move in cm
        float y_offset_cm = lng_offset * _lon_to_cm_scaling;

        // move position
        _position_base.y -= y_offset_cm;
        _position.y -= y_offset_cm;

        // update historic positions
        for (uint8_t i = 0; i < _hist_position_estimate_y.size(); i++) {
            float &y = _hist_position_estimate_y.peek_mutable(i);
            y -= y_offset_cm;
        }
    }

    // store updated lat, lon position
    _last_home_lat = _ahrs.get_home().lat;
    _last_home_lng = _ahrs.get_home().lng;
}

// check_gps - check if new gps readings have arrived and use them to correct position estimates
void AP_InertialNav::check_gps()
{
    const uint32_t now = hal.scheduler->millis();

    // compare gps time to previous reading
    const AP_GPS &gps = _ahrs.get_gps();
    if(gps.last_fix_time_ms() != _gps_last_time ) {

        // call position correction method
        correct_with_gps(now, gps.location().lng, gps.location().lat);

        // record gps time and system time of this update
        _gps_last_time = gps.last_fix_time_ms();
    }else{
        // if GPS updates stop arriving degrade position error to 10% over 2 seconds (assumes 100hz update rate)
        if (now - _gps_last_update > AP_INTERTIALNAV_GPS_TIMEOUT_MS) {
            _position_error.x *= 0.9886f;
            _position_error.y *= 0.9886f;
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
    if( dt > 1.0f || dt == 0.0f || !_xy_enabled) {
        return;
    }

    // calculate distance from base location
    x = (float)(lat - _ahrs.get_home().lat) * LATLON_TO_CM;
    y = (float)(lon - _ahrs.get_home().lng) * _lon_to_cm_scaling;

    // sanity check the gps position.  Relies on the main code calling GPS_Glitch::check_position() immediatley after a GPS update
    if (_glitch_detector.glitching()) {
        // failed sanity check so degrate position_error to 10% over 2 seconds (assumes 5hz update rate)
        _position_error.x *= 0.7943f;
        _position_error.y *= 0.7943f;
    }else{
        // if our internal glitching flag (from previous iteration) is true we have just recovered from a glitch
        // reset the inertial nav position and velocity to gps values
        if (_flags.gps_glitching) {
            set_position_xy(x,y);
            _position_error.x = 0.0f;
            _position_error.y = 0.0f;
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

    return _ahrs.get_home().lat + (int32_t)(_position.x/LATLON_TO_CM);
}

// get accel based longitude
int32_t AP_InertialNav::get_longitude() const
{
    // make sure we've been initialised
    if( !_xy_enabled ) {
        return 0;
    }

    return _ahrs.get_home().lng + (int32_t)(_position.y / _lon_to_cm_scaling);
}

// setup_home_position - reset state for home position change
void AP_InertialNav::setup_home_position(void)
{
    // set longitude to meters scaling to offset the shrinking longitude as we go towards the poles
    _lon_to_cm_scaling = longitude_scale(_ahrs.get_home()) * LATLON_TO_CM;

    // reset corrections to base position to zero
    _position_base.x = 0.0f;
    _position_base.y = 0.0f;
    _position_correction.x = 0.0f;
    _position_correction.y = 0.0f;
    _position.x = 0.0f;
    _position.y = 0.0f;
    _last_home_lat = _ahrs.get_home().lat;
    _last_home_lng = _ahrs.get_home().lng;

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
        return 0.0f;
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
float AP_InertialNav::get_velocity_xy() const
{
	return pythagorous2(_velocity.x, _velocity.y);
}

//
// Z Axis methods
//

// check_baro - check if new baro readings have arrived and use them to correct vertical accelerometer offsets
void AP_InertialNav::check_baro()
{
    uint32_t baro_update_time;

    // calculate time since last baro reading (in ms)
    baro_update_time = _baro.get_last_update();
    if( baro_update_time != _baro_last_update ) {
        const float dt = (float)(baro_update_time - _baro_last_update) * 0.001f; // in seconds
        // call correction method
        correct_with_baro(_baro.get_altitude()*100.0f, dt);
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

    // sanity check the baro position.  Relies on the main code calling Baro_Glitch::check_alt() immediatley after a baro update
    if (_baro_glitch.glitching()) {
        // failed sanity check so degrate position_error to 10% over 2 seconds (assumes 10hz update rate)
        _position_error.z *= 0.89715f;
    }else{
        // if our internal baro glitching flag (from previous iteration) is true we have just recovered from a glitch
        // reset the inertial nav alt to baro alt
        if (_flags.baro_glitching) {
            set_altitude(baro_alt);
            _position_error.z = 0.0f;
        }else{
            // 3rd order samples (i.e. position from baro) are delayed by 150ms (15 iterations at 100hz)
            // so we should calculate error using historical estimates
            float hist_position_base_z;
            if (_hist_position_estimate_z.is_full()) {
                hist_position_base_z = _hist_position_estimate_z.front();
            } else {
                hist_position_base_z = _position_base.z;
            }

            // calculate error in position from baro with our estimate
            _position_error.z = baro_alt - (hist_position_base_z + _position_correction.z);
        }
    }

    // update our internal record of glitching flag so that we can notice a change
    _flags.baro_glitching = _baro_glitch.glitching();
}

// set_altitude - set base altitude estimate in cm
void AP_InertialNav::set_altitude( float new_altitude)
{
    _position_base.z = new_altitude;
    _position_correction.z = 0;
    _position.z = new_altitude; // _position = _position_base + _position_correction
    _hist_position_estimate_z.clear(); // reset z history to avoid fake z velocity at next baro calibration (next rearm)
}

//
// Private methods
//

// update_gains - update gains from time constant (given in seconds)
void AP_InertialNav::update_gains()
{
    // X & Y axis time constant
    if (_time_constant_xy == 0.0f) {
        _k1_xy = _k2_xy = _k3_xy = 0.0f;
    }else{
        _k1_xy = 3.0f / _time_constant_xy;
        _k2_xy = 3.0f / (_time_constant_xy*_time_constant_xy);
        _k3_xy = 1.0f / (_time_constant_xy*_time_constant_xy*_time_constant_xy);
    }

    // Z axis time constant
    if (_time_constant_z == 0.0f) {
        _k1_z = _k2_z = _k3_z = 0.0f;
    }else{
        _k1_z = 3.0f / _time_constant_z;
        _k2_z = 3.0f / (_time_constant_z*_time_constant_z);
        _k3_z = 1.0f / (_time_constant_z*_time_constant_z*_time_constant_z);
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
    _position_correction.x = 0.0f;
    _position_correction.y = 0.0f;

    // clear historic estimates
    _hist_position_estimate_x.clear();
    _hist_position_estimate_y.clear();

    // add new position for future use
    _historic_xy_counter = 0;
    _hist_position_estimate_x.push_back(_position_base.x);
    _hist_position_estimate_y.push_back(_position_base.y);
}
