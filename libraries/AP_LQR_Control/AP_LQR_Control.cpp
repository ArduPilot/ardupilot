// Bank angle command based on non linear state dependent adaptive optimal control.
// Akshath Singhal, 2018

#include <AP_HAL/AP_HAL.h>
#include "AP_LQR_Control.h"

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_LQR_Control::var_info[] = {
    
    // @Param: LIM_BANK
    // @DisplayName: Loiter Radius Bank Angle Limit
    // @Description: The sealevel bank angle limit for a continous loiter. (Used to calculate airframe loading limits at higher altitudes). Setting to 0, will instead just scale the loiter radius directly
    // @Units: deg
    // @Range: 0 89
    // @User: Advanced
    AP_GROUPINFO_FRAME("LIM_BANK",   1, AP_LQR_Control, _loiter_bank_limit, 0.0f, AP_PARAM_FRAME_PLANE),

    // @Param: K
    // @DisplayName: Exponential Gain
    // @Description: Exponential Gain for computing Q1. A very high value can lead to high frequency of oscillations. A very small value leads to slow convergence to expected path.
    // @Range: 0.0 0.1
    // @Increment: 0.005
    // @User: Advanced
    AP_GROUPINFO("K",   2, AP_LQR_Control, _k_val, 0.01),

    // @Param: Q2_VAL
    // @DisplayName: VALUE of Q2 squared
    // @Description: The value of Q2 squared to be used. Penalty on rate of change of crosstrack error.
    // @Range: 0 100
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO("Q2_VAL",   3, AP_LQR_Control, _q2_val, 1),

    AP_GROUPEND
};

/*
  Wrap AHRS yaw if in reverse - radians
 */
float AP_LQR_Control::get_yaw_rad()
{   
    if (_reverse) {
        return wrap_PI(M_PI + _ahrs.yaw);
    }
    return _ahrs.yaw;
}

/*
  Wrap AHRS yaw sensor if in reverse - centi-degress
 */
int32_t AP_LQR_Control::get_yaw_sensor() const
{
    if (_reverse) {
        return wrap_180_cd(18000 + _ahrs.yaw_sensor);
    }
    return _ahrs.yaw_sensor;
}

float AP_LQR_Control::get_gs_angle_cd()
{
    Vector2f _groundspeed_vector = _ahrs.groundspeed_vector();
    if (_reverse) {
        return wrap_180_cd(18000 + RadiansToCentiDegrees(atan2f(_groundspeed_vector.y, _groundspeed_vector.x)));
    }
    return RadiansToCentiDegrees(atan2f(_groundspeed_vector.y, _groundspeed_vector.x));
}

/*
  return the bank angle needed to achieve tracking from the last
  update_*() operation
 */
int32_t AP_LQR_Control::nav_roll_cd(void) const
{
    float ret;
    ret = cosf(_ahrs.pitch)*degrees(atanf(_latAccDem * 0.101972f) * 100.0f); // 0.101972 = 1/9.81
    ret = constrain_float(ret, -9000, 9000);
    AP::logger().Write("NAVC", "TimeUS,Xtrk,LAcc", "Qff",
                        AP_HAL::micros64(),
                        (double)_crosstrack_error,
                        (double)_latAccDem);
    return ret;
}

/*
  return the lateral acceleration needed to achieve tracking from the last
  update_*() operation
 */
float AP_LQR_Control::lateral_acceleration(void) const
{
    return _latAccDem;
}

int32_t AP_LQR_Control::nav_bearing_cd(void) const
{
    return wrap_180_cd(RadiansToCentiDegrees(_nav_bearing));
}

int32_t AP_LQR_Control::bearing_error_cd(void) const
{
    return RadiansToCentiDegrees(_bearing_error);
}

int32_t AP_LQR_Control::target_bearing_cd(void) const
{
    return wrap_180_cd(_target_bearing_cd);
}

/*
  this is the turn distance assuming a 90 degree turn
 */
float AP_LQR_Control::turn_distance(float wp_radius) const
{
    wp_radius *= sq(_ahrs.get_EAS2TAS());
    return wp_radius;
}

/*
  this approximates the turn distance for a given turn angle. If the
  turn_angle is > 90 then a 90 degree turn distance is used, otherwise
  the turn distance is reduced linearly.
  This function allows straight ahead mission legs to avoid thinking
  they have reached the waypoint early, which makes things like camera
  trigger and ball drop at exact positions under mission control much easier
 */
float AP_LQR_Control::turn_distance(float wp_radius, float turn_angle) const
{
    float distance_90 = turn_distance(wp_radius);
    turn_angle = fabsf(turn_angle);
    if (turn_angle >= 90) {
        return distance_90;
    }
    return distance_90 * turn_angle / 90.0f;
}

float AP_LQR_Control::loiter_radius(const float radius) const
{
    // prevent an insane loiter bank limit
    float sanitized_bank_limit = constrain_float(_loiter_bank_limit, 0.0f, 89.0f);
    float lateral_accel_sea_level = tanf(radians(sanitized_bank_limit)) * GRAVITY_MSS;

    float nominal_velocity_sea_level;
    if(_spdHgtControl == nullptr) {
        nominal_velocity_sea_level = 0.0f;
    } else {
        nominal_velocity_sea_level =  _spdHgtControl->get_target_airspeed();
    }

    float eas2tas_sq = sq(_ahrs.get_EAS2TAS());

    if (is_zero(sanitized_bank_limit) || is_zero(nominal_velocity_sea_level) ||
        is_zero(lateral_accel_sea_level)) {
        // Missing a sane input for calculating the limit, or the user has
        // requested a straight scaling with altitude. This will always vary
        // with the current altitude, but will at least protect the airframe
        return radius * eas2tas_sq;
    } else {
        float sea_level_radius = sq(nominal_velocity_sea_level) / lateral_accel_sea_level;
        if (sea_level_radius > radius) {
            // If we've told the plane that its sea level radius is unachievable fallback to
            // straight altitude scaling
            return radius * eas2tas_sq;
        } else {
            // select the requested radius, or the required altitude scale, whichever is safer
            return MAX(sea_level_radius * eas2tas_sq, radius);
        }
    }
}

bool AP_LQR_Control::reached_loiter_target(void)
{
    return _WPcircle;
}

/**
   prevent indecision in our turning by using our previous turn
   decision if we are in a narrow angle band pointing away from the
   target and the turn angle has changed sign
 */

// update LQR control for waypoint navigation
void AP_LQR_Control::update_waypoint(const struct Location &prev_WP, const struct Location &next_WP, float dist_min)
{
    struct Location _current_loc;
    
    uint32_t now = AP_HAL::micros();
    float dt = (now - _last_update_waypoint_us) * 1.0e-6f;
    if (dt > 0.1) {
        dt = 0.1;
    }
    _last_update_waypoint_us = now;

    // Get current position and velocity
    if (_ahrs.get_position(_current_loc) == false) {
        // if no GPS loc available, maintain last nav/target_bearing
        _data_is_stale = true;
        return;
    }

    Vector2f _groundspeed_vector = _ahrs.groundspeed_vector();

    // update _target_bearing_cd
    _target_bearing_cd = _current_loc.get_bearing_to(next_WP);

    //Calculate groundspeed
    float groundSpeed = _groundspeed_vector.length();
    if (groundSpeed < 0.1f) {
        // use a small ground speed vector in the right direction,
        // allowing us to use the compass heading at zero GPS velocity
        groundSpeed = 0.1f;
        _groundspeed_vector = Vector2f(cosf(get_yaw_rad()), sinf(get_yaw_rad())) * groundSpeed;
    }

    // Calculate the NE position of WP B relative to WP A
    Vector2f AB = prev_WP.get_distance_NE(next_WP);

    // Check for AB zero length and track directly to the destination
    // if too small
    if (AB.length() < 1.0e-6f) {
        AB = _current_loc.get_distance_NE(next_WP);
        if (AB.length() < 1.0e-6f) {
            AB = Vector2f(cosf(get_yaw_rad()), sinf(get_yaw_rad()));
        }
    }
    AB.normalize();

    // Calculate the NE position of the aircraft relative to WP A
    Vector2f A_air = prev_WP.get_distance_NE(_current_loc);

    // calculate distance to target track, for reporting
    _crosstrack_error = (A_air % AB)/(AB.length());

    _crosstrack_error=- _crosstrack_error;
    
    //Calculate adaptive gains
    float q1 = sqrtf(float(exp(_k_val*abs(_crosstrack_error))));
    
    //Calculate the approach velocity towards path
    float si = get_gs_angle_cd()*0.01;
    float si_p = (prev_WP.get_bearing_to(next_WP))*0.01;
    float temp_sin=sinf(radians(si - si_p));
    float v_d= groundSpeed * temp_sin;

    float cos_term = cosf(radians(si-si_p));
    if(fabs(cos_term) < 0.01)
    {
        if (cos_term < 0)
            cos_term = float(-0.01);
        else
            cos_term = float(0.01);
    }

    float p12 = float(q1/fabs(cos_term));
    float p22 = sqrtf((float)(2*p12 + _q2_val))/(cos_term);

    //Compute lateral acceleration based on current state and adaptive gains
    float u = -(_crosstrack_error*p12*cosf(radians(si - si_p)) + p22*v_d*cosf(radians(si - si_p)));
    
    _latAccDem= u;
    _bearing_error = radians(si - si_p);
    _nav_bearing = radians(si_p);
    
    // Waypoint capture status is always false during waypoint following
    _WPcircle = false;
    _data_is_stale = false; // status are correctly updated with current waypoint data
   
}

// update LQR control for loitering
void AP_LQR_Control::update_loiter(const struct Location &center_WP, float radius, int8_t loiter_direction)
{
    struct Location _current_loc;
    // Get current position and velocity
    if (_ahrs.get_position(_current_loc) == false) {
        // if no GPS loc available, maintain last nav/target_bearing
        _data_is_stale = true;
        return;
    }
    Vector2f _groundspeed_vector = _ahrs.groundspeed_vector();

    // scale loiter radius with square of EAS2TAS to allow us to stay
    // stable at high altitude
    radius = loiter_radius(radius);
    float groundSpeed=_groundspeed_vector.length();
    Vector2f location_difference=center_WP.get_distance_NE(_current_loc);
    
    //Calculate rate change of heading of path
    float si_p_dot=groundSpeed/location_difference.length();
    _crosstrack_error = location_difference.length() - radius;
    
    //Set minimum turn radius equal to twice the groundspeed
    float min_turn_rad = 2*groundSpeed;
    
    float u =0;
    
    // update _target_bearing_cd
    _target_bearing_cd = center_WP.get_bearing_to(_current_loc);
    //Compute adaptive gains
    float q1 = sqrtf(float(exp(_k_val*_crosstrack_error)));
    
    //Compute velocity of approach towards desired path
    float si = get_gs_angle_cd()*0.01;
    // check if vehicle is not very far from the desired circular path
    if (fabsf(_crosstrack_error) < (min_turn_rad))
    {
        //Compute desired heading perpendicular
        float si_p = ((_target_bearing_cd + (loiter_direction)*(9000))*0.01);
        float temp_sin=sinf(radians(si - si_p));
        float v_d= groundSpeed * temp_sin;
        //Compute desired lateral acceleration
        u = ((q1*_crosstrack_error)-(sqrtf((_q2_val+(2*q1)))*v_d)+groundSpeed*si_p_dot);
        _WPcircle = true;
        _bearing_error = radians(si - si_p);
        _nav_bearing = radians(si_p);
    }
    
    //lead towards center if vehicle is very far from circular path until crosstrack error < minimum turning radius
    else
    {
        float si_p = (_target_bearing_cd*0.01);
        if (_crosstrack_error > 0)
        {
            si_p = si_p+180;
        }
        float temp_sin=sinf(radians(si - si_p));
        float v_d= groundSpeed * temp_sin;
        u = - (sqrtf((float)(_q2_val+(2*q1)))*v_d);
        _WPcircle = false;
        _bearing_error = radians(si - si_p);
        _nav_bearing = radians(si_p);
    }

    _latAccDem = u;
    
    _data_is_stale = false; // status are correctly updated with current waypoint data

  
}


// update LQR control for heading hold navigation
void AP_LQR_Control::update_heading_hold(int32_t navigation_heading_cd)
{

    // copy to _target_bearing_cd and _nav_bearing
    _target_bearing_cd = wrap_180_cd(navigation_heading_cd);
    _nav_bearing = radians(navigation_heading_cd * 0.01f);

    Vector2f _groundspeed_vector = _ahrs.groundspeed_vector();

    //Calculate groundspeed
    float groundSpeed = _groundspeed_vector.length();
    if (groundSpeed < 0.1f) {
        // use a small ground speed vector in the right direction,
        // allowing us to use the compass heading at zero GPS velocity
        groundSpeed = 0.1f;
        _groundspeed_vector = Vector2f(cosf(get_yaw_rad()), sinf(get_yaw_rad())) * groundSpeed;
    }

    float si = get_gs_angle_cd()*0.01;
    float si_p = (_target_bearing_cd)*0.01;
    float temp_sin = sinf(radians(si - si_p));
    float v_d = groundSpeed * temp_sin;
    float u =  - (sqrtf(_q2_val)*v_d);
    _bearing_error = radians(si - si_p);
    _latAccDem = u;
    _WPcircle = false;

    _data_is_stale = false; // status are correctly updated with current waypoint data
}

// update LQR control for level flight on current heading
void AP_LQR_Control::update_level_flight(void)
{
    // copy to _target_bearing_cd and _nav_bearing
    _target_bearing_cd = _ahrs.yaw_sensor;
    _nav_bearing = _ahrs.yaw;
    _bearing_error = 0;
    _crosstrack_error = 0;

    // Waypoint capture status is always false during heading hold
    _WPcircle = false;

    _latAccDem = 0;

    _data_is_stale = false; // status are correctly updated with current waypoint data
}

