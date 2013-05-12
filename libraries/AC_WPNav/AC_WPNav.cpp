/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>
#include <AC_WPNav.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_WPNav::var_info[] PROGMEM = {
    // index 0 was used for the old orientation matrix

    // @Param: SPEED
    // @DisplayName: Waypoint Horizontal Speed Target
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain horizontally during a WP mission
    // @Units: Centimeters/Second
    // @Range: 0 2000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("SPEED",       0, AC_WPNav, _wp_speed_cms, WPNAV_WP_SPEED),

    // @Param: RADIUS
    // @DisplayName: Waypoint Radius
    // @Description: Defines the distance from a waypoint, that when crossed indicates the wp has been hit.
    // @Units: Centimeters
    // @Range: 100 1000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RADIUS",      1, AC_WPNav, _wp_radius_cm, WPNAV_WP_RADIUS),

    // @Param: SPEED_UP
    // @DisplayName: Waypoint Climb Speed Target
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain while climbing during a WP mission
    // @Units: Centimeters/Second
    // @Range: 0 1000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("SPEED_UP",    2, AC_WPNav, _wp_speed_up_cms, WPNAV_WP_SPEED_UP),

    // @Param: SPEED_DN
    // @DisplayName: Waypoint Descent Speed Target
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain while descending during a WP mission
    // @Units: Centimeters/Second
    // @Range: 0 1000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("SPEED_DN",    3, AC_WPNav, _wp_speed_down_cms, WPNAV_WP_SPEED_DOWN),

    // @Param: LOIT_SPEED
    // @DisplayName: Loiter Horizontal Maximum Speed
    // @Description: Defines the maximum speed in cm/s which the aircraft will travel horizontally while in loiter mode
    // @Units: Centimeters/Second
    // @Range: 0 2000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("LOIT_SPEED",  4, AC_WPNav, _loiter_speed_cms, WPNAV_LOITER_SPEED),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_WPNav::AC_WPNav(AP_InertialNav* inav, AP_AHRS* ahrs, APM_PI* pid_pos_lat, APM_PI* pid_pos_lon, AC_PID* pid_rate_lat, AC_PID* pid_rate_lon) :
    _inav(inav),
    _ahrs(ahrs),
    _pid_pos_lat(pid_pos_lat),
    _pid_pos_lon(pid_pos_lon),
    _pid_rate_lat(pid_rate_lat),
    _pid_rate_lon(pid_rate_lon),
    _loiter_last_update(0),
    _wpnav_last_update(0),
    _cos_yaw(1.0),
    _sin_yaw(0.0),
    _cos_pitch(1.0),
    _desired_roll(0),
    _desired_pitch(0),
    _target(0,0,0),
    _pilot_vel_forward_cms(0),
    _pilot_vel_right_cms(0),
    _target_vel(0,0,0),
    _vel_last(0,0,0),
    _lean_angle_max(MAX_LEAN_ANGLE),
    dist_error(0,0),
    desired_vel(0,0),
    desired_accel(0,0)
{
    AP_Param::setup_object_defaults(this, var_info);

    // calculate loiter leash
    calculate_loiter_leash_length();
}

///
/// simple loiter controller
///

/// get_stopping_point - returns vector to stopping point based on a horizontal position and velocity
void AC_WPNav::get_stopping_point(const Vector3f& position, const Vector3f& velocity, Vector3f &target) const
{
    float linear_distance;      // half the distace we swap between linear and sqrt and the distace we offset sqrt.
    float linear_velocity;      // the velocity we swap between linear and sqrt.
    float vel_total;
    float target_dist;

    // avoid divide by zero
    if( _pid_pos_lat->kP() <= 0.1 ) {
        target = position;
        return;
    }

    // calculate point at which velocity switches from linear to sqrt
    linear_velocity = MAX_LOITER_POS_ACCEL/_pid_pos_lat->kP();

    // calculate total current velocity
    vel_total = safe_sqrt(velocity.x*velocity.x + velocity.y*velocity.y);

    // calculate distance within which we can stop
    if (vel_total < linear_velocity) {
        target_dist = vel_total/_pid_pos_lat->kP();
    } else {
        linear_distance = MAX_LOITER_POS_ACCEL/(2*_pid_pos_lat->kP()*_pid_pos_lat->kP());
        target_dist = linear_distance + (vel_total*vel_total)/(2*MAX_LOITER_POS_ACCEL);
    }
    target_dist = constrain_float(target_dist, 0, _loiter_leash);

    target.x = position.x + (target_dist * velocity.x / vel_total);
    target.y = position.y + (target_dist * velocity.y / vel_total);
    target.z = position.z;
}

/// set_loiter_target - set initial loiter target based on current position and velocity
void AC_WPNav::set_loiter_target(const Vector3f& position, const Vector3f& velocity)
{
    Vector3f target;
    calculate_loiter_leash_length();
    get_stopping_point(position, velocity, target);
    _target.x = target.x;
    _target.y = target.y;

    // initialise desired roll and pitch to current roll and pitch.  This avoids a random twitch between now and when the loiter controller is first run
    _desired_roll = constrain_int32(_ahrs->roll_sensor,-MAX_LEAN_ANGLE,MAX_LEAN_ANGLE);
    _desired_pitch = constrain_int32(_ahrs->pitch_sensor,-MAX_LEAN_ANGLE,MAX_LEAN_ANGLE);
}

/// move_loiter_target - move loiter target by velocity provided in front/right directions in cm/s
void AC_WPNav::move_loiter_target(float control_roll, float control_pitch, float dt)
{
    // convert pilot input to desired velocity in cm/s
    _pilot_vel_forward_cms = -control_pitch * _loiter_speed_cms / 4500.0f;
    _pilot_vel_right_cms = control_roll * _loiter_speed_cms / 4500.0f;
}

/// translate_loiter_target_movements - consumes adjustments created by move_loiter_target
void AC_WPNav::translate_loiter_target_movements(float nav_dt)
{
    Vector2f target_vel_adj;
    float vel_delta_total;
    float vel_max;
    float vel_total;

    // range check nav_dt
    if( nav_dt < 0 ) {
        return;
    }

    // rotate pilot input to lat/lon frame
    target_vel_adj.x = (_pilot_vel_forward_cms*_cos_yaw - _pilot_vel_right_cms*_sin_yaw) - _target_vel.x;
    target_vel_adj.y = (_pilot_vel_forward_cms*_sin_yaw + _pilot_vel_right_cms*_cos_yaw) - _target_vel.y;

    // constrain the velocity vector and scale if necessary
    vel_delta_total = safe_sqrt(target_vel_adj.x*target_vel_adj.x + target_vel_adj.y*target_vel_adj.y);
    vel_max = 2.0*MAX_LOITER_POS_ACCEL*nav_dt;
    if( vel_delta_total >  vel_max) {
        target_vel_adj.x = vel_max * target_vel_adj.x/vel_delta_total;
        target_vel_adj.y = vel_max * target_vel_adj.y/vel_delta_total;
    }

    // add desired change in velocity to current target velocity
    _target_vel.x += target_vel_adj.x;
    _target_vel.y += target_vel_adj.y;

    // constrain the velocity vector and scale if necessary
    vel_total = safe_sqrt(_target_vel.x*_target_vel.x + _target_vel.y*_target_vel.y);
    if( vel_total > _loiter_speed_cms ) {
        _target_vel.x = _loiter_speed_cms * _target_vel.x/vel_total;
        _target_vel.y = _loiter_speed_cms * _target_vel.y/vel_total;
    }

    // update target position
    _target.x += _target_vel.x * nav_dt;
    _target.y += _target_vel.y * nav_dt;

    // constrain target position to within reasonable distance of current location
    Vector3f curr_pos = _inav->get_position();
    Vector3f distance_err = _target - curr_pos;
    float distance = safe_sqrt(distance_err.x*distance_err.x + distance_err.y*distance_err.y);
    if( distance > _loiter_leash ) {
        _target.x = curr_pos.x + _loiter_leash * distance_err.x/distance;
        _target.y = curr_pos.y + _loiter_leash * distance_err.y/distance;
    }
}

/// get_distance_to_target - get horizontal distance to loiter target in cm
float AC_WPNav::get_distance_to_target() const
{
    return _distance_to_target;
}

/// get_bearing_to_target - get bearing to loiter target in centi-degrees
int32_t AC_WPNav::get_bearing_to_target() const
{
    return get_bearing_cd(_inav->get_position(), _target);
}

/// update_loiter - run the loiter controller - should be called at 10hz
void AC_WPNav::update_loiter()
{
    uint32_t now = hal.scheduler->millis();
    float dt = (now - _loiter_last_update) / 1000.0f;
    _loiter_last_update = now;

    // catch if we've just been started
    if( dt >= 1.0 ) {
        dt = 0.0;
        reset_I();
    }

    // translate any adjustments from pilot to loiter target
    translate_loiter_target_movements(dt);

    // run loiter position controller
    get_loiter_position_to_velocity(dt, _loiter_speed_cms);
}

/// calculate_loiter_leash_length - calculates the maximum distance in cm that the target position may be from the current location
void AC_WPNav::calculate_loiter_leash_length()
{
    // get loiter position P
    float kP = _pid_pos_lat->kP();

    // calculate horiztonal leash length
    if(_loiter_speed_cms <= MAX_LOITER_POS_ACCEL / kP) {
        // linear leash length based on speed close in
        _loiter_leash = _loiter_speed_cms / kP;
    }else{
        // leash length grows at sqrt of speed further out
        _loiter_leash = (MAX_LOITER_POS_ACCEL / (2.0*kP*kP)) + (_loiter_speed_cms*_loiter_speed_cms / (2*MAX_LOITER_POS_ACCEL));
    }

    // ensure leash is at least 1m long
    if( _loiter_leash < 100 ) {
        _loiter_leash = 100;
    }
}

///
/// waypoint navigation
///

/// set_destination - set destination using cm from home
void AC_WPNav::set_destination(const Vector3f& destination)
{
    // if waypoint controlls is active and copter has reached the previous waypoint use it for the origin
    if( _flags.reached_destination && ((hal.scheduler->millis() - _wpnav_last_update) < 1000) ) {
        _origin = _destination;
    }else{
        // otherwise calculate origin from the current position and velocity
        get_stopping_point(_inav->get_position(), _inav->get_velocity(), _origin);
    }

    // set origin and destination
    set_origin_and_destination(_origin, destination);
}

/// set_origin_and_destination - set origin and destination using lat/lon coordinates
void AC_WPNav::set_origin_and_destination(const Vector3f& origin, const Vector3f& destination)
{
    // store origin and destination locations
    _origin = origin;
    _destination = destination;
    Vector3f pos_delta = _destination - _origin;

    // calculate leash lengths 
    bool climb = pos_delta.z >= 0;  // climbing vs descending leads to different leash lengths because speed_up_cms and speed_down_cms can be different
    calculate_wp_leash_length(climb);  // update leash lengths and _vert_track_scale

    // scale up z-axis position delta (i.e. distance) to make later leash length calculations simpler
    pos_delta.z = pos_delta.z * _vert_track_scale;
    _track_length = pos_delta.length();
    _pos_delta_unit = pos_delta/_track_length;

    // initialise intermediate point to the origin
    _track_desired = 0;
    _target = origin;
    _flags.reached_destination = false;

    // reset limited speed to zero to slow initial acceleration away from wp
    _limited_speed_xy_cms = 0;

    // default waypoint back to slow
    _flags.fast_waypoint = false;

    // initialise desired roll and pitch to current roll and pitch.  This avoids a random twitch between now and when the wpnav controller is first run
    _desired_roll = constrain_int32(_ahrs->roll_sensor,-MAX_LEAN_ANGLE,MAX_LEAN_ANGLE);
    _desired_pitch = constrain_int32(_ahrs->pitch_sensor,-MAX_LEAN_ANGLE,MAX_LEAN_ANGLE);
}

/// advance_target_along_track - move target location along track from origin to destination
void AC_WPNav::advance_target_along_track(float dt)
{
    float track_covered;
    float track_error;
    float track_desired_max;
    float track_desired_temp = _track_desired;
    float track_extra_max;
    float curr_delta_length;

    // get current location
    Vector3f curr_pos = _inav->get_position();
    Vector3f curr_delta = curr_pos - _origin;
    curr_delta.z = curr_delta.z * _vert_track_scale;
    curr_delta_length = curr_delta.length();

    // increase intermediate target point's velocity if not yet at target speed
    if(dt > 0 && _limited_speed_xy_cms < _wp_speed_cms) {
        _limited_speed_xy_cms += WPNAV_WP_ACCELERATION * dt;
    }
    if(_limited_speed_xy_cms > _wp_speed_cms) {
        _limited_speed_xy_cms = _wp_speed_cms;
    }

    // calculate how far along the track we are
    track_covered = curr_delta.x * _pos_delta_unit.x + curr_delta.y * _pos_delta_unit.y + curr_delta.z * _pos_delta_unit.z;
    track_error = safe_sqrt(curr_delta_length*curr_delta_length - track_covered*track_covered);

    // calculate how far along the track we could move the intermediate target before reaching the end of the leash
    track_extra_max = safe_sqrt(_wp_leash_xy*_wp_leash_xy - track_error*track_error);
    track_desired_max = track_covered + track_extra_max;

    // advance the current target
    track_desired_temp += _limited_speed_xy_cms * dt;

    // constrain the target from moving too far
    if( track_desired_temp > track_desired_max ) {
        track_desired_temp = track_desired_max;
    }
    // do not let desired point go past the end of the segment
    track_desired_temp = constrain_float(track_desired_temp, 0, _track_length);
    _track_desired = max(_track_desired, track_desired_temp);

    // recalculate the desired position
    _target.x = _origin.x + _pos_delta_unit.x * _track_desired;
    _target.y = _origin.y + _pos_delta_unit.y * _track_desired;
    _target.z = _origin.z + (_pos_delta_unit.z * _track_desired)/_vert_track_scale;

    // check if we've reached the waypoint
    if( !_flags.reached_destination ) {
        if( _track_desired >= _track_length ) {
            // "fast" waypoints are complete once the intermediate point reaches the destination
            if (_flags.fast_waypoint) {
                _flags.reached_destination = true;
            }else{
                // regular waypoints also require the copter to be within the waypoint radius
                Vector3f dist_to_dest = curr_pos - _destination;
                dist_to_dest.z *=_vert_track_scale;
                if( dist_to_dest.length() <= _wp_radius_cm ) {
                    _flags.reached_destination = true;
                }
            }
        }
    }
}

/// get_distance_to_destination - get horizontal distance to destination in cm
float AC_WPNav::get_distance_to_destination()
{
    // get current location
    Vector3f curr = _inav->get_position();
    return pythagorous2(_destination.x-curr.x,_destination.y-curr.y);
}

/// get_bearing_to_destination - get bearing to next waypoint in centi-degrees
int32_t AC_WPNav::get_bearing_to_destination()
{
    return get_bearing_cd(_inav->get_position(), _destination);
}

/// update_wpnav - run the wp controller - should be called at 10hz
void AC_WPNav::update_wpnav()
{
    uint32_t now = hal.scheduler->millis();
    float dt = (now - _wpnav_last_update) / 1000.0f;
    _wpnav_last_update = now;

    // catch if we've just been started
    if( dt >= 1.0 ) {
        dt = 0.0;
        reset_I();
    }else{
        // advance the target if necessary
        advance_target_along_track(dt);
    }

    // run loiter position controller
    get_loiter_position_to_velocity(dt, _wp_speed_cms);
}

///
/// shared methods
///

/// get_loiter_position_to_velocity - loiter position controller
///     converts desired position held in _target vector to desired velocity
void AC_WPNav::get_loiter_position_to_velocity(float dt, float max_speed_cms)
{
    Vector3f curr = _inav->get_position();
    float dist_error_total;

    float vel_sqrt;
    float vel_total;

    float linear_distance;      // the distace we swap between linear and sqrt.

    // calculate distance error
    dist_error.x = _target.x - curr.x;
    dist_error.y = _target.y - curr.y;

    linear_distance = MAX_LOITER_POS_ACCEL/(2*_pid_pos_lat->kP()*_pid_pos_lat->kP());
    _distance_to_target = linear_distance;      // for reporting purposes

    dist_error_total = safe_sqrt(dist_error.x*dist_error.x + dist_error.y*dist_error.y);
    if( dist_error_total > 2*linear_distance ) {
        vel_sqrt = safe_sqrt(2*MAX_LOITER_POS_ACCEL*(dist_error_total-linear_distance));
        desired_vel.x = vel_sqrt * dist_error.x/dist_error_total;
        desired_vel.y = vel_sqrt * dist_error.y/dist_error_total;
    }else{
        desired_vel.x = _pid_pos_lat->get_p(dist_error.x);
        desired_vel.y = _pid_pos_lon->get_p(dist_error.y);
    }

    // ensure velocity stays within limits
    vel_total = safe_sqrt(desired_vel.x*desired_vel.x + desired_vel.y*desired_vel.y);
    if( vel_total > max_speed_cms ) {
        desired_vel.x = max_speed_cms * desired_vel.x/vel_total;
        desired_vel.y = max_speed_cms * desired_vel.y/vel_total;
    }

    // call velocity to acceleration controller
    get_loiter_velocity_to_acceleration(desired_vel.x, desired_vel.y, dt);
}

/// get_loiter_velocity_to_acceleration - loiter velocity controller
///    converts desired velocities in lat/lon directions to accelerations in lat/lon frame
void AC_WPNav::get_loiter_velocity_to_acceleration(float vel_lat, float vel_lon, float dt)
{
    Vector3f vel_curr = _inav->get_velocity();  // current velocity in cm/s
    Vector3f vel_error;                         // The velocity error in cm/s.
    float accel_total;                          // total acceleration in cm/s/s

    // reset last velocity if this controller has just been engaged or dt is zero
    if( dt == 0.0 ) {
        desired_accel.x = 0;
        desired_accel.y = 0;
    } else {
        // feed forward desired acceleration calculation
        desired_accel.x = (vel_lat - _vel_last.x)/dt;
        desired_accel.y = (vel_lon - _vel_last.y)/dt;
    }

    // store this iteration's velocities for the next iteration
    _vel_last.x = vel_lat;
    _vel_last.y = vel_lon;

    // calculate velocity error
    vel_error.x = vel_lat - vel_curr.x;
    vel_error.y = vel_lon - vel_curr.y;

    // combine feed foward accel with PID outpu from velocity error
    desired_accel.x += _pid_rate_lat->get_pid(vel_error.x, dt);
    desired_accel.y += _pid_rate_lon->get_pid(vel_error.y, dt);

    // scale desired acceleration if it's beyond acceptable limit
    accel_total = safe_sqrt(desired_accel.x*desired_accel.x + desired_accel.y*desired_accel.y);
    if( accel_total > MAX_LOITER_VEL_ACCEL ) {
        desired_accel.x = MAX_LOITER_VEL_ACCEL * desired_accel.x/accel_total;
        desired_accel.y = MAX_LOITER_VEL_ACCEL * desired_accel.y/accel_total;
    }

    // call accel based controller with desired acceleration
    get_loiter_acceleration_to_lean_angles(desired_accel.x, desired_accel.y);
}

/// get_loiter_acceleration_to_lean_angles - loiter acceleration controller
///    converts desired accelerations provided in lat/lon frame to roll/pitch angles
void AC_WPNav::get_loiter_acceleration_to_lean_angles(float accel_lat, float accel_lon)
{
    float z_accel_meas = -GRAVITY_MSS * 100;    // gravity in cm/s/s
    float accel_forward;
    float accel_right;

    // To-Do: add 1hz filter to accel_lat, accel_lon

    // rotate accelerations into body forward-right frame
    accel_forward = accel_lat*_cos_yaw + accel_lon*_sin_yaw;
    accel_right = -accel_lat*_sin_yaw + accel_lon*_cos_yaw;

    // update angle targets that will be passed to stabilize controller
    _desired_roll = constrain_float((accel_right*_cos_pitch/(-z_accel_meas))*(18000/M_PI), -_lean_angle_max, _lean_angle_max);
    _desired_pitch = constrain_float((-accel_forward/(-z_accel_meas))*(18000/M_PI), -_lean_angle_max, _lean_angle_max);
}

// get_bearing_cd - return bearing in centi-degrees between two positions
// To-Do: move this to math library
float AC_WPNav::get_bearing_cd(const Vector3f &origin, const Vector3f &destination) const
{
    float bearing = 9000 + atan2f(-(destination.x-origin.x), destination.y-origin.y) * 5729.57795f;
    if (bearing < 0) {
        bearing += 36000;
    }
    return bearing;
}

/// reset_I - clears I terms from loiter PID controller
void AC_WPNav::reset_I()
{
    _pid_pos_lon->reset_I();
    _pid_pos_lat->reset_I();
    _pid_rate_lon->reset_I();
    _pid_rate_lat->reset_I();

    // set last velocity to current velocity
    _vel_last = _inav->get_velocity();

    // reset target velocity - only used by loiter controller's interpretation of pilot input
    _target_vel.x = 0;
    _target_vel.y = 0;

    // reset limited speed to zero to slow initial acceleration
    _limited_speed_xy_cms = 0;
}

/// calculate_wp_leash_length - calculates horizontal and vertical leash lengths for waypoint controller
void AC_WPNav::calculate_wp_leash_length(bool climb)
{

    // get loiter position P
    float kP = _pid_pos_lat->kP();

    // calculate horiztonal leash length
    if(_wp_speed_cms <= MAX_LOITER_POS_ACCEL / kP) {
        // linear leash length based on speed close in
        _wp_leash_xy = _wp_speed_cms / kP;
    }else{
        // leash length grows at sqrt of speed further out
        _wp_leash_xy = (MAX_LOITER_POS_ACCEL / (2.0*kP*kP)) + (_wp_speed_cms*_wp_speed_cms / (2*MAX_LOITER_POS_ACCEL));
    }

    // ensure leash is at least 1m long
    if( _wp_leash_xy < 100 ) {
        _wp_leash_xy = 100;
    }

    // calculate vertical leash length
    float speed_vert, leash_z;
    if( climb ) {
        speed_vert = _wp_speed_up_cms;
    }else{
        speed_vert = _wp_speed_down_cms;
    }
    if(speed_vert <= WPNAV_ALT_HOLD_ACCEL_MAX / WPNAV_ALT_HOLD_P) {
        // linear leash length based on speed close in
        leash_z = speed_vert / WPNAV_ALT_HOLD_P;
    }else{
        // leash length grows at sqrt of speed further out
        leash_z = (WPNAV_ALT_HOLD_ACCEL_MAX / (2.0*WPNAV_ALT_HOLD_P*WPNAV_ALT_HOLD_P)) + (speed_vert*speed_vert / (2*WPNAV_ALT_HOLD_ACCEL_MAX));
    }

    // ensure leash is at least 1m long
    if( leash_z < 100 ) {
        leash_z = 100;
    }

    // calculate vertical track scale used to give altitude equal weighting to horizontal position
    _vert_track_scale = _wp_leash_xy / leash_z;
}
