/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>
#include <AC_WPNav.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_WPNav::var_info[] PROGMEM = {
    // index 0 was used for the old orientation matrix

    // @Param: SPEED
    // @DisplayName: Waypoint Horizontal Speed Target
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain horizontally during a WP mission
    // @Units: cm/s
    // @Range: 0 2000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("SPEED",       0, AC_WPNav, _wp_speed_cms, WPNAV_WP_SPEED),

    // @Param: RADIUS
    // @DisplayName: Waypoint Radius
    // @Description: Defines the distance from a waypoint, that when crossed indicates the wp has been hit.
    // @Units: cm
    // @Range: 100 1000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RADIUS",      1, AC_WPNav, _wp_radius_cm, WPNAV_WP_RADIUS),

    // @Param: SPEED_UP
    // @DisplayName: Waypoint Climb Speed Target
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain while climbing during a WP mission
    // @Units: cm/s
    // @Range: 0 1000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("SPEED_UP",    2, AC_WPNav, _wp_speed_up_cms, WPNAV_WP_SPEED_UP),

    // @Param: SPEED_DN
    // @DisplayName: Waypoint Descent Speed Target
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain while descending during a WP mission
    // @Units: cm/s
    // @Range: 0 1000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("SPEED_DN",    3, AC_WPNav, _wp_speed_down_cms, WPNAV_WP_SPEED_DOWN),

    // @Param: LOIT_SPEED
    // @DisplayName: Loiter Horizontal Maximum Speed
    // @Description: Defines the maximum speed in cm/s which the aircraft will travel horizontally while in loiter mode
    // @Units: cm/s
    // @Range: 0 2000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("LOIT_SPEED",  4, AC_WPNav, _loiter_speed_cms, WPNAV_LOITER_SPEED),

    // @Param: ACCEL
    // @DisplayName: Waypoint Acceleration 
    // @Description: Defines the horizontal acceleration in cm/s/s used during missions
    // @Units: cm/s/s
    // @Range: 0 980
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("ACCEL",       5, AC_WPNav, _wp_accel_cms, WPNAV_ACCELERATION),

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
    _althold_kP(WPNAV_ALT_HOLD_P),
    _desired_roll(0),
    _desired_pitch(0),
    _target(0,0,0),
    _pilot_vel_forward_cms(0),
    _pilot_vel_right_cms(0),
    _target_vel(0,0,0),
    _vel_last(0,0,0),
    _loiter_leash(WPNAV_MIN_LEASH_LENGTH),
    _loiter_accel_cms(WPNAV_LOITER_ACCEL_MAX),
    _lean_angle_max_cd(MAX_LEAN_ANGLE),
    _wp_leash_xy(WPNAV_MIN_LEASH_LENGTH),
    _wp_leash_z(WPNAV_MIN_LEASH_LENGTH),
    _track_accel(0),
    _track_speed(0),
    _track_leash_length(0),
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
    float kP = _pid_pos_lat->kP();

    // calculate current velocity
    vel_total = safe_sqrt(velocity.x*velocity.x + velocity.y*velocity.y);

    // avoid divide by zero by using current position if the velocity is below 10cm/s, kP is very low or acceleration is zero
    if (vel_total < 10.0f || kP <= 0.0f || _wp_accel_cms <= 0.0f) {
        target = position;
        return;
    }

    // calculate point at which velocity switches from linear to sqrt
    linear_velocity = _wp_accel_cms/kP;

    // calculate distance within which we can stop
    if (vel_total < linear_velocity) {
        target_dist = vel_total/kP;
    } else {
        linear_distance = _wp_accel_cms/(2.0f*kP*kP);
        target_dist = linear_distance + (vel_total*vel_total)/(2.0f*_wp_accel_cms);
    }
    target_dist = constrain_float(target_dist, 0, _wp_leash_xy*2.0f);

    target.x = position.x + (target_dist * velocity.x / vel_total);
    target.y = position.y + (target_dist * velocity.y / vel_total);
    target.z = position.z;
}

/// set_loiter_target in cm from home
void AC_WPNav::set_loiter_target(const Vector3f& position)
{
    _target = position;
    _target_vel.x = 0;
    _target_vel.y = 0;
}

/// init_loiter_target - set initial loiter target based on current position and velocity
void AC_WPNav::init_loiter_target(const Vector3f& position, const Vector3f& velocity)
{
    // set target position and velocity based on current pos and velocity
    _target.x = position.x;
    _target.y = position.y;
    _target_vel.x = velocity.x;
    _target_vel.y = velocity.y;

    // initialise desired roll and pitch to current roll and pitch.  This avoids a random twitch between now and when the loiter controller is first run
    _desired_roll = constrain_int32(_ahrs->roll_sensor,-_lean_angle_max_cd,_lean_angle_max_cd);
    _desired_pitch = constrain_int32(_ahrs->pitch_sensor,-_lean_angle_max_cd,_lean_angle_max_cd);

    // initialise pilot input
    _pilot_vel_forward_cms = 0;
    _pilot_vel_right_cms = 0;

    // set last velocity to current velocity
    // To-Do: remove the line below by instead forcing reset_I to be called on the first loiter_update call
    _vel_last = _inav->get_velocity();
}

/// move_loiter_target - move loiter target by velocity provided in front/right directions in cm/s
void AC_WPNav::move_loiter_target(float control_roll, float control_pitch, float dt)
{
    // convert pilot input to desired velocity in cm/s
    _pilot_vel_forward_cms = -control_pitch * _loiter_accel_cms / 4500.0f;
    _pilot_vel_right_cms = control_roll * _loiter_accel_cms / 4500.0f;
}

/// translate_loiter_target_movements - consumes adjustments created by move_loiter_target
void AC_WPNav::translate_loiter_target_movements(float nav_dt)
{
    Vector2f target_vel_adj;
    float vel_total;

    // range check nav_dt
    if( nav_dt < 0 ) {
        return;
    }

    // check loiter speed and avoid divide by zero
    if( _loiter_speed_cms < 100.0f) {
        _loiter_speed_cms = 100.0f;
    }

    // rotate pilot input to lat/lon frame
    target_vel_adj.x = (_pilot_vel_forward_cms*_cos_yaw - _pilot_vel_right_cms*_sin_yaw);
    target_vel_adj.y = (_pilot_vel_forward_cms*_sin_yaw + _pilot_vel_right_cms*_cos_yaw);

    // add desired change in velocity to current target velocit
    _target_vel.x += target_vel_adj.x*nav_dt;
    _target_vel.y += target_vel_adj.y*nav_dt;
    if(_target_vel.x > 0 ) {
        _target_vel.x -= (_loiter_accel_cms-WPNAV_LOITER_ACCEL_MIN)*nav_dt*_target_vel.x/_loiter_speed_cms;
        _target_vel.x = max(_target_vel.x - WPNAV_LOITER_ACCEL_MIN*nav_dt, 0);
    }else if(_target_vel.x < 0) {
        _target_vel.x -= (_loiter_accel_cms-WPNAV_LOITER_ACCEL_MIN)*nav_dt*_target_vel.x/_loiter_speed_cms;
        _target_vel.x = min(_target_vel.x + WPNAV_LOITER_ACCEL_MIN*nav_dt, 0);
    }
    if(_target_vel.y > 0 ) {
        _target_vel.y -= (_loiter_accel_cms-WPNAV_LOITER_ACCEL_MIN)*nav_dt*_target_vel.y/_loiter_speed_cms;
        _target_vel.y = max(_target_vel.y - WPNAV_LOITER_ACCEL_MIN*nav_dt, 0);
    }else if(_target_vel.y < 0) {
        _target_vel.y -= (_loiter_accel_cms-WPNAV_LOITER_ACCEL_MIN)*nav_dt*_target_vel.y/_loiter_speed_cms;
        _target_vel.y = min(_target_vel.y + WPNAV_LOITER_ACCEL_MIN*nav_dt, 0);
    }

    // constrain the velocity vector and scale if necessary
    vel_total = safe_sqrt(_target_vel.x*_target_vel.x + _target_vel.y*_target_vel.y);
    if (vel_total > _loiter_speed_cms && vel_total > 0.0f) {
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
    if (distance > _loiter_leash && distance > 0.0f) {
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
    // calculate dt
    uint32_t now = hal.scheduler->millis();
    float dt = (now - _loiter_last_update) / 1000.0f;

    // catch if we've just been started
    if( dt >= 1.0 ) {
        dt = 0.0;
        reset_I();
        _loiter_step = 0;
    }

    // reset step back to 0 if 0.1 seconds has passed and we completed the last full cycle
    if (dt > 0.095f && _loiter_step > 3) {
        _loiter_step = 0;
    }

    // run loiter steps
    switch (_loiter_step) {
        case 0:
            // capture time since last iteration
            _loiter_dt = dt;
            _loiter_last_update = now;

            // translate any adjustments from pilot to loiter target
            translate_loiter_target_movements(_loiter_dt);
            _loiter_step++;
            break;
        case 1:
            // run loiter's position to velocity step
            get_loiter_position_to_velocity(_loiter_dt, WPNAV_LOITER_SPEED_MAX_TO_CORRECT_ERROR);
            _loiter_step++;
            break;
        case 2:
            // run loiter's velocity to acceleration step
            get_loiter_velocity_to_acceleration(desired_vel.x, desired_vel.y, _loiter_dt);
            _loiter_step++;
            break;
        case 3:
            // run loiter's acceleration to lean angle step
            get_loiter_acceleration_to_lean_angles(desired_accel.x, desired_accel.y);
            _loiter_step++;
            break;
    }
}

/// calculate_loiter_leash_length - calculates the maximum distance in cm that the target position may be from the current location
void AC_WPNav::calculate_loiter_leash_length()
{
    // get loiter position P
    float kP = _pid_pos_lat->kP();

    // check loiter speed
    if( _loiter_speed_cms < 100.0f) {
        _loiter_speed_cms = 100.0f;
    }

    // set loiter acceleration to 1/2 loiter speed
    _loiter_accel_cms = _loiter_speed_cms / 2.0f;

    // avoid divide by zero
    if (kP <= 0.0f || _wp_accel_cms <= 0.0f) {
        _loiter_leash = WPNAV_MIN_LEASH_LENGTH;
        return;
    }

    // calculate horizontal leash length
    if(WPNAV_LOITER_SPEED_MAX_TO_CORRECT_ERROR <= _wp_accel_cms / kP) {
        // linear leash length based on speed close in
        _loiter_leash = WPNAV_LOITER_SPEED_MAX_TO_CORRECT_ERROR / kP;
    }else{
        // leash length grows at sqrt of speed further out
        _loiter_leash = (_wp_accel_cms / (2.0f*kP*kP)) + (WPNAV_LOITER_SPEED_MAX_TO_CORRECT_ERROR*WPNAV_LOITER_SPEED_MAX_TO_CORRECT_ERROR / (2.0f*_wp_accel_cms));
    }

    // ensure leash is at least 1m long
    if( _loiter_leash < WPNAV_MIN_LEASH_LENGTH ) {
        _loiter_leash = WPNAV_MIN_LEASH_LENGTH;
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

    _track_length = pos_delta.length(); // get track length

    // calculate each axis' percentage of the total distance to the destination
    if (_track_length == 0.0f) {
        // avoid possible divide by zero
        _pos_delta_unit.x = 0;
        _pos_delta_unit.y = 0;
        _pos_delta_unit.z = 0;
    }else{
        _pos_delta_unit = pos_delta/_track_length;
    }
    calculate_wp_leash_length(climb);  // update leash lengths

    // initialise intermediate point to the origin
    _track_desired = 0;
    _target = origin;
    _flags.reached_destination = false;

    // initialise the limited speed to current speed along the track
    Vector3f curr_vel = _inav->get_velocity();
    // get speed along track (note: we convert vertical speed into horizontal speed equivalent)
    float speed_along_track = curr_vel.x * _pos_delta_unit.x + curr_vel.y * _pos_delta_unit.y + curr_vel.z * _pos_delta_unit.z;
    _limited_speed_xy_cms = constrain_float(speed_along_track,0,_wp_speed_cms);

    // default waypoint back to slow
    _flags.fast_waypoint = false;

    // initialise desired roll and pitch to current roll and pitch.  This avoids a random twitch between now and when the wpnav controller is first run
    _desired_roll = constrain_int32(_ahrs->roll_sensor,-_lean_angle_max_cd,_lean_angle_max_cd);
    _desired_pitch = constrain_int32(_ahrs->pitch_sensor,-_lean_angle_max_cd,_lean_angle_max_cd);

    // reset target velocity - only used by loiter controller's interpretation of pilot input
    _target_vel.x = 0;
    _target_vel.y = 0;
}

/// advance_target_along_track - move target location along track from origin to destination
void AC_WPNav::advance_target_along_track(float dt)
{
    float track_covered;
    Vector3f track_error;
    float track_desired_max;
    float track_desired_temp = _track_desired;
    float track_extra_max;

    // get current location
    Vector3f curr_pos = _inav->get_position();
    Vector3f curr_delta = curr_pos - _origin;

    // calculate how far along the track we are
    track_covered = curr_delta.x * _pos_delta_unit.x + curr_delta.y * _pos_delta_unit.y + curr_delta.z * _pos_delta_unit.z;

    Vector3f track_covered_pos = _pos_delta_unit * track_covered;
    track_error = curr_delta - track_covered_pos;

    // calculate the horizontal error
    float track_error_xy = safe_sqrt(track_error.x*track_error.x + track_error.y*track_error.y);

    // calculate the vertical error
    float track_error_z = fabsf(track_error.z);

    // calculate how far along the track we could move the intermediate target before reaching the end of the leash
    track_extra_max = min(_track_leash_length*(_wp_leash_z-track_error_z)/_wp_leash_z, _track_leash_length*(_wp_leash_xy-track_error_xy)/_wp_leash_xy);
    if(track_extra_max <0) {
        track_desired_max = track_covered;
    }else{
        track_desired_max = track_covered + track_extra_max;
    }

    // get current velocity
    Vector3f curr_vel = _inav->get_velocity();
    // get speed along track
    float speed_along_track = curr_vel.x * _pos_delta_unit.x + curr_vel.y * _pos_delta_unit.y + curr_vel.z * _pos_delta_unit.z;

    // calculate point at which velocity switches from linear to sqrt
    float linear_velocity = _wp_speed_cms;
    float kP = _pid_pos_lat->kP();
    if (kP >= 0.0f) {   // avoid divide by zero
        linear_velocity = _track_accel/kP;
    }

    // let the limited_speed_xy_cms be some range above or below current velocity along track
    if (speed_along_track < -linear_velocity) {
        // we are travelling fast in the opposite direction of travel to the waypoint so do not move the intermediate point
        _limited_speed_xy_cms = 0;
    }else{
        // increase intermediate target point's velocity if not yet at target speed (we will limit it below)
        if(dt > 0) {
            if(track_desired_max > _track_desired) {
                _limited_speed_xy_cms += 2.0 * _track_accel * dt;
            }else{
                // do nothing, velocity stays constant
                _track_desired = track_desired_max;
            }
        }
        // do not go over top speed
        if(_limited_speed_xy_cms > _track_speed) {
            _limited_speed_xy_cms = _track_speed;
        }
        // if our current velocity is within the linear velocity range limit the intermediate point's velocity to be no more than the linear_velocity above or below our current velocity
        if (fabsf(speed_along_track) < linear_velocity) {
            _limited_speed_xy_cms = constrain_float(_limited_speed_xy_cms,speed_along_track-linear_velocity,speed_along_track+linear_velocity);
        }
    }
    // advance the current target
    track_desired_temp += _limited_speed_xy_cms * dt;

    // do not let desired point go past the end of the segment
    track_desired_temp = constrain_float(track_desired_temp, 0, _track_length);
    _track_desired = max(_track_desired, track_desired_temp);

    // recalculate the desired position
    _target = _origin + _pos_delta_unit * _track_desired;

    // check if we've reached the waypoint
    if( !_flags.reached_destination ) {
        if( _track_desired >= _track_length ) {
            // "fast" waypoints are complete once the intermediate point reaches the destination
            if (_flags.fast_waypoint) {
                _flags.reached_destination = true;
            }else{
                // regular waypoints also require the copter to be within the waypoint radius
                Vector3f dist_to_dest = curr_pos - _destination;
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
    // calculate dt
    uint32_t now = hal.scheduler->millis();
    float dt = (now - _wpnav_last_update) / 1000.0f;

    // catch if we've just been started
    if( dt >= 1.0 ) {
        dt = 0.0;
        reset_I();
        _wpnav_step = 0;
    }

    // reset step back to 0 if 0.1 seconds has passed and we completed the last full cycle
    if (dt > 0.095f && _wpnav_step > 3) {
        _wpnav_step = 0;
    }

    // run loiter steps
    switch (_wpnav_step) {
        case 0:
            // capture time since last iteration
            _wpnav_dt = dt;
            _wpnav_last_update = now;

            // advance the target if necessary
            if (dt > 0.0f) {
                advance_target_along_track(dt);
            }
            _wpnav_step++;
            break;
        case 1:
            // run loiter's position to velocity step
            get_loiter_position_to_velocity(_wpnav_dt, _wp_speed_cms);
            _wpnav_step++;
            break;
        case 2:
            // run loiter's velocity to acceleration step
            get_loiter_velocity_to_acceleration(desired_vel.x, desired_vel.y, _wpnav_dt);
            _wpnav_step++;
            break;
        case 3:
            // run loiter's acceleration to lean angle step
            get_loiter_acceleration_to_lean_angles(desired_accel.x, desired_accel.y);
            _wpnav_step++;
            break;
    }
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
    float kP = _pid_pos_lat->kP();

    // avoid divide by zero
    if (kP <= 0.0f) {
        desired_vel.x = 0.0;
        desired_vel.y = 0.0;
    }else{
        // calculate distance error
        dist_error.x = _target.x - curr.x;
        dist_error.y = _target.y - curr.y;

        linear_distance = _wp_accel_cms/(2.0f*kP*kP);

        dist_error_total = safe_sqrt(dist_error.x*dist_error.x + dist_error.y*dist_error.y);
        _distance_to_target = dist_error_total;      // for reporting purposes

        if( dist_error_total > 2.0f*linear_distance ) {
            vel_sqrt = safe_sqrt(2.0f*_wp_accel_cms*(dist_error_total-linear_distance));
            desired_vel.x = vel_sqrt * dist_error.x/dist_error_total;
            desired_vel.y = vel_sqrt * dist_error.y/dist_error_total;
        }else{
            desired_vel.x = _pid_pos_lat->kP() * dist_error.x;
            desired_vel.y = _pid_pos_lon->kP() * dist_error.y;
        }

        // ensure velocity stays within limits
        vel_total = safe_sqrt(desired_vel.x*desired_vel.x + desired_vel.y*desired_vel.y);
        if( vel_total > max_speed_cms ) {
            desired_vel.x = max_speed_cms * desired_vel.x/vel_total;
            desired_vel.y = max_speed_cms * desired_vel.y/vel_total;
        }

        // feed forward velocity request
        desired_vel.x += _target_vel.x;
        desired_vel.y += _target_vel.y;
    }
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
    if( accel_total > WPNAV_ACCEL_MAX ) {
        desired_accel.x = WPNAV_ACCEL_MAX * desired_accel.x/accel_total;
        desired_accel.y = WPNAV_ACCEL_MAX * desired_accel.y/accel_total;
    }
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
    _desired_roll = constrain_float(fast_atan(accel_right*_cos_pitch/(-z_accel_meas))*(18000/M_PI), -_lean_angle_max_cd, _lean_angle_max_cd);
    _desired_pitch = constrain_float(fast_atan(-accel_forward/(-z_accel_meas))*(18000/M_PI), -_lean_angle_max_cd, _lean_angle_max_cd);
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
}

/// calculate_wp_leash_length - calculates horizontal and vertical leash lengths for waypoint controller
void AC_WPNav::calculate_wp_leash_length(bool climb)
{

    // get loiter position P
    float kP = _pid_pos_lat->kP();

    // sanity check acceleration and avoid divide by zero
    if (_wp_accel_cms <= 0.0f) {
        _wp_accel_cms = WPNAV_ACCELERATION_MIN;
    }
    
    // avoid divide by zero
    if (kP <= 0.0f) {
        _wp_leash_xy = WPNAV_MIN_LEASH_LENGTH;
        return;
    }
    // calculate horiztonal leash length
    if(_wp_speed_cms <= _wp_accel_cms / kP) {
        // linear leash length based on speed close in
        _wp_leash_xy = _wp_speed_cms / kP;
    }else{
        // leash length grows at sqrt of speed further out
        _wp_leash_xy = (_wp_accel_cms / (2.0f*kP*kP)) + (_wp_speed_cms*_wp_speed_cms / (2.0f*_wp_accel_cms));
    }

    // ensure leash is at least 1m long
    if( _wp_leash_xy < WPNAV_MIN_LEASH_LENGTH ) {
        _wp_leash_xy = WPNAV_MIN_LEASH_LENGTH;
    }

    // calculate vertical leash length
    float speed_vert;
    if( climb ) {
        speed_vert = _wp_speed_up_cms;
    }else{
        speed_vert = _wp_speed_down_cms;
    }
    if(speed_vert <= WPNAV_ALT_HOLD_ACCEL_MAX / _althold_kP) {
        // linear leash length based on speed close in
        _wp_leash_z = speed_vert / _althold_kP;
    }else{
        // leash length grows at sqrt of speed further out
        _wp_leash_z = (WPNAV_ALT_HOLD_ACCEL_MAX / (2.0*_althold_kP*_althold_kP)) + (speed_vert*speed_vert / (2*WPNAV_ALT_HOLD_ACCEL_MAX));
    }

    // ensure leash is at least 1m long
    if( _wp_leash_z < WPNAV_MIN_LEASH_LENGTH ) {
        _wp_leash_z = WPNAV_MIN_LEASH_LENGTH;
    }

    // length of the unit direction vector in the horizontal
    float pos_delta_unit_xy = sqrt(_pos_delta_unit.x*_pos_delta_unit.x+_pos_delta_unit.y*_pos_delta_unit.y);
    float pos_delta_unit_z = fabsf(_pos_delta_unit.z);

    // calculate the maximum acceleration, maximum velocity, and leash length in the direction of travel
    if(pos_delta_unit_z == 0 && pos_delta_unit_xy == 0){
        _track_accel = 0;
        _track_speed = 0;
        _track_leash_length = WPNAV_MIN_LEASH_LENGTH;
    }else if(_pos_delta_unit.z == 0){
        _track_accel = _wp_accel_cms/pos_delta_unit_xy;
        _track_speed = _wp_speed_cms/pos_delta_unit_xy;
        _track_leash_length = _wp_leash_xy/pos_delta_unit_xy;
    }else if(pos_delta_unit_xy == 0){
        _track_accel = WPNAV_ALT_HOLD_ACCEL_MAX/pos_delta_unit_z;
        _track_speed = speed_vert/pos_delta_unit_z;
        _track_leash_length = _wp_leash_z/pos_delta_unit_z;
    }else{	
        _track_accel = min(WPNAV_ALT_HOLD_ACCEL_MAX/pos_delta_unit_z, _wp_accel_cms/pos_delta_unit_xy);
        _track_speed = min(speed_vert/pos_delta_unit_z, _wp_speed_cms/pos_delta_unit_xy);
        _track_leash_length = min(_wp_leash_z/pos_delta_unit_z, _wp_leash_xy/pos_delta_unit_xy);
    }
}
