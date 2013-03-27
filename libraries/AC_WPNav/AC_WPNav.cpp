/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>
#include <AC_WPNav.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_WPNav::var_info[] PROGMEM = {
    // index 0 was used for the old orientation matrix

    // @Param: SPEED
    // @DisplayName: Speed in cm/s to travel between waypoints
    // @Description: The desired horizontal speed in cm/s while travelling between waypoints
    // @Range: 0 1000
    // @Increment: 50
    AP_GROUPINFO("SPEED",    0, AC_WPNav, _speed_cms, WP_SPEED),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_WPNav::AC_WPNav(AP_InertialNav* inav, APM_PI* pid_pos_lat, APM_PI* pid_pos_lon, AC_PID* pid_rate_lat, AC_PID* pid_rate_lon) :
    _inav(inav),
    _pid_pos_lat(pid_pos_lat),
    _pid_pos_lon(pid_pos_lon),
    _pid_rate_lat(pid_rate_lat),
    _pid_rate_lon(pid_rate_lon)
{
    AP_Param::setup_object_defaults(this, var_info);
}

///
/// simple loiter controller
///

/// set_loiter_target - set initial loiter target based on current position and velocity
void AC_WPNav::set_loiter_target(const Vector3f& position, const Vector3f& velocity)
{
    float linear_distance;      // half the distace we swap between linear and sqrt and the distace we offset sqrt.
    float linear_velocity;      // the velocity we swap between linear and sqrt.
    float vel_total;
    float target_dist;

    // avoid divide by zero
    if( _pid_pos_lat->kP() <= 0.1 ) {
        set_loiter_target(position);
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
    target_dist = constrain(target_dist, 0, MAX_LOITER_OVERSHOOT);

    _target.x = position.x + (target_dist * velocity.x / vel_total);
    _target.y = position.y + (target_dist * velocity.y / vel_total);
}

/// move_loiter_target - move loiter target by velocity provided in front/right directions in cm/s
void AC_WPNav::move_loiter_target(int16_t vel_forward_cms, int16_t vel_right_cms, float dt)
{
    int32_t vel_lat;
    int32_t vel_lon;
    int32_t vel_total;

    vel_lat = vel_forward_cms*_cos_yaw - vel_right_cms*_sin_yaw;
    vel_lon = vel_forward_cms*_sin_yaw + vel_right_cms*_cos_yaw;

    // constrain the velocity vector and scale if necessary
    vel_total = safe_sqrt(vel_lat*vel_lat + vel_lon*vel_lon);
    if( vel_total > MAX_LOITER_POS_VEL_VELOCITY ) {
        vel_lat = MAX_LOITER_POS_VEL_VELOCITY * vel_lat/vel_total;
        vel_lon = MAX_LOITER_POS_VEL_VELOCITY * vel_lon/vel_total;
    }

    // update target position
    _target.x += vel_lat * dt;
    _target.y += vel_lon * dt;
}

/// get_distance_to_target - get horizontal distance to loiter target in cm
float AC_WPNav::get_distance_to_target()
{
    return _distance_to_target;
}

/// get_bearing_to_target - get bearing to loiter target in centi-degrees
int32_t AC_WPNav::get_bearing_to_target()
{
    return get_bearing_cd(_inav->get_position(), _target);
}

/// update_loiter - run the loiter controller - should be called at 10hz
void AC_WPNav::update_loiter()
{
    uint32_t now = hal.scheduler->millis();
    float dt = (now - _last_update) / 1000.0f;
    _last_update = now;

    // catch if we've just been started
    if( dt >= 1.0 ) {
        dt = 0.0;
        reset_I();
    }

    // run loiter position controller
    get_loiter_pos_lat_lon(_target.x, _target.y, dt);
}

///
/// waypoint navigation
///

/// set_destination - set destination using cm from home
void AC_WPNav::set_destination(const Vector3f& destination)
{
    set_origin_and_destination(_inav->get_position(), destination);
}

/// set_origin_and_destination - set origin and destination using lat/lon coordinates
void AC_WPNav::set_origin_and_destination(const Vector3f& origin, const Vector3f& destination)
{
    _origin = origin;
    _destination = destination;
    _pos_delta = _destination - _origin;
    _track_length = safe_sqrt(_pos_delta.x * _pos_delta.x + _pos_delta.y * _pos_delta.y);  
    _track_desired = 0;
}

/// advance_target_along_track - move target location along track from origin to destination
void AC_WPNav::advance_target_along_track(float velocity_cms, float dt)
{
    float cross_track_dist;
    float track_covered;
    float track_desired_max;
    float line_a, line_b, line_c, line_m;

    // get current location
    Vector3f curr = _inav->get_position();

    // check for zero length segment
    if( _pos_delta.x == 0 && _pos_delta.y == 0) {
        _target = _destination;
        return;
    }

    if( _pos_delta.x == 0 ) {
        // x is zero
        cross_track_dist = fabsf(curr.x - _destination.x);
        track_covered = fabsf(curr.y - _origin.y);
    }else if(_pos_delta.y == 0) {
        // y is zero
        cross_track_dist = fabsf(curr.y - _destination.y);
        track_covered = fabsf(curr.x - _origin.x);
    }else{
        // both x and y non zero
        line_a = _pos_delta.y;
        line_b = -_pos_delta.x;
        line_c = _pos_delta.x * _origin.y - _pos_delta.y * _origin.x;
        line_m = line_a / line_b;
        cross_track_dist = fabsf(line_a * curr.x + line_b * curr.y + line_c ) / _track_length;

        line_m = 1/line_m;
        line_a = line_m;
        line_b = -1;
        line_c = curr.y - line_m * curr.x;

        // calculate the distance to the closest point along the track and it's distance from the origin
        track_covered = fabsf(line_a*_origin.x + line_b*_origin.y + line_c) / safe_sqrt(line_a*line_a+line_b*line_b);
    }

    // maximum distance along the track that we will allow (stops target point from getting too far from the current position)
    track_desired_max = track_covered + safe_sqrt(WPINAV_MAX_POS_ERROR*WPINAV_MAX_POS_ERROR-cross_track_dist*cross_track_dist);

    // advance the current target
    _track_desired += velocity_cms * dt;

    // constrain the target from moving too far
    if( _track_desired > track_desired_max ) {
        _track_desired = track_desired_max;
    }
    if( _track_desired > _track_length ) {
        _track_desired = _track_length;
    }

    // recalculate the desired position
    float track_length_pct = _track_desired/_track_length;
    _target.x = _origin.x + _pos_delta.x * track_length_pct;
    _target.y = _origin.y + _pos_delta.y * track_length_pct;
    _target.z = _destination.z;
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
    float dt = (now - _last_update) / 1000.0f;
    _last_update = now;

    // catch if we've just been started
    if( dt >= 1.0 ) {
        dt = 0.0;
        reset_I();
    }else{
        // advance the target if necessary
        advance_target_along_track(_speed_cms, dt);
    }

    // run loiter position controller
    get_loiter_pos_lat_lon(_target.x, _target.y, dt);
}

///
/// shared methods
///

// get_loiter_pos_lat_lon - loiter position controller
//     converts desired position provided as distance from home in lat/lon directions to desired velocity
void AC_WPNav::get_loiter_pos_lat_lon(int32_t target_lat_from_home, int32_t target_lon_from_home, float dt)
{
    float dist_error_lat;
    int32_t desired_vel_lat;

    float dist_error_lon;
    int32_t desired_vel_lon;

    int32_t dist_error_total;

    int16_t vel_sqrt;
    int32_t vel_total;

    int16_t linear_distance;      // the distace we swap between linear and sqrt.

    // calculate distance error
    dist_error_lat = target_lat_from_home - _inav->get_latitude_diff();
    dist_error_lon = target_lon_from_home - _inav->get_longitude_diff();

    linear_distance = MAX_LOITER_POS_ACCEL/(2*_pid_pos_lat->kP()*_pid_pos_lat->kP());
    _distance_to_target = linear_distance;      // for reporting purposes

    dist_error_total = safe_sqrt(dist_error_lat*dist_error_lat + dist_error_lon*dist_error_lon);
    if( dist_error_total > 2*linear_distance ) {
        vel_sqrt = constrain(safe_sqrt(2*MAX_LOITER_POS_ACCEL*(dist_error_total-linear_distance)),0,1000);
        desired_vel_lat = vel_sqrt * dist_error_lat/dist_error_total;
        desired_vel_lon = vel_sqrt * dist_error_lon/dist_error_total;
    }else{
        desired_vel_lat = _pid_pos_lat->get_p(dist_error_lat);
        desired_vel_lon = _pid_pos_lon->get_p(dist_error_lon);
    }

    vel_total = safe_sqrt(desired_vel_lat*desired_vel_lat + desired_vel_lon*desired_vel_lon);
    if( vel_total > MAX_LOITER_POS_VELOCITY ) {
        desired_vel_lat = MAX_LOITER_POS_VELOCITY * desired_vel_lat/vel_total;
        desired_vel_lon = MAX_LOITER_POS_VELOCITY * desired_vel_lon/vel_total;
    }

    get_loiter_vel_lat_lon(desired_vel_lat, desired_vel_lon, dt);
}

// get_loiter_vel_lat_lon - loiter velocity controller
//    converts desired velocities in lat/lon frame to accelerations in lat/lon frame
void AC_WPNav::get_loiter_vel_lat_lon(int16_t vel_lat, int16_t vel_lon, float dt)
{
    float speed_error_lat = 0;     // The velocity in cm/s.
    float speed_error_lon = 0;     // The velocity in cm/s.

    float speed_lat = _inav->get_latitude_velocity();
    float speed_lon = _inav->get_longitude_velocity();

    int32_t accel_lat;
    int32_t accel_lon;
    int32_t accel_total;

    int16_t lat_p,lat_i,lat_d;
    int16_t lon_p,lon_i,lon_d;

    // calculate vel error
    speed_error_lat = vel_lat - speed_lat;
    speed_error_lon = vel_lon - speed_lon;

    lat_p   = _pid_rate_lat->get_p(speed_error_lat);
    lat_i   = _pid_rate_lat->get_i(speed_error_lat, dt);
    lat_d   = _pid_rate_lat->get_d(speed_error_lat, dt);

    lon_p   = _pid_rate_lon->get_p(speed_error_lon);
    lon_i   = _pid_rate_lon->get_i(speed_error_lon, dt);
    lon_d   = _pid_rate_lon->get_d(speed_error_lon, dt);

    accel_lat = (lat_p+lat_i+lat_d);
    accel_lon = (lon_p+lon_i+lon_d);

    accel_total = safe_sqrt(accel_lat*accel_lat + accel_lon*accel_lon);

    if( accel_total > MAX_LOITER_VEL_ACCEL ) {
        accel_lat = MAX_LOITER_VEL_ACCEL * accel_lat/accel_total;
        accel_lon = MAX_LOITER_VEL_ACCEL * accel_lon/accel_total;
    }

    get_loiter_accel_lat_lon(accel_lat, accel_lon);
}

// get_loiter_accel_lat_lon - loiter acceration controller
//    converts desired accelerations provided in lat/lon frame to roll/pitch angles
void AC_WPNav::get_loiter_accel_lat_lon(int16_t accel_lat, int16_t accel_lon)
{
    float z_accel_meas = -AP_INTERTIALNAV_GRAVITY * 100;    // gravity in cm/s/s
    float accel_forward;
    float accel_right;

    // To-Do: add 1hz filter to accel_lat, accel_lon

    // rotate accelerations into body forward-right frame
    accel_forward = accel_lat*_cos_yaw + accel_lon*_sin_yaw;
    accel_right = -accel_lat*_sin_yaw + accel_lon*_cos_yaw;

    // update angle targets that will be passed to stabilize controller
    _desired_roll = constrain((accel_right/(-z_accel_meas))*(18000/M_PI), -4500, 4500);
    _desired_pitch = constrain((-accel_forward/(-z_accel_meas*_cos_roll))*(18000/M_PI), -4500, 4500);
}

// get_bearing_cd - return bearing in centi-degrees between two positions
// To-Do: move this to math library
float AC_WPNav::get_bearing_cd(const Vector3f origin, const Vector3f destination)
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
}