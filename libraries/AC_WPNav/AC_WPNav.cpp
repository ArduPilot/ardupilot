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
AC_WPNav::AC_WPNav(AP_InertialNav* inav, APM_PI* pid_pos_lat, APM_PI* pid_pos_lon, AC_PID* pid_lat_rate, AC_PID* pid_lon_rate) :
    _inav(inav),
    _pid_pos_lat(pid_pos_lat),
    _pid_pos_lon(pid_pos_lon),
    _pid_rate_lat(pid_lat_rate),
    _pid_rate_lon(pid_lon_rate)
{
    AP_Param::setup_object_defaults(this, var_info);
}

///
/// simple loiter controller
///

/// set_loiter_target in cm from home
void AC_WPNav::set_loiter_target(const Vector3f& position)
{
    _state = WPNAV_STATE_LOITER;
    _target = position;
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
    _state = WPNAV_STATE_WPNAV;
    _origin = origin;
    _destination = destination;
    _pos_delta = _destination - _origin;
    _track_length = _pos_delta.length();
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
        cross_track_dist = fabs(curr.x - _destination.x);
        track_covered = fabs(curr.y - _origin.y);
    }else if(_pos_delta.y == 0) {
        // y is zero
        cross_track_dist = fabs(curr.y - _destination.y);
        track_covered = fabs(curr.x - _origin.x);
    }else{
        // both x and y non zero
        line_a = _pos_delta.y;
        line_b = -_pos_delta.x;
        line_c = _pos_delta.x * _origin.y - _pos_delta.y * _origin.x;
        line_m = line_a / line_b;
        cross_track_dist = abs(line_a * curr.x + line_b * curr.y + line_c ) / _track_length;

        line_m = 1/line_m;
        line_a = line_m;
        line_b = -1;
        line_c = curr.y - line_m * curr.x;

        // calculate the distance to the closest point along the track and it's distance from the origin
        track_covered = abs(line_a*_origin.x + line_b*_origin.y + line_c) / safe_sqrt(line_a*line_a+line_b*line_b);
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
}

///
/// shared methods
///

/// update - run the loiter and wpnav controllers - should be called at 10hz
void AC_WPNav::update()
{
    float dt = hal.scheduler->millis() - _last_update;

    // prevent run up in dt value
    dt = min(dt, 1.0f);

    // advance the target if necessary
    if( _state == WPNAV_STATE_WPNAV ) {
        advance_target_along_track(_speed_cms, dt);
    }

    // run loiter position controller
    get_loiter_pos_lat_lon(_target.x, _target.y, dt);
}

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

    // rotate accelerations into body forward-right frame
    accel_forward = accel_lat*_cos_yaw + accel_lon*_sin_yaw;
    accel_right = -accel_lat*_sin_yaw + accel_lon*_cos_yaw;

    // update angle targets that will be passed to stabilize controller
    _desired_roll = constrain((accel_right/(-z_accel_meas))*(18000/M_PI), -4500, 4500);
    _desired_pitch = constrain((-accel_forward/(-z_accel_meas*_cos_roll))*(18000/M_PI), -4500, 4500);
}
