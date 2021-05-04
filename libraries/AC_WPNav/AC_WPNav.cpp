#include <AP_HAL/AP_HAL.h>
#include "AC_WPNav.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_WPNav::var_info[] = {
    // index 0 was used for the old orientation matrix

    // @Param: SPEED
    // @DisplayName: Waypoint Horizontal Speed Target
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain horizontally during a WP mission
    // @Units: cm/s
    // @Range: 20 2000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("SPEED",       0, AC_WPNav, _wp_speed_cms, WPNAV_WP_SPEED),

    // @Param: RADIUS
    // @DisplayName: Waypoint Radius
    // @Description: Defines the distance from a waypoint, that when crossed indicates the wp has been hit.
    // @Units: cm
    // @Range: 5 1000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RADIUS",      1, AC_WPNav, _wp_radius_cm, WPNAV_WP_RADIUS),

    // @Param: SPEED_UP
    // @DisplayName: Waypoint Climb Speed Target
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain while climbing during a WP mission
    // @Units: cm/s
    // @Range: 10 1000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("SPEED_UP",    2, AC_WPNav, _wp_speed_up_cms, WPNAV_WP_SPEED_UP),

    // @Param: SPEED_DN
    // @DisplayName: Waypoint Descent Speed Target
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain while descending during a WP mission
    // @Units: cm/s
    // @Range: 10 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("SPEED_DN",    3, AC_WPNav, _wp_speed_down_cms, WPNAV_WP_SPEED_DOWN),

    // @Param: ACCEL
    // @DisplayName: Waypoint Acceleration 
    // @Description: Defines the horizontal acceleration in cm/s/s used during missions
    // @Units: cm/s/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("ACCEL",       5, AC_WPNav, _wp_accel_cmss, WPNAV_ACCELERATION),

    // @Param: ACCEL_Z
    // @DisplayName: Waypoint Vertical Acceleration
    // @Description: Defines the vertical acceleration in cm/s/s used during missions
    // @Units: cm/s/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("ACCEL_Z",     6, AC_WPNav, _wp_accel_z_cmss, WPNAV_WP_ACCEL_Z_DEFAULT),

    // @Param: RFND_USE
    // @DisplayName: Waypoint missions use rangefinder for terrain following
    // @Description: This controls if waypoint missions use rangefinder for terrain following
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO("RFND_USE",   10, AC_WPNav, _rangefinder_use, 1),

    // @Param: JERK
    // @DisplayName: Waypoint Jerk
    // @Description: Defines the horizontal jerk in m/s/s used during missions
    // @Units: m/s/s
    // @Range: 1 20
    // @User: Standard
    AP_GROUPINFO("JERK",   11, AC_WPNav, _wp_jerk, 1.0f),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_WPNav::AC_WPNav(const AP_InertialNav& inav, const AP_AHRS_View& ahrs, AC_PosControl& pos_control, const AC_AttitudeControl& attitude_control) :
    _inav(inav),
    _ahrs(ahrs),
    _pos_control(pos_control),
    _attitude_control(attitude_control)
{
    AP_Param::setup_object_defaults(this, var_info);

    // init flags
    _flags.reached_destination = false;
    _flags.fast_waypoint = false;

    // sanity check some parameters
    _wp_accel_cmss = MIN(_wp_accel_cmss, GRAVITY_MSS * 100.0f * tanf(ToRad(_attitude_control.lean_angle_max() * 0.01f)));
    _wp_radius_cm = MAX(_wp_radius_cm, WPNAV_WP_RADIUS_MIN);
}

// get expected source of terrain data if alt-above-terrain command is executed (used by Copter's ModeRTL)
AC_WPNav::TerrainSource AC_WPNav::get_terrain_source() const
{
    // use range finder if connected
    if (_rangefinder_available && _rangefinder_use) {
        return AC_WPNav::TerrainSource::TERRAIN_FROM_RANGEFINDER;
    }
#if AP_TERRAIN_AVAILABLE
    if ((_terrain != nullptr) && _terrain->enabled()) {
        return AC_WPNav::TerrainSource::TERRAIN_FROM_TERRAINDATABASE;
    } else {
        return AC_WPNav::TerrainSource::TERRAIN_UNAVAILABLE;
    }
#else
    return AC_WPNav::TerrainSource::TERRAIN_UNAVAILABLE;
#endif
}

///
/// waypoint navigation
///

/// wp_and_spline_init - initialise straight line and spline waypoint controllers
///     speed_cms should be a positive value or left at zero to use the default speed
///     updates target roll, pitch targets and I terms based on vehicle lean angles
///     should be called once before the waypoint controller is used but does not need to be called before subsequent updates to destination
void AC_WPNav::wp_and_spline_init(float speed_cms)
{
    // check _wp_accel_cmss is reasonable
    if (_wp_accel_cmss <= 0) {
        _wp_accel_cmss.set_and_save(WPNAV_ACCELERATION);
    }

    // initialise position controller
    _pos_control.set_desired_accel_xy(0.0f,0.0f);
    _pos_control.init_xy_controller();

    // initialise feed forward velocity to zero
    _pos_control.set_desired_velocity_xy(0.0f, 0.0f);

    // initialize the desired wp speed if not already done
    _wp_desired_speed_xy_cms = is_positive(speed_cms) ? speed_cms : _wp_speed_cms;

    // initialise position controller speed and acceleration
    _pos_control.set_max_speed_xy(_wp_desired_speed_xy_cms);
    _pos_control.set_max_accel_xy(_wp_accel_cmss);
    _pos_control.set_max_speed_z(-_wp_speed_down_cms, _wp_speed_up_cms);
    _pos_control.set_max_accel_z(_wp_accel_z_cmss);
    _pos_control.calc_leash_length_xy();
    _pos_control.calc_leash_length_z();

    // calculate scurve jerk and jerk time
    if (!is_positive(_wp_jerk)) {
        _wp_jerk = _wp_accel_cmss;
    }
    calc_scurve_jerk_and_jerk_time();

    _scurve_prev_leg.init();
    _scurve_this_leg.init();
    _scurve_next_leg.init();
    _track_scalar_dt = 1.0f;

    // reset input shaped terrain offsets
    _pos_terrain_offset = 0.0f;
    _vel_terrain_offset = 0.0f;
    _accel_terrain_offset = 0.0f;

    // set flag so get_yaw() returns current heading target
    _flags.wp_yaw_set = false;
    _flags.reached_destination = false;
    _flags.fast_waypoint = false;

    // initialise origin and destination to stopping point
    Vector3f stopping_point;
    get_wp_stopping_point(stopping_point);
    _origin = _destination = stopping_point;
    _terrain_alt = false;
    _this_leg_is_spline = false;
}

/// set_speed_xy - allows main code to pass target horizontal velocity for wp navigation
void AC_WPNav::set_speed_xy(float speed_cms)
{
    // range check target speed
    if (speed_cms >= WPNAV_WP_SPEED_MIN) {
        _wp_desired_speed_xy_cms = speed_cms;
        _pos_control.set_max_speed_xy(_wp_desired_speed_xy_cms);
        update_track_with_speed_accel_limits();
    }
}

/// set current target climb rate during wp navigation
void AC_WPNav::set_speed_up(float speed_up_cms)
{
    _pos_control.set_max_speed_z(_pos_control.get_max_speed_down(), speed_up_cms);
    update_track_with_speed_accel_limits();
}

/// set current target descent rate during wp navigation
void AC_WPNav::set_speed_down(float speed_down_cms)
{
    _pos_control.set_max_speed_z(speed_down_cms, _pos_control.get_max_speed_up());
    update_track_with_speed_accel_limits();
}

/// set_wp_destination waypoint using location class
///     returns false if conversion from location to vector from ekf origin cannot be calculated
bool AC_WPNav::set_wp_destination_loc(const Location& destination)
{
    bool terr_alt;
    Vector3f dest_neu;

    // convert destination location to vector
    if (!get_vector_NEU(destination, dest_neu, terr_alt)) {
        return false;
    }

    // set target as vector from EKF origin
    return set_wp_destination(dest_neu, terr_alt);
}

/// set next destination using location class
///     returns false if conversion from location to vector from ekf origin cannot be calculated
bool AC_WPNav::set_wp_destination_next_loc(const Location& destination)
{
    bool terr_alt;
    Vector3f dest_neu;

    // convert destination location to vector
    if (!get_vector_NEU(destination, dest_neu, terr_alt)) {
        return false;
    }

    // set target as vector from EKF origin
    return set_wp_destination_next(dest_neu, terr_alt);
}

// get destination as a location.  Altitude frame will be absolute (AMSL) or above terrain
// returns false if unable to return a destination (for example if origin has not yet been set)
bool AC_WPNav::get_wp_destination_loc(Location& destination) const
{
    if (!AP::ahrs().get_origin(destination)) {
        return false;
    }
    destination.offset(_destination.x*0.01f, _destination.y*0.01f);
    if (_terrain_alt) {
        destination.set_alt_cm(_destination.z, Location::AltFrame::ABOVE_TERRAIN);
    } else {
        destination.alt += _destination.z;
    }
    return true;
}

/// set_wp_destination - set destination waypoints using position vectors (distance from ekf origin in cm)
///     terrain_alt should be true if destination.z is an altitude above terrain (false if alt-above-ekf-origin)
///     returns false on failure (likely caused by missing terrain data)
bool AC_WPNav::set_wp_destination(const Vector3f& destination, bool terrain_alt)
{
    // re-initialise if previous destination has been interrupted
    if (!is_active() || !_flags.reached_destination) {
        wp_and_spline_init(_wp_desired_speed_xy_cms);
    }

    _scurve_prev_leg.init();
    float origin_speed = 0.0f;

    // use previous destination as origin
    _origin = _destination;

    if (terrain_alt == _terrain_alt) {
        if (_this_leg_is_spline) {
            // if previous leg was a spline we can use current target velocity vector for origin velocity vector
            Vector3f curr_target_vel = _pos_control.get_desired_velocity();
            curr_target_vel.z -= _vel_terrain_offset;
            origin_speed = curr_target_vel.length();
        } else {
            // store previous leg
            _scurve_prev_leg = _scurve_this_leg;
        }
    } else {

        // get current alt above terrain
        float origin_terr_offset;
        if (!get_terrain_offset(origin_terr_offset)) {
            return false;
        }

        // convert origin to alt-above-terrain if necessary
        if (terrain_alt) {
            // new destination is alt-above-terrain, previous destination was alt-above-ekf-origin
            _origin.z -= origin_terr_offset;
            _pos_terrain_offset += origin_terr_offset;
        } else {
            // new destination is alt-above-ekf-origin, previous destination was alt-above-terrain
            _origin.z += origin_terr_offset;
            _pos_terrain_offset -= origin_terr_offset;
        }
    }

    // update destination
    _destination = destination;
    _terrain_alt = terrain_alt;

    if (_flags.fast_waypoint && !_this_leg_is_spline && !_next_leg_is_spline && !_scurve_next_leg.finished()) {
        _scurve_this_leg = _scurve_next_leg;
    } else {
        _scurve_this_leg.calculate_track(_origin, _destination,
                                         _pos_control.get_max_speed_xy(), _pos_control.get_max_speed_up(), _pos_control.get_max_speed_down(),
                                         _wp_accel_cmss, _wp_accel_z_cmss,
                                         _scurve_jerk_time, _scurve_jerk * 100.0f);
        if (!is_zero(origin_speed)) {
            // rebuild start of scurve if we have a non-zero origin speed
            _scurve_this_leg.set_origin_speed_max(origin_speed);
        }
    }

    _this_leg_is_spline = false;
    _scurve_next_leg.init();
    _flags.fast_waypoint = false;   // default waypoint back to slow
    _flags.reached_destination = false;
    _flags.wp_yaw_set = false;

    return true;
}

/// set next destination using position vector (distance from ekf origin in cm)
///     terrain_alt should be true if destination.z is a desired altitude above terrain
///     provide next_destination
bool AC_WPNav::set_wp_destination_next(const Vector3f& destination, bool terrain_alt)
{
    // do not add next point if alt types don't match
    if (terrain_alt != _terrain_alt) {
        return true;
    }

    _scurve_next_leg.calculate_track(_destination, destination,
                                     _pos_control.get_max_speed_xy(), _pos_control.get_max_speed_up(), _pos_control.get_max_speed_down(),
                                     _wp_accel_cmss, _wp_accel_z_cmss,
                                     _scurve_jerk_time, _scurve_jerk * 100.0f);
    if (_this_leg_is_spline) {
        const float this_leg_dest_speed_max = _spline_this_leg.get_destination_speed_max();
        const float next_leg_origin_speed_max = _scurve_next_leg.set_origin_speed_max(this_leg_dest_speed_max);
        _spline_this_leg.set_destination_speed_max(next_leg_origin_speed_max);
    }
    _next_leg_is_spline = false;

    // next destination provided so fast waypoint
    _flags.fast_waypoint = true;

    return true;
}

/// set waypoint destination using NED position vector from ekf origin in meters
bool AC_WPNav::set_wp_destination_NED(const Vector3f& destination_NED)
{
    // convert NED to NEU and do not use terrain following
    return set_wp_destination(Vector3f(destination_NED.x * 100.0f, destination_NED.y * 100.0f, -destination_NED.z * 100.0f), false);
}

/// set waypoint destination using NED position vector from ekf origin in meters
bool AC_WPNav::set_wp_destination_next_NED(const Vector3f& destination_NED)
{
    // convert NED to NEU and do not use terrain following
    return set_wp_destination_next(Vector3f(destination_NED.x * 100.0f, destination_NED.y * 100.0f, -destination_NED.z * 100.0f), false);
}

/// shifts the origin and destination horizontally to the current position
///     used to reset the track when taking off without horizontal position control
///     relies on set_wp_destination or set_wp_origin_and_destination having been called first
void AC_WPNav::shift_wp_origin_and_destination_to_current_pos_xy()
{
    // get current and target locations
    const Vector3f& curr_pos = _inav.get_position();

    // shift origin and destination horizontally
    _origin.x = curr_pos.x;
    _origin.y = curr_pos.y;
    _destination.x = curr_pos.x;
    _destination.y = curr_pos.y;

    // move pos controller target horizontally
    _pos_control.set_xy_target(curr_pos.x, curr_pos.y);
}

/// shifts the origin and destination horizontally to the achievable stopping point
///     used to reset the track when horizontal navigation is enabled after having been disabled (see Copter's wp_navalt_min)
///     relies on set_wp_destination or set_wp_origin_and_destination having been called first
void AC_WPNav::shift_wp_origin_and_destination_to_stopping_point_xy()
{
    // relax position control in xy axis
    // removing velocity error also impacts stopping point calculation
    _pos_control.relax_velocity_controller_xy();

    // get current and target locations
    Vector3f stopping_point;
    get_wp_stopping_point_xy(stopping_point);

    // shift origin and destination horizontally
    _origin.x = stopping_point.x;
    _origin.y = stopping_point.y;
    _destination.x = stopping_point.x;
    _destination.y = stopping_point.y;

    // move pos controller target horizontally
    _pos_control.set_xy_target(stopping_point.x, stopping_point.y);
}

/// get_wp_stopping_point_xy - returns vector to stopping point based on a horizontal position and velocity
void AC_WPNav::get_wp_stopping_point_xy(Vector3f& stopping_point) const
{
	_pos_control.get_stopping_point_xy(stopping_point);
}

/// get_wp_stopping_point - returns vector to stopping point based on 3D position and velocity
void AC_WPNav::get_wp_stopping_point(Vector3f& stopping_point) const
{
    _pos_control.get_stopping_point_xy(stopping_point);
    _pos_control.get_stopping_point_z(stopping_point);
}

/// advance_wp_target_along_track - move target location along track from origin to destination
bool AC_WPNav::advance_wp_target_along_track(float dt)
{
    // calculate terrain adjustments
    float terr_offset = 0.0f;
    if (_terrain_alt && !get_terrain_offset(terr_offset)) {
        return false;
    }

    // get current position and adjust altitude to origin and destination's frame (i.e. _frame)
    const Vector3f &curr_pos = _inav.get_position() - Vector3f{0, 0, terr_offset};

    // Use _track_scalar_dt to slow down S-Curve time to prevent target moving too far in front of aircraft
    Vector3f curr_target_vel = _pos_control.get_desired_velocity();
    curr_target_vel.z -= _vel_terrain_offset;

    float track_scaler_dt = 1.0f;
    // check target velocity is non-zero
    if (is_positive(curr_target_vel.length())) {
        Vector3f track_direction = curr_target_vel.normalized();
        const float track_error = _pos_control.get_pos_error().dot(track_direction);
        const float track_velocity = _inav.get_velocity().dot(track_direction);
        // set time scaler to be consistent with the achievable aircraft speed with a 5% buffer for short term variation.
        track_scaler_dt = constrain_float(0.05f + (track_velocity - _pos_control.get_pos_xy_p().kP() * track_error) / curr_target_vel.length(), 0.1f, 1.0f);
        // set time scaler to not exceed the maximum vertical velocity during terrain following.
        if (is_positive(curr_target_vel.z)) {
            track_scaler_dt = MIN(track_scaler_dt, fabsf(_wp_speed_up_cms / curr_target_vel.z));
        } else if (is_negative(curr_target_vel.z)) {
            track_scaler_dt = MIN(track_scaler_dt, fabsf(_wp_speed_down_cms / curr_target_vel.z));
        }
    }
    // change s-curve time speed with a time constant of maximum acceleration / maximum jerk
    float track_scaler_tc = 1.0f;
    if (!is_zero(_wp_jerk)) {
        track_scaler_tc = 0.01f * _wp_accel_cmss/_wp_jerk;
    }
    _track_scalar_dt += (track_scaler_dt - _track_scalar_dt) * (dt / track_scaler_tc);

    // target position, velocity and acceleration from straight line or spline calculators
    Vector3f target_pos, target_vel, target_accel;

    bool s_finished;
    if (!_this_leg_is_spline) {
        // update target position, velocity and acceleration
        target_pos = _origin;
        s_finished = _scurve_this_leg.advance_target_along_track(_scurve_prev_leg, _scurve_next_leg, _wp_radius_cm, _flags.fast_waypoint, _track_scalar_dt * dt, target_pos, target_vel, target_accel);
    } else {
        // splinetarget_vel
        target_vel = curr_target_vel;
        _spline_this_leg.advance_target_along_track(_track_scalar_dt * dt, target_pos, target_vel);
        s_finished = _spline_this_leg.reached_destination();
    }

    // input shape the terrain offset
    shape_pos_vel(terr_offset, 0.0f, _pos_terrain_offset, _vel_terrain_offset, _accel_terrain_offset, 0.0f, 0.0f, _pos_control.get_max_accel_z(), 0.05f, _track_scalar_dt * dt);
    update_pos_vel_accel(_pos_terrain_offset, _vel_terrain_offset, _accel_terrain_offset, _track_scalar_dt * dt);

    // convert final_target.z to altitude above the ekf origin
    target_pos.z += _pos_terrain_offset;
    target_vel.z += _vel_terrain_offset;
    target_accel.z += _accel_terrain_offset;

    // pass new target to the position controller
    _pos_control.set_pos_vel_accel_target(target_pos, target_vel, target_accel);

    // check if we've reached the waypoint
    if (!_flags.reached_destination) {
        if (s_finished) {
            // "fast" waypoints are complete once the intermediate point reaches the destination
            if (_flags.fast_waypoint) {
                _flags.reached_destination = true;
            } else {
                // regular waypoints also require the copter to be within the waypoint radius
                const Vector3f dist_to_dest = curr_pos - _destination;
                if (dist_to_dest.length_squared() <= sq(_wp_radius_cm)) {
                    _flags.reached_destination = true;
                }
            }
        }
    }

    // successfully advanced along track
    return true;
}

/// recalculate path with update speed and/or acceleration limits
void AC_WPNav::update_track_with_speed_accel_limits()
{
    // update this leg
    if (_this_leg_is_spline) {
        _spline_this_leg.set_speed_accel(_pos_control.get_max_speed_xy(), _pos_control.get_max_speed_up(), _pos_control.get_max_speed_down(),
                                         _wp_accel_cmss, _wp_accel_z_cmss);
    } else {
        _scurve_this_leg.set_speed_max(_pos_control.get_max_speed_xy(), _pos_control.get_max_speed_up(), _pos_control.get_max_speed_down());
    }

    // update next leg
    if (_next_leg_is_spline) {
        _spline_next_leg.set_speed_accel(_pos_control.get_max_speed_xy(), _pos_control.get_max_speed_up(), _pos_control.get_max_speed_down(),
                                         _wp_accel_cmss, _wp_accel_z_cmss);
    } else {
        _scurve_next_leg.set_speed_max(_pos_control.get_max_speed_xy(), _pos_control.get_max_speed_up(), _pos_control.get_max_speed_down());
    }
}

/// get_wp_distance_to_destination - get horizontal distance to destination in cm
float AC_WPNav::get_wp_distance_to_destination() const
{
    // get current location
    const Vector3f &curr = _inav.get_position();
    return norm(_destination.x-curr.x,_destination.y-curr.y);
}

/// get_wp_bearing_to_destination - get bearing to next waypoint in centi-degrees
int32_t AC_WPNav::get_wp_bearing_to_destination() const
{
    return get_bearing_cd(_inav.get_position(), _destination);
}

/// update_wpnav - run the wp controller - should be called at 100hz or higher
bool AC_WPNav::update_wpnav()
{
    bool ret = true;

    // get dt from pos controller
    float dt = _pos_control.get_dt();

    // allow the accel and speed values to be set without changing
    // out of auto mode. This makes it easier to tune auto flight
    _pos_control.set_max_accel_xy(_wp_accel_cmss);
    _pos_control.set_max_accel_z(_wp_accel_z_cmss);

    // advance the target if necessary
    if (!advance_wp_target_along_track(dt)) {
        // To-Do: handle inability to advance along track (probably because of missing terrain data)
        ret = false;
    }

    _pos_control.update_xy_controller();

    _wp_last_update = AP_HAL::millis();

    return ret;
}

// returns true if update_wpnav has been run very recently
bool AC_WPNav::is_active() const
{
    return (AP_HAL::millis() - _wp_last_update) < 200;
}

// get terrain's altitude (in cm above the ekf origin) at the current position (+ve means terrain below vehicle is above ekf origin's altitude)
bool AC_WPNav::get_terrain_offset(float& offset_cm)
{
    // calculate offset based on source (rangefinder or terrain database)
    switch (get_terrain_source()) {
    case AC_WPNav::TerrainSource::TERRAIN_UNAVAILABLE:
        return false;
    case AC_WPNav::TerrainSource::TERRAIN_FROM_RANGEFINDER:
        if (_rangefinder_healthy) {
            offset_cm = _inav.get_altitude() - _rangefinder_alt_cm;
            return true;
        }
        return false;
    case AC_WPNav::TerrainSource::TERRAIN_FROM_TERRAINDATABASE:
#if AP_TERRAIN_AVAILABLE
        float terr_alt = 0.0f;
        if (_terrain != nullptr && _terrain->height_above_terrain(terr_alt, true)) {
            offset_cm = _inav.get_altitude() - (terr_alt * 100.0f);
            return true;
        }
#endif
        return false;
    }

    // we should never get here
    return false;
}

///
/// spline methods
///

/// set_spline_destination waypoint using location class
///     returns false if conversion from location to vector from ekf origin cannot be calculated
///     next_destination should be the next segment's destination
///     next_is_spline should be true if path to next_destination should be a spline
bool AC_WPNav::set_spline_destination_loc(const Location& destination, const Location& next_destination, bool next_is_spline)
{
    // convert destination location to vector
    Vector3f dest_neu;
    bool dest_terr_alt;
    if (!get_vector_NEU(destination, dest_neu, dest_terr_alt)) {
        return false;
    }

    // convert next destination to vector
    Vector3f next_dest_neu;
    bool next_dest_terr_alt;
    if (!get_vector_NEU(next_destination, next_dest_neu, next_dest_terr_alt)) {
        return false;
    }

    // set target as vector from EKF origin
    return set_spline_destination(dest_neu, dest_terr_alt, next_dest_neu, next_dest_terr_alt, next_is_spline);
}

/// set next destination (e.g. the one after the current destination) as a spline segment specified as a location
///     returns false if conversion from location to vector from ekf origin cannot be calculated
///     next_next_destination should be the next segment's destination
bool AC_WPNav::set_spline_destination_next_loc(const Location& next_destination, const Location& next_next_destination, bool next_next_is_spline)
{
    // convert next_destination location to vector
    Vector3f next_dest_neu;
    bool next_dest_terr_alt;
    if (!get_vector_NEU(next_destination, next_dest_neu, next_dest_terr_alt)) {
        return false;
    }

    // convert next_next_destination to vector
    Vector3f next_next_dest_neu;
    bool next_next_dest_terr_alt;
    if (!get_vector_NEU(next_next_destination, next_next_dest_neu, next_next_dest_terr_alt)) {
        return false;
    }

    // set target as vector from EKF origin
    return set_spline_destination_next(next_dest_neu, next_dest_terr_alt, next_next_dest_neu, next_next_dest_terr_alt, next_next_is_spline);
}

/// set_spline_destination waypoint using position vector (distance from ekf origin in cm)
///     terrain_alt should be true if destination.z is a desired altitude above terrain (false if its desired altitudes above ekf origin)
///     next_destination should be set to the next segment's destination
///     next_terrain_alt should be true if next_destination.z is a desired altitude above terrain (false if its desired altitudes above ekf origin)
///     next_destination.z  must be in the same "frame" as destination.z (i.e. if destination is a alt-above-terrain, next_destination should be too)
bool AC_WPNav::set_spline_destination(const Vector3f& destination, bool terrain_alt, const Vector3f& next_destination, bool next_terrain_alt, bool next_is_spline)
{
    // re-initialise if previous destination has been interrupted
    if (!is_active() || !_flags.reached_destination) {
        wp_and_spline_init(_wp_desired_speed_xy_cms);
    }

    // update spline calculators speeds, accelerations and leash lengths
    _spline_this_leg.set_speed_accel(_pos_control.get_max_speed_xy(), _pos_control.get_max_speed_up(), _pos_control.get_max_speed_down(),
                                     _pos_control.get_max_accel_xy(), _pos_control.get_max_accel_z());

    // calculate origin and origin velocity vector
    Vector3f origin_vector;
    if (terrain_alt == _terrain_alt) {
        if (_flags.fast_waypoint) {
            // calculate origin vector
            if (_this_leg_is_spline) {
                // if previous leg was a spline we can use destination velocity vector for origin velocity vector
                origin_vector = _spline_this_leg.get_destination_vel();
            } else {
                // use direction of the previous straight line segment
                origin_vector = _destination - _origin;
            }
        }

        // use previous destination as origin
        _origin = _destination;
    } else {

        // use previous destination as origin
        _origin = _destination;

        // get current alt above terrain
        float origin_terr_offset;
        if (!get_terrain_offset(origin_terr_offset)) {
            return false;
        }

        // convert origin to alt-above-terrain if necessary
        if (terrain_alt) {
            // new destination is alt-above-terrain, previous destination was alt-above-ekf-origin
            _origin.z -= origin_terr_offset;
            _pos_terrain_offset += origin_terr_offset;
        } else {
            // new destination is alt-above-ekf-origin, previous destination was alt-above-terrain
            _origin.z += origin_terr_offset;
            _pos_terrain_offset -= origin_terr_offset;
        }
    }

    // store destination location
    _destination = destination;
    _terrain_alt = terrain_alt;

    // calculate destination velocity vector
    Vector3f destination_vector;
    if (terrain_alt == next_terrain_alt) {
        if (next_is_spline) {
            // leave this segment moving parallel to vector from origin to next destination
            destination_vector = next_destination - _origin;
        } else {
            // leave this segment moving parallel to next segment
            destination_vector = next_destination - _destination;
        }
    }
    _flags.fast_waypoint = !destination_vector.is_zero();

    // setup spline leg
    _spline_this_leg.set_origin_and_destination(_origin, _destination, origin_vector, destination_vector);
    _this_leg_is_spline = true;
    _flags.reached_destination = false;
    _flags.wp_yaw_set = false;

    return true;
}

/// set next destination (e.g. the one after the current destination) as an offset (in cm, NEU frame) from the EKF origin
///     next_terrain_alt should be true if next_destination.z is a desired altitude above terrain (false if its desired altitudes above ekf origin)
///     next_next_destination should be set to the next segment's destination
///     next_next_terrain_alt should be true if next_next_destination.z is a desired altitude above terrain (false if it is desired altitude above ekf origin)
///     next_next_destination.z  must be in the same "frame" as destination.z (i.e. if next_destination is a alt-above-terrain, next_next_destination should be too)
bool AC_WPNav::set_spline_destination_next(const Vector3f& next_destination, bool next_terrain_alt, const Vector3f& next_next_destination, bool next_next_terrain_alt, bool next_next_is_spline)
{
    // do not add next point if alt types don't match
    if (next_terrain_alt != _terrain_alt) {
        return true;
    }

    // calculate origin and origin velocity vector
    Vector3f origin_vector;
    if (_this_leg_is_spline) {
        // if previous leg was a spline we can use destination velocity vector for origin velocity vector
        origin_vector = _spline_this_leg.get_destination_vel();
    } else {
        // use the direction of the previous straight line segment
        origin_vector = _destination - _origin;
    }

    // calculate destination velocity vector
    Vector3f destination_vector;
    if (next_terrain_alt == next_next_terrain_alt) {
        if (next_next_is_spline) {
            // leave this segment moving parallel to vector from this leg's origin (i.e. prev leg's destination) to next next destination
            destination_vector = next_next_destination - _destination;
        } else {
            // leave this segment moving parallel to next segment
            destination_vector = next_next_destination - next_destination;
        }
    }

    // update spline calculators speeds and accelerations
    _spline_next_leg.set_speed_accel(_pos_control.get_max_speed_xy(), _pos_control.get_max_speed_up(), _pos_control.get_max_speed_down(),
                                     _pos_control.get_max_accel_xy(), _pos_control.get_max_accel_z());

    // setup next spline leg.  Note this could be made local
    _spline_next_leg.set_origin_and_destination(_destination, next_destination, origin_vector, destination_vector);
    _next_leg_is_spline = true;

    // next destination provided so fast waypoint
    _flags.fast_waypoint = true;

    // update this_leg's final velocity to match next spline leg
    if (!_this_leg_is_spline) {
        _scurve_this_leg.set_destination_speed_max(_spline_next_leg.get_origin_speed_max());
    } else {
        _spline_this_leg.set_destination_speed_max(_spline_next_leg.get_origin_speed_max());
    }

    return true;
}

// convert location to vector from ekf origin.  terrain_alt is set to true if resulting vector's z-axis should be treated as alt-above-terrain
//      returns false if conversion failed (likely because terrain data was not available)
bool AC_WPNav::get_vector_NEU(const Location &loc, Vector3f &vec, bool &terrain_alt)
{
    // convert location to NE vector2f
    Vector2f res_vec;
    if (!loc.get_vector_xy_from_origin_NE(res_vec)) {
        return false;
    }

    // convert altitude
    if (loc.get_alt_frame() == Location::AltFrame::ABOVE_TERRAIN) {
        int32_t terr_alt;
        if (!loc.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, terr_alt)) {
            return false;
        }
        vec.z = terr_alt;
        terrain_alt = true;
    } else {
        terrain_alt = false;
        int32_t temp_alt;
        if (!loc.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN, temp_alt)) {
            return false;
        }
        vec.z = temp_alt;
        terrain_alt = false;
    }

    // copy xy (we do this to ensure we do not adjust vector unless the overall conversion is successful
    vec.x = res_vec.x;
    vec.y = res_vec.y;

    return true;
}

// helper function to calculate scurve jerk and jerk_time values
// updates _scurve_jerk and _scurve_jerk_time
void AC_WPNav::calc_scurve_jerk_and_jerk_time()
{
    // calculate jerk
    _scurve_jerk = MIN(_attitude_control.get_ang_vel_roll_max_radss() * GRAVITY_MSS, _attitude_control.get_ang_vel_pitch_max_radss() * GRAVITY_MSS);
    if (is_zero(_scurve_jerk)) {
        _scurve_jerk = _wp_jerk;
    } else {
        _scurve_jerk = MIN(_scurve_jerk, _wp_jerk);
    }

    // calculate jerk time
    // jounce (the rate of change of jerk) uses the attitude control input time constant because multicopters
    // lean to accelerate meaning the change in angle is equivalent to the change in acceleration
    const float jounce = MIN(_attitude_control.get_accel_roll_max() * GRAVITY_MSS, _attitude_control.get_accel_pitch_max() * GRAVITY_MSS);
    if (is_positive(jounce)) {
        _scurve_jerk_time = MAX(_attitude_control.get_input_tc(), 0.5f * _scurve_jerk * M_PI / jounce);
    } else {
        _scurve_jerk_time = MAX(_attitude_control.get_input_tc(), 0.1f);
    }
    _scurve_jerk_time *= 2.0f;
}
