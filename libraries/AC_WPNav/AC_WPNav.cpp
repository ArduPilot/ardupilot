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
    AP_GROUPINFO("JERK",   11, AC_WPNav, _wp_jerk, 1),

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
    _flags.slowing_down = false;
    _flags.recalc_wp_leash = false;
    _flags.new_wp_destination = false;
    _flags.segment_type = SEGMENT_STRAIGHT;

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
///     updates target roll, pitch targets and I terms based on vehicle lean angles
///     should be called once before the waypoint controller is used but does not need to be called before subsequent updates to destination
void AC_WPNav::wp_and_spline_init()
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

    // initialise position controller speed and acceleration
    _pos_control.set_max_speed_xy(_wp_speed_cms);
    _pos_control.set_max_accel_xy(_wp_accel_cmss);
    _pos_control.set_max_speed_z(-_wp_speed_down_cms, _wp_speed_up_cms);
    _pos_control.set_max_accel_z(_wp_accel_z_cmss);
    _pos_control.calc_leash_length_xy();
    _pos_control.calc_leash_length_z();

    float jerk = MIN(_attitude_control.get_ang_vel_roll_max() * GRAVITY_MSS, _attitude_control.get_ang_vel_pitch_max() * GRAVITY_MSS);
    if (is_zero(jerk)) {
        jerk = _wp_jerk;
    } else {
        jerk = MIN(jerk, _wp_jerk);
    }
    float jounce = MIN(_attitude_control.get_accel_roll_max() * GRAVITY_MSS, _attitude_control.get_accel_pitch_max() * GRAVITY_MSS);

    // time constant may also be a useful parameter
    float tc;
    if (is_positive(jounce)) {
        tc = MAX(_attitude_control.get_input_tc(), 0.5 * jerk * M_PI / jounce);
    } else {
        tc = MAX(_attitude_control.get_input_tc(), 0.1f);
    }
//    float tc = MAX(0.25 * 2.0, 0.5 * jerk * M_PI / jounce);
    _scurve_last_leg = scurves(tc * 2.0, jerk * 100.0, _wp_accel_cmss, _wp_speed_cms);
    _scurve_this_leg = scurves(tc * 2.0, jerk * 100.0, _wp_accel_cmss, _wp_speed_cms);
    _scurve_next_leg = scurves(tc * 2.0, jerk * 100.0, _wp_accel_cmss, _wp_speed_cms);

    // initialise yaw heading to current heading target
    _flags.wp_yaw_set = false;

    _scurve_this_leg.Cal_Init(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    _scurve_last_leg.Cal_Init(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

/// set_speed_xy - allows main code to pass target horizontal velocity for wp navigation
void AC_WPNav::set_speed_xy(float speed_cms)
{
    // range check new target speed and update position controller
    if (speed_cms >= WPNAV_WP_SPEED_MIN) {
        _pos_control.set_max_speed_xy(speed_cms);
        // flag that wp leash must be recalculated
        _flags.recalc_wp_leash = true;
    }
}

/// set current target climb rate during wp navigation
void AC_WPNav::set_speed_up(float speed_up_cms)
{
    _pos_control.set_max_speed_z(_pos_control.get_max_speed_down(), speed_up_cms);
    // flag that wp leash must be recalculated
    _flags.recalc_wp_leash = true;
}

/// set current target descent rate during wp navigation
void AC_WPNav::set_speed_down(float speed_down_cms)
{
    _pos_control.set_max_speed_z(speed_down_cms, _pos_control.get_max_speed_up());
    // flag that wp leash must be recalculated
    _flags.recalc_wp_leash = true;
}

/// set_wp_destination waypoint using location class
///     returns false if conversion from location to vector from ekf origin cannot be calculated
bool AC_WPNav::set_wp_destination(const Location& destination)
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

/// set_wp_destination waypoint using location class
///     returns false if conversion from location to vector from ekf origin cannot be calculated
bool AC_WPNav::set_wp_destination_next(const Location& destination)
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

bool AC_WPNav::get_wp_destination(Location& destination) const
{
    Vector3f dest = get_wp_destination();
    if (!AP::ahrs().get_origin(destination)) {
        return false;
    }
    destination.offset(dest.x*0.01f, dest.y*0.01f);
    destination.alt += dest.z;
    return true;
}

/// set_wp_destination waypoint using position vector (distance from home in cm)
///     terrain_alt should be true if destination.z is a desired altitude above terrain
bool AC_WPNav::set_wp_destination(const Vector3f& destination, bool terrain_alt)
{
    Vector3f origin;

    // if waypoint controller is active use the existing destination as the origin
    if ((AP_HAL::millis() - _wp_last_update) < 1000) {
        origin = _destination;
    } else {
        // if waypoint controller is not active, set origin to reasonable stopping point (using curr pos and velocity)
        _pos_control.get_stopping_point_xy(origin);
        _pos_control.get_stopping_point_z(origin);
    }

    // convert origin to alt-above-terrain
    if (terrain_alt) {
        float origin_terr_offset;
        if (!get_terrain_offset(origin_terr_offset)) {
            return false;
        }
        origin.z -= origin_terr_offset;
    }

    // set origin and destination
    return set_wp_origin_and_destination(origin, destination, terrain_alt);
}

/// set_wp_destination waypoint using position vector (distance from home in cm)
///     terrain_alt should be true if destination.z is a desired altitude above terrain
bool AC_WPNav::set_wp_destination_next(const Vector3f& destination, bool terrain_alt)
{
    Vector3f pos_delta = destination - _destination;

    float track_length = pos_delta.length(); // get track length
    if (is_zero(track_length)) {
        // avoid possible divide by zero
        _pos_delta_unit_next.x = 0;
        _pos_delta_unit_next.y = 0;
        _pos_delta_unit_next.z = 0;
    }else{
        _pos_delta_unit_next = pos_delta/track_length;
    }
    _scurve_next_leg.Cal_Init(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    _scurve_next_leg.Cal_Pn(track_length);
    return true;
}

/// set waypoint destination using NED position vector from ekf origin in meters
bool AC_WPNav::set_wp_destination_NED(const Vector3f& destination_NED)
{
    // convert NED to NEU and do not use terrain following
    return set_wp_destination(Vector3f(destination_NED.x * 100.0f, destination_NED.y * 100.0f, -destination_NED.z * 100.0f), false);
}

/// set_origin_and_destination - set origin and destination waypoints using position vectors (distance from home in cm)
///     terrain_alt should be true if origin.z and destination.z are desired altitudes above terrain (false if these are alt-above-ekf-origin)
///     returns false on failure (likely caused by missing terrain data)
bool AC_WPNav::set_wp_origin_and_destination(const Vector3f& origin, const Vector3f& destination, bool terrain_alt)
{
    // store origin and destination locations
    _origin = origin;
    _destination = destination;
    _terrain_alt = terrain_alt;
    Vector3f pos_delta = _destination - _origin;

    _track_length = pos_delta.length(); // get track length
    _track_length_xy = safe_sqrt(sq(pos_delta.x)+sq(pos_delta.y));  // get horizontal track length (used to decide if we should update yaw)

    // calculate each axis' percentage of the total distance to the destination
    _pos_delta_unit_last = _pos_delta_unit;
    if (is_zero(_track_length)) {
        // avoid possible divide by zero
        _pos_delta_unit.x = 0;
        _pos_delta_unit.y = 0;
        _pos_delta_unit.z = 0;
    }else{
        _pos_delta_unit = pos_delta/_track_length;
    }

    // calculate leash lengths
    calculate_wp_leash_length();

    // get origin's alt-above-terrain
    float origin_terr_offset = 0.0f;
    if (terrain_alt) {
        if (!get_terrain_offset(origin_terr_offset)) {
            return false;
        }
    }

    // initialise intermediate point to the origin
    _pos_control.set_pos_target(origin + Vector3f(0,0,origin_terr_offset));
    _track_desired = 0;             // target is at beginning of track
    _flags.reached_destination = false;
    _flags.fast_waypoint = false;   // default waypoint back to slow
    _flags.slowing_down = false;    // target is not slowing down yet
    _flags.segment_type = SEGMENT_STRAIGHT;
    _flags.new_wp_destination = true;   // flag new waypoint so we can freeze the pos controller's feed forward and smooth the transition
    _flags.wp_yaw_set = false;

    // initialise the limited speed to current speed along the track
    const Vector3f &curr_vel = _inav.get_velocity();
    // get speed along track (note: we convert vertical speed into horizontal speed equivalent)
    float speed_along_track = curr_vel.x * _pos_delta_unit.x + curr_vel.y * _pos_delta_unit.y + curr_vel.z * _pos_delta_unit.z;
    _limited_speed_xy_cms = constrain_float(speed_along_track, 0, _pos_control.get_max_speed_xy());
    _scurve_last_leg = _scurve_this_leg;
    _scurve_this_leg.Cal_Init(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    _scurve_this_leg.Cal_Pn(_track_length);
    // advance spline time to next step

    return true;
}

/// shift_wp_origin_to_current_pos - shifts the origin and destination so the origin starts at the current position
///     used to reset the position just before takeoff
///     relies on set_wp_destination or set_wp_origin_and_destination having been called first
void AC_WPNav::shift_wp_origin_to_current_pos()
{
    // return immediately if vehicle is not at the origin
    if (_track_desired > 0.0f) {
        return;
    }

    // get current and target locations
    const Vector3f &curr_pos = _inav.get_position();
    const Vector3f pos_target = _pos_control.get_pos_target();

    // calculate difference between current position and target
    Vector3f pos_diff = curr_pos - pos_target;

    // shift origin and destination
    _origin += pos_diff;
    _destination += pos_diff;

    // move pos controller target and disable feed forward
    _pos_control.set_pos_target(curr_pos);
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
    float track_covered;        // distance (in cm) along the track that the vehicle has traveled.  Measured by drawing a perpendicular line from the track to the vehicle.
    Vector3f track_error;       // distance error (in cm) from the track_covered position (i.e. closest point on the line to the vehicle) and the vehicle

    // get current location
    const Vector3f &curr_pos = _inav.get_position();
    const Vector3f &curr_vel = _inav.get_velocity();
    const Vector3f &desired_vel = _pos_control.get_desired_velocity();
    float track_desired_vel = desired_vel.length();

    float nav_tc = 1.0f;
    if (is_positive(track_desired_vel)) {
        float track_vel = (curr_vel.x * desired_vel.x + curr_vel.y * desired_vel.y + curr_vel.z * desired_vel.z) / track_desired_vel;
        _track_scaler_dt += (track_vel/(_track_scaler_dt*track_desired_vel)-_track_scaler_dt) * (dt / (dt + nav_tc));
        _track_scaler_dt = constrain_float(_track_scaler_dt, 0.1f, 1.0f);
    } else {
        _track_scaler_dt = 1.0f;
    }


    // calculate terrain adjustments
    float terr_offset = 0.0f;
    if (_terrain_alt && !get_terrain_offset(terr_offset)) {
        return false;
    }

    // calculate 3d vector from segment's origin
    Vector3f curr_delta = (curr_pos - Vector3f(0,0,terr_offset)) - _origin;

    // calculate how far along the track we are
    track_covered = curr_delta.x * _pos_delta_unit.x + curr_delta.y * _pos_delta_unit.y + curr_delta.z * _pos_delta_unit.z;

    // calculate the point closest to the vehicle on the segment from origin to destination
    Vector3f track_covered_pos = _pos_delta_unit * track_covered;

    // calculate the distance vector from the vehicle to the closest point on the segment from origin to destination
    track_error = curr_delta - track_covered_pos;

    // calculate the horizontal error
    _track_error_xy = norm(track_error.x, track_error.y);

    // advance spline time to next step
    float scurve_P1, scurve_V1, scurve_A1, scurve_J1;
    _scurve_this_leg.advance_time(_track_scaler_dt * dt);
    bool s_finish = _scurve_this_leg.runme(scurve_J1, scurve_A1, scurve_V1, scurve_P1);
    Vector3f target_accel = _pos_delta_unit*scurve_A1*_track_scaler_dt;
    Vector3f target_vel = _pos_delta_unit*scurve_V1 * MIN(_track_scaler_dt * 1.1f, 1.0f);
    Vector3f target_pos = _origin + _pos_delta_unit*scurve_P1;

    float scurve_P2, scurve_V2, scurve_A2, scurve_J2;
    _scurve_last_leg.advance_time(_track_scaler_dt * dt);
    _scurve_last_leg.runme(scurve_J2, scurve_A2, scurve_V2, scurve_P2);
    target_accel += _pos_delta_unit_last*scurve_A2*_track_scaler_dt;
    target_vel += _pos_delta_unit_last*scurve_V2 * MIN(_track_scaler_dt * 1.1f, 1.0f);
    target_pos += _pos_delta_unit_last*(scurve_P2 - _scurve_last_leg.pos_end());

    // convert target_pos.z to altitude above the ekf origin
    target_pos.z += terr_offset;

    _pos_control.set_pos_vel_accel(target_pos, target_vel, target_accel);

    if (_flags.fast_waypoint && _scurve_this_leg.breaking()) {
        float time_to_destination = _scurve_this_leg.time_to_end();
        _scurve_this_leg.runme(_scurve_this_leg.time_now() + time_to_destination/2.0, scurve_J1, scurve_A1, scurve_V1, scurve_P1);
        _scurve_next_leg.runme(time_to_destination/2.0, scurve_J2, scurve_A2, scurve_V2, scurve_P2);
        Vector3f turn_pos = _pos_delta_unit * (scurve_P1 - _scurve_this_leg.pos_end()) + _pos_delta_unit_next * scurve_P2;
        Vector3f turn_vel = _pos_delta_unit * scurve_V1 + _pos_delta_unit_next * scurve_V2;
        Vector3f turn_accel = _pos_delta_unit * scurve_A1 + _pos_delta_unit_next * scurve_A2;
//      s_finish = s_finish || ((_scurve_this_leg.time_to_end() < _scurve_next_leg.time_end()/2.0) && (turn_pos.length() < _wp_radius_cm) && (Vector2f(turn_vel.x, turn_vel.y).length() < _wp_speed_cms));
        s_finish = s_finish || ((_scurve_this_leg.time_to_end() < _scurve_next_leg.time_end()/2.0) && (turn_pos.length() < _wp_radius_cm) && (Vector2f(turn_vel.x, turn_vel.y).length() < _wp_speed_cms) && (Vector2f(turn_accel.x, turn_accel.y).length() < 2*_wp_accel_cmss));
    }

    // check if we've reached the waypoint
    if( !_flags.reached_destination ) {
        if( s_finish ) {
            // "fast" waypoints are complete once the intermediate point reaches the destination
            if (_flags.fast_waypoint) {
                _flags.reached_destination = true;
            }else{
                // regular waypoints also require the copter to be within the waypoint radius
                Vector3f dist_to_dest = (curr_pos - Vector3f(0,0,terr_offset)) - _destination;
                if( dist_to_dest.length() <= _wp_radius_cm ) {
                    _flags.reached_destination = true;
                }
            }
        }
    }

    float turn_rate, accel_forward;
    Vector3f accel_turn;
    if (is_positive(track_desired_vel) && is_positive(target_vel.length())) {
        accel_forward = (target_accel.x * target_vel.x + target_accel.y * target_vel.y + target_accel.z * target_vel.z)/target_vel.length();
        accel_turn = target_accel - target_vel * accel_forward / target_vel.length();
        turn_rate = accel_turn.length() / (_track_scaler_dt*target_vel.length());
        if(accel_turn.y * target_vel.x - accel_turn.x * target_vel.y < 0.0 ) {
            turn_rate *= -1.0f;
        }
    } else {
        turn_rate = 0.0f;
    }

    // update the target yaw if origin and destination are at least 2m apart horizontally
    float heading_vel = 0.0f;
    if (_track_length_xy >= WPNAV_YAW_DIST_MIN) {
        heading_vel = get_bearing_cd(Vector3f(), target_vel);
        // todo: add feed forward yaw for coordinated turns
        set_yaw_cd(heading_vel);
        set_yaw_cds(turn_rate*degrees(100));
    }

        AP::logger().Write("LEN2", "TimeUS,tvx,tvy,tax,tay,ltx,lty,tr,ta,ts", "Qfffffffff",
                                                      AP_HAL::micros64(),
                                                      (double)target_vel.x,
                                                      (double)target_vel.y,
                                                      (double)target_accel.x,
                                                      (double)target_accel.y,
                                                      (double)accel_turn.x,
                                                      (double)accel_turn.y,
                                                      (double)degrees(turn_rate),
                                                      (double)heading_vel/100.0f,
                                                      (double)_track_scaler_dt);

    // successfully advanced along track
    return true;
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

    // freeze feedforwards during known discontinuities
    if (_flags.new_wp_destination) {
        _flags.new_wp_destination = false;
    }

    _pos_control.update_xy_controller();
    check_wp_leash_length();

    _wp_last_update = AP_HAL::millis();

    return ret;
}

// check_wp_leash_length - check if waypoint leash lengths need to be recalculated
//  should be called after _pos_control.update_xy_controller which may have changed the position controller leash lengths
void AC_WPNav::check_wp_leash_length()
{
    // exit immediately if recalc is not required
    if (_flags.recalc_wp_leash) {
        calculate_wp_leash_length();
    }
}

/// calculate_wp_leash_length - calculates horizontal and vertical leash lengths for waypoint controller
void AC_WPNav::calculate_wp_leash_length()
{
    // length of the unit direction vector in the horizontal
    float pos_delta_unit_xy = norm(_pos_delta_unit.x, _pos_delta_unit.y);
    float pos_delta_unit_z = fabsf(_pos_delta_unit.z);

    float speed_z;
    float leash_z;
    if (_pos_delta_unit.z >= 0.0f) {
        speed_z = _pos_control.get_max_speed_up();
        leash_z = _pos_control.get_leash_up_z();
    }else{
        speed_z = fabsf(_pos_control.get_max_speed_down());
        leash_z = _pos_control.get_leash_down_z();
    }

    // calculate the maximum acceleration, maximum velocity, and leash length in the direction of travel
    if(is_zero(pos_delta_unit_z) && is_zero(pos_delta_unit_xy)){
        _track_accel = 0;
        _track_speed = 0;
        _track_leash_length = WPNAV_LEASH_LENGTH_MIN;
    }else if(is_zero(_pos_delta_unit.z)){
        _track_accel = _wp_accel_cmss/pos_delta_unit_xy;
        _track_speed = _pos_control.get_max_speed_xy() / pos_delta_unit_xy;
        _track_leash_length = _pos_control.get_leash_xy()/pos_delta_unit_xy;
    }else if(is_zero(pos_delta_unit_xy)){
        _track_accel = _wp_accel_z_cmss/pos_delta_unit_z;
        _track_speed = speed_z/pos_delta_unit_z;
        _track_leash_length = leash_z/pos_delta_unit_z;
    }else{
        _track_accel = MIN(_wp_accel_z_cmss/pos_delta_unit_z, _wp_accel_cmss/pos_delta_unit_xy);
        _track_speed = MIN(speed_z/pos_delta_unit_z, _pos_control.get_max_speed_xy() / pos_delta_unit_xy);
        _track_leash_length = MIN(leash_z/pos_delta_unit_z, _pos_control.get_leash_xy()/pos_delta_unit_xy);
    }

    // calculate slow down distance (the distance from the destination when the target point should begin to slow down)
    calc_slow_down_distance(_track_speed, _track_accel);

    // set recalc leash flag to false
    _flags.recalc_wp_leash = false;
}

// returns target yaw in centi-degrees (used for wp and spline navigation)
float AC_WPNav::get_yaw() const
{
    if (_flags.wp_yaw_set) {
        return _yaw;
    } else {
        // if yaw has not been set return attitude controller's current target
        return _attitude_control.get_att_target_euler_cd().z;
    }
}

// returns target yaw in centi-degrees (used for wp and spline navigation)
float AC_WPNav::get_yaw_rate() const
{
    if (_flags.wp_yaw_set) {
        return _yaw_rate;
    } else {
        // if yaw has not been set return attitude controller's current target
        return 0.0f;
    }
}

// set heading used for spline and waypoint navigation
void AC_WPNav::set_yaw_cd(float heading_cd)
{
    _yaw = heading_cd;
    _flags.wp_yaw_set = true;
}

// set heading used for spline and waypoint navigation
void AC_WPNav::set_yaw_cds(float heading_cds)
{
    _yaw_rate = heading_cds;
}

///
/// spline methods
///

/// set_spline_destination waypoint using location class
///     returns false if conversion from location to vector from ekf origin cannot be calculated
///     stopped_at_start should be set to true if vehicle is stopped at the origin
///     seg_end_type should be set to stopped, straight or spline depending upon the next segment's type
///     next_destination should be set to the next segment's destination if the seg_end_type is SEGMENT_END_STRAIGHT or SEGMENT_END_SPLINE
bool AC_WPNav::set_spline_destination(const Location& destination, bool stopped_at_start, spline_segment_end_type seg_end_type, Location next_destination)
{
    // convert destination location to vector
    Vector3f dest_neu;
    bool dest_terr_alt;
    if (!get_vector_NEU(destination, dest_neu, dest_terr_alt)) {
        return false;
    }

    Vector3f next_dest_neu; // left uninitialised for valgrind
    if (seg_end_type == SEGMENT_END_STRAIGHT ||
        seg_end_type == SEGMENT_END_SPLINE) {
        // make altitude frames consistent
        if (!next_destination.change_alt_frame(destination.get_alt_frame())) {
            return false;
        }

        // convert next destination to vector
        bool next_dest_terr_alt;
        if (!get_vector_NEU(next_destination, next_dest_neu, next_dest_terr_alt)) {
            return false;
        }
    }

    // set target as vector from EKF origin
    return set_spline_destination(dest_neu, dest_terr_alt, stopped_at_start, seg_end_type, next_dest_neu);
}

/// set_spline_destination waypoint using position vector (distance from home in cm)
///     returns false if conversion from location to vector from ekf origin cannot be calculated
///     terrain_alt should be true if destination.z is a desired altitudes above terrain (false if its desired altitudes above ekf origin)
///     stopped_at_start should be set to true if vehicle is stopped at the origin
///     seg_end_type should be set to stopped, straight or spline depending upon the next segment's type
///     next_destination should be set to the next segment's destination if the seg_end_type is SEGMENT_END_STRAIGHT or SEGMENT_END_SPLINE
bool AC_WPNav::set_spline_destination(const Vector3f& destination, bool terrain_alt, bool stopped_at_start, spline_segment_end_type seg_end_type, const Vector3f& next_destination)
{
    Vector3f origin;

    // if waypoint controller is active and copter has reached the previous waypoint use current pos target as the origin
    if ((AP_HAL::millis() - _wp_last_update) < 1000) {
        origin = _pos_control.get_pos_target();
    }else{
        // otherwise calculate origin from the current position and velocity
        _pos_control.get_stopping_point_xy(origin);
        _pos_control.get_stopping_point_z(origin);
    }

    // convert origin to alt-above-terrain
    if (terrain_alt) {
        float terr_offset;
        if (!get_terrain_offset(terr_offset)) {
            return false;
        }
        origin.z -= terr_offset;
    }

    // set origin and destination
    return set_spline_origin_and_destination(origin, destination, terrain_alt, stopped_at_start, seg_end_type, next_destination);
}

/// set_spline_origin_and_destination - set origin and destination waypoints using position vectors (distance from home in cm)
///     terrain_alt should be true if origin.z and destination.z are desired altitudes above terrain (false if desired altitudes above ekf origin)
///     seg_type should be calculated by calling function based on the mission
bool AC_WPNav::set_spline_origin_and_destination(const Vector3f& origin, const Vector3f& destination, bool terrain_alt, bool stopped_at_start, spline_segment_end_type seg_end_type, const Vector3f& next_destination)
{
    // mission is "active" if wpnav has been called recently and vehicle reached the previous waypoint
    bool prev_segment_exists = (_flags.reached_destination && ((AP_HAL::millis() - _wp_last_update) < 1000));

    // get dt from pos controller
    float dt = _pos_control.get_dt();

    // check _wp_accel_cmss is reasonable to avoid divide by zero
    if (_wp_accel_cmss <= 0) {
        _wp_accel_cmss.set_and_save(WPNAV_ACCELERATION);
    }

    // segment start types
    // stop - vehicle is not moving at origin
    // straight-fast - vehicle is moving, previous segment is straight.  vehicle will fly straight through the waypoint before beginning it's spline path to the next wp
    //     _flag.segment_type holds whether prev segment is straight vs spline but we don't know if it has a delay
    // spline-fast - vehicle is moving, previous segment is splined, vehicle will fly through waypoint but previous segment should have it flying in the correct direction (i.e. exactly parallel to position difference vector from previous segment's origin to this segment's destination)

    // calculate spline velocity at origin
    if (stopped_at_start || !prev_segment_exists) {
    	// if vehicle is stopped at the origin, set origin velocity to 0.02 * distance vector from origin to destination
    	_spline_origin_vel = (destination - origin) * dt;
    	_spline_time = 0.0f;
    	_spline_vel_scaler = 0.0f;
    }else{
    	// look at previous segment to determine velocity at origin
        if (_flags.segment_type == SEGMENT_STRAIGHT) {
            // previous segment is straight, vehicle is moving so vehicle should fly straight through the origin
            // before beginning it's spline path to the next waypoint. Note: we are using the previous segment's origin and destination
            _spline_origin_vel = (_destination - _origin);
            _spline_time = 0.0f;	// To-Do: this should be set based on how much overrun there was from straight segment?
            _spline_vel_scaler = _pos_control.get_vel_target().length();    // start velocity target from current target velocity
        }else{
            // previous segment is splined, vehicle will fly through origin
            // we can use the previous segment's destination velocity as this segment's origin velocity
            // Note: previous segment will leave destination velocity parallel to position difference vector
            //       from previous segment's origin to this segment's destination)
            _spline_origin_vel = _spline_destination_vel;
            if (_spline_time > 1.0f && _spline_time < 1.1f) {    // To-Do: remove hard coded 1.1f
                _spline_time -= 1.0f;
            }else{
                _spline_time = 0.0f;
            }
            // Note: we leave _spline_vel_scaler as it was from end of previous segment
        }
    }

    // calculate spline velocity at destination
    switch (seg_end_type) {

    case SEGMENT_END_STOP:
        // if vehicle stops at the destination set destination velocity to 0.02 * distance vector from origin to destination
        _spline_destination_vel = (destination - origin) * dt;
        _flags.fast_waypoint = false;
        break;

    case SEGMENT_END_STRAIGHT:
        // if next segment is straight, vehicle's final velocity should face along the next segment's position
        _spline_destination_vel = (next_destination - destination);
        _flags.fast_waypoint = true;
        break;

    case SEGMENT_END_SPLINE:
        // if next segment is splined, vehicle's final velocity should face parallel to the line from the origin to the next destination
        _spline_destination_vel = (next_destination - origin);
        _flags.fast_waypoint = true;
        break;
    }

    // code below ensures we don't get too much overshoot when the next segment is short
    float vel_len = _spline_origin_vel.length() + _spline_destination_vel.length();
    float pos_len = (destination - origin).length() * 4.0f;
    if (vel_len > pos_len) {
        // if total start+stop velocity is more than twice position difference
        // use a scaled down start and stop velocityscale the  start and stop velocities down
        float vel_scaling = pos_len / vel_len;
        // update spline calculator
        update_spline_solution(origin, destination, _spline_origin_vel * vel_scaling, _spline_destination_vel * vel_scaling);
    }else{
        // update spline calculator
        update_spline_solution(origin, destination, _spline_origin_vel, _spline_destination_vel);
    }

    // store origin and destination locations
    _origin = origin;
    _destination = destination;
    _terrain_alt = terrain_alt;

    // calculate slow down distance
    calc_slow_down_distance(_pos_control.get_max_speed_xy(), _wp_accel_cmss);

    // get alt-above-terrain
    float terr_offset = 0.0f;
    if (terrain_alt) {
        if (!get_terrain_offset(terr_offset)) {
            return false;
        }
    }

    // initialise intermediate point to the origin
    _pos_control.set_pos_target(origin + Vector3f(0,0,terr_offset));
    _flags.reached_destination = false;
    _flags.segment_type = SEGMENT_SPLINE;
    _flags.new_wp_destination = true;   // flag new waypoint so we can freeze the pos controller's feed forward and smooth the transition
    _flags.wp_yaw_set = false;

    // initialise yaw related variables
    _track_length_xy = safe_sqrt(sq(_destination.x - _origin.x)+sq(_destination.y - _origin.y));  // horizontal track length (used to decide if we should update yaw)

    return true;
}

/// update_spline - update spline controller
bool AC_WPNav::update_spline()
{
    // exit immediately if this is not a spline segment
    if (_flags.segment_type != SEGMENT_SPLINE) {
        return false;
    }

    bool ret = true;

    // get dt from pos controller
    float dt = _pos_control.get_dt();

    // advance the target if necessary
    if (!advance_spline_target_along_track(dt)) {
        // To-Do: handle failure to advance along track (due to missing terrain data)
        ret = false;
    }

    // freeze feedforwards during known discontinuities
    if (_flags.new_wp_destination) {
        _flags.new_wp_destination = false;
    }

    // run horizontal position controller
    _pos_control.update_xy_controller();

    _wp_last_update = AP_HAL::millis();

    return ret;
}

/// update_spline_solution - recalculates hermite_spline_solution grid
///		relies on _spline_origin_vel, _spline_destination_vel and _origin and _destination
void AC_WPNav::update_spline_solution(const Vector3f& origin, const Vector3f& dest, const Vector3f& origin_vel, const Vector3f& dest_vel)
{
    _hermite_spline_solution[0] = origin;
    _hermite_spline_solution[1] = origin_vel;
    _hermite_spline_solution[2] = -origin*3.0f -origin_vel*2.0f + dest*3.0f - dest_vel;
    _hermite_spline_solution[3] = origin*2.0f + origin_vel -dest*2.0f + dest_vel;
 }

/// advance_spline_target_along_track - move target location along track from origin to destination
bool AC_WPNav::advance_spline_target_along_track(float dt)
{
    if (!_flags.reached_destination) {
        Vector3f target_pos, target_vel;

        // update target position and velocity from spline calculator
        calc_spline_pos_vel(_spline_time, target_pos, target_vel);

        // if target velocity is zero the origin and destination must be the same
        // so flag reached destination (and protect against divide by zero)
        float target_vel_length = target_vel.length();
        if (is_zero(target_vel_length)) {
            _flags.reached_destination = true;
            return true;
        }

        _pos_delta_unit = target_vel / target_vel_length;
        calculate_wp_leash_length();

        // get current location
        const Vector3f &curr_pos = _inav.get_position();

        // get terrain altitude offset for origin and current position (i.e. change in terrain altitude from a position vs ekf origin)
        float terr_offset = 0.0f;
        if (_terrain_alt && !get_terrain_offset(terr_offset)) {
            return false;
        }

        // calculate position error
        Vector3f track_error = curr_pos - target_pos;
        track_error.z -= terr_offset;

        // calculate the horizontal error
        _track_error_xy = norm(track_error.x, track_error.y);

        // calculate the vertical error
        float track_error_z = fabsf(track_error.z);

        // get position control leash lengths
        float leash_xy = _pos_control.get_leash_xy();
        float leash_z;
        if (track_error.z >= 0) {
            leash_z = _pos_control.get_leash_up_z();
        }else{
            leash_z = _pos_control.get_leash_down_z();
        }

        // calculate how far along the track we could move the intermediate target before reaching the end of the leash
        float track_leash_slack = MIN(_track_leash_length*(leash_z-track_error_z)/leash_z, _track_leash_length*(leash_xy-_track_error_xy)/leash_xy);
        if (track_leash_slack < 0.0f) {
            track_leash_slack = 0.0f;
        }

        // update velocity
        float spline_dist_to_wp = (_destination - target_pos).length();
        float vel_limit = _pos_control.get_max_speed_xy();
        if (!is_zero(dt)) {
            vel_limit = MIN(vel_limit, track_leash_slack/dt);
        }

        // if within the stopping distance from destination, set target velocity to sqrt of distance * 2 * acceleration
        if (!_flags.fast_waypoint && spline_dist_to_wp < _slow_down_dist) {
            _spline_vel_scaler = safe_sqrt(spline_dist_to_wp * 2.0f * _wp_accel_cmss);
        }else if(_spline_vel_scaler < vel_limit) {
            // increase velocity using acceleration
            _spline_vel_scaler += _wp_accel_cmss * dt;
        }

        // constrain target velocity
        _spline_vel_scaler = constrain_float(_spline_vel_scaler, 0.0f, vel_limit);

        // scale the spline_time by the velocity we've calculated vs the velocity that came out of the spline calculator
        _spline_time_scale = _spline_vel_scaler / target_vel_length;

        // update target position
        target_pos.z += terr_offset;
        _pos_control.set_pos_target(target_pos);

        // update the target yaw if origin and destination are at least 2m apart horizontally
        if (_track_length_xy >= WPNAV_YAW_DIST_MIN) {
            if (_pos_control.get_leash_xy() < WPNAV_YAW_DIST_MIN) {
                // if the leash is very short (i.e. flying at low speed) use the target point's velocity along the track
                if (!is_zero(target_vel.x) && !is_zero(target_vel.y)) {
                    set_yaw_cd(RadiansToCentiDegrees(atan2f(target_vel.y,target_vel.x)));
                }
            } else {
                // point vehicle along the leash (i.e. point vehicle towards target point on the segment from origin to destination)
                float track_error_xy_length = safe_sqrt(sq(track_error.x)+sq(track_error.y));
                if (track_error_xy_length > MIN(WPNAV_YAW_DIST_MIN, _pos_control.get_leash_xy()*WPNAV_YAW_LEASH_PCT_MIN)) {
                    // To-Do: why is track_error sign reversed?
                    set_yaw_cd(RadiansToCentiDegrees(atan2f(-track_error.y,-track_error.x)));
                }
            }
        }

        // advance spline time to next step
        _spline_time += _spline_time_scale*dt;

        // we will reach the next waypoint in the next step so set reached_destination flag
        // To-Do: is this one step too early?
        if (_spline_time >= 1.0f) {
            _flags.reached_destination = true;
        }
    }
    return true;
}

// calc_spline_pos_vel_accel - calculates target position, velocity and acceleration for the given "spline_time"
/// 	relies on update_spline_solution being called when the segment's origin and destination were set
void AC_WPNav::calc_spline_pos_vel(float spline_time, Vector3f& position, Vector3f& velocity)
{
    float spline_time_sqrd = spline_time * spline_time;
    float spline_time_cubed = spline_time_sqrd * spline_time;

    position = _hermite_spline_solution[0] + \
               _hermite_spline_solution[1] * spline_time + \
               _hermite_spline_solution[2] * spline_time_sqrd + \
               _hermite_spline_solution[3] * spline_time_cubed;

    velocity = _hermite_spline_solution[1] + \
               _hermite_spline_solution[2] * 2.0f * spline_time + \
               _hermite_spline_solution[3] * 3.0f * spline_time_sqrd;
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

    // we should never get here but just in case
    return false;
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

///
/// shared methods
///

/// calc_slow_down_distance - calculates distance before waypoint that target point should begin to slow-down assuming it is travelling at full speed
void AC_WPNav::calc_slow_down_distance(float speed_cms, float accel_cmss)
{
	// protect against divide by zero
	if (accel_cmss <= 0.0f) {
		_slow_down_dist = 0.0f;
		return;
	}
    // To-Do: should we use a combination of horizontal and vertical speeds?
    // To-Do: update this automatically when speed or acceleration is changed
    _slow_down_dist = speed_cms * speed_cms / (4.0f*accel_cmss);
}

/// get_slow_down_speed - returns target speed of target point based on distance from the destination (in cm)
float AC_WPNav::get_slow_down_speed(float dist_from_dest_cm, float accel_cmss)
{
    // return immediately if distance is zero (or less)
    if (dist_from_dest_cm <= 0) {
        return WPNAV_WP_TRACK_SPEED_MIN;
    }

    // calculate desired speed near destination
    float target_speed = safe_sqrt(dist_from_dest_cm * 4.0f * accel_cmss);

    // ensure desired speed never becomes too low
    if (target_speed < WPNAV_WP_TRACK_SPEED_MIN) {
        return WPNAV_WP_TRACK_SPEED_MIN;
    } else {
        return target_speed;
    }
}
