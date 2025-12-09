#include <AP_HAL/AP_HAL.h>
#include "AC_WPNav.h"

extern const AP_HAL::HAL& hal;

// Default waypoint navigation constraints
#define WPNAV_WP_SPEED_CMS             1000.0f     // default horizontal speed between waypoints (cm/s)
#define WPNAV_WP_SPEED_MIN_MS             0.01f    // minimum horizontal speed allowed (m/s)
#define WPNAV_WP_RADIUS_CM              200.0f     // default radius within which a waypoint is considered reached (cm)
#define WPNAV_WP_RADIUS_MIN_CM            5.0f     // minimum allowable waypoint radius (cm)
#define WPNAV_WP_SPEED_UP_CMS           250.0f     // default maximum climb speed (cm/s)
#define WPNAV_WP_SPEED_DOWN_CMS         150.0f     // default maximum descent speed (cm/s)
#define WPNAV_WP_ACCEL_Z_DEFAULT_CMSS   100.0f     // default vertical acceleration limit (cm/s²)

const AP_Param::GroupInfo AC_WPNav::var_info[] = {
    // index 0 was used for the old orientation matrix

    // @Param: SPEED
    // @DisplayName: Waypoint Horizontal Speed Target
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain horizontally during a WP mission
    // @Units: cm/s
    // @Range: 10 2000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("SPEED",       0, AC_WPNav, _wp_speed_cms, WPNAV_WP_SPEED_CMS),

    // @Param: RADIUS
    // @DisplayName: Waypoint Radius
    // @Description: Defines the distance from a waypoint, that when crossed indicates the wp has been hit.
    // @Units: cm
    // @Range: 5 1000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RADIUS",      1, AC_WPNav, _wp_radius_cm, WPNAV_WP_RADIUS_CM),

    // @Param: SPEED_UP
    // @DisplayName: Waypoint Climb Speed Target
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain while climbing during a WP mission
    // @Units: cm/s
    // @Range: 10 1000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("SPEED_UP",    2, AC_WPNav, _wp_speed_up_cms, WPNAV_WP_SPEED_UP_CMS),

    // @Param: SPEED_DN
    // @DisplayName: Waypoint Descent Speed Target
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain while descending during a WP mission
    // @Units: cm/s
    // @Range: 10 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("SPEED_DN",    3, AC_WPNav, _wp_speed_down_cms, WPNAV_WP_SPEED_DOWN_CMS),

    // @Param: ACCEL
    // @DisplayName: Waypoint Acceleration 
    // @Description: Defines the horizontal acceleration in cm/s/s used during missions
    // @Units: cm/s/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("ACCEL",       5, AC_WPNav, _wp_accel_cmss, WPNAV_ACCELERATION_MS * 100.0),

    // @Param: ACCEL_Z
    // @DisplayName: Waypoint Vertical Acceleration
    // @Description: Defines the vertical acceleration in cm/s/s used during missions
    // @Units: cm/s/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("ACCEL_Z",     6, AC_WPNav, _wp_accel_z_cmss, WPNAV_WP_ACCEL_Z_DEFAULT_CMSS),

    // @Param: RFND_USE
    // @DisplayName: Waypoint missions use rangefinder for terrain following
    // @Description: This controls if waypoint missions use rangefinder for terrain following
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO("RFND_USE",   10, AC_WPNav, _rangefinder_use, 1),

    // @Param: JERK
    // @DisplayName: Waypoint Jerk
    // @Description: Defines the horizontal jerk in m/s/s used during missions
    // @Units: m/s/s/s
    // @Range: 1 20
    // @User: Standard
    AP_GROUPINFO("JERK",   11, AC_WPNav, _wp_jerk_msss, 1.0f),

    // @Param: TER_MARGIN
    // @DisplayName: Waypoint Terrain following altitude margin
    // @Description: Waypoint Terrain following altitude margin.  Vehicle will stop if distance from target altitude is larger than this margin (in meters)
    // @Units: m
    // @Range: 0.1 100
    // @User: Advanced
    AP_GROUPINFO("TER_MARGIN",  12, AC_WPNav, _terrain_margin_m, 10.0),

    // @Param: ACCEL_C
    // @DisplayName: Waypoint Cornering Acceleration
    // @Description: Defines the maximum cornering acceleration in cm/s/s used during missions.  If zero uses 2x accel value.
    // @Units: cm/s/s
    // @Range: 0 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("ACCEL_C",     13, AC_WPNav, _wp_accel_c_cmss, 0.0),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero their values.
AC_WPNav::AC_WPNav(const AP_AHRS_View& ahrs, AC_PosControl& pos_control, const AC_AttitudeControl& attitude_control) :
    _ahrs(ahrs),
    _pos_control(pos_control),
    _attitude_control(attitude_control)
{
    AP_Param::setup_object_defaults(this, var_info);

    // init flags
    _flags.reached_destination = false;
    _flags.fast_waypoint = false;

    // initialise old WPNAV_SPEED values
    _last_wp_speed_cms = _wp_speed_cms;
    _last_wp_speed_up_cms = _wp_speed_up_cms;
    _last_wp_speed_down_cms = get_default_speed_down_cms();
}

// Returns the expected source of terrain data when using alt-above-terrain commands.
// Used by systems like ModeRTL to determine which terrain provider is active.
AC_WPNav::TerrainSource AC_WPNav::get_terrain_source() const
{
    // use range finder if connected
    // prioritise rangefinder when enabled and available
    if (_rangefinder_available && _rangefinder_use) {
        return AC_WPNav::TerrainSource::TERRAIN_FROM_RANGEFINDER;
    }
#if AP_TERRAIN_AVAILABLE
    const AP_Terrain *terrain = AP::terrain();
    // use terrain database if enabled
    if (terrain != nullptr && terrain->enabled()) {
        return AC_WPNav::TerrainSource::TERRAIN_FROM_TERRAINDATABASE;
    } else {
        return AC_WPNav::TerrainSource::TERRAIN_UNAVAILABLE;
    }
#else
    // fallback if no terrain support is compiled in
    return AC_WPNav::TerrainSource::TERRAIN_UNAVAILABLE;
#endif
}

///
/// waypoint navigation
///

// Initializes waypoint and spline navigation using inputs in meters.
// Sets speed and acceleration limits, calculates jerk constraints,
// and initializes spline or S-curve leg with a defined starting point.
void AC_WPNav::wp_and_spline_init_m(float speed_ms, Vector3p stopping_point_ned_m)
{    
    // ensure waypoint radius is not below minimum allowed value
    _wp_radius_cm.set_and_save_ifchanged(MAX(_wp_radius_cm, WPNAV_WP_RADIUS_MIN_CM));

    // ensure waypoint speed is not below minimum allowed value
    _wp_speed_cms.set_and_save_ifchanged(MAX(_wp_speed_cms, WPNAV_WP_SPEED_MIN_MS * 100.0));

    // initialise position controller
    _pos_control.D_init_controller_stopping_point();
    _pos_control.NE_init_controller_stopping_point();

    // determine desired waypoint speed; fallback to default if not provided
    _check_wp_speed_change = !is_positive(speed_ms);
    _wp_desired_speed_ne_ms = is_positive(speed_ms) ? speed_ms : get_default_speed_NE_ms();
    _wp_desired_speed_ne_ms = MAX(_wp_desired_speed_ne_ms, WPNAV_WP_SPEED_MIN_MS);

    // initialise position controller speed and acceleration
    _pos_control.NE_set_max_speed_accel_m(_wp_desired_speed_ne_ms, get_wp_acceleration_mss());
    _pos_control.NE_set_correction_speed_accel_m(_wp_desired_speed_ne_ms, get_wp_acceleration_mss());
    _pos_control.D_set_max_speed_accel_m(get_default_speed_down_ms(), get_default_speed_up_ms(), get_accel_D_mss());
    _pos_control.D_set_correction_speed_accel_m(get_default_speed_down_ms(), get_default_speed_up_ms(), get_accel_D_mss());

    // calculate jerk limit if not explicitly set by parameter
    if (!is_positive(_wp_jerk_msss)) {
        _wp_jerk_msss.set(get_wp_acceleration_mss());
    }
    calc_scurve_jerk_and_snap();

    _scurve_prev_leg.init();
    _scurve_this_leg.init();
    _scurve_next_leg.init();
    _track_dt_scalar = 1.0f;

    _flags.reached_destination = true;
    _flags.fast_waypoint = false;

    // determine initial origin and destination; fallback to current stopping point if not provided
    if (stopping_point_ned_m.is_zero()) {
        get_wp_stopping_point_NED_m(stopping_point_ned_m);
    }
    _origin_ned_m = _destination_ned_m = stopping_point_ned_m;
    _is_terrain_alt = false;
    _this_leg_is_spline = false;

    // initialise velocity profile for terrain margin shaping
    _offset_vel_ms = _wp_desired_speed_ne_ms;
    _offset_accel_mss = 0.0;
    _paused = false;

    // mark as active
    _wp_last_update_ms = AP_HAL::millis();
}

// Sets the target horizontal speed in cm/s during waypoint navigation.
// See set_speed_NE_ms() for full details.
void AC_WPNav::set_speed_NE_cms(float speed_cms)
{
    set_speed_NE_ms(speed_cms * 0.01);
}

// Sets the target horizontal speed in m/s during waypoint navigation.
// Also updates internal velocity offsets and path shaping limits.
void AC_WPNav::set_speed_NE_ms(float speed_ms)
{
    // validate input: speed must be above minimum and current desired speed must be non-zero
    if (speed_ms >= WPNAV_WP_SPEED_MIN_MS && is_positive(_wp_desired_speed_ne_ms)) {
        // adjust internal velocity offset to preserve relative motion scaling
        _offset_vel_ms = speed_ms * _offset_vel_ms / _wp_desired_speed_ne_ms;

        // set new desired horizontal speed for waypoint controller
        _wp_desired_speed_ne_ms = speed_ms;

        // update horizontal shaping and correction constraints
        _pos_control.NE_set_max_speed_accel_m(_wp_desired_speed_ne_ms, get_wp_acceleration_mss());
        _pos_control.NE_set_correction_speed_accel_m(_wp_desired_speed_ne_ms, get_wp_acceleration_mss());

        // update internal trajectory with new limits
        update_track_with_speed_accel_limits();
    }
}

// Sets the climb speed for waypoint navigation in m/s.
// Updates the vertical controller with the new ascent rate limit.
void AC_WPNav::set_speed_up_ms(float speed_up_ms)
{
    // update vertical controller's max speed and accel limits (U axis)
    _pos_control.D_set_max_speed_accel_m(_pos_control.get_max_speed_down_ms(), speed_up_ms, _pos_control.D_get_max_accel_mss());

    // recompute the trajectory using updated limits
    update_track_with_speed_accel_limits();
}

// Sets the descent speed for waypoint navigation in m/s.
// Updates the vertical controller with the new descent rate limit.
void AC_WPNav::set_speed_down_ms(float speed_down_ms)
{
    // update vertical controller descent speed
    _pos_control.D_set_max_speed_accel_m(speed_down_ms, _pos_control.get_max_speed_up_ms(), _pos_control.D_get_max_accel_mss());

    // recompute the trajectory using updated limits
    update_track_with_speed_accel_limits();
}

// Sets the current waypoint destination using a Location object.
// Converts global coordinates to NED position and sets destination.
// arc_rad specifies the signed arc angle in radians for an ARC_WAYPOINT segment (0 for straight path)
// Returns false if conversion fails (e.g. missing terrain data).
bool AC_WPNav::set_wp_destination_loc(const Location& destination, float arc_rad)
{
    bool is_terrain_alt;
    Vector3p dest_ned_m;

    // convert Location to NED position vector in meters and determine altitude reference frame
    if (!get_vector_NED_m(destination, dest_ned_m, is_terrain_alt)) {
        return false;
    }

    // apply destination as the active waypoint leg
    return set_wp_destination_NED_m(dest_ned_m, is_terrain_alt, arc_rad);
}

// Sets the next waypoint destination using a Location object.
// Converts global coordinates to NED position and preloads the trajectory.
// arc_rad specifies the signed arc angle in radians for an ARC_WAYPOINT segment (0 for straight path)
// Returns false if conversion fails or terrain data is unavailable.
bool AC_WPNav::set_wp_destination_next_loc(const Location& destination, float arc_rad)
{
    bool is_terrain_alt;
    Vector3p dest_ned_m;

    // convert Location to NED position vector in meters and determine altitude reference frame
    if (!get_vector_NED_m(destination, dest_ned_m, is_terrain_alt)) {
        return false;
    }

    // apply destination as the next waypoint leg
    return set_wp_destination_next_NED_m(dest_ned_m, is_terrain_alt, arc_rad);
}

// Gets the current waypoint destination as a Location object.
// Altitude frame will be ABOVE_TERRAIN or ABOVE_ORIGIN depending on path configuration.
// Returns false if origin is not set or coordinate conversion fails.
bool AC_WPNav::get_wp_destination_loc(Location& destination) const
{
    // retrieve global origin for coordinate conversion
    if (!AP::ahrs().get_origin(destination)) {
        return false;
    }

    // convert NED waypoint to global Location format with appropriate altitude frame
    destination = Location::from_ekf_offset_NED_m(get_wp_destination_NED_m(), _is_terrain_alt ? Location::AltFrame::ABOVE_TERRAIN : Location::AltFrame::ABOVE_ORIGIN);
    return true;
}

// Sets waypoint destination using NEU position vector in centimeters from EKF origin.
// See set_wp_destination_NED_m() for full details.
bool AC_WPNav::set_wp_destination_NEU_cm(const Vector3f& destination_neu_cm, bool is_terrain_alt)
{
    Vector3p destination_ned_m = Vector3p(destination_neu_cm.x, destination_neu_cm.y, -destination_neu_cm.z) * 0.01;
    return set_wp_destination_NED_m(destination_ned_m, is_terrain_alt);
}

// Sets waypoint destination using NED position vector in meters from EKF origin.
// If `is_terrain_alt` is true, altitude is interpreted as height above terrain.
// Reinitializes the current leg if interrupted, updates origin, and computes trajectory.
// arc_rad specifies the signed arc angle in radians for an ARC_WAYPOINT segment (0 for straight path)
// Returns false if terrain offset cannot be determined when required.
bool AC_WPNav::set_wp_destination_NED_m(const Vector3p& destination_ned_m, bool is_terrain_alt, float arc_rad)
{
    // re-initialise if previous destination has been interrupted
    if (!is_active() || !_flags.reached_destination) {
        wp_and_spline_init_m(_wp_desired_speed_ne_ms);
    }

    _scurve_prev_leg.init();
    float origin_speed_m = 0.0f;

    // use previous destination as origin
    _origin_ned_m = _destination_ned_m;

    if (is_terrain_alt == _is_terrain_alt) {
        if (_this_leg_is_spline) {
            // Use velocity from end of spline leg to seed new S-curve origin speed
            Vector3f curr_target_vel_ned_ms = _pos_control.get_vel_desired_NED_ms();
            curr_target_vel_ned_ms.z -= _pos_control.get_vel_offset_D_ms();
            origin_speed_m = curr_target_vel_ned_ms.length();
        } else {
            // Preserve current leg profile to enable blending with new leg
            _scurve_prev_leg = _scurve_this_leg;
        }
    } else {
        // Handle transition between terrain-relative and origin-relative altitude frames
        float terrain_d_m;
        if (!get_terrain_D_m(terrain_d_m)) {
            return false;
        }

        // convert origin to alt-above-terrain if necessary
        if (is_terrain_alt) {
            // Convert origin.z to terrain-relative altitude
            _origin_ned_m.z -= terrain_d_m;
            _pos_control.init_pos_terrain_D_m(terrain_d_m);
        } else {
            // Convert origin.z to origin-relative altitude
            _origin_ned_m.z += terrain_d_m;
            _pos_control.init_pos_terrain_D_m(0.0);
        }
    }

    // update destination
    _destination_ned_m = destination_ned_m;
    _is_terrain_alt = is_terrain_alt;

    if (_flags.fast_waypoint && !_this_leg_is_spline && !_next_leg_is_spline && !_scurve_next_leg.finished()) {
        // Reuse preloaded next leg if valid to avoid unnecessary recalculation
        _scurve_this_leg = _scurve_next_leg;
    } else {
        // Generate a new S-curve segment to the new destination
        _scurve_this_leg.calculate_track(_origin_ned_m, _destination_ned_m, arc_rad,
                                         _pos_control.NE_get_max_speed_ms(), _pos_control.get_max_speed_up_ms(), _pos_control.get_max_speed_down_ms(),
                                         get_wp_acceleration_mss(), get_accel_D_mss(), get_corner_acceleration_mss(),
                                         _scurve_snap_max_mssss, _scurve_jerk_max_msss);
        if (!is_zero(origin_speed_m)) {
            // If we have a valid starting speed, seed it into the S-curve
            _scurve_this_leg.set_origin_speed_max(origin_speed_m);
        }
    }

    _this_leg_is_spline = false;
    _scurve_next_leg.init();
    _next_destination_ned_m.zero(); // clear next destination_ned_m
    _flags.fast_waypoint = false;   // default waypoint back to slow
    _flags.reached_destination = false;

    return true;
}

// Sets the next waypoint destination using a NED position vector in meters.
// Only updates if terrain frame matches current leg.
// Calculates trajectory preview for smoother transition into next segment.
// Updates velocity handoff if previous leg is a spline.
// arc_rad specifies the signed arc angle in radians for an ARC_WAYPOINT segment (0 for straight path)
bool AC_WPNav::set_wp_destination_next_NED_m(const Vector3p& destination_ned_m, bool is_terrain_alt, float arc_rad)
{
    // do not add next point if alt types don't match
    if (is_terrain_alt != _is_terrain_alt) {
        return true;
    }

    // Preload next S-curve leg with current speed and acceleration constraints
    _scurve_next_leg.calculate_track(_destination_ned_m, destination_ned_m, arc_rad,
                                     _pos_control.NE_get_max_speed_ms(), _pos_control.get_max_speed_up_ms(), _pos_control.get_max_speed_down_ms(),
                                     get_wp_acceleration_mss(), get_accel_D_mss(), get_corner_acceleration_mss(),
                                     _scurve_snap_max_mssss, _scurve_jerk_max_msss);
    if (_this_leg_is_spline) {
        const float this_leg_dest_speed_max_ms = _spline_this_leg.get_destination_speed_max();
        // Pass velocity forward to allow continuous spline-scurve transition
        const float next_leg_origin_speed_max_ms = _scurve_next_leg.set_origin_speed_max(this_leg_dest_speed_max_ms);
        _spline_this_leg.set_destination_speed_max(next_leg_origin_speed_max_ms);
    }
    _next_leg_is_spline = false;

    // Enable fast waypoint transition since next leg is known
    _flags.fast_waypoint = true;

    // Store the upcoming destination for reference
    _next_destination_ned_m = destination_ned_m;

    return true;
}

// Computes the horizontal stopping point in NE frame, returned in centimeters.
// See get_wp_stopping_point_NE_m() for full details.
void AC_WPNav::get_wp_stopping_point_NE_cm(Vector2f& stopping_point_ne_cm) const
{
    // convert input/output to meters to match internal representation
    Vector2p stopping_point_ne_m = stopping_point_ne_cm.topostype() * 0.01;
    get_wp_stopping_point_NE_m(stopping_point_ne_m);
    stopping_point_ne_cm = stopping_point_ne_m.tofloat() * 100.0;
}

// Computes the horizontal stopping point in NE frame, in meters, based on current velocity and configured acceleration.
// This is the point where the vehicle would come to a stop if decelerated using the configured limits.
void AC_WPNav::get_wp_stopping_point_NE_m(Vector2p& stopping_point_ne_m) const
{
    // request stopping point from the position controller
    _pos_control.get_stopping_point_NE_m(stopping_point_ne_m);
}

// Computes the full 3D NEU stopping point vector in centimeters based on current kinematics.
// See get_wp_stopping_point_NED_m() for full details.
void AC_WPNav::get_wp_stopping_point_NEU_cm(Vector3f& stopping_point_neu_cm) const
{
    // convert input from cm to m for internal calculation
    Vector3p stopping_point_ned_m = Vector3p(stopping_point_neu_cm.x, stopping_point_neu_cm.y, -stopping_point_neu_cm.z) * 0.01;
    // compute stopping point using meters
    get_wp_stopping_point_NED_m(stopping_point_ned_m);
    // convert result back to centimeters
    stopping_point_neu_cm = Vector3f(stopping_point_ned_m.x, stopping_point_ned_m.y, -stopping_point_ned_m.z) * 100.0;
}

// Computes the full 3D NED stopping point in meters based on current velocity and configured acceleration in all axes.
// Represents where the vehicle will stop if decelerated from current velocity using configured limits.
void AC_WPNav::get_wp_stopping_point_NED_m(Vector3p& stopping_point_ned_m) const
{
    // get horizontal stopping point (North and East)
    _pos_control.get_stopping_point_NE_m(stopping_point_ned_m.xy());
    // get vertical stopping point (Down)
    _pos_control.get_stopping_point_D_m(stopping_point_ned_m.z);
}

// Advances the target location along the current path segment.
// Updates target position, velocity, and acceleration based on jerk-limited profile (or spline).
// Returns true if the update succeeded (e.g., terrain data was available).
bool AC_WPNav::advance_wp_target_along_track(float dt)
{
    // calculate terrain offset if using alt-above-terrain frame
    float terr_offset_d_m = 0.0f;
    if (_is_terrain_alt && !get_terrain_D_m(terr_offset_d_m)) {
        return false;
    }

    // calculate terrain-based velocity scaling factor
    const float offset_d_scalar = _pos_control.terrain_scaler_D_m(terr_offset_d_m, get_terrain_margin_m());

    // input shape the terrain offset
    _pos_control.set_pos_terrain_target_D_m(terr_offset_d_m);

    // get position controller's post-shaped position offset for use in position error computation
    const Vector3p& psc_pos_offset_ned_m = _pos_control.get_pos_offset_NED_m();

    // compute current position in NED frame, adjusted to destination frame (e.g., terrain-relative if needed)
    Vector3p curr_pos_ned_m = _pos_control.get_pos_estimate_NED_m() - psc_pos_offset_ned_m;
    curr_pos_ned_m.z -= terr_offset_d_m;

    // get desired velocity and remove offset
    Vector3f curr_target_vel_ned_ms = _pos_control.get_vel_desired_NED_ms();
    curr_target_vel_ned_ms.z -= _pos_control.get_vel_offset_D_ms();

    // scale progression time (track_dt_scalar) based on aircraft’s speed alignment with desired path
    float track_dt_scalar = 1.0f;
    if (is_positive(curr_target_vel_ned_ms.length_squared())) {
        Vector3f track_direction_neu = curr_target_vel_ned_ms.normalized();
        const float track_error_ned_m = _pos_control.get_pos_error_NED_m().dot(track_direction_neu);
        const float track_velocity_ned_ms = _pos_control.get_vel_estimate_NED_ms().dot(track_direction_neu);
        // limit time step scalar to [0,1], with 5% buffer
        track_dt_scalar = constrain_float(0.05f + (track_velocity_ned_ms - _pos_control.NE_get_pos_p().kP() * track_error_ned_m) / curr_target_vel_ned_ms.length(), 0.0f, 1.0f);
    }

    // compute velocity scaling (vel_dt_scalar) and apply jerk-limited velocity shaping
    float vel_dt_scalar = 1.0;
    if (is_positive(_wp_desired_speed_ne_ms)) {
        update_vel_accel(_offset_vel_ms, _offset_accel_mss, dt, 0.0, 0.0);
        const float vel_input_ms = !_paused ? _wp_desired_speed_ne_ms * offset_d_scalar : 0.0;
        shape_vel_accel(vel_input_ms, 0.0, _offset_vel_ms, _offset_accel_mss, -get_wp_acceleration_mss(), get_wp_acceleration_mss(),
                        _pos_control.get_shaping_jerk_NE_msss(), dt, true);
        vel_dt_scalar = _offset_vel_ms / _wp_desired_speed_ne_ms;
    }

    // apply exponential filter to track_dt_scalar using jerk-based time constant
    float track_dt_scalar_tc = 1.0f;
    if (!is_zero(_wp_jerk_msss)) {
        track_dt_scalar_tc = get_wp_acceleration_mss()/_wp_jerk_msss;
    }
    _track_dt_scalar += (track_dt_scalar - _track_dt_scalar) * (dt / track_dt_scalar_tc);

    // select path generator (S-curve or spline) and compute target state
    Vector3p target_pos_ned_m;
    Vector3f target_vel_ned_ms, target_accel_ned_mss;
    bool s_finished;
    if (!_this_leg_is_spline) {
        // update target position, velocity and acceleration
        target_pos_ned_m = _origin_ned_m;
        s_finished = _scurve_this_leg.advance_target_along_track(_scurve_prev_leg, _scurve_next_leg, _wp_radius_cm * 0.01, get_corner_acceleration_mss(), _flags.fast_waypoint, _track_dt_scalar * vel_dt_scalar * dt, target_pos_ned_m, target_vel_ned_ms, target_accel_ned_mss);
    } else {
        // splinetarget_vel
        target_vel_ned_ms = curr_target_vel_ned_ms;
        _spline_this_leg.advance_target_along_track(_track_dt_scalar * vel_dt_scalar * dt, target_pos_ned_m, target_vel_ned_ms);
        s_finished = _spline_this_leg.reached_destination();
    }

    // apply scaled acceleration offset along current direction of travel
    Vector3f accel_offset_ned_mss;
    if (is_positive(target_vel_ned_ms.length_squared())) {
        Vector3f track_direction_neu = target_vel_ned_ms.normalized();
        accel_offset_ned_mss = track_direction_neu * _offset_accel_mss * target_vel_ned_ms.length() / _wp_desired_speed_ne_ms;
    }

    target_vel_ned_ms *= vel_dt_scalar;
    target_accel_ned_mss *= sq(vel_dt_scalar);
    target_accel_ned_mss += accel_offset_ned_mss;

    // send updated position, velocity, and acceleration targets to position controller
    _pos_control.set_pos_vel_accel_NED_m(target_pos_ned_m, target_vel_ned_ms, target_accel_ned_mss);

    // check if waypoint has been reached based on mode and radius
    if (!_flags.reached_destination) {
        if (s_finished) {
            // "fast" waypoints are complete once the intermediate point reaches the destination
            if (_flags.fast_waypoint) {
                _flags.reached_destination = true;
            } else {
                // regular waypoints also require the copter to be within the waypoint radius
                const Vector3f dist_to_dest_m = (curr_pos_ned_m - _destination_ned_m).tofloat();
                if (dist_to_dest_m.length_squared() <= sq(_wp_radius_cm * 0.01)) {
                    _flags.reached_destination = true;
                }
            }
        }
    }

    // successfully advanced along track
    return true;
}

// Updates the current and next path segment to reflect new speed and acceleration limits.
// Should be called after modifying NE/U controller limits or vehicle configuration.
void AC_WPNav::update_track_with_speed_accel_limits()
{
    // update speed and acceleration limits for the current segment
    if (_this_leg_is_spline) {
        _spline_this_leg.set_speed_accel(_pos_control.NE_get_max_speed_ms(), _pos_control.get_max_speed_up_ms(), _pos_control.get_max_speed_down_ms(),
                                         get_wp_acceleration_mss(), get_accel_D_mss());
    } else {
        _scurve_this_leg.set_speed_max(_pos_control.NE_get_max_speed_ms(), _pos_control.get_max_speed_up_ms(), _pos_control.get_max_speed_down_ms());
    }

    // update speed and acceleration limits for the next segment
    if (_next_leg_is_spline) {
        _spline_next_leg.set_speed_accel(_pos_control.NE_get_max_speed_ms(), _pos_control.get_max_speed_up_ms(), _pos_control.get_max_speed_down_ms(),
                                         get_wp_acceleration_mss(), get_accel_D_mss());
    } else {
        _scurve_next_leg.set_speed_max(_pos_control.NE_get_max_speed_ms(), _pos_control.get_max_speed_up_ms(), _pos_control.get_max_speed_down_ms());
    }
}

// Returns the horizontal distance to the destination waypoint in centimeters.
// See get_wp_distance_to_destination_m() for full details.
float AC_WPNav::get_wp_distance_to_destination_cm() const
{
    // convert distance from meters to centimeters
    return get_wp_distance_to_destination_m() * 100.0;
}

// Returns the horizontal distance in meters between the current position and the destination waypoint.
float AC_WPNav::get_wp_distance_to_destination_m() const
{
    // calculate 2D ground distance between current position and destination in NE frame
    return get_horizontal_distance(_pos_control.get_pos_estimate_NED_m().xy().tofloat(), _destination_ned_m.xy().tofloat());
}

// Returns the bearing to the current waypoint destination in centidegrees.
// See get_wp_bearing_to_destination_rad() for full details.
int32_t AC_WPNav::get_wp_bearing_to_destination_cd() const
{
    // compute heading from current position to destination in centidegrees
    return get_bearing_cd(_pos_control.get_pos_estimate_NED_m().xy().tofloat(), _destination_ned_m.xy().tofloat());
}

// Returns the bearing to the current waypoint destination in radians.
// The bearing is measured clockwise from North, with 0 = North.
float AC_WPNav::get_wp_bearing_to_destination_rad() const
{
    // compute heading from current position to destination in radians
    return get_bearing_rad(_pos_control.get_pos_estimate_NED_m().xy().tofloat(), _destination_ned_m.xy().tofloat());
}

// Runs the waypoint navigation controller.
// Advances the target position and updates the position controller.
// Should be called at 100 Hz or higher for accurate tracking.
bool AC_WPNav::update_wpnav()
{
    // check for changes in WPNAV_SPEED parameter (horizontal speed target)
    if (_check_wp_speed_change) {
        if (!is_equal(_wp_speed_cms.get(), _last_wp_speed_cms)) {
            // apply new WPNAV_SPEED value
            set_speed_NE_ms(get_default_speed_NE_ms());
            _last_wp_speed_cms = _wp_speed_cms;
        }
    }
    // check for climb and descent speed updates
    if (!is_equal(_wp_speed_up_cms.get(), _last_wp_speed_up_cms)) {
        set_speed_up_ms(get_default_speed_up_ms());
        _last_wp_speed_up_cms = _wp_speed_up_cms;
    }
    if (!is_equal(_wp_speed_down_cms.get(), _last_wp_speed_down_cms)) {
        set_speed_down_ms(get_default_speed_down_ms());
        _last_wp_speed_down_cms = _wp_speed_down_cms;
    }

    // advance the waypoint target based on current position and timing
    bool ret = true;
    if (!advance_wp_target_along_track(_pos_control.get_dt_s())) {
        // To-Do: handle inability to advance along track (probably because of missing terrain data)
        ret = false;
    }

    // run the horizontal position controller
    _pos_control.NE_update_controller();

    // record update time for is_active()
    _wp_last_update_ms = AP_HAL::millis();

    return ret;
}

// Returns true if update_wpnav() has been called within the last 200 ms.
// Used to check if waypoint navigation is currently active.
bool AC_WPNav::is_active() const
{
    // consider WPNAV active if last update was within 200 milliseconds
    return (AP_HAL::millis() - _wp_last_update_ms) < 200;
}

// Forces a stop at the next waypoint instead of continuing to the subsequent one.
// Used by Dijkstra’s object avoidance when the future path is obstructed.
// Only affects regular (non-spline) waypoints.
// Returns true if the stop behavior was newly enforced.
bool AC_WPNav::force_stop_at_next_wp()
{
    // skip if vehicle was already going to stop at the next waypoint
    if (!_flags.fast_waypoint) {
        return false;
    }

    _flags.fast_waypoint = false;

    // override final velocity for current leg and reset preview for next leg
    if (!_this_leg_is_spline) {
        _scurve_this_leg.set_destination_speed_max(0);
    }
    if (!_next_leg_is_spline) {
        _scurve_next_leg.init();
    }

    return true;
}

// Returns terrain offset in meters above the EKF origin at the current position.
// Positive values mean terrain lies above the EKF origin altitude.
// Source may be rangefinder or terrain database depending on availability.
bool AC_WPNav::get_terrain_U_m(float& terrain_u_m)
{
    // determine terrain data source and compute offset accordingly
    switch (get_terrain_source()) {
    case AC_WPNav::TerrainSource::TERRAIN_UNAVAILABLE:
        return false;

    case AC_WPNav::TerrainSource::TERRAIN_FROM_RANGEFINDER:
        if (_rangefinder_healthy) {
            // return previously saved offset based on rangefinder
            terrain_u_m = _rangefinder_terrain_u_m;
            return true;
        }
        return false;

    case AC_WPNav::TerrainSource::TERRAIN_FROM_TERRAINDATABASE:
#if AP_TERRAIN_AVAILABLE
        float height_above_terrain_m = 0.0f;
        AP_Terrain *terrain = AP::terrain();
        if (terrain != nullptr &&
            terrain->height_above_terrain(height_above_terrain_m, true)) {
            // compute offset as difference between current altitude and terrain height
            terrain_u_m = _pos_control.get_pos_estimate_U_m() - height_above_terrain_m;
            return true;
        }
#endif
        return false;
    }

    // unreachable fallback path
    return false;
}
bool AC_WPNav::get_terrain_D_m(float& terrain_d_m)
{
        if (!get_terrain_U_m(terrain_d_m)) {
            return false;
        }
        terrain_d_m *= -1.0;
        return true;
}

///
/// spline methods
///

// Sets the current spline waypoint using global coordinates.
// Converts `destination` and `next_destination` to NED position vectors and sets up a spline between them.
// Returns false if conversion from location to vector fails.
bool AC_WPNav::set_spline_destination_loc(const Location& destination, const Location& next_destination, bool next_is_spline)
{
    // convert destination location to NED vector and altitude frame
    Vector3p dest_ned_m;
    bool dest_is_terrain_alt;
    if (!get_vector_NED_m(destination, dest_ned_m, dest_is_terrain_alt)) {
        return false;
    }

    // convert next destination location to NED vector and altitude frame
    Vector3p next_dest_ned_m;
    bool next_dest_is_terrain_alt;
    if (!get_vector_NED_m(next_destination, next_dest_ned_m, next_dest_is_terrain_alt)) {
        return false;
    }

    // initialize spline path between converted destination and next_destination
    return set_spline_destination_NED_m(dest_ned_m, dest_is_terrain_alt, next_dest_ned_m, next_dest_is_terrain_alt, next_is_spline);
}

// Sets the next spline segment using global coordinates.
// Converts the next and next-next destinations to NED position vectors and initializes the spline transition.
// Returns false if any conversion from location to vector fails.
bool AC_WPNav::set_spline_destination_next_loc(const Location& next_destination, const Location& next_next_destination, bool next_next_is_spline)
{
    // convert next_destination location to NED vector and determine if it uses terrain-relative altitude
    Vector3p next_dest_ned_m;
    bool next_dest_is_terrain_alt;
    if (!get_vector_NED_m(next_destination, next_dest_ned_m, next_dest_is_terrain_alt)) {
        return false;
    }

    // convert next_next_destination location to NED vector and determine if it uses terrain-relative altitude
    Vector3p next_next_dest_ned_m;
    bool next_next_dest_is_terr_alt;
    if (!get_vector_NED_m(next_next_destination, next_next_dest_ned_m, next_next_dest_is_terr_alt)) {
        return false;
    }

    // initialize next spline segment using converted waypoints
    return set_spline_destination_next_NED_m(next_dest_ned_m, next_dest_is_terrain_alt, next_next_dest_ned_m, next_next_dest_is_terr_alt, next_next_is_spline);
}

// Sets the current spline waypoint using NED position vectors in meters.
// Initializes a spline path from `destination_ned_m` to `next_destination_ned_m`, respecting terrain altitude framing.
// Both waypoints must use the same altitude frame (either above terrain or above origin).
// Returns false if terrain altitude cannot be determined when required.
bool AC_WPNav::set_spline_destination_NED_m(const Vector3p& destination_ned_m, bool is_terrain_alt, const Vector3p& next_destination_ned_m, bool next_is_terrain_alt, bool next_is_spline)
{
    // re-initialise path state if previous destination was not completed or controller inactive
    if (!is_active() || !_flags.reached_destination) {
        wp_and_spline_init_m(_wp_desired_speed_ne_ms);
    }

    // update spline calculators speeds and accelerations
    _spline_this_leg.set_speed_accel(_pos_control.NE_get_max_speed_ms(), _pos_control.get_max_speed_up_ms(), _pos_control.get_max_speed_down_ms(),
                                     _pos_control.NE_get_max_accel_mss(), _pos_control.D_get_max_accel_mss());

    // calculate origin and origin velocity vector
    Vector3f origin_vector_ned_m;
    if (is_terrain_alt == _is_terrain_alt) {
        if (_flags.fast_waypoint) {
            // calculate origin vector
            if (_this_leg_is_spline) {
                // use velocity vector from end of previous spline segment
                origin_vector_ned_m = _spline_this_leg.get_destination_vel();
            } else {
                // use direction vector from previous straight-line segment
                origin_vector_ned_m = (_destination_ned_m - _origin_ned_m).tofloat();
            }
        }

        // use previous destination as origin
        _origin_ned_m = _destination_ned_m;
    } else {

        // use previous destination as origin
        _origin_ned_m = _destination_ned_m;

        // get current alt above terrain
        float terrain_d_m;
        if (!get_terrain_D_m(terrain_d_m)) {
            return false;
        }

        // convert origin to alt-above-terrain if necessary
        if (is_terrain_alt) {
            // new destination is alt-above-terrain, previous destination was alt-above-ekf-origin
            _origin_ned_m.z -= terrain_d_m;
            _pos_control.init_pos_terrain_D_m(terrain_d_m);
        } else {
            // new destination is alt-above-ekf-origin, previous destination was alt-above-terrain
            _origin_ned_m.z += terrain_d_m;
            _pos_control.init_pos_terrain_D_m(0.0);
        }
    }

    // store destination location
    _destination_ned_m = destination_ned_m;
    _is_terrain_alt = is_terrain_alt;

    // calculate destination velocity vector
    Vector3f destination_vector_ned_m;
    if (is_terrain_alt == next_is_terrain_alt) {
        if (next_is_spline) {
            // aim to leave segment in direction of full arc (origin to next)
            destination_vector_ned_m = (next_destination_ned_m - _origin_ned_m).tofloat();
        } else {
            // aim to leave segment in direction of following leg
            destination_vector_ned_m = (next_destination_ned_m - _destination_ned_m).tofloat();
        }
    }
    _flags.fast_waypoint = !destination_vector_ned_m.is_zero();

    // setup spline leg using origin and destination vectors
    _spline_this_leg.set_origin_and_destination(_origin_ned_m, _destination_ned_m, origin_vector_ned_m, destination_vector_ned_m);
    _this_leg_is_spline = true;
    _flags.reached_destination = false;

    return true;
}

// Sets the next spline segment using NED position vectors in meters.
// Creates a spline path from the current destination to `next_destination_ned_m`, and prepares transition toward `next_next_destination_ned_m`.
// All waypoints must use the same altitude frame (above terrain or origin).
// Returns false if terrain data is missing and required.
bool AC_WPNav::set_spline_destination_next_NED_m(const Vector3p& next_destination_ned_m, bool next_is_terrain_alt, const Vector3p& next_next_destination_ned_m, bool next_next_is_terrain_alt, bool next_next_is_spline)
{
    // do not add next point if alt types don't match
    if (next_is_terrain_alt != _is_terrain_alt) {
        return true;
    }

    // calculate origin and origin velocity vector
    Vector3f origin_vector_ned_m;
    if (_this_leg_is_spline) {
        // use final velocity vector from current spline segment
        origin_vector_ned_m = _spline_this_leg.get_destination_vel();
    } else {
        // use vector from previous origin to current destination
        origin_vector_ned_m = (_destination_ned_m - _origin_ned_m).tofloat();
    }

    // calculate destination velocity vector
    Vector3f destination_vector_ned_m;
    if (next_is_terrain_alt == next_next_is_terrain_alt) {
        if (next_next_is_spline) {
            // aim to leave segment in direction of the arc from current to next-next
            destination_vector_ned_m = (next_next_destination_ned_m - _destination_ned_m).tofloat();
        } else {
            // aim to leave segment in direction of the upcoming straight leg
            destination_vector_ned_m = (next_next_destination_ned_m - next_destination_ned_m).tofloat();
        }
    }

    // update spline calculators speeds and accelerations
    _spline_next_leg.set_speed_accel(_pos_control.NE_get_max_speed_ms(), _pos_control.get_max_speed_up_ms(), _pos_control.get_max_speed_down_ms(),
                                     _pos_control.NE_get_max_accel_mss(), _pos_control.D_get_max_accel_mss());

    // setup next spline leg.  Note this could be made local
    _spline_next_leg.set_origin_and_destination(_destination_ned_m, next_destination_ned_m, origin_vector_ned_m, destination_vector_ned_m);
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

// Converts a Location to a NED position vector in meters from the EKF origin.
// Sets `is_terrain_alt` to true if the resulting Z position is relative to terrain.
// Returns false if terrain data is unavailable or conversion fails.
bool AC_WPNav::get_vector_NED_m(const Location &loc, Vector3p &pos_from_origin_ned_m, bool &is_terrain_alt)
{
    // convert horizontal position (latitude/longitude) to NE vector from EKF origin
    Vector2p loc_pos_from_origin_ned_m;
    if (!loc.get_vector_xy_from_origin_NE_m(loc_pos_from_origin_ned_m)) {
        return false;
    }

    // convert altitude
    if (loc.get_alt_frame() == Location::AltFrame::ABOVE_TERRAIN) {
        float terrain_u_m;
        if (!loc.get_alt_m(Location::AltFrame::ABOVE_TERRAIN, terrain_u_m)) {
            return false;
        }
        pos_from_origin_ned_m.z = -terrain_u_m;
        is_terrain_alt = true;
    } else {
        is_terrain_alt = false;
        float origin_alt_m;
        if (!loc.get_alt_m(Location::AltFrame::ABOVE_ORIGIN, origin_alt_m)) {
            return false;
        }
        pos_from_origin_ned_m.z = -origin_alt_m;
        is_terrain_alt = false;
    }

    // set horizontal components (x/y) of NED vector after successful conversion
    pos_from_origin_ned_m.xy() = loc_pos_from_origin_ned_m;

    return true;
}

// Calculates s-curve jerk and snap limits based on attitude controller capabilities.
// Updates _scurve_jerk_max_msss and _scurve_snap_max_mssss with constrained values.
void AC_WPNav::calc_scurve_jerk_and_snap()
{
    // calculate max horizontal jerk based on roll/pitch angular velocity limits
    _scurve_jerk_max_msss = MIN(_attitude_control.get_ang_vel_roll_max_rads() * GRAVITY_MSS, _attitude_control.get_ang_vel_pitch_max_rads() * GRAVITY_MSS);
    if (is_zero(_scurve_jerk_max_msss)) {
        _scurve_jerk_max_msss = _wp_jerk_msss;
    } else {
        _scurve_jerk_max_msss = MIN(_scurve_jerk_max_msss, _wp_jerk_msss);
    }

    // calculate maximum snap (rate of change of jerk)
    // uses input time constant as proxy for lean angle actuation latency
    _scurve_snap_max_mssss = (_scurve_jerk_max_msss * M_PI) / (2.0 * MAX(_attitude_control.get_input_tc(), 0.1f));

    // constrain snap based on roll/pitch angular acceleration capability
    const float snap = MIN(_attitude_control.get_accel_roll_max_radss(), _attitude_control.get_accel_pitch_max_radss()) * GRAVITY_MSS;
    if (is_positive(snap)) {
        _scurve_snap_max_mssss = MIN(_scurve_snap_max_mssss, snap);
    }

    // apply safety margin: reduce snap to 50% of calculated maximum
    _scurve_snap_max_mssss *= 0.5;
}
