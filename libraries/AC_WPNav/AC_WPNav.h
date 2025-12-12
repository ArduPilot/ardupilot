#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/SCurve.h>
#include <AP_Math/SplineCurve.h>
#include <AP_Common/Location.h>
#include <AC_AttitudeControl/AC_PosControl.h>      // Position control library
#include <AC_AttitudeControl/AC_AttitudeControl.h> // Attitude control library
#include <AP_Terrain/AP_Terrain.h>
#include <AC_Avoidance/AC_Avoid.h>                 // Stop at fence library

// maximum velocities and accelerations
#define WPNAV_ACCELERATION_MS           2.5        // default horizontal acceleration limit for waypoint navigation (m/s²)

class AC_WPNav
{
public:

    /// Constructor
    AC_WPNav(const AP_AHRS_View& ahrs, AC_PosControl& pos_control, const AC_AttitudeControl& attitude_control);

    // Sets terrain offset in cm from EKF origin using rangefinder.
    // See set_rangefinder_terrain_U_m() for full details.
    void set_rangefinder_terrain_U_cm(bool use, bool healthy, float terrain_u_cm) { _rangefinder_available = use; _rangefinder_healthy = healthy; _rangefinder_terrain_u_m = terrain_u_cm * 0.01;}

    // Sets terrain offset in meters from EKF origin using rangefinder data.
    // This value is used to determine alt-above-terrain for terrain following.
    void set_rangefinder_terrain_U_m(bool use, bool healthy, float terrain_u_m) { _rangefinder_available = use; _rangefinder_healthy = healthy; _rangefinder_terrain_u_m = terrain_u_m;}

    // Returns true if rangefinder is enabled and may be used for terrain following.
    bool rangefinder_used() const { return _rangefinder_use; }

    // Returns true if rangefinder is enabled and currently healthy.
    bool rangefinder_used_and_healthy() const { return _rangefinder_use && _rangefinder_healthy; }

    // Returns the expected source of terrain data when using alt-above-terrain commands.
    // Used by systems like ModeRTL to determine which terrain provider is active.
    enum class TerrainSource {
        TERRAIN_UNAVAILABLE,
        TERRAIN_FROM_RANGEFINDER,
        TERRAIN_FROM_TERRAINDATABASE,
    };
    AC_WPNav::TerrainSource get_terrain_source() const;

    // Returns terrain offset in meters above the EKF origin at the current position.
    // Positive values mean terrain lies above the EKF origin altitude.
    // Source may be rangefinder or terrain database depending on availability.
    bool get_terrain_U_m(float& terrain_u_m);
    bool get_terrain_D_m(float& terrain_d_m);

    // Returns the terrain following altitude margin in meters.
    // Vehicle will stop if distance from target altitude exceeds this margin.
    float get_terrain_margin_m() const { return MAX(_terrain_margin_m, 0.1); }

    // Converts a Location to a NED position vector in meters from the EKF origin.
    // Sets `is_terrain_alt` to true if the resulting Z position is relative to terrain.
    // Returns false if terrain data is unavailable or conversion fails.
    bool get_vector_NED_m(const Location &loc, Vector3p &pos_from_origin_ned_m, bool &is_terrain_alt);

    ///
    /// waypoint controller
    ///

    // Initializes waypoint and spline navigation using inputs in meters.
    // Sets speed and acceleration limits, calculates jerk constraints,
    // and initializes spline or S-curve leg with a defined starting point.
    void wp_and_spline_init_m(float speed_ms = 0.0f, Vector3p stopping_point_ned_m = Vector3p{});

    // Sets the target horizontal speed in cm/s during waypoint navigation.
    // See set_speed_NE_ms() for full details.
    void set_speed_NE_cms(float speed_cms);

    // Sets the target horizontal speed in m/s during waypoint navigation.
    // Also updates internal velocity offsets and path shaping limits.
    void set_speed_NE_ms(float speed_ms);

    // Pauses progression along the waypoint track.
    void set_pause() { _paused = true; }

    // Resumes waypoint navigation after a pause. Track advancement continues from the current position.
    void set_resume() { _paused = false; }

    // Returns true if waypoint navigation is currently paused via set_pause().
    bool paused() { return _paused; }

    // Sets the climb speed for waypoint navigation in m/s.
    // Updates the vertical controller with the new ascent rate limit.
    void set_speed_up_ms(float speed_up_ms);

    // Sets the descent speed for waypoint navigation in m/s.
    // Updates the vertical controller with the new descent rate limit.
    void set_speed_down_ms(float speed_down_ms);

    // Returns the default horizontal speed in cm/s used during waypoint navigation.
    // See get_default_speed_NE_ms() for full details.
    float get_default_speed_NE_cms() const { return get_default_speed_NE_ms() * 100.0; }

    // Returns the default horizontal speed in m/s used during waypoint navigation.
    // Derived from the WPNAV_SPEED parameter.
    float get_default_speed_NE_ms() const { return _wp_speed_cms * 0.01; }

    // Returns the default climb speed in cm/s used during waypoint navigation.
    // See get_default_speed_up_ms() for full details.
    float get_default_speed_up_cms() const { return get_default_speed_up_ms() * 100.0; }

    // Returns the default climb speed in m/s used during waypoint navigation.
    // Derived from the WPNAV_SPEED_UP parameter.
    float get_default_speed_up_ms() const { return _wp_speed_up_cms * 0.01; }

    // Returns the default descent rate in cm/s used during waypoint navigation.
    // Always positive. See get_default_speed_down_ms() for full details.
    float get_default_speed_down_cms() const { return get_default_speed_down_ms() * 100.0; }

    // Returns the default descent rate in m/s used during waypoint navigation.
    // Derived from the WPNAV_SPEED_DN parameter. Always positive.
    float get_default_speed_down_ms() const { return fabsf(_wp_speed_down_cms * 0.01); }

    // Returns the vertical acceleration in cm/s² used during waypoint navigation.
    // Always positive. See get_accel_D_mss() for full details.
    float get_accel_D_cmss() const { return get_accel_D_mss() * 100.0; }

    // Returns the vertical acceleration in m/s² used during waypoint navigation.
    // Derived from the WPNAV_ACCEL_Z parameter. Always positive.
    float get_accel_D_mss() const { return _wp_accel_z_cmss * 0.01; }

    // Returns the horizontal acceleration in cm/s² used during waypoint navigation.
    // See get_wp_acceleration_mss() for full details.
    float get_wp_acceleration_cmss() const { return get_wp_acceleration_mss() * 100.0; }

    // Returns the horizontal acceleration in m/s² used during waypoint navigation.
    // Derived from the WPNAV_ACCEL parameter. Falls back to a default if unset.
    float get_wp_acceleration_mss() const { return (is_positive(_wp_accel_cmss)) ? _wp_accel_cmss * 0.01 : WPNAV_ACCELERATION_MS; }

    // Returns the maximum lateral acceleration in m/s² used during waypoint cornering.
    // Derived from WPNAV_ACCEL_C or defaults to 2x WPNAV_ACCEL if unset.
    float get_corner_acceleration_mss() const { return (is_positive(_wp_accel_c_cmss)) ? _wp_accel_c_cmss * 0.01 : 2.0 * get_wp_acceleration_mss(); }

    // Returns the destination waypoint vector in NEU frame, in centimeters from EKF origin.
    // See get_wp_destination_NED_m() for full details.
    const Vector3f get_wp_destination_NEU_cm() const { return Vector3f(_destination_ned_m.x, _destination_ned_m.y, -_destination_ned_m.z) * 100.0; }

    // Returns the destination waypoint vector in NED frame, in meters from EKF origin.
    // Z is relative to terrain or EKF origin, depending on _is_terrain_alt.
    const Vector3p &get_wp_destination_NED_m() const { return _destination_ned_m; }

    // Returns the origin waypoint vector in NED frame, in centimeters from EKF origin.
    // See get_wp_origin_NED_m() for full details.
    const Vector3f get_wp_origin_NEU_cm() const { return Vector3f(_origin_ned_m.x, _origin_ned_m.y, -_origin_ned_m.z) * 100.0; }

    // Returns the origin waypoint vector in NED frame, in meters from EKF origin.
    // This marks the start of the current waypoint leg.
    const Vector3p &get_wp_origin_NED_m() const { return _origin_ned_m; }

    // Returns true if origin.z and destination.z are specified as altitudes above terrain.
    // Returns false if altitudes are relative to the EKF origin.
    bool origin_and_destination_are_terrain_alt() const { return _is_terrain_alt; }

    // Sets the current waypoint destination using a Location object.
    // Converts global coordinates to NED position and sets destination.
    // arc_rad specifies the signed arc angle in radians for an ARC_WAYPOINT segment (0 for straight path)
    // Returns false if conversion fails (e.g. missing terrain data).
    bool set_wp_destination_loc(const Location& destination, float arc_rad = 0.0);

    // Sets the next waypoint destination using a Location object.
    // Converts global coordinates to NED position and preloads the trajectory.
    // arc_rad specifies the signed arc angle in radians for an ARC_WAYPOINT segment (0 for straight path)
    // Returns false if conversion fails or terrain data is unavailable.
    bool set_wp_destination_next_loc(const Location& destination, float arc_rad = 0.0);

    // Gets the current waypoint destination as a Location object.
    // Altitude frame will be ABOVE_TERRAIN or ABOVE_ORIGIN depending on path configuration.
    // Returns false if origin is not set or coordinate conversion fails.
    bool get_wp_destination_loc(Location& destination) const;

    // Returns the waypoint destination adjusted for object avoidance.
    // In the base class this is identical to get_wp_destination_loc().
    // Used to unify the AC_WPNav and AC_WPNav_OA interfaces.
    virtual bool get_oa_wp_destination(Location& destination) const { return get_wp_destination_loc(destination); }

    // Sets waypoint destination using NED position vector in centimeters from EKF origin.
    // See set_wp_destination_NED_m() for full details.
    virtual bool set_wp_destination_NEU_cm(const Vector3f& destination_neu_cm, bool is_terrain_alt = false);

    // Sets waypoint destination using NED position vector in meters from EKF origin.
    // If `is_terrain_alt` is true, altitude is interpreted as height above terrain.
    // Reinitializes the current leg if interrupted, updates origin, and computes trajectory.
    // arc_rad specifies the signed arc angle in radians for an ARC_WAYPOINT segment (0 for straight path)
    // Returns false if terrain offset cannot be determined when required.
    virtual bool set_wp_destination_NED_m(const Vector3p& destination_ned_m, bool is_terrain_alt = false, float arc_rad = 0.0);

    // Sets the next waypoint destination using a NED position vector in meters.
    // Only updates if terrain frame matches current leg.
    // Calculates trajectory preview for smoother transition into next segment.
    // Updates velocity handoff if previous leg is a spline.
    // arc_rad specifies the signed arc angle in radians for an ARC_WAYPOINT segment (0 for straight path)
    bool set_wp_destination_next_NED_m(const Vector3p& destination_ned_m, bool is_terrain_alt = false, float arc_rad = 0.0);

    // Computes the horizontal stopping point in NE frame, returned in centimeters.
    // See get_wp_stopping_point_NE_m() for full details.
    void get_wp_stopping_point_NE_cm(Vector2f& stopping_point_ne_cm) const;

    // Computes the horizontal stopping point in NE frame, in meters, based on current velocity and configured acceleration.
    // This is the point where the vehicle would come to a stop if decelerated using the configured limits.
    void get_wp_stopping_point_NE_m(Vector2p& stopping_point_ne_m) const;

    // Computes the full 3D NED stopping point vector in centimeters based on current kinematics.
    // See get_wp_stopping_point_NED_m() for full details.
    void get_wp_stopping_point_NEU_cm(Vector3f& stopping_point_neu_cm) const;

    // Computes the full 3D NED stopping point in meters based on current velocity and configured acceleration in all axes.
    // Represents where the vehicle will stop if decelerated from current velocity using configured limits.
    void get_wp_stopping_point_NED_m(Vector3p& stopping_point_ned_m) const;

    // Returns the horizontal distance to the destination waypoint in centimeters.
    // See get_wp_distance_to_destination_m() for full details.
    virtual float get_wp_distance_to_destination_cm() const;

    // Returns the horizontal distance in meters between the current position and the destination waypoint.
    virtual float get_wp_distance_to_destination_m() const;

    // Returns the bearing to the current waypoint destination in centidegrees.
    // See get_wp_bearing_to_destination_rad() for full details.
    virtual int32_t get_wp_bearing_to_destination_cd() const;

    // Returns the bearing to the current waypoint destination in radians.
    // The bearing is measured clockwise from North, with 0 = North.
    virtual float get_wp_bearing_to_destination_rad() const;

    // Returns true if the vehicle has reached the waypoint destination.
    // A waypoint is considered reached when the vehicle comes within the defined radius threshold.
    virtual bool reached_wp_destination() const { return _flags.reached_destination; }

    // Returns true if the vehicle's horizontal (NE) distance to the waypoint is less than the waypoint radius.
    // Uses the waypoint radius in meters for comparison.
    bool reached_wp_destination_NE() const {
        return get_wp_distance_to_destination_m() < _wp_radius_cm * 0.01;
    }

    // Returns the waypoint acceptance radius in meters.
    // This radius defines the distance from the target waypoint within which the vehicle is considered to have arrived.
    float get_wp_radius_m() const { return _wp_radius_cm * 0.01; }

    // Runs the waypoint navigation controller.
    // Advances the target position and updates the position controller.
    // Should be called at 100 Hz or higher for accurate tracking.
    virtual bool update_wpnav();

    // Returns true if update_wpnav() has been called within the last 200 ms.
    // Used to check if waypoint navigation is currently active.
    bool is_active() const;

    // Forces a stop at the next waypoint instead of continuing to the subsequent one.
    // Used by Dijkstra’s object avoidance when the future path is obstructed.
    // Only affects regular (non-spline) waypoints.
    // Returns true if the stop behavior was newly enforced.
    bool force_stop_at_next_wp();

    ///
    /// spline methods
    ///

    // Sets the current spline waypoint using global coordinates.
    // Converts `destination` and `next_destination` to NED position vectors and sets up a spline between them.
    // Returns false if conversion from location to vector fails.
    bool set_spline_destination_loc(const Location& destination, const Location& next_destination, bool next_is_spline);

    // Sets the next spline segment using global coordinates.
    // Converts the next and next-next destinations to NED position vectors and initializes the spline transition.
    // Returns false if any conversion from location to vector fails.
    bool set_spline_destination_next_loc(const Location& next_destination, const Location& next_next_destination, bool next_next_is_spline);

    // Sets the current spline waypoint using NED position vectors in meters.
    // Initializes a spline path from `destination_ned_m` to `next_destination_ned_m`, respecting terrain altitude framing.
    // Both waypoints must use the same altitude frame (either above terrain or above origin).
    // Returns false if terrain altitude cannot be determined when required.
    bool set_spline_destination_NED_m(const Vector3p& destination_ned_m, bool is_terrain_alt, const Vector3p& next_destination_ned_m, bool next_terrain_alt, bool next_is_spline);

    // Sets the next spline segment using NED position vectors in meters.
    // Creates a spline path from the current destination to `next_destination_ned_m`, and prepares transition toward `next_next_destination_ned_m`.
    // All waypoints must use the same altitude frame (above terrain or origin).
    // Returns false if terrain data is missing and required.
    bool set_spline_destination_next_NED_m(const Vector3p& next_destination_ned_m, bool next_is_terrain_alt, const Vector3p& next_next_destination_ned_m, bool next_next_is_terrain_alt, bool next_next_is_spline);

    ///
    /// shared methods
    ///

    // Returns the desired roll angle in radians from the position controller.
    // This value is passed to the attitude controller to achieve lateral motion.
    float get_roll_rad() const { return _pos_control.get_roll_rad(); }

    // Returns the desired pitch angle in radians from the position controller.
    // This angle controls forward or backward acceleration.
    float get_pitch_rad() const { return _pos_control.get_pitch_rad(); }

    // Returns the desired yaw angle in radians from the position controller.
    // Used to align the vehicle heading with track direction or command input.
    float get_yaw_rad() const { return _pos_control.get_yaw_rad(); }

    // Returns the desired 3D unit vector representing the commanded thrust direction.
    // Used to calculate tilt angles for the attitude controller.
    Vector3f get_thrust_vector() const { return _pos_control.get_thrust_vector(); }

    // Returns the desired roll angle in centidegrees from the position controller.
    // See get_roll_rad() for full details.
    float get_roll() const { return rad_to_cd(get_roll_rad()); }

    // Returns the desired pitch angle in centidegrees from the position controller.
    // See get_pitch_rad() for full details.
    float get_pitch() const { return rad_to_cd(get_pitch_rad()); }

    // Returns the desired yaw angle in centidegrees from the position controller.
    // See get_yaw_rad() for full details.
    float get_yaw() const { return rad_to_cd(get_yaw_rad()); }

    // Advances the target location along the current path segment.
    // Updates target position, velocity, and acceleration based on jerk-limited profile (or spline).
    // Returns true if the update succeeded (e.g., terrain data was available).
    bool advance_wp_target_along_track(float dt);

    // Updates the current and next path segment to reflect new speed and acceleration limits.
    // Should be called after modifying NE/U controller limits or vehicle configuration.
    void update_track_with_speed_accel_limits();

    // Returns lateral (cross-track) position error in meters.
    // Computed as the perpendicular distance between current position and the planned path.
    // Used to assess horizontal deviation from the trajectory.
    float crosstrack_error_m() const { return _pos_control.crosstrack_error_m();}

    static const struct AP_Param::GroupInfo var_info[];

protected:

    // flags structure
    struct wpnav_flags {
        uint8_t reached_destination     : 1;    // true if we have reached the destination
        uint8_t fast_waypoint           : 1;    // true if we should ignore the waypoint radius and consider the waypoint complete once the intermediate target has reached the waypoint
        uint8_t wp_yaw_set              : 1;    // true if yaw target has been set
    } _flags;

    // Calculates s-curve jerk and snap limits based on attitude controller capabilities.
    // Updates _scurve_jerk_max_msss and _scurve_snap_max_mssss with constrained values.
    void calc_scurve_jerk_and_snap();

    // References to shared sensor fusion, position, and attitude control subsystems.
    const AP_AHRS_View&     _ahrs;
    AC_PosControl&          _pos_control;
    const AC_AttitudeControl& _attitude_control;

    // parameters
    AP_Float    _wp_speed_cms;      // default horizontal speed in cm/s for waypoint navigation
    AP_Float    _wp_speed_up_cms;   // default climb rate in cm/s for waypoint navigation
    AP_Float    _wp_speed_down_cms; // default descent rate in cm/s for waypoint navigation
    AP_Float    _wp_radius_cm;      // waypoint radius in cm; waypoint is considered reached when within this distance
    AP_Float    _wp_accel_cmss;     // maximum horizontal acceleration in cm/s² used during waypoint tracking
    AP_Float    _wp_accel_c_cmss;   // maximum acceleration in cm/s² for turns; defaults to 2x horizontal accel if unset
    AP_Float    _wp_accel_z_cmss;   // maximum vertical acceleration in cm/s² used during climb or descent
    AP_Float    _wp_jerk_msss;      // maximum jerk in m/s³ used for s-curve trajectory shaping
    AP_Float    _terrain_margin_m;  // minimum altitude margin in meters when terrain following is active

    // WPNAV_SPEED param change checker
    bool _check_wp_speed_change;    // true if WPNAV_SPEED should be monitored for changes during flight
    float _last_wp_speed_cms;       // last recorded WPNAV_SPEED value (cm/s) for change detection
    float _last_wp_speed_up_cms;    // last recorded WPNAV_SPEED_UP value (cm/s)
    float _last_wp_speed_down_cms;  // last recorded WPNAV_SPEED_DN value (cm/s)

    // s-curve trajectory objects
    SCurve _scurve_prev_leg;        // s-curve for the previous waypoint leg, used for smoothing transitions
    SCurve _scurve_this_leg;        // s-curve for the current active waypoint leg
    SCurve _scurve_next_leg;        // s-curve for the next waypoint leg, used for lookahead blending
    float _scurve_jerk_max_msss;    // computed maximum jerk in m/s³ used for trajectory shaping
    float _scurve_snap_max_mssss;   // computed maximum snap in m/s⁴ derived from controller responsiveness

    // spline curves
    SplineCurve _spline_this_leg;   // spline curve for the current segment
    SplineCurve _spline_next_leg;   // spline curve for the next segment

    // path type flags
    bool _this_leg_is_spline;       // true if the current leg uses spline trajectory
    bool _next_leg_is_spline;       // true if the next leg will use spline trajectory

    // waypoint navigation state
    uint32_t _wp_last_update_ms;         // timestamp of the last update_wpnav() call (milliseconds)
    float _wp_desired_speed_ne_ms;       // desired horizontal speed in m/s for the current segment
    Vector3p _origin_ned_m;              // origin of the current leg in meters (NED frame)
    Vector3p _destination_ned_m;         // destination of the current leg in meters (NED frame)
    Vector3p _next_destination_ned_m;    // destination of the next leg in meters (NED frame)
    float _track_dt_scalar;              // scalar to reduce or increase the advancement along the track (0.0–1.0)
    float _offset_vel_ms;                // filtered horizontal speed target (used for terrain following or pause handling)
    float _offset_accel_mss;             // filtered horizontal acceleration target (used for terrain following or pause handling)
    bool _paused;                        // true if waypoint controller is paused

    // terrain following state
    bool _is_terrain_alt;               // true if altitude values are relative to terrain height, false if relative to EKF origin
    bool _rangefinder_available;        // true if a rangefinder is enabled and available for use
    AP_Int8 _rangefinder_use;           // parameter specifying whether rangefinder should be used for terrain tracking
    bool _rangefinder_healthy;          // true if the rangefinder reading is valid and within operational range
    float _rangefinder_terrain_u_m;     // rangefinder-derived terrain offset (meters above EKF origin)
};
