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
#define WPNAV_ACCELERATION_MS           2.5        // maximum horizontal acceleration in cm/s/s that wp navigation will request

class AC_WPNav
{
public:

    /// Constructor
    AC_WPNav(const AP_AHRS_View& ahrs, AC_PosControl& pos_control, const AC_AttitudeControl& attitude_control);

    /// provide rangefinder based terrain offset
    /// terrain offset is the terrain's height above the EKF origin
    void set_rangefinder_terrain_offset_cm(bool use, bool healthy, float terrain_offset_cm) { _rangefinder_available = use; _rangefinder_healthy = healthy; _rangefinder_terrain_offset_m = terrain_offset_cm * 0.01;}
    void set_rangefinder_terrain_offset_m(bool use, bool healthy, float terrain_offset_m) { _rangefinder_available = use; _rangefinder_healthy = healthy; _rangefinder_terrain_offset_m = terrain_offset_m;}

    // return true if range finder may be used for terrain following
    bool rangefinder_used() const { return _rangefinder_use; }
    bool rangefinder_used_and_healthy() const { return _rangefinder_use && _rangefinder_healthy; }

    // get expected source of terrain data if alt-above-terrain command is executed (used by Copter's ModeRTL)
    enum class TerrainSource {
        TERRAIN_UNAVAILABLE,
        TERRAIN_FROM_RANGEFINDER,
        TERRAIN_FROM_TERRAINDATABASE,
    };
    AC_WPNav::TerrainSource get_terrain_source() const;

    // get terrain's altitude (in cm above the ekf origin) at the current position (+ve means terrain below vehicle is above ekf origin's altitude)
    bool get_terrain_offset_cm(float& offset_cm);
    bool get_terrain_offset_m(float& offset_m);

    // return terrain following altitude margin.  vehicle will stop if distance from target altitude is larger than this margin
    float get_terrain_margin_m() const { return MAX(_terrain_margin_m, 0.1); }

    // convert location to vector from ekf origin.  is_terrain_alt is set to true if resulting vector's z-axis should be treated as alt-above-terrain
    //      returns false if conversion failed (likely because terrain data was not available)
    bool get_vector_NEU_cm(const Location &loc, Vector3f &pos_from_origin_NEU_cm, bool &is_terrain_alt);
    bool get_vector_NEU_m(const Location &loc, Vector3f &pos_from_origin_NEU_m, bool &is_terrain_alt);

    ///
    /// waypoint controller
    ///

    /// wp_and_spline_init_cm - initialise straight line and spline waypoint controllers
    ///     speed_cms is the desired max speed to travel between waypoints.  should be a positive value or omitted to use the default speed
    ///     updates target roll, pitch targets and I terms based on vehicle lean angles
    ///     should be called once before the waypoint controller is used but does not need to be called before subsequent updates to destination
    void wp_and_spline_init_cm(float speed_cms = 0.0f, Vector3f stopping_point = Vector3f{});
    void wp_and_spline_init_m(float speed_ms = 0.0f, Vector3f stopping_point = Vector3f{});

    /// set current target horizontal speed during wp navigation
    void set_speed_NE_cms(float speed_cms);
    void set_speed_NE_ms(float speed_ms);

    /// set pause or resume during wp navigation
    void set_pause() { _paused = true; }
    void set_resume() { _paused = false; }

    /// get paused status
    bool paused() { return _paused; }

    /// set current target climb or descent rate during wp navigation
    void set_speed_up_cms(float speed_up_cms);
    void set_speed_up_ms(float speed_up_ms);
    void set_speed_down_cms(float speed_down_cms);
    void set_speed_down_ms(float speed_down_ms);

    /// get default target horizontal velocity during wp navigation
    float get_default_speed_NE_cms() const { return get_default_speed_NE_ms() * 100.0; }
    float get_default_speed_NE_ms() const { return _wp_speed_cms * 0.01; }

    /// get default target climb speed in cm/s during missions
    float get_default_speed_up_cms() const { return get_default_speed_up_ms() * 100.0; }
    float get_default_speed_up_ms() const { return _wp_speed_up_cms * 0.01; }

    /// get default target descent rate in cm/s during missions.  Note: always positive
    float get_default_speed_down_cms() const { return get_default_speed_down_ms() * 100.0; }
    float get_default_speed_down_ms() const { return fabsf(_wp_speed_down_cms * 0.01); }

    /// get_accel_U_cmss - returns vertical acceleration in cm/s/s during missions.  Note: always positive
    float get_accel_U_cmss() const { return get_accel_U_mss() * 100.0; }
    float get_accel_U_mss() const { return _wp_accel_z_cmss * 0.01; }

    /// get_wp_acceleration - returns acceleration in cm/s/s during missions
    float get_wp_acceleration_cmss() const { return get_wp_acceleration_mss() * 100.0; }
    float get_wp_acceleration_mss() const { return (is_positive(_wp_accel_cmss)) ? _wp_accel_cmss * 0.01 : WPNAV_ACCELERATION_MS; }

    /// get_corner_acceleration_mss - returns maximum acceleration in m/s/s used during cornering in missions
    float get_corner_acceleration_mss() const { return (is_positive(_wp_accel_c_cmss)) ? _wp_accel_c_cmss * 0.01 : 2.0 * get_wp_acceleration_mss(); }

    /// get_wp_destination_NEU_cm waypoint using position vector
    /// x,y are distance from ekf origin in cm
    /// z may be cm above ekf origin or terrain (see origin_and_destination_are_is_terrain_alt method)
    const Vector3f get_wp_destination_NEU_cm() const { return get_wp_destination_NEU_m() * 100.0; }
    const Vector3f &get_wp_destination_NEU_m() const { return _destination_neu_m; }

    /// get origin using position vector (distance from ekf origin in cm)
    const Vector3f get_wp_origin_NEU_cm() const { return get_wp_origin_NEU_m() * 100.0; }
    const Vector3f &get_wp_origin_NEU_m() const { return _origin_neu_m; }

    /// true if origin.z and destination.z are alt-above-terrain, false if alt-above-ekf-origin
    bool origin_and_destination_are_terrain_alt() const { return _is_terrain_alt; }

    /// set_wp_destination_NEU_cm waypoint using location class
    ///     provide the next_destination if known
    ///     returns false if conversion from location to vector from ekf origin cannot be calculated
    bool set_wp_destination_loc(const Location& destination);
    bool set_wp_destination_next_loc(const Location& destination);

    // get destination as a location.  Altitude frame will be absolute (AMSL) or above terrain
    // returns false if unable to return a destination (for example if origin has not yet been set)
    bool get_wp_destination_loc(Location& destination) const;

    // returns object avoidance adjusted destination which is always the same as get_wp_destination_NEU_cm
    // having this function unifies the AC_WPNav_OA and AC_WPNav interfaces making vehicle code simpler
    virtual bool get_oa_wp_destination(Location& destination) const { return get_wp_destination_loc(destination); }

    /// set_wp_destination_NEU_cm waypoint using position vector (distance from ekf origin in cm)
    ///     is_terrain_alt should be true if destination.z is a desired altitude above terrain
    virtual bool set_wp_destination_NEU_cm(const Vector3f& destination_neu_cm, bool is_terrain_alt = false);
    virtual bool set_wp_destination_NEU_m(const Vector3f& destination_neu_m, bool is_terrain_alt = false);
    bool set_wp_destination_next_NEU_cm(const Vector3f& destination_neu_cm, bool is_terrain_alt = false);
    bool set_wp_destination_next_NEU_m(const Vector3f& destination_neu_m, bool is_terrain_alt = false);

    /// set waypoint destination using NED position vector from ekf origin in meters
    ///     provide next_destination_NED if known
    bool set_wp_destination_NED_m(const Vector3f& destination_NED_m);
    bool set_wp_destination_next_NED_m(const Vector3f& destination_NED_m);

    /// get_wp_stopping_point_cm - calculates stopping point based on current position, velocity, waypoint acceleration
    ///		results placed in stopping_position vector
    void get_wp_stopping_point_NE_cm(Vector2f& stopping_point_ne_cm) const;
    void get_wp_stopping_point_NE_m(Vector2f& stopping_point_ne_m) const;
    void get_wp_stopping_point_NEU_cm(Vector3f& stopping_point_neu_cm) const;
    void get_wp_stopping_point_NEU_m(Vector3f& stopping_point_neu_m) const;

    /// get_wp_distance_to_destination - get horizontal distance to destination in cm
    virtual float get_wp_distance_to_destination_cm() const;
    virtual float get_wp_distance_to_destination_m() const;

    /// get_bearing_to_destination - get bearing to next waypoint in centi-degrees
    virtual int32_t get_wp_bearing_to_destination_cd() const;

    /// get_bearing_to_destination - get bearing to next waypoint in radians
    virtual float get_wp_bearing_to_destination_rad() const;

    /// reached_destination - true when we have come within RADIUS cm of the waypoint
    virtual bool reached_wp_destination() const { return _flags.reached_destination; }

    // reached_wp_destination_NE - true if within RADIUS_CM of waypoint in x/y
    bool reached_wp_destination_NE() const {
        return get_wp_distance_to_destination_m() < _wp_radius_cm * 0.01;
    }

    // get wp_radius parameter value in cm
    float get_wp_radius_cm() const { return get_wp_radius_m() * 100.0; }
    float get_wp_radius_m() const { return _wp_radius_cm * 0.01; }

    /// update_wpnav - run the wp controller - should be called at 100hz or higher
    virtual bool update_wpnav();

    // returns true if update_wpnav has been run very recently
    bool is_active() const;

    // force stopping at next waypoint.  Used by Dijkstra's object avoidance when path from destination to next destination is not clear
    // only affects regular (e.g. non-spline) waypoints
    // returns true if this had any affect on the path
    bool force_stop_at_next_wp();

    ///
    /// spline methods
    ///

    /// set_spline_destination_NEU_cm waypoint using location class
    ///     returns false if conversion from location to vector from ekf origin cannot be calculated
    ///     next_destination should be the next segment's destination
    ///     next_is_spline should be true if next_destination is a spline segment
    bool set_spline_destination_loc(const Location& destination, const Location& next_destination, bool next_is_spline);

    /// set next destination (e.g. the one after the current destination) as a spline segment specified as a location
    ///     returns false if conversion from location to vector from ekf origin cannot be calculated
    ///     next_next_destination should be the next segment's destination
    ///     next_next_is_spline should be true if next_next_destination is a spline segment
    bool set_spline_destination_next_loc(const Location& next_destination, const Location& next_next_destination, bool next_next_is_spline);

    /// set_spline_destination_NEU_cm waypoint using position vector (distance from ekf origin in cm)
    ///     is_terrain_alt should be true if destination.z is a desired altitude above terrain (false if its desired altitudes above ekf origin)
    ///     next_destination is the next segment's destination
    ///     next_is_terrain_alt should be true if next_destination.z is a desired altitude above terrain (false if its desired altitudes above ekf origin)
    ///     next_destination.z must be in the same "frame" as destination.z (i.e. if destination is a alt-above-terrain, next_destination must be too)
    ///     next_is_spline should be true if next_destination is a spline segment
    bool set_spline_destination_NEU_cm(const Vector3f& destination_neu_cm, bool is_terrain_alt, const Vector3f& next_destination_neu_cm, bool next_terrain_alt, bool next_is_spline);
    bool set_spline_destination_NEU_m(const Vector3f& destination_neu_m, bool is_terrain_alt, const Vector3f& next_destination_neu_m, bool next_terrain_alt, bool next_is_spline);

    /// set next destination (e.g. the one after the current destination) as an offset (in cm, NEU frame) from the EKF origin
    ///     next_is_terrain_alt should be true if next_destination.z is a desired altitude above terrain (false if its desired altitudes above ekf origin)
    ///     next_next_destination is the next segment's destination
    ///     next_next_is_terrain_alt should be true if next_next_destination.z is a desired altitude above terrain (false if it is desired altitude above ekf origin)
    ///     next_next_destination.z must be in the same "frame" as destination.z (i.e. if next_destination is a alt-above-terrain, next_next_destination must be too)
    ///     next_next_is_spline should be true if next_next_destination is a spline segment
    bool set_spline_destination_next_NEU_cm(const Vector3f& next_destination_neu_cm, bool next_is_terrain_alt, const Vector3f& next_next_destination_neu_cm, bool next_next_is_terrain_alt, bool next_next_is_spline);
    bool set_spline_destination_next_NEU_m(const Vector3f& next_destination_neu_m, bool next_is_terrain_alt, const Vector3f& next_next_destination_neu_m, bool next_next_is_terrain_alt, bool next_next_is_spline);

    ///
    /// shared methods
    ///

    /// Returns the desired roll angle in radians from the position controller.
    float get_roll_rad() const { return _pos_control.get_roll_rad(); }

    /// Returns the desired pitch angle in radians from the position controller.
    float get_pitch_rad() const { return _pos_control.get_pitch_rad(); }

    /// Returns the desired yaw target in radians from the position controller.
    float get_yaw_rad() const { return _pos_control.get_yaw_rad(); }

    /// Returns the desired thrust direction vector for tilt control from the position controller.
    Vector3f get_thrust_vector() const { return _pos_control.get_thrust_vector(); }

    /// Returns the desired roll angle in centidegrees from the position controller.
    float get_roll() const { return _pos_control.get_roll_cd(); }

    /// Returns the desired pitch angle in centidegrees from the position controller.
    float get_pitch() const { return _pos_control.get_pitch_cd(); }

    /// Returns the desired yaw target in centidegrees from the position controller.
    float get_yaw() const { return _pos_control.get_yaw_cd(); }

    /// advance_wp_target_along_track - move target location along track from origin to destination
    bool advance_wp_target_along_track(float dt);

    /// recalculate path with update speed and/or acceleration limits
    void update_track_with_speed_accel_limits();

    /// return the crosstrack_error - horizontal error of the actual position vs the desired position
    float crosstrack_error() const { return _pos_control.crosstrack_error();}

    static const struct AP_Param::GroupInfo var_info[];

protected:

    // flags structure
    struct wpnav_flags {
        uint8_t reached_destination     : 1;    // true if we have reached the destination
        uint8_t fast_waypoint           : 1;    // true if we should ignore the waypoint radius and consider the waypoint complete once the intermediate target has reached the waypoint
        uint8_t wp_yaw_set              : 1;    // true if yaw target has been set
    } _flags;

    // helper function to calculate scurve jerk and jerk_time values
    // updates _scurve_jerk_max_msss and _scurve_snap_max_mssss
    void calc_scurve_jerk_and_snap();

    // references and pointers to external libraries
    const AP_AHRS_View&     _ahrs;
    AC_PosControl&          _pos_control;
    const AC_AttitudeControl& _attitude_control;

    // parameters
    AP_Float    _wp_speed_cms;       // default horizontal speed in cm/s for waypoint navigation
    AP_Float    _wp_speed_up_cms;    // default climb rate in cm/s for waypoint navigation
    AP_Float    _wp_speed_down_cms;  // default descent rate in cm/s for waypoint navigation
    AP_Float    _wp_radius_cm;       // waypoint radius in cm; waypoint is considered reached when within this distance
    AP_Float    _wp_accel_cmss;      // maximum horizontal acceleration in cm/s² used during waypoint tracking
    AP_Float    _wp_accel_c_cmss;    // maximum acceleration in cm/s² for turns; defaults to 2x horizontal accel if unset
    AP_Float    _wp_accel_z_cmss;    // maximum vertical acceleration in cm/s² used during climb or descent
    AP_Float    _wp_jerk_msss;       // maximum jerk in m/s³ used for s-curve trajectory shaping
    AP_Float    _terrain_margin_m;   // minimum altitude margin in meters when terrain following is active

    // WPNAV_SPEED param change checker
    bool _check_wp_speed_change;     // true if WPNAV_SPEED should be monitored for changes during flight
    float _last_wp_speed_cms;        // last recorded WPNAV_SPEED value (cm/s) for change detection
    float _last_wp_speed_up_cms;     // last recorded WPNAV_SPEED_UP value (cm/s)
    float _last_wp_speed_down_cms;   // last recorded WPNAV_SPEED_DN value (cm/s)

    // s-curve trajectory objects
    SCurve _scurve_prev_leg;         // s-curve for the previous waypoint leg, used for smoothing transitions
    SCurve _scurve_this_leg;         // s-curve for the current active waypoint leg
    SCurve _scurve_next_leg;         // s-curve for the next waypoint leg, used for lookahead blending
    float _scurve_jerk_max_msss;     // computed maximum jerk in m/s³ used for trajectory shaping
    float _scurve_snap_max_mssss;    // computed maximum snap in m/s⁴ derived from controller responsiveness

    // spline curves
    SplineCurve _spline_this_leg;    // spline curve for the current segment
    SplineCurve _spline_next_leg;    // spline curve for the next segment

    // path type flags
    bool _this_leg_is_spline;        // true if the current leg uses spline trajectory
    bool _next_leg_is_spline;        // true if the next leg will use spline trajectory

    // waypoint navigation state
    uint32_t _wp_last_update_ms;         // timestamp of the last update_wpnav() call (milliseconds)
    float _wp_desired_speed_ne_ms;       // desired horizontal speed in m/s for the current segment
    Vector3f _origin_neu_m;              // origin of the current leg in meters (NEU frame)
    Vector3f _destination_neu_m;         // destination of the current leg in meters (NEU frame)
    Vector3f _next_destination_neu_m;    // destination of the next leg in meters (NEU frame)
    float _track_dt_scalar;              // scalar to reduce or increase the advancement along the track (0.0–1.0)
    float _offset_vel_ms;                // filtered horizontal speed target (used for terrain following or pause handling)
    float _offset_accel_mss;             // filtered horizontal acceleration target (used for terrain following or pause handling)
    bool _paused;                        // true if waypoint controller is paused

    // terrain following state
    bool _is_terrain_alt;                // true if altitude values are relative to terrain height, false if relative to EKF origin
    bool _rangefinder_available;         // true if a rangefinder is enabled and available for use
    AP_Int8 _rangefinder_use;            // parameter specifying whether rangefinder should be used for terrain tracking
    bool _rangefinder_healthy;           // true if the rangefinder reading is valid and within operational range
    float _rangefinder_terrain_offset_m; // rangefinder-derived terrain offset (meters above EKF origin)
};
