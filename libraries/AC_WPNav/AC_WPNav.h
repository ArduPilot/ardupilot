#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/SCurve.h>
#include <AP_Math/SplineCurve.h>
#include <AP_Common/Location.h>
#include <AP_InertialNav/AP_InertialNav.h>     // Inertial Navigation library
#include <AC_AttitudeControl/AC_PosControl.h>      // Position control library
#include <AC_AttitudeControl/AC_AttitudeControl.h> // Attitude control library
#include <AP_Terrain/AP_Terrain.h>
#include <AC_Avoidance/AC_Avoid.h>                 // Stop at fence library

// maximum velocities and accelerations
#define WPNAV_ACCELERATION              250.0f      // maximum horizontal acceleration in cm/s/s that wp navigation will request
#define WPNAV_WP_SPEED                 1000.0f      // default horizontal speed between waypoints in cm/s
#define WPNAV_WP_SPEED_MIN               20.0f      // minimum horizontal speed between waypoints in cm/s
#define WPNAV_WP_RADIUS                 200.0f      // default waypoint radius in cm
#define WPNAV_WP_RADIUS_MIN               5.0f      // minimum waypoint radius in cm

#define WPNAV_WP_SPEED_UP               250.0f      // default maximum climb velocity
#define WPNAV_WP_SPEED_DOWN             150.0f      // default maximum descent velocity

#define WPNAV_WP_ACCEL_Z_DEFAULT        100.0f      // default vertical acceleration between waypoints in cm/s/s

class AC_WPNav
{
public:

    /// Constructor
    AC_WPNav(const AP_InertialNav& inav, const AP_AHRS_View& ahrs, AC_PosControl& pos_control, const AC_AttitudeControl& attitude_control);

    /// provide pointer to terrain database
    void set_terrain(AP_Terrain* terrain_ptr) { _terrain = terrain_ptr; }

    /// provide rangefinder altitude
    void set_rangefinder_alt(bool use, bool healthy, float alt_cm) { _rangefinder_available = use; _rangefinder_healthy = healthy; _rangefinder_alt_cm = alt_cm; }

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
    bool get_terrain_offset(float& offset_cm);

    // return terrain following altitude margin.  vehicle will stop if distance from target altitude is larger than this margin
    float get_terrain_margin() const { return MAX(_terrain_margin, 0.1); }

    // convert location to vector from ekf origin.  terrain_alt is set to true if resulting vector's z-axis should be treated as alt-above-terrain
    //      returns false if conversion failed (likely because terrain data was not available)
    bool get_vector_NEU(const Location &loc, Vector3f &vec, bool &terrain_alt);

    ///
    /// waypoint controller
    ///

    /// wp_and_spline_init - initialise straight line and spline waypoint controllers
    ///     speed_cms is the desired max speed to travel between waypoints.  should be a positive value or omitted to use the default speed
    ///     updates target roll, pitch targets and I terms based on vehicle lean angles
    ///     should be called once before the waypoint controller is used but does not need to be called before subsequent updates to destination
    void wp_and_spline_init(float speed_cms = 0.0f);

    /// set current target horizontal speed during wp navigation
    void set_speed_xy(float speed_cms);

    /// set current target climb or descent rate during wp navigation
    void set_speed_up(float speed_up_cms);
    void set_speed_down(float speed_down_cms);

    /// get default target horizontal velocity during wp navigation
    float get_default_speed_xy() const { return _wp_speed_cms; }

    /// get default target climb speed in cm/s during missions
    float get_default_speed_up() const { return _wp_speed_up_cms; }

    /// get default target descent rate in cm/s during missions.  Note: always positive
    float get_default_speed_down() const { return _wp_speed_down_cms; }

    /// get_speed_z - returns target descent speed in cm/s during missions.  Note: always positive
    float get_accel_z() const { return _wp_accel_z_cmss; }

    /// get_wp_acceleration - returns acceleration in cm/s/s during missions
    float get_wp_acceleration() const { return _wp_accel_cmss.get(); }

    /// get_wp_destination waypoint using position vector
    /// x,y are distance from ekf origin in cm
    /// z may be cm above ekf origin or terrain (see origin_and_destination_are_terrain_alt method)
    const Vector3f &get_wp_destination() const { return _destination; }

    /// get origin using position vector (distance from ekf origin in cm)
    const Vector3f &get_wp_origin() const { return _origin; }

    /// true if origin.z and destination.z are alt-above-terrain, false if alt-above-ekf-origin
    bool origin_and_destination_are_terrain_alt() const { return _terrain_alt; }

    /// set_wp_destination waypoint using location class
    ///     provide the next_destination if known
    ///     returns false if conversion from location to vector from ekf origin cannot be calculated
    bool set_wp_destination_loc(const Location& destination);
    bool set_wp_destination_next_loc(const Location& destination);

    // get destination as a location.  Altitude frame will be absolute (AMSL) or above terrain
    // returns false if unable to return a destination (for example if origin has not yet been set)
    bool get_wp_destination_loc(Location& destination) const;

    // returns object avoidance adjusted destination which is always the same as get_wp_destination
    // having this function unifies the AC_WPNav_OA and AC_WPNav interfaces making vehicle code simpler
    virtual bool get_oa_wp_destination(Location& destination) const { return get_wp_destination_loc(destination); }

    /// set_wp_destination waypoint using position vector (distance from ekf origin in cm)
    ///     terrain_alt should be true if destination.z is a desired altitude above terrain
    virtual bool set_wp_destination(const Vector3f& destination, bool terrain_alt = false);
    bool set_wp_destination_next(const Vector3f& destination, bool terrain_alt = false);

    /// set waypoint destination using NED position vector from ekf origin in meters
    ///     provide next_destination_NED if known
    bool set_wp_destination_NED(const Vector3f& destination_NED);
    bool set_wp_destination_next_NED(const Vector3f& destination_NED);

    /// shifts the origin and destination horizontally to the current position
    ///     used to reset the track when taking off without horizontal position control
    ///     relies on set_wp_destination or set_wp_origin_and_destination having been called first
    void shift_wp_origin_and_destination_to_current_pos_xy();

    /// shifts the origin and destination horizontally to the achievable stopping point
    ///     used to reset the track when horizontal navigation is enabled after having been disabled (see Copter's wp_navalt_min)
    ///     relies on set_wp_destination or set_wp_origin_and_destination having been called first
    void shift_wp_origin_and_destination_to_stopping_point_xy();

    /// get_wp_stopping_point_xy - calculates stopping point based on current position, velocity, waypoint acceleration
    ///		results placed in stopping_position vector
    void get_wp_stopping_point_xy(Vector2f& stopping_point) const;
    void get_wp_stopping_point(Vector3f& stopping_point) const;

    /// get_wp_distance_to_destination - get horizontal distance to destination in cm
    virtual float get_wp_distance_to_destination() const;

    /// get_bearing_to_destination - get bearing to next waypoint in centi-degrees
    virtual int32_t get_wp_bearing_to_destination() const;

    /// reached_destination - true when we have come within RADIUS cm of the waypoint
    virtual bool reached_wp_destination() const { return _flags.reached_destination; }

    // reached_wp_destination_xy - true if within RADIUS_CM of waypoint in x/y
    bool reached_wp_destination_xy() const {
        return get_wp_distance_to_destination() < _wp_radius_cm;
    }

    /// update_wpnav - run the wp controller - should be called at 100hz or higher
    virtual bool update_wpnav();

    // returns true if update_wpnav has been run very recently
    bool is_active() const;

    ///
    /// spline methods
    ///

    /// set_spline_destination waypoint using location class
    ///     returns false if conversion from location to vector from ekf origin cannot be calculated
    ///     next_destination should be the next segment's destination
    ///     next_is_spline should be true if next_destination is a spline segment
    bool set_spline_destination_loc(const Location& destination, const Location& next_destination, bool next_is_spline);

    /// set next destination (e.g. the one after the current destination) as a spline segment specified as a location
    ///     returns false if conversion from location to vector from ekf origin cannot be calculated
    ///     next_next_destination should be the next segment's destination
    ///     next_next_is_spline should be true if next_next_destination is a spline segment
    bool set_spline_destination_next_loc(const Location& next_destination, const Location& next_next_destination, bool next_next_is_spline);

    /// set_spline_destination waypoint using position vector (distance from ekf origin in cm)
    ///     terrain_alt should be true if destination.z is a desired altitude above terrain (false if its desired altitudes above ekf origin)
    ///     next_destination is the next segment's destination
    ///     next_terrain_alt should be true if next_destination.z is a desired altitude above terrain (false if its desired altitudes above ekf origin)
    ///     next_destination.z must be in the same "frame" as destination.z (i.e. if destination is a alt-above-terrain, next_destination must be too)
    ///     next_is_spline should be true if next_destination is a spline segment
    bool set_spline_destination(const Vector3f& destination, bool terrain_alt, const Vector3f& next_destination, bool next_terrain_alt, bool next_is_spline);

    /// set next destination (e.g. the one after the current destination) as an offset (in cm, NEU frame) from the EKF origin
    ///     next_terrain_alt should be true if next_destination.z is a desired altitude above terrain (false if its desired altitudes above ekf origin)
    ///     next_next_destination is the next segment's destination
    ///     next_next_terrain_alt should be true if next_next_destination.z is a desired altitude above terrain (false if it is desired altitude above ekf origin)
    ///     next_next_destination.z must be in the same "frame" as destination.z (i.e. if next_destination is a alt-above-terrain, next_next_destination must be too)
    ///     next_next_is_spline should be true if next_next_destination is a spline segment
    bool set_spline_destination_next(const Vector3f& next_destination, bool next_terrain_alt, const Vector3f& next_next_destination, bool next_next_terrain_alt, bool next_next_is_spline);

    ///
    /// shared methods
    ///

    /// get desired roll, pitch which should be fed into stabilize controllers
    float get_roll() const { return _pos_control.get_roll_cd(); }
    float get_pitch() const { return _pos_control.get_pitch_cd(); }
    Vector3f get_thrust_vector() const { return _pos_control.get_thrust_vector(); }

    // get target yaw in centi-degrees
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
    // updates _scurve_jerk and _scurve_jerk_time
    void calc_scurve_jerk_and_jerk_time();

    // references and pointers to external libraries
    const AP_InertialNav&   _inav;
    const AP_AHRS_View&     _ahrs;
    AC_PosControl&          _pos_control;
    const AC_AttitudeControl& _attitude_control;
    AP_Terrain              *_terrain;

    // parameters
    AP_Float    _wp_speed_cms;          // default maximum horizontal speed in cm/s during missions
    AP_Float    _wp_speed_up_cms;       // default maximum climb rate in cm/s
    AP_Float    _wp_speed_down_cms;     // default maximum descent rate in cm/s
    AP_Float    _wp_radius_cm;          // distance from a waypoint in cm that, when crossed, indicates the wp has been reached
    AP_Float    _wp_accel_cmss;         // horizontal acceleration in cm/s/s during missions
    AP_Float    _wp_accel_z_cmss;       // vertical acceleration in cm/s/s during missions
    AP_Float    _wp_jerk;               // maximum jerk used to generate scurve trajectories in m/s/s/s
    AP_Float    _terrain_margin;        // terrain following altitude margin. vehicle will stop if distance from target altitude is larger than this margin

    // scurve
    SCurve _scurve_prev_leg;            // previous scurve trajectory used to blend with current scurve trajectory
    SCurve _scurve_this_leg;            // current scurve trajectory
    SCurve _scurve_next_leg;            // next scurve trajectory used to blend with current scurve trajectory
    float _scurve_jerk;                 // scurve jerk max in m/s/s/s
    float _scurve_jerk_time;            // scurve jerk time (time in seconds for jerk to increase from zero _scurve_jerk)

    // spline curves
    SplineCurve _spline_this_leg;      // spline curve for current segment
    SplineCurve _spline_next_leg;      // spline curve for next segment

    // the type of this leg
    bool _this_leg_is_spline;           // true if this leg is a spline
    bool _next_leg_is_spline;           // true if the next leg is a spline

    // waypoint controller internal variables
    uint32_t    _wp_last_update;        // time of last update_wpnav call
    float       _wp_desired_speed_xy_cms;   // desired wp speed in cm/sec
    Vector3f    _origin;                // starting point of trip to next waypoint in cm from ekf origin
    Vector3f    _destination;           // target destination in cm from ekf origin
    float       _track_scalar_dt;       // time compression multiplier to slow the progress along the track
    float       _terain_vel;            // maximum horizontal velocity used to ensure the aircraft can maintain height above terain
    float       _terain_accel;          // acceleration value used to change _terain_vel

    // terrain following variables
    bool        _terrain_alt;   // true if origin and destination.z are alt-above-terrain, false if alt-above-ekf-origin
    bool        _rangefinder_available; // true if rangefinder is enabled (user switch can turn this true/false)
    AP_Int8     _rangefinder_use;       // parameter that specifies if the range finder should be used for terrain following commands
    bool        _rangefinder_healthy;   // true if rangefinder distance is healthy (i.e. between min and maximum)
    float       _rangefinder_alt_cm;    // latest distance from the rangefinder
};
