#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_InertialNav/AP_InertialNav.h>     // Inertial Navigation library
#include <AC_AttitudeControl/AC_PosControl.h>      // Position control library

// loiter maximum velocities and accelerations
#define AC_CIRCLE_RADIUS_DEFAULT    1000.0f     // radius of the circle in cm that the vehicle will fly
#define AC_CIRCLE_RATE_DEFAULT      20.0f       // turn rate in deg/sec.  Positive to turn clockwise, negative for counter clockwise
#define AC_CIRCLE_ANGULAR_ACCEL_MIN 2.0f        // angular acceleration should never be less than 2deg/sec
#define AC_CIRCLE_RADIUS_MAX        200000.0f   // maximum allowed circle radius of 2km

class AC_Circle
{
public:

    /// Constructor
    AC_Circle(const AP_InertialNav& inav, const AP_AHRS_View& ahrs, AC_PosControl& pos_control);

    /// init - initialise circle controller setting center specifically
    ///     set terrain_alt to true if center.z should be interpreted as an alt-above-terrain
    ///     caller should set the position controller's x,y and z speeds and accelerations before calling this
    void init(const Vector3f& center, bool terrain_alt);

    /// init - initialise circle controller setting center using stopping point and projecting out based on the copter's heading
    ///     caller should set the position controller's x,y and z speeds and accelerations before calling this
    void init();

    /// set circle center to a Location
    void set_center(const Location& center);

    /// set_circle_center as a vector from ekf origin
    ///     terrain_alt should be true if center.z is alt is above terrain
    void set_center(const Vector3f& center, bool terrain_alt) { _center = center; _terrain_alt = terrain_alt; }

    /// get_circle_center in cm from home
    const Vector3f& get_center() const { return _center; }

    /// returns true if using terrain altitudes
    bool center_is_terrain_alt() const { return _terrain_alt; }

    /// get_radius - returns radius of circle in cm
    float get_radius() const { return _radius; }

    /// set_radius - sets circle radius in cm
    void set_radius(float radius_cm);

    /// get_rate - returns target rate in deg/sec held in RATE parameter
    float get_rate() const { return _rate; }

    /// get_rate_current - returns actual calculated rate target in deg/sec, which may be less than _rate
    float get_rate_current() const { return ToDeg(_angular_vel); }

    /// set_rate - set circle rate in degrees per second
    void set_rate(float deg_per_sec);

    /// get_angle_total - return total angle in radians that vehicle has circled
    float get_angle_total() const { return _angle_total; }

    /// update - update circle controller
    ///     returns false on failure which indicates a terrain failsafe
    bool update() WARN_IF_UNUSED;

    /// get desired roll, pitch which should be fed into stabilize controllers
    float get_roll() const { return _pos_control.get_roll(); }
    float get_pitch() const { return _pos_control.get_pitch(); }
    float get_yaw() const { return _yaw; }

    /// returns true if update has been run recently
    /// used by vehicle code to determine if get_yaw() is valid
    bool is_active() const;

    // get_closest_point_on_circle - returns closest point on the circle
    //  circle's center should already have been set
    //  closest point on the circle will be placed in result
    //  result's altitude (i.e. z) will be set to the circle_center's altitude
    //  if vehicle is at the center of the circle, the edge directly behind vehicle will be returned
    void get_closest_point_on_circle(Vector3f &result) const;

    /// get horizontal distance to loiter target in cm
    float get_distance_to_target() const { return _pos_control.get_distance_to_target(); }

    /// get bearing to target in centi-degrees
    int32_t get_bearing_to_target() const { return _pos_control.get_bearing_to_target(); }

    /// true if pilot control of radius and turn rate is enabled
    bool pilot_control_enabled() const { return (_options.get() & CircleOptions::MANUAL_CONTROL) != 0; }

    /// provide rangefinder altitude
    void set_rangefinder_alt(bool use, bool healthy, float alt_cm) { _rangefinder_available = use; _rangefinder_healthy = healthy; _rangefinder_alt_cm = alt_cm; }

    static const struct AP_Param::GroupInfo var_info[];

private:

    // calc_velocities - calculate angular velocity max and acceleration based on radius and rate
    //      this should be called whenever the radius or rate are changed
    //      initialises the yaw and current position around the circle
    //      init_velocity should be set true if vehicle is just starting circle
    void calc_velocities(bool init_velocity);

    // init_start_angle - sets the starting angle around the circle and initialises the angle_total
    //  if use_heading is true the vehicle's heading will be used to init the angle causing minimum yaw movement
    //  if use_heading is false the vehicle's position from the center will be used to initialise the angle
    void init_start_angle(bool use_heading);

    // get expected source of terrain data
    enum class TerrainSource {
        TERRAIN_UNAVAILABLE,
        TERRAIN_FROM_RANGEFINDER,
        TERRAIN_FROM_TERRAINDATABASE,
    };
    AC_Circle::TerrainSource get_terrain_source() const;

    // get terrain's altitude (in cm above the ekf origin) at the current position (+ve means terrain below vehicle is above ekf origin's altitude)
    bool get_terrain_offset(float& offset_cm);

    // flags structure
    struct circle_flags {
        uint8_t panorama    : 1;    // true if we are doing a panorama
    } _flags;

    // references to inertial nav and ahrs libraries
    const AP_InertialNav&       _inav;
    const AP_AHRS_View&         _ahrs;
    AC_PosControl&              _pos_control;

    enum CircleOptions {
        MANUAL_CONTROL           = 1U << 0,
        FACE_DIRECTION_OF_TRAVEL = 1U << 1,
        INIT_AT_CENTER           = 1U << 2, // true then the circle center will be the current location, false and the center will be 1 radius ahead
    };

    // parameters
    AP_Float    _radius;        // maximum horizontal speed in cm/s during missions
    AP_Float    _rate;          // rotation speed in deg/sec
    AP_Int16    _options;       // stick control enable/disable

    // internal variables
    Vector3f    _center;        // center of circle in cm from home
    float       _yaw;           // yaw heading (normally towards circle center)
    float       _angle;         // current angular position around circle in radians (0=directly north of the center of the circle)
    float       _angle_total;   // total angle traveled in radians
    float       _angular_vel;   // angular velocity in radians/sec
    float       _angular_vel_max;   // maximum velocity in radians/sec
    float       _angular_accel; // angular acceleration in radians/sec/sec
    uint32_t    _last_update_ms;    // system time of last update

    // terrain following variables
    bool        _terrain_alt;           // true if _center.z is alt-above-terrain, false if alt-above-ekf-origin
    bool        _rangefinder_available; // true if range finder could be used
    bool        _rangefinder_healthy;   // true if range finder is healthy
    float       _rangefinder_alt_cm;    // latest rangefinder altitude
};
