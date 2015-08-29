/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AC_CIRCLE_H
#define AC_CIRCLE_H

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_InertialNav/AP_InertialNav.h>     // Inertial Navigation library
#include <AC_AttitudeControl/AC_PosControl.h>      // Position control library

// loiter maximum velocities and accelerations
#define AC_CIRCLE_RADIUS_DEFAULT    1000.0f     // radius of the circle in cm that the vehicle will fly
#define AC_CIRCLE_RATE_DEFAULT      20.0f       // turn rate in deg/sec.  Positive to turn clockwise, negative for counter clockwise
#define AC_CIRCLE_ANGULAR_ACCEL_MIN 2.0f        // angular acceleration should never be less than 2deg/sec

#define AC_CIRCLE_DEGX100           5729.57795f // constant to convert from radians to centi-degrees

class AC_Circle
{
public:

    /// Constructor
    AC_Circle(const AP_InertialNav& inav, const AP_AHRS& ahrs, AC_PosControl& pos_control);

    /// init - initialise circle controller setting center specifically
    ///     caller should set the position controller's x,y and z speeds and accelerations before calling this
    void init(const Vector3f& center);

    /// init - initialise circle controller setting center using stopping point and projecting out based on the copter's heading
    ///     caller should set the position controller's x,y and z speeds and accelerations before calling this
    void init();

    /// set_circle_center in cm from home
    void set_center(const Vector3f& center) { _center = center; }

    /// get_circle_center in cm from home
    const Vector3f& get_center() const { return _center; }

    /// get_radius - returns radius of circle in cm
    float get_radius() { return _radius; }
    /// set_radius - sets circle radius in cm
    void set_radius(float radius_cm) { _radius = radius_cm; }

    /// set_circle_rate - set circle rate in degrees per second
    void set_rate(float deg_per_sec);

    /// get_angle_total - return total angle in radians that vehicle has circled
    float get_angle_total() const { return _angle_total; }

    /// update - update circle controller
    void update();

    /// get desired roll, pitch which should be fed into stabilize controllers
    int32_t get_roll() const { return _pos_control.get_roll(); }
    int32_t get_pitch() const { return _pos_control.get_pitch(); }
    int32_t get_yaw() const { return _yaw; }

    // get_closest_point_on_circle - returns closest point on the circle
    //  circle's center should already have been set
    //  closest point on the circle will be placed in result
    //  result's altitude (i.e. z) will be set to the circle_center's altitude
    //  if vehicle is at the center of the circle, the edge directly behind vehicle will be returned
    void get_closest_point_on_circle(Vector3f &result);

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

    // flags structure
    struct circle_flags {
        uint8_t panorama    : 1;    // true if we are doing a panorama
    } _flags;

    // references to inertial nav and ahrs libraries
    const AP_InertialNav&       _inav;
    const AP_AHRS&              _ahrs;
    AC_PosControl&              _pos_control;

    // parameters
    AP_Float    _radius;        // maximum horizontal speed in cm/s during missions
    AP_Float    _rate;          // rotation speed in deg/sec

    // internal variables
    uint32_t    _last_update;   // time of last update_loiter call
    Vector3f    _center;        // center of circle in cm from home
    float       _yaw;           // yaw heading (normally towards circle center)
    float       _angle;         // current angular position around circle in radians (0=directly north of the center of the circle)
    float       _angle_total;   // total angle travelled in radians
    float       _angular_vel;   // angular velocity in radians/sec
    float       _angular_vel_max;   // maximum velocity in radians/sec
    float       _angular_accel; // angular acceleration in radians/sec/sec
};
#endif	// AC_CIRCLE_H
