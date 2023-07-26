#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Terrain/AP_Terrain.h>
#include "AC_Circle.h"

#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_Circle::var_info[] = {
    // @Param: RADIUS
    // @DisplayName: Circle Radius
    // @Description: Defines the radius of the circle the vehicle will fly when in Circle flight mode
    // @Units: cm
    // @Range: 0 200000
    // @Increment: 100
    // @User: Standard
    AP_GROUPINFO("RADIUS",  0,  AC_Circle, _radius_parm, AC_CIRCLE_RADIUS_DEFAULT),

    // @Param: RATE
    // @DisplayName: Circle rate
    // @Description: Circle mode's turn rate in deg/sec.  Positive to turn clockwise, negative for counter clockwise. Circle rate must be less than ATC_SLEW_YAW parameter.
    // @Units: deg/s
    // @Range: -90 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RATE",    1, AC_Circle, _rate_parm,    AC_CIRCLE_RATE_DEFAULT),

    // @Param: OPTIONS
    // @DisplayName: Circle options
    // @Description: 0:Enable or disable using the pitch/roll stick control circle mode's radius and rate
    // @Bitmask: 0:manual control, 1:face direction of travel, 2:Start at center rather than on perimeter, 3:Make Mount ROI the center of the circle
    // @User: Standard
    AP_GROUPINFO("OPTIONS", 2, AC_Circle, _options, 1),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_Circle::AC_Circle(const AP_InertialNav& inav, const AP_AHRS_View& ahrs, AC_PosControl& pos_control) :
    _inav(inav),
    _ahrs(ahrs),
    _pos_control(pos_control),
    _yaw(0.0f),
    _angle(0.0f),
    _angle_total(0.0f),
    _angular_vel(0.0f),
    _angular_vel_max(0.0f),
    _angular_accel(0.0f)
{
    AP_Param::setup_object_defaults(this, var_info);

    // init flags
    _flags.panorama = false;
    _rate = _rate_parm;
}

/// init - initialise circle controller setting center specifically
///     set terrain_alt to true if center.z should be interpreted as an alt-above-terrain. Rate should be +ve in deg/sec for cw turn
///     caller should set the position controller's x,y and z speeds and accelerations before calling this
void AC_Circle::init(const Vector3p& center, bool terrain_alt, float rate_deg_per_sec)
{
    _center = center;
    _terrain_alt = terrain_alt;
    _rate = rate_deg_per_sec;

    // initialise position controller (sets target roll angle, pitch angle and I terms based on vehicle current lean angles)
    _pos_control.init_xy_controller_stopping_point();
    _pos_control.init_z_controller_stopping_point();

    // calculate velocities
    calc_velocities(true);

    // set start angle from position
    init_start_angle(false);
}

/// init - initialise circle controller setting center using stopping point and projecting out based on the copter's heading
///     caller should set the position controller's x,y and z speeds and accelerations before calling this
void AC_Circle::init()
{
    // initialize radius and rate from params
    _radius = _radius_parm;
    _last_radius_param = _radius_parm;
    _rate = _rate_parm;

    // initialise position controller (sets target roll angle, pitch angle and I terms based on vehicle current lean angles)
    _pos_control.init_xy_controller_stopping_point();
    _pos_control.init_z_controller_stopping_point();

    // get stopping point
    const Vector3p& stopping_point = _pos_control.get_pos_target_cm();

    // set circle center to circle_radius ahead of stopping point
    _center = stopping_point;
    if ((_options.get() & CircleOptions::INIT_AT_CENTER) == 0) {
        _center.x += _radius * _ahrs.cos_yaw();
        _center.y += _radius * _ahrs.sin_yaw();
    }
    _terrain_alt = false;

    // calculate velocities
    calc_velocities(true);

    // set starting angle from vehicle heading
    init_start_angle(true);
}

/// set circle center to a Location
void AC_Circle::set_center(const Location& center)
{
    if (center.get_alt_frame() == Location::AltFrame::ABOVE_TERRAIN) {
        // convert Location with terrain altitude
        Vector2f center_xy;
        int32_t terr_alt_cm;
        if (center.get_vector_xy_from_origin_NE(center_xy) && center.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, terr_alt_cm)) {
            set_center(Vector3f(center_xy.x, center_xy.y, terr_alt_cm), true);
        } else {
            // failed to convert location so set to current position and log error
            set_center(_inav.get_position_neu_cm(), false);
            AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_CIRCLE_INIT);
        }
    } else {
        // convert Location with alt-above-home, alt-above-origin or absolute alt
        Vector3f circle_center_neu;
        if (!center.get_vector_from_origin_NEU(circle_center_neu)) {
            // default to current position and log error
            circle_center_neu = _inav.get_position_neu_cm();
            AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_CIRCLE_INIT);
        }
        set_center(circle_center_neu, false);
    }
}

/// set_circle_rate - set circle rate in degrees per second
void AC_Circle::set_rate(float deg_per_sec)
{
    if (!is_equal(deg_per_sec, _rate)) {
        _rate = deg_per_sec;
    }
}

/// set_circle_rate - set circle rate in degrees per second
void AC_Circle::set_radius_cm(float radius_cm)
{
    _radius = constrain_float(radius_cm, 0, AC_CIRCLE_RADIUS_MAX);
}

/// returns true if update has been run recently
/// used by vehicle code to determine if get_yaw() is valid
bool AC_Circle::is_active() const
{
    return (AP_HAL::millis() - _last_update_ms < 200);
}

/// update - update circle controller
bool AC_Circle::update(float climb_rate_cms)
{
    calc_velocities(false);

    // calculate dt
    const float dt = _pos_control.get_dt();

    // ramp angular velocity to maximum
    if (_angular_vel < _angular_vel_max) {
        _angular_vel += fabsf(_angular_accel) * dt;
        _angular_vel = MIN(_angular_vel, _angular_vel_max);
    }
    if (_angular_vel > _angular_vel_max) {
        _angular_vel -= fabsf(_angular_accel) * dt;
        _angular_vel = MAX(_angular_vel, _angular_vel_max);
    }

    // update the target angle and total angle traveled
    float angle_change = _angular_vel * dt;
    _angle += angle_change;
    _angle = wrap_PI(_angle);
    _angle_total += angle_change;

    // calculate terrain adjustments
    float terr_offset = 0.0f;
    if (_terrain_alt && !get_terrain_offset(terr_offset)) {
        return false;
    }

    // calculate z-axis target
    float target_z_cm;
    if (_terrain_alt) {
        target_z_cm = _center.z + terr_offset;
    } else {
        target_z_cm = _pos_control.get_pos_target_z_cm();
    }

    // if the circle_radius is zero we are doing panorama so no need to update loiter target
    Vector3p target {
        _center.x,
        _center.y,
        target_z_cm
    };
    if (!is_zero(_radius)) {
        // calculate target position
        target.x += _radius * cosf(-_angle);
        target.y += - _radius * sinf(-_angle);

        // heading is from vehicle to center of circle
        _yaw = get_bearing_cd(_inav.get_position_xy_cm(), _center.tofloat().xy());

        if ((_options.get() & CircleOptions::FACE_DIRECTION_OF_TRAVEL) != 0) {
            _yaw += is_positive(_rate)?-9000.0f:9000.0f;
            _yaw = wrap_360_cd(_yaw);
        }

    } else {
        // heading is same as _angle but converted to centi-degrees
        _yaw = _angle * DEGX100;
    }

    // update position controller target
    Vector2f zero;
    _pos_control.input_pos_vel_accel_xy(target.xy(), zero, zero);
    if (_terrain_alt) {
        float zero2 = 0;
        float target_zf = target.z;
        _pos_control.input_pos_vel_accel_z(target_zf, zero2, 0);
    } else {
        _pos_control.set_pos_target_z_from_climb_rate_cm(climb_rate_cms);
    }

    // update position controller
    _pos_control.update_xy_controller();

    // set update time
    _last_update_ms = AP_HAL::millis();

    return true;
}

// get_closest_point_on_circle - returns closest point on the circle
//  circle's center should already have been set
//  closest point on the circle will be placed in result
//  result's altitude (i.e. z) will be set to the circle_center's altitude
//  if vehicle is at the center of the circle, the edge directly behind vehicle will be returned
void AC_Circle::get_closest_point_on_circle(Vector3f &result) const
{
    // return center if radius is zero
    if (_radius <= 0) {
        result = _center.tofloat();
        return;
    }

    // get current position
    Vector2p stopping_point;
    _pos_control.get_stopping_point_xy_cm(stopping_point);

    // calc vector from stopping point to circle center
    Vector2f vec = (stopping_point - _center.xy()).tofloat();
    float dist = vec.length();

    // if current location is exactly at the center of the circle return edge directly behind vehicle
    if (is_zero(dist)) {
        result.x = _center.x - _radius * _ahrs.cos_yaw();
        result.y = _center.y - _radius * _ahrs.sin_yaw();
        result.z = _center.z;
        return;
    }

    // calculate closest point on edge of circle
    result.x = _center.x + vec.x / dist * _radius;
    result.y = _center.y + vec.y / dist * _radius;
    result.z = _center.z;
}

// calc_velocities - calculate angular velocity max and acceleration based on radius and rate
//      this should be called whenever the radius or rate are changed
//      initialises the yaw and current position around the circle
void AC_Circle::calc_velocities(bool init_velocity)
{
    // if we are doing a panorama set the circle_angle to the current heading
    if (_radius <= 0) {
        _angular_vel_max = ToRad(_rate);
        _angular_accel = MAX(fabsf(_angular_vel_max),ToRad(AC_CIRCLE_ANGULAR_ACCEL_MIN));  // reach maximum yaw velocity in 1 second
    }else{
        // calculate max velocity based on waypoint speed ensuring we do not use more than half our max acceleration for accelerating towards the center of the circle
        float velocity_max = MIN(_pos_control.get_max_speed_xy_cms(), safe_sqrt(0.5f*_pos_control.get_max_accel_xy_cmss()*_radius));

        // angular_velocity in radians per second
        _angular_vel_max = velocity_max/_radius;
        _angular_vel_max = constrain_float(ToRad(_rate),-_angular_vel_max,_angular_vel_max);

        // angular_velocity in radians per second
        _angular_accel = MAX(_pos_control.get_max_accel_xy_cmss()/_radius, ToRad(AC_CIRCLE_ANGULAR_ACCEL_MIN));
    }

    // initialise angular velocity
    if (init_velocity) {
        _angular_vel = 0;
    }
}

// init_start_angle - sets the starting angle around the circle and initialises the angle_total
//  if use_heading is true the vehicle's heading will be used to init the angle causing minimum yaw movement
//  if use_heading is false the vehicle's position from the center will be used to initialise the angle
void AC_Circle::init_start_angle(bool use_heading)
{
    // initialise angle total
    _angle_total = 0;

    // if the radius is zero we are doing panorama so init angle to the current heading
    if (_radius <= 0) {
        _angle = _ahrs.yaw;
        return;
    }

    // if use_heading is true
    if (use_heading) {
        _angle = wrap_PI(_ahrs.yaw-M_PI);
    } else {
        // if we are exactly at the center of the circle, init angle to directly behind vehicle (so vehicle will backup but not change heading)
        const Vector3f &curr_pos = _inav.get_position_neu_cm();
        if (is_equal(curr_pos.x,float(_center.x)) && is_equal(curr_pos.y,float(_center.y))) {
            _angle = wrap_PI(_ahrs.yaw-M_PI);
        } else {
            // get bearing from circle center to vehicle in radians
            float bearing_rad = atan2f(curr_pos.y-_center.y,curr_pos.x-_center.x);
            _angle = wrap_PI(bearing_rad);
        }
    }
}

// get expected source of terrain data
AC_Circle::TerrainSource AC_Circle::get_terrain_source() const
{
    // use range finder if connected
    if (_rangefinder_available) {
        return AC_Circle::TerrainSource::TERRAIN_FROM_RANGEFINDER;
    }
#if AP_TERRAIN_AVAILABLE
    const AP_Terrain *terrain = AP_Terrain::get_singleton();
    if ((terrain != nullptr) && terrain->enabled()) {
        return AC_Circle::TerrainSource::TERRAIN_FROM_TERRAINDATABASE;
    } else {
        return AC_Circle::TerrainSource::TERRAIN_UNAVAILABLE;
    }
#else
    return AC_Circle::TerrainSource::TERRAIN_UNAVAILABLE;
#endif
}

// get terrain's altitude (in cm above the ekf origin) at the current position (+ve means terrain below vehicle is above ekf origin's altitude)
bool AC_Circle::get_terrain_offset(float& offset_cm)
{
    // calculate offset based on source (rangefinder or terrain database)
    switch (get_terrain_source()) {
    case AC_Circle::TerrainSource::TERRAIN_UNAVAILABLE:
        return false;
    case AC_Circle::TerrainSource::TERRAIN_FROM_RANGEFINDER:
        if (_rangefinder_healthy) {
            offset_cm = _rangefinder_terrain_offset_cm;
            return true;
        }
        return false;
    case AC_Circle::TerrainSource::TERRAIN_FROM_TERRAINDATABASE:
#if AP_TERRAIN_AVAILABLE
        float terr_alt = 0.0f;
        AP_Terrain *terrain = AP_Terrain::get_singleton();
        if (terrain != nullptr && terrain->height_above_terrain(terr_alt, true)) {
            offset_cm = _inav.get_position_z_up_cm() - (terr_alt * 100.0);
            return true;
        }
#endif
        return false;
    }

    // we should never get here but just in case
    return false;
}

void AC_Circle::check_param_change()
{
    if (!is_equal(_last_radius_param,_radius_parm.get())) {
        _radius = _radius_parm;
        _last_radius_param = _radius_parm;
    }
}
