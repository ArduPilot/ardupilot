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
    AP_GROUPINFO("RADIUS",  0,  AC_Circle, _radius_parm_cm, AC_CIRCLE_RADIUS_DEFAULT),

    // @Param: RATE
    // @DisplayName: Circle rate
    // @Description: Circle mode's turn rate in deg/sec.  Positive to turn clockwise, negative for counter clockwise. Circle rate must be less than ATC_SLEW_YAW parameter.
    // @Units: deg/s
    // @Range: -90 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RATE",    1, AC_Circle, _rate_parm_degs,    AC_CIRCLE_RATE_DEFAULT),

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
AC_Circle::AC_Circle(const AP_AHRS_View& ahrs, AC_PosControl& pos_control) :
    _ahrs(ahrs),
    _pos_control(pos_control)
{
    AP_Param::setup_object_defaults(this, var_info);

    // init flags
    _flags.panorama = false;
    _rate_degs = _rate_parm_degs;
}

/// init - initialise circle controller setting center specifically
///     set is_terrain_alt to true if center_neu_cm.z should be interpreted as an alt-above-terrain. Rate should be +ve in deg/sec for cw turn
///     caller should set the position controller's x,y and z speeds and accelerations before calling this
void AC_Circle::init_NEU_cm(const Vector3p& center_neu_cm, bool is_terrain_alt, float rate_degs)
{
    _center_neu_cm = center_neu_cm;
    _is_terrain_alt = is_terrain_alt;
    _rate_degs = rate_degs;

    // initialise position controller (sets target roll angle, pitch angle and I terms based on vehicle current lean angles)
    _pos_control.init_NE_controller_stopping_point();
    _pos_control.init_U_controller_stopping_point();

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
    _radius_cm = _radius_parm_cm;
    _last_radius_param_cm = _radius_parm_cm;
    _rate_degs = _rate_parm_degs;

    // initialise position controller (sets target roll angle, pitch angle and I terms based on vehicle current lean angles)
    _pos_control.init_NE_controller_stopping_point();
    _pos_control.init_U_controller_stopping_point();

    // get stopping point
    const Vector3p& stopping_point_neu_cm = _pos_control.get_pos_desired_NEU_cm();

    // set circle center to circle_radius ahead of stopping point
    _center_neu_cm = stopping_point_neu_cm;
    if ((_options.get() & CircleOptions::INIT_AT_CENTER) == 0) {
        _center_neu_cm.x += _radius_cm * _ahrs.cos_yaw();
        _center_neu_cm.y += _radius_cm * _ahrs.sin_yaw();
    }
    _is_terrain_alt = false;

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
        Vector2f center_ne_cm;
        int32_t terr_alt_cm;
        if (center.get_vector_xy_from_origin_NE_cm(center_ne_cm) && 
        center.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, terr_alt_cm)) {
            set_center_NEU_cm(Vector3f(center_ne_cm.x, center_ne_cm.y, terr_alt_cm), true);
        } else {
            // failed to convert location so set to current position and log error
            set_center_NEU_cm(_pos_control.get_pos_estimate_NEU_cm().tofloat(), false);
            LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_CIRCLE_INIT);
        }
    } else {
        // convert Location with alt-above-home, alt-above-origin or absolute alt
        Vector3f circle_center_neu_cm;
        if (!center.get_vector_from_origin_NEU_cm(circle_center_neu_cm)) {
            // default to current position and log error
            circle_center_neu_cm = _pos_control.get_pos_estimate_NEU_cm().tofloat();
            LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_CIRCLE_INIT);
        }
        set_center_NEU_cm(circle_center_neu_cm, false);
    }
}

/// set_circle_rate - set circle rate in degrees per second
void AC_Circle::set_rate_degs(float rate_degs)
{
    if (!is_equal(rate_degs, _rate_degs)) {
        _rate_degs = rate_degs;
    }
}

/// set_circle_rate - set circle rate in degrees per second
void AC_Circle::set_radius_cm(float radius_cm)
{
    _radius_cm = constrain_float(radius_cm, 0, AC_CIRCLE_RADIUS_MAX);
}

/// returns true if update has been run recently
/// used by vehicle code to determine if get_yaw() is valid
bool AC_Circle::is_active() const
{
    return (AP_HAL::millis() - _last_update_ms < 200);
}

/// update - update circle controller
bool AC_Circle::update_cms(float climb_rate_cms)
{
    calc_velocities(false);

    // calculate dt
    const float dt = _pos_control.get_dt_s();

    // ramp angular velocity to maximum
    if (_angular_vel_rads < _angular_vel_max_rads) {
        _angular_vel_rads += fabsf(_angular_accel_radss) * dt;
        _angular_vel_rads = MIN(_angular_vel_rads, _angular_vel_max_rads);
    }
    if (_angular_vel_rads > _angular_vel_max_rads) {
        _angular_vel_rads -= fabsf(_angular_accel_radss) * dt;
        _angular_vel_rads = MAX(_angular_vel_rads, _angular_vel_max_rads);
    }

    // update the target angle and total angle travelled
    float angle_change_rad = _angular_vel_rads * dt;
    _angle_rad += angle_change_rad;
    _angle_rad = wrap_PI(_angle_rad);
    _angle_total_rad += angle_change_rad;

    // calculate terrain adjustments
    float terr_offset_cm = 0.0f;
    if (_is_terrain_alt && !get_terrain_offset_cm(terr_offset_cm)) {
        return false;
    }

    // calculate z-axis target
    float target_z_cm;
    if (_is_terrain_alt) {
        target_z_cm = _center_neu_cm.z + terr_offset_cm;
    } else {
        target_z_cm = _pos_control.get_pos_desired_U_cm();
    }

    // if the circle_radius is zero we are doing panorama so no need to update loiter target
    Vector3p target_neu_cm {
        _center_neu_cm.x,
        _center_neu_cm.y,
        target_z_cm
    };
    if (!is_zero(_radius_cm)) {
        // calculate target position
        target_neu_cm.x += _radius_cm * cosf(-_angle_rad);
        target_neu_cm.y += - _radius_cm * sinf(-_angle_rad);

        // heading is from vehicle to center of circle
        _yaw_cd = get_bearing_cd(_pos_control.get_pos_desired_NEU_cm().xy().tofloat(), _center_neu_cm.xy().tofloat());

        if ((_options.get() & CircleOptions::FACE_DIRECTION_OF_TRAVEL) != 0) {
            _yaw_cd += is_positive(_rate_degs)?-9000.0f:9000.0f;
            _yaw_cd = wrap_360_cd(_yaw_cd);
        }

    } else {
        // heading is same as _angle_rad but converted to centi-degrees
        _yaw_cd = rad_to_cd(_angle_rad);
    }

    // update position controller target
    Vector2f zero_ne;
    _pos_control.input_pos_vel_accel_NE_cm(target_neu_cm.xy(), zero_ne, zero_ne);
    if (_is_terrain_alt) {
        float zero_u = 0;
        float target_u_cm = target_neu_cm.z;
        _pos_control.input_pos_vel_accel_U_cm(target_u_cm, zero_u, 0);
    } else {
        _pos_control.set_pos_target_U_from_climb_rate_cm(climb_rate_cms);
    }

    // update position controller
    _pos_control.update_NE_controller();

    // set update time
    _last_update_ms = AP_HAL::millis();

    return true;
}

// get_closest_point_on_circle_NEU_cm - returns closest point on the circle
//      circle's center should already have been set
//      closest point on the circle will be placed in result_neu_cm, dist_cm will be updated with the 3D distance to the center
//      result_neu_cm's altitude (i.e. z) will be set to the circle_center's altitude
//      if vehicle is at the center of the circle, the edge directly behind vehicle will be returned
void AC_Circle::get_closest_point_on_circle_NEU_cm(Vector3f& result_neu_cm, float& dist_cm) const
{
    // get current position
    Vector3p stopping_point_neu_cm;
    _pos_control.get_stopping_point_NE_cm(stopping_point_neu_cm.xy());
    _pos_control.get_stopping_point_U_cm(stopping_point_neu_cm.z);

    // calc vector from stopping point to circle center
    Vector3f vec_to_center_neu_cm = (stopping_point_neu_cm - _center_neu_cm).tofloat();
    dist_cm = vec_to_center_neu_cm.length();

    // return center if radius is zero
    if (!is_positive(_radius_cm)) {
        result_neu_cm = _center_neu_cm.tofloat();
        return;
    }

    // if current location is exactly at the center of the circle return edge directly behind vehicle
    if (is_zero(dist_cm)) {
        result_neu_cm.x = _center_neu_cm.x - _radius_cm * _ahrs.cos_yaw();
        result_neu_cm.y = _center_neu_cm.y - _radius_cm * _ahrs.sin_yaw();
        result_neu_cm.z = _center_neu_cm.z;
        return;
    }

    // calculate closest point on edge of circle
    result_neu_cm.x = _center_neu_cm.x + vec_to_center_neu_cm.x / dist_cm * _radius_cm;
    result_neu_cm.y = _center_neu_cm.y + vec_to_center_neu_cm.y / dist_cm * _radius_cm;
    result_neu_cm.z = _center_neu_cm.z;
}

// calc_velocities - calculate angular velocity max and acceleration based on radius and rate
//      this should be called whenever the radius or rate are changed
//      initialises the yaw and current position around the circle
void AC_Circle::calc_velocities(bool init_velocity)
{
    // if we are doing a panorama set the circle_angle to the current heading
    if (_radius_cm <= 0) {
        _angular_vel_max_rads = radians(_rate_degs);
        _angular_accel_radss = MAX(fabsf(_angular_vel_max_rads),radians(AC_CIRCLE_ANGULAR_ACCEL_MIN));  // reach maximum yaw velocity in 1 second
    }else{
        // calculate max velocity based on waypoint speed ensuring we do not use more than half our max acceleration for accelerating towards the center of the circle
        float vel_max_cms = MIN(_pos_control.get_max_speed_NE_cms(), safe_sqrt(0.5f*_pos_control.get_max_accel_NE_cmss()*_radius_cm));

        // angular_velocity in radians per second
        _angular_vel_max_rads = vel_max_cms/_radius_cm;
        _angular_vel_max_rads = constrain_float(radians(_rate_degs),-_angular_vel_max_rads,_angular_vel_max_rads);

        // angular_velocity in radians per second
        _angular_accel_radss = MAX(_pos_control.get_max_accel_NE_cmss()/_radius_cm, radians(AC_CIRCLE_ANGULAR_ACCEL_MIN));
    }

    // initialise angular velocity
    if (init_velocity) {
        _angular_vel_rads = 0;
    }
}

// init_start_angle - sets the starting angle around the circle and initialises the angle_total
//      if use_heading is true the vehicle's heading will be used to init the angle causing minimum yaw movement
//      if use_heading is false the vehicle's position from the center will be used to initialise the angle
void AC_Circle::init_start_angle(bool use_heading)
{
    // initialise angle total
    _angle_total_rad = 0;

    // if the radius is zero we are doing panorama so init angle to the current heading
    if (_radius_cm <= 0) {
        _angle_rad = _ahrs.yaw;
        return;
    }

    // if use_heading is true
    if (use_heading) {
        _angle_rad = wrap_PI(_ahrs.yaw-M_PI);
    } else {
        // if we are exactly at the center of the circle, init angle to directly behind vehicle (so vehicle will backup but not change heading)
        // curr_pos_desired_neu_cm is the position before we add offsets and terrain
        const Vector3f &curr_pos_desired_neu_cm = _pos_control.get_pos_desired_NEU_cm().tofloat();
        if (is_equal(curr_pos_desired_neu_cm.x, float(_center_neu_cm.x)) && is_equal(curr_pos_desired_neu_cm.y, float(_center_neu_cm.y))) {
            _angle_rad = wrap_PI(_ahrs.yaw-M_PI);
        } else {
            // get bearing from circle center to vehicle in radians
            float bearing_rad = atan2f(curr_pos_desired_neu_cm.y - _center_neu_cm.y, curr_pos_desired_neu_cm.x - _center_neu_cm.x);
            _angle_rad = wrap_PI(bearing_rad);
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
bool AC_Circle::get_terrain_offset_cm(float& offset_cm)
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
            offset_cm = _pos_control.get_pos_estimate_NEU_cm().z - (terr_alt * 100.0);
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
    if (!is_equal(_last_radius_param_cm,_radius_parm_cm.get())) {
        _radius_cm = _radius_parm_cm;
        _last_radius_param_cm = _radius_parm_cm;
    }
}
