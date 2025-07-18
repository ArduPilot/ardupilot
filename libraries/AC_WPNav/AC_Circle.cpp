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
    // @Description: Circle mode's maximum turn rate in deg/sec.  Positive to turn clockwise, negative for counter clockwise. Final circle rate is also limited by speed and acceleration settings.
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
    _rotation_rate_max_rads = radians(_rate_parm_degs);
}

// Initializes circle flight using a center position in centimeters relative to the EKF origin.
// See init_NEU_m() for full details.
void AC_Circle::init_NEU_cm(const Vector3p& center_neu_cm, bool is_terrain_alt, float rate_degs)
{
    // Convert input from cm to meters and delegate to meter-based initializer
    init_NEU_m(center_neu_cm * 0.01, is_terrain_alt, rate_degs);
}

// Initializes circle flight mode using a specified center position in meters.
// Parameters:
//  - center_neu_m: Center of the circle in NEU frame (meters, relative to EKF origin)
//  - is_terrain_alt: If true, center.z is interpreted as height above terrain; otherwise, above EKF origin
//  - rate_degs: Desired turn rate in degrees per second (positive = clockwise, negative = counter-clockwise)
// Caller must preconfigure the position controller's speed and acceleration settings before calling.
void AC_Circle::init_NEU_m(const Vector3p& center_neu_m, bool is_terrain_alt, float rate_degs)
{
    // Store circle center and frame reference
    _center_neu_m = center_neu_m;
    _is_terrain_alt = is_terrain_alt;

    // Convert desired turn rate from degrees to radians
    _rotation_rate_max_rads = radians(rate_degs);

    // Initialise position controller using current lean angles
    _pos_control.init_NE_controller_stopping_point();
    _pos_control.init_U_controller_stopping_point();

    // Calculate velocity and acceleration limits based on circle configuration
    calc_velocities(true);

    // Use position-based angle initialization (avoids sharp yaw discontinuity)
    init_start_angle(false);
}

// Initializes the circle flight mode using the current stopping point as a reference.
// If the INIT_AT_CENTER option is not set, the circle center is projected one radius ahead along the vehicle's heading.
// Caller must configure position controller speeds and accelerations beforehand.
void AC_Circle::init()
{
    // Load radius and rate from parameters
    _radius_m = _radius_parm_cm * 0.01;
    _last_radius_param_cm = _radius_parm_cm;
    _rotation_rate_max_rads = radians(_rate_parm_degs);

    // Initialise position controller using current lean angles
    _pos_control.init_NE_controller_stopping_point();
    _pos_control.init_U_controller_stopping_point();

    // Get stopping point as initial reference for center
    const Vector3p& stopping_point_neu_m = _pos_control.get_pos_desired_NEU_m();

    // By default, set center to stopping point
    _center_neu_m = stopping_point_neu_m;

    // If INIT_AT_CENTER is not set, project center forward by radius in heading direction
    if ((_options.get() & CircleOptions::INIT_AT_CENTER) == 0) {
        _center_neu_m.x += _radius_m * _ahrs.cos_yaw();
        _center_neu_m.y += _radius_m * _ahrs.sin_yaw();
    }

    // Circle altitude is relative to EKF origin by default
    _is_terrain_alt = false;

    // Calculate velocity and acceleration constraints
    calc_velocities(true);

    // Initialize angle using vehicle heading
    init_start_angle(true);
}

// Sets the circle center using a Location object.
// Automatically determines whether the location uses terrain-relative or origin-relative altitude.
// If conversion fails, defaults to current position and logs a navigation error.
void AC_Circle::set_center(const Location& center)
{
    // Check if the location uses terrain-relative altitude
    if (center.get_alt_frame() == Location::AltFrame::ABOVE_TERRAIN) {
        // convert Location with terrain altitude
        Vector2f center_ne_m;
        float terr_alt_m;

        // Attempt to convert XY and Z to NEU frame with terrain altitude
        if (center.get_vector_xy_from_origin_NE_m(center_ne_m) && 
        center.get_alt_m(Location::AltFrame::ABOVE_TERRAIN, terr_alt_m)) {
            set_center_NEU_m(Vector3f(center_ne_m.x, center_ne_m.y, terr_alt_m), true);
        } else {
            // Conversion failed: fall back to current position and log error
            set_center_NEU_m(_pos_control.get_pos_estimate_NEU_m().tofloat(), false);
            LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_CIRCLE_INIT);
        }
    } else {
        // Handle alt-above-origin, alt-above-home, or absolute altitudes
        Vector3f circle_center_neu_m;
        if (!center.get_vector_from_origin_NEU_m(circle_center_neu_m)) {
            // Conversion failed: fall back to current position and log error
            circle_center_neu_m = _pos_control.get_pos_estimate_NEU_m().tofloat();
            LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_CIRCLE_INIT);
        }

        // Apply converted center and mark it as origin-relative
        set_center_NEU_m(circle_center_neu_m, false);
    }
}

// Sets the target circle rate in degrees per second.
// Positive values result in clockwise rotation; negative for counter-clockwise.
void AC_Circle::set_rate_degs(float rate_degs)
{
    // Convert the rate from degrees/sec to radians/sec
    _rotation_rate_max_rads = radians(rate_degs);
}

// Sets the circle radius in centimeters.
// See set_radius_m() for full details.
void AC_Circle::set_radius_cm(float radius_cm)
{
    // Convert input from cm to meters and call set_radius_m()
    set_radius_m(radius_cm * 0.01);
}

// Sets the circle radius in meters.
// Radius is constrained to AC_CIRCLE_RADIUS_MAX_M.
void AC_Circle::set_radius_m(float radius_m)
{
    // Constrain the radius to prevent unsafe or invalid circle sizes
    _radius_m = constrain_float(radius_m, 0, AC_CIRCLE_RADIUS_MAX_M);
}

// Returns true if the circle controller's update() function has run recently.
// Used by vehicle code to determine if yaw and position outputs are valid.
bool AC_Circle::is_active() const
{
    // Consider the controller active if the last update occurred within the past 200 milliseconds
    return (AP_HAL::millis() - _last_update_ms < 200);
}

// Updates the circle controller using a climb rate in cm/s.
// See update_ms() for full implementation details.
bool AC_Circle::update_cms(float climb_rate_cms)
{
    // Convert climb rate from cm/s to m/s and call the meter-based update
    return update_ms(climb_rate_cms * 0.01);
}

// Updates the circle controller using a climb rate in m/s.
// Computes new angular position, yaw, and vertical trajectory, then updates the position controller.
// Returns false if terrain data is required but unavailable.
bool AC_Circle::update_ms(float climb_rate_ms)
{
    // Recalculate angular velocities based on the current radius and rate
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
    float terr_offset_m = 0.0f;
    if (_is_terrain_alt && !get_terrain_offset_m(terr_offset_m)) {
        return false;
    }

    // calculate z-axis target
    float target_z_m;
    if (_is_terrain_alt) {
        target_z_m = _center_neu_m.z + terr_offset_m;
    } else {
        target_z_m = _pos_control.get_pos_desired_U_m();
    }

    // Construct target position centered on the circle center
    Vector3p target_neu_m {
        _center_neu_m.x,
        _center_neu_m.y,
        target_z_m
    };
    if (!is_zero(_radius_m)) {
        // Calculate position on the circle edge based on current angle
        target_neu_m.x += _radius_m * cosf(-_angle_rad);
        target_neu_m.y += - _radius_m * sinf(-_angle_rad);

        // Compute yaw toward the circle center
        _yaw_rad = get_bearing_rad(_pos_control.get_pos_desired_NEU_m().xy().tofloat(), _center_neu_m.xy().tofloat());

        // Optionally adjust yaw to face direction of travel
        if ((_options.get() & CircleOptions::FACE_DIRECTION_OF_TRAVEL) != 0) {
            _yaw_rad += is_positive(_rotation_rate_max_rads) ? -radians(90.0) : radians(90.0);
            _yaw_rad = wrap_PI(_yaw_rad);
        }

    } else {
        // heading is same as _angle_rad but converted to centi-degrees
        _yaw_rad = _angle_rad;
    }

    // update position controller target
    Vector2f zero_ne;
    _pos_control.input_pos_vel_accel_NE_m(target_neu_m.xy(), zero_ne, zero_ne);
    if (_is_terrain_alt) {
        float zero_u = 0;
        float target_u_m = target_neu_m.z;
        _pos_control.input_pos_vel_accel_U_m(target_u_m, zero_u, 0);
    } else {
        _pos_control.set_pos_target_U_from_climb_rate_m(climb_rate_ms);
    }

    // update position controller
    _pos_control.update_NE_controller();

    // set update time
    _last_update_ms = AP_HAL::millis();

    return true;
}

// Returns the closest point on the circle to the vehicle's current position in centimeters.
// See get_closest_point_on_circle_NEU_m() for full details.
void AC_Circle::get_closest_point_on_circle_NEU_cm(Vector3f& result_neu_cm, float& dist_cm) const
{
    // Convert input arguments from cm to meters
    Vector3f result_neu_m = result_neu_cm * 0.01;
    float dist_m = dist_cm * 0.01;

    // Compute closest point in meters
    get_closest_point_on_circle_NEU_m(result_neu_m, dist_m);

    // Convert results back to centimeters
    result_neu_cm = result_neu_m * 100.0;
    dist_cm = dist_m * 100.0;
}

// Returns the closest point on the circle to the vehicle's current position in meters.
// The result vector is updated with the NEU position of the closest point on the circle.
// The altitude (z) is set to match the circle center's altitude.
// dist_m is updated with the horizontal distance to the circle center.
// If the vehicle is at the center, the point directly behind the vehicle (based on yaw) is returned.
void AC_Circle::get_closest_point_on_circle_NEU_m(Vector3f& result_neu_m, float& dist_m) const
{
    // Get vehicle stopping point from the position controller (in NEU frame, meters)
    Vector3p stopping_point_neu_m;
    _pos_control.get_stopping_point_NE_m(stopping_point_neu_m.xy());
    _pos_control.get_stopping_point_U_m(stopping_point_neu_m.z);

    // Compute vector from stopping point to the circle center
    Vector3f vec_to_center_neu_m = (stopping_point_neu_m - _center_neu_m).tofloat();
    dist_m = vec_to_center_neu_m.length();

    // Return circle center if radius is zero (vehicle orbits in place)
    if (!is_positive(_radius_m)) {
        result_neu_m = _center_neu_m.tofloat();
        return;
    }

    // Handle edge case: vehicle is at the exact center of the circle
    if (is_zero(dist_m)) {
        result_neu_m.x = _center_neu_m.x - _radius_m * _ahrs.cos_yaw();
        result_neu_m.y = _center_neu_m.y - _radius_m * _ahrs.sin_yaw();
        result_neu_m.z = _center_neu_m.z;
        return;
    }

    // Calculate the closest point on the circle's edge by projecting out from center
    result_neu_m.x = _center_neu_m.x + vec_to_center_neu_m.x / dist_m * _radius_m;
    result_neu_m.y = _center_neu_m.y + vec_to_center_neu_m.y / dist_m * _radius_m;
    result_neu_m.z = _center_neu_m.z;
}

// Calculates angular velocity and acceleration limits based on the configured radius and rate.
// Should be called whenever radius or rate changes.
// If `init_velocity` is true, resets angular velocity to zero (used on controller startup).
void AC_Circle::calc_velocities(bool init_velocity)
{
    // If performing a panorama (radius is zero), use current yaw rate and enforce minimum acceleration
    if (_radius_m <= 0) {
        _angular_vel_max_rads = _rotation_rate_max_rads;
        _angular_accel_radss = MAX(fabsf(_angular_vel_max_rads), radians(AC_CIRCLE_ANGULAR_ACCEL_MIN));  // reach maximum yaw velocity in 1 second
    }else{
        // Limit max horizontal speed based on radius and available acceleration
        float vel_max_ms = MIN(_pos_control.get_max_speed_NE_ms(), safe_sqrt(0.5f*_pos_control.get_max_accel_NE_mss()*_radius_m));

        // Convert linear speed to angular velocity (rad/s)
        _angular_vel_max_rads = vel_max_ms/_radius_m;

        // Constrain to requested circle rate
        _angular_vel_max_rads = constrain_float(_rotation_rate_max_rads, -_angular_vel_max_rads, _angular_vel_max_rads);

        // Derive maximum angular acceleration
        _angular_accel_radss = MAX(_pos_control.get_max_accel_NE_mss() / _radius_m, radians(AC_CIRCLE_ANGULAR_ACCEL_MIN));
    }

    // Reset angular velocity to zero at initialization if requested
    if (init_velocity) {
        _angular_vel_rads = 0.0;
    }
}

// Sets the initial angle around the circle and resets the accumulated angle.
// If `use_heading` is true, uses vehicle heading to initialize angle for minimal yaw motion.
// If false, uses position relative to circle center to set angle.
void AC_Circle::init_start_angle(bool use_heading)
{
    // Reset the accumulated angle traveled around the circle
    _angle_total_rad = 0.0;

    // If radius is zero, we're doing a panorama—set angle to current yaw heading
    if (_radius_m <= 0) {
        _angle_rad = _ahrs.yaw;
        return;
    }

    // if use_heading is true
    if (use_heading) {
        // Initialize angle directly behind current heading to minimize yaw motion
        _angle_rad = wrap_PI(_ahrs.yaw - M_PI);
    } else {
        // If vehicle is exactly at the center, init angle behind vehicle (prevent undefined bearing)
        const Vector3f &curr_pos_desired_neu_m = _pos_control.get_pos_desired_NEU_m().tofloat();
        if (is_equal(curr_pos_desired_neu_m.x, float(_center_neu_m.x)) && is_equal(curr_pos_desired_neu_m.y, float(_center_neu_m.y))) {
            _angle_rad = wrap_PI(_ahrs.yaw - M_PI);
        } else {
            // Calculate bearing from circle center to current position
            float bearing_rad = atan2f(curr_pos_desired_neu_m.y - _center_neu_m.y, curr_pos_desired_neu_m.x - _center_neu_m.x);
            _angle_rad = wrap_PI(bearing_rad);
        }
    }
}

// Returns the expected source of terrain data for the circle controller.
// Used to determine whether terrain offset comes from rangefinder, terrain database, or is unavailable.
AC_Circle::TerrainSource AC_Circle::get_terrain_source() const
{
    // Use rangefinder if available and enabled
    if (_rangefinder_available) {
        return AC_Circle::TerrainSource::TERRAIN_FROM_RANGEFINDER;
    }
#if AP_TERRAIN_AVAILABLE
    // Fallback to terrain database if available and enabled
    const AP_Terrain *terrain = AP_Terrain::get_singleton();
    if ((terrain != nullptr) && terrain->enabled()) {
        return AC_Circle::TerrainSource::TERRAIN_FROM_TERRAINDATABASE;
    } else {
        return AC_Circle::TerrainSource::TERRAIN_UNAVAILABLE;
    }
#else
    // Terrain unavailable if no valid source
    return AC_Circle::TerrainSource::TERRAIN_UNAVAILABLE;
#endif
}

// Returns terrain offset in centimeters above the EKF origin at the current position.
// See get_terrain_offset_m() for full details.
bool AC_Circle::get_terrain_offset_cm(float& offset_cm)
{
    // Convert input value to meters for internal processing
    float offset_m = offset_cm * 0.01;
    // Retrieve terrain offset in meters
    bool terrain_valid = get_terrain_offset_m(offset_m);
    // Convert result back to centimeters
    offset_cm = offset_m * 100.0;
    return terrain_valid;
}

// Returns terrain offset in meters above the EKF origin at the current position.
// Positive values indicate terrain is above the EKF origin altitude.
// Terrain source may be rangefinder or terrain database.
bool AC_Circle::get_terrain_offset_m(float& offset_m)
{
    // Determine terrain source and calculate offset accordingly
    switch (get_terrain_source()) {
    case AC_Circle::TerrainSource::TERRAIN_UNAVAILABLE:
        return false;
    case AC_Circle::TerrainSource::TERRAIN_FROM_RANGEFINDER:
        if (_rangefinder_healthy) {
            // Use last known healthy rangefinder terrain offset
            offset_m = _rangefinder_terrain_offset_m;
            return true;
        }
        return false;
    case AC_Circle::TerrainSource::TERRAIN_FROM_TERRAINDATABASE:
#if AP_TERRAIN_AVAILABLE
        float terr_alt_m = 0.0f;
        AP_Terrain *terrain = AP_Terrain::get_singleton();
        if (terrain != nullptr && terrain->height_above_terrain(terr_alt_m, true)) {
            // Calculate offset from EKF origin altitude to terrain altitude
            offset_m = _pos_control.get_pos_estimate_NEU_m().z - terr_alt_m;
            return true;
        }
#endif
        return false;
    }

    // This fallback should never be reached
    return false;
}

// Checks if the circle radius parameter has changed.
// If so, updates internal `_radius_m` and stores the new parameter value.
void AC_Circle::check_param_change()
{
    // Check if the stored radius param has changed
    if (!is_equal(_last_radius_param_cm, _radius_parm_cm.get())) {
        // Convert radius from cm to meters and update internal radius
        _radius_m = _radius_parm_cm * 0.01;
        // Store new parameter value to detect future changes
        _last_radius_param_cm = _radius_parm_cm;
    }
}
