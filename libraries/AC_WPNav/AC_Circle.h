#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AC_AttitudeControl/AC_PosControl.h>      // Position control library

// Circle mode default and limit constants
#define AC_CIRCLE_RADIUS_DEFAULT     1000.0f   // Default circle radius in cm (10 meters).
#define AC_CIRCLE_RATE_DEFAULT       20.0f     // Default circle turn rate in degrees per second. Positive = clockwise, negative = counter-clockwise.
#define AC_CIRCLE_ANGULAR_ACCEL_MIN  2.0f      // Minimum angular acceleration in deg/s² (used to avoid sluggish yaw transitions).
#define AC_CIRCLE_RADIUS_MAX_M       2000.0    // Maximum allowed circle radius in meters (2000 m = 2 km).

class AC_Circle
{
public:

    /// Constructor
    AC_Circle(const AP_AHRS_View& ahrs, AC_PosControl& pos_control);

    // Initializes circle flight using a center position in centimeters relative to the EKF origin.
    // See init_NED_m() for full details.
    void init_NEU_cm(const Vector3p& center_neu_cm, bool is_terrain_alt, float rate_degs);

    // Initializes circle flight mode using a specified center position in meters.
    // Parameters:
    //  - center_ned_m: Center of the circle in NED frame (meters, relative to EKF origin)
    // - is_terrain_alt: If true, center_ned_m.z is interpreted as relative to terrain; otherwise, above EKF origin
    //  - rate_degs: Desired turn rate in degrees per second (positive = clockwise, negative = counter-clockwise)
    // Caller must preconfigure the position controller's speed and acceleration settings before calling.
    void init_NED_m(const Vector3p& center_ned_m, bool is_terrain_alt, float rate_degs);

    // Initializes the circle flight mode using the current stopping point as a reference.
    // If the INIT_AT_CENTER option is not set, the circle center is projected one radius ahead along the vehicle's heading.
    // Caller must configure position controller speeds and accelerations beforehand.
    void init();

    // Sets the circle center using a Location object.
    // Automatically determines whether the location uses terrain-relative or origin-relative altitude.
    // If conversion fails, defaults to current position and logs a navigation error.
    void set_center(const Location& center);

    // Sets the circle center using a NED position vector in meters from the EKF origin.
    // - is_terrain_alt: If true, center_ned_m.z is interpreted as relative to terrain; otherwise, above EKF origin
    void set_center_NED_m(const Vector3p& center_ned_m, bool is_terrain_alt) { _center_ned_m = center_ned_m; _is_terrain_alt = is_terrain_alt; }

    // Returns the circle center in centimeters from the EKF origin.
    // See get_center_NED_m() for full details.
    const Vector3p get_center_NEU_cm() const { return Vector3p(_center_ned_m.x, _center_ned_m.y, -_center_ned_m.z) * 100.0; }

    // Returns the circle center in meters from the EKF origin.
    // Altitude frame is determined by `center_is_terrain_alt()`.
    const Vector3p& get_center_NED_m() const { return _center_ned_m; }

    // Returns true if the circle center altitude is relative to terrain height.
    bool center_is_terrain_alt() const { return _is_terrain_alt; }

    // Returns the circle radius in centimeters.
    // See get_radius_m() for full details.
    float get_radius_cm() const { return get_radius_m() * 100.0; }

    // Returns the circle radius in meters.
    // If `_radius_m` is non-positive, falls back to the RADIUS parameter.
    float get_radius_m() const { return is_positive(_radius_m)?_radius_m:_radius_parm_cm * 0.01; }

    // Sets the circle radius in centimeters.
    // See set_radius_m() for full details.
    void set_radius_cm(float radius_cm);

    // Sets the circle radius in meters.
    // Radius is constrained to AC_CIRCLE_RADIUS_MAX_M.
    void set_radius_m(float radius_m);

    // Returns the configured circle turn rate in degrees per second from the RATE parameter.
    float get_rate_degs() const { return _rate_parm_degs; }

    // Returns the current angular velocity in degrees per second.
    // May be lower than the configured maximum due to ramp constraints.
    float get_rate_current() const { return degrees(_angular_vel_rads); }

    // Sets the target circle rate in degrees per second.
    // Positive values result in clockwise rotation; negative for counter-clockwise.
    void set_rate_degs(float rate_degs);

    // Returns the total angle, in radians, that the vehicle has traveled along the circular path.
    float get_angle_total_rad() const { return _angle_total_rad; }

    // Updates the circle controller using a climb rate in cm/s.
    // See update_ms() for full implementation details.
    bool update_cms(float climb_rate_cms = 0.0f) WARN_IF_UNUSED;

    // Updates the circle controller using a climb rate in m/s.
    // Computes new angular position, yaw, and vertical trajectory, then updates the position controller.
    // Returns false if terrain data is required but unavailable.
    bool update_ms(float climb_rate_ms = 0.0f) WARN_IF_UNUSED;

    // Returns the desired roll angle in centidegrees from the position controller.
    // Should be passed to the attitude controller as a stabilization input.
    float get_roll_cd() const { return rad_to_cd(_pos_control.get_roll_rad()); }

    // Returns the desired pitch angle in centidegrees from the position controller.
    // Should be passed to the attitude controller as a stabilization input.
    float get_pitch_cd() const { return rad_to_cd(_pos_control.get_pitch_rad()); }

    // Returns the desired yaw angle in centidegrees computed by the circle controller.
    // May be oriented toward the circle center or along the path depending on configuration.
    float get_yaw_cd() const { return rad_to_cd(_yaw_rad); }

    // Returns the desired yaw angle in radians.
    // Used for directional control based on circle configuration.
    float get_yaw_rad() const { return _yaw_rad; }

    // Returns the desired thrust vector (unit vector in body frame) from the position controller.
    // Can be used by the attitude controller to align thrust direction.
    Vector3f get_thrust_vector() const { return _pos_control.get_thrust_vector(); }

    // Returns true if the circle controller's update() function has run recently.
    // Used by vehicle code to determine if yaw and position outputs are valid.
    bool is_active() const;

    // Returns the closest point on the circle to the vehicle's current position in centimeters.
    // See get_closest_point_on_circle_NED_m() for full details.
    void get_closest_point_on_circle_NEU_cm(Vector3f& result_neu_cm, float& dist_cm) const;

    // Returns the closest point on the circle to the vehicle's stopping point in meters.
    // The result vector is updated with the NED position of the closest point on the circle.
    // The altitude (z) is set to match the circle center's altitude.
    // dist_m is updated with the 3D distance to the circle edge from the stopping point.
    // If the vehicle is at the center, the point directly behind the vehicle (based on yaw) is returned.
    void get_closest_point_on_circle_NED_m(Vector3p& result_ned_m, float& dist_m) const;

    // Returns the horizontal distance to the circle target in meters.
    // Calculated using the position controller’s NE position error norm.
    float get_distance_to_target_m() const { return _pos_control.get_pos_error_NE_m(); }

    // Returns the bearing from the vehicle to the circle target in radians.
    // Bearing is measured clockwise from North (0 = North).
    float get_bearing_to_target_rad() const { return _pos_control.get_bearing_to_target_rad(); }

    // Returns true if pilot stick control of circle radius and rate is enabled.
    // See pilot_control_enabled() for flag logic.
    bool pilot_control_enabled() const { return (_options.get() & CircleOptions::MANUAL_CONTROL) != 0; }

    // Returns true if the mount ROI is fixed at the circle center.
    // See roi_at_center() for flag logic.
    bool roi_at_center() const { return (_options.get() & CircleOptions::ROI_AT_CENTER) != 0; }

    // Sets rangefinder terrain offset (in centimeters) above EKF origin.
    // See set_rangefinder_terrain_U_m() for full details.
    void set_rangefinder_terrain_U_cm(bool use, bool healthy, float terrain_u_cm) { _rangefinder_available = use; _rangefinder_healthy = healthy; _rangefinder_terrain_u_m = terrain_u_cm * 0.01;}

    // Sets rangefinder terrain offset (in meters) above EKF origin.
    // Used for terrain-relative altitude tracking during circular flight.
    void set_rangefinder_terrain_U_m(bool use, bool healthy, float terrain_u_m) { _rangefinder_available = use; _rangefinder_healthy = healthy; _rangefinder_terrain_u_m = terrain_u_m;}

    // Checks if the circle radius parameter has changed.
    // If so, updates internal `_radius_m` and stores the new parameter value.
    void check_param_change();

    static const struct AP_Param::GroupInfo var_info[];

private:

    // Calculates angular velocity and acceleration limits based on the configured radius and rate.
    // Should be called whenever radius or rate changes.
    // If `init_velocity` is true, resets angular velocity to zero (used on controller startup).
    void calc_velocities(bool init_velocity);

    // Sets the initial angle around the circle and resets the accumulated angle.
    // If `use_heading` is true, uses vehicle heading to initialize angle for minimal yaw motion.
    // If false, uses position relative to circle center to set angle.
    void init_start_angle(bool use_heading);

    // Returns the expected source of terrain data for the circle controller.
    // Used to determine whether terrain offset comes from rangefinder, terrain database, or is unavailable.
    enum class TerrainSource {
        TERRAIN_UNAVAILABLE,
        TERRAIN_FROM_RANGEFINDER,
        TERRAIN_FROM_TERRAINDATABASE,
    };
    AC_Circle::TerrainSource get_terrain_source() const;

    // Returns terrain offset in meters above the EKF origin at the current position.
    // Positive values indicate terrain is above the EKF origin altitude.
    // Terrain source may be rangefinder or terrain database.
    bool get_terrain_U_m(float& terrain_u_m);

    // references to inertial nav and ahrs libraries
    const AP_AHRS_View&         _ahrs;
    AC_PosControl&              _pos_control;

    enum CircleOptions {
        MANUAL_CONTROL           = 1U << 0, // Enables pilot stick input to adjust circle radius and turn rate.
        FACE_DIRECTION_OF_TRAVEL = 1U << 1, // Yaw aligns with direction of travel (tangent to circle path).
        INIT_AT_CENTER           = 1U << 2, // Initializes circle with center at current position (instead of radius ahead).
        ROI_AT_CENTER            = 1U << 3, // Sets camera mount ROI to circle center during circle mode.
    };

    // parameters
    AP_Float _radius_parm_cm;     // Circle radius in centimeters, loaded from parameters.
    AP_Float _rate_parm_degs;     // Circle rotation rate in degrees per second, loaded from parameters.
    AP_Int16 _options;            // Bitmask of CircleOptions (e.g. manual control, ROI at center, etc.).

    // internal variables
    Vector3p _center_ned_m;             // Center of the circle in meters from EKF origin (NED frame).
    float    _radius_m;                 // Current circle radius in meters.
    float    _rotation_rate_max_rads;   // Requested circle turn rate in rad/s (+ve = CW, -ve = CCW).
    float    _yaw_rad;                  // Desired yaw heading in radians (typically toward circle center or tangent).
    float    _angle_rad;                // Current angular position around the circle in radians (0 = due north of center).
    float    _angle_total_rad;          // Accumulated angle travelled in radians (used for full rotations).
    float    _angular_vel_rads;         // Current angular velocity in rad/s.
    float    _angular_vel_max_rads;     // Maximum allowed angular velocity in rad/s.
    float    _angular_accel_radss;      // Angular acceleration limit in rad/s².
    uint32_t _last_update_ms;           // Timestamp (in milliseconds) of the last update() call.
    float    _last_radius_param_cm;     // Cached copy of radius parameter (cm) to detect parameter changes.

    // terrain following variables
    bool  _is_terrain_alt;               // True if _center_ned_m.z is relative to terrain height; false if relative to EKF origin.
    bool  _rangefinder_available;        // True if rangefinder is available and enabled.
    bool  _rangefinder_healthy;          // True if rangefinder reading is within valid operating range.
    float _rangefinder_terrain_u_m; // Terrain height above EKF origin (meters), from rangefinder or terrain database.
};
