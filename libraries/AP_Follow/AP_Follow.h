/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once
#include "AP_Follow_config.h"

#if AP_FOLLOW_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AC_PID/AC_P.h>
#include <AP_RTC/JitterCorrection.h>

//==============================================================================
// AP_Follow Class
// Target Following Logic for ArduPilot Vehicles
//==============================================================================

class AP_Follow
{

public:
    //==========================================================================
    // Public Enums
    //==========================================================================

    // enum for FOLLOW_OPTIONS parameter
    enum class Option {
        MOUNT_FOLLOW_ON_ENTER = 1
    };

    // enum for YAW_BEHAVE parameter
    enum YawBehave {
        YAW_BEHAVE_NONE = 0,
        YAW_BEHAVE_FACE_LEAD_VEHICLE = 1,
        YAW_BEHAVE_SAME_AS_LEAD_VEHICLE = 2,
        YAW_BEHAVE_DIR_OF_FLIGHT = 3
    };

    //==========================================================================
    // Constructor and Singleton Access
    //==========================================================================

    // constructor
    AP_Follow();

    // enable as singleton
    static AP_Follow *get_singleton(void) {
        return _singleton;
    }

    // returns true if library is enabled
    bool enabled() const { return _enabled; }

    // set which target to follow
    void set_target_sysid(uint8_t sysid) { _sysid.set(sysid); }

    // Resets the follow mode offsets to zero if they were automatically initialized. Should be called when exiting Follow mode.
    void clear_offsets_if_required();

    //==========================================================================
    // Target Estimation and Tracking Methods
    //==========================================================================

    // Returns true if following is enabled and a recent target location update has been received.
    bool have_target() const;

    // Projects the target’s position, velocity, and heading forward using the latest updates, smoothing with input shaping if necessary 
    void update_estimates();

    // Retrieves the estimated target position, velocity, and acceleration in the NED frame relative to the origin (units: meters and meters/second).
    bool get_target_pos_vel_accel_NED_m(Vector3p &pos_ned_m, Vector3f &vel_ned_ms, Vector3f &accel_ned_mss) const;

    // Retrieves the estimated target position, velocity, and acceleration in the NED frame, including configured positional offsets.
    bool get_ofs_pos_vel_accel_NED_m(Vector3p &pos_ofs_ned_m, Vector3f &vel_ofs_ned_ms, Vector3f &accel_ofs_ned_mss) const;

    // Retrieves the estimated target heading and heading rate in radians.
    bool get_heading_heading_rate_rad(float &heading_rad, float &heading_rate_rads) const;

    //==========================================================================
    // Global Location and Velocity Retrieval (LUA Bindings)
    //==========================================================================

    // Retrieves the estimated global location and velocity of the target
    bool get_target_location_and_velocity(Location &loc, Vector3f &vel_ned);

    // Retrieves the estimated global location including configured positional offsets and velocity of the target,  (for LUA bindings).
    bool get_target_location_and_velocity_ofs(Location &loc, Vector3f &vel_ned);

    // Retrieves the estimated target heading in degrees (0° = North, 90° = East) for LUA bindings.
    bool get_target_heading_deg(float &heading);

    // Retrieves the estimated target heading rate in degrees per second.
    bool get_target_heading_rate_degs(float &_target_heading_rate_degs);

    // Retrieves the distance vector to the target, the distance vector including configured offsets, and the target’s velocity in the NED frame (units: meters).
    bool get_target_dist_and_vel_NED_m(Vector3f &dist_ned, Vector3f &dist_with_ofs, Vector3f &vel_ned);

    //==========================================================================
    // Accessor Methods
    //==========================================================================

    // get target sysid as a 32 bit number to allow for future expansion of MAV_SYSID
    uint32_t get_target_sysid() const { return (uint32_t)_sysid.get(); }

    // get position controller.  this controller is not used within this library but it is convenient to hold it here
    const AC_P& get_pos_p() const { return _p_pos; }

    // get user defined yaw behaviour
    YawBehave get_yaw_behave() const { return (YawBehave)_yaw_behave.get(); }

    // initialise offsets to provided distance vector to other vehicle (in meters in NED frame) if required
    void init_offsets_if_required();

    //==========================================================================
    // MAVLink Message Handling
    //==========================================================================

    // parse mavlink messages which may hold target's position, velocity and attitude
    void handle_msg(const mavlink_message_t &msg);

    //==========================================================================
    // GCS Reporting Methods
    //==========================================================================

    // get horizontal distance to target (including offset) in meters (for reporting purposes)
    float get_distance_to_target_m() const { return _dist_to_target_m; }

    // get bearing to target (including offset) in degrees (for reporting purposes)
    float get_bearing_to_target_deg() const { return _bearing_to_target_deg; }

    // get system time of last position update
    // LUA bindings
    uint32_t get_last_update_ms() const { return _last_location_update_ms; }

    // returns true if a follow option enabled
    bool option_is_enabled(Option option) const { return (_options.get() & (uint16_t)option) != 0; }

    //==========================================================================
    // Parameter Group Info
    //==========================================================================

    // parameter list
    static const struct AP_Param::GroupInfo var_info[];

private:
    //==========================================================================
    // Private Helper Functions and Singleton
    //==========================================================================

    // Singleton instance of the AP_Follow class.
    static AP_Follow *_singleton;

    // returns true if we should extract information from msg
    bool should_handle_message(const mavlink_message_t &msg) const;

    // Checks whether the current estimate should be reset based on position and velocity errors.
    bool estimate_error_too_large() const;
    
    // Calculates max velocity change under trapezoidal or triangular acceleration profile (jerk-limited).
    float calc_max_velocity_change(float accel_max, float jerk_max, float timeout_sec) const;

    // Rotates a 3D vector clockwise by the specified angle in degrees.
    Vector3f rotate_vector(const Vector3f &vec, float angle_deg) const;

    // Resets the recorded distance and bearing to the target to zero.
    void clear_dist_and_bearing_to_target();
    void update_dist_and_bearing_to_target();

    // handle various mavlink messages supplying position:
    bool handle_global_position_int_message(const mavlink_message_t &msg);
    bool handle_follow_target_message(const mavlink_message_t &msg);

    // write out an onboard-log message to help diagnose follow problems:
    void Log_Write_FOLL();

    //==========================================================================
    // Parameters
    //==========================================================================

    AP_Int8     _enabled;           // 1 = Follow mode is enabled; 0 = disabled
    AP_Int16    _sysid;             // MAVLink system ID of the target (0 = auto-select first sender)
    AP_Float    _dist_max_m;        // Maximum allowed distance to target in meters; if exceeded, estimation is rejected
    AP_Int8     _offset_type;       // Offset frame type: 0 = NED, 1 = relative to lead vehicle heading
    AP_Vector3f _offset_m;          // Offset from lead vehicle (meters), in NED or FRD frame depending on _offset_type
    AP_Int8     _yaw_behave;        // Yaw behavior mode (see YawBehave enum)
    AP_Enum<Location::AltFrame>    _alt_type;          // altitude source for follow mode
    AC_P        _p_pos;             // Position error P-controller for optional altitude following
    AP_Int16    _options;           // Bitmask of follow behavior options (e.g., mount follow, etc.)
    AP_Float    _timeout;           // position estimate timeout after x milliseconds

    AP_Float    _accel_max_ne_mss;  // Max horizontal acceleration for kinematic shaping (m/s²)
    AP_Float    _jerk_max_ne_msss;  // Max horizontal jerk for kinematic shaping (m/s³)
    AP_Float    _accel_max_d_mss;   // Max vertical acceleration for kinematic shaping (m/s²)
    AP_Float    _jerk_max_d_msss;   // Max vertical jerk for kinematic shaping (m/s³)
    AP_Float    _accel_max_h_degss; // Max angular acceleration for heading shaping (deg/s²)
    AP_Float    _jerk_max_h_degsss; // Max angular jerk for heading shaping (deg/s³)

    //==========================================================================
    // Internal State Variables
    //==========================================================================

    uint32_t    _last_location_update_ms;       // Time of last target position update (ms)
    uint32_t    _last_estimation_update_ms;     // Time of last estimate update (ms)

    Vector3p    _target_pos_ned_m;              // Latest received target position (NED frame, meters)
    Vector3f    _target_vel_ned_ms;             // Latest received target velocity (NED frame, m/s)
    Vector3f    _target_accel_ned_mss;          // Latest received target acceleration (NED frame, m/s²)
    float       _target_heading_deg;            // Latest received target heading (degrees, 0 = North)
    float       _target_heading_rate_degs;      // Latest received target yaw rate (deg/s)

    bool        _estimate_valid;                // True if internal estimate is valid and in sync with updates
    Vector3p    _estimate_pos_ned_m;            // Estimated target position (NED frame, meters)
    Vector3f    _estimate_vel_ned_ms;           // Estimated target velocity (NED frame, m/s)
    Vector3f    _estimate_accel_ned_mss;        // Estimated target acceleration (NED frame, m/s²)
    float       _estimate_heading_rad;          // Estimated heading (radians, range -PI to PI)
    float       _estimate_heading_rate_rads;    // Estimated heading rate (rad/s)
    float       _estimate_heading_accel_radss;  // Estimated heading acceleration (rad/s²)

    Vector3p    _ofs_estimate_pos_ned_m;        // Estimated position with offsets applied (NED frame)
    Vector3f    _ofs_estimate_vel_ned_ms;       // Estimated velocity with offsets applied (NED frame)
    Vector3f    _ofs_estimate_accel_ned_mss;    // Estimated acceleration with offsets applied (NED frame)

    bool        _automatic_sysid;               // True if target sysid was automatically selected
    int16_t     _sysid_used;                    // Currently active sysid used for updates
    float       _dist_to_target_m;              // Horizontal distance to target, for reporting (meters)
    float       _bearing_to_target_deg;         // Bearing to target from vehicle (degrees, 0 = North)
    bool        _offsets_were_zero;             // True if initial offset was zero before being initialized
    bool        _using_follow_target;           // True if FOLLOW_TARGET messages are being used instead of GLOBAL_POSITION_INT

    HAL_Semaphore   _follow_sem;                // semaphore for multi-thread use of update_estimates and LUA calls

    //==========================================================================
    // Utilities
    //==========================================================================

    // Jitter correction helper for smoothing offboard timestamps.
    JitterCorrection _jitter{500};
};

//==============================================================================
// AP_Follow Accessor
//==============================================================================

namespace AP {
    AP_Follow &follow();
};

#endif
