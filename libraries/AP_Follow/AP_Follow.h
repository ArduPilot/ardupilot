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
#include <AP_Camera/AP_Camera_config.h>

class AP_Follow
{

public:

    // enum for FOLLOW_OPTIONS parameter
    enum class Option {
        MOUNT_FOLLOW_ON_ENTER = 1,
#if AP_CAMERA_OFFBOARD_TRACKING_ENABLED
        OBJECT_FOLLOW_ON_ENTER = 2
#endif
    };

    // enum for YAW_BEHAVE parameter
    enum YawBehave {
        YAW_BEHAVE_NONE = 0,
        YAW_BEHAVE_FACE_LEAD_VEHICLE = 1,
        YAW_BEHAVE_SAME_AS_LEAD_VEHICLE = 2,
        YAW_BEHAVE_DIR_OF_FLIGHT = 3,
#if AP_CAMERA_OFFBOARD_TRACKING_ENABLED
        YAW_BEHAVE_OBJECT_FOLLOW = 4
#endif
    };

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

    // restore offsets to zero if necessary, should be called when vehicle exits follow mode
    void clear_offsets_if_required();

    //
    // position tracking related methods
    //

    // true if we have a valid target location estimate
    bool have_target() const;

    // get target's estimated location and velocity (in NED)
    bool get_target_location_and_velocity(Location &loc, Vector3f &vel_ned);

    // get target's estimated location and velocity (in NED), with offsets added
    bool get_target_location_and_velocity_ofs(Location &loc, Vector3f &vel_ned);
    
    // get distance vector to target (in meters), target plus offsets, and target's velocity all in NED frame
    bool get_target_dist_and_vel_ned(Vector3f &dist_ned, Vector3f &dist_with_ofs, Vector3f &vel_ned);

    // get target sysid
    uint8_t get_target_sysid() const { return _sysid.get(); }

    // get position controller.  this controller is not used within this library but it is convenient to hold it here
    const AC_P& get_pos_p() const { return _p_pos; }

    //
    // yaw/heading related methods
    //

    // get user defined yaw behaviour
    YawBehave get_yaw_behave() const { return (YawBehave)_yaw_behave.get(); }

#if AP_CAMERA_OFFBOARD_TRACKING_ENABLED
    // get object follow yaw reset 
    float get_object_follow_yaw_reset() {float ret = _object_follow_yaw_reset.get(); return ret; }

    // get object follow pitch reset 
    float get_object_follow_pitch_reset() {float ret = _object_follow_pitch_reset.get(); return ret; }

    // Get the last safe yaw value such that the gimbal wont acheve its limits during the object follow
    bool get_yaw_under_gimbal_limit_to_point_to(float &yaw) { 
        if (yaw_under_gimbal_limit_to_point_to_is_valid) {
            yaw = yaw_under_gimbal_limit_to_point_to;
            return true;
        }
        return false; 
    }
    void set_yaw_under_gimbal_limit_to_point_to(float yaw_cd) { yaw_under_gimbal_limit_to_point_to = yaw_cd; yaw_under_gimbal_limit_to_point_to_is_valid = true; }
#endif

    // get target's heading in degrees (0 = north, 90 = east)
    bool get_target_heading_deg(float &heading) const;

    // parse mavlink messages which may hold target's position, velocity and attitude
    void handle_msg(const mavlink_message_t &msg);

    //
    // GCS reporting functions
    //

    // get horizontal distance to target (including offset) in meters (for reporting purposes)
    float get_distance_to_target() const { return _dist_to_target; }

    // get bearing to target (including offset) in degrees (for reporting purposes)
    float get_bearing_to_target() const { return _bearing_to_target; }

    // get system time of last position update
    uint32_t get_last_update_ms() const { return _last_location_update_ms; }

    // returns true if a follow option enabled
    bool option_is_enabled(Option option) const { return (_options.get() & (uint16_t)option) != 0; }

    // parameter list
    static const struct AP_Param::GroupInfo var_info[];

private:
    static AP_Follow *_singleton;

    // returns true if we should extract information from msg
    bool should_handle_message(const mavlink_message_t &msg) const;

    // get velocity estimate in m/s in NED frame using dt since last update
    bool get_velocity_ned(Vector3f &vel_ned, float dt) const;

    // initialise offsets to provided distance vector to other vehicle (in meters in NED frame) if required
    void init_offsets_if_required(const Vector3f &dist_vec_ned);

    // get offsets in meters in NED frame
    bool get_offsets_ned(Vector3f &offsets) const;

    // rotate 3D vector clockwise by specified angle (in degrees)
    Vector3f rotate_vector(const Vector3f &vec, float angle_deg) const;

    // set recorded distance and bearing to target to zero
    void clear_dist_and_bearing_to_target();

    // handle various mavlink messages supplying position:
    bool handle_global_position_int_message(const mavlink_message_t &msg);
    bool handle_follow_target_message(const mavlink_message_t &msg);

    // write out an onboard-log message to help diagnose follow problems:
    void Log_Write_FOLL();

    // parameters
    AP_Int8     _enabled;           // 1 if this subsystem is enabled
    AP_Int16    _sysid;             // target's mavlink system id (0 to use first sysid seen)
    AP_Float    _dist_max;          // maximum distance to target.  targets further than this will be ignored
    AP_Int8     _offset_type;       // offset frame type (0:North-East-Down, 1:RelativeToLeadVehicleHeading)
    AP_Vector3f _offset;            // offset from lead vehicle in meters
    AP_Int8     _yaw_behave;        // following vehicle's yaw/heading behaviour (see YAW_BEHAVE enum)
    AP_Int8     _alt_type;          // altitude source for follow mode
    AC_P        _p_pos;             // position error P controller
    AP_Int16    _options;           // options for mount behaviour follow mode
#if AP_CAMERA_OFFBOARD_TRACKING_ENABLED
    AP_Float    _object_follow_margin; // object follow margin from the center of the frame
    AP_Float    _poi_delay;
    AP_Float    _object_follow_yaw_reset;
    AP_Float    _object_follow_pitch_reset;
#endif

    // local variables
    uint32_t _last_location_update_ms;  // system time of last position update
    Location _target_location;      // last known location of target
    Vector3f _target_velocity_ned;  // last known velocity of target in NED frame in m/s
    Vector3f _target_accel_ned;     // last known acceleration of target in NED frame in m/s/s
    uint32_t _last_heading_update_ms;   // system time of last heading update
    float _target_heading;          // heading in degrees
    bool _automatic_sysid;          // did we lock onto a sysid automatically?
    float _dist_to_target;          // latest distance to target in meters (for reporting purposes)
    float _bearing_to_target;       // latest bearing to target in degrees (for reporting purposes)
    bool _offsets_were_zero;        // true if offsets were originally zero and then initialised to the offset from lead vehicle
    Location _last_object_location; // last location of the object
#if AP_CAMERA_OFFBOARD_TRACKING_ENABLED
    bool _last_location_valid = false;
    uint32_t _last_poi_update_ms;       // system time of last POI update
    float yaw_under_gimbal_limit_to_point_to;
    bool yaw_under_gimbal_limit_to_point_to_is_valid = false;
#endif

    // setup jitter correction with max transport lag of 3s
    JitterCorrection _jitter{3000};
};

namespace AP {
    AP_Follow &follow();
};

#endif
