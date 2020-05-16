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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <AC_PID/AC_P.h>
#include <AP_RTC/JitterCorrection.h>

class AP_Follow
{

public:
    // constructor
    AP_Follow();

    /* Do not allow copies */
    AP_Follow(const AP_Follow &other) = delete;
    AP_Follow &operator=(const AP_Follow&) = delete;

    // get singleton
    static AP_Follow *get_singleton() {
        return _singleton;
    }


    enum class TargetType {
        SYSID       = 0,
        GIMBAL1,
//        GIMBAL1_ROI,
        GIMBAL2,
//        GIMBAL2_ROI,
        MAX_SIZE,
    };

    // enum for YAW_BEHAVE parameter
    enum YawBehave {
        YAW_BEHAVE_NONE = 0,
        YAW_BEHAVE_FACE_LEAD_VEHICLE = 1,
        YAW_BEHAVE_SAME_AS_LEAD_VEHICLE = 2,
        YAW_BEHAVE_DIR_OF_FLIGHT = 3
    };

    // returns true if library is enabled
    bool enabled() const { return _enabled; }

    // set which target to follow
    void set_target_sysid(uint8_t sysid) { _sysid = sysid; }

    // restore offsets to zero if necessary, should be called when vehicle exits follow mode
    void clear_offsets_if_required();

    //
    // position tracking related methods
    //
    // get target's estimated location and velocity (in NED)
//    bool get_target_location(Location &loc) { return get_target_location((TargetType)_type.get(), loc); }
//    bool get_target_location(TargetType type, Location &loc) const;

    // get target's estimated location and velocity (in NED)
    bool get_target_location_and_velocity(Location &loc, Vector3f &vel_ned) { return get_target_location_and_velocity((TargetType)_type.get(), loc, vel_ned); }
    bool get_target_location_and_velocity(TargetType type, Location &loc, Vector3f &vel_ned) const;

    // set target's location and velocity (in NED) or a MAVLink packet
    void set_target_location_and_velocity(Location &loc, Vector3f &vel_ned, uint32_t timestamp = 0) { return set_target_location_and_velocity((TargetType)_type.get(), loc, vel_ned, timestamp); }
    void set_target_location_and_velocity(TargetType type, Location &loc, Vector3f &vel_ned, uint32_t timestamp = 0);

    // get distance vector to target (in meters), target plus offsets, and target's velocity all in NED frame
    bool get_target_dist_and_vel_ned(Vector3f &dist_ned, Vector3f &dist_with_ofs, Vector3f &vel_ned)  { return get_target_dist_and_vel_ned((TargetType)_type.get(), dist_ned, dist_with_ofs, vel_ned); }
    bool get_target_dist_and_vel_ned(TargetType type, Vector3f &dist_ned, Vector3f &dist_with_ofs, Vector3f &vel_ned);

    // get position controller.  this controller is not used within this library but it is convenient to hold it here
    const AC_P& get_pos_p() const { return _p_pos; }

    //
    // yaw/heading related methods
    //

    // get user defined yaw behaviour
    YawBehave get_yaw_behave() const { return (YawBehave)_yaw_behave.get(); }

    // get target's heading in degrees (0 = north, 90 = east)
    bool get_target_heading_deg(float &heading) const { return get_target_heading_deg((TargetType)_type.get(), heading); }
    bool get_target_heading_deg(TargetType type, float &heading) const;

    // parse mavlink messages which may hold target's position, velocity and attitude
    void handle_msg(const mavlink_message_t &msg);

    //
    // GCS reporting functions
    //

    // get horizontal distance to target (including offset) in meters (for reporting purposes)
    float get_distance_to_target() const { return get_distance_to_target((TargetType)_type.get()); }
    float get_distance_to_target(TargetType type) const { return _targets[uint8_t(type)].dist_to_target; }

    // get bearing to target (including offset) in degrees (for reporting purposes)
    float get_bearing_to_target() const { return get_bearing_to_target((TargetType)_type.get()); }
    float get_bearing_to_target(TargetType type) const { return _targets[uint8_t(type)].bearing_to_target; }

    // get offsets in meters in NED frame
    bool get_offsets_ned(Vector3f &offsets) const { return get_offsets_ned((TargetType)_type.get(), offsets); }

    // get offsets in meters in NED frame
    void set_alt_offset(float alt) {
        Vector3f offset = _offset.get();
        offset.z = alt;
        _offset.set(offset);
    }


    // parameter list
    static const struct AP_Param::GroupInfo var_info[];

private:
    // singleton
    static AP_Follow *_singleton;

    // get velocity estimate in m/s in NED frame using dt since last update
    bool get_velocity_ned(TargetType type, Vector3f &vel_ned, float dt) const;

    // initialise offsets to provided distance vector to other vehicle (in meters in NED frame) if required
    void init_offsets_if_required(TargetType type, const Vector3f &dist_vec_ned);

    // get offsets in meters in NED frame
    bool get_offsets_ned(TargetType type, Vector3f &offsets) const;

    // rotate 3D vector clockwise by specified angle (in degrees)
    Vector3f rotate_vector(const Vector3f &vec, float angle_deg) const;

    // set recorded distance and bearing to target to zero
    void clear_dist_and_bearing_to_target(TargetType type);

    // write a log entry for the target
    void log_target(TargetType type);

    void handle_packet(mavlink_follow_target_t &packet);
    void handle_packet(mavlink_global_position_int_t &packet);

    // parameters
    AP_Int8     _enabled;           // 1 if this subsystem is enabled
    AP_Int8     _type;              // type of target to follow
    AP_Int16    _sysid;             // target's mavlink system id (0 to use first sysid seen)
    AP_Float    _dist_max;          // maximum distance to target.  targets further than this will be ignored
    AP_Int8     _offset_type;       // offset frame type (0:North-East-Down, 1:RelativeToLeadVehicleHeading)
    AP_Vector3f _offset;            // offset from lead vehicle in meters
    AP_Int8     _yaw_behave;        // following vehicle's yaw/heading behaviour (see YAW_BEHAVE enum)
    AP_Int8     _alt_type;          // altitude source for follow mode
    AC_P        _p_pos;             // position error P controller
    bool        _automatic_sysid;          // did we lock onto a sysid automatically?
    bool        _offsets_were_zero;        // true if offsets were originally zero and then initialised to the offset from lead vehicle
    AP_Float    _standoff_distance; // distance in meters to keep waypoint away from target. Exclusion radius.

    struct  {
        uint32_t last_location_update_ms;  // system time of last position update
        Location location;             // last known location of target
        Vector3f velocity_ned;         // last known velocity of target in NED frame in m/s
        Vector3f accel_ned;            // last known acceleration of target in NED frame in m/s/s
        uint32_t last_heading_update_ms;   // system time of last heading update
        float heading;                 // heading in degrees
        float dist_to_target;          // latest distance to target in meters (for reporting purposes)
        float bearing_to_target;       // latest bearing to target in degrees (for reporting purposes)
    } _targets[uint8_t(TargetType::MAX_SIZE)];

    // setup jitter correction with max transport lag of 3s
    JitterCorrection _jitter[uint8_t(TargetType::MAX_SIZE)] {3000}; // NOTE: this initializes _max_lag_ms=3000 only for SYSID and other indexes are default
};


namespace AP {
AP_Follow *follow();
};

