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
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>
#include <AC_PID/AC_P.h>

class AP_Follow
{

public:

    // enum for YAW_BEHAVE parameter
    enum YawBehave {
        YAW_BEHAVE_NONE = 0,
        YAW_BEHAVE_FACE_LEAD_VEHICLE = 1,
        YAW_BEHAVE_SAME_AS_LEAD_VEHICLE = 2,
        YAW_BEHAVE_DIR_OF_FLIGHT = 3
    };

    // constructor
    AP_Follow();

    // set which target to follow
    void set_target_sysid(uint8_t sysid) { _sysid = sysid; }

    //
    // position tracking related methods
    //

    // true if we have a valid target location estimate
    bool have_target() const;

    // get target's estimated location and velocity (in NED)
    bool get_target_location_and_velocity(Location &loc, Vector3f &vel_ned) const;

    // get distance vector to target (in meters), target plus offsets, and target's velocity all in NED frame
    bool get_target_dist_and_vel_ned(Vector3f &dist_ned, Vector3f &dist_with_ofs, Vector3f &vel_ned);

    // get position controller.  this controller is not used within this library but it is convenient to hold it here
    const AC_P& get_pos_p() const { return _p_pos; }

    //
    // yaw/heading related methods
    //

    // get user defined yaw behaviour
    YawBehave get_yaw_behave() const { return (YawBehave)_yaw_behave.get(); }

    // get target's heading in degrees (0 = north, 90 = east)
    bool get_target_heading(float &heading) const;

    // parse mavlink messages which may hold target's position, velocity and attitude
    void handle_msg(const mavlink_message_t &msg);

    // parameter list
    static const struct AP_Param::GroupInfo var_info[];

    // returns true if library is enabled
    bool enabled() const { return _enabled; }

private:

    // get velocity estimate in m/s in NED frame using dt since last update
    bool get_velocity_ned(Vector3f &vel_ned, float dt) const;

    // initialise offsets to provided distance vector (in meters in NED frame) if required
    void init_offsets_if_required(const Vector3f &dist_vec_ned);

    // get offsets in meters in NED frame
    bool get_offsets_ned(Vector3f &offsets) const;

    // parameters
    AP_Int8     _enabled;           // 1 if this subsystem is enabled
    AP_Int16    _sysid;             // target's mavlink system id (0 to use first sysid seen)
    AP_Float    _dist_max;          // maximum distance to target.  targets further than this will be ignored
    AP_Int8     _offset_type;       // offset frame type (0:North-East-Down, 1:RelativeToLeadVehicleHeading)
    AP_Vector3f _offset;            // offset from lead vehicle in meters
    AP_Int8     _yaw_behave;        // following vehicle's yaw/heading behaviour
    AP_Int8     _alt_type;          // altitude source for follow mode
    AC_P        _p_pos;             // position error P controller

    // local variables
    bool _healthy;                  // true if we are receiving mavlink messages (regardless of whether they have target position info within them)
    uint8_t _sysid_to_follow = 0;   // mavlink system id of vehicle to follow
    uint32_t _last_location_update_ms;  // system time of last position update
    Location _target_location;      // last known location of target
    Vector3f _target_velocity_ned;  // last known velocity of target in NED frame in m/s
    Vector3f _target_accel_ned;     // last known acceleration of target in NED frame in m/s/s
    uint32_t _last_heading_update_ms;   // system time of last heading update
    float _target_heading;          // heading in degrees
    uint32_t _last_location_sent_to_gcs; // last time GCS was told position
};
