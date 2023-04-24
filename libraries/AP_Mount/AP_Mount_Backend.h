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

/*
  Mount driver backend class. Each supported mount type
  needs to have an object derived from this class.
 */
#pragma once

#include "AP_Mount.h"
#if HAL_MOUNT_ENABLED
#include <AP_Common/AP_Common.h>
#include <RC_Channel/RC_Channel.h>

class AP_Mount_Backend
{
public:
    // Constructor
    AP_Mount_Backend(AP_Mount &frontend, AP_Mount_Params &params, uint8_t instance) :
        _frontend(frontend),
        _params(params),
        _instance(instance)
    {}

    // init - performs any required initialisation for this instance
    virtual void init();

    // update mount position - should be called periodically
    virtual void update() = 0;

    // used for gimbals that need to read INS data at full rate
    virtual void update_fast() {}

    // return true if healthy
    virtual bool healthy() const { return true; }

    // return true if this mount accepts roll or pitch targets
    virtual bool has_roll_control() const;
    virtual bool has_pitch_control() const;

    // returns true if this mount can control its pan (required for multicopters)
    virtual bool has_pan_control() const = 0;

    // get attitude as a quaternion.  returns true on success.
    // att_quat will be an earth-frame quaternion rotated such that
    // yaw is in body-frame.
    virtual bool get_attitude_quaternion(Quaternion& att_quat) = 0;

    // get mount's mode
    enum MAV_MOUNT_MODE get_mode() const { return _mode; }

    // set mount's mode
    void set_mode(enum MAV_MOUNT_MODE mode) { _mode = mode; }

    // set yaw_lock.  If true, the gimbal's yaw target is maintained in earth-frame meaning it will lock onto an earth-frame heading (e.g. North)
    // If false (aka "follow") the gimbal's yaw is maintained in body-frame meaning it will rotate with the vehicle
    void set_yaw_lock(bool yaw_lock) { _yaw_lock = yaw_lock; }

    // set angle target in degrees
    // yaw_is_earth_frame (aka yaw_lock) should be true if yaw angle is earth-frame, false if body-frame
    void set_angle_target(float roll_deg, float pitch_deg, float yaw_deg, bool yaw_is_earth_frame);

    // sets rate target in deg/s
    // yaw_lock should be true if the yaw rate is earth-frame, false if body-frame (e.g. rotates with body of vehicle)
    void set_rate_target(float roll_degs, float pitch_degs, float yaw_degs, bool yaw_is_earth_frame);

    // set_roi_target - sets target location that mount should attempt to point towards
    void set_roi_target(const Location &target_loc);
    // clear_roi_target - clears target location that mount should attempt to point towards
    void clear_roi_target();

    // set_sys_target - sets system that mount should attempt to point towards
    void set_target_sysid(uint8_t sysid);

    // handle do_mount_control command.  Returns MAV_RESULT_ACCEPTED on success
    MAV_RESULT handle_command_do_mount_control(const mavlink_command_long_t &packet);
    
    // process MOUNT_CONFIGURE messages received from GCS. deprecated.
    void handle_mount_configure(const mavlink_mount_configure_t &msg);

    // process MOUNT_CONTROL messages received from GCS. deprecated.
    void handle_mount_control(const mavlink_mount_control_t &packet);

    // send a GIMBAL_DEVICE_ATTITUDE_STATUS message to GCS
    void send_gimbal_device_attitude_status(mavlink_channel_t chan);

    // return gimbal capabilities sent to GCS in the GIMBAL_MANAGER_INFORMATION
    virtual uint32_t get_gimbal_manager_capability_flags() const;

    // send a GIMBAL_MANAGER_INFORMATION message to GCS
    void send_gimbal_manager_information(mavlink_channel_t chan);

    // handle a GIMBAL_REPORT message
    virtual void handle_gimbal_report(mavlink_channel_t chan, const mavlink_message_t &msg) {}

    // handle a PARAM_VALUE message
    virtual void handle_param_value(const mavlink_message_t &msg) {}

    // handle a GLOBAL_POSITION_INT message
    bool handle_global_position_int(uint8_t msg_sysid, const mavlink_global_position_int_t &packet);

    // handle GIMBAL_DEVICE_INFORMATION message
    virtual void handle_gimbal_device_information(const mavlink_message_t &msg) {}

    // handle GIMBAL_DEVICE_ATTITUDE_STATUS message
    virtual void handle_gimbal_device_attitude_status(const mavlink_message_t &msg) {}

    // accessors for scripting backends
    virtual bool get_rate_target(float& roll_degs, float& pitch_degs, float& yaw_degs, bool& yaw_is_earth_frame) { return false; }
    virtual bool get_angle_target(float& roll_deg, float& pitch_deg, float& yaw_deg, bool& yaw_is_earth_frame) { return false; }
    virtual bool get_location_target(Location &target_loc) { return false; }
    virtual void set_attitude_euler(float roll_deg, float pitch_deg, float yaw_bf_deg) {};

    //
    // camera controls for gimbals that include a camera
    //

    // take a picture.  returns true on success
    virtual bool take_picture() { return false; }

    // start or stop video recording.  returns true on success
    // set start_recording = true to start record, false to stop recording
    virtual bool record_video(bool start_recording) { return false; }

    // set zoom specified as a rate or percentage
    virtual bool set_zoom(ZoomType zoom_type, float zoom_value) { return false; }

    // set focus in, out or hold.  returns true on success
    // focus in = -1, focus hold = 0, focus out = 1
    virtual bool set_manual_focus_step(int8_t focus_step) { return false; }

    // auto focus.  returns true on success
    virtual bool set_auto_focus() { return false; }

protected:

    enum class MountTargetType {
        ANGLE,
        RATE,
    };

    // structure for a single angle or rate target
    struct MountTarget {
        float roll;
        float pitch;
        float yaw;
        bool yaw_is_ef;
    };

    // returns true if user has configured a valid yaw angle range
    // allows user to disable yaw even on 3-axis gimbal
    bool yaw_range_valid() const { return (_params.yaw_angle_min < _params.yaw_angle_max); }

    // returns true if mavlink heartbeat should be suppressed for this gimbal (only used by Solo gimbal)
    virtual bool suppress_heartbeat() const { return false; }

    // get pilot input (in the range -1 to +1) received through RC
    void get_rc_input(float& roll_in, float& pitch_in, float& yaw_in) const;

    // get rate targets (in rad/s) from pilot RC
    // returns true on success (RC is providing rate targets), false on failure (RC is providing angle targets)
    bool get_rc_rate_target(MountTarget& rate_rads) const WARN_IF_UNUSED;

    // get angle targets (in radians) from pilot RC
    // returns true on success (RC is providing angle targets), false on failure (RC is providing rate targets)
    bool get_rc_angle_target(MountTarget& angle_rad) const WARN_IF_UNUSED;

    // get angle targets (in radians) to a Location
    // returns true on success, false on failure
    bool get_angle_target_to_location(const Location &loc, MountTarget& angle_rad) const WARN_IF_UNUSED;

    // get angle targets (in radians) to ROI location
    // returns true on success, false on failure
    bool get_angle_target_to_roi(MountTarget& angle_rad) const WARN_IF_UNUSED;

    // get angle targets (in radians) to home location
    // returns true on success, false on failure
    bool get_angle_target_to_home(MountTarget& angle_rad) const WARN_IF_UNUSED;

    // get angle targets (in radians) to a vehicle with sysid of _target_sysid
    // returns true on success, false on failure
    bool get_angle_target_to_sysid(MountTarget& angle_rad) const WARN_IF_UNUSED;

    // return body-frame yaw angle from a mount target
    float get_bf_yaw_angle(const MountTarget& angle_rad) const;

    // return earth-frame yaw angle from a mount target
    float get_ef_yaw_angle(const MountTarget& angle_rad) const;

    // update angle targets using a given rate target
    // the resulting angle_rad yaw frame will match the rate_rad yaw frame
    // assumes a 50hz update rate
    void update_angle_target_from_rate(const MountTarget& rate_rad, MountTarget& angle_rad) const;

    // helper function to provide GIMBAL_DEVICE_FLAGS for use in GIMBAL_DEVICE_ATTITUDE_STATUS message
    uint16_t get_gimbal_device_flags() const;

    // sent warning to GCS
    void send_warning_to_GCS(const char* warning_str);

    AP_Mount    &_frontend; // reference to the front end which holds parameters
    AP_Mount_Params &_params; // parameters for this backend
    uint8_t     _instance;  // this instance's number

    MAV_MOUNT_MODE  _mode;          // current mode (see MAV_MOUNT_MODE enum)
    bool _yaw_lock;                 // True if the gimbal's yaw target is maintained in earth-frame, if false (aka "follow") it is maintained in body-frame

    // structure for MAVLink Targeting angle and rate targets
    struct {
        MountTargetType target_type;// MAVLink targeting mode's current target type (e.g. angle or rate)
        MountTarget angle_rad;      // angle target in radians
        MountTarget rate_rads;      // rate target in rad/s
    } mavt_target;

    Location _roi_target;           // roi target location
    bool _roi_target_set;           // true if the roi target has been set

    uint8_t _target_sysid;          // sysid to track
    Location _target_sysid_location;// sysid target location
    bool _target_sysid_location_set;// true if _target_sysid has been set

    uint32_t _last_warning_ms;      // system time of last warning sent to GCS
};

#endif // HAL_MOUNT_ENABLED
