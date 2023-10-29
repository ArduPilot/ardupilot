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

#include "AP_Mount_config.h"

#if HAL_MOUNT_ENABLED

#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_Camera/AP_Camera_shareddefs.h>
#include "AP_Mount.h"

class AP_Mount_Backend
{
public:
    // Constructor
    AP_Mount_Backend(class AP_Mount &frontend, class AP_Mount_Params &params, uint8_t instance) :
        _frontend(frontend),
        _params(params),
        _instance(instance)
    {}

    // init - performs any required initialisation for this instance
    virtual void init();

    // set device id of this instance, for MNTx_DEVID parameter
    void set_dev_id(uint32_t id);

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
    MAV_RESULT handle_command_do_mount_control(const mavlink_command_int_t &packet);

    // handle do_gimbal_manager_configure.  Returns MAV_RESULT_ACCEPTED on success
    // requires original message in order to extract caller's sysid and compid
    MAV_RESULT handle_command_do_gimbal_manager_configure(const mavlink_command_int_t &packet, const mavlink_message_t &msg);

#if AP_MAVLINK_MSG_MOUNT_CONFIGURE_ENABLED
    // process MOUNT_CONFIGURE messages received from GCS. deprecated.
    void handle_mount_configure(const mavlink_mount_configure_t &msg);
#endif

#if AP_MAVLINK_MSG_MOUNT_CONTROL_ENABLED
    // process MOUNT_CONTROL messages received from GCS. deprecated.
    void handle_mount_control(const mavlink_mount_control_t &packet);
#endif

    // send a GIMBAL_DEVICE_ATTITUDE_STATUS message to GCS
    void send_gimbal_device_attitude_status(mavlink_channel_t chan);

    // return gimbal capabilities sent to GCS in the GIMBAL_MANAGER_INFORMATION
    virtual uint32_t get_gimbal_manager_capability_flags() const;

    // send a GIMBAL_MANAGER_INFORMATION message to GCS
    void send_gimbal_manager_information(mavlink_channel_t chan);

    // send a GIMBAL_MANAGER_STATUS message to GCS
    void send_gimbal_manager_status(mavlink_channel_t chan);

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

    // get target rate in deg/sec. returns true on success
    bool get_rate_target(float& roll_degs, float& pitch_degs, float& yaw_degs, bool& yaw_is_earth_frame);

    // get target angle in deg. returns true on success
    bool get_angle_target(float& roll_deg, float& pitch_deg, float& yaw_deg, bool& yaw_is_earth_frame);

    // accessors for scripting backends
    virtual bool get_location_target(Location &target_loc) { return false; }
    virtual void set_attitude_euler(float roll_deg, float pitch_deg, float yaw_bf_deg) {};

    // write mount log packet
    void write_log(uint64_t timestamp_us);

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

    // set focus specified as rate, percentage or auto
    // focus in = -1, focus hold = 0, focus out = 1
    virtual SetFocusResult set_focus(FocusType focus_type, float focus_value) { return SetFocusResult::UNSUPPORTED; }

    // set tracking to none, point or rectangle (see TrackingType enum)
    // if POINT only p1 is used, if RECTANGLE then p1 is top-left, p2 is bottom-right
    // p1,p2 are in range 0 to 1.  0 is left or top, 1 is right or bottom
    virtual bool set_tracking(TrackingType tracking_type, const Vector2f& p1, const Vector2f& p2) { return false; }

    // set camera lens as a value from 0 to 5
    virtual bool set_lens(uint8_t lens) { return false; }

    // send camera information message to GCS
    virtual void send_camera_information(mavlink_channel_t chan) const {}

    // send camera settings message to GCS
    virtual void send_camera_settings(mavlink_channel_t chan) const {}

    //
    // rangefinder
    //

    // get rangefinder distance.  Returns true on success
    virtual bool get_rangefinder_distance(float& distance_m) const { return false; }

protected:

    enum class MountTargetType {
        ANGLE,
        RATE,
    };

    // class for a single angle or rate target
    class MountTarget {
    public:
        float roll;
        float pitch;
        float yaw;
        bool yaw_is_ef;

        // return body-frame yaw angle from a mount target (in radians)
        float get_bf_yaw() const;

        // return earth-frame yaw angle from a mount target (in radians)
        float get_ef_yaw() const;

        // set roll, pitch, yaw and yaw_is_ef from Vector3f
        void set(const Vector3f& rpy, bool yaw_is_ef_in);
    };

    // returns true if user has configured a valid yaw angle range
    // allows user to disable yaw even on 3-axis gimbal
    bool yaw_range_valid() const { return (_params.yaw_angle_min < _params.yaw_angle_max); }

    // returns true if mavlink heartbeat should be suppressed for this gimbal (only used by Solo gimbal)
    virtual bool suppress_heartbeat() const { return false; }

    // get pilot input (in the range -1 to +1) received through RC
    void get_rc_input(float& roll_in, float& pitch_in, float& yaw_in) const;

    // get angle or rate targets from pilot RC
    // target_type will be either ANGLE or RATE, rpy will be the target angle in deg or rate in deg/s
    void get_rc_target(MountTargetType& target_type, MountTarget& rpy) const;

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
    } mnt_target;

    Location _roi_target;           // roi target location
    bool _roi_target_set;           // true if the roi target has been set

    uint8_t _target_sysid;          // sysid to track
    Location _target_sysid_location;// sysid target location
    bool _target_sysid_location_set;// true if _target_sysid has been set

    uint32_t _last_warning_ms;      // system time of last warning sent to GCS

    // structure holding mavlink sysid and compid of controller of this gimbal
    // see MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE and GIMBAL_MANAGER_STATUS
    struct {
        uint8_t sysid;
        uint8_t compid;
    } mavlink_control_id;
};

#endif // HAL_MOUNT_ENABLED
