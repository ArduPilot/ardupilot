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
#include <SRV_Channel/SRV_Channel.h>
#include "AP_Mount.h"

class AP_Mount_Backend
{
public:
    // Constructor
    AP_Mount_Backend(class AP_Mount &frontend, class AP_Mount_Params &params, uint8_t instance) :
        _frontend(frontend),
        _params(params),
        _instance(instance),
        _open_idx(_instance == 0? SRV_Channel::k_mount_open : SRV_Channel::k_mount2_open)
    {}

    // init - performs any required initialisation for this instance
    virtual void init();

    // set device id of this instance, for MNTx_DEVID parameter
    void set_dev_id(uint32_t id);

    // update mount position - should be called periodically
    virtual void update();

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

    // get angular velocity of mount. Only available on some backends
    virtual bool get_angular_velocity(Vector3f& rates) { return false; }

    // returns true if mode is a valid mode, false otherwise:
    bool valid_mode(MAV_MOUNT_MODE mode) const;

    // get mount's mode
    enum MAV_MOUNT_MODE get_mode() const { return _mode; }

    // set mount's mode
    bool set_mode(enum MAV_MOUNT_MODE mode);

    // set yaw_lock used in RC_TARGETING mode.  If true, the gimbal's yaw target is maintained in earth-frame meaning it will lock onto an earth-frame heading (e.g. North)
    // If false (aka "follow") the gimbal's yaw is maintained in body-frame meaning it will rotate with the vehicle
    void set_yaw_lock(bool yaw_lock);

    // set pitch_lock used in RC_TARGETING mode.  If true, the gimbal's tilt target is maintained in earth-frame meaning it will lock onto an earth-frame
    // If false (aka "follow") the gimbal's tilt is maintained in body-frame meaning it will tilt with the vehicle
    void set_pitch_lock(bool pitch_lock) { _pitch_lock = pitch_lock; }

    // set roll_lock used in RC_TARGETING mode.  If true, the gimbal's roll target is maintained in earth-frame meaning it will lock onto an earth-frame
    // If false (aka "follow") the gimbal's tilt is maintained in body-frame meaning it will roll with the vehicle
    void set_roll_lock(bool roll_lock) { _roll_lock = roll_lock; }

    // set angle target in degrees
    // roll and pitch are in earth-frame
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

#if AP_SCRIPTING_ENABLED
    // get mount target location. returns true on success
    bool get_location_target(Location &target_loc);
#endif

    // accessors for scripting backends
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

#if HAL_MOUNT_SET_CAMERA_SOURCE_ENABLED
    // set_camera_source is functionally the same as set_lens except primary and secondary lenses are specified by type
    // primary and secondary sources use the AP_Camera::CameraSource enum cast to uint8_t
    virtual bool set_camera_source(uint8_t primary_source, uint8_t secondary_source) { return false; }
#endif

    // send camera information message to GCS
    virtual void send_camera_information(mavlink_channel_t chan) const {}

    // send camera settings message to GCS
    virtual void send_camera_settings(mavlink_channel_t chan) const {}

    // send camera capture status message to GCS
    virtual void send_camera_capture_status(mavlink_channel_t chan) const {}

#if AP_MOUNT_SEND_THERMAL_RANGE_ENABLED
    // send camera thermal status message to GCS
    virtual void send_camera_thermal_range(mavlink_channel_t chan) const {}
#endif

    // change camera settings not normally used by autopilot
    virtual bool change_setting(CameraSetting setting, float value) { return false; }

#if AP_MOUNT_POI_TO_LATLONALT_ENABLED
    // get poi information.  Returns true on success and fills in gimbal attitude, location and poi location
    bool get_poi(uint8_t instance, Quaternion &quat, Location &loc, Location &poi_loc);
#endif

    //
    // rangefinder
    //

    // get rangefinder distance.  Returns true on success
    virtual bool get_rangefinder_distance(float& distance_m) const { return false; }

    // enable/disable rangefinder.  Returns true on success
    virtual bool set_rangefinder_enable(bool enable) { return false; }

protected:

    enum class MountTargetType {
        ANGLE     = 0,
        RATE      = 1,
        RETRACTED = 2,
        NEUTRAL   = 3,
    };

    // class for a single angle or rate target
    class MountAngleTarget {
    public:
        float roll;
        float pitch;
        float yaw;
        bool pitch_is_ef; //only changed and used by CADDX currently
        bool roll_is_ef; //only changed and used  by CADDX currently
        bool yaw_is_ef;

        // return body-frame yaw angle from a mount target (in radians)
        float get_bf_yaw() const;

        // return earth-frame yaw angle from a mount target (in radians)
        float get_ef_yaw() const;

        // set roll, pitch, yaw and yaw_is_ef from Vector3f
        void set(const Vector3f& rpy, bool yaw_is_ef_in);
    };
    class MountRateTarget {
    public:
        float roll;      // roll rate in radians/second
        float pitch;     // roll rate in radians/second
        float yaw;       // roll rate in radians/second
        bool yaw_is_ef;  // if set then `yaw` is a rate in earth frame
    };

    // returns a bitmask of MountTargetTypes which this backend supports
    // FIXME: make this pure-virtual
    virtual uint8_t natively_supported_mount_target_types() const { return 0; };

    // some static const masks to try to make the backends easier to read:
    static constexpr uint8_t NATIVE_ANGLES_ONLY = (1U << uint8_t(MountTargetType::ANGLE));
    static constexpr uint8_t NATIVE_RATES_ONLY = (1U << uint8_t(MountTargetType::RATE));
    static constexpr uint8_t NATIVE_ANGLES_AND_RATES_ONLY = (
        1U << uint8_t(MountTargetType::ANGLE) |
        1U << uint8_t(MountTargetType::RATE)
    );

    // returns true if the backend natively supports type.  e.g. if
    // "type" here is "MountTargetType::ANGLE" and the backend has
    // indicated support of angles in the
    // natively_supported_mount_target_types() bitmask then we can can
    // send_angle_targets on the backend and have that work (meaning
    // the backend has overrriden that method).
    bool natively_supports(MountTargetType type) const {
        uint8_t supported = natively_supported_mount_target_types();
        return (supported & (1U<<(uint8_t(type)))) != 0;
    }

    // look at what MountTargetTypes this mount supports; convert to
    // supported mount target type from unsupported mount target type
    // if required and then send commands to gimbal via virtual
    // methods:
    void send_target_to_gimbal();

    // FIXME: make it an internal error for these to ever be called:
    virtual void send_target_angles(const MountAngleTarget &angle_rad) { }
    virtual void send_target_rates(const MountRateTarget &rate_rads) { }
    virtual void send_target_retracted() { }
    virtual void send_target_neutral() { }

    // options parameter bitmask handling
    enum class Options : uint8_t {
        RCTARGETING_LOCK_FROM_PREVMODE = (1U << 0), // RC_TARGETING mode's lock/follow state maintained from previous mode
        NEUTRAL_ON_RC_FS               = (1U << 1), // move mount to netral position on RC failsafe
        FPV_LOCK                       = (1U << 2), // if mount supports, force body frame FPV lock on yaw/roll/pitch unconditionally in RC Targeting mode
    };
    bool option_set(Options opt) const { return (_params.options.get() & (uint8_t)opt) != 0; }

    // called when mount mode is RC-targetting, updates the mnt_target object from RC inputs:
    void update_mnt_target_from_rc_target();
    
    //called to remove lean angle in roll/pitch mount angle to convert to bodyframe
    void adjust_mnt_target_if_RP_locked();
    
    // if this function returns true, then when roll and pitch have
    // been locked to body frame (e.g. the user is using a switch or mount option to
    // lock them to body frame) we add in the vehicle's roll and
    // pitch to the target angles to position the gimbal in body
    // frame.  This is used on mounts which do not have the option to
    // move in body frame themselves (e.g. SToRM32 in its normal
    // configuration) and can be overriden in their heading files
    virtual bool apply_bf_roll_pitch_adjustments_in_rc_targeting() const {
        return false;
    }

    // method for the mount backends to call to update mnt_target based on
    // the mount mode.  Methods in here may be overridden by the derived
    // class to customise behaviour
    void update_mnt_target();
    void _update_mnt_target();

    // returns true if user has configured a valid roll angle range
    // allows user to disable roll even on 3-axis gimbal
    bool roll_range_valid() const { return (_params.roll_angle_min < _params.roll_angle_max); }

    // returns true if user has configured a valid pitch angle range
    // allows user to disable pitch even on 3-axis gimbal
    bool pitch_range_valid() const { return (_params.pitch_angle_min < _params.pitch_angle_max); }

    // returns true if user has configured a valid yaw angle range
    // allows user to disable yaw even on 3-axis gimbal
    bool yaw_range_valid() const { return (_params.yaw_angle_min < _params.yaw_angle_max); }

    // returns true if mavlink heartbeat should be suppressed for this gimbal (only used by Solo gimbal)
    virtual bool suppress_heartbeat() const { return false; }

    // change to RC_TARGETTING mode if rc inputs have changed by more than the dead zone
    // should be called on every update
    void set_rctargeting_on_rcinput_change();

    // get angle targets (in radians) to ROI location
    // returns true on success, false on failure
    bool get_angle_target_to_roi(MountAngleTarget& angle_rad) const WARN_IF_UNUSED;

    // get angle targets (in radians) to home location
    // returns true on success, false on failure
    bool get_angle_target_to_home(MountAngleTarget& angle_rad) const WARN_IF_UNUSED;

    // get angle targets (in radians) to a vehicle with sysid of _target_sysid
    // returns true on success, false on failure
    bool get_angle_target_to_sysid(MountAngleTarget& angle_rad) const WARN_IF_UNUSED;

    // update angle targets using a given rate target
    // the resulting angle_rad yaw frame will match the rate_rad yaw frame
    // assumes a 50hz update rate
    void update_angle_target_from_rate(const MountRateTarget& rate_rad, MountAngleTarget& angle_rad) const;

    // helper function to provide GIMBAL_DEVICE_FLAGS for use in GIMBAL_DEVICE_ATTITUDE_STATUS message
    uint16_t get_gimbal_device_flags() const;

    // sent warning to GCS
    void send_warning_to_GCS(const char* warning_str);

    AP_Mount    &_frontend; // reference to the front end which holds parameters
    AP_Mount_Params &_params; // parameters for this backend
    uint8_t     _instance;  // this instance's number

    MAV_MOUNT_MODE  _mode;          // current mode (see MAV_MOUNT_MODE enum)

    // structure for MAVLink Targeting angle and rate targets
    struct {
        MountTargetType target_type;// MAVLink targeting mode's current target type (e.g. angle or rate)
        MountAngleTarget angle_rad; // angle target in radians
        MountRateTarget rate_rads;  // rate target in rad/s
        uint32_t last_rate_request_ms;
    } mnt_target;
    
    // RP earth frame locks accessible by backend
    bool _pitch_lock = true;              // pitch_lock used in RC_TARGETING mode. True if the gimbal's tilt target is maintained in earth-frame, if false (aka "follow") it is maintained in body-frame
    bool _roll_lock = true;               // roll_lock used in RC_TARGETING mode. True if the gimbal's roll target is maintained in earth-frame, if false (aka "follow") it is maintained in body-frame
    

private:

    // get pilot input (in the range -1 to +1) received through RC
    void get_rc_input(float& roll_in, float& pitch_in, float& yaw_in) const;

    // get angle targets (in radians) to a Location
    // returns true on success, false on failure
    bool get_angle_target_to_location(const Location &loc, MountAngleTarget& angle_rad) const WARN_IF_UNUSED;

#if AP_MOUNT_POI_TO_LATLONALT_ENABLED
    // calculate the Location that the gimbal is pointing at
    void calculate_poi();
#endif

    bool _yaw_lock;                 // yaw_lock used in RC_TARGETING mode. True if the gimbal's yaw target is maintained in earth-frame, if false (aka "follow") it is maintained in body-frame
    
    float _yaw_lock_heading_rad;            // mount earth frame direction captured upon calling set_yaw_lock

#if AP_MOUNT_POI_TO_LATLONALT_ENABLED
    struct {
        HAL_Semaphore sem;        // semaphore protecting this structure
        uint32_t poi_request_ms;  // system time POI was last requested
        uint32_t poi_update_ms;   // system time POI was calculated
        Location loc;             // gimbal location used for poi calculation
        Location poi_loc;         // location of the POI
        Quaternion att_quat;      // attitude quaternion of the gimbal
    } poi_calculation;
#endif

    Location _roi_target;           // roi target location

    uint8_t _target_sysid;          // sysid to track
    Location _target_sysid_location;// sysid target location

    uint32_t _last_warning_ms;      // system time of last warning sent to GCS

    // structure holding the last RC inputs
    struct {
        bool    initialised;
        int16_t roll_in;
        int16_t pitch_in;
        int16_t yaw_in;
    } last_rc_input;

    // structure holding mavlink sysid and compid of controller of this gimbal
    // see MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE and GIMBAL_MANAGER_STATUS
    struct mavlink_control_id_t {
        uint8_t sysid;
        uint8_t compid;

        // equality operators
        bool operator==(const mavlink_control_id_t &rhs) const { return (sysid == rhs.sysid && compid == rhs.compid); }
        bool operator!=(const mavlink_control_id_t &rhs) const { return !(*this == rhs); }
    } mavlink_control_id;

    // SRV_Channel mount open function index
    SRV_Channel::Function    _open_idx;
};

#endif // HAL_MOUNT_ENABLED
