/************************************************************
* AP_mount -- library to control a 2 or 3 axis mount.		*
*															*
* Author:  Joe Holdsworth;									*
*		   Ritchie Wilson;									*
*		   Amilcar Lucas;									*
*		   Gregory Fletcher;								*
*          heavily modified by Randy Mackay                 *
*															*
* Purpose:  Move a 2 or 3 axis mount attached to vehicle,	*
*			Used for mount to track targets or stabilise	*
*			camera plus	other modes.						*
*															*
* Usage:	Use in main code to control	mounts attached to	*
*			vehicle.										*
*															*
* Comments: All angles in degrees, distances in meters      *
*			unless otherwise stated.						*
************************************************************/
#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_ENABLED

#include <GCS_MAVLink/GCS_config.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Camera/AP_Camera_shareddefs.h>
#include "AP_Mount_Params.h"

// maximum number of mounts
#define AP_MOUNT_MAX_INSTANCES          2

// declare backend classes
class AP_Mount_Backend;
class AP_Mount_Servo;
class AP_Mount_SoloGimbal;
class AP_Mount_Alexmos;
class AP_Mount_SToRM32;
class AP_Mount_SToRM32_serial;
class AP_Mount_Gremsy;
class AP_Mount_Siyi;
class AP_Mount_Scripting;
class AP_Mount_Xacti;
class AP_Mount_Viewpro;
class AP_Mount_Topotek;

/*
  This is a workaround to allow the MAVLink backend access to the
  SmallEKF. It would be nice to find a neater solution to this
 */

class AP_Mount
{
    // declare backends as friends
    friend class AP_Mount_Backend;
    friend class AP_Mount_Servo;
    friend class AP_Mount_SoloGimbal;
    friend class AP_Mount_Alexmos;
    friend class AP_Mount_SToRM32;
    friend class AP_Mount_SToRM32_serial;
    friend class AP_Mount_Gremsy;
    friend class AP_Mount_Siyi;
    friend class AP_Mount_Scripting;
    friend class AP_Mount_Xacti;
    friend class AP_Mount_Viewpro;
    friend class AP_Mount_Topotek;

public:
    AP_Mount();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Mount);

    // get singleton instance
    static AP_Mount *get_singleton() {
        return _singleton;
    }

    // Enums
    enum class Type {
        None = 0,            /// no mount
#if HAL_MOUNT_SERVO_ENABLED
        Servo = 1,           /// servo controlled mount
#endif
#if HAL_SOLO_GIMBAL_ENABLED
        SoloGimbal = 2,      /// Solo's gimbal
#endif
#if HAL_MOUNT_ALEXMOS_ENABLED
        Alexmos = 3,         /// Alexmos mount
#endif
#if HAL_MOUNT_STORM32MAVLINK_ENABLED
        SToRM32 = 4,         /// SToRM32 mount using MAVLink protocol
#endif
#if HAL_MOUNT_STORM32SERIAL_ENABLED
        SToRM32_serial = 5,  /// SToRM32 mount using custom serial protocol
#endif
#if HAL_MOUNT_GREMSY_ENABLED
        Gremsy = 6,          /// Gremsy gimbal using MAVLink v2 Gimbal protocol
#endif
#if HAL_MOUNT_SERVO_ENABLED
        BrushlessPWM = 7,    /// Brushless (stabilized) gimbal using PWM protocol
#endif
#if HAL_MOUNT_SIYI_ENABLED
        Siyi = 8,            /// Siyi gimbal using custom serial protocol
#endif
#if HAL_MOUNT_SCRIPTING_ENABLED
        Scripting = 9,       /// Scripting gimbal driver
#endif
#if HAL_MOUNT_XACTI_ENABLED
        Xacti = 10,          /// Xacti DroneCAN gimbal driver
#endif
#if HAL_MOUNT_VIEWPRO_ENABLED
        Viewpro = 11,        /// Viewpro gimbal using a custom serial protocol
#endif
#if HAL_MOUNT_TOPOTEK_ENABLED
        Topotek = 12,        /// Topotek gimbal using a custom serial protocol
#endif
    };

    // init - detect and initialise all mounts
    void init();

    // update - give mount opportunity to update servos.  should be called at 10hz or higher
    void update();

    // used for gimbals that need to read INS data at full rate
    void update_fast();

    // return primary instance ID
    uint8_t get_primary_instance() const { return _primary; }

    // get_mount_type - returns the type of mount
    Type get_mount_type() const { return get_mount_type(_primary); }
    Type get_mount_type(uint8_t instance) const;

    // has_pan_control - returns true if the mount has yaw control (required for copters)
    bool has_pan_control() const { return has_pan_control(_primary); }
    bool has_pan_control(uint8_t instance) const;

    // get_mode - returns current mode of mount (i.e. Retracted, Neutral, RC_Targeting, GPS Point)
    enum MAV_MOUNT_MODE get_mode() const { return get_mode(_primary); }
    enum MAV_MOUNT_MODE get_mode(uint8_t instance) const;

    // set_mode - sets mount's mode
    //  returns true if mode is successfully set
    void set_mode(enum MAV_MOUNT_MODE mode) { return set_mode(_primary, mode); }
    void set_mode(uint8_t instance, enum MAV_MOUNT_MODE mode);

    // set_mode_to_default - restores the mode to it's default mode held in the MNTx_DEFLT_MODE parameter
    //      this operation requires 60us on a Pixhawk/PX4
    void set_mode_to_default() { set_mode_to_default(_primary); }
    void set_mode_to_default(uint8_t instance);

    // set yaw_lock used in RC_TARGETING mode.  If true, the gimbal's yaw target is maintained in earth-frame meaning it will lock onto an earth-frame heading (e.g. North)
    // If false (aka "follow") the gimbal's yaw is maintained in body-frame meaning it will rotate with the vehicle
    void set_yaw_lock(bool yaw_lock) { set_yaw_lock(_primary, yaw_lock); }
    void set_yaw_lock(uint8_t instance, bool yaw_lock);

    // set angle target in degrees
    // roll and pitch are in earth-frame
    // yaw_is_earth_frame (aka yaw_lock) should be true if yaw angle is earth-frame, false if body-frame
    void set_angle_target(float roll_deg, float pitch_deg, float yaw_deg, bool yaw_is_earth_frame) { set_angle_target(_primary, roll_deg, pitch_deg, yaw_deg, yaw_is_earth_frame); }
    void set_angle_target(uint8_t instance, float roll_deg, float pitch_deg, float yaw_deg, bool yaw_is_earth_frame);

    // sets rate target in deg/s
    // yaw_lock should be true if the yaw rate is earth-frame, false if body-frame (e.g. rotates with body of vehicle)
    void set_rate_target(float roll_degs, float pitch_degs, float yaw_degs, bool yaw_lock) { set_rate_target(_primary, roll_degs, pitch_degs, yaw_degs, yaw_lock); }
    void set_rate_target(uint8_t instance, float roll_degs, float pitch_degs, float yaw_degs, bool yaw_lock);

    // set_roi_target - sets target location that mount should attempt to point towards
    void set_roi_target(const Location &target_loc) { set_roi_target(_primary,target_loc); }
    void set_roi_target(uint8_t instance, const Location &target_loc);

    // clear_roi_target - clears target location that mount should attempt to point towards
    void clear_roi_target() { clear_roi_target(_primary); }
    void clear_roi_target(uint8_t instance);

    // point at system ID sysid
    void set_target_sysid(uint8_t sysid) { set_target_sysid(_primary, sysid); }
    void set_target_sysid(uint8_t instance, uint8_t sysid);

    // handling of set_roi_sysid message
    MAV_RESULT handle_command_do_set_roi_sysid(const mavlink_command_int_t &packet);

    // mavlink message handling:
    MAV_RESULT handle_command(const mavlink_command_int_t &packet, const mavlink_message_t &msg);
    void handle_param_value(const mavlink_message_t &msg);
    void handle_message(mavlink_channel_t chan, const mavlink_message_t &msg);

    // send a GIMBAL_DEVICE_ATTITUDE_STATUS message to GCS
    void send_gimbal_device_attitude_status(mavlink_channel_t chan);

    // send a GIMBAL_MANAGER_INFORMATION message to GCS
    void send_gimbal_manager_information(mavlink_channel_t chan);

    // send a GIMBAL_MANAGER_STATUS message to GCS
    void send_gimbal_manager_status(mavlink_channel_t chan);

#if AP_MOUNT_POI_TO_LATLONALT_ENABLED
    // get poi information.  Returns true on success and fills in gimbal attitude, location and poi location
    bool get_poi(uint8_t instance, Quaternion &quat, Location &loc, Location &poi_loc) const;
#endif

    // get mount's current attitude in euler angles in degrees.  yaw angle is in body-frame
    // returns true on success
    bool get_attitude_euler(uint8_t instance, float& roll_deg, float& pitch_deg, float& yaw_bf_deg);

    // run pre-arm check.  returns false on failure and fills in failure_msg
    // any failure_msg returned will not include a prefix
    bool pre_arm_checks(char *failure_msg, uint8_t failure_msg_len);

    // get target rate in deg/sec. returns true on success
    bool get_rate_target(uint8_t instance, float& roll_degs, float& pitch_degs, float& yaw_degs, bool& yaw_is_earth_frame);

    // get target angle in deg. returns true on success
    bool get_angle_target(uint8_t instance, float& roll_deg, float& pitch_deg, float& yaw_deg, bool& yaw_is_earth_frame);

    // accessors for scripting backends and logging
    bool get_location_target(uint8_t instance, Location& target_loc);
    void set_attitude_euler(uint8_t instance, float roll_deg, float pitch_deg, float yaw_bf_deg);

    // write mount log packet for all backends
    void write_log();

    // write mount log packet for a single backend (called by camera library)
    void write_log(uint8_t instance, uint64_t timestamp_us);

    //
    // camera controls for gimbals that include a camera
    //

    // take a picture
    bool take_picture(uint8_t instance);

    // start or stop video recording
    // set start_recording = true to start record, false to stop recording
    bool record_video(uint8_t instance, bool start_recording);

    // set zoom specified as a rate or percentage
    bool set_zoom(uint8_t instance, ZoomType zoom_type, float zoom_value);

    // set focus specified as rate, percentage or auto
    // focus in = -1, focus hold = 0, focus out = 1
    SetFocusResult set_focus(uint8_t instance, FocusType focus_type, float focus_value);

    // set tracking to none, point or rectangle (see TrackingType enum)
    // if POINT only p1 is used, if RECTANGLE then p1 is top-left, p2 is bottom-right
    // p1,p2 are in range 0 to 1.  0 is left or top, 1 is right or bottom
    bool set_tracking(uint8_t instance, TrackingType tracking_type, const Vector2f& p1, const Vector2f& p2);

    // set camera lens as a value from 0 to 5
    bool set_lens(uint8_t instance, uint8_t lens);

#if HAL_MOUNT_SET_CAMERA_SOURCE_ENABLED
    // set_camera_source is functionally the same as set_lens except primary and secondary lenses are specified by type
    // primary and secondary sources use the AP_Camera::CameraSource enum cast to uint8_t
    bool set_camera_source(uint8_t instance, uint8_t primary_source, uint8_t secondary_source);
#endif

    // send camera information message to GCS
    void send_camera_information(uint8_t instance, mavlink_channel_t chan) const;

    // send camera settings message to GCS
    void send_camera_settings(uint8_t instance, mavlink_channel_t chan) const;

    // send camera capture status message to GCS
    void send_camera_capture_status(uint8_t instance, mavlink_channel_t chan) const;

#if AP_MOUNT_SEND_THERMAL_RANGE_ENABLED
    // send camera thermal range message to GCS
    void send_camera_thermal_range(uint8_t instance, mavlink_channel_t chan) const;
#endif

    // change camera settings not normally used by autopilot
    // setting values from AP_Camera::Setting enum
    bool change_setting(uint8_t instance, CameraSetting setting, float value);

    //
    // rangefinder
    //

    // get rangefinder distance.  Returns true on success
    bool get_rangefinder_distance(uint8_t instance, float& distance_m) const;

    // enable/disable rangefinder.  Returns true on success
    bool set_rangefinder_enable(uint8_t instance, bool enable);

    // parameter var table
    static const struct AP_Param::GroupInfo        var_info[];

protected:

    static AP_Mount *_singleton;

    // parameters for backends
    AP_Mount_Params _params[AP_MOUNT_MAX_INSTANCES];

    // front end members
    uint8_t             _num_instances;     // number of mounts instantiated
    uint8_t             _primary;           // primary mount
    AP_Mount_Backend    *_backends[AP_MOUNT_MAX_INSTANCES];         // pointers to instantiated mounts

private:
    // Check if instance backend is ok
    AP_Mount_Backend *get_primary() const;
    AP_Mount_Backend *get_instance(uint8_t instance) const;

    void handle_gimbal_report(mavlink_channel_t chan, const mavlink_message_t &msg);
#if AP_MAVLINK_MSG_MOUNT_CONFIGURE_ENABLED
    void handle_mount_configure(const mavlink_message_t &msg);
#endif
#if AP_MAVLINK_MSG_MOUNT_CONTROL_ENABLED
    void handle_mount_control(const mavlink_message_t &msg);
#endif

    MAV_RESULT handle_command_do_mount_configure(const mavlink_command_int_t &packet);
    MAV_RESULT handle_command_do_mount_control(const mavlink_command_int_t &packet);
    MAV_RESULT handle_command_do_gimbal_manager_pitchyaw(const mavlink_command_int_t &packet);
    MAV_RESULT handle_command_do_gimbal_manager_configure(const mavlink_command_int_t &packet, const mavlink_message_t &msg);
    void handle_gimbal_manager_set_attitude(const mavlink_message_t &msg);
    void handle_gimbal_manager_set_pitchyaw(const mavlink_message_t &msg);
    void handle_global_position_int(const mavlink_message_t &msg);
    void handle_gimbal_device_information(const mavlink_message_t &msg);
    void handle_gimbal_device_attitude_status(const mavlink_message_t &msg);

    // perform any required parameter conversion
    void convert_params();
};

namespace AP {
    AP_Mount *mount();
};

#endif // HAL_MOUNT_ENABLED
