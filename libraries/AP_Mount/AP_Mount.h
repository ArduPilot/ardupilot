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

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_MOUNT_ENABLED
#define HAL_MOUNT_ENABLED !HAL_MINIMIZE_FEATURES
#endif

#ifndef HAL_SOLO_GIMBAL_ENABLED
#define HAL_SOLO_GIMBAL_ENABLED 0
#endif

#if HAL_MOUNT_ENABLED

#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
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

public:
    AP_Mount();

    /* Do not allow copies */
    AP_Mount(const AP_Mount &other) = delete;
    AP_Mount &operator=(const AP_Mount&) = delete;

    // get singleton instance
    static AP_Mount *get_singleton() {
        return _singleton;
    }

    // Enums
    enum MountType {
        Mount_Type_None = 0,            /// no mount
        Mount_Type_Servo = 1,           /// servo controlled mount
        Mount_Type_SoloGimbal = 2,      /// Solo's gimbal
        Mount_Type_Alexmos = 3,         /// Alexmos mount
        Mount_Type_SToRM32 = 4,         /// SToRM32 mount using MAVLink protocol
        Mount_Type_SToRM32_serial = 5,  /// SToRM32 mount using custom serial protocol
        Mount_Type_Gremsy = 6,          /// Gremsy gimbal using MAVLink v2 Gimbal protocol
        Mount_Type_BrushlessPWM = 7,    /// Brushless (stabilized) gimbal using PWM protocol
        Mount_Type_Siyi = 8,            /// Siyi gimbal using custom serial protocol
    };

    // init - detect and initialise all mounts
    void init();

    // update - give mount opportunity to update servos.  should be called at 10hz or higher
    void update();

    // used for gimbals that need to read INS data at full rate
    void update_fast();

    // return primary instance
    uint8_t get_primary() const { return _primary; }

    // get_mount_type - returns the type of mount
    AP_Mount::MountType get_mount_type() const { return get_mount_type(_primary); }
    AP_Mount::MountType get_mount_type(uint8_t instance) const;

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

    // set yaw_lock.  If true, the gimbal's yaw target is maintained in earth-frame meaning it will lock onto an earth-frame heading (e.g. North)
    // If false (aka "follow") the gimbal's yaw is maintained in body-frame meaning it will rotate with the vehicle
    void set_yaw_lock(bool yaw_lock) { set_yaw_lock(_primary, yaw_lock); }
    void set_yaw_lock(uint8_t instance, bool yaw_lock);

    // set angle target in degrees
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

    // point at system ID sysid
    void set_target_sysid(uint8_t sysid) { set_target_sysid(_primary, sysid); }
    void set_target_sysid(uint8_t instance, uint8_t sysid);

    // mavlink message handling:
    MAV_RESULT handle_command_long(const mavlink_command_long_t &packet);
    void handle_param_value(const mavlink_message_t &msg);
    void handle_message(mavlink_channel_t chan, const mavlink_message_t &msg);

    // send a GIMBAL_DEVICE_ATTITUDE_STATUS message to GCS
    void send_gimbal_device_attitude_status(mavlink_channel_t chan);

    // run pre-arm check.  returns false on failure and fills in failure_msg
    // any failure_msg returned will not include a prefix
    bool pre_arm_checks(char *failure_msg, uint8_t failure_msg_len);

    //
    // camera controls for gimbals that include a camera
    //

    // take a picture
    bool take_picture(uint8_t instance);

    // start or stop video recording
    // set start_recording = true to start record, false to stop recording
    bool record_video(uint8_t instance, bool start_recording);

    // set camera zoom step
    // zoom out = -1, hold = 0, zoom in = 1
    bool set_zoom_step(uint8_t instance, int8_t zoom_step);

    // set focus in, out or hold
    // focus in = -1, focus hold = 0, focus out = 1
    bool set_manual_focus_step(uint8_t instance, int8_t focus_step);

    // auto focus
    bool set_auto_focus(uint8_t instance);

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
    bool check_primary() const;
    bool check_instance(uint8_t instance) const;

    void handle_gimbal_report(mavlink_channel_t chan, const mavlink_message_t &msg);
    void handle_mount_configure(const mavlink_message_t &msg);
    void handle_mount_control(const mavlink_message_t &msg);

    MAV_RESULT handle_command_do_mount_configure(const mavlink_command_long_t &packet);
    MAV_RESULT handle_command_do_mount_control(const mavlink_command_long_t &packet);
    MAV_RESULT handle_command_do_gimbal_manager_pitchyaw(const mavlink_command_long_t &packet);
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
