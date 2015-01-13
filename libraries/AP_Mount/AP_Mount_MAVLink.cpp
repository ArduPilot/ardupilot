// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_Mount_MAVLink.h>

// init - performs any required initialisation for this instance
void AP_Mount_MAVLink::init()
{
    // do nothing
}

// update mount position - should be called periodically
void AP_Mount_MAVLink::update()
{
    // nothing to do for a MAVlink gimbal that supports all modes:
    //  MAV_MOUNT_MODE_RETRACT - nothing to do if the mount holds the retracted angles and
    //                           we do not implement a separate servo based retract mechanism
    //  MAV_MOUNT_MODE_NEUTRAL - nothing to do if the mount holds the neutral angles
    //  MAV_MOUNT_MODE_MAVLINK_TARGETING - targets should already have been sent by a call to control_msg
    //  MAV_MOUNT_MODE_RC_TARGETING - mount should be able to see RC inputs published by flight controller
    //  MAV_MOUNT_MODE_GPS_POINT - mount should have received target sent from control_msg and
    //                             should know vehicle's position which has been published by flight controller
}

// has_pan_control - returns true if this mount can control it's pan (required for multicopters)
bool AP_Mount_MAVLink::has_pan_control() const
{
    // we do not have yaw control
    return false;
}

// set_mode - sets mount's mode
void AP_Mount_MAVLink::set_mode(enum MAV_MOUNT_MODE mode)
{
    // exit immediately if mount has not been found
    if (!_enabled) {
        return;
    }

    // prepare and send command_long message with DO_SET_MODE command
    mavlink_msg_command_long_send(
            _chan, mavlink_system.sysid, AP_MOUNT_MAVLINK_COMPID, // channel, system id, component id
            MAV_CMD_DO_SET_MODE,    // command number
            0,  // confirmation: 0=first confirmation of this command
            mode,   // param1: mode
            0,      // param2: custom mode
            0.0f, 0.0f, 0.0f,0.0f, 0.0f);    // param3 ~ param 7: not used

    // record the mode change
    _frontend.state[_instance]._mode = mode;
}

// set_roi_target - sets target location that mount should attempt to point towards
void AP_Mount_MAVLink::set_roi_target(const struct Location &target_loc)
{
    // exit immediately if mount has not been found
    if (!_enabled) {
        return;
    }

    // prepare and send command_long message
    mavlink_msg_command_long_send(
            _chan, mavlink_system.sysid, AP_MOUNT_MAVLINK_COMPID, // channel, system id, component id
            MAV_CMD_DO_MOUNT_CONTROL,   // command number
            0, // confirmation: 0=first confirmation of this command
            target_loc.lat,  // param1: pitch (in degrees) or lat (as int32_t)
            target_loc.lng,  // param2: roll (in degrees) or lon (as int32_t)
            target_loc.alt,  // param3: yaw (in degrees) or alt (in meters).  To-Do: clarify if this is absolute alt or relative to home
            0.0f,0.0f,0.0f,             // param4 ~ param6 : not used
            MAV_MOUNT_MODE_GPS_POINT);  // param7: MAV_MOUNT_MODE enum value
}

// configure_msg - process MOUNT_CONFIGURE messages received from GCS
void AP_Mount_MAVLink::configure_msg(mavlink_message_t* msg)
{
    // exit immediately if mount has not been found
    if (!_enabled) {
        return;
    }

    // forward on message with updated channel, sysid, compid
    mavlink_msg_mount_configure_send(
            _chan, mavlink_system.sysid, AP_MOUNT_MAVLINK_COMPID,
            mavlink_msg_mount_configure_get_mount_mode(msg),
            mavlink_msg_mount_configure_get_stab_roll(msg),
            mavlink_msg_mount_configure_get_stab_pitch(msg),
            mavlink_msg_mount_configure_get_stab_yaw(msg));
}

// control_msg - process MOUNT_CONTROL messages received from GCS
void AP_Mount_MAVLink::control_msg(mavlink_message_t* msg)
{
    // exit immediately if mount has not been found
    if (!_enabled) {
        return;
    }

    // forward on message with updated channel, sysid, compid
    mavlink_msg_mount_control_send(
            _chan, mavlink_system.sysid, AP_MOUNT_MAVLINK_COMPID,
            mavlink_msg_mount_control_get_input_a(msg),
            mavlink_msg_mount_control_get_input_b(msg),
            mavlink_msg_mount_control_get_input_c(msg),
            mavlink_msg_mount_control_get_save_position(msg));
}

// status_msg - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
void AP_Mount_MAVLink::status_msg(mavlink_channel_t chan)
{
    // exit immediately if mount has not been found
    if (!_enabled) {
        return;
    }
}

// find_mount - search for MAVLink enabled mount
void AP_Mount_MAVLink::find_mount()
{
    // return immediately if target has already been found
    if (_enabled) {
        return;
    }

    // To-Do: search for MAVLink enabled mount using MAVLink_routing table
}

// send_angle_target - send earth-frame angle targets to mount
void AP_Mount_MAVLink::send_angle_target(const Vector3f& target_deg)
{
    // exit immediately if mount has not been found
    if (!_enabled) {
        return;
    }

    // prepare and send command_long message with DO_MOUNT_CONTROL command
    mavlink_msg_command_long_send(
            _chan, mavlink_system.sysid, AP_MOUNT_MAVLINK_COMPID, // channel, system id, component id
            MAV_CMD_DO_MOUNT_CONTROL,   // command number
            0,  // confirmation: 0=first confirmation of this command
            target_deg.y,   // param1: pitch (in degrees) or lat (as int32_t)
            target_deg.x,   // param2: roll (in degrees) or lon (as int32_t)
            target_deg.z,   // param3: yaw (in degrees) or alt (in meters).
            0.0f,0.0f,0.0f,                     // param4 ~ param6 : not used
            MAV_MOUNT_MODE_MAVLINK_TARGETING);  // param7: MAV_MOUNT_MODE enum value
}
