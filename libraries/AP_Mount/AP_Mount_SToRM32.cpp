#include "AP_Mount_config.h"

#if HAL_MOUNT_STORM32MAVLINK_ENABLED

#include "AP_Mount_SToRM32.h"

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define AP_MOUNT_STORM32_SEARCH_MS  60000   // search for gimbal for 1 minute after startup

// update mount position - should be called periodically
void AP_Mount_SToRM32::update()
{
    AP_Mount_Backend::update();

    // exit immediately if not initialised
    if (!_initialised) {
        find_gimbal();
        return;
    }

    AP_Mount_Backend::update_mnt_target();

    // send target angles (which may be derived from other target types)
    AP_Mount_Backend::send_target_to_gimbal();
}

// get attitude as a quaternion.  returns true on success
bool AP_Mount_SToRM32::get_attitude_quaternion(Quaternion& att_quat)
{
    att_quat.from_euler(mnt_target.angle_rad.roll, mnt_target.angle_rad.pitch, mnt_target.angle_rad.get_bf_yaw());
    return true;
}

// search for gimbal in GCS_MAVLink routing table
void AP_Mount_SToRM32::find_gimbal()
{
    // return immediately if initialised
    if (_initialised) {
        return;
    }

    // search for gimbal for 60 seconds or until armed
    if ((AP_HAL::millis() > AP_MOUNT_STORM32_SEARCH_MS) && hal.util->get_soft_armed()) {
        return;
    }

    // we expect that instance 0 has compid = MAV_COMP_ID_GIMBAL, instance 1 has compid = MAV_COMP_ID_GIMBAL2, etc
    uint8_t compid = (_instance == 0) ? MAV_COMP_ID_GIMBAL : MAV_COMP_ID_GIMBAL2 + (_instance - 1);
    if (GCS_MAVLINK::find_by_mavtype_and_compid(MAV_TYPE_GIMBAL, compid, _sysid, _chan)) {
        _compid = compid;
        _initialised = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Mount: SToRM32");
    }
}

// send_target_angles - send a COMMAND_LONG containing a do_mount_control message
void AP_Mount_SToRM32::send_target_angles(const MountAngleTarget& angle_target_rad)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // check we have space for the message
    if (!HAVE_PAYLOAD_SPACE(_chan, COMMAND_LONG)) {
        return;
    }

    // send command_long command containing a do_mount_control command
    // Note: pitch and yaw are reversed
    mavlink_msg_command_long_send(_chan,
                                  _sysid,
                                  _compid,
                                  MAV_CMD_DO_MOUNT_CONTROL,
                                  0,        // confirmation of zero means this is the first time this message has been sent
                                  -degrees(angle_target_rad.pitch),
                                  degrees(angle_target_rad.roll),
                                  -degrees(angle_target_rad.get_bf_yaw()),
                                  0, 0, 0,  // param4 ~ param6 unused
                                  MAV_MOUNT_MODE_MAVLINK_TARGETING);
}
#endif // HAL_MOUNT_STORM32MAVLINK_ENABLED
