#include "AP_Mount_config.h"

#if HAL_MOUNT_GIMBAL_MANAGER_ENABLED

#include "AP_Mount_GimbalManager.h"

#include <GCS_MAVLink/GCS.h>

// send GIMBAL_MANAGER_SET_PITCHYAW to the gimbal manager to retract the gimbal
void AP_Mount_GimbalManager::send_target_retracted()
{
    send_pitchyaw(GIMBAL_MANAGER_FLAGS_RETRACT, NAN, NAN, NAN, NAN);
}

// send GIMBAL_MANAGER_SET_PITCHYAW to the gimbal manager to control rate
void AP_Mount_GimbalManager::send_target_rates(const MountRateTarget &rate_rads)
{
    send_pitchyaw(rate_rads.yaw_is_ef ? GIMBAL_MANAGER_FLAGS_YAW_LOCK : 0, NAN, NAN, rate_rads.pitch, rate_rads.yaw);
}

// send GIMBAL_MANAGER_SET_PITCHYAW to the gimbal manager to control attitude
void AP_Mount_GimbalManager::send_target_angles(const MountAngleTarget &angle_rad)
{
    send_pitchyaw(angle_rad.yaw_is_ef ? GIMBAL_MANAGER_FLAGS_YAW_LOCK : 0, angle_rad.pitch, angle_rad.yaw, NAN, NAN);
}

// send GIMBAL_MANAGER_SET_PITCHYAW to every link with a component of this system.
// Only changed targets are sent so other clients of the gimbal manager are not overridden
void AP_Mount_GimbalManager::send_pitchyaw(uint32_t flags, float pitch, float yaw, float pitch_rate, float yaw_rate)
{
    const mavlink_gimbal_manager_set_pitchyaw_t pkt {
        flags | GIMBAL_MANAGER_FLAGS_ROLL_LOCK | GIMBAL_MANAGER_FLAGS_PITCH_LOCK,  // roll and pitch are always earth frame
        pitch,
        yaw,
        pitch_rate,
        yaw_rate,
        mavlink_msg_target_field(_sysid),
        0,  // gimbal manager component id is unknown so broadcast to the system
        _compid  // gimbal device id
    };

    if (_last_target_msgid == MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW &&
        memcmp(&_last_target.pitchyaw, &pkt, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_LEN) == 0) {
        return;
    }
    _last_target.pitchyaw = pkt;
    _last_target_msgid = MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW;
    GCS_MAVLINK::send_to_components(MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW, (const char*)&pkt, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_LEN);
}

#endif // HAL_MOUNT_GIMBAL_MANAGER_ENABLED
