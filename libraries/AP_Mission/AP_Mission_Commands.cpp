#include "AP_Mission_config.h"

#if AP_MISSION_ENABLED

#include "AP_Mission.h"

#include <GCS_MAVLink/GCS.h>
#include <AP_Camera/AP_Camera.h>
#include <AP_Camera/AP_Camera_Backend.h>
#include <AP_Gripper/AP_Gripper.h>
#include <AP_Parachute/AP_Parachute.h>
#include <AP_ServoRelayEvents/AP_ServoRelayEvents.h>
#include <AC_Sprayer/AC_Sprayer.h>
#include <AP_Scripting/AP_Scripting.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_Mount/AP_Mount.h>
#include <AC_Fence/AC_Fence.h>

#if AP_RC_CHANNEL_ENABLED
bool AP_Mission::start_command_do_aux_function(const AP_Mission::Mission_Command& cmd)
{
    const RC_Channel::AUX_FUNC function = (RC_Channel::AUX_FUNC)cmd.content.auxfunction.function;
    const RC_Channel::AuxSwitchPos pos = (RC_Channel::AuxSwitchPos)cmd.content.auxfunction.switchpos;

    // sanity check the switch position.  Could map from the mavlink
    // enumeration if we were really keen
    switch (pos) {
    case RC_Channel::AuxSwitchPos::HIGH:
    case RC_Channel::AuxSwitchPos::MIDDLE:
    case RC_Channel::AuxSwitchPos::LOW:
        break;
    default:
        return false;
    }
    rc().run_aux_function(function, pos, RC_Channel::AuxFuncTrigger::Source::MISSION, cmd.index);
    return true;
}
#endif  // AP_RC_CHANNEL_ENABLED

#if AP_GRIPPER_ENABLED
bool AP_Mission::start_command_do_gripper(const AP_Mission::Mission_Command& cmd)
{
    AP_Gripper &gripper = AP::gripper();

    // Note: we ignore the gripper num parameter because we only
    // support one gripper
    switch (cmd.content.gripper.action) {
    case GRIPPER_ACTION_RELEASE:
        gripper.release();
        // Log_Write_Event(DATA_GRIPPER_RELEASE);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Gripper Released");
        return true;
    case GRIPPER_ACTION_GRAB:
        gripper.grab();
        // Log_Write_Event(DATA_GRIPPER_GRAB);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Gripper Grabbed");
        return true;
    default:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Unhandled gripper case");
#endif
        return false;
    }
}
#endif  // AP_GRIPPER_ENABLED

#if AP_SERVORELAYEVENTS_ENABLED
bool AP_Mission::start_command_do_servorelayevents(const AP_Mission::Mission_Command& cmd)
{
    AP_ServoRelayEvents *sre = AP::servorelayevents();
    if (sre == nullptr) {
        return false;
    }

    switch (cmd.id) {
    case MAV_CMD_DO_SET_SERVO:
        return sre->do_set_servo(cmd.content.servo.channel, cmd.content.servo.pwm);

#if AP_RELAY_ENABLED
    case MAV_CMD_DO_SET_RELAY:
        return sre->do_set_relay(cmd.content.relay.num, cmd.content.relay.state);
#endif

    case MAV_CMD_DO_REPEAT_SERVO:
        return sre->do_repeat_servo(cmd.content.repeat_servo.channel,
                                    cmd.content.repeat_servo.pwm,
                                    cmd.content.repeat_servo.repeat_count,
                                    cmd.content.repeat_servo.cycle_time * 1000.0f);

#if AP_RELAY_ENABLED
    case MAV_CMD_DO_REPEAT_RELAY:
        return sre->do_repeat_relay(cmd.content.repeat_relay.num,
                                    cmd.content.repeat_relay.repeat_count,
                                    cmd.content.repeat_relay.cycle_time * 1000.0f);
#endif

    default:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Unhandled servo/relay case");
#endif
        return false;
    }
}
#endif  // AP_SERVORELAYEVENTS_ENABLED

#if AP_CAMERA_ENABLED
bool AP_Mission::start_command_camera(const AP_Mission::Mission_Command& cmd)
{
    AP_Camera *camera = AP::camera();
    if (camera == nullptr) {
        return false;
    }

    mavlink_mission_item_int_t item {};
    if (!mission_cmd_to_mavlink_int(cmd, item)) {
        return false;
    }
    mavlink_command_int_t packet {};
    packet.command = item.command;
    packet.param1 = item.param1;
    packet.param2 = item.param2;
    packet.param3 = item.param3;
    packet.param4 = item.param4;
    packet.x = item.x;
    packet.y = item.y;
    packet.z = item.z;
    return camera->handle_command(packet) == MAV_RESULT_ACCEPTED;
}
#endif

bool AP_Mission::start_command_parachute(const AP_Mission::Mission_Command& cmd)
{
#if HAL_PARACHUTE_ENABLED
    AP_Parachute *parachute = AP::parachute();
    if (parachute == nullptr) {
        return false;
    }

    switch (cmd.p1) {
    case PARACHUTE_DISABLE:
        parachute->enabled(false);
        break;
    case PARACHUTE_ENABLE:
        parachute->enabled(true);
        break;
    case PARACHUTE_RELEASE:
        parachute->release();
        break;
    default:
        // do nothing
        return false;
    }

    return true;
#else
    return false;
#endif // HAL_PARACHUTE_ENABLED
}

bool AP_Mission::command_do_set_repeat_dist(const AP_Mission::Mission_Command& cmd)
{
    _repeat_dist = cmd.p1;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Resume repeat dist set to %u m",_repeat_dist);
    return true;
}

bool AP_Mission::start_command_do_sprayer(const AP_Mission::Mission_Command& cmd)
{
#if HAL_SPRAYER_ENABLED
    AC_Sprayer *sprayer = AP::sprayer();
    if (sprayer == nullptr) {
        return false;
    }

    if (cmd.p1 == 1) {
        sprayer->run(true);
    } else {
        sprayer->run(false);
    }

    return true;
#else
    return false;
#endif // HAL_SPRAYER_ENABLED
}

bool AP_Mission::start_command_do_scripting(const AP_Mission::Mission_Command& cmd)
{
#if AP_SCRIPTING_ENABLED
    AP_Scripting *scripting = AP_Scripting::get_singleton();
    if (scripting == nullptr) {
        return false;
    }

    scripting->handle_mission_command(cmd);

    return true;
#else
    return false;
#endif // AP_SCRIPTING_ENABLED
}

bool AP_Mission::start_command_do_gimbal_manager_pitchyaw(const AP_Mission::Mission_Command& cmd)
{
#if HAL_MOUNT_ENABLED
    AP_Mount *mount = AP::mount();
    if (mount == nullptr) {
        return false;
    }

    uint8_t gimbal_instance;
    if (!mount->get_instance_from_device_id(cmd.content.gimbal_manager_pitchyaw.gimbal_id, gimbal_instance)) {
        return false;
    }

    // check flags for change to RETRACT
    if ((cmd.content.gimbal_manager_pitchyaw.flags & GIMBAL_MANAGER_FLAGS_RETRACT) > 0) {
        mount->set_mode(gimbal_instance, MAV_MOUNT_MODE_RETRACT);
        return true;
    }
    // check flags for change to NEUTRAL
    if ((cmd.content.gimbal_manager_pitchyaw.flags & GIMBAL_MANAGER_FLAGS_NEUTRAL) > 0) {
        mount->set_mode(gimbal_instance, MAV_MOUNT_MODE_NEUTRAL);
        return true;
    }

    // handle angle target
    const bool pitch_angle_valid = abs(cmd.content.gimbal_manager_pitchyaw.pitch_angle_deg) <= 90;
    const bool yaw_angle_valid = abs(cmd.content.gimbal_manager_pitchyaw.yaw_angle_deg) <= 360;
    if (pitch_angle_valid && yaw_angle_valid) {
        mount->set_angle_target(gimbal_instance, 0, cmd.content.gimbal_manager_pitchyaw.pitch_angle_deg, cmd.content.gimbal_manager_pitchyaw.yaw_angle_deg, cmd.content.gimbal_manager_pitchyaw.flags & GIMBAL_MANAGER_FLAGS_YAW_LOCK);
        return true;
    }

    // handle rate target
    if (!isnan(cmd.content.gimbal_manager_pitchyaw.pitch_rate_degs) && !isnan(cmd.content.gimbal_manager_pitchyaw.yaw_rate_degs)) {
        mount->set_rate_target(gimbal_instance, 0, cmd.content.gimbal_manager_pitchyaw.pitch_rate_degs, cmd.content.gimbal_manager_pitchyaw.yaw_rate_degs, cmd.content.gimbal_manager_pitchyaw.flags & GIMBAL_MANAGER_FLAGS_YAW_LOCK);
        return true;
    }

#endif // HAL_MOUNT_ENABLED
    // if we got this far then message is not handled
    return false;
}

bool AP_Mission::start_command_do_set_roi(const AP_Mission::Mission_Command &cmd)
{
#if HAL_MOUNT_ENABLED
    AP_Mount *mount = AP::mount();
    uint8_t instance;
    if (mount == nullptr || !mount->get_instance_from_device_id(cmd.p1, instance)) {
        return false;
    }
    if (cmd.id == MAV_CMD_DO_SET_ROI_NONE) {
        mount->clear_roi_target(instance);
    } else {
        mount->set_roi_target(instance, cmd.content.location);
    }
    return true;
#else
    return false;
#endif  // HAL_MOUNT_ENABLED
}

bool AP_Mission::start_command_fence(const AP_Mission::Mission_Command& cmd)
{
#if AP_FENCE_ENABLED
    AC_Fence* fence = AP::fence();

    if (fence == nullptr) {
        return false;
    }

    if (cmd.p1 == uint8_t(AC_Fence::MavlinkFenceActions::DISABLE_FENCE)) {          // disable fence
        uint8_t fences = fence->enable_configured(false);
        fence->print_fence_message("disabled", fences);
        return true;
    } else if (cmd.p1 == uint8_t(AC_Fence::MavlinkFenceActions::ENABLE_FENCE)) {   // enable fence
        uint8_t fences = fence->enable_configured(true);
        fence->print_fence_message("enabled", fences);
        return true;
    } else if (cmd.p1 == uint8_t(AC_Fence::MavlinkFenceActions::DISABLE_ALT_MIN_FENCE)) {   // disable fence floor only
        fence->disable_floor();
        fence->print_fence_message("disabled", AC_FENCE_TYPE_ALT_MIN);
        return true;
    }
#endif // AP_FENCE_ENABLED
    return false;
}

#endif  // AP_MISSION_ENABLED

#if AP_MISSION_MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET_ENABLED
bool AP_Mission::start_command_do_set_roi_wpnext_offset(const AP_Mission::Mission_Command& cmd)
{
#if HAL_MOUNT_ENABLED
    AP_Mount *mount = AP::mount();
    if (mount == nullptr) {
        return false;
    }

    uint8_t instance;
    if (!mount->get_instance_from_device_id(cmd.content.wpnext_offset.gimbal_id, instance)) {
        return false;
    }
    mount->set_roi_target_wpnext_offset(instance, Vector3f{
        cmd.content.wpnext_offset.roll_offset_cd * 0.01f,
        cmd.content.wpnext_offset.pitch_offset_cd * 0.01f,
        cmd.content.wpnext_offset.yaw_offset_cd * 0.01f
    });

    return true;
#else
    return false;
#endif // HAL_MOUNT_ENABLED
}
#endif // AP_MISSION_MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET_ENABLED
