#include "AP_Mission.h"

#include <GCS_MAVLink/GCS.h>
#include <AP_Camera/AP_Camera.h>
#include <AP_Gripper/AP_Gripper.h>
#include <AP_Parachute/AP_Parachute.h>
#include <AP_ServoRelayEvents/AP_ServoRelayEvents.h>
#include <AC_Sprayer/AC_Sprayer.h>
#include <AP_Scripting/AP_Scripting.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_Mount/AP_Mount.h>

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
    rc().run_aux_function(function, pos, RC_Channel::AuxFuncTriggerSource::MISSION);
    return true;
}

bool AP_Mission::start_command_do_gripper(const AP_Mission::Mission_Command& cmd)
{
    AP_Gripper *gripper = AP::gripper();
    if (gripper == nullptr) {
        return false;
    }

    // Note: we ignore the gripper num parameter because we only
    // support one gripper
    switch (cmd.content.gripper.action) {
    case GRIPPER_ACTION_RELEASE:
        gripper->release();
        // Log_Write_Event(DATA_GRIPPER_RELEASE);
        gcs().send_text(MAV_SEVERITY_INFO, "Gripper Released");
        return true;
    case GRIPPER_ACTION_GRAB:
        gripper->grab();
        // Log_Write_Event(DATA_GRIPPER_GRAB);
        gcs().send_text(MAV_SEVERITY_INFO, "Gripper Grabbed");
        return true;
    default:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Unhandled gripper case");
#endif
        return false;
    }
}

bool AP_Mission::start_command_do_servorelayevents(const AP_Mission::Mission_Command& cmd)
{
    AP_ServoRelayEvents *sre = AP::servorelayevents();
    if (sre == nullptr) {
        return false;
    }

    switch (cmd.id) {
    case MAV_CMD_DO_SET_SERVO:
        return sre->do_set_servo(cmd.content.servo.channel, cmd.content.servo.pwm);

    case MAV_CMD_DO_SET_RELAY:
        return sre->do_set_relay(cmd.content.relay.num, cmd.content.relay.state);

    case MAV_CMD_DO_REPEAT_SERVO:
        return sre->do_repeat_servo(cmd.content.repeat_servo.channel,
                                    cmd.content.repeat_servo.pwm,
                                    cmd.content.repeat_servo.repeat_count,
                                    cmd.content.repeat_servo.cycle_time * 1000.0f);

    case MAV_CMD_DO_REPEAT_RELAY:
        return sre->do_repeat_relay(cmd.content.repeat_relay.num,
                                    cmd.content.repeat_relay.repeat_count,
                                    cmd.content.repeat_relay.cycle_time * 1000.0f);
    default:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Unhandled servo/relay case");
#endif
        return false;
    }
}

bool AP_Mission::start_command_camera(const AP_Mission::Mission_Command& cmd)
{
    AP_Camera *camera = AP::camera();
    if (camera == nullptr) {
        return false;
    }

    switch (cmd.id) {

    case MAV_CMD_DO_DIGICAM_CONFIGURE:                  // Mission command to configure an on-board camera controller system. |Modes: P, TV, AV, M, Etc| Shutter speed: Divisor number for one second| Aperture: F stop number| ISO number e.g. 80, 100, 200, Etc| Exposure type enumerator| Command Identity| Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)|
        camera->configure(
            cmd.content.digicam_configure.shooting_mode,
            cmd.content.digicam_configure.shutter_speed,
            cmd.content.digicam_configure.aperture,
            cmd.content.digicam_configure.ISO,
            cmd.content.digicam_configure.exposure_type,
            cmd.content.digicam_configure.cmd_id,
            cmd.content.digicam_configure.engine_cutoff_time);
        return true;

    case MAV_CMD_DO_DIGICAM_CONTROL:                    // Mission command to control an on-board camera controller system. |Session control e.g. show/hide lens| Zoom's absolute position| Zooming step value to offset zoom from the current position| Focus Locking, Unlocking or Re-locking| Shooting Command| Command Identity| Empty|
        camera->control(
            cmd.content.digicam_control.session,
            cmd.content.digicam_control.zoom_pos,
            cmd.content.digicam_control.zoom_step,
            cmd.content.digicam_control.focus_lock,
            cmd.content.digicam_control.shooting_cmd,
            cmd.content.digicam_control.cmd_id);
        return true;

    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
        camera->set_trigger_distance(cmd.content.cam_trigg_dist.meters);
        if (cmd.content.cam_trigg_dist.trigger == 1) {
            camera->take_picture();
        }
        return true;

    default:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Unhandled camera case");
#endif
        return false;
    }
}

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
    gcs().send_text(MAV_SEVERITY_INFO, "Resume repeat dist set to %u m",_repeat_dist);
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
    // check flags for change to RETRACT
    if ((cmd.content.gimbal_manager_pitchyaw.flags & GIMBAL_MANAGER_FLAGS_RETRACT) > 0) {
        mount->set_mode(MAV_MOUNT_MODE_RETRACT);
        return true;
    }
    // check flags for change to NEUTRAL
    if ((cmd.content.gimbal_manager_pitchyaw.flags & GIMBAL_MANAGER_FLAGS_NEUTRAL) > 0) {
        mount->set_mode(MAV_MOUNT_MODE_NEUTRAL);
        return true;
    }

    // To-Do: handle gimbal device id

    // handle angle target
    const bool pitch_angle_valid = !isnan(cmd.content.gimbal_manager_pitchyaw.pitch_angle_deg) && (fabsf(cmd.content.gimbal_manager_pitchyaw.pitch_angle_deg) <= 90);
    const bool yaw_angle_valid = !isnan(cmd.content.gimbal_manager_pitchyaw.yaw_angle_deg) && (fabsf(cmd.content.gimbal_manager_pitchyaw.yaw_angle_deg) <= 360);
    if (pitch_angle_valid && yaw_angle_valid) {
        mount->set_angle_target(0, cmd.content.gimbal_manager_pitchyaw.pitch_angle_deg, cmd.content.gimbal_manager_pitchyaw.yaw_angle_deg, cmd.content.gimbal_manager_pitchyaw.flags & GIMBAL_MANAGER_FLAGS_YAW_LOCK);
        return true;
    }

    // handle rate target
    if (!isnan(cmd.content.gimbal_manager_pitchyaw.pitch_rate_degs) && !isnan(cmd.content.gimbal_manager_pitchyaw.yaw_rate_degs)) {
        mount->set_rate_target(0, cmd.content.gimbal_manager_pitchyaw.pitch_rate_degs, cmd.content.gimbal_manager_pitchyaw.yaw_rate_degs, cmd.content.gimbal_manager_pitchyaw.flags & GIMBAL_MANAGER_FLAGS_YAW_LOCK);
        return true;
    }

    // if we got this far then message is not handled
    return false;
#else
    return false;
#endif // HAL_MOUNT_ENABLED
}
