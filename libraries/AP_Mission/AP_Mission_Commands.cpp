#include "AP_Mission.h"

#include <GCS_MAVLink/GCS.h>
#include <AP_Gripper/AP_Gripper.h>
#include <AP_ServoRelayEvents/AP_ServoRelayEvents.h>

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
        sre->do_set_servo(cmd.content.servo.channel, cmd.content.servo.pwm);
        return true;

    case MAV_CMD_DO_SET_RELAY:
        sre->do_set_relay(cmd.content.relay.num, cmd.content.relay.state);
        return true;

    case MAV_CMD_DO_REPEAT_SERVO:
        sre->do_repeat_servo(cmd.content.repeat_servo.channel,
                             cmd.content.repeat_servo.pwm,
                             cmd.content.repeat_servo.repeat_count,
                             cmd.content.repeat_servo.cycle_time * 1000.0f);
        return true;

    case MAV_CMD_DO_REPEAT_RELAY:
        sre->do_repeat_relay(cmd.content.repeat_relay.num,
                             cmd.content.repeat_relay.repeat_count,
                             cmd.content.repeat_relay.cycle_time * 1000.0f);
        return true;
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
        return true;

    default:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Unhandled camera case");
#endif
        return false;
    }
}

