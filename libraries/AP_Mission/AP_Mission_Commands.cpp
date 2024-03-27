#include "AP_Mission_config.h"

#if AP_MISSION_ENABLED

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
    rc().run_aux_function(function, pos, RC_Channel::AuxFuncTriggerSource::MISSION);
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

    case MAV_CMD_SET_CAMERA_ZOOM:
        if (cmd.content.set_camera_zoom.zoom_type == ZOOM_TYPE_CONTINUOUS) {
            return camera->set_zoom(ZoomType::RATE, cmd.content.set_camera_zoom.zoom_value);
        }
        if (cmd.content.set_camera_zoom.zoom_type == ZOOM_TYPE_RANGE) {
            return camera->set_zoom(ZoomType::PCT, cmd.content.set_camera_zoom.zoom_value);
        }
        return false;

    case MAV_CMD_SET_CAMERA_FOCUS:
        // accept any of the auto focus types
        if ((cmd.content.set_camera_focus.focus_type == FOCUS_TYPE_AUTO) ||
            (cmd.content.set_camera_focus.focus_type == FOCUS_TYPE_AUTO_SINGLE) ||
            (cmd.content.set_camera_focus.focus_type == FOCUS_TYPE_AUTO_CONTINUOUS)) {
            return camera->set_focus(FocusType::AUTO, 0) == SetFocusResult::ACCEPTED;
        }
        // accept continuous manual focus
        if (cmd.content.set_camera_focus.focus_type == FOCUS_TYPE_CONTINUOUS) {
            return camera->set_focus(FocusType::RATE, cmd.content.set_camera_focus.focus_value) == SetFocusResult::ACCEPTED;
        }
        // accept range manual focus
        if (cmd.content.set_camera_focus.focus_type == FOCUS_TYPE_RANGE) {
            return camera->set_focus(FocusType::PCT, cmd.content.set_camera_focus.focus_value) == SetFocusResult::ACCEPTED;
        }
        return false;

#if AP_CAMERA_SET_CAMERA_SOURCE_ENABLED
    case MAV_CMD_SET_CAMERA_SOURCE:
        if (cmd.content.set_camera_source.instance == 0) {
            // set lens for every backend
            bool ret = false;
            for (uint8_t i=0; i<AP_CAMERA_MAX_INSTANCES; i++) {
                ret |= camera->set_camera_source(i, (AP_Camera::CameraSource)cmd.content.set_camera_source.primary_source, (AP_Camera::CameraSource)cmd.content.set_camera_source.secondary_source);
            }
            return ret;
        }
        return camera->set_camera_source(cmd.content.set_camera_source.instance-1, (AP_Camera::CameraSource)cmd.content.set_camera_source.primary_source, (AP_Camera::CameraSource)cmd.content.set_camera_source.secondary_source);
#endif

    case MAV_CMD_IMAGE_START_CAPTURE:
        // check if this is a single picture request (e.g. total images is 1 or interval and total images are zero)
        if ((cmd.content.image_start_capture.total_num_images == 1) ||
            (cmd.content.image_start_capture.total_num_images == 0 && is_zero(cmd.content.image_start_capture.interval_s))) {
            if (cmd.content.image_start_capture.instance == 0) {
                // take pictures for every backend
                return camera->take_picture();
            }
            return camera->take_picture(cmd.content.image_start_capture.instance-1);
        } else if (cmd.content.image_start_capture.total_num_images == 0) {
            // multiple picture request, take pictures forever
            if (cmd.content.image_start_capture.instance == 0) {
                // take pictures for every backend
                return camera->take_multiple_pictures(cmd.content.image_start_capture.interval_s*1000, -1);
            }
            return camera->take_multiple_pictures(cmd.content.image_start_capture.instance-1, cmd.content.image_start_capture.interval_s*1000, -1);
        } else {
            if (cmd.content.image_start_capture.instance == 0) {
                // take pictures for every backend
                return camera->take_multiple_pictures(cmd.content.image_start_capture.interval_s*1000, cmd.content.image_start_capture.total_num_images);
            }
            return camera->take_multiple_pictures(cmd.content.image_start_capture.instance-1, cmd.content.image_start_capture.interval_s*1000, cmd.content.image_start_capture.total_num_images);
        }
    case MAV_CMD_IMAGE_STOP_CAPTURE:
        if (cmd.p1 == 0) {
            // stop capture for each backend
            camera->stop_capture();
            return true;
        }
        return camera->stop_capture(cmd.p1 - 1);

    case MAV_CMD_VIDEO_START_CAPTURE:
    case MAV_CMD_VIDEO_STOP_CAPTURE:
    {
        const bool start_recording = (cmd.id == MAV_CMD_VIDEO_START_CAPTURE);
        if (cmd.content.video_start_capture.video_stream_id == 0) {
            // stream id of zero interpreted as primary camera
            return camera->record_video(start_recording);
        } else {
            // non-zero stream id is converted to camera instance
            return camera->record_video(cmd.content.video_start_capture.video_stream_id - 1, start_recording);
        }
    }

    default:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Unhandled camera case");
#endif
        return false;
    }
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

    // check gimbal device id.  0 is primary, 1 is 1st gimbal, 2 is 2nd gimbal, etc
    uint8_t gimbal_instance = mount->get_primary_instance();
    if (cmd.content.gimbal_manager_pitchyaw.gimbal_id > 0) {
        gimbal_instance = cmd.content.gimbal_manager_pitchyaw.gimbal_id - 1;
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
    const bool pitch_angle_valid = !isnan(cmd.content.gimbal_manager_pitchyaw.pitch_angle_deg) && (fabsF(cmd.content.gimbal_manager_pitchyaw.pitch_angle_deg) <= 90);
    const bool yaw_angle_valid = !isnan(cmd.content.gimbal_manager_pitchyaw.yaw_angle_deg) && (fabsF(cmd.content.gimbal_manager_pitchyaw.yaw_angle_deg) <= 360);
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

#endif  // AP_MISSION_ENABLED
