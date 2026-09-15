/*
   Simulator mixin for MAVLink Camera Protocol v2 peripherals.
*/

#include "SIM_config.h"

#if AP_SIM_MAVLINKCAMV2_ENABLED

#include "SIM_MAVLinkCamV2.h"
#include <AP_HAL/AP_HAL.h>
#include <stdio.h>

namespace SITL {

void MAVLinkCamV2::set_camera_instance(uint8_t instance)
{
    _camera_compid = MIN(MAV_COMP_ID_CAMERA + instance, (uint8_t)MAV_COMP_ID_CAMERA6);
}

void MAVLinkCamV2::update(const class Aircraft &aircraft)
{
    if (camera_vehicle_sysid() == 0) {
        return;
    }
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_camera_heartbeat_ms >= 1000) {
        _last_camera_heartbeat_ms = now_ms;
        send_camera_heartbeat();
    }
}

void MAVLinkCamV2::handle_message(const mavlink_message_t &msg)
{
    if (msg.msgid != MAVLINK_MSG_ID_COMMAND_LONG) {
        return;
    }
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(&msg, &cmd);
    if (cmd.target_system    != camera_vehicle_sysid() ||
        cmd.target_component != _camera_compid) {
        return;
    }
    switch ((MAV_CMD)cmd.command) {
    case MAV_CMD_REQUEST_MESSAGE:
        if ((uint32_t)cmd.param1 == MAVLINK_MSG_ID_CAMERA_INFORMATION) {
            send_camera_information(msg.sysid, msg.compid);
            send_camera_command_ack(msg.sysid, msg.compid,
                                    MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_ACCEPTED);
        } else if ((uint32_t)cmd.param1 == MAVLINK_MSG_ID_CAMERA_SETTINGS) {
            send_camera_settings();
            send_camera_command_ack(msg.sysid, msg.compid,
                                    MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_ACCEPTED);
        } else if ((uint32_t)cmd.param1 == MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS) {
            send_camera_capture_status();
            send_camera_command_ack(msg.sysid, msg.compid,
                                    MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_ACCEPTED);
        } else if ((uint32_t)cmd.param1 == MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION) {
            MAV_RESULT result = MAV_RESULT_DENIED;
            const uint8_t stream_count = get_video_stream_count();
            if (isfinite(cmd.param2) && cmd.param2 >= 0 &&
                cmd.param2 <= stream_count) {
                const uint8_t requested_stream = (uint8_t)cmd.param2;
                if (is_equal(cmd.param2, (float)requested_stream)) {
                    if (requested_stream == 0) {
                        result = stream_count > 0 ? MAV_RESULT_ACCEPTED : MAV_RESULT_DENIED;
                        for (uint8_t stream_id = 1; stream_id <= stream_count; stream_id++) {
                            if (!send_video_stream_information(stream_id)) {
                                result = MAV_RESULT_FAILED;
                            }
                        }
                    } else if (send_video_stream_information(requested_stream)) {
                        result = MAV_RESULT_ACCEPTED;
                    }
                }
            }
            send_camera_command_ack(msg.sysid, msg.compid,
                                    MAV_CMD_REQUEST_MESSAGE, result);
        } else {
            send_camera_command_ack(msg.sysid, msg.compid,
                                    MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_DENIED);
        }
        break;
    case MAV_CMD_IMAGE_START_CAPTURE:
        _camera.trigger_shutter();
        ::printf("MAVLinkCamV2[compid=%u]: image captured (shot %u)\n",
                 (unsigned)_camera_compid, (unsigned)_camera.shot_count());
        send_camera_command_ack(msg.sysid, msg.compid,
                                MAV_CMD_IMAGE_START_CAPTURE, MAV_RESULT_ACCEPTED);
        send_camera_capture_status();
        break;
    case MAV_CMD_IMAGE_STOP_CAPTURE:
        send_camera_command_ack(msg.sysid, msg.compid,
                                MAV_CMD_IMAGE_STOP_CAPTURE, MAV_RESULT_ACCEPTED);
        send_camera_capture_status();
        break;
    case MAV_CMD_VIDEO_START_CAPTURE:
        _camera.set_recording(true);
        _recording_started_ms = AP_HAL::millis();
        send_camera_command_ack(msg.sysid, msg.compid,
                                MAV_CMD_VIDEO_START_CAPTURE, MAV_RESULT_ACCEPTED);
        send_camera_capture_status();
        break;
    case MAV_CMD_VIDEO_STOP_CAPTURE:
        _camera.set_recording(false);
        _recording_started_ms = 0;
        send_camera_command_ack(msg.sysid, msg.compid,
                                MAV_CMD_VIDEO_STOP_CAPTURE, MAV_RESULT_ACCEPTED);
        send_camera_capture_status();
        break;
    case MAV_CMD_SET_CAMERA_MODE:
        if (isfinite(cmd.param2) &&
            (is_equal(cmd.param2, (float)CAMERA_MODE_IMAGE) ||
             is_equal(cmd.param2, (float)CAMERA_MODE_VIDEO))) {
            _camera.set_mode((uint8_t)cmd.param2);
            send_camera_command_ack(msg.sysid, msg.compid,
                                    MAV_CMD_SET_CAMERA_MODE, MAV_RESULT_ACCEPTED);
            send_camera_settings();
        } else {
            send_camera_command_ack(msg.sysid, msg.compid,
                                    MAV_CMD_SET_CAMERA_MODE, MAV_RESULT_DENIED);
        }
        break;
    case MAV_CMD_SET_CAMERA_ZOOM:
        if ((uint32_t)cmd.param1 == ZOOM_TYPE_RANGE) {
            _camera.set_zoom_pct(cmd.param2);
            ::printf("MAVLinkCamV2[compid=%u]: zoom set to %.1f%%\n",
                     (unsigned)_camera_compid, (double)cmd.param2);
            send_camera_command_ack(msg.sysid, msg.compid,
                                    MAV_CMD_SET_CAMERA_ZOOM, MAV_RESULT_ACCEPTED);
            send_camera_settings();
        } else {
            send_camera_command_ack(msg.sysid, msg.compid,
                                    MAV_CMD_SET_CAMERA_ZOOM, MAV_RESULT_DENIED);
        }
        break;
    case MAV_CMD_SET_CAMERA_FOCUS:
        if ((uint32_t)cmd.param1 == FOCUS_TYPE_RANGE) {
            _camera.set_focus_pct(cmd.param2);
            ::printf("MAVLinkCamV2[compid=%u]: focus set to %.1f%%\n",
                     (unsigned)_camera_compid, (double)cmd.param2);
            send_camera_command_ack(msg.sysid, msg.compid,
                                    MAV_CMD_SET_CAMERA_FOCUS, MAV_RESULT_ACCEPTED);
            send_camera_settings();
        } else {
            send_camera_command_ack(msg.sysid, msg.compid,
                                    MAV_CMD_SET_CAMERA_FOCUS, MAV_RESULT_DENIED);
        }
        break;
    default:
        send_camera_command_ack(msg.sysid, msg.compid, (MAV_CMD)cmd.command,
                                MAV_RESULT_UNSUPPORTED);
        break;
    }
}

void MAVLinkCamV2::send_camera_heartbeat()
{
    mavlink_heartbeat_t hb {};
    hb.type            = MAV_TYPE_CAMERA;
    hb.autopilot       = MAV_AUTOPILOT_INVALID;
    hb.system_status   = MAV_STATE_ACTIVE;
    hb.mavlink_version = 3;

    mavlink_message_t msg;
    mavlink_msg_heartbeat_encode_status(
        camera_vehicle_sysid(), _camera_compid,
        &camera_mav_status(), &msg, &hb);
    camera_send_mavlink_message(msg);
}

void MAVLinkCamV2::send_camera_information(uint8_t target_sysid, uint8_t target_compid)
{
    mavlink_camera_information_t info {};
    info.time_boot_ms     = AP_HAL::millis();
    info.firmware_version = get_camera_firmware_version();
    info.focal_length     = NAN;
    info.sensor_size_h    = NAN;
    info.sensor_size_v    = NAN;
    info.resolution_h     = get_camera_resolution_h();
    info.resolution_v     = get_camera_resolution_v();
    info.flags            = get_camera_cap_flags();
    info.gimbal_device_id = get_camera_gimbal_device_id();
    strncpy_noterm((char *)info.vendor_name, get_camera_vendor_name(), sizeof(info.vendor_name));
    strncpy_noterm((char *)info.model_name,  get_camera_model_name(),  sizeof(info.model_name));

    mavlink_message_t msg;
    mavlink_msg_camera_information_encode_status(
        camera_vehicle_sysid(), _camera_compid,
        &camera_mav_status(), &msg, &info);
    camera_send_mavlink_message(msg);
}

void MAVLinkCamV2::send_camera_settings()
{
    mavlink_camera_settings_t settings {};
    settings.time_boot_ms = AP_HAL::millis();
    settings.mode_id      = _camera.mode();
    settings.zoomLevel    = _camera.zoom_pct();
    settings.focusLevel   = _camera.focus_pct();

    mavlink_message_t msg;
    mavlink_msg_camera_settings_encode_status(
        camera_vehicle_sysid(), _camera_compid,
        &camera_mav_status(), &msg, &settings);
    camera_send_mavlink_message(msg);
}

void MAVLinkCamV2::send_camera_capture_status()
{
    mavlink_camera_capture_status_t status {};
    status.time_boot_ms = AP_HAL::millis();
    status.video_status = _camera.recording() ? 1 : 0;
    status.recording_time_ms = _camera.recording() ?
        status.time_boot_ms - _recording_started_ms : 0;
    status.available_capacity = NAN;
    status.image_count = _camera.shot_count();

    mavlink_message_t msg;
    mavlink_msg_camera_capture_status_encode_status(
        camera_vehicle_sysid(), _camera_compid,
        &camera_mav_status(), &msg, &status);
    camera_send_mavlink_message(msg);
}

bool MAVLinkCamV2::send_video_stream_information(uint8_t stream_id)
{
    mavlink_video_stream_information_t info {};
    if (stream_id == 0 || stream_id > get_video_stream_count() ||
        !get_video_stream_information(stream_id, info)) {
        return false;
    }
    info.stream_id = stream_id;
    info.count = get_video_stream_count();

    mavlink_message_t msg;
    mavlink_msg_video_stream_information_encode_status(
        camera_vehicle_sysid(), _camera_compid,
        &camera_mav_status(), &msg, &info);
    camera_send_mavlink_message(msg);
    return true;
}

void MAVLinkCamV2::send_camera_command_ack(uint8_t target_sysid, uint8_t target_compid,
                                            MAV_CMD cmd, MAV_RESULT result)
{
    mavlink_command_ack_t ack {};
    ack.command          = (uint16_t)cmd;
    ack.result           = (uint8_t)result;
    ack.progress         = 255;
    ack.target_system    = target_sysid;
    ack.target_component = target_compid;

    mavlink_message_t msg;
    mavlink_msg_command_ack_encode_status(
        camera_vehicle_sysid(), _camera_compid,
        &camera_mav_status(), &msg, &ack);
    camera_send_mavlink_message(msg);
}

}  // namespace SITL

#endif  // AP_SIM_MAVLINKCAMV2_ENABLED
