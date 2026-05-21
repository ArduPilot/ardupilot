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
        }
        break;
    case MAV_CMD_IMAGE_START_CAPTURE:
        _camera.trigger_shutter();
        ::printf("MAVLinkCamV2[compid=%u]: image captured (shot %u)\n",
                 (unsigned)_camera_compid, (unsigned)_camera.shot_count());
        send_camera_command_ack(msg.sysid, msg.compid,
                                MAV_CMD_IMAGE_START_CAPTURE, MAV_RESULT_ACCEPTED);
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
    info.flags            = get_camera_cap_flags();
    strncpy_noterm((char *)info.vendor_name, get_camera_vendor_name(), sizeof(info.vendor_name));
    strncpy_noterm((char *)info.model_name,  get_camera_model_name(),  sizeof(info.model_name));

    mavlink_message_t msg;
    mavlink_msg_camera_information_encode_status(
        camera_vehicle_sysid(), _camera_compid,
        &camera_mav_status(), &msg, &info);
    camera_send_mavlink_message(msg);
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
