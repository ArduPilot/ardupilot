/*
   Simulator for MAVLink Gimbal Protocol v2 peripherals
*/

#include "SIM_config.h"

#if AP_SIM_MAVLINKGIMBALV2_ENABLED

#include "SIM_MAVLinkGimbalv2.h"
#include "SIM_Aircraft.h"
#include <AP_HAL/AP_HAL.h>
#include <stdio.h>

namespace SITL {

void MAVLinkGimbalv2::set_instance(uint8_t instance)
{
    Mount::set_instance(instance);
    // select the component ID matching this gimbal instance:
    // instance 0 → MAV_COMP_ID_GIMBAL (154)
    // instance 1 → MAV_COMP_ID_GIMBAL2 (171), 2 → 172, …
    _compid = (instance == 0) ? MAV_COMP_ID_GIMBAL
                               : MAV_COMP_ID_GIMBAL2 + (instance - 1);

    // configure joint limits from subclass declarations (roll kept at ±40°)
    _gimbal.set_joint_limits(
        Vector3f(radians(-40.0f), get_pitch_min_rad(), get_yaw_min_rad()),
        Vector3f(radians( 40.0f), get_pitch_max_rad(), get_yaw_max_rad())
    );
}

void MAVLinkGimbalv2::send_mavlink_message(const mavlink_message_t &msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    if (len > 0) {
        write_to_autopilot((char*)buf, len);
    }
}

void MAVLinkGimbalv2::update_input()
{
    uint8_t buf[128];
    const ssize_t nread = read_from_autopilot((char*)buf, sizeof(buf));
    for (ssize_t i = 0; i < nread; i++) {
        mavlink_message_t msg;
        mavlink_status_t status;
        if (mavlink_frame_char_buffer(&mav.rxmsg, &mav.status,
                                      buf[i], &msg, &status) == MAVLINK_FRAMING_OK) {
            handle_message(msg);
        }
    }
}

void MAVLinkGimbalv2::handle_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: {
        if (!_seen_autopilot_heartbeat) {
            _seen_autopilot_heartbeat = true;
            _vehicle_system_id = msg.sysid;
            _vehicle_component_id = msg.compid;
            ::printf("MAVLinkGimbalv2[%u]: using sysid=%u compid=%u\n",
                     (unsigned)_instance,
                     (unsigned)_vehicle_system_id,
                     (unsigned)_compid);
        }
        break;
    }
    case MAVLINK_MSG_ID_COMMAND_LONG: {
        mavlink_command_long_t cmd;
        mavlink_msg_command_long_decode(&msg, &cmd);
        // only handle messages addressed to this gimbal
        if (cmd.target_system != _vehicle_system_id ||
            cmd.target_component != _compid) {
            break;
        }
        if (cmd.command == MAV_CMD_REQUEST_MESSAGE) {
            const uint32_t requested_msgid = (uint32_t)cmd.param1;
            if (requested_msgid == MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION) {
                send_gimbal_device_information();
                send_command_ack(msg.sysid, msg.compid,
                                 MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_ACCEPTED);
            }
        }
        break;
    }
    case MAVLINK_MSG_ID_COMMAND_INT: {
        mavlink_command_int_t cmd;
        mavlink_msg_command_int_decode(&msg, &cmd);
        if (cmd.target_system != _vehicle_system_id ||
            cmd.target_component != _compid) {
            break;
        }
        if (cmd.command == MAV_CMD_DO_SET_ROI_LOCATION) {
            _roi.loc.lat = cmd.x;
            _roi.loc.lng = cmd.y;
            _roi.loc.set_alt_m(cmd.z, (Location::AltFrame)cmd.frame);
            _roi.valid = true;
        } else if (cmd.command == MAV_CMD_DO_SET_ROI_NONE) {
            _roi.valid = false;
        }
        break;
    }
    case MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE: {
        mavlink_gimbal_device_set_attitude_t cmd;
        mavlink_msg_gimbal_device_set_attitude_decode(&msg, &cmd);
        if (cmd.target_system != _vehicle_system_id ||
            cmd.target_component != _compid) {
            break;
        }
        const uint16_t flags = cmd.flags;
        if (flags & GIMBAL_DEVICE_FLAGS_RETRACT) {
            _target.valid = false;
            _roi.valid = false;
            break;
        }
        const bool yaw_ef = (flags & GIMBAL_DEVICE_FLAGS_YAW_LOCK) != 0;
        const bool rates_valid = !isnan(cmd.angular_velocity_x);
        const bool q_valid     = !isnan(cmd.q[0]);
        if (rates_valid || q_valid) {
            // any explicit attitude/rate command cancels ROI tracking
            _roi.valid = false;
        }
        if (rates_valid) {
            _target.valid      = true;
            _target.is_rate    = true;
            _target.yaw_is_ef  = yaw_ef;
            _target.rates_rads = Vector3f(cmd.angular_velocity_x,
                                          cmd.angular_velocity_y,
                                          cmd.angular_velocity_z);
        } else if (q_valid) {
            _target.valid     = true;
            _target.is_rate   = false;
            _target.yaw_is_ef = yaw_ef;
            _target.attitude  = Quaternion(cmd.q[0], cmd.q[1], cmd.q[2], cmd.q[3]);
        }
        break;
    }
    default:
        break;
    }
}

void MAVLinkGimbalv2::send_heartbeat()
{
    if (!_seen_autopilot_heartbeat) {
        return;
    }

    mavlink_heartbeat_t hb {};
    hb.type            = MAV_TYPE_GIMBAL;
    hb.autopilot       = MAV_AUTOPILOT_INVALID;
    hb.base_mode       = 0;
    hb.system_status   = MAV_STATE_ACTIVE;
    hb.mavlink_version = 3;

    mavlink_message_t msg;
    mavlink_msg_heartbeat_encode_status(
        _vehicle_system_id, _compid,
        &mav.status, &msg, &hb);
    send_mavlink_message(msg);
}

void MAVLinkGimbalv2::send_gimbal_device_information()
{
    mavlink_gimbal_device_information_t info {};
    info.time_boot_ms     = AP_HAL::millis();
    info.firmware_version = get_firmware_version();
    info.hardware_version = 0;
    info.roll_min  = NAN;
    info.roll_max  = NAN;
    info.pitch_min = get_pitch_min_rad();
    info.pitch_max = get_pitch_max_rad();
    info.yaw_min   = get_yaw_min_rad();
    info.yaw_max   = get_yaw_max_rad();
    info.cap_flags = get_cap_flags();
    strncpy_noterm(info.vendor_name, get_vendor_name(), sizeof(info.vendor_name));
    strncpy_noterm(info.model_name,  get_model_name(),  sizeof(info.model_name));

    mavlink_message_t msg;
    mavlink_msg_gimbal_device_information_encode_status(
        _vehicle_system_id, _compid,
        &mav.status, &msg, &info);
    send_mavlink_message(msg);
}

// update gimbal physics and compute demanded rates from the current target
void MAVLinkGimbalv2::update_gimbal(const Aircraft &aircraft)
{
    _vehicle_dcm = aircraft.get_dcm();

    Matrix3f gimbal_dcm;
    _gimbal.get_dcm(gimbal_dcm);

    // if tracking an ROI, continuously recompute the earth-frame target
    // (replicates AP_Mount_Backend::get_angle_target_to_location())
    if (_roi.valid) {
        const Location &veh = aircraft.get_location();
        const float gps_x = Location::diff_longitude(_roi.loc.lng, veh.lng)
            * cosf(radians((veh.lat + _roi.loc.lat) * 0.00000005f)) * 0.01113195f;
        const float gps_y = (_roi.loc.lat - veh.lat) * 0.01113195f;
        int32_t target_alt_cm = 0, veh_alt_cm = 0;
        if (_roi.loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, target_alt_cm) &&
            veh.get_alt_cm(Location::AltFrame::ABOVE_HOME, veh_alt_cm)) {
            const float gps_z = (float)(target_alt_cm - veh_alt_cm);  // cm
            const float horiz_cm = 100.0f * norm(gps_x, gps_y);
            float pitch_ef = atan2f(gps_z, horiz_cm);
            pitch_ef = constrain_float(pitch_ef, get_pitch_min_rad(), get_pitch_max_rad());
            const float yaw_ef = atan2f(gps_x, gps_y);
            _target.valid = true;
            _target.is_rate = false;
            _target.yaw_is_ef = true;
            _target.attitude.from_euler(0.0f, pitch_ef, yaw_ef);
        }
    }

    if (_target.valid) {
        Vector3f demanded_rates;
        if (_target.is_rate) {
            demanded_rates = _target.rates_rads;
            if (_target.yaw_is_ef) {
                // transform earth-frame rates to gimbal body frame
                demanded_rates = gimbal_dcm.transposed() * demanded_rates;
            }
            // add vehicle body rates so that rate=0 tracks the vehicle body
            // (body-relative stabilisation); matches neutral-mode behaviour
            demanded_rates += gimbal_dcm.transposed() * _vehicle_dcm * aircraft.get_gyro();
            _gimbal.set_demanded_rates(demanded_rates);
        } else if (_target.yaw_is_ef) {
            // earth-frame angle control: P-controller
            // clamp target pitch to hardware joint limits before computing error
            float t_r, t_p, t_y;
            _target.attitude.to_euler(t_r, t_p, t_y);
            t_p = constrain_float(t_p, get_pitch_min_rad(), get_pitch_max_rad());
            Quaternion target_clamped;
            target_clamped.from_euler(t_r, t_p, t_y);
            Quaternion q_current;
            q_current.from_rotation_matrix(gimbal_dcm);
            Quaternion q_error = q_current.inverse() * target_clamped;
            q_error.normalize();
            Vector3f av(q_error.q2, q_error.q3, q_error.q4);
            if (q_error.q1 < 0.0f) {
                av = -av;
            }
            const float attitude_gain = 2.0f;  // rad/s per radian of error
            demanded_rates = av * (2.0f * attitude_gain);
            _gimbal.set_demanded_rates(demanded_rates);
        } else {
            // body-frame angle control: snap gimbal DCM to the commanded
            // body-relative orientation directly
            Matrix3f body_dcm;
            _target.attitude.rotation_matrix(body_dcm);
            _gimbal.set_dcm(_vehicle_dcm * body_dcm);
            _gimbal.set_demanded_rates(Vector3f{});
        }
    } else {
        // neutral: track vehicle body using vehicle angular rate feedforward
        const Vector3f vehicle_rate_gimbal = gimbal_dcm.transposed() * _vehicle_dcm * aircraft.get_gyro();
        _gimbal.set_demanded_rates(vehicle_rate_gimbal);
    }
    _gimbal.update(aircraft);
}

void MAVLinkGimbalv2::send_attitude_status()
{
    if (!_seen_autopilot_heartbeat) {
        return;
    }

    // report actual gimbal attitude from physics model
    Matrix3f gimbal_dcm;
    _gimbal.get_dcm(gimbal_dcm);

    uint16_t flags;
    Quaternion q;
    if (_target.valid && _target.yaw_is_ef) {
        // earth-frame target: report actual gimbal DCM so convergence is visible
        flags = GIMBAL_DEVICE_FLAGS_ROLL_LOCK |
                GIMBAL_DEVICE_FLAGS_PITCH_LOCK |
                GIMBAL_DEVICE_FLAGS_YAW_LOCK;
        q.from_rotation_matrix(gimbal_dcm);
    } else if (_target.valid && !_target.yaw_is_ef && !_target.is_rate) {
        // body-frame angle target: report the commanded body-relative attitude
        // directly, matching how servo mounts report commanded angles
        flags = 0;
        q = _target.attitude;
    } else {
        // neutral or rate control: report body-relative attitude from DCM
        flags = 0;
        const Matrix3f body_to_gimbal = _vehicle_dcm.transposed() * gimbal_dcm;
        q.from_rotation_matrix(body_to_gimbal);
    }

    mavlink_gimbal_device_attitude_status_t status {};
    status.target_system    = _vehicle_system_id;
    status.target_component = _vehicle_component_id;
    status.time_boot_ms     = AP_HAL::millis();
    status.flags = flags;
    status.q[0] = q.q1;  // w
    status.q[1] = q.q2;  // x
    status.q[2] = q.q3;  // y
    status.q[3] = q.q4;  // z
    status.angular_velocity_x = 0.0f;
    status.angular_velocity_y = 0.0f;
    status.angular_velocity_z = 0.0f;
    status.failure_flags = 0;

    mavlink_message_t msg;
    mavlink_msg_gimbal_device_attitude_status_encode_status(
        _vehicle_system_id, _compid,
        &mav.status, &msg, &status);
    send_mavlink_message(msg);
}

void MAVLinkGimbalv2::send_command_ack(uint8_t target_sysid, uint8_t target_compid,
                                        MAV_CMD command, MAV_RESULT result)
{
    mavlink_command_ack_t ack {};
    ack.command          = (uint16_t)command;
    ack.result           = (uint8_t)result;
    ack.progress         = 255;
    ack.result_param2    = 0;
    ack.target_system    = target_sysid;
    ack.target_component = target_compid;

    mavlink_message_t msg;
    mavlink_msg_command_ack_encode_status(
        _vehicle_system_id, _compid,
        &mav.status, &msg, &ack);
    send_mavlink_message(msg);
}

void MAVLinkGimbalv2::update(const Aircraft &aircraft)
{
    if (!init_sitl_pointer()) {
        return;
    }

    update_input();
    update_gimbal(aircraft);

    const uint32_t now_ms = AP_HAL::millis();

    if (now_ms - _last_heartbeat_ms >= 1000) {
        _last_heartbeat_ms = now_ms;
        send_heartbeat();
    }

    if (now_ms - _last_attitude_status_ms >= 100) {
        _last_attitude_status_ms = now_ms;
        send_attitude_status();
    }
}

}  // namespace SITL

#endif  // AP_SIM_MAVLINKGIMBALV2_ENABLED
