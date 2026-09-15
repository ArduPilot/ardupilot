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
        mavlink_heartbeat_t heartbeat;
        mavlink_msg_heartbeat_decode(&msg, &heartbeat);
        if (heartbeat.autopilot == MAV_AUTOPILOT_INVALID) {
            break;
        }
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
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        if (!_seen_autopilot_heartbeat || msg.sysid != _vehicle_system_id ||
            msg.compid != _vehicle_component_id) {
            break;
        }
        mavlink_global_position_int_t packet;
        mavlink_msg_global_position_int_decode(&msg, &packet);
        _estimator.handle_position(packet, AP_HAL::millis());
        break;
    }
    case MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE: {
        if (!_seen_autopilot_heartbeat || msg.sysid != _vehicle_system_id ||
            msg.compid != _vehicle_component_id) {
            break;
        }
        mavlink_autopilot_state_for_gimbal_device_t packet;
        mavlink_msg_autopilot_state_for_gimbal_device_decode(&msg, &packet);
        if ((packet.target_system != 0 && packet.target_system != _vehicle_system_id) ||
            (packet.target_component != 0 && packet.target_component != _compid)) {
            break;
        }
        _estimator.handle_attitude(packet, AP_HAL::millis());
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
            MAV_RESULT result = MAV_RESULT_DENIED;
            const bool global_frame = cmd.frame == MAV_FRAME_GLOBAL ||
                                      cmd.frame == MAV_FRAME_GLOBAL_INT;
            if (global_frame &&
                (get_cap_flags() & GIMBAL_DEVICE_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL) != 0) {
                _roi.loc.lat = cmd.x;
                _roi.loc.lng = cmd.y;
                _roi.loc.set_alt_m(cmd.z, Location::AltFrame::ABSOLUTE);
                _roi.valid = true;
                result = MAV_RESULT_ACCEPTED;
            }
            send_command_ack(msg.sysid, msg.compid,
                             MAV_CMD_DO_SET_ROI_LOCATION, result);
        } else if (cmd.command == MAV_CMD_DO_SET_ROI_NONE) {
            _roi.valid = false;
            send_command_ack(msg.sysid, msg.compid,
                             MAV_CMD_DO_SET_ROI_NONE, MAV_RESULT_ACCEPTED);
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
        bool yaw_ef;
        if (!decode_yaw_frame(flags, yaw_ef)) {
            break;
        }
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
    const uint32_t cap_flags = get_cap_flags();
    info.cap_flags = (uint16_t)cap_flags;
    info.cap_flags2 = cap_flags;
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
    // Truth is available only to the physical sensor/actuator simulation.
    _gimbal.update(aircraft);
    Vector3f encoders;
    _gimbal.get_joint_angles(encoders);
    const uint32_t now_ms = AP_HAL::millis();
    if (!_estimator.get_attitude(now_ms, encoders, _vehicle_dcm, _gimbal_dcm)) {
        _gimbal.set_demanded_rates(Vector3f{});
        return;
    }

    // if tracking an ROI, continuously recompute the earth-frame target
    // (replicates AP_Mount_Backend::get_angle_target_to_location())
    if (_roi.valid) {
        Location veh;
        if (!_estimator.get_location(now_ms, veh)) {
            _gimbal.set_demanded_rates(Vector3f{});
            return;
        }
        const float gps_x = Location::diff_longitude(_roi.loc.lng, veh.lng)
            * cosf(radians((veh.lat + _roi.loc.lat) * 0.00000005f)) * 0.01113195f;
        const float gps_y = (_roi.loc.lat - veh.lat) * 0.01113195f;
        int32_t target_alt_cm = 0, veh_alt_cm = 0;
        if (_roi.loc.get_alt_cm(Location::AltFrame::ABSOLUTE, target_alt_cm) &&
            veh.get_alt_cm(Location::AltFrame::ABSOLUTE, veh_alt_cm)) {
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
            demanded_rates = rate_target_body(_vehicle_dcm, _gimbal_dcm, _estimator.vehicle_rates(),
                                             _target.rates_rads, _target.yaw_is_ef);
            _gimbal.set_demanded_rates(demanded_rates);
        } else {
            // Angle control uses the estimated attitude, including encoders.
            // clamp target pitch to hardware joint limits before computing error
            float t_r, t_p, t_y;
            _target.attitude.to_euler(t_r, t_p, t_y);
            t_p = constrain_float(t_p, get_pitch_min_rad(), get_pitch_max_rad());
            Quaternion target_clamped;
            target_clamped.from_euler(t_r, t_p, t_y);
            target_clamped = attitude_target_quaternion(_vehicle_dcm, target_clamped, _target.yaw_is_ef);
            Quaternion q_current;
            q_current.from_rotation_matrix(_gimbal_dcm);
            Quaternion q_error = q_current.inverse() * target_clamped;
            q_error.normalize();
            Vector3f av(q_error.q2, q_error.q3, q_error.q4);
            if (q_error.q1 < 0.0f) {
                av = -av;
            }
            const float attitude_gain = 2.0f;  // rad/s per radian of error
            demanded_rates = av * (2.0f * attitude_gain);
            if (!_target.yaw_is_ef) {
                demanded_rates += rate_target_body(_vehicle_dcm, _gimbal_dcm, _estimator.vehicle_rates(),
                                                  Vector3f{}, false);
            }
            _gimbal.set_demanded_rates(demanded_rates);
        }
    } else {
        // neutral: track vehicle body using vehicle angular rate feedforward
        const Vector3f vehicle_rate_gimbal = _gimbal_dcm.transposed() * _vehicle_dcm * _estimator.vehicle_rates();
        _gimbal.set_demanded_rates(vehicle_rate_gimbal);
    }
}

bool MAVLinkGimbalv2::decode_yaw_frame(uint16_t flags, bool &yaw_is_ef)
{
    const uint16_t frame_flags = flags & (GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME |
                                          GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME);
    switch (frame_flags) {
    case GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME:
        yaw_is_ef = true;
        return true;
    case GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME:
        yaw_is_ef = false;
        return true;
    case 0:
        yaw_is_ef = (flags & GIMBAL_DEVICE_FLAGS_YAW_LOCK) != 0;
        return true;
    default:
        // Both explicit frames set is an invalid command.
        return false;
    }
}

Quaternion MAVLinkGimbalv2::attitude_target_quaternion(const Matrix3f &vehicle_dcm,
                                                    const Quaternion &target, bool yaw_is_ef)
{
    if (yaw_is_ef) {
        return target;
    }
    float vehicle_yaw;
    vehicle_dcm.to_euler(nullptr, nullptr, &vehicle_yaw);
    Quaternion heading;
    heading.from_euler(0, 0, vehicle_yaw);
    return heading * target;
}

Vector3f MAVLinkGimbalv2::rate_target_body(const Matrix3f &vehicle_dcm, const Matrix3f &gimbal_dcm,
                                        const Vector3f &vehicle_rates, const Vector3f &target, bool yaw_is_ef)
{
    Vector3f rates_ef = target;
    if (!yaw_is_ef) {
        float vehicle_yaw;
        vehicle_dcm.to_euler(nullptr, nullptr, &vehicle_yaw);
        Matrix3f heading;
        heading.from_euler(0, 0, vehicle_yaw);
        rates_ef = heading * target;
        // Only the heading frame's yaw moves with the vehicle. Euler yaw
        // rate is (q*sin(roll) + r*cos(roll))/cos(pitch), not body gyro z.
        const float cos_pitch_sq = sq(vehicle_dcm.c.y) + sq(vehicle_dcm.c.z);
        if (!is_zero(cos_pitch_sq)) {
            rates_ef.z += (vehicle_dcm.c.y * vehicle_rates.y +
                           vehicle_dcm.c.z * vehicle_rates.z) / cos_pitch_sq;
        }
    }
    return gimbal_dcm.transposed() * rates_ef;
}

Quaternion MAVLinkGimbalv2::attitude_status_quaternion(const Matrix3f &vehicle_dcm,
                                                    const Matrix3f &gimbal_dcm, bool yaw_is_ef)
{
    Quaternion q;
    q.from_rotation_matrix(gimbal_dcm);
    if (!yaw_is_ef) {
        // Vehicle frame means relative to its heading, not its roll/pitch.
        float roll, pitch, yaw;
        q.to_euler(roll, pitch, yaw);
        float vehicle_yaw;
        vehicle_dcm.to_euler(nullptr, nullptr, &vehicle_yaw);
        q.from_euler(roll, pitch, wrap_PI(yaw - vehicle_yaw));
    }
    return q;
}

void MAVLinkGimbalv2::send_attitude_status()
{
    if (!_seen_autopilot_heartbeat) {
        return;
    }

    uint16_t flags;
    const bool yaw_is_ef = _target.valid && _target.yaw_is_ef;
    const Quaternion q = attitude_status_quaternion(_vehicle_dcm, _gimbal_dcm, yaw_is_ef);
    if (yaw_is_ef) {
        // earth-frame target: report actual gimbal DCM so convergence is visible
        flags = GIMBAL_DEVICE_FLAGS_ROLL_LOCK |
                GIMBAL_DEVICE_FLAGS_PITCH_LOCK |
                GIMBAL_DEVICE_FLAGS_YAW_LOCK |
                GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME;
    } else {
        flags = GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME;
    }
    if ((get_cap_flags() & GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_YAW_IN_EARTH_FRAME) != 0) {
        flags |= GIMBAL_DEVICE_FLAGS_ACCEPTS_YAW_IN_EARTH_FRAME;
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
    if (!_estimator.attitude_valid(status.time_boot_ms) ||
        (_roi.valid && !_estimator.position_valid(status.time_boot_ms))) {
        status.failure_flags = GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR;
    }
    status.delta_yaw = NAN;
    status.delta_yaw_velocity = NAN;

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

void MAVLinkGimbalv2::request_telemetry()
{
    if (!_seen_autopilot_heartbeat) {
        return;
    }
    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t message_ids[] = {
        MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE,
    };
    const bool valid[] = {
        _estimator.position_valid(now_ms),
        _estimator.attitude_valid(now_ms),
    };
    for (uint8_t i = 0; i < ARRAY_SIZE(message_ids); i++) {
        if (valid[i]) {
            continue;
        }
        mavlink_command_long_t request {};
        request.target_system = _vehicle_system_id;
        request.target_component = _vehicle_component_id;
        request.command = MAV_CMD_SET_MESSAGE_INTERVAL;
        request.param1 = message_ids[i];
        request.param2 = 100000; // 10 Hz, including on private links
        mavlink_message_t msg;
        mavlink_msg_command_long_encode_status(_vehicle_system_id, _compid,
                                              &mav.status, &msg, &request);
        send_mavlink_message(msg);
    }
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
        request_telemetry();
    }

    if (now_ms - _last_attitude_status_ms >= 100) {
        _last_attitude_status_ms = now_ms;
        send_attitude_status();
    }
}

}  // namespace SITL

#endif  // AP_SIM_MAVLINKGIMBALV2_ENABLED
