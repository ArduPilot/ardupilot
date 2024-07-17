/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  gimbal simulator class for MAVLink gimbal
*/

#include "SIM_SoloGimbal.h"

#if AP_SIM_SOLOGIMBAL_ENABLED

#include <stdio.h>

#include "SIM_Aircraft.h"

extern const AP_HAL::HAL& hal;

#define GIMBAL_DEBUG 0

#if GIMBAL_DEBUG
#define debug(fmt, args...)  do { printf("GIMBAL: " fmt, ##args); } while(0)
#else
#define debug(fmt, args...)  do { } while(0)
#endif

namespace SITL {

/*
  update the gimbal state
*/
void SoloGimbal::update(const Aircraft &aircraft)
{
    gimbal.update(aircraft);

    // see if we should do a report
    send_report();
}

static struct gimbal_param {
    const char *name;
    float value;
} gimbal_params[] = {
    {"GMB_OFF_ACC_X", 0},
    {"GMB_OFF_ACC_Y", 0},
    {"GMB_OFF_ACC_Z", 0},
    {"GMB_GN_ACC_X", 0},
    {"GMB_GN_ACC_Y", 0},
    {"GMB_GN_ACC_Z", 0},
    {"GMB_OFF_GYRO_X", 0},
    {"GMB_OFF_GYRO_Y", 0},
    {"GMB_OFF_GYRO_Z", 0},
    {"GMB_OFF_JNT_X", 0},
    {"GMB_OFF_JNT_Y", 0},
    {"GMB_OFF_JNT_Z", 0},
    {"GMB_K_RATE", 0},
    {"GMB_POS_HOLD", 0},
    {"GMB_MAX_TORQUE", 0},
    {"GMB_SND_TORQUE", 0},
    {"GMB_SYSID", 0},
    {"GMB_FLASH", 0},
};

/*
  find a parameter structure
 */
struct gimbal_param *SoloGimbal::param_find(const char *name)
{
    for (uint8_t i=0; i<ARRAY_SIZE(gimbal_params); i++) {
        if (strncmp(name, gimbal_params[i].name, 16) == 0) {
            return &gimbal_params[i];
        }
    }
    return nullptr;
}
    
/*
  send a parameter to flight board
 */
void SoloGimbal::param_send(const struct gimbal_param *p)
{
    mavlink_message_t msg;
    mavlink_param_value_t param_value{};
    strncpy_noterm(param_value.param_id, p->name, sizeof(param_value.param_id));
    param_value.param_value = p->value;
    param_value.param_count = 0;
    param_value.param_index = 0;
    param_value.param_type = MAV_PARAM_TYPE_REAL32;

    uint16_t len = mavlink_msg_param_value_encode_status(vehicle_system_id,
                                                         gimbal_component_id,
                                                         &mavlink.status,
                                                         &msg, &param_value);

    uint8_t msgbuf[len];
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);
    if (len > 0) {
        mav_socket.send(msgbuf, len);
    }
}

    
/*
  send a report to the vehicle control code over MAVLink
*/
void SoloGimbal::send_report(void)
{
    uint32_t now = AP_HAL::millis();
    if (now < 10000) {
        // don't send gimbal reports until 10s after startup. This
        // avoids a windows threading issue with non-blocking sockets
        // and the initial wait on SERIAL0
        return;
    }
    if (!mavlink.connected && mav_socket.connect(target_address, target_port)) {
        ::printf("SoloGimbal connected to %s:%u\n", target_address, (unsigned)target_port);
        mavlink.connected = true;
    }
    if (!mavlink.connected) {
        return;
    }

    if (param_send_last_ms && now - param_send_last_ms > 100) {
        param_send(&gimbal_params[param_send_idx]);
        if (++param_send_idx == ARRAY_SIZE(gimbal_params)) {
            printf("Finished sending parameters\n");
            param_send_last_ms = 0;
        }
    }

    // check for incoming MAVLink messages
    uint8_t buf[100];
    ssize_t ret;

    while ((ret=mav_socket.recv(buf, sizeof(buf), 0)) > 0) {
        for (uint8_t i=0; i<ret; i++) {
            mavlink_message_t msg;
            mavlink_status_t status;
            if (mavlink_frame_char_buffer(&mavlink.rxmsg, &mavlink.status,
                                          buf[i],
                                          &msg, &status) == MAVLINK_FRAMING_OK) {
                switch (msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT: {
                    mavlink_heartbeat_t pkt;
                    mavlink_msg_heartbeat_decode(&msg, &pkt);
                    debug("got HB type=%u autopilot=%u base_mode=0x%x\n", pkt.type, pkt.autopilot, pkt.base_mode);
                    if (!seen_heartbeat) {
                        seen_heartbeat = true;
                        vehicle_component_id = msg.compid;
                        vehicle_system_id = msg.sysid;
                        ::printf("SoloGimbal using srcSystem %u\n", (unsigned)vehicle_system_id);
                    }
                    break;
                }
                case MAVLINK_MSG_ID_GIMBAL_CONTROL: {
                    static uint32_t counter;
                    if (counter++ % 100 == 0) {
                        printf("GIMBAL_CONTROL %u\n", counter);
                    }
                    mavlink_gimbal_control_t pkt;
                    mavlink_msg_gimbal_control_decode(&msg, &pkt);
                    gimbal.set_demanded_rates(Vector3f(pkt.demanded_rate_x,
                                                       pkt.demanded_rate_y,
                                                       pkt.demanded_rate_z));
                    seen_gimbal_control = true;
                    break;
                }
                case MAVLINK_MSG_ID_PARAM_SET: {
                    mavlink_param_set_t pkt;
                    mavlink_msg_param_set_decode(&msg, &pkt);
                    printf("SoloGimbal got PARAM_SET %.16s %f\n", pkt.param_id, pkt.param_value);

                    struct gimbal_param *p = param_find(pkt.param_id);
                    if (p) {
                        p->value = pkt.param_value;
                        param_send(p);
                    }

                    break;
                }
                case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
                    mavlink_param_request_list_t pkt;
                    mavlink_msg_param_request_list_decode(&msg, &pkt);
                    if (pkt.target_system == 0 && pkt.target_component == MAV_COMP_ID_GIMBAL) {
                        // start param send
                        param_send_idx = 0;
                        param_send_last_ms = AP_HAL::millis();
                    }
                    printf("SoloGimbal sending %u parameters\n", (unsigned)ARRAY_SIZE(gimbal_params));
                    break;
                }
                default:
                    debug("got unexpected msg %u\n", msg.msgid);
                    break;
                }
            }
        }
    }

    if (!seen_heartbeat) {
        return;
    }
    mavlink_message_t msg;
    uint16_t len;

    if (now - last_heartbeat_ms >= 1000) {
        mavlink_heartbeat_t heartbeat;
        heartbeat.type = MAV_TYPE_GIMBAL;
        heartbeat.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
        heartbeat.base_mode = 0;
        heartbeat.system_status = 0;
        heartbeat.mavlink_version = 0;
        heartbeat.custom_mode = 0;

        len = mavlink_msg_heartbeat_encode_status(vehicle_system_id,
                                                  gimbal_component_id,
                                                  &mavlink.status,
                                                  &msg, &heartbeat);

        mav_socket.send(&msg.magic, len);
        last_heartbeat_ms = now;
    }

    /*
      send a GIMBAL_REPORT message
     */
    uint32_t now_us = AP_HAL::micros();
    if (now_us - last_report_us > reporting_period_ms*1000UL) {
        last_report_us = now_us;

        uint32_t delta_time_us;
        Vector3f delta_angle;
        Vector3f delta_velocity;
        gimbal.get_deltas(delta_angle, delta_velocity, delta_time_us);

        Vector3f joint_angles;
        gimbal.get_joint_angles(joint_angles);

        mavlink_gimbal_report_t gimbal_report;
        gimbal_report.target_system = vehicle_system_id;
        gimbal_report.target_component = vehicle_component_id;
        gimbal_report.delta_time = delta_time_us * 1e-6;
        gimbal_report.delta_angle_x = delta_angle.x;
        gimbal_report.delta_angle_y = delta_angle.y;
        gimbal_report.delta_angle_z = delta_angle.z;
        gimbal_report.delta_velocity_x = delta_velocity.x;
        gimbal_report.delta_velocity_y = delta_velocity.y;
        gimbal_report.delta_velocity_z = delta_velocity.z;
        gimbal_report.joint_roll = joint_angles.x;
        gimbal_report.joint_el = joint_angles.y;
        gimbal_report.joint_az = joint_angles.z;

        len = mavlink_msg_gimbal_report_encode_status(vehicle_system_id,
                                                      gimbal_component_id,
                                                      &mavlink.status,
                                                      &msg, &gimbal_report);

        uint8_t msgbuf[len];
        len = mavlink_msg_to_send_buffer(msgbuf, &msg);
        if (len > 0) {
            mav_socket.send(msgbuf, len);
        }

        delta_velocity.zero();
        delta_angle.zero();
    }
}

} // namespace SITL

#endif  // AP_SIM_SOLOGIMBAL_ENABLED
