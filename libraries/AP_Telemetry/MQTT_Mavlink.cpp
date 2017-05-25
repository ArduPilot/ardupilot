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

#include <stdio.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <string>
#include "AP_Telemetry_MQTT.h"
#include "define_MQTT.h"

extern int mqtt_to_mavlink_message(const char* mqtt_cmd, mavlink_message_t *msg)
{
    int ret;
    char cmd[MAX_PAYLOAD];

    strcpy(cmd, mqtt_cmd);
    ret = 0;
    AP_Telemetry_MQTT* tele_mqtt = AP_Telemetry_MQTT::get_telemetry_mqtt();
    char topic[MAX_TOPIC];

    printf("received mqtt from Pc %s \n", cmd);
    if (strncmp(cmd, "arm", 3) == 0) {
        sprintf(topic, "$ardupilot/copter/quad/ack/%04d", mavlink_system.sysid);
        //arm コマンド発行
        memset(msg, 0, sizeof(mavlink_message_t));
        msg->msgid = MAVLINK_MSG_ID_COMMAND_LONG;
        mavlink_msg_command_long_pack_chan(
            0,0,0,
            msg,
            0,0,MAV_CMD_COMPONENT_ARM_DISARM,0,
            1.0,0.0,0.0,0.0,0.0,0.0,0.0);
        tele_mqtt->send_message("Arm command received", topic);
        ret = 1;
    } else if (strncmp(cmd, "disarm", 6) == 0) {
        sprintf(topic, "$ardupilot/copter/quad/ack/%04d", mavlink_system.sysid);
        tele_mqtt->send_message("Disarm command received", topic);
        // disarm コマンド発行
        memset(msg, 0, sizeof(mavlink_message_t));
        msg->msgid = MAVLINK_MSG_ID_COMMAND_LONG;
        mavlink_msg_command_long_pack_chan(
            0,0,0,
            msg,
            0,0,MAV_CMD_COMPONENT_ARM_DISARM,0,
            0.0,0.0,0.0,0.0,0.0,0.0,0.0);
        ret = 1;
    } else if (strncmp(cmd, "takeoff", 7) == 0) {
        float takeoff_alt = 20;// param7
        takeoff_alt = atof(&cmd[8]);
        char payload[MAX_PAYLOAD];
        sprintf(topic, "$ardupilot/copter/quad/ack/%04d", mavlink_system.sysid);
        sprintf(payload, "Took off to %f", takeoff_alt);
        tele_mqtt->send_message(payload, topic);
        if (strlen(cmd) >= 9) {
            takeoff_alt = atof(&cmd[8]);
        }
        float hnbpa = 1.0; // param 3 horizontal navigation by pilot acceptable
        memset(msg, 0, sizeof(mavlink_message_t));
        msg->msgid = MAVLINK_MSG_ID_COMMAND_LONG;
        mavlink_msg_command_long_pack_chan(
            0,0,0,
            msg,
            0,0,MAV_CMD_NAV_TAKEOFF,0,
            0.0,0.0,hnbpa,0.0,0.0,0.0,takeoff_alt);
        ret = 1;
    } else if (strncmp(cmd, "mode land", 9) == 0) {
        //mode land
        char buf[30] = {0};
        float hnbpa = 1.0; // param 3 horizontal navigation by pilot acceptable
        msg->msgid = MAVLINK_MSG_ID_COMMAND_LONG;
        mavlink_msg_command_long_pack_chan(
            0,0,0,
            msg,
            0,0,MAV_CMD_NAV_LAND,0,
            0.0,0.0,hnbpa,0.0,0.0,0.0,0.0);
        msg->msgid = MAVLINK_MSG_ID_SET_MODE;
        msg->len = 6;
        _mav_put_uint32_t(buf, 0,9);
        _mav_put_uint8_t(buf, 4,1);
        _mav_put_uint8_t(buf, 5,1);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 30);// 30 is about
        memset(msg, 0, sizeof(mavlink_message_t));
        sprintf(topic, "$ardupilot/copter/quad/ack/%04d", mavlink_system.sysid);
        tele_mqtt->send_message("Landing receive", topic);
        ret = 1;
    } else if ((strncmp(cmd, "mode alt_hold", 13) == 0) || (strncmp(cmd, "mode althold", 12) == 0)) {
        //mode alt hold
        char buf[30] = {0};
        memset(msg, 0, sizeof(mavlink_message_t));
        msg->msgid = MAVLINK_MSG_ID_SET_MODE;
        msg->len = 6;
        _mav_put_uint32_t(buf, 0,2);
        _mav_put_uint8_t(buf, 4,1);
        _mav_put_uint8_t(buf, 5,1);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 30);// 30 is about
        memset(msg, 0, sizeof(mavlink_message_t));
        sprintf(topic, "$ardupilot/copter/quad/ack/%04d", mavlink_system.sysid);
        tele_mqtt->send_message("Alt old recevied", topic);
        ret = 1;
    } else if (strncmp(cmd, "mode loiter", 11) == 0) {
        //mode loiter
        char buf[30]= {0};
        memset(msg, 0, sizeof(mavlink_message_t));
        msg->msgid = MAVLINK_MSG_ID_SET_MODE;
        msg->len = 6;
        _mav_put_uint32_t(buf, 0,5);
        _mav_put_uint8_t(buf, 4,1);
        _mav_put_uint8_t(buf, 5,1);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 30);// 30 is about
        memset(msg, 0, sizeof(mavlink_message_t));
        sprintf(topic, "$ardupilot/copter/quad/ack/%04d", mavlink_system.sysid);
        tele_mqtt->send_message("Loiter mode received", topic);
        ret = 1;
    } else if (strncmp(cmd, "mode guided", 11) == 0) {
        //mode guided
        char buf[30] = {0};
        memset(msg, 0, sizeof(mavlink_message_t));
        msg->msgid = MAVLINK_MSG_ID_SET_MODE;
        msg->len = 6;
        _mav_put_uint32_t(buf, 0,4);
        _mav_put_uint8_t(buf, 4,1);
        _mav_put_uint8_t(buf, 5,1);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 30);// 30 is about
        memset(msg, 0, sizeof(mavlink_message_t));
        sprintf(topic, "$ardupilot/copter/quad/ack/%04d", mavlink_system.sysid);
        tele_mqtt->send_message("Guided mode received", topic);
        ret = 1;
    } else if (strncmp(cmd, "mode circle", 11) == 0) {
        //mode circle
        char buf[30] = {0};
        memset(buf, 0, sizeof(buf));
        memset(msg, 0, sizeof(mavlink_message_t));
        msg->msgid = MAVLINK_MSG_ID_SET_MODE;
        msg->len = 6;
        _mav_put_uint32_t(buf, 0,7);
        _mav_put_uint8_t(buf, 4,1);
        _mav_put_uint8_t(buf, 5,1);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 30);// 30 is about
        memset(msg, 0, sizeof(mavlink_message_t));
        sprintf(topic, "$ardupilot/copter/quad/ack/%04d", mavlink_system.sysid);
        tele_mqtt->send_message("Circle mode received", topic);
        ret = 1;
    } else if (strncmp(cmd, "mode stabilize", 14) == 0) {
        //mode stabilize
        char buf[30] = {0};
        memset(msg, 0, sizeof(mavlink_message_t));
        msg->msgid = MAVLINK_MSG_ID_SET_MODE;
        msg->len = 6;
        _mav_put_uint32_t(buf, 0,0);
        _mav_put_uint8_t(buf, 4,1);
        _mav_put_uint8_t(buf, 5,1);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 30);// 30 is about
        memset(msg, 0, sizeof(mavlink_message_t));
        sprintf(topic, "$ardupilot/copter/quad/ack/%04d", mavlink_system.sysid);
        tele_mqtt->send_message("Stabilized mode received", topic);
        ret = 1;
    } else if (strncmp(cmd, "mode rtl", 8) == 0) {
        //mode rtl
        char buf[30] = {0};
        memset(msg, 0, sizeof(mavlink_message_t));
        msg->msgid = MAVLINK_MSG_ID_SET_MODE;
        msg->len = 6;
        _mav_put_uint32_t(buf, 0,6);
        _mav_put_uint8_t(buf, 4,1);
        _mav_put_uint8_t(buf, 5,1);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 30);// 30 is about
        memset(msg, 0, sizeof(mavlink_message_t));
        sprintf(topic, "$ardupilot/copter/quad/ack/%04d", mavlink_system.sysid);
        tele_mqtt->send_message("RTL received", topic);
        ret = 1;
    } else if (strncmp(cmd, "flyto", 5) == 0) {
        char buf[40] = {0};
        memset(msg, 0, sizeof(mavlink_message_t));
        msg->msgid = MAVLINK_MSG_ID_MISSION_ITEM;
        msg->len = 36;
        mavlink_mission_item_t mission_item;
        mission_item.param1 = 0.0; // 0 float
        _mav_put_float(buf, 0, mission_item.param1);
        mission_item.param2 = 0.0; // 4 float
        _mav_put_float(buf, 4, mission_item.param2);
        mission_item.param3 = 0.0; // 8 float
        _mav_put_float(buf, 8, mission_item.param3);
        mission_item.param4 = 0.0; // 12 float
        _mav_put_float(buf, 12, mission_item.param4);
        mission_item.x = 0;
        mission_item.y = 0;
        mission_item.z = 0;
        if (strlen(cmd) >= 7) {
            char *token;
            token = strtok(&cmd[6], ",");
            if (token != nullptr) {
                mission_item.x = atof(token); // 16 float
            } else {
                return 0;
            }
            token = strtok(nullptr, ",");
            if (token != nullptr) {
                mission_item.y = atof(token); // 16 float
            } else {
                return 0;
            }
            token = strtok(nullptr, ",");
            if (token != nullptr) {
                mission_item.z = atof(token); // 16 float
            } else {
                return 0;
            }
        }
        _mav_put_float(buf, 16,mission_item.x);
        _mav_put_float(buf, 20,mission_item.y);
        _mav_put_float(buf, 24,mission_item.z);

        mission_item.seq = 0;// 28 uint16
        _mav_put_uint16_t(buf, 28, mission_item.seq);
        mission_item.command = 16; // 30 uint16
        _mav_put_uint16_t(buf, 30, mission_item.command);
        mission_item.target_system = 1; //32 uint8
        _mav_put_uint16_t(buf, 32, mission_item.target_system);
        mission_item.target_component = 0; // 33 uint8
        _mav_put_uint16_t(buf, 33, mission_item.target_component);
        mission_item.frame = 3; // 34 uint8
        _mav_put_uint16_t(buf, 34, mission_item.frame);
        mission_item.current = 2; // 35 uint8
        _mav_put_uint16_t(buf, 35, mission_item.current);
        mission_item.autocontinue = 0; // 36 uint8
        _mav_put_uint16_t(buf, 36, mission_item.autocontinue);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 40);// 30 is about
        memset(msg, 0, sizeof(mavlink_message_t));
        sprintf(topic, "$ardupilot/copter/quad/ack/%04d", mavlink_system.sysid);
        tele_mqtt->send_message("Fly to received", topic);
        ret = 1;
    } else if (strncmp(cmd, "param set circle_radius", 23) == 0) {
        mavlink_param_set_t packet1;
        unsigned char system_id;
        unsigned char component_id;

        system_id = 0;
        component_id = 0;
        memset(&packet1, 0, sizeof(packet1));
        packet1.param_value = atof(&cmd[23]);
        packet1.target_system = 0;
        packet1.target_component = 0;
        packet1.param_type = MAV_PARAM_TYPE_REAL32;
        strcpy(packet1.param_id, "CIRCLE_RADIUS");
        mavlink_msg_param_set_pack(system_id,
                                   component_id, msg ,
                                   packet1.target_system ,
                                   packet1.target_component ,
                                   packet1.param_id ,
                                   packet1.param_value ,
                                   packet1.param_type);
        memset(msg, 0, sizeof(mavlink_message_t));
        sprintf(topic, "$ardupilot/copter/quad/ack/%04d", mavlink_system.sysid);
        tele_mqtt->send_message("Param for circle received", topic);
        ret = 1;
    } else if (strcmp(cmd, "mqtt log_off") == 0) {
        tele_mqtt->send_log_flag = MQTT_SEND_LOG_OFF;
        ret = 1;
    } else if (strcmp(cmd, "mqtt log_on") == 0) {
        tele_mqtt->send_log_flag = MQTT_SEND_LOG_ON;
        ret = 1;
    }
    return ret;
}
