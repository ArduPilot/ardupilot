/** @file
 *    @brief MAVLink comm protocol testsuite generated from common.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef COMMON_TESTSUITE_H
#define COMMON_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_minimal(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_minimal(system_id, component_id, last_msg);
    mavlink_test_common(system_id, component_id, last_msg);
}
#endif

#include "../minimal/testsuite.h"


static void mavlink_test_sys_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SYS_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_sys_status_t packet_in = {
        963497464,963497672,963497880,17859,17963,18067,18171,18275,18379,18483,18587,18691,223
    };
    mavlink_sys_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.onboard_control_sensors_present = packet_in.onboard_control_sensors_present;
        packet1.onboard_control_sensors_enabled = packet_in.onboard_control_sensors_enabled;
        packet1.onboard_control_sensors_health = packet_in.onboard_control_sensors_health;
        packet1.load = packet_in.load;
        packet1.voltage_battery = packet_in.voltage_battery;
        packet1.current_battery = packet_in.current_battery;
        packet1.drop_rate_comm = packet_in.drop_rate_comm;
        packet1.errors_comm = packet_in.errors_comm;
        packet1.errors_count1 = packet_in.errors_count1;
        packet1.errors_count2 = packet_in.errors_count2;
        packet1.errors_count3 = packet_in.errors_count3;
        packet1.errors_count4 = packet_in.errors_count4;
        packet1.battery_remaining = packet_in.battery_remaining;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SYS_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SYS_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sys_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_sys_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sys_status_pack(system_id, component_id, &msg , packet1.onboard_control_sensors_present , packet1.onboard_control_sensors_enabled , packet1.onboard_control_sensors_health , packet1.load , packet1.voltage_battery , packet1.current_battery , packet1.battery_remaining , packet1.drop_rate_comm , packet1.errors_comm , packet1.errors_count1 , packet1.errors_count2 , packet1.errors_count3 , packet1.errors_count4 );
    mavlink_msg_sys_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sys_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.onboard_control_sensors_present , packet1.onboard_control_sensors_enabled , packet1.onboard_control_sensors_health , packet1.load , packet1.voltage_battery , packet1.current_battery , packet1.battery_remaining , packet1.drop_rate_comm , packet1.errors_comm , packet1.errors_count1 , packet1.errors_count2 , packet1.errors_count3 , packet1.errors_count4 );
    mavlink_msg_sys_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_sys_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sys_status_send(MAVLINK_COMM_1 , packet1.onboard_control_sensors_present , packet1.onboard_control_sensors_enabled , packet1.onboard_control_sensors_health , packet1.load , packet1.voltage_battery , packet1.current_battery , packet1.battery_remaining , packet1.drop_rate_comm , packet1.errors_comm , packet1.errors_count1 , packet1.errors_count2 , packet1.errors_count3 , packet1.errors_count4 );
    mavlink_msg_sys_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SYS_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SYS_STATUS) != NULL);
#endif
}

static void mavlink_test_system_time(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SYSTEM_TIME >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_system_time_t packet_in = {
        93372036854775807ULL,963497880
    };
    mavlink_system_time_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_unix_usec = packet_in.time_unix_usec;
        packet1.time_boot_ms = packet_in.time_boot_ms;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SYSTEM_TIME_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SYSTEM_TIME_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_system_time_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_system_time_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_system_time_pack(system_id, component_id, &msg , packet1.time_unix_usec , packet1.time_boot_ms );
    mavlink_msg_system_time_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_system_time_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_unix_usec , packet1.time_boot_ms );
    mavlink_msg_system_time_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_system_time_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_system_time_send(MAVLINK_COMM_1 , packet1.time_unix_usec , packet1.time_boot_ms );
    mavlink_msg_system_time_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SYSTEM_TIME") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SYSTEM_TIME) != NULL);
#endif
}

static void mavlink_test_ping(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PING >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_ping_t packet_in = {
        93372036854775807ULL,963497880,41,108
    };
    mavlink_ping_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.seq = packet_in.seq;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PING_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PING_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ping_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_ping_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ping_pack(system_id, component_id, &msg , packet1.time_usec , packet1.seq , packet1.target_system , packet1.target_component );
    mavlink_msg_ping_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ping_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.seq , packet1.target_system , packet1.target_component );
    mavlink_msg_ping_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_ping_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ping_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.seq , packet1.target_system , packet1.target_component );
    mavlink_msg_ping_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("PING") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_PING) != NULL);
#endif
}

static void mavlink_test_change_operator_control(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_change_operator_control_t packet_in = {
        5,72,139,"DEFGHIJKLMNOPQRSTUVWXYZA"
    };
    mavlink_change_operator_control_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.control_request = packet_in.control_request;
        packet1.version = packet_in.version;
        
        mav_array_memcpy(packet1.passkey, packet_in.passkey, sizeof(char)*25);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_change_operator_control_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_change_operator_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_change_operator_control_pack(system_id, component_id, &msg , packet1.target_system , packet1.control_request , packet1.version , packet1.passkey );
    mavlink_msg_change_operator_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_change_operator_control_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.control_request , packet1.version , packet1.passkey );
    mavlink_msg_change_operator_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_change_operator_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_change_operator_control_send(MAVLINK_COMM_1 , packet1.target_system , packet1.control_request , packet1.version , packet1.passkey );
    mavlink_msg_change_operator_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CHANGE_OPERATOR_CONTROL") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL) != NULL);
#endif
}

static void mavlink_test_change_operator_control_ack(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_change_operator_control_ack_t packet_in = {
        5,72,139
    };
    mavlink_change_operator_control_ack_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.gcs_system_id = packet_in.gcs_system_id;
        packet1.control_request = packet_in.control_request;
        packet1.ack = packet_in.ack;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_change_operator_control_ack_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_change_operator_control_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_change_operator_control_ack_pack(system_id, component_id, &msg , packet1.gcs_system_id , packet1.control_request , packet1.ack );
    mavlink_msg_change_operator_control_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_change_operator_control_ack_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.gcs_system_id , packet1.control_request , packet1.ack );
    mavlink_msg_change_operator_control_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_change_operator_control_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_change_operator_control_ack_send(MAVLINK_COMM_1 , packet1.gcs_system_id , packet1.control_request , packet1.ack );
    mavlink_msg_change_operator_control_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CHANGE_OPERATOR_CONTROL_ACK") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK) != NULL);
#endif
}

static void mavlink_test_auth_key(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_AUTH_KEY >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_auth_key_t packet_in = {
        "ABCDEFGHIJKLMNOPQRSTUVWXYZABCDE"
    };
    mavlink_auth_key_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        mav_array_memcpy(packet1.key, packet_in.key, sizeof(char)*32);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_AUTH_KEY_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_AUTH_KEY_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_auth_key_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_auth_key_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_auth_key_pack(system_id, component_id, &msg , packet1.key );
    mavlink_msg_auth_key_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_auth_key_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.key );
    mavlink_msg_auth_key_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_auth_key_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_auth_key_send(MAVLINK_COMM_1 , packet1.key );
    mavlink_msg_auth_key_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("AUTH_KEY") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_AUTH_KEY) != NULL);
#endif
}

static void mavlink_test_set_mode(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_MODE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_mode_t packet_in = {
        963497464,17,84
    };
    mavlink_set_mode_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.custom_mode = packet_in.custom_mode;
        packet1.target_system = packet_in.target_system;
        packet1.base_mode = packet_in.base_mode;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_MODE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_MODE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_mode_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_mode_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_mode_pack(system_id, component_id, &msg , packet1.target_system , packet1.base_mode , packet1.custom_mode );
    mavlink_msg_set_mode_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_mode_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.base_mode , packet1.custom_mode );
    mavlink_msg_set_mode_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_mode_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_mode_send(MAVLINK_COMM_1 , packet1.target_system , packet1.base_mode , packet1.custom_mode );
    mavlink_msg_set_mode_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SET_MODE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SET_MODE) != NULL);
#endif
}

static void mavlink_test_param_request_read(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PARAM_REQUEST_READ >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_param_request_read_t packet_in = {
        17235,139,206,"EFGHIJKLMNOPQRS"
    };
    mavlink_param_request_read_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.param_index = packet_in.param_index;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        mav_array_memcpy(packet1.param_id, packet_in.param_id, sizeof(char)*16);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PARAM_REQUEST_READ_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PARAM_REQUEST_READ_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_request_read_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_param_request_read_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_request_read_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.param_id , packet1.param_index );
    mavlink_msg_param_request_read_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_request_read_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.param_id , packet1.param_index );
    mavlink_msg_param_request_read_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_param_request_read_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_request_read_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.param_id , packet1.param_index );
    mavlink_msg_param_request_read_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("PARAM_REQUEST_READ") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_PARAM_REQUEST_READ) != NULL);
#endif
}

static void mavlink_test_param_request_list(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PARAM_REQUEST_LIST >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_param_request_list_t packet_in = {
        5,72
    };
    mavlink_param_request_list_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PARAM_REQUEST_LIST_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PARAM_REQUEST_LIST_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_request_list_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_param_request_list_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_request_list_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component );
    mavlink_msg_param_request_list_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_request_list_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component );
    mavlink_msg_param_request_list_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_param_request_list_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_request_list_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component );
    mavlink_msg_param_request_list_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("PARAM_REQUEST_LIST") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_PARAM_REQUEST_LIST) != NULL);
#endif
}

static void mavlink_test_param_value(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PARAM_VALUE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_param_value_t packet_in = {
        17.0,17443,17547,"IJKLMNOPQRSTUVW",77
    };
    mavlink_param_value_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.param_value = packet_in.param_value;
        packet1.param_count = packet_in.param_count;
        packet1.param_index = packet_in.param_index;
        packet1.param_type = packet_in.param_type;
        
        mav_array_memcpy(packet1.param_id, packet_in.param_id, sizeof(char)*16);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PARAM_VALUE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PARAM_VALUE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_value_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_param_value_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_value_pack(system_id, component_id, &msg , packet1.param_id , packet1.param_value , packet1.param_type , packet1.param_count , packet1.param_index );
    mavlink_msg_param_value_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_value_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.param_id , packet1.param_value , packet1.param_type , packet1.param_count , packet1.param_index );
    mavlink_msg_param_value_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_param_value_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_value_send(MAVLINK_COMM_1 , packet1.param_id , packet1.param_value , packet1.param_type , packet1.param_count , packet1.param_index );
    mavlink_msg_param_value_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("PARAM_VALUE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_PARAM_VALUE) != NULL);
#endif
}

static void mavlink_test_param_set(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PARAM_SET >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_param_set_t packet_in = {
        17.0,17,84,"GHIJKLMNOPQRSTU",199
    };
    mavlink_param_set_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.param_value = packet_in.param_value;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.param_type = packet_in.param_type;
        
        mav_array_memcpy(packet1.param_id, packet_in.param_id, sizeof(char)*16);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PARAM_SET_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PARAM_SET_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_set_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_param_set_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_set_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.param_id , packet1.param_value , packet1.param_type );
    mavlink_msg_param_set_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_set_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.param_id , packet1.param_value , packet1.param_type );
    mavlink_msg_param_set_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_param_set_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_set_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.param_id , packet1.param_value , packet1.param_type );
    mavlink_msg_param_set_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("PARAM_SET") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_PARAM_SET) != NULL);
#endif
}

static void mavlink_test_gps_raw_int(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GPS_RAW_INT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_gps_raw_int_t packet_in = {
        93372036854775807ULL,963497880,963498088,963498296,18275,18379,18483,18587,89,156,963499024,963499232,963499440,963499648,963499856,19835
    };
    mavlink_gps_raw_int_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        packet1.alt = packet_in.alt;
        packet1.eph = packet_in.eph;
        packet1.epv = packet_in.epv;
        packet1.vel = packet_in.vel;
        packet1.cog = packet_in.cog;
        packet1.fix_type = packet_in.fix_type;
        packet1.satellites_visible = packet_in.satellites_visible;
        packet1.alt_ellipsoid = packet_in.alt_ellipsoid;
        packet1.h_acc = packet_in.h_acc;
        packet1.v_acc = packet_in.v_acc;
        packet1.vel_acc = packet_in.vel_acc;
        packet1.hdg_acc = packet_in.hdg_acc;
        packet1.yaw = packet_in.yaw;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GPS_RAW_INT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GPS_RAW_INT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_raw_int_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_gps_raw_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_raw_int_pack(system_id, component_id, &msg , packet1.time_usec , packet1.fix_type , packet1.lat , packet1.lon , packet1.alt , packet1.eph , packet1.epv , packet1.vel , packet1.cog , packet1.satellites_visible , packet1.alt_ellipsoid , packet1.h_acc , packet1.v_acc , packet1.vel_acc , packet1.hdg_acc , packet1.yaw );
    mavlink_msg_gps_raw_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_raw_int_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.fix_type , packet1.lat , packet1.lon , packet1.alt , packet1.eph , packet1.epv , packet1.vel , packet1.cog , packet1.satellites_visible , packet1.alt_ellipsoid , packet1.h_acc , packet1.v_acc , packet1.vel_acc , packet1.hdg_acc , packet1.yaw );
    mavlink_msg_gps_raw_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_gps_raw_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_raw_int_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.fix_type , packet1.lat , packet1.lon , packet1.alt , packet1.eph , packet1.epv , packet1.vel , packet1.cog , packet1.satellites_visible , packet1.alt_ellipsoid , packet1.h_acc , packet1.v_acc , packet1.vel_acc , packet1.hdg_acc , packet1.yaw );
    mavlink_msg_gps_raw_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("GPS_RAW_INT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_GPS_RAW_INT) != NULL);
#endif
}

static void mavlink_test_gps_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GPS_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_gps_status_t packet_in = {
        5,{ 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91 },{ 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151 },{ 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211 },{ 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 },{ 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75 }
    };
    mavlink_gps_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.satellites_visible = packet_in.satellites_visible;
        
        mav_array_memcpy(packet1.satellite_prn, packet_in.satellite_prn, sizeof(uint8_t)*20);
        mav_array_memcpy(packet1.satellite_used, packet_in.satellite_used, sizeof(uint8_t)*20);
        mav_array_memcpy(packet1.satellite_elevation, packet_in.satellite_elevation, sizeof(uint8_t)*20);
        mav_array_memcpy(packet1.satellite_azimuth, packet_in.satellite_azimuth, sizeof(uint8_t)*20);
        mav_array_memcpy(packet1.satellite_snr, packet_in.satellite_snr, sizeof(uint8_t)*20);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GPS_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GPS_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_gps_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_status_pack(system_id, component_id, &msg , packet1.satellites_visible , packet1.satellite_prn , packet1.satellite_used , packet1.satellite_elevation , packet1.satellite_azimuth , packet1.satellite_snr );
    mavlink_msg_gps_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.satellites_visible , packet1.satellite_prn , packet1.satellite_used , packet1.satellite_elevation , packet1.satellite_azimuth , packet1.satellite_snr );
    mavlink_msg_gps_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_gps_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_status_send(MAVLINK_COMM_1 , packet1.satellites_visible , packet1.satellite_prn , packet1.satellite_used , packet1.satellite_elevation , packet1.satellite_azimuth , packet1.satellite_snr );
    mavlink_msg_gps_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("GPS_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_GPS_STATUS) != NULL);
#endif
}

static void mavlink_test_scaled_imu(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SCALED_IMU >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_scaled_imu_t packet_in = {
        963497464,17443,17547,17651,17755,17859,17963,18067,18171,18275,18379
    };
    mavlink_scaled_imu_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.xacc = packet_in.xacc;
        packet1.yacc = packet_in.yacc;
        packet1.zacc = packet_in.zacc;
        packet1.xgyro = packet_in.xgyro;
        packet1.ygyro = packet_in.ygyro;
        packet1.zgyro = packet_in.zgyro;
        packet1.xmag = packet_in.xmag;
        packet1.ymag = packet_in.ymag;
        packet1.zmag = packet_in.zmag;
        packet1.temperature = packet_in.temperature;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SCALED_IMU_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SCALED_IMU_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_scaled_imu_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_scaled_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_scaled_imu_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xmag , packet1.ymag , packet1.zmag , packet1.temperature );
    mavlink_msg_scaled_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_scaled_imu_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xmag , packet1.ymag , packet1.zmag , packet1.temperature );
    mavlink_msg_scaled_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_scaled_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_scaled_imu_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xmag , packet1.ymag , packet1.zmag , packet1.temperature );
    mavlink_msg_scaled_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SCALED_IMU") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SCALED_IMU) != NULL);
#endif
}

static void mavlink_test_raw_imu(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RAW_IMU >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_raw_imu_t packet_in = {
        93372036854775807ULL,17651,17755,17859,17963,18067,18171,18275,18379,18483,211,18639
    };
    mavlink_raw_imu_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.xacc = packet_in.xacc;
        packet1.yacc = packet_in.yacc;
        packet1.zacc = packet_in.zacc;
        packet1.xgyro = packet_in.xgyro;
        packet1.ygyro = packet_in.ygyro;
        packet1.zgyro = packet_in.zgyro;
        packet1.xmag = packet_in.xmag;
        packet1.ymag = packet_in.ymag;
        packet1.zmag = packet_in.zmag;
        packet1.id = packet_in.id;
        packet1.temperature = packet_in.temperature;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_RAW_IMU_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RAW_IMU_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_raw_imu_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_raw_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_raw_imu_pack(system_id, component_id, &msg , packet1.time_usec , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xmag , packet1.ymag , packet1.zmag , packet1.id , packet1.temperature );
    mavlink_msg_raw_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_raw_imu_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xmag , packet1.ymag , packet1.zmag , packet1.id , packet1.temperature );
    mavlink_msg_raw_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_raw_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_raw_imu_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xmag , packet1.ymag , packet1.zmag , packet1.id , packet1.temperature );
    mavlink_msg_raw_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("RAW_IMU") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_RAW_IMU) != NULL);
#endif
}

static void mavlink_test_raw_pressure(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RAW_PRESSURE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_raw_pressure_t packet_in = {
        93372036854775807ULL,17651,17755,17859,17963
    };
    mavlink_raw_pressure_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.press_abs = packet_in.press_abs;
        packet1.press_diff1 = packet_in.press_diff1;
        packet1.press_diff2 = packet_in.press_diff2;
        packet1.temperature = packet_in.temperature;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_RAW_PRESSURE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RAW_PRESSURE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_raw_pressure_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_raw_pressure_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_raw_pressure_pack(system_id, component_id, &msg , packet1.time_usec , packet1.press_abs , packet1.press_diff1 , packet1.press_diff2 , packet1.temperature );
    mavlink_msg_raw_pressure_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_raw_pressure_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.press_abs , packet1.press_diff1 , packet1.press_diff2 , packet1.temperature );
    mavlink_msg_raw_pressure_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_raw_pressure_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_raw_pressure_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.press_abs , packet1.press_diff1 , packet1.press_diff2 , packet1.temperature );
    mavlink_msg_raw_pressure_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("RAW_PRESSURE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_RAW_PRESSURE) != NULL);
#endif
}

static void mavlink_test_scaled_pressure(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SCALED_PRESSURE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_scaled_pressure_t packet_in = {
        963497464,45.0,73.0,17859,17963
    };
    mavlink_scaled_pressure_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.press_abs = packet_in.press_abs;
        packet1.press_diff = packet_in.press_diff;
        packet1.temperature = packet_in.temperature;
        packet1.temperature_press_diff = packet_in.temperature_press_diff;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SCALED_PRESSURE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SCALED_PRESSURE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_scaled_pressure_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_scaled_pressure_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_scaled_pressure_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.press_abs , packet1.press_diff , packet1.temperature , packet1.temperature_press_diff );
    mavlink_msg_scaled_pressure_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_scaled_pressure_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.press_abs , packet1.press_diff , packet1.temperature , packet1.temperature_press_diff );
    mavlink_msg_scaled_pressure_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_scaled_pressure_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_scaled_pressure_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.press_abs , packet1.press_diff , packet1.temperature , packet1.temperature_press_diff );
    mavlink_msg_scaled_pressure_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SCALED_PRESSURE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SCALED_PRESSURE) != NULL);
#endif
}

static void mavlink_test_attitude(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ATTITUDE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_attitude_t packet_in = {
        963497464,45.0,73.0,101.0,129.0,157.0,185.0
    };
    mavlink_attitude_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.roll = packet_in.roll;
        packet1.pitch = packet_in.pitch;
        packet1.yaw = packet_in.yaw;
        packet1.rollspeed = packet_in.rollspeed;
        packet1.pitchspeed = packet_in.pitchspeed;
        packet1.yawspeed = packet_in.yawspeed;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ATTITUDE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ATTITUDE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_attitude_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_attitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_attitude_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.roll , packet1.pitch , packet1.yaw , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed );
    mavlink_msg_attitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_attitude_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.roll , packet1.pitch , packet1.yaw , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed );
    mavlink_msg_attitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_attitude_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_attitude_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.roll , packet1.pitch , packet1.yaw , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed );
    mavlink_msg_attitude_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ATTITUDE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ATTITUDE) != NULL);
#endif
}

static void mavlink_test_attitude_quaternion(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ATTITUDE_QUATERNION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_attitude_quaternion_t packet_in = {
        963497464,45.0,73.0,101.0,129.0,157.0,185.0,213.0,{ 241.0, 242.0, 243.0, 244.0 }
    };
    mavlink_attitude_quaternion_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.q1 = packet_in.q1;
        packet1.q2 = packet_in.q2;
        packet1.q3 = packet_in.q3;
        packet1.q4 = packet_in.q4;
        packet1.rollspeed = packet_in.rollspeed;
        packet1.pitchspeed = packet_in.pitchspeed;
        packet1.yawspeed = packet_in.yawspeed;
        
        mav_array_memcpy(packet1.repr_offset_q, packet_in.repr_offset_q, sizeof(float)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ATTITUDE_QUATERNION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ATTITUDE_QUATERNION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_attitude_quaternion_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_attitude_quaternion_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_attitude_quaternion_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.q1 , packet1.q2 , packet1.q3 , packet1.q4 , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed , packet1.repr_offset_q );
    mavlink_msg_attitude_quaternion_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_attitude_quaternion_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.q1 , packet1.q2 , packet1.q3 , packet1.q4 , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed , packet1.repr_offset_q );
    mavlink_msg_attitude_quaternion_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_attitude_quaternion_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_attitude_quaternion_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.q1 , packet1.q2 , packet1.q3 , packet1.q4 , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed , packet1.repr_offset_q );
    mavlink_msg_attitude_quaternion_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ATTITUDE_QUATERNION") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ATTITUDE_QUATERNION) != NULL);
#endif
}

static void mavlink_test_local_position_ned(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LOCAL_POSITION_NED >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_local_position_ned_t packet_in = {
        963497464,45.0,73.0,101.0,129.0,157.0,185.0
    };
    mavlink_local_position_ned_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.vx = packet_in.vx;
        packet1.vy = packet_in.vy;
        packet1.vz = packet_in.vz;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LOCAL_POSITION_NED_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LOCAL_POSITION_NED_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_local_position_ned_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_local_position_ned_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_local_position_ned_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.x , packet1.y , packet1.z , packet1.vx , packet1.vy , packet1.vz );
    mavlink_msg_local_position_ned_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_local_position_ned_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.x , packet1.y , packet1.z , packet1.vx , packet1.vy , packet1.vz );
    mavlink_msg_local_position_ned_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_local_position_ned_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_local_position_ned_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.x , packet1.y , packet1.z , packet1.vx , packet1.vy , packet1.vz );
    mavlink_msg_local_position_ned_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("LOCAL_POSITION_NED") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_LOCAL_POSITION_NED) != NULL);
#endif
}

static void mavlink_test_global_position_int(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GLOBAL_POSITION_INT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_global_position_int_t packet_in = {
        963497464,963497672,963497880,963498088,963498296,18275,18379,18483,18587
    };
    mavlink_global_position_int_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        packet1.alt = packet_in.alt;
        packet1.relative_alt = packet_in.relative_alt;
        packet1.vx = packet_in.vx;
        packet1.vy = packet_in.vy;
        packet1.vz = packet_in.vz;
        packet1.hdg = packet_in.hdg;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GLOBAL_POSITION_INT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GLOBAL_POSITION_INT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_global_position_int_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_global_position_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_global_position_int_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.lat , packet1.lon , packet1.alt , packet1.relative_alt , packet1.vx , packet1.vy , packet1.vz , packet1.hdg );
    mavlink_msg_global_position_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_global_position_int_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.lat , packet1.lon , packet1.alt , packet1.relative_alt , packet1.vx , packet1.vy , packet1.vz , packet1.hdg );
    mavlink_msg_global_position_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_global_position_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_global_position_int_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.lat , packet1.lon , packet1.alt , packet1.relative_alt , packet1.vx , packet1.vy , packet1.vz , packet1.hdg );
    mavlink_msg_global_position_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("GLOBAL_POSITION_INT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_GLOBAL_POSITION_INT) != NULL);
#endif
}

static void mavlink_test_rc_channels_scaled(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RC_CHANNELS_SCALED >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_rc_channels_scaled_t packet_in = {
        963497464,17443,17547,17651,17755,17859,17963,18067,18171,65,132
    };
    mavlink_rc_channels_scaled_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.chan1_scaled = packet_in.chan1_scaled;
        packet1.chan2_scaled = packet_in.chan2_scaled;
        packet1.chan3_scaled = packet_in.chan3_scaled;
        packet1.chan4_scaled = packet_in.chan4_scaled;
        packet1.chan5_scaled = packet_in.chan5_scaled;
        packet1.chan6_scaled = packet_in.chan6_scaled;
        packet1.chan7_scaled = packet_in.chan7_scaled;
        packet1.chan8_scaled = packet_in.chan8_scaled;
        packet1.port = packet_in.port;
        packet1.rssi = packet_in.rssi;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_RC_CHANNELS_SCALED_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RC_CHANNELS_SCALED_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rc_channels_scaled_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_rc_channels_scaled_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rc_channels_scaled_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.port , packet1.chan1_scaled , packet1.chan2_scaled , packet1.chan3_scaled , packet1.chan4_scaled , packet1.chan5_scaled , packet1.chan6_scaled , packet1.chan7_scaled , packet1.chan8_scaled , packet1.rssi );
    mavlink_msg_rc_channels_scaled_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rc_channels_scaled_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.port , packet1.chan1_scaled , packet1.chan2_scaled , packet1.chan3_scaled , packet1.chan4_scaled , packet1.chan5_scaled , packet1.chan6_scaled , packet1.chan7_scaled , packet1.chan8_scaled , packet1.rssi );
    mavlink_msg_rc_channels_scaled_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_rc_channels_scaled_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rc_channels_scaled_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.port , packet1.chan1_scaled , packet1.chan2_scaled , packet1.chan3_scaled , packet1.chan4_scaled , packet1.chan5_scaled , packet1.chan6_scaled , packet1.chan7_scaled , packet1.chan8_scaled , packet1.rssi );
    mavlink_msg_rc_channels_scaled_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("RC_CHANNELS_SCALED") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_RC_CHANNELS_SCALED) != NULL);
#endif
}

static void mavlink_test_rc_channels_raw(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RC_CHANNELS_RAW >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_rc_channels_raw_t packet_in = {
        963497464,17443,17547,17651,17755,17859,17963,18067,18171,65,132
    };
    mavlink_rc_channels_raw_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.chan1_raw = packet_in.chan1_raw;
        packet1.chan2_raw = packet_in.chan2_raw;
        packet1.chan3_raw = packet_in.chan3_raw;
        packet1.chan4_raw = packet_in.chan4_raw;
        packet1.chan5_raw = packet_in.chan5_raw;
        packet1.chan6_raw = packet_in.chan6_raw;
        packet1.chan7_raw = packet_in.chan7_raw;
        packet1.chan8_raw = packet_in.chan8_raw;
        packet1.port = packet_in.port;
        packet1.rssi = packet_in.rssi;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_RC_CHANNELS_RAW_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RC_CHANNELS_RAW_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rc_channels_raw_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_rc_channels_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rc_channels_raw_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.port , packet1.chan1_raw , packet1.chan2_raw , packet1.chan3_raw , packet1.chan4_raw , packet1.chan5_raw , packet1.chan6_raw , packet1.chan7_raw , packet1.chan8_raw , packet1.rssi );
    mavlink_msg_rc_channels_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rc_channels_raw_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.port , packet1.chan1_raw , packet1.chan2_raw , packet1.chan3_raw , packet1.chan4_raw , packet1.chan5_raw , packet1.chan6_raw , packet1.chan7_raw , packet1.chan8_raw , packet1.rssi );
    mavlink_msg_rc_channels_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_rc_channels_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rc_channels_raw_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.port , packet1.chan1_raw , packet1.chan2_raw , packet1.chan3_raw , packet1.chan4_raw , packet1.chan5_raw , packet1.chan6_raw , packet1.chan7_raw , packet1.chan8_raw , packet1.rssi );
    mavlink_msg_rc_channels_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("RC_CHANNELS_RAW") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_RC_CHANNELS_RAW) != NULL);
#endif
}

static void mavlink_test_servo_output_raw(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SERVO_OUTPUT_RAW >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_servo_output_raw_t packet_in = {
        963497464,17443,17547,17651,17755,17859,17963,18067,18171,65,18327,18431,18535,18639,18743,18847,18951,19055
    };
    mavlink_servo_output_raw_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.servo1_raw = packet_in.servo1_raw;
        packet1.servo2_raw = packet_in.servo2_raw;
        packet1.servo3_raw = packet_in.servo3_raw;
        packet1.servo4_raw = packet_in.servo4_raw;
        packet1.servo5_raw = packet_in.servo5_raw;
        packet1.servo6_raw = packet_in.servo6_raw;
        packet1.servo7_raw = packet_in.servo7_raw;
        packet1.servo8_raw = packet_in.servo8_raw;
        packet1.port = packet_in.port;
        packet1.servo9_raw = packet_in.servo9_raw;
        packet1.servo10_raw = packet_in.servo10_raw;
        packet1.servo11_raw = packet_in.servo11_raw;
        packet1.servo12_raw = packet_in.servo12_raw;
        packet1.servo13_raw = packet_in.servo13_raw;
        packet1.servo14_raw = packet_in.servo14_raw;
        packet1.servo15_raw = packet_in.servo15_raw;
        packet1.servo16_raw = packet_in.servo16_raw;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_servo_output_raw_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_servo_output_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_servo_output_raw_pack(system_id, component_id, &msg , packet1.time_usec , packet1.port , packet1.servo1_raw , packet1.servo2_raw , packet1.servo3_raw , packet1.servo4_raw , packet1.servo5_raw , packet1.servo6_raw , packet1.servo7_raw , packet1.servo8_raw , packet1.servo9_raw , packet1.servo10_raw , packet1.servo11_raw , packet1.servo12_raw , packet1.servo13_raw , packet1.servo14_raw , packet1.servo15_raw , packet1.servo16_raw );
    mavlink_msg_servo_output_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_servo_output_raw_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.port , packet1.servo1_raw , packet1.servo2_raw , packet1.servo3_raw , packet1.servo4_raw , packet1.servo5_raw , packet1.servo6_raw , packet1.servo7_raw , packet1.servo8_raw , packet1.servo9_raw , packet1.servo10_raw , packet1.servo11_raw , packet1.servo12_raw , packet1.servo13_raw , packet1.servo14_raw , packet1.servo15_raw , packet1.servo16_raw );
    mavlink_msg_servo_output_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_servo_output_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_servo_output_raw_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.port , packet1.servo1_raw , packet1.servo2_raw , packet1.servo3_raw , packet1.servo4_raw , packet1.servo5_raw , packet1.servo6_raw , packet1.servo7_raw , packet1.servo8_raw , packet1.servo9_raw , packet1.servo10_raw , packet1.servo11_raw , packet1.servo12_raw , packet1.servo13_raw , packet1.servo14_raw , packet1.servo15_raw , packet1.servo16_raw );
    mavlink_msg_servo_output_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SERVO_OUTPUT_RAW") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SERVO_OUTPUT_RAW) != NULL);
#endif
}

static void mavlink_test_mission_request_partial_list(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mission_request_partial_list_t packet_in = {
        17235,17339,17,84,151
    };
    mavlink_mission_request_partial_list_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.start_index = packet_in.start_index;
        packet1.end_index = packet_in.end_index;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.mission_type = packet_in.mission_type;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_request_partial_list_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mission_request_partial_list_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_request_partial_list_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.start_index , packet1.end_index , packet1.mission_type );
    mavlink_msg_mission_request_partial_list_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_request_partial_list_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.start_index , packet1.end_index , packet1.mission_type );
    mavlink_msg_mission_request_partial_list_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mission_request_partial_list_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_request_partial_list_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.start_index , packet1.end_index , packet1.mission_type );
    mavlink_msg_mission_request_partial_list_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MISSION_REQUEST_PARTIAL_LIST") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST) != NULL);
#endif
}

static void mavlink_test_mission_write_partial_list(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mission_write_partial_list_t packet_in = {
        17235,17339,17,84,151
    };
    mavlink_mission_write_partial_list_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.start_index = packet_in.start_index;
        packet1.end_index = packet_in.end_index;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.mission_type = packet_in.mission_type;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_write_partial_list_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mission_write_partial_list_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_write_partial_list_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.start_index , packet1.end_index , packet1.mission_type );
    mavlink_msg_mission_write_partial_list_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_write_partial_list_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.start_index , packet1.end_index , packet1.mission_type );
    mavlink_msg_mission_write_partial_list_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mission_write_partial_list_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_write_partial_list_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.start_index , packet1.end_index , packet1.mission_type );
    mavlink_msg_mission_write_partial_list_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MISSION_WRITE_PARTIAL_LIST") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST) != NULL);
#endif
}

static void mavlink_test_mission_item(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MISSION_ITEM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mission_item_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,185.0,18691,18795,101,168,235,46,113,180
    };
    mavlink_mission_item_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.param1 = packet_in.param1;
        packet1.param2 = packet_in.param2;
        packet1.param3 = packet_in.param3;
        packet1.param4 = packet_in.param4;
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.seq = packet_in.seq;
        packet1.command = packet_in.command;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.frame = packet_in.frame;
        packet1.current = packet_in.current;
        packet1.autocontinue = packet_in.autocontinue;
        packet1.mission_type = packet_in.mission_type;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MISSION_ITEM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MISSION_ITEM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_item_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mission_item_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_item_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.seq , packet1.frame , packet1.command , packet1.current , packet1.autocontinue , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.x , packet1.y , packet1.z , packet1.mission_type );
    mavlink_msg_mission_item_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_item_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.seq , packet1.frame , packet1.command , packet1.current , packet1.autocontinue , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.x , packet1.y , packet1.z , packet1.mission_type );
    mavlink_msg_mission_item_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mission_item_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_item_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.seq , packet1.frame , packet1.command , packet1.current , packet1.autocontinue , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.x , packet1.y , packet1.z , packet1.mission_type );
    mavlink_msg_mission_item_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MISSION_ITEM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MISSION_ITEM) != NULL);
#endif
}

static void mavlink_test_mission_request(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MISSION_REQUEST >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mission_request_t packet_in = {
        17235,139,206,17
    };
    mavlink_mission_request_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.seq = packet_in.seq;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.mission_type = packet_in.mission_type;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MISSION_REQUEST_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MISSION_REQUEST_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_request_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mission_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_request_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.seq , packet1.mission_type );
    mavlink_msg_mission_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_request_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.seq , packet1.mission_type );
    mavlink_msg_mission_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mission_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_request_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.seq , packet1.mission_type );
    mavlink_msg_mission_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MISSION_REQUEST") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MISSION_REQUEST) != NULL);
#endif
}

static void mavlink_test_mission_set_current(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MISSION_SET_CURRENT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mission_set_current_t packet_in = {
        17235,139,206
    };
    mavlink_mission_set_current_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.seq = packet_in.seq;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MISSION_SET_CURRENT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MISSION_SET_CURRENT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_set_current_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mission_set_current_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_set_current_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.seq );
    mavlink_msg_mission_set_current_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_set_current_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.seq );
    mavlink_msg_mission_set_current_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mission_set_current_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_set_current_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.seq );
    mavlink_msg_mission_set_current_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MISSION_SET_CURRENT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MISSION_SET_CURRENT) != NULL);
#endif
}

static void mavlink_test_mission_current(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MISSION_CURRENT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mission_current_t packet_in = {
        17235,17339,17,84
    };
    mavlink_mission_current_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.seq = packet_in.seq;
        packet1.total = packet_in.total;
        packet1.mission_state = packet_in.mission_state;
        packet1.mission_mode = packet_in.mission_mode;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MISSION_CURRENT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MISSION_CURRENT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_current_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mission_current_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_current_pack(system_id, component_id, &msg , packet1.seq , packet1.total , packet1.mission_state , packet1.mission_mode );
    mavlink_msg_mission_current_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_current_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.seq , packet1.total , packet1.mission_state , packet1.mission_mode );
    mavlink_msg_mission_current_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mission_current_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_current_send(MAVLINK_COMM_1 , packet1.seq , packet1.total , packet1.mission_state , packet1.mission_mode );
    mavlink_msg_mission_current_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MISSION_CURRENT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MISSION_CURRENT) != NULL);
#endif
}

static void mavlink_test_mission_request_list(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MISSION_REQUEST_LIST >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mission_request_list_t packet_in = {
        5,72,139
    };
    mavlink_mission_request_list_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.mission_type = packet_in.mission_type;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MISSION_REQUEST_LIST_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MISSION_REQUEST_LIST_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_request_list_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mission_request_list_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_request_list_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.mission_type );
    mavlink_msg_mission_request_list_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_request_list_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.mission_type );
    mavlink_msg_mission_request_list_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mission_request_list_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_request_list_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.mission_type );
    mavlink_msg_mission_request_list_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MISSION_REQUEST_LIST") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MISSION_REQUEST_LIST) != NULL);
#endif
}

static void mavlink_test_mission_count(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MISSION_COUNT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mission_count_t packet_in = {
        17235,139,206,17
    };
    mavlink_mission_count_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.count = packet_in.count;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.mission_type = packet_in.mission_type;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MISSION_COUNT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MISSION_COUNT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_count_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mission_count_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_count_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.count , packet1.mission_type );
    mavlink_msg_mission_count_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_count_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.count , packet1.mission_type );
    mavlink_msg_mission_count_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mission_count_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_count_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.count , packet1.mission_type );
    mavlink_msg_mission_count_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MISSION_COUNT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MISSION_COUNT) != NULL);
#endif
}

static void mavlink_test_mission_clear_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MISSION_CLEAR_ALL >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mission_clear_all_t packet_in = {
        5,72,139
    };
    mavlink_mission_clear_all_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.mission_type = packet_in.mission_type;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MISSION_CLEAR_ALL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MISSION_CLEAR_ALL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_clear_all_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mission_clear_all_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_clear_all_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.mission_type );
    mavlink_msg_mission_clear_all_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_clear_all_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.mission_type );
    mavlink_msg_mission_clear_all_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mission_clear_all_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_clear_all_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.mission_type );
    mavlink_msg_mission_clear_all_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MISSION_CLEAR_ALL") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MISSION_CLEAR_ALL) != NULL);
#endif
}

static void mavlink_test_mission_item_reached(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MISSION_ITEM_REACHED >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mission_item_reached_t packet_in = {
        17235
    };
    mavlink_mission_item_reached_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.seq = packet_in.seq;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MISSION_ITEM_REACHED_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MISSION_ITEM_REACHED_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_item_reached_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mission_item_reached_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_item_reached_pack(system_id, component_id, &msg , packet1.seq );
    mavlink_msg_mission_item_reached_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_item_reached_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.seq );
    mavlink_msg_mission_item_reached_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mission_item_reached_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_item_reached_send(MAVLINK_COMM_1 , packet1.seq );
    mavlink_msg_mission_item_reached_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MISSION_ITEM_REACHED") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MISSION_ITEM_REACHED) != NULL);
#endif
}

static void mavlink_test_mission_ack(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MISSION_ACK >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mission_ack_t packet_in = {
        5,72,139,206
    };
    mavlink_mission_ack_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.type = packet_in.type;
        packet1.mission_type = packet_in.mission_type;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MISSION_ACK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MISSION_ACK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_ack_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mission_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_ack_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.type , packet1.mission_type );
    mavlink_msg_mission_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_ack_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.type , packet1.mission_type );
    mavlink_msg_mission_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mission_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_ack_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.type , packet1.mission_type );
    mavlink_msg_mission_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MISSION_ACK") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MISSION_ACK) != NULL);
#endif
}

static void mavlink_test_set_gps_global_origin(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_gps_global_origin_t packet_in = {
        963497464,963497672,963497880,41,93372036854776626ULL
    };
    mavlink_set_gps_global_origin_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.latitude = packet_in.latitude;
        packet1.longitude = packet_in.longitude;
        packet1.altitude = packet_in.altitude;
        packet1.target_system = packet_in.target_system;
        packet1.time_usec = packet_in.time_usec;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_gps_global_origin_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_gps_global_origin_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_gps_global_origin_pack(system_id, component_id, &msg , packet1.target_system , packet1.latitude , packet1.longitude , packet1.altitude , packet1.time_usec );
    mavlink_msg_set_gps_global_origin_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_gps_global_origin_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.latitude , packet1.longitude , packet1.altitude , packet1.time_usec );
    mavlink_msg_set_gps_global_origin_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_gps_global_origin_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_gps_global_origin_send(MAVLINK_COMM_1 , packet1.target_system , packet1.latitude , packet1.longitude , packet1.altitude , packet1.time_usec );
    mavlink_msg_set_gps_global_origin_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SET_GPS_GLOBAL_ORIGIN") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN) != NULL);
#endif
}

static void mavlink_test_gps_global_origin(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_gps_global_origin_t packet_in = {
        963497464,963497672,963497880,93372036854776563ULL
    };
    mavlink_gps_global_origin_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.latitude = packet_in.latitude;
        packet1.longitude = packet_in.longitude;
        packet1.altitude = packet_in.altitude;
        packet1.time_usec = packet_in.time_usec;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_global_origin_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_gps_global_origin_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_global_origin_pack(system_id, component_id, &msg , packet1.latitude , packet1.longitude , packet1.altitude , packet1.time_usec );
    mavlink_msg_gps_global_origin_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_global_origin_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.latitude , packet1.longitude , packet1.altitude , packet1.time_usec );
    mavlink_msg_gps_global_origin_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_gps_global_origin_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_global_origin_send(MAVLINK_COMM_1 , packet1.latitude , packet1.longitude , packet1.altitude , packet1.time_usec );
    mavlink_msg_gps_global_origin_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("GPS_GLOBAL_ORIGIN") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN) != NULL);
#endif
}

static void mavlink_test_param_map_rc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PARAM_MAP_RC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_param_map_rc_t packet_in = {
        17.0,45.0,73.0,101.0,18067,187,254,"UVWXYZABCDEFGHI",113
    };
    mavlink_param_map_rc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.param_value0 = packet_in.param_value0;
        packet1.scale = packet_in.scale;
        packet1.param_value_min = packet_in.param_value_min;
        packet1.param_value_max = packet_in.param_value_max;
        packet1.param_index = packet_in.param_index;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.parameter_rc_channel_index = packet_in.parameter_rc_channel_index;
        
        mav_array_memcpy(packet1.param_id, packet_in.param_id, sizeof(char)*16);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PARAM_MAP_RC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PARAM_MAP_RC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_map_rc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_param_map_rc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_map_rc_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.param_id , packet1.param_index , packet1.parameter_rc_channel_index , packet1.param_value0 , packet1.scale , packet1.param_value_min , packet1.param_value_max );
    mavlink_msg_param_map_rc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_map_rc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.param_id , packet1.param_index , packet1.parameter_rc_channel_index , packet1.param_value0 , packet1.scale , packet1.param_value_min , packet1.param_value_max );
    mavlink_msg_param_map_rc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_param_map_rc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_map_rc_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.param_id , packet1.param_index , packet1.parameter_rc_channel_index , packet1.param_value0 , packet1.scale , packet1.param_value_min , packet1.param_value_max );
    mavlink_msg_param_map_rc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("PARAM_MAP_RC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_PARAM_MAP_RC) != NULL);
#endif
}

static void mavlink_test_mission_request_int(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MISSION_REQUEST_INT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mission_request_int_t packet_in = {
        17235,139,206,17
    };
    mavlink_mission_request_int_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.seq = packet_in.seq;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.mission_type = packet_in.mission_type;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MISSION_REQUEST_INT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MISSION_REQUEST_INT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_request_int_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mission_request_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_request_int_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.seq , packet1.mission_type );
    mavlink_msg_mission_request_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_request_int_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.seq , packet1.mission_type );
    mavlink_msg_mission_request_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mission_request_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_request_int_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.seq , packet1.mission_type );
    mavlink_msg_mission_request_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MISSION_REQUEST_INT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MISSION_REQUEST_INT) != NULL);
#endif
}

static void mavlink_test_safety_set_allowed_area(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_safety_set_allowed_area_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,77,144,211
    };
    mavlink_safety_set_allowed_area_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.p1x = packet_in.p1x;
        packet1.p1y = packet_in.p1y;
        packet1.p1z = packet_in.p1z;
        packet1.p2x = packet_in.p2x;
        packet1.p2y = packet_in.p2y;
        packet1.p2z = packet_in.p2z;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.frame = packet_in.frame;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_safety_set_allowed_area_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_safety_set_allowed_area_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_safety_set_allowed_area_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.frame , packet1.p1x , packet1.p1y , packet1.p1z , packet1.p2x , packet1.p2y , packet1.p2z );
    mavlink_msg_safety_set_allowed_area_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_safety_set_allowed_area_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.frame , packet1.p1x , packet1.p1y , packet1.p1z , packet1.p2x , packet1.p2y , packet1.p2z );
    mavlink_msg_safety_set_allowed_area_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_safety_set_allowed_area_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_safety_set_allowed_area_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.frame , packet1.p1x , packet1.p1y , packet1.p1z , packet1.p2x , packet1.p2y , packet1.p2z );
    mavlink_msg_safety_set_allowed_area_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SAFETY_SET_ALLOWED_AREA") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA) != NULL);
#endif
}

static void mavlink_test_safety_allowed_area(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SAFETY_ALLOWED_AREA >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_safety_allowed_area_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,77
    };
    mavlink_safety_allowed_area_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.p1x = packet_in.p1x;
        packet1.p1y = packet_in.p1y;
        packet1.p1z = packet_in.p1z;
        packet1.p2x = packet_in.p2x;
        packet1.p2y = packet_in.p2y;
        packet1.p2z = packet_in.p2z;
        packet1.frame = packet_in.frame;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SAFETY_ALLOWED_AREA_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SAFETY_ALLOWED_AREA_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_safety_allowed_area_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_safety_allowed_area_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_safety_allowed_area_pack(system_id, component_id, &msg , packet1.frame , packet1.p1x , packet1.p1y , packet1.p1z , packet1.p2x , packet1.p2y , packet1.p2z );
    mavlink_msg_safety_allowed_area_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_safety_allowed_area_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.frame , packet1.p1x , packet1.p1y , packet1.p1z , packet1.p2x , packet1.p2y , packet1.p2z );
    mavlink_msg_safety_allowed_area_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_safety_allowed_area_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_safety_allowed_area_send(MAVLINK_COMM_1 , packet1.frame , packet1.p1x , packet1.p1y , packet1.p1z , packet1.p2x , packet1.p2y , packet1.p2z );
    mavlink_msg_safety_allowed_area_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SAFETY_ALLOWED_AREA") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SAFETY_ALLOWED_AREA) != NULL);
#endif
}

static void mavlink_test_attitude_quaternion_cov(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_attitude_quaternion_cov_t packet_in = {
        93372036854775807ULL,{ 73.0, 74.0, 75.0, 76.0 },185.0,213.0,241.0,{ 269.0, 270.0, 271.0, 272.0, 273.0, 274.0, 275.0, 276.0, 277.0 }
    };
    mavlink_attitude_quaternion_cov_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.rollspeed = packet_in.rollspeed;
        packet1.pitchspeed = packet_in.pitchspeed;
        packet1.yawspeed = packet_in.yawspeed;
        
        mav_array_memcpy(packet1.q, packet_in.q, sizeof(float)*4);
        mav_array_memcpy(packet1.covariance, packet_in.covariance, sizeof(float)*9);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_attitude_quaternion_cov_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_attitude_quaternion_cov_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_attitude_quaternion_cov_pack(system_id, component_id, &msg , packet1.time_usec , packet1.q , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed , packet1.covariance );
    mavlink_msg_attitude_quaternion_cov_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_attitude_quaternion_cov_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.q , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed , packet1.covariance );
    mavlink_msg_attitude_quaternion_cov_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_attitude_quaternion_cov_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_attitude_quaternion_cov_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.q , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed , packet1.covariance );
    mavlink_msg_attitude_quaternion_cov_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ATTITUDE_QUATERNION_COV") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV) != NULL);
#endif
}

static void mavlink_test_nav_controller_output(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_nav_controller_output_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,18275,18379,18483
    };
    mavlink_nav_controller_output_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.nav_roll = packet_in.nav_roll;
        packet1.nav_pitch = packet_in.nav_pitch;
        packet1.alt_error = packet_in.alt_error;
        packet1.aspd_error = packet_in.aspd_error;
        packet1.xtrack_error = packet_in.xtrack_error;
        packet1.nav_bearing = packet_in.nav_bearing;
        packet1.target_bearing = packet_in.target_bearing;
        packet1.wp_dist = packet_in.wp_dist;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_nav_controller_output_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_nav_controller_output_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_nav_controller_output_pack(system_id, component_id, &msg , packet1.nav_roll , packet1.nav_pitch , packet1.nav_bearing , packet1.target_bearing , packet1.wp_dist , packet1.alt_error , packet1.aspd_error , packet1.xtrack_error );
    mavlink_msg_nav_controller_output_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_nav_controller_output_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.nav_roll , packet1.nav_pitch , packet1.nav_bearing , packet1.target_bearing , packet1.wp_dist , packet1.alt_error , packet1.aspd_error , packet1.xtrack_error );
    mavlink_msg_nav_controller_output_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_nav_controller_output_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_nav_controller_output_send(MAVLINK_COMM_1 , packet1.nav_roll , packet1.nav_pitch , packet1.nav_bearing , packet1.target_bearing , packet1.wp_dist , packet1.alt_error , packet1.aspd_error , packet1.xtrack_error );
    mavlink_msg_nav_controller_output_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("NAV_CONTROLLER_OUTPUT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT) != NULL);
#endif
}

static void mavlink_test_global_position_int_cov(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_global_position_int_cov_t packet_in = {
        93372036854775807ULL,963497880,963498088,963498296,963498504,185.0,213.0,241.0,{ 269.0, 270.0, 271.0, 272.0, 273.0, 274.0, 275.0, 276.0, 277.0, 278.0, 279.0, 280.0, 281.0, 282.0, 283.0, 284.0, 285.0, 286.0, 287.0, 288.0, 289.0, 290.0, 291.0, 292.0, 293.0, 294.0, 295.0, 296.0, 297.0, 298.0, 299.0, 300.0, 301.0, 302.0, 303.0, 304.0 },33
    };
    mavlink_global_position_int_cov_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        packet1.alt = packet_in.alt;
        packet1.relative_alt = packet_in.relative_alt;
        packet1.vx = packet_in.vx;
        packet1.vy = packet_in.vy;
        packet1.vz = packet_in.vz;
        packet1.estimator_type = packet_in.estimator_type;
        
        mav_array_memcpy(packet1.covariance, packet_in.covariance, sizeof(float)*36);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_global_position_int_cov_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_global_position_int_cov_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_global_position_int_cov_pack(system_id, component_id, &msg , packet1.time_usec , packet1.estimator_type , packet1.lat , packet1.lon , packet1.alt , packet1.relative_alt , packet1.vx , packet1.vy , packet1.vz , packet1.covariance );
    mavlink_msg_global_position_int_cov_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_global_position_int_cov_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.estimator_type , packet1.lat , packet1.lon , packet1.alt , packet1.relative_alt , packet1.vx , packet1.vy , packet1.vz , packet1.covariance );
    mavlink_msg_global_position_int_cov_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_global_position_int_cov_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_global_position_int_cov_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.estimator_type , packet1.lat , packet1.lon , packet1.alt , packet1.relative_alt , packet1.vx , packet1.vy , packet1.vz , packet1.covariance );
    mavlink_msg_global_position_int_cov_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("GLOBAL_POSITION_INT_COV") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV) != NULL);
#endif
}

static void mavlink_test_local_position_ned_cov(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_local_position_ned_cov_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,{ 325.0, 326.0, 327.0, 328.0, 329.0, 330.0, 331.0, 332.0, 333.0, 334.0, 335.0, 336.0, 337.0, 338.0, 339.0, 340.0, 341.0, 342.0, 343.0, 344.0, 345.0, 346.0, 347.0, 348.0, 349.0, 350.0, 351.0, 352.0, 353.0, 354.0, 355.0, 356.0, 357.0, 358.0, 359.0, 360.0, 361.0, 362.0, 363.0, 364.0, 365.0, 366.0, 367.0, 368.0, 369.0 },165
    };
    mavlink_local_position_ned_cov_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.vx = packet_in.vx;
        packet1.vy = packet_in.vy;
        packet1.vz = packet_in.vz;
        packet1.ax = packet_in.ax;
        packet1.ay = packet_in.ay;
        packet1.az = packet_in.az;
        packet1.estimator_type = packet_in.estimator_type;
        
        mav_array_memcpy(packet1.covariance, packet_in.covariance, sizeof(float)*45);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_local_position_ned_cov_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_local_position_ned_cov_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_local_position_ned_cov_pack(system_id, component_id, &msg , packet1.time_usec , packet1.estimator_type , packet1.x , packet1.y , packet1.z , packet1.vx , packet1.vy , packet1.vz , packet1.ax , packet1.ay , packet1.az , packet1.covariance );
    mavlink_msg_local_position_ned_cov_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_local_position_ned_cov_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.estimator_type , packet1.x , packet1.y , packet1.z , packet1.vx , packet1.vy , packet1.vz , packet1.ax , packet1.ay , packet1.az , packet1.covariance );
    mavlink_msg_local_position_ned_cov_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_local_position_ned_cov_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_local_position_ned_cov_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.estimator_type , packet1.x , packet1.y , packet1.z , packet1.vx , packet1.vy , packet1.vz , packet1.ax , packet1.ay , packet1.az , packet1.covariance );
    mavlink_msg_local_position_ned_cov_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("LOCAL_POSITION_NED_COV") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV) != NULL);
#endif
}

static void mavlink_test_rc_channels(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RC_CHANNELS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_rc_channels_t packet_in = {
        963497464,17443,17547,17651,17755,17859,17963,18067,18171,18275,18379,18483,18587,18691,18795,18899,19003,19107,19211,125,192
    };
    mavlink_rc_channels_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.chan1_raw = packet_in.chan1_raw;
        packet1.chan2_raw = packet_in.chan2_raw;
        packet1.chan3_raw = packet_in.chan3_raw;
        packet1.chan4_raw = packet_in.chan4_raw;
        packet1.chan5_raw = packet_in.chan5_raw;
        packet1.chan6_raw = packet_in.chan6_raw;
        packet1.chan7_raw = packet_in.chan7_raw;
        packet1.chan8_raw = packet_in.chan8_raw;
        packet1.chan9_raw = packet_in.chan9_raw;
        packet1.chan10_raw = packet_in.chan10_raw;
        packet1.chan11_raw = packet_in.chan11_raw;
        packet1.chan12_raw = packet_in.chan12_raw;
        packet1.chan13_raw = packet_in.chan13_raw;
        packet1.chan14_raw = packet_in.chan14_raw;
        packet1.chan15_raw = packet_in.chan15_raw;
        packet1.chan16_raw = packet_in.chan16_raw;
        packet1.chan17_raw = packet_in.chan17_raw;
        packet1.chan18_raw = packet_in.chan18_raw;
        packet1.chancount = packet_in.chancount;
        packet1.rssi = packet_in.rssi;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_RC_CHANNELS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RC_CHANNELS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rc_channels_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_rc_channels_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rc_channels_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.chancount , packet1.chan1_raw , packet1.chan2_raw , packet1.chan3_raw , packet1.chan4_raw , packet1.chan5_raw , packet1.chan6_raw , packet1.chan7_raw , packet1.chan8_raw , packet1.chan9_raw , packet1.chan10_raw , packet1.chan11_raw , packet1.chan12_raw , packet1.chan13_raw , packet1.chan14_raw , packet1.chan15_raw , packet1.chan16_raw , packet1.chan17_raw , packet1.chan18_raw , packet1.rssi );
    mavlink_msg_rc_channels_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rc_channels_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.chancount , packet1.chan1_raw , packet1.chan2_raw , packet1.chan3_raw , packet1.chan4_raw , packet1.chan5_raw , packet1.chan6_raw , packet1.chan7_raw , packet1.chan8_raw , packet1.chan9_raw , packet1.chan10_raw , packet1.chan11_raw , packet1.chan12_raw , packet1.chan13_raw , packet1.chan14_raw , packet1.chan15_raw , packet1.chan16_raw , packet1.chan17_raw , packet1.chan18_raw , packet1.rssi );
    mavlink_msg_rc_channels_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_rc_channels_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rc_channels_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.chancount , packet1.chan1_raw , packet1.chan2_raw , packet1.chan3_raw , packet1.chan4_raw , packet1.chan5_raw , packet1.chan6_raw , packet1.chan7_raw , packet1.chan8_raw , packet1.chan9_raw , packet1.chan10_raw , packet1.chan11_raw , packet1.chan12_raw , packet1.chan13_raw , packet1.chan14_raw , packet1.chan15_raw , packet1.chan16_raw , packet1.chan17_raw , packet1.chan18_raw , packet1.rssi );
    mavlink_msg_rc_channels_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("RC_CHANNELS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_RC_CHANNELS) != NULL);
#endif
}

static void mavlink_test_request_data_stream(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_REQUEST_DATA_STREAM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_request_data_stream_t packet_in = {
        17235,139,206,17,84
    };
    mavlink_request_data_stream_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.req_message_rate = packet_in.req_message_rate;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.req_stream_id = packet_in.req_stream_id;
        packet1.start_stop = packet_in.start_stop;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_REQUEST_DATA_STREAM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_REQUEST_DATA_STREAM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_request_data_stream_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_request_data_stream_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_request_data_stream_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.req_stream_id , packet1.req_message_rate , packet1.start_stop );
    mavlink_msg_request_data_stream_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_request_data_stream_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.req_stream_id , packet1.req_message_rate , packet1.start_stop );
    mavlink_msg_request_data_stream_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_request_data_stream_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_request_data_stream_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.req_stream_id , packet1.req_message_rate , packet1.start_stop );
    mavlink_msg_request_data_stream_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("REQUEST_DATA_STREAM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_REQUEST_DATA_STREAM) != NULL);
#endif
}

static void mavlink_test_data_stream(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DATA_STREAM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_data_stream_t packet_in = {
        17235,139,206
    };
    mavlink_data_stream_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.message_rate = packet_in.message_rate;
        packet1.stream_id = packet_in.stream_id;
        packet1.on_off = packet_in.on_off;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DATA_STREAM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DATA_STREAM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_data_stream_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_data_stream_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_data_stream_pack(system_id, component_id, &msg , packet1.stream_id , packet1.message_rate , packet1.on_off );
    mavlink_msg_data_stream_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_data_stream_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.stream_id , packet1.message_rate , packet1.on_off );
    mavlink_msg_data_stream_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_data_stream_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_data_stream_send(MAVLINK_COMM_1 , packet1.stream_id , packet1.message_rate , packet1.on_off );
    mavlink_msg_data_stream_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("DATA_STREAM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_DATA_STREAM) != NULL);
#endif
}

static void mavlink_test_manual_control(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MANUAL_CONTROL >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_manual_control_t packet_in = {
        17235,17339,17443,17547,17651,163,17807,108,17963,18067,18171,18275,18379,18483,18587,18691
    };
    mavlink_manual_control_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.r = packet_in.r;
        packet1.buttons = packet_in.buttons;
        packet1.target = packet_in.target;
        packet1.buttons2 = packet_in.buttons2;
        packet1.enabled_extensions = packet_in.enabled_extensions;
        packet1.s = packet_in.s;
        packet1.t = packet_in.t;
        packet1.aux1 = packet_in.aux1;
        packet1.aux2 = packet_in.aux2;
        packet1.aux3 = packet_in.aux3;
        packet1.aux4 = packet_in.aux4;
        packet1.aux5 = packet_in.aux5;
        packet1.aux6 = packet_in.aux6;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MANUAL_CONTROL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MANUAL_CONTROL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_manual_control_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_manual_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_manual_control_pack(system_id, component_id, &msg , packet1.target , packet1.x , packet1.y , packet1.z , packet1.r , packet1.buttons , packet1.buttons2 , packet1.enabled_extensions , packet1.s , packet1.t , packet1.aux1 , packet1.aux2 , packet1.aux3 , packet1.aux4 , packet1.aux5 , packet1.aux6 );
    mavlink_msg_manual_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_manual_control_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target , packet1.x , packet1.y , packet1.z , packet1.r , packet1.buttons , packet1.buttons2 , packet1.enabled_extensions , packet1.s , packet1.t , packet1.aux1 , packet1.aux2 , packet1.aux3 , packet1.aux4 , packet1.aux5 , packet1.aux6 );
    mavlink_msg_manual_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_manual_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_manual_control_send(MAVLINK_COMM_1 , packet1.target , packet1.x , packet1.y , packet1.z , packet1.r , packet1.buttons , packet1.buttons2 , packet1.enabled_extensions , packet1.s , packet1.t , packet1.aux1 , packet1.aux2 , packet1.aux3 , packet1.aux4 , packet1.aux5 , packet1.aux6 );
    mavlink_msg_manual_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MANUAL_CONTROL") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MANUAL_CONTROL) != NULL);
#endif
}

static void mavlink_test_rc_channels_override(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_rc_channels_override_t packet_in = {
        17235,17339,17443,17547,17651,17755,17859,17963,53,120,18171,18275,18379,18483,18587,18691,18795,18899,19003,19107
    };
    mavlink_rc_channels_override_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.chan1_raw = packet_in.chan1_raw;
        packet1.chan2_raw = packet_in.chan2_raw;
        packet1.chan3_raw = packet_in.chan3_raw;
        packet1.chan4_raw = packet_in.chan4_raw;
        packet1.chan5_raw = packet_in.chan5_raw;
        packet1.chan6_raw = packet_in.chan6_raw;
        packet1.chan7_raw = packet_in.chan7_raw;
        packet1.chan8_raw = packet_in.chan8_raw;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.chan9_raw = packet_in.chan9_raw;
        packet1.chan10_raw = packet_in.chan10_raw;
        packet1.chan11_raw = packet_in.chan11_raw;
        packet1.chan12_raw = packet_in.chan12_raw;
        packet1.chan13_raw = packet_in.chan13_raw;
        packet1.chan14_raw = packet_in.chan14_raw;
        packet1.chan15_raw = packet_in.chan15_raw;
        packet1.chan16_raw = packet_in.chan16_raw;
        packet1.chan17_raw = packet_in.chan17_raw;
        packet1.chan18_raw = packet_in.chan18_raw;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rc_channels_override_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_rc_channels_override_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rc_channels_override_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.chan1_raw , packet1.chan2_raw , packet1.chan3_raw , packet1.chan4_raw , packet1.chan5_raw , packet1.chan6_raw , packet1.chan7_raw , packet1.chan8_raw , packet1.chan9_raw , packet1.chan10_raw , packet1.chan11_raw , packet1.chan12_raw , packet1.chan13_raw , packet1.chan14_raw , packet1.chan15_raw , packet1.chan16_raw , packet1.chan17_raw , packet1.chan18_raw );
    mavlink_msg_rc_channels_override_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rc_channels_override_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.chan1_raw , packet1.chan2_raw , packet1.chan3_raw , packet1.chan4_raw , packet1.chan5_raw , packet1.chan6_raw , packet1.chan7_raw , packet1.chan8_raw , packet1.chan9_raw , packet1.chan10_raw , packet1.chan11_raw , packet1.chan12_raw , packet1.chan13_raw , packet1.chan14_raw , packet1.chan15_raw , packet1.chan16_raw , packet1.chan17_raw , packet1.chan18_raw );
    mavlink_msg_rc_channels_override_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_rc_channels_override_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rc_channels_override_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.chan1_raw , packet1.chan2_raw , packet1.chan3_raw , packet1.chan4_raw , packet1.chan5_raw , packet1.chan6_raw , packet1.chan7_raw , packet1.chan8_raw , packet1.chan9_raw , packet1.chan10_raw , packet1.chan11_raw , packet1.chan12_raw , packet1.chan13_raw , packet1.chan14_raw , packet1.chan15_raw , packet1.chan16_raw , packet1.chan17_raw , packet1.chan18_raw );
    mavlink_msg_rc_channels_override_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("RC_CHANNELS_OVERRIDE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE) != NULL);
#endif
}

static void mavlink_test_mission_item_int(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MISSION_ITEM_INT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mission_item_int_t packet_in = {
        17.0,45.0,73.0,101.0,963498296,963498504,185.0,18691,18795,101,168,235,46,113,180
    };
    mavlink_mission_item_int_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.param1 = packet_in.param1;
        packet1.param2 = packet_in.param2;
        packet1.param3 = packet_in.param3;
        packet1.param4 = packet_in.param4;
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.seq = packet_in.seq;
        packet1.command = packet_in.command;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.frame = packet_in.frame;
        packet1.current = packet_in.current;
        packet1.autocontinue = packet_in.autocontinue;
        packet1.mission_type = packet_in.mission_type;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MISSION_ITEM_INT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MISSION_ITEM_INT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_item_int_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mission_item_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_item_int_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.seq , packet1.frame , packet1.command , packet1.current , packet1.autocontinue , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.x , packet1.y , packet1.z , packet1.mission_type );
    mavlink_msg_mission_item_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_item_int_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.seq , packet1.frame , packet1.command , packet1.current , packet1.autocontinue , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.x , packet1.y , packet1.z , packet1.mission_type );
    mavlink_msg_mission_item_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mission_item_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_item_int_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.seq , packet1.frame , packet1.command , packet1.current , packet1.autocontinue , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.x , packet1.y , packet1.z , packet1.mission_type );
    mavlink_msg_mission_item_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MISSION_ITEM_INT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MISSION_ITEM_INT) != NULL);
#endif
}

static void mavlink_test_vfr_hud(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_VFR_HUD >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_vfr_hud_t packet_in = {
        17.0,45.0,73.0,101.0,18067,18171
    };
    mavlink_vfr_hud_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.airspeed = packet_in.airspeed;
        packet1.groundspeed = packet_in.groundspeed;
        packet1.alt = packet_in.alt;
        packet1.climb = packet_in.climb;
        packet1.heading = packet_in.heading;
        packet1.throttle = packet_in.throttle;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_VFR_HUD_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_VFR_HUD_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_vfr_hud_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_vfr_hud_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_vfr_hud_pack(system_id, component_id, &msg , packet1.airspeed , packet1.groundspeed , packet1.heading , packet1.throttle , packet1.alt , packet1.climb );
    mavlink_msg_vfr_hud_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_vfr_hud_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.airspeed , packet1.groundspeed , packet1.heading , packet1.throttle , packet1.alt , packet1.climb );
    mavlink_msg_vfr_hud_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_vfr_hud_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_vfr_hud_send(MAVLINK_COMM_1 , packet1.airspeed , packet1.groundspeed , packet1.heading , packet1.throttle , packet1.alt , packet1.climb );
    mavlink_msg_vfr_hud_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("VFR_HUD") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_VFR_HUD) != NULL);
#endif
}

static void mavlink_test_command_int(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_COMMAND_INT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_command_int_t packet_in = {
        17.0,45.0,73.0,101.0,963498296,963498504,185.0,18691,223,34,101,168,235
    };
    mavlink_command_int_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.param1 = packet_in.param1;
        packet1.param2 = packet_in.param2;
        packet1.param3 = packet_in.param3;
        packet1.param4 = packet_in.param4;
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.command = packet_in.command;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.frame = packet_in.frame;
        packet1.current = packet_in.current;
        packet1.autocontinue = packet_in.autocontinue;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_COMMAND_INT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_COMMAND_INT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_int_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_command_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_int_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.frame , packet1.command , packet1.current , packet1.autocontinue , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.x , packet1.y , packet1.z );
    mavlink_msg_command_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_int_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.frame , packet1.command , packet1.current , packet1.autocontinue , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.x , packet1.y , packet1.z );
    mavlink_msg_command_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_command_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_int_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.frame , packet1.command , packet1.current , packet1.autocontinue , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.x , packet1.y , packet1.z );
    mavlink_msg_command_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("COMMAND_INT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_COMMAND_INT) != NULL);
#endif
}

static void mavlink_test_command_long(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_COMMAND_LONG >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_command_long_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,185.0,18691,223,34,101
    };
    mavlink_command_long_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.param1 = packet_in.param1;
        packet1.param2 = packet_in.param2;
        packet1.param3 = packet_in.param3;
        packet1.param4 = packet_in.param4;
        packet1.param5 = packet_in.param5;
        packet1.param6 = packet_in.param6;
        packet1.param7 = packet_in.param7;
        packet1.command = packet_in.command;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.confirmation = packet_in.confirmation;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_COMMAND_LONG_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_COMMAND_LONG_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_long_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_command_long_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_long_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.command , packet1.confirmation , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.param5 , packet1.param6 , packet1.param7 );
    mavlink_msg_command_long_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_long_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.command , packet1.confirmation , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.param5 , packet1.param6 , packet1.param7 );
    mavlink_msg_command_long_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_command_long_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_long_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.command , packet1.confirmation , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.param5 , packet1.param6 , packet1.param7 );
    mavlink_msg_command_long_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("COMMAND_LONG") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_COMMAND_LONG) != NULL);
#endif
}

static void mavlink_test_command_ack(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_COMMAND_ACK >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_command_ack_t packet_in = {
        17235,139,206,963497672,29,96
    };
    mavlink_command_ack_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.command = packet_in.command;
        packet1.result = packet_in.result;
        packet1.progress = packet_in.progress;
        packet1.result_param2 = packet_in.result_param2;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_COMMAND_ACK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_COMMAND_ACK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_ack_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_command_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_ack_pack(system_id, component_id, &msg , packet1.command , packet1.result , packet1.progress , packet1.result_param2 , packet1.target_system , packet1.target_component );
    mavlink_msg_command_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_ack_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.command , packet1.result , packet1.progress , packet1.result_param2 , packet1.target_system , packet1.target_component );
    mavlink_msg_command_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_command_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_ack_send(MAVLINK_COMM_1 , packet1.command , packet1.result , packet1.progress , packet1.result_param2 , packet1.target_system , packet1.target_component );
    mavlink_msg_command_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("COMMAND_ACK") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_COMMAND_ACK) != NULL);
#endif
}

static void mavlink_test_manual_setpoint(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MANUAL_SETPOINT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_manual_setpoint_t packet_in = {
        963497464,45.0,73.0,101.0,129.0,65,132
    };
    mavlink_manual_setpoint_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.roll = packet_in.roll;
        packet1.pitch = packet_in.pitch;
        packet1.yaw = packet_in.yaw;
        packet1.thrust = packet_in.thrust;
        packet1.mode_switch = packet_in.mode_switch;
        packet1.manual_override_switch = packet_in.manual_override_switch;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MANUAL_SETPOINT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MANUAL_SETPOINT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_manual_setpoint_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_manual_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_manual_setpoint_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust , packet1.mode_switch , packet1.manual_override_switch );
    mavlink_msg_manual_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_manual_setpoint_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust , packet1.mode_switch , packet1.manual_override_switch );
    mavlink_msg_manual_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_manual_setpoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_manual_setpoint_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust , packet1.mode_switch , packet1.manual_override_switch );
    mavlink_msg_manual_setpoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MANUAL_SETPOINT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MANUAL_SETPOINT) != NULL);
#endif
}

static void mavlink_test_set_attitude_target(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_ATTITUDE_TARGET >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_attitude_target_t packet_in = {
        963497464,{ 45.0, 46.0, 47.0, 48.0 },157.0,185.0,213.0,241.0,113,180,247
    };
    mavlink_set_attitude_target_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.body_roll_rate = packet_in.body_roll_rate;
        packet1.body_pitch_rate = packet_in.body_pitch_rate;
        packet1.body_yaw_rate = packet_in.body_yaw_rate;
        packet1.thrust = packet_in.thrust;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.type_mask = packet_in.type_mask;
        
        mav_array_memcpy(packet1.q, packet_in.q, sizeof(float)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_ATTITUDE_TARGET_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_ATTITUDE_TARGET_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_attitude_target_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_attitude_target_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_attitude_target_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.target_system , packet1.target_component , packet1.type_mask , packet1.q , packet1.body_roll_rate , packet1.body_pitch_rate , packet1.body_yaw_rate , packet1.thrust );
    mavlink_msg_set_attitude_target_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_attitude_target_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.target_system , packet1.target_component , packet1.type_mask , packet1.q , packet1.body_roll_rate , packet1.body_pitch_rate , packet1.body_yaw_rate , packet1.thrust );
    mavlink_msg_set_attitude_target_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_attitude_target_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_attitude_target_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.target_system , packet1.target_component , packet1.type_mask , packet1.q , packet1.body_roll_rate , packet1.body_pitch_rate , packet1.body_yaw_rate , packet1.thrust );
    mavlink_msg_set_attitude_target_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SET_ATTITUDE_TARGET") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SET_ATTITUDE_TARGET) != NULL);
#endif
}

static void mavlink_test_attitude_target(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ATTITUDE_TARGET >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_attitude_target_t packet_in = {
        963497464,{ 45.0, 46.0, 47.0, 48.0 },157.0,185.0,213.0,241.0,113
    };
    mavlink_attitude_target_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.body_roll_rate = packet_in.body_roll_rate;
        packet1.body_pitch_rate = packet_in.body_pitch_rate;
        packet1.body_yaw_rate = packet_in.body_yaw_rate;
        packet1.thrust = packet_in.thrust;
        packet1.type_mask = packet_in.type_mask;
        
        mav_array_memcpy(packet1.q, packet_in.q, sizeof(float)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ATTITUDE_TARGET_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ATTITUDE_TARGET_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_attitude_target_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_attitude_target_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_attitude_target_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.type_mask , packet1.q , packet1.body_roll_rate , packet1.body_pitch_rate , packet1.body_yaw_rate , packet1.thrust );
    mavlink_msg_attitude_target_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_attitude_target_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.type_mask , packet1.q , packet1.body_roll_rate , packet1.body_pitch_rate , packet1.body_yaw_rate , packet1.thrust );
    mavlink_msg_attitude_target_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_attitude_target_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_attitude_target_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.type_mask , packet1.q , packet1.body_roll_rate , packet1.body_pitch_rate , packet1.body_yaw_rate , packet1.thrust );
    mavlink_msg_attitude_target_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ATTITUDE_TARGET") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ATTITUDE_TARGET) != NULL);
#endif
}

static void mavlink_test_set_position_target_local_ned(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_position_target_local_ned_t packet_in = {
        963497464,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,19731,27,94,161
    };
    mavlink_set_position_target_local_ned_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.vx = packet_in.vx;
        packet1.vy = packet_in.vy;
        packet1.vz = packet_in.vz;
        packet1.afx = packet_in.afx;
        packet1.afy = packet_in.afy;
        packet1.afz = packet_in.afz;
        packet1.yaw = packet_in.yaw;
        packet1.yaw_rate = packet_in.yaw_rate;
        packet1.type_mask = packet_in.type_mask;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.coordinate_frame = packet_in.coordinate_frame;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_position_target_local_ned_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_position_target_local_ned_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_position_target_local_ned_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.target_system , packet1.target_component , packet1.coordinate_frame , packet1.type_mask , packet1.x , packet1.y , packet1.z , packet1.vx , packet1.vy , packet1.vz , packet1.afx , packet1.afy , packet1.afz , packet1.yaw , packet1.yaw_rate );
    mavlink_msg_set_position_target_local_ned_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_position_target_local_ned_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.target_system , packet1.target_component , packet1.coordinate_frame , packet1.type_mask , packet1.x , packet1.y , packet1.z , packet1.vx , packet1.vy , packet1.vz , packet1.afx , packet1.afy , packet1.afz , packet1.yaw , packet1.yaw_rate );
    mavlink_msg_set_position_target_local_ned_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_position_target_local_ned_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_position_target_local_ned_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.target_system , packet1.target_component , packet1.coordinate_frame , packet1.type_mask , packet1.x , packet1.y , packet1.z , packet1.vx , packet1.vy , packet1.vz , packet1.afx , packet1.afy , packet1.afz , packet1.yaw , packet1.yaw_rate );
    mavlink_msg_set_position_target_local_ned_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SET_POSITION_TARGET_LOCAL_NED") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED) != NULL);
#endif
}

static void mavlink_test_position_target_local_ned(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_position_target_local_ned_t packet_in = {
        963497464,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,19731,27
    };
    mavlink_position_target_local_ned_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.vx = packet_in.vx;
        packet1.vy = packet_in.vy;
        packet1.vz = packet_in.vz;
        packet1.afx = packet_in.afx;
        packet1.afy = packet_in.afy;
        packet1.afz = packet_in.afz;
        packet1.yaw = packet_in.yaw;
        packet1.yaw_rate = packet_in.yaw_rate;
        packet1.type_mask = packet_in.type_mask;
        packet1.coordinate_frame = packet_in.coordinate_frame;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_position_target_local_ned_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_position_target_local_ned_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_position_target_local_ned_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.coordinate_frame , packet1.type_mask , packet1.x , packet1.y , packet1.z , packet1.vx , packet1.vy , packet1.vz , packet1.afx , packet1.afy , packet1.afz , packet1.yaw , packet1.yaw_rate );
    mavlink_msg_position_target_local_ned_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_position_target_local_ned_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.coordinate_frame , packet1.type_mask , packet1.x , packet1.y , packet1.z , packet1.vx , packet1.vy , packet1.vz , packet1.afx , packet1.afy , packet1.afz , packet1.yaw , packet1.yaw_rate );
    mavlink_msg_position_target_local_ned_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_position_target_local_ned_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_position_target_local_ned_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.coordinate_frame , packet1.type_mask , packet1.x , packet1.y , packet1.z , packet1.vx , packet1.vy , packet1.vz , packet1.afx , packet1.afy , packet1.afz , packet1.yaw , packet1.yaw_rate );
    mavlink_msg_position_target_local_ned_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("POSITION_TARGET_LOCAL_NED") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED) != NULL);
#endif
}

static void mavlink_test_set_position_target_global_int(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_position_target_global_int_t packet_in = {
        963497464,963497672,963497880,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,19731,27,94,161
    };
    mavlink_set_position_target_global_int_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.lat_int = packet_in.lat_int;
        packet1.lon_int = packet_in.lon_int;
        packet1.alt = packet_in.alt;
        packet1.vx = packet_in.vx;
        packet1.vy = packet_in.vy;
        packet1.vz = packet_in.vz;
        packet1.afx = packet_in.afx;
        packet1.afy = packet_in.afy;
        packet1.afz = packet_in.afz;
        packet1.yaw = packet_in.yaw;
        packet1.yaw_rate = packet_in.yaw_rate;
        packet1.type_mask = packet_in.type_mask;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.coordinate_frame = packet_in.coordinate_frame;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_position_target_global_int_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_position_target_global_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_position_target_global_int_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.target_system , packet1.target_component , packet1.coordinate_frame , packet1.type_mask , packet1.lat_int , packet1.lon_int , packet1.alt , packet1.vx , packet1.vy , packet1.vz , packet1.afx , packet1.afy , packet1.afz , packet1.yaw , packet1.yaw_rate );
    mavlink_msg_set_position_target_global_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_position_target_global_int_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.target_system , packet1.target_component , packet1.coordinate_frame , packet1.type_mask , packet1.lat_int , packet1.lon_int , packet1.alt , packet1.vx , packet1.vy , packet1.vz , packet1.afx , packet1.afy , packet1.afz , packet1.yaw , packet1.yaw_rate );
    mavlink_msg_set_position_target_global_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_position_target_global_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_position_target_global_int_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.target_system , packet1.target_component , packet1.coordinate_frame , packet1.type_mask , packet1.lat_int , packet1.lon_int , packet1.alt , packet1.vx , packet1.vy , packet1.vz , packet1.afx , packet1.afy , packet1.afz , packet1.yaw , packet1.yaw_rate );
    mavlink_msg_set_position_target_global_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SET_POSITION_TARGET_GLOBAL_INT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT) != NULL);
#endif
}

static void mavlink_test_position_target_global_int(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_position_target_global_int_t packet_in = {
        963497464,963497672,963497880,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,19731,27
    };
    mavlink_position_target_global_int_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.lat_int = packet_in.lat_int;
        packet1.lon_int = packet_in.lon_int;
        packet1.alt = packet_in.alt;
        packet1.vx = packet_in.vx;
        packet1.vy = packet_in.vy;
        packet1.vz = packet_in.vz;
        packet1.afx = packet_in.afx;
        packet1.afy = packet_in.afy;
        packet1.afz = packet_in.afz;
        packet1.yaw = packet_in.yaw;
        packet1.yaw_rate = packet_in.yaw_rate;
        packet1.type_mask = packet_in.type_mask;
        packet1.coordinate_frame = packet_in.coordinate_frame;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_position_target_global_int_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_position_target_global_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_position_target_global_int_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.coordinate_frame , packet1.type_mask , packet1.lat_int , packet1.lon_int , packet1.alt , packet1.vx , packet1.vy , packet1.vz , packet1.afx , packet1.afy , packet1.afz , packet1.yaw , packet1.yaw_rate );
    mavlink_msg_position_target_global_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_position_target_global_int_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.coordinate_frame , packet1.type_mask , packet1.lat_int , packet1.lon_int , packet1.alt , packet1.vx , packet1.vy , packet1.vz , packet1.afx , packet1.afy , packet1.afz , packet1.yaw , packet1.yaw_rate );
    mavlink_msg_position_target_global_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_position_target_global_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_position_target_global_int_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.coordinate_frame , packet1.type_mask , packet1.lat_int , packet1.lon_int , packet1.alt , packet1.vx , packet1.vy , packet1.vz , packet1.afx , packet1.afy , packet1.afz , packet1.yaw , packet1.yaw_rate );
    mavlink_msg_position_target_global_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("POSITION_TARGET_GLOBAL_INT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT) != NULL);
#endif
}

static void mavlink_test_local_position_ned_system_global_offset(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_local_position_ned_system_global_offset_t packet_in = {
        963497464,45.0,73.0,101.0,129.0,157.0,185.0
    };
    mavlink_local_position_ned_system_global_offset_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.roll = packet_in.roll;
        packet1.pitch = packet_in.pitch;
        packet1.yaw = packet_in.yaw;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_local_position_ned_system_global_offset_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_local_position_ned_system_global_offset_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_local_position_ned_system_global_offset_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.x , packet1.y , packet1.z , packet1.roll , packet1.pitch , packet1.yaw );
    mavlink_msg_local_position_ned_system_global_offset_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_local_position_ned_system_global_offset_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.x , packet1.y , packet1.z , packet1.roll , packet1.pitch , packet1.yaw );
    mavlink_msg_local_position_ned_system_global_offset_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_local_position_ned_system_global_offset_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_local_position_ned_system_global_offset_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.x , packet1.y , packet1.z , packet1.roll , packet1.pitch , packet1.yaw );
    mavlink_msg_local_position_ned_system_global_offset_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET) != NULL);
#endif
}

static void mavlink_test_hil_state(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_HIL_STATE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_hil_state_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,963499128,963499336,963499544,19523,19627,19731,19835,19939,20043
    };
    mavlink_hil_state_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.roll = packet_in.roll;
        packet1.pitch = packet_in.pitch;
        packet1.yaw = packet_in.yaw;
        packet1.rollspeed = packet_in.rollspeed;
        packet1.pitchspeed = packet_in.pitchspeed;
        packet1.yawspeed = packet_in.yawspeed;
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        packet1.alt = packet_in.alt;
        packet1.vx = packet_in.vx;
        packet1.vy = packet_in.vy;
        packet1.vz = packet_in.vz;
        packet1.xacc = packet_in.xacc;
        packet1.yacc = packet_in.yacc;
        packet1.zacc = packet_in.zacc;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_HIL_STATE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_HIL_STATE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hil_state_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_hil_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hil_state_pack(system_id, component_id, &msg , packet1.time_usec , packet1.roll , packet1.pitch , packet1.yaw , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed , packet1.lat , packet1.lon , packet1.alt , packet1.vx , packet1.vy , packet1.vz , packet1.xacc , packet1.yacc , packet1.zacc );
    mavlink_msg_hil_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hil_state_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.roll , packet1.pitch , packet1.yaw , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed , packet1.lat , packet1.lon , packet1.alt , packet1.vx , packet1.vy , packet1.vz , packet1.xacc , packet1.yacc , packet1.zacc );
    mavlink_msg_hil_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_hil_state_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hil_state_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.roll , packet1.pitch , packet1.yaw , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed , packet1.lat , packet1.lon , packet1.alt , packet1.vx , packet1.vy , packet1.vz , packet1.xacc , packet1.yacc , packet1.zacc );
    mavlink_msg_hil_state_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("HIL_STATE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_HIL_STATE) != NULL);
#endif
}

static void mavlink_test_hil_controls(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_HIL_CONTROLS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_hil_controls_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,125,192
    };
    mavlink_hil_controls_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.roll_ailerons = packet_in.roll_ailerons;
        packet1.pitch_elevator = packet_in.pitch_elevator;
        packet1.yaw_rudder = packet_in.yaw_rudder;
        packet1.throttle = packet_in.throttle;
        packet1.aux1 = packet_in.aux1;
        packet1.aux2 = packet_in.aux2;
        packet1.aux3 = packet_in.aux3;
        packet1.aux4 = packet_in.aux4;
        packet1.mode = packet_in.mode;
        packet1.nav_mode = packet_in.nav_mode;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_HIL_CONTROLS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_HIL_CONTROLS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hil_controls_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_hil_controls_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hil_controls_pack(system_id, component_id, &msg , packet1.time_usec , packet1.roll_ailerons , packet1.pitch_elevator , packet1.yaw_rudder , packet1.throttle , packet1.aux1 , packet1.aux2 , packet1.aux3 , packet1.aux4 , packet1.mode , packet1.nav_mode );
    mavlink_msg_hil_controls_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hil_controls_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.roll_ailerons , packet1.pitch_elevator , packet1.yaw_rudder , packet1.throttle , packet1.aux1 , packet1.aux2 , packet1.aux3 , packet1.aux4 , packet1.mode , packet1.nav_mode );
    mavlink_msg_hil_controls_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_hil_controls_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hil_controls_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.roll_ailerons , packet1.pitch_elevator , packet1.yaw_rudder , packet1.throttle , packet1.aux1 , packet1.aux2 , packet1.aux3 , packet1.aux4 , packet1.mode , packet1.nav_mode );
    mavlink_msg_hil_controls_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("HIL_CONTROLS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_HIL_CONTROLS) != NULL);
#endif
}

static void mavlink_test_hil_rc_inputs_raw(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_hil_rc_inputs_raw_t packet_in = {
        93372036854775807ULL,17651,17755,17859,17963,18067,18171,18275,18379,18483,18587,18691,18795,101
    };
    mavlink_hil_rc_inputs_raw_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.chan1_raw = packet_in.chan1_raw;
        packet1.chan2_raw = packet_in.chan2_raw;
        packet1.chan3_raw = packet_in.chan3_raw;
        packet1.chan4_raw = packet_in.chan4_raw;
        packet1.chan5_raw = packet_in.chan5_raw;
        packet1.chan6_raw = packet_in.chan6_raw;
        packet1.chan7_raw = packet_in.chan7_raw;
        packet1.chan8_raw = packet_in.chan8_raw;
        packet1.chan9_raw = packet_in.chan9_raw;
        packet1.chan10_raw = packet_in.chan10_raw;
        packet1.chan11_raw = packet_in.chan11_raw;
        packet1.chan12_raw = packet_in.chan12_raw;
        packet1.rssi = packet_in.rssi;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hil_rc_inputs_raw_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_hil_rc_inputs_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hil_rc_inputs_raw_pack(system_id, component_id, &msg , packet1.time_usec , packet1.chan1_raw , packet1.chan2_raw , packet1.chan3_raw , packet1.chan4_raw , packet1.chan5_raw , packet1.chan6_raw , packet1.chan7_raw , packet1.chan8_raw , packet1.chan9_raw , packet1.chan10_raw , packet1.chan11_raw , packet1.chan12_raw , packet1.rssi );
    mavlink_msg_hil_rc_inputs_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hil_rc_inputs_raw_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.chan1_raw , packet1.chan2_raw , packet1.chan3_raw , packet1.chan4_raw , packet1.chan5_raw , packet1.chan6_raw , packet1.chan7_raw , packet1.chan8_raw , packet1.chan9_raw , packet1.chan10_raw , packet1.chan11_raw , packet1.chan12_raw , packet1.rssi );
    mavlink_msg_hil_rc_inputs_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_hil_rc_inputs_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hil_rc_inputs_raw_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.chan1_raw , packet1.chan2_raw , packet1.chan3_raw , packet1.chan4_raw , packet1.chan5_raw , packet1.chan6_raw , packet1.chan7_raw , packet1.chan8_raw , packet1.chan9_raw , packet1.chan10_raw , packet1.chan11_raw , packet1.chan12_raw , packet1.rssi );
    mavlink_msg_hil_rc_inputs_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("HIL_RC_INPUTS_RAW") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW) != NULL);
#endif
}

static void mavlink_test_hil_actuator_controls(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_hil_actuator_controls_t packet_in = {
        93372036854775807ULL,93372036854776311ULL,{ 129.0, 130.0, 131.0, 132.0, 133.0, 134.0, 135.0, 136.0, 137.0, 138.0, 139.0, 140.0, 141.0, 142.0, 143.0, 144.0 },245
    };
    mavlink_hil_actuator_controls_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.flags = packet_in.flags;
        packet1.mode = packet_in.mode;
        
        mav_array_memcpy(packet1.controls, packet_in.controls, sizeof(float)*16);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hil_actuator_controls_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_hil_actuator_controls_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hil_actuator_controls_pack(system_id, component_id, &msg , packet1.time_usec , packet1.controls , packet1.mode , packet1.flags );
    mavlink_msg_hil_actuator_controls_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hil_actuator_controls_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.controls , packet1.mode , packet1.flags );
    mavlink_msg_hil_actuator_controls_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_hil_actuator_controls_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hil_actuator_controls_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.controls , packet1.mode , packet1.flags );
    mavlink_msg_hil_actuator_controls_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("HIL_ACTUATOR_CONTROLS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS) != NULL);
#endif
}

static void mavlink_test_optical_flow(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_OPTICAL_FLOW >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_optical_flow_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,18275,18379,77,144,199.0,227.0
    };
    mavlink_optical_flow_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.flow_comp_m_x = packet_in.flow_comp_m_x;
        packet1.flow_comp_m_y = packet_in.flow_comp_m_y;
        packet1.ground_distance = packet_in.ground_distance;
        packet1.flow_x = packet_in.flow_x;
        packet1.flow_y = packet_in.flow_y;
        packet1.sensor_id = packet_in.sensor_id;
        packet1.quality = packet_in.quality;
        packet1.flow_rate_x = packet_in.flow_rate_x;
        packet1.flow_rate_y = packet_in.flow_rate_y;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_OPTICAL_FLOW_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_OPTICAL_FLOW_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_optical_flow_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_optical_flow_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_optical_flow_pack(system_id, component_id, &msg , packet1.time_usec , packet1.sensor_id , packet1.flow_x , packet1.flow_y , packet1.flow_comp_m_x , packet1.flow_comp_m_y , packet1.quality , packet1.ground_distance , packet1.flow_rate_x , packet1.flow_rate_y );
    mavlink_msg_optical_flow_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_optical_flow_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.sensor_id , packet1.flow_x , packet1.flow_y , packet1.flow_comp_m_x , packet1.flow_comp_m_y , packet1.quality , packet1.ground_distance , packet1.flow_rate_x , packet1.flow_rate_y );
    mavlink_msg_optical_flow_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_optical_flow_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_optical_flow_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.sensor_id , packet1.flow_x , packet1.flow_y , packet1.flow_comp_m_x , packet1.flow_comp_m_y , packet1.quality , packet1.ground_distance , packet1.flow_rate_x , packet1.flow_rate_y );
    mavlink_msg_optical_flow_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("OPTICAL_FLOW") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_OPTICAL_FLOW) != NULL);
#endif
}

static void mavlink_test_global_vision_position_estimate(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_global_vision_position_estimate_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,{ 241.0, 242.0, 243.0, 244.0, 245.0, 246.0, 247.0, 248.0, 249.0, 250.0, 251.0, 252.0, 253.0, 254.0, 255.0, 256.0, 257.0, 258.0, 259.0, 260.0, 261.0 },97
    };
    mavlink_global_vision_position_estimate_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.usec = packet_in.usec;
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.roll = packet_in.roll;
        packet1.pitch = packet_in.pitch;
        packet1.yaw = packet_in.yaw;
        packet1.reset_counter = packet_in.reset_counter;
        
        mav_array_memcpy(packet1.covariance, packet_in.covariance, sizeof(float)*21);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_global_vision_position_estimate_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_global_vision_position_estimate_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_global_vision_position_estimate_pack(system_id, component_id, &msg , packet1.usec , packet1.x , packet1.y , packet1.z , packet1.roll , packet1.pitch , packet1.yaw , packet1.covariance , packet1.reset_counter );
    mavlink_msg_global_vision_position_estimate_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_global_vision_position_estimate_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.usec , packet1.x , packet1.y , packet1.z , packet1.roll , packet1.pitch , packet1.yaw , packet1.covariance , packet1.reset_counter );
    mavlink_msg_global_vision_position_estimate_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_global_vision_position_estimate_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_global_vision_position_estimate_send(MAVLINK_COMM_1 , packet1.usec , packet1.x , packet1.y , packet1.z , packet1.roll , packet1.pitch , packet1.yaw , packet1.covariance , packet1.reset_counter );
    mavlink_msg_global_vision_position_estimate_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("GLOBAL_VISION_POSITION_ESTIMATE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE) != NULL);
#endif
}

static void mavlink_test_vision_position_estimate(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_vision_position_estimate_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,{ 241.0, 242.0, 243.0, 244.0, 245.0, 246.0, 247.0, 248.0, 249.0, 250.0, 251.0, 252.0, 253.0, 254.0, 255.0, 256.0, 257.0, 258.0, 259.0, 260.0, 261.0 },97
    };
    mavlink_vision_position_estimate_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.usec = packet_in.usec;
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.roll = packet_in.roll;
        packet1.pitch = packet_in.pitch;
        packet1.yaw = packet_in.yaw;
        packet1.reset_counter = packet_in.reset_counter;
        
        mav_array_memcpy(packet1.covariance, packet_in.covariance, sizeof(float)*21);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_vision_position_estimate_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_vision_position_estimate_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_vision_position_estimate_pack(system_id, component_id, &msg , packet1.usec , packet1.x , packet1.y , packet1.z , packet1.roll , packet1.pitch , packet1.yaw , packet1.covariance , packet1.reset_counter );
    mavlink_msg_vision_position_estimate_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_vision_position_estimate_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.usec , packet1.x , packet1.y , packet1.z , packet1.roll , packet1.pitch , packet1.yaw , packet1.covariance , packet1.reset_counter );
    mavlink_msg_vision_position_estimate_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_vision_position_estimate_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_vision_position_estimate_send(MAVLINK_COMM_1 , packet1.usec , packet1.x , packet1.y , packet1.z , packet1.roll , packet1.pitch , packet1.yaw , packet1.covariance , packet1.reset_counter );
    mavlink_msg_vision_position_estimate_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("VISION_POSITION_ESTIMATE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE) != NULL);
#endif
}

static void mavlink_test_vision_speed_estimate(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_vision_speed_estimate_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,{ 157.0, 158.0, 159.0, 160.0, 161.0, 162.0, 163.0, 164.0, 165.0 },173
    };
    mavlink_vision_speed_estimate_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.usec = packet_in.usec;
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.reset_counter = packet_in.reset_counter;
        
        mav_array_memcpy(packet1.covariance, packet_in.covariance, sizeof(float)*9);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_vision_speed_estimate_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_vision_speed_estimate_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_vision_speed_estimate_pack(system_id, component_id, &msg , packet1.usec , packet1.x , packet1.y , packet1.z , packet1.covariance , packet1.reset_counter );
    mavlink_msg_vision_speed_estimate_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_vision_speed_estimate_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.usec , packet1.x , packet1.y , packet1.z , packet1.covariance , packet1.reset_counter );
    mavlink_msg_vision_speed_estimate_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_vision_speed_estimate_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_vision_speed_estimate_send(MAVLINK_COMM_1 , packet1.usec , packet1.x , packet1.y , packet1.z , packet1.covariance , packet1.reset_counter );
    mavlink_msg_vision_speed_estimate_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("VISION_SPEED_ESTIMATE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE) != NULL);
#endif
}

static void mavlink_test_vicon_position_estimate(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_vicon_position_estimate_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,{ 241.0, 242.0, 243.0, 244.0, 245.0, 246.0, 247.0, 248.0, 249.0, 250.0, 251.0, 252.0, 253.0, 254.0, 255.0, 256.0, 257.0, 258.0, 259.0, 260.0, 261.0 }
    };
    mavlink_vicon_position_estimate_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.usec = packet_in.usec;
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.roll = packet_in.roll;
        packet1.pitch = packet_in.pitch;
        packet1.yaw = packet_in.yaw;
        
        mav_array_memcpy(packet1.covariance, packet_in.covariance, sizeof(float)*21);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_vicon_position_estimate_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_vicon_position_estimate_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_vicon_position_estimate_pack(system_id, component_id, &msg , packet1.usec , packet1.x , packet1.y , packet1.z , packet1.roll , packet1.pitch , packet1.yaw , packet1.covariance );
    mavlink_msg_vicon_position_estimate_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_vicon_position_estimate_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.usec , packet1.x , packet1.y , packet1.z , packet1.roll , packet1.pitch , packet1.yaw , packet1.covariance );
    mavlink_msg_vicon_position_estimate_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_vicon_position_estimate_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_vicon_position_estimate_send(MAVLINK_COMM_1 , packet1.usec , packet1.x , packet1.y , packet1.z , packet1.roll , packet1.pitch , packet1.yaw , packet1.covariance );
    mavlink_msg_vicon_position_estimate_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("VICON_POSITION_ESTIMATE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE) != NULL);
#endif
}

static void mavlink_test_highres_imu(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_HIGHRES_IMU >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_highres_imu_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0,409.0,20355,63
    };
    mavlink_highres_imu_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.xacc = packet_in.xacc;
        packet1.yacc = packet_in.yacc;
        packet1.zacc = packet_in.zacc;
        packet1.xgyro = packet_in.xgyro;
        packet1.ygyro = packet_in.ygyro;
        packet1.zgyro = packet_in.zgyro;
        packet1.xmag = packet_in.xmag;
        packet1.ymag = packet_in.ymag;
        packet1.zmag = packet_in.zmag;
        packet1.abs_pressure = packet_in.abs_pressure;
        packet1.diff_pressure = packet_in.diff_pressure;
        packet1.pressure_alt = packet_in.pressure_alt;
        packet1.temperature = packet_in.temperature;
        packet1.fields_updated = packet_in.fields_updated;
        packet1.id = packet_in.id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_HIGHRES_IMU_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_HIGHRES_IMU_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_highres_imu_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_highres_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_highres_imu_pack(system_id, component_id, &msg , packet1.time_usec , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xmag , packet1.ymag , packet1.zmag , packet1.abs_pressure , packet1.diff_pressure , packet1.pressure_alt , packet1.temperature , packet1.fields_updated , packet1.id );
    mavlink_msg_highres_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_highres_imu_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xmag , packet1.ymag , packet1.zmag , packet1.abs_pressure , packet1.diff_pressure , packet1.pressure_alt , packet1.temperature , packet1.fields_updated , packet1.id );
    mavlink_msg_highres_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_highres_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_highres_imu_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xmag , packet1.ymag , packet1.zmag , packet1.abs_pressure , packet1.diff_pressure , packet1.pressure_alt , packet1.temperature , packet1.fields_updated , packet1.id );
    mavlink_msg_highres_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("HIGHRES_IMU") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_HIGHRES_IMU) != NULL);
#endif
}

static void mavlink_test_optical_flow_rad(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_OPTICAL_FLOW_RAD >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_optical_flow_rad_t packet_in = {
        93372036854775807ULL,963497880,101.0,129.0,157.0,185.0,213.0,963499128,269.0,19315,3,70
    };
    mavlink_optical_flow_rad_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.integration_time_us = packet_in.integration_time_us;
        packet1.integrated_x = packet_in.integrated_x;
        packet1.integrated_y = packet_in.integrated_y;
        packet1.integrated_xgyro = packet_in.integrated_xgyro;
        packet1.integrated_ygyro = packet_in.integrated_ygyro;
        packet1.integrated_zgyro = packet_in.integrated_zgyro;
        packet1.time_delta_distance_us = packet_in.time_delta_distance_us;
        packet1.distance = packet_in.distance;
        packet1.temperature = packet_in.temperature;
        packet1.sensor_id = packet_in.sensor_id;
        packet1.quality = packet_in.quality;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_OPTICAL_FLOW_RAD_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_OPTICAL_FLOW_RAD_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_optical_flow_rad_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_optical_flow_rad_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_optical_flow_rad_pack(system_id, component_id, &msg , packet1.time_usec , packet1.sensor_id , packet1.integration_time_us , packet1.integrated_x , packet1.integrated_y , packet1.integrated_xgyro , packet1.integrated_ygyro , packet1.integrated_zgyro , packet1.temperature , packet1.quality , packet1.time_delta_distance_us , packet1.distance );
    mavlink_msg_optical_flow_rad_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_optical_flow_rad_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.sensor_id , packet1.integration_time_us , packet1.integrated_x , packet1.integrated_y , packet1.integrated_xgyro , packet1.integrated_ygyro , packet1.integrated_zgyro , packet1.temperature , packet1.quality , packet1.time_delta_distance_us , packet1.distance );
    mavlink_msg_optical_flow_rad_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_optical_flow_rad_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_optical_flow_rad_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.sensor_id , packet1.integration_time_us , packet1.integrated_x , packet1.integrated_y , packet1.integrated_xgyro , packet1.integrated_ygyro , packet1.integrated_zgyro , packet1.temperature , packet1.quality , packet1.time_delta_distance_us , packet1.distance );
    mavlink_msg_optical_flow_rad_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("OPTICAL_FLOW_RAD") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_OPTICAL_FLOW_RAD) != NULL);
#endif
}

static void mavlink_test_hil_sensor(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_HIL_SENSOR >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_hil_sensor_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0,409.0,963500584,197
    };
    mavlink_hil_sensor_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.xacc = packet_in.xacc;
        packet1.yacc = packet_in.yacc;
        packet1.zacc = packet_in.zacc;
        packet1.xgyro = packet_in.xgyro;
        packet1.ygyro = packet_in.ygyro;
        packet1.zgyro = packet_in.zgyro;
        packet1.xmag = packet_in.xmag;
        packet1.ymag = packet_in.ymag;
        packet1.zmag = packet_in.zmag;
        packet1.abs_pressure = packet_in.abs_pressure;
        packet1.diff_pressure = packet_in.diff_pressure;
        packet1.pressure_alt = packet_in.pressure_alt;
        packet1.temperature = packet_in.temperature;
        packet1.fields_updated = packet_in.fields_updated;
        packet1.id = packet_in.id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_HIL_SENSOR_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_HIL_SENSOR_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hil_sensor_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_hil_sensor_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hil_sensor_pack(system_id, component_id, &msg , packet1.time_usec , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xmag , packet1.ymag , packet1.zmag , packet1.abs_pressure , packet1.diff_pressure , packet1.pressure_alt , packet1.temperature , packet1.fields_updated , packet1.id );
    mavlink_msg_hil_sensor_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hil_sensor_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xmag , packet1.ymag , packet1.zmag , packet1.abs_pressure , packet1.diff_pressure , packet1.pressure_alt , packet1.temperature , packet1.fields_updated , packet1.id );
    mavlink_msg_hil_sensor_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_hil_sensor_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hil_sensor_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xmag , packet1.ymag , packet1.zmag , packet1.abs_pressure , packet1.diff_pressure , packet1.pressure_alt , packet1.temperature , packet1.fields_updated , packet1.id );
    mavlink_msg_hil_sensor_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("HIL_SENSOR") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_HIL_SENSOR) != NULL);
#endif
}

static void mavlink_test_sim_state(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SIM_STATE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_sim_state_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0,409.0,437.0,465.0,493.0,521.0,549.0,577.0,963501832,963502040
    };
    mavlink_sim_state_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.q1 = packet_in.q1;
        packet1.q2 = packet_in.q2;
        packet1.q3 = packet_in.q3;
        packet1.q4 = packet_in.q4;
        packet1.roll = packet_in.roll;
        packet1.pitch = packet_in.pitch;
        packet1.yaw = packet_in.yaw;
        packet1.xacc = packet_in.xacc;
        packet1.yacc = packet_in.yacc;
        packet1.zacc = packet_in.zacc;
        packet1.xgyro = packet_in.xgyro;
        packet1.ygyro = packet_in.ygyro;
        packet1.zgyro = packet_in.zgyro;
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        packet1.alt = packet_in.alt;
        packet1.std_dev_horz = packet_in.std_dev_horz;
        packet1.std_dev_vert = packet_in.std_dev_vert;
        packet1.vn = packet_in.vn;
        packet1.ve = packet_in.ve;
        packet1.vd = packet_in.vd;
        packet1.lat_int = packet_in.lat_int;
        packet1.lon_int = packet_in.lon_int;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SIM_STATE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SIM_STATE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sim_state_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_sim_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sim_state_pack(system_id, component_id, &msg , packet1.q1 , packet1.q2 , packet1.q3 , packet1.q4 , packet1.roll , packet1.pitch , packet1.yaw , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.lat , packet1.lon , packet1.alt , packet1.std_dev_horz , packet1.std_dev_vert , packet1.vn , packet1.ve , packet1.vd , packet1.lat_int , packet1.lon_int );
    mavlink_msg_sim_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sim_state_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.q1 , packet1.q2 , packet1.q3 , packet1.q4 , packet1.roll , packet1.pitch , packet1.yaw , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.lat , packet1.lon , packet1.alt , packet1.std_dev_horz , packet1.std_dev_vert , packet1.vn , packet1.ve , packet1.vd , packet1.lat_int , packet1.lon_int );
    mavlink_msg_sim_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_sim_state_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sim_state_send(MAVLINK_COMM_1 , packet1.q1 , packet1.q2 , packet1.q3 , packet1.q4 , packet1.roll , packet1.pitch , packet1.yaw , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.lat , packet1.lon , packet1.alt , packet1.std_dev_horz , packet1.std_dev_vert , packet1.vn , packet1.ve , packet1.vd , packet1.lat_int , packet1.lon_int );
    mavlink_msg_sim_state_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SIM_STATE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SIM_STATE) != NULL);
#endif
}

static void mavlink_test_radio_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RADIO_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_radio_status_t packet_in = {
        17235,17339,17,84,151,218,29
    };
    mavlink_radio_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.rxerrors = packet_in.rxerrors;
        packet1.fixed = packet_in.fixed;
        packet1.rssi = packet_in.rssi;
        packet1.remrssi = packet_in.remrssi;
        packet1.txbuf = packet_in.txbuf;
        packet1.noise = packet_in.noise;
        packet1.remnoise = packet_in.remnoise;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_RADIO_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RADIO_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_radio_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_radio_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_radio_status_pack(system_id, component_id, &msg , packet1.rssi , packet1.remrssi , packet1.txbuf , packet1.noise , packet1.remnoise , packet1.rxerrors , packet1.fixed );
    mavlink_msg_radio_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_radio_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.rssi , packet1.remrssi , packet1.txbuf , packet1.noise , packet1.remnoise , packet1.rxerrors , packet1.fixed );
    mavlink_msg_radio_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_radio_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_radio_status_send(MAVLINK_COMM_1 , packet1.rssi , packet1.remrssi , packet1.txbuf , packet1.noise , packet1.remnoise , packet1.rxerrors , packet1.fixed );
    mavlink_msg_radio_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("RADIO_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_RADIO_STATUS) != NULL);
#endif
}

static void mavlink_test_file_transfer_protocol(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_file_transfer_protocol_t packet_in = {
        5,72,139,{ 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200 }
    };
    mavlink_file_transfer_protocol_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_network = packet_in.target_network;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        mav_array_memcpy(packet1.payload, packet_in.payload, sizeof(uint8_t)*251);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_file_transfer_protocol_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_file_transfer_protocol_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_file_transfer_protocol_pack(system_id, component_id, &msg , packet1.target_network , packet1.target_system , packet1.target_component , packet1.payload );
    mavlink_msg_file_transfer_protocol_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_file_transfer_protocol_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_network , packet1.target_system , packet1.target_component , packet1.payload );
    mavlink_msg_file_transfer_protocol_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_file_transfer_protocol_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_file_transfer_protocol_send(MAVLINK_COMM_1 , packet1.target_network , packet1.target_system , packet1.target_component , packet1.payload );
    mavlink_msg_file_transfer_protocol_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("FILE_TRANSFER_PROTOCOL") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL) != NULL);
#endif
}

static void mavlink_test_timesync(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TIMESYNC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_timesync_t packet_in = {
        93372036854775807LL,93372036854776311LL
    };
    mavlink_timesync_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.tc1 = packet_in.tc1;
        packet1.ts1 = packet_in.ts1;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TIMESYNC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TIMESYNC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_timesync_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_timesync_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_timesync_pack(system_id, component_id, &msg , packet1.tc1 , packet1.ts1 );
    mavlink_msg_timesync_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_timesync_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.tc1 , packet1.ts1 );
    mavlink_msg_timesync_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_timesync_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_timesync_send(MAVLINK_COMM_1 , packet1.tc1 , packet1.ts1 );
    mavlink_msg_timesync_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("TIMESYNC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_TIMESYNC) != NULL);
#endif
}

static void mavlink_test_camera_trigger(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CAMERA_TRIGGER >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_camera_trigger_t packet_in = {
        93372036854775807ULL,963497880
    };
    mavlink_camera_trigger_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.seq = packet_in.seq;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CAMERA_TRIGGER_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CAMERA_TRIGGER_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_trigger_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_camera_trigger_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_trigger_pack(system_id, component_id, &msg , packet1.time_usec , packet1.seq );
    mavlink_msg_camera_trigger_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_trigger_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.seq );
    mavlink_msg_camera_trigger_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_camera_trigger_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_trigger_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.seq );
    mavlink_msg_camera_trigger_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CAMERA_TRIGGER") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CAMERA_TRIGGER) != NULL);
#endif
}

static void mavlink_test_hil_gps(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_HIL_GPS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_hil_gps_t packet_in = {
        93372036854775807ULL,963497880,963498088,963498296,18275,18379,18483,18587,18691,18795,18899,235,46,113,19159
    };
    mavlink_hil_gps_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        packet1.alt = packet_in.alt;
        packet1.eph = packet_in.eph;
        packet1.epv = packet_in.epv;
        packet1.vel = packet_in.vel;
        packet1.vn = packet_in.vn;
        packet1.ve = packet_in.ve;
        packet1.vd = packet_in.vd;
        packet1.cog = packet_in.cog;
        packet1.fix_type = packet_in.fix_type;
        packet1.satellites_visible = packet_in.satellites_visible;
        packet1.id = packet_in.id;
        packet1.yaw = packet_in.yaw;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_HIL_GPS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_HIL_GPS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hil_gps_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_hil_gps_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hil_gps_pack(system_id, component_id, &msg , packet1.time_usec , packet1.fix_type , packet1.lat , packet1.lon , packet1.alt , packet1.eph , packet1.epv , packet1.vel , packet1.vn , packet1.ve , packet1.vd , packet1.cog , packet1.satellites_visible , packet1.id , packet1.yaw );
    mavlink_msg_hil_gps_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hil_gps_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.fix_type , packet1.lat , packet1.lon , packet1.alt , packet1.eph , packet1.epv , packet1.vel , packet1.vn , packet1.ve , packet1.vd , packet1.cog , packet1.satellites_visible , packet1.id , packet1.yaw );
    mavlink_msg_hil_gps_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_hil_gps_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hil_gps_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.fix_type , packet1.lat , packet1.lon , packet1.alt , packet1.eph , packet1.epv , packet1.vel , packet1.vn , packet1.ve , packet1.vd , packet1.cog , packet1.satellites_visible , packet1.id , packet1.yaw );
    mavlink_msg_hil_gps_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("HIL_GPS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_HIL_GPS) != NULL);
#endif
}

static void mavlink_test_hil_optical_flow(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_HIL_OPTICAL_FLOW >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_hil_optical_flow_t packet_in = {
        93372036854775807ULL,963497880,101.0,129.0,157.0,185.0,213.0,963499128,269.0,19315,3,70
    };
    mavlink_hil_optical_flow_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.integration_time_us = packet_in.integration_time_us;
        packet1.integrated_x = packet_in.integrated_x;
        packet1.integrated_y = packet_in.integrated_y;
        packet1.integrated_xgyro = packet_in.integrated_xgyro;
        packet1.integrated_ygyro = packet_in.integrated_ygyro;
        packet1.integrated_zgyro = packet_in.integrated_zgyro;
        packet1.time_delta_distance_us = packet_in.time_delta_distance_us;
        packet1.distance = packet_in.distance;
        packet1.temperature = packet_in.temperature;
        packet1.sensor_id = packet_in.sensor_id;
        packet1.quality = packet_in.quality;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hil_optical_flow_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_hil_optical_flow_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hil_optical_flow_pack(system_id, component_id, &msg , packet1.time_usec , packet1.sensor_id , packet1.integration_time_us , packet1.integrated_x , packet1.integrated_y , packet1.integrated_xgyro , packet1.integrated_ygyro , packet1.integrated_zgyro , packet1.temperature , packet1.quality , packet1.time_delta_distance_us , packet1.distance );
    mavlink_msg_hil_optical_flow_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hil_optical_flow_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.sensor_id , packet1.integration_time_us , packet1.integrated_x , packet1.integrated_y , packet1.integrated_xgyro , packet1.integrated_ygyro , packet1.integrated_zgyro , packet1.temperature , packet1.quality , packet1.time_delta_distance_us , packet1.distance );
    mavlink_msg_hil_optical_flow_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_hil_optical_flow_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hil_optical_flow_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.sensor_id , packet1.integration_time_us , packet1.integrated_x , packet1.integrated_y , packet1.integrated_xgyro , packet1.integrated_ygyro , packet1.integrated_zgyro , packet1.temperature , packet1.quality , packet1.time_delta_distance_us , packet1.distance );
    mavlink_msg_hil_optical_flow_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("HIL_OPTICAL_FLOW") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_HIL_OPTICAL_FLOW) != NULL);
#endif
}

static void mavlink_test_hil_state_quaternion(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_HIL_STATE_QUATERNION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_hil_state_quaternion_t packet_in = {
        93372036854775807ULL,{ 73.0, 74.0, 75.0, 76.0 },185.0,213.0,241.0,963499336,963499544,963499752,19731,19835,19939,20043,20147,20251,20355,20459
    };
    mavlink_hil_state_quaternion_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.rollspeed = packet_in.rollspeed;
        packet1.pitchspeed = packet_in.pitchspeed;
        packet1.yawspeed = packet_in.yawspeed;
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        packet1.alt = packet_in.alt;
        packet1.vx = packet_in.vx;
        packet1.vy = packet_in.vy;
        packet1.vz = packet_in.vz;
        packet1.ind_airspeed = packet_in.ind_airspeed;
        packet1.true_airspeed = packet_in.true_airspeed;
        packet1.xacc = packet_in.xacc;
        packet1.yacc = packet_in.yacc;
        packet1.zacc = packet_in.zacc;
        
        mav_array_memcpy(packet1.attitude_quaternion, packet_in.attitude_quaternion, sizeof(float)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_HIL_STATE_QUATERNION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_HIL_STATE_QUATERNION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hil_state_quaternion_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_hil_state_quaternion_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hil_state_quaternion_pack(system_id, component_id, &msg , packet1.time_usec , packet1.attitude_quaternion , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed , packet1.lat , packet1.lon , packet1.alt , packet1.vx , packet1.vy , packet1.vz , packet1.ind_airspeed , packet1.true_airspeed , packet1.xacc , packet1.yacc , packet1.zacc );
    mavlink_msg_hil_state_quaternion_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hil_state_quaternion_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.attitude_quaternion , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed , packet1.lat , packet1.lon , packet1.alt , packet1.vx , packet1.vy , packet1.vz , packet1.ind_airspeed , packet1.true_airspeed , packet1.xacc , packet1.yacc , packet1.zacc );
    mavlink_msg_hil_state_quaternion_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_hil_state_quaternion_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hil_state_quaternion_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.attitude_quaternion , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed , packet1.lat , packet1.lon , packet1.alt , packet1.vx , packet1.vy , packet1.vz , packet1.ind_airspeed , packet1.true_airspeed , packet1.xacc , packet1.yacc , packet1.zacc );
    mavlink_msg_hil_state_quaternion_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("HIL_STATE_QUATERNION") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_HIL_STATE_QUATERNION) != NULL);
#endif
}

static void mavlink_test_scaled_imu2(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SCALED_IMU2 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_scaled_imu2_t packet_in = {
        963497464,17443,17547,17651,17755,17859,17963,18067,18171,18275,18379
    };
    mavlink_scaled_imu2_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.xacc = packet_in.xacc;
        packet1.yacc = packet_in.yacc;
        packet1.zacc = packet_in.zacc;
        packet1.xgyro = packet_in.xgyro;
        packet1.ygyro = packet_in.ygyro;
        packet1.zgyro = packet_in.zgyro;
        packet1.xmag = packet_in.xmag;
        packet1.ymag = packet_in.ymag;
        packet1.zmag = packet_in.zmag;
        packet1.temperature = packet_in.temperature;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SCALED_IMU2_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SCALED_IMU2_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_scaled_imu2_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_scaled_imu2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_scaled_imu2_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xmag , packet1.ymag , packet1.zmag , packet1.temperature );
    mavlink_msg_scaled_imu2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_scaled_imu2_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xmag , packet1.ymag , packet1.zmag , packet1.temperature );
    mavlink_msg_scaled_imu2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_scaled_imu2_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_scaled_imu2_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xmag , packet1.ymag , packet1.zmag , packet1.temperature );
    mavlink_msg_scaled_imu2_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SCALED_IMU2") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SCALED_IMU2) != NULL);
#endif
}

static void mavlink_test_log_request_list(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LOG_REQUEST_LIST >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_log_request_list_t packet_in = {
        17235,17339,17,84
    };
    mavlink_log_request_list_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.start = packet_in.start;
        packet1.end = packet_in.end;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LOG_REQUEST_LIST_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LOG_REQUEST_LIST_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_log_request_list_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_log_request_list_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_log_request_list_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.start , packet1.end );
    mavlink_msg_log_request_list_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_log_request_list_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.start , packet1.end );
    mavlink_msg_log_request_list_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_log_request_list_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_log_request_list_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.start , packet1.end );
    mavlink_msg_log_request_list_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("LOG_REQUEST_LIST") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_LOG_REQUEST_LIST) != NULL);
#endif
}

static void mavlink_test_log_entry(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LOG_ENTRY >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_log_entry_t packet_in = {
        963497464,963497672,17651,17755,17859
    };
    mavlink_log_entry_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_utc = packet_in.time_utc;
        packet1.size = packet_in.size;
        packet1.id = packet_in.id;
        packet1.num_logs = packet_in.num_logs;
        packet1.last_log_num = packet_in.last_log_num;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LOG_ENTRY_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LOG_ENTRY_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_log_entry_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_log_entry_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_log_entry_pack(system_id, component_id, &msg , packet1.id , packet1.num_logs , packet1.last_log_num , packet1.time_utc , packet1.size );
    mavlink_msg_log_entry_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_log_entry_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.id , packet1.num_logs , packet1.last_log_num , packet1.time_utc , packet1.size );
    mavlink_msg_log_entry_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_log_entry_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_log_entry_send(MAVLINK_COMM_1 , packet1.id , packet1.num_logs , packet1.last_log_num , packet1.time_utc , packet1.size );
    mavlink_msg_log_entry_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("LOG_ENTRY") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_LOG_ENTRY) != NULL);
#endif
}

static void mavlink_test_log_request_data(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LOG_REQUEST_DATA >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_log_request_data_t packet_in = {
        963497464,963497672,17651,163,230
    };
    mavlink_log_request_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.ofs = packet_in.ofs;
        packet1.count = packet_in.count;
        packet1.id = packet_in.id;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LOG_REQUEST_DATA_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LOG_REQUEST_DATA_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_log_request_data_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_log_request_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_log_request_data_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.id , packet1.ofs , packet1.count );
    mavlink_msg_log_request_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_log_request_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.id , packet1.ofs , packet1.count );
    mavlink_msg_log_request_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_log_request_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_log_request_data_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.id , packet1.ofs , packet1.count );
    mavlink_msg_log_request_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("LOG_REQUEST_DATA") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_LOG_REQUEST_DATA) != NULL);
#endif
}

static void mavlink_test_log_data(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LOG_DATA >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_log_data_t packet_in = {
        963497464,17443,151,{ 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51 }
    };
    mavlink_log_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.ofs = packet_in.ofs;
        packet1.id = packet_in.id;
        packet1.count = packet_in.count;
        
        mav_array_memcpy(packet1.data, packet_in.data, sizeof(uint8_t)*90);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LOG_DATA_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LOG_DATA_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_log_data_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_log_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_log_data_pack(system_id, component_id, &msg , packet1.id , packet1.ofs , packet1.count , packet1.data );
    mavlink_msg_log_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_log_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.id , packet1.ofs , packet1.count , packet1.data );
    mavlink_msg_log_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_log_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_log_data_send(MAVLINK_COMM_1 , packet1.id , packet1.ofs , packet1.count , packet1.data );
    mavlink_msg_log_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("LOG_DATA") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_LOG_DATA) != NULL);
#endif
}

static void mavlink_test_log_erase(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LOG_ERASE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_log_erase_t packet_in = {
        5,72
    };
    mavlink_log_erase_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LOG_ERASE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LOG_ERASE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_log_erase_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_log_erase_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_log_erase_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component );
    mavlink_msg_log_erase_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_log_erase_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component );
    mavlink_msg_log_erase_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_log_erase_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_log_erase_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component );
    mavlink_msg_log_erase_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("LOG_ERASE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_LOG_ERASE) != NULL);
#endif
}

static void mavlink_test_log_request_end(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LOG_REQUEST_END >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_log_request_end_t packet_in = {
        5,72
    };
    mavlink_log_request_end_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LOG_REQUEST_END_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LOG_REQUEST_END_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_log_request_end_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_log_request_end_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_log_request_end_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component );
    mavlink_msg_log_request_end_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_log_request_end_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component );
    mavlink_msg_log_request_end_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_log_request_end_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_log_request_end_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component );
    mavlink_msg_log_request_end_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("LOG_REQUEST_END") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_LOG_REQUEST_END) != NULL);
#endif
}

static void mavlink_test_gps_inject_data(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GPS_INJECT_DATA >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_gps_inject_data_t packet_in = {
        5,72,139,{ 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59 }
    };
    mavlink_gps_inject_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.len = packet_in.len;
        
        mav_array_memcpy(packet1.data, packet_in.data, sizeof(uint8_t)*110);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GPS_INJECT_DATA_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GPS_INJECT_DATA_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_inject_data_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_gps_inject_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_inject_data_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.len , packet1.data );
    mavlink_msg_gps_inject_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_inject_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.len , packet1.data );
    mavlink_msg_gps_inject_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_gps_inject_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_inject_data_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.len , packet1.data );
    mavlink_msg_gps_inject_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("GPS_INJECT_DATA") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_GPS_INJECT_DATA) != NULL);
#endif
}

static void mavlink_test_gps2_raw(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GPS2_RAW >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_gps2_raw_t packet_in = {
        93372036854775807ULL,963497880,963498088,963498296,963498504,18483,18587,18691,18795,101,168,235,19055,963499388,963499596,963499804,963500012,963500220
    };
    mavlink_gps2_raw_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        packet1.alt = packet_in.alt;
        packet1.dgps_age = packet_in.dgps_age;
        packet1.eph = packet_in.eph;
        packet1.epv = packet_in.epv;
        packet1.vel = packet_in.vel;
        packet1.cog = packet_in.cog;
        packet1.fix_type = packet_in.fix_type;
        packet1.satellites_visible = packet_in.satellites_visible;
        packet1.dgps_numch = packet_in.dgps_numch;
        packet1.yaw = packet_in.yaw;
        packet1.alt_ellipsoid = packet_in.alt_ellipsoid;
        packet1.h_acc = packet_in.h_acc;
        packet1.v_acc = packet_in.v_acc;
        packet1.vel_acc = packet_in.vel_acc;
        packet1.hdg_acc = packet_in.hdg_acc;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GPS2_RAW_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GPS2_RAW_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps2_raw_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_gps2_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps2_raw_pack(system_id, component_id, &msg , packet1.time_usec , packet1.fix_type , packet1.lat , packet1.lon , packet1.alt , packet1.eph , packet1.epv , packet1.vel , packet1.cog , packet1.satellites_visible , packet1.dgps_numch , packet1.dgps_age , packet1.yaw , packet1.alt_ellipsoid , packet1.h_acc , packet1.v_acc , packet1.vel_acc , packet1.hdg_acc );
    mavlink_msg_gps2_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps2_raw_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.fix_type , packet1.lat , packet1.lon , packet1.alt , packet1.eph , packet1.epv , packet1.vel , packet1.cog , packet1.satellites_visible , packet1.dgps_numch , packet1.dgps_age , packet1.yaw , packet1.alt_ellipsoid , packet1.h_acc , packet1.v_acc , packet1.vel_acc , packet1.hdg_acc );
    mavlink_msg_gps2_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_gps2_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps2_raw_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.fix_type , packet1.lat , packet1.lon , packet1.alt , packet1.eph , packet1.epv , packet1.vel , packet1.cog , packet1.satellites_visible , packet1.dgps_numch , packet1.dgps_age , packet1.yaw , packet1.alt_ellipsoid , packet1.h_acc , packet1.v_acc , packet1.vel_acc , packet1.hdg_acc );
    mavlink_msg_gps2_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("GPS2_RAW") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_GPS2_RAW) != NULL);
#endif
}

static void mavlink_test_power_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_POWER_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_power_status_t packet_in = {
        17235,17339,17443
    };
    mavlink_power_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.Vcc = packet_in.Vcc;
        packet1.Vservo = packet_in.Vservo;
        packet1.flags = packet_in.flags;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_POWER_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_POWER_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_power_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_power_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_power_status_pack(system_id, component_id, &msg , packet1.Vcc , packet1.Vservo , packet1.flags );
    mavlink_msg_power_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_power_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.Vcc , packet1.Vservo , packet1.flags );
    mavlink_msg_power_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_power_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_power_status_send(MAVLINK_COMM_1 , packet1.Vcc , packet1.Vservo , packet1.flags );
    mavlink_msg_power_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("POWER_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_POWER_STATUS) != NULL);
#endif
}

static void mavlink_test_serial_control(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SERIAL_CONTROL >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_serial_control_t packet_in = {
        963497464,17443,151,218,29,{ 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165 }
    };
    mavlink_serial_control_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.baudrate = packet_in.baudrate;
        packet1.timeout = packet_in.timeout;
        packet1.device = packet_in.device;
        packet1.flags = packet_in.flags;
        packet1.count = packet_in.count;
        
        mav_array_memcpy(packet1.data, packet_in.data, sizeof(uint8_t)*70);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SERIAL_CONTROL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SERIAL_CONTROL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_serial_control_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_serial_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_serial_control_pack(system_id, component_id, &msg , packet1.device , packet1.flags , packet1.timeout , packet1.baudrate , packet1.count , packet1.data );
    mavlink_msg_serial_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_serial_control_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.device , packet1.flags , packet1.timeout , packet1.baudrate , packet1.count , packet1.data );
    mavlink_msg_serial_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_serial_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_serial_control_send(MAVLINK_COMM_1 , packet1.device , packet1.flags , packet1.timeout , packet1.baudrate , packet1.count , packet1.data );
    mavlink_msg_serial_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SERIAL_CONTROL") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SERIAL_CONTROL) != NULL);
#endif
}

static void mavlink_test_gps_rtk(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GPS_RTK >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_gps_rtk_t packet_in = {
        963497464,963497672,963497880,963498088,963498296,963498504,963498712,18691,223,34,101,168,235
    };
    mavlink_gps_rtk_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_last_baseline_ms = packet_in.time_last_baseline_ms;
        packet1.tow = packet_in.tow;
        packet1.baseline_a_mm = packet_in.baseline_a_mm;
        packet1.baseline_b_mm = packet_in.baseline_b_mm;
        packet1.baseline_c_mm = packet_in.baseline_c_mm;
        packet1.accuracy = packet_in.accuracy;
        packet1.iar_num_hypotheses = packet_in.iar_num_hypotheses;
        packet1.wn = packet_in.wn;
        packet1.rtk_receiver_id = packet_in.rtk_receiver_id;
        packet1.rtk_health = packet_in.rtk_health;
        packet1.rtk_rate = packet_in.rtk_rate;
        packet1.nsats = packet_in.nsats;
        packet1.baseline_coords_type = packet_in.baseline_coords_type;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GPS_RTK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GPS_RTK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_rtk_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_gps_rtk_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_rtk_pack(system_id, component_id, &msg , packet1.time_last_baseline_ms , packet1.rtk_receiver_id , packet1.wn , packet1.tow , packet1.rtk_health , packet1.rtk_rate , packet1.nsats , packet1.baseline_coords_type , packet1.baseline_a_mm , packet1.baseline_b_mm , packet1.baseline_c_mm , packet1.accuracy , packet1.iar_num_hypotheses );
    mavlink_msg_gps_rtk_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_rtk_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_last_baseline_ms , packet1.rtk_receiver_id , packet1.wn , packet1.tow , packet1.rtk_health , packet1.rtk_rate , packet1.nsats , packet1.baseline_coords_type , packet1.baseline_a_mm , packet1.baseline_b_mm , packet1.baseline_c_mm , packet1.accuracy , packet1.iar_num_hypotheses );
    mavlink_msg_gps_rtk_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_gps_rtk_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_rtk_send(MAVLINK_COMM_1 , packet1.time_last_baseline_ms , packet1.rtk_receiver_id , packet1.wn , packet1.tow , packet1.rtk_health , packet1.rtk_rate , packet1.nsats , packet1.baseline_coords_type , packet1.baseline_a_mm , packet1.baseline_b_mm , packet1.baseline_c_mm , packet1.accuracy , packet1.iar_num_hypotheses );
    mavlink_msg_gps_rtk_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("GPS_RTK") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_GPS_RTK) != NULL);
#endif
}

static void mavlink_test_gps2_rtk(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GPS2_RTK >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_gps2_rtk_t packet_in = {
        963497464,963497672,963497880,963498088,963498296,963498504,963498712,18691,223,34,101,168,235
    };
    mavlink_gps2_rtk_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_last_baseline_ms = packet_in.time_last_baseline_ms;
        packet1.tow = packet_in.tow;
        packet1.baseline_a_mm = packet_in.baseline_a_mm;
        packet1.baseline_b_mm = packet_in.baseline_b_mm;
        packet1.baseline_c_mm = packet_in.baseline_c_mm;
        packet1.accuracy = packet_in.accuracy;
        packet1.iar_num_hypotheses = packet_in.iar_num_hypotheses;
        packet1.wn = packet_in.wn;
        packet1.rtk_receiver_id = packet_in.rtk_receiver_id;
        packet1.rtk_health = packet_in.rtk_health;
        packet1.rtk_rate = packet_in.rtk_rate;
        packet1.nsats = packet_in.nsats;
        packet1.baseline_coords_type = packet_in.baseline_coords_type;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GPS2_RTK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GPS2_RTK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps2_rtk_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_gps2_rtk_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps2_rtk_pack(system_id, component_id, &msg , packet1.time_last_baseline_ms , packet1.rtk_receiver_id , packet1.wn , packet1.tow , packet1.rtk_health , packet1.rtk_rate , packet1.nsats , packet1.baseline_coords_type , packet1.baseline_a_mm , packet1.baseline_b_mm , packet1.baseline_c_mm , packet1.accuracy , packet1.iar_num_hypotheses );
    mavlink_msg_gps2_rtk_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps2_rtk_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_last_baseline_ms , packet1.rtk_receiver_id , packet1.wn , packet1.tow , packet1.rtk_health , packet1.rtk_rate , packet1.nsats , packet1.baseline_coords_type , packet1.baseline_a_mm , packet1.baseline_b_mm , packet1.baseline_c_mm , packet1.accuracy , packet1.iar_num_hypotheses );
    mavlink_msg_gps2_rtk_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_gps2_rtk_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps2_rtk_send(MAVLINK_COMM_1 , packet1.time_last_baseline_ms , packet1.rtk_receiver_id , packet1.wn , packet1.tow , packet1.rtk_health , packet1.rtk_rate , packet1.nsats , packet1.baseline_coords_type , packet1.baseline_a_mm , packet1.baseline_b_mm , packet1.baseline_c_mm , packet1.accuracy , packet1.iar_num_hypotheses );
    mavlink_msg_gps2_rtk_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("GPS2_RTK") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_GPS2_RTK) != NULL);
#endif
}

static void mavlink_test_scaled_imu3(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SCALED_IMU3 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_scaled_imu3_t packet_in = {
        963497464,17443,17547,17651,17755,17859,17963,18067,18171,18275,18379
    };
    mavlink_scaled_imu3_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.xacc = packet_in.xacc;
        packet1.yacc = packet_in.yacc;
        packet1.zacc = packet_in.zacc;
        packet1.xgyro = packet_in.xgyro;
        packet1.ygyro = packet_in.ygyro;
        packet1.zgyro = packet_in.zgyro;
        packet1.xmag = packet_in.xmag;
        packet1.ymag = packet_in.ymag;
        packet1.zmag = packet_in.zmag;
        packet1.temperature = packet_in.temperature;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SCALED_IMU3_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SCALED_IMU3_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_scaled_imu3_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_scaled_imu3_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_scaled_imu3_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xmag , packet1.ymag , packet1.zmag , packet1.temperature );
    mavlink_msg_scaled_imu3_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_scaled_imu3_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xmag , packet1.ymag , packet1.zmag , packet1.temperature );
    mavlink_msg_scaled_imu3_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_scaled_imu3_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_scaled_imu3_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xmag , packet1.ymag , packet1.zmag , packet1.temperature );
    mavlink_msg_scaled_imu3_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SCALED_IMU3") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SCALED_IMU3) != NULL);
#endif
}

static void mavlink_test_data_transmission_handshake(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_data_transmission_handshake_t packet_in = {
        963497464,17443,17547,17651,163,230,41
    };
    mavlink_data_transmission_handshake_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.size = packet_in.size;
        packet1.width = packet_in.width;
        packet1.height = packet_in.height;
        packet1.packets = packet_in.packets;
        packet1.type = packet_in.type;
        packet1.payload = packet_in.payload;
        packet1.jpg_quality = packet_in.jpg_quality;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_data_transmission_handshake_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_data_transmission_handshake_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_data_transmission_handshake_pack(system_id, component_id, &msg , packet1.type , packet1.size , packet1.width , packet1.height , packet1.packets , packet1.payload , packet1.jpg_quality );
    mavlink_msg_data_transmission_handshake_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_data_transmission_handshake_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.type , packet1.size , packet1.width , packet1.height , packet1.packets , packet1.payload , packet1.jpg_quality );
    mavlink_msg_data_transmission_handshake_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_data_transmission_handshake_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_data_transmission_handshake_send(MAVLINK_COMM_1 , packet1.type , packet1.size , packet1.width , packet1.height , packet1.packets , packet1.payload , packet1.jpg_quality );
    mavlink_msg_data_transmission_handshake_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("DATA_TRANSMISSION_HANDSHAKE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE) != NULL);
#endif
}

static void mavlink_test_encapsulated_data(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ENCAPSULATED_DATA >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_encapsulated_data_t packet_in = {
        17235,{ 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135 }
    };
    mavlink_encapsulated_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.seqnr = packet_in.seqnr;
        
        mav_array_memcpy(packet1.data, packet_in.data, sizeof(uint8_t)*253);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ENCAPSULATED_DATA_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ENCAPSULATED_DATA_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_encapsulated_data_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_encapsulated_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_encapsulated_data_pack(system_id, component_id, &msg , packet1.seqnr , packet1.data );
    mavlink_msg_encapsulated_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_encapsulated_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.seqnr , packet1.data );
    mavlink_msg_encapsulated_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_encapsulated_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_encapsulated_data_send(MAVLINK_COMM_1 , packet1.seqnr , packet1.data );
    mavlink_msg_encapsulated_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ENCAPSULATED_DATA") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ENCAPSULATED_DATA) != NULL);
#endif
}

static void mavlink_test_distance_sensor(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DISTANCE_SENSOR >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_distance_sensor_t packet_in = {
        963497464,17443,17547,17651,163,230,41,108,115.0,143.0,{ 171.0, 172.0, 173.0, 174.0 },247
    };
    mavlink_distance_sensor_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.min_distance = packet_in.min_distance;
        packet1.max_distance = packet_in.max_distance;
        packet1.current_distance = packet_in.current_distance;
        packet1.type = packet_in.type;
        packet1.id = packet_in.id;
        packet1.orientation = packet_in.orientation;
        packet1.covariance = packet_in.covariance;
        packet1.horizontal_fov = packet_in.horizontal_fov;
        packet1.vertical_fov = packet_in.vertical_fov;
        packet1.signal_quality = packet_in.signal_quality;
        
        mav_array_memcpy(packet1.quaternion, packet_in.quaternion, sizeof(float)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DISTANCE_SENSOR_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DISTANCE_SENSOR_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_distance_sensor_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_distance_sensor_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_distance_sensor_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.min_distance , packet1.max_distance , packet1.current_distance , packet1.type , packet1.id , packet1.orientation , packet1.covariance , packet1.horizontal_fov , packet1.vertical_fov , packet1.quaternion , packet1.signal_quality );
    mavlink_msg_distance_sensor_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_distance_sensor_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.min_distance , packet1.max_distance , packet1.current_distance , packet1.type , packet1.id , packet1.orientation , packet1.covariance , packet1.horizontal_fov , packet1.vertical_fov , packet1.quaternion , packet1.signal_quality );
    mavlink_msg_distance_sensor_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_distance_sensor_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_distance_sensor_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.min_distance , packet1.max_distance , packet1.current_distance , packet1.type , packet1.id , packet1.orientation , packet1.covariance , packet1.horizontal_fov , packet1.vertical_fov , packet1.quaternion , packet1.signal_quality );
    mavlink_msg_distance_sensor_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("DISTANCE_SENSOR") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_DISTANCE_SENSOR) != NULL);
#endif
}

static void mavlink_test_terrain_request(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TERRAIN_REQUEST >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_terrain_request_t packet_in = {
        93372036854775807ULL,963497880,963498088,18067
    };
    mavlink_terrain_request_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.mask = packet_in.mask;
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        packet1.grid_spacing = packet_in.grid_spacing;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TERRAIN_REQUEST_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TERRAIN_REQUEST_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_terrain_request_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_terrain_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_terrain_request_pack(system_id, component_id, &msg , packet1.lat , packet1.lon , packet1.grid_spacing , packet1.mask );
    mavlink_msg_terrain_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_terrain_request_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.lat , packet1.lon , packet1.grid_spacing , packet1.mask );
    mavlink_msg_terrain_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_terrain_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_terrain_request_send(MAVLINK_COMM_1 , packet1.lat , packet1.lon , packet1.grid_spacing , packet1.mask );
    mavlink_msg_terrain_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("TERRAIN_REQUEST") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_TERRAIN_REQUEST) != NULL);
#endif
}

static void mavlink_test_terrain_data(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TERRAIN_DATA >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_terrain_data_t packet_in = {
        963497464,963497672,17651,{ 17755, 17756, 17757, 17758, 17759, 17760, 17761, 17762, 17763, 17764, 17765, 17766, 17767, 17768, 17769, 17770 },3
    };
    mavlink_terrain_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        packet1.grid_spacing = packet_in.grid_spacing;
        packet1.gridbit = packet_in.gridbit;
        
        mav_array_memcpy(packet1.data, packet_in.data, sizeof(int16_t)*16);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TERRAIN_DATA_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TERRAIN_DATA_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_terrain_data_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_terrain_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_terrain_data_pack(system_id, component_id, &msg , packet1.lat , packet1.lon , packet1.grid_spacing , packet1.gridbit , packet1.data );
    mavlink_msg_terrain_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_terrain_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.lat , packet1.lon , packet1.grid_spacing , packet1.gridbit , packet1.data );
    mavlink_msg_terrain_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_terrain_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_terrain_data_send(MAVLINK_COMM_1 , packet1.lat , packet1.lon , packet1.grid_spacing , packet1.gridbit , packet1.data );
    mavlink_msg_terrain_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("TERRAIN_DATA") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_TERRAIN_DATA) != NULL);
#endif
}

static void mavlink_test_terrain_check(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TERRAIN_CHECK >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_terrain_check_t packet_in = {
        963497464,963497672
    };
    mavlink_terrain_check_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TERRAIN_CHECK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TERRAIN_CHECK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_terrain_check_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_terrain_check_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_terrain_check_pack(system_id, component_id, &msg , packet1.lat , packet1.lon );
    mavlink_msg_terrain_check_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_terrain_check_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.lat , packet1.lon );
    mavlink_msg_terrain_check_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_terrain_check_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_terrain_check_send(MAVLINK_COMM_1 , packet1.lat , packet1.lon );
    mavlink_msg_terrain_check_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("TERRAIN_CHECK") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_TERRAIN_CHECK) != NULL);
#endif
}

static void mavlink_test_terrain_report(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TERRAIN_REPORT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_terrain_report_t packet_in = {
        963497464,963497672,73.0,101.0,18067,18171,18275
    };
    mavlink_terrain_report_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        packet1.terrain_height = packet_in.terrain_height;
        packet1.current_height = packet_in.current_height;
        packet1.spacing = packet_in.spacing;
        packet1.pending = packet_in.pending;
        packet1.loaded = packet_in.loaded;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TERRAIN_REPORT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TERRAIN_REPORT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_terrain_report_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_terrain_report_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_terrain_report_pack(system_id, component_id, &msg , packet1.lat , packet1.lon , packet1.spacing , packet1.terrain_height , packet1.current_height , packet1.pending , packet1.loaded );
    mavlink_msg_terrain_report_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_terrain_report_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.lat , packet1.lon , packet1.spacing , packet1.terrain_height , packet1.current_height , packet1.pending , packet1.loaded );
    mavlink_msg_terrain_report_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_terrain_report_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_terrain_report_send(MAVLINK_COMM_1 , packet1.lat , packet1.lon , packet1.spacing , packet1.terrain_height , packet1.current_height , packet1.pending , packet1.loaded );
    mavlink_msg_terrain_report_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("TERRAIN_REPORT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_TERRAIN_REPORT) != NULL);
#endif
}

static void mavlink_test_scaled_pressure2(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SCALED_PRESSURE2 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_scaled_pressure2_t packet_in = {
        963497464,45.0,73.0,17859,17963
    };
    mavlink_scaled_pressure2_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.press_abs = packet_in.press_abs;
        packet1.press_diff = packet_in.press_diff;
        packet1.temperature = packet_in.temperature;
        packet1.temperature_press_diff = packet_in.temperature_press_diff;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SCALED_PRESSURE2_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SCALED_PRESSURE2_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_scaled_pressure2_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_scaled_pressure2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_scaled_pressure2_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.press_abs , packet1.press_diff , packet1.temperature , packet1.temperature_press_diff );
    mavlink_msg_scaled_pressure2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_scaled_pressure2_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.press_abs , packet1.press_diff , packet1.temperature , packet1.temperature_press_diff );
    mavlink_msg_scaled_pressure2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_scaled_pressure2_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_scaled_pressure2_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.press_abs , packet1.press_diff , packet1.temperature , packet1.temperature_press_diff );
    mavlink_msg_scaled_pressure2_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SCALED_PRESSURE2") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SCALED_PRESSURE2) != NULL);
#endif
}

static void mavlink_test_att_pos_mocap(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ATT_POS_MOCAP >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_att_pos_mocap_t packet_in = {
        93372036854775807ULL,{ 73.0, 74.0, 75.0, 76.0 },185.0,213.0,241.0,{ 269.0, 270.0, 271.0, 272.0, 273.0, 274.0, 275.0, 276.0, 277.0, 278.0, 279.0, 280.0, 281.0, 282.0, 283.0, 284.0, 285.0, 286.0, 287.0, 288.0, 289.0 }
    };
    mavlink_att_pos_mocap_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        
        mav_array_memcpy(packet1.q, packet_in.q, sizeof(float)*4);
        mav_array_memcpy(packet1.covariance, packet_in.covariance, sizeof(float)*21);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ATT_POS_MOCAP_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ATT_POS_MOCAP_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_att_pos_mocap_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_att_pos_mocap_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_att_pos_mocap_pack(system_id, component_id, &msg , packet1.time_usec , packet1.q , packet1.x , packet1.y , packet1.z , packet1.covariance );
    mavlink_msg_att_pos_mocap_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_att_pos_mocap_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.q , packet1.x , packet1.y , packet1.z , packet1.covariance );
    mavlink_msg_att_pos_mocap_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_att_pos_mocap_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_att_pos_mocap_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.q , packet1.x , packet1.y , packet1.z , packet1.covariance );
    mavlink_msg_att_pos_mocap_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ATT_POS_MOCAP") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ATT_POS_MOCAP) != NULL);
#endif
}

static void mavlink_test_set_actuator_control_target(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_actuator_control_target_t packet_in = {
        93372036854775807ULL,{ 73.0, 74.0, 75.0, 76.0, 77.0, 78.0, 79.0, 80.0 },125,192,3
    };
    mavlink_set_actuator_control_target_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.group_mlx = packet_in.group_mlx;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        mav_array_memcpy(packet1.controls, packet_in.controls, sizeof(float)*8);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_actuator_control_target_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_actuator_control_target_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_actuator_control_target_pack(system_id, component_id, &msg , packet1.time_usec , packet1.group_mlx , packet1.target_system , packet1.target_component , packet1.controls );
    mavlink_msg_set_actuator_control_target_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_actuator_control_target_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.group_mlx , packet1.target_system , packet1.target_component , packet1.controls );
    mavlink_msg_set_actuator_control_target_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_actuator_control_target_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_actuator_control_target_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.group_mlx , packet1.target_system , packet1.target_component , packet1.controls );
    mavlink_msg_set_actuator_control_target_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SET_ACTUATOR_CONTROL_TARGET") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET) != NULL);
#endif
}

static void mavlink_test_actuator_control_target(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_actuator_control_target_t packet_in = {
        93372036854775807ULL,{ 73.0, 74.0, 75.0, 76.0, 77.0, 78.0, 79.0, 80.0 },125
    };
    mavlink_actuator_control_target_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.group_mlx = packet_in.group_mlx;
        
        mav_array_memcpy(packet1.controls, packet_in.controls, sizeof(float)*8);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_actuator_control_target_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_actuator_control_target_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_actuator_control_target_pack(system_id, component_id, &msg , packet1.time_usec , packet1.group_mlx , packet1.controls );
    mavlink_msg_actuator_control_target_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_actuator_control_target_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.group_mlx , packet1.controls );
    mavlink_msg_actuator_control_target_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_actuator_control_target_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_actuator_control_target_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.group_mlx , packet1.controls );
    mavlink_msg_actuator_control_target_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ACTUATOR_CONTROL_TARGET") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET) != NULL);
#endif
}

static void mavlink_test_altitude(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ALTITUDE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_altitude_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0
    };
    mavlink_altitude_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.altitude_monotonic = packet_in.altitude_monotonic;
        packet1.altitude_amsl = packet_in.altitude_amsl;
        packet1.altitude_local = packet_in.altitude_local;
        packet1.altitude_relative = packet_in.altitude_relative;
        packet1.altitude_terrain = packet_in.altitude_terrain;
        packet1.bottom_clearance = packet_in.bottom_clearance;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ALTITUDE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ALTITUDE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_altitude_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_altitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_altitude_pack(system_id, component_id, &msg , packet1.time_usec , packet1.altitude_monotonic , packet1.altitude_amsl , packet1.altitude_local , packet1.altitude_relative , packet1.altitude_terrain , packet1.bottom_clearance );
    mavlink_msg_altitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_altitude_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.altitude_monotonic , packet1.altitude_amsl , packet1.altitude_local , packet1.altitude_relative , packet1.altitude_terrain , packet1.bottom_clearance );
    mavlink_msg_altitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_altitude_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_altitude_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.altitude_monotonic , packet1.altitude_amsl , packet1.altitude_local , packet1.altitude_relative , packet1.altitude_terrain , packet1.bottom_clearance );
    mavlink_msg_altitude_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ALTITUDE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ALTITUDE) != NULL);
#endif
}

static void mavlink_test_resource_request(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RESOURCE_REQUEST >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_resource_request_t packet_in = {
        5,72,{ 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2 },243,{ 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173 }
    };
    mavlink_resource_request_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.request_id = packet_in.request_id;
        packet1.uri_type = packet_in.uri_type;
        packet1.transfer_type = packet_in.transfer_type;
        
        mav_array_memcpy(packet1.uri, packet_in.uri, sizeof(uint8_t)*120);
        mav_array_memcpy(packet1.storage, packet_in.storage, sizeof(uint8_t)*120);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_RESOURCE_REQUEST_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RESOURCE_REQUEST_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_resource_request_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_resource_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_resource_request_pack(system_id, component_id, &msg , packet1.request_id , packet1.uri_type , packet1.uri , packet1.transfer_type , packet1.storage );
    mavlink_msg_resource_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_resource_request_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.request_id , packet1.uri_type , packet1.uri , packet1.transfer_type , packet1.storage );
    mavlink_msg_resource_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_resource_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_resource_request_send(MAVLINK_COMM_1 , packet1.request_id , packet1.uri_type , packet1.uri , packet1.transfer_type , packet1.storage );
    mavlink_msg_resource_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("RESOURCE_REQUEST") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_RESOURCE_REQUEST) != NULL);
#endif
}

static void mavlink_test_scaled_pressure3(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SCALED_PRESSURE3 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_scaled_pressure3_t packet_in = {
        963497464,45.0,73.0,17859,17963
    };
    mavlink_scaled_pressure3_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.press_abs = packet_in.press_abs;
        packet1.press_diff = packet_in.press_diff;
        packet1.temperature = packet_in.temperature;
        packet1.temperature_press_diff = packet_in.temperature_press_diff;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SCALED_PRESSURE3_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SCALED_PRESSURE3_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_scaled_pressure3_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_scaled_pressure3_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_scaled_pressure3_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.press_abs , packet1.press_diff , packet1.temperature , packet1.temperature_press_diff );
    mavlink_msg_scaled_pressure3_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_scaled_pressure3_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.press_abs , packet1.press_diff , packet1.temperature , packet1.temperature_press_diff );
    mavlink_msg_scaled_pressure3_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_scaled_pressure3_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_scaled_pressure3_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.press_abs , packet1.press_diff , packet1.temperature , packet1.temperature_press_diff );
    mavlink_msg_scaled_pressure3_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SCALED_PRESSURE3") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SCALED_PRESSURE3) != NULL);
#endif
}

static void mavlink_test_follow_target(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_FOLLOW_TARGET >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_follow_target_t packet_in = {
        93372036854775807ULL,93372036854776311ULL,963498296,963498504,185.0,{ 213.0, 214.0, 215.0 },{ 297.0, 298.0, 299.0 },{ 381.0, 382.0, 383.0, 384.0 },{ 493.0, 494.0, 495.0 },{ 577.0, 578.0, 579.0 },25
    };
    mavlink_follow_target_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.custom_state = packet_in.custom_state;
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        packet1.alt = packet_in.alt;
        packet1.est_capabilities = packet_in.est_capabilities;
        
        mav_array_memcpy(packet1.vel, packet_in.vel, sizeof(float)*3);
        mav_array_memcpy(packet1.acc, packet_in.acc, sizeof(float)*3);
        mav_array_memcpy(packet1.attitude_q, packet_in.attitude_q, sizeof(float)*4);
        mav_array_memcpy(packet1.rates, packet_in.rates, sizeof(float)*3);
        mav_array_memcpy(packet1.position_cov, packet_in.position_cov, sizeof(float)*3);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_FOLLOW_TARGET_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_FOLLOW_TARGET_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_follow_target_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_follow_target_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_follow_target_pack(system_id, component_id, &msg , packet1.timestamp , packet1.est_capabilities , packet1.lat , packet1.lon , packet1.alt , packet1.vel , packet1.acc , packet1.attitude_q , packet1.rates , packet1.position_cov , packet1.custom_state );
    mavlink_msg_follow_target_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_follow_target_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.est_capabilities , packet1.lat , packet1.lon , packet1.alt , packet1.vel , packet1.acc , packet1.attitude_q , packet1.rates , packet1.position_cov , packet1.custom_state );
    mavlink_msg_follow_target_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_follow_target_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_follow_target_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.est_capabilities , packet1.lat , packet1.lon , packet1.alt , packet1.vel , packet1.acc , packet1.attitude_q , packet1.rates , packet1.position_cov , packet1.custom_state );
    mavlink_msg_follow_target_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("FOLLOW_TARGET") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_FOLLOW_TARGET) != NULL);
#endif
}

static void mavlink_test_control_system_state(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CONTROL_SYSTEM_STATE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_control_system_state_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,{ 353.0, 354.0, 355.0 },{ 437.0, 438.0, 439.0 },{ 521.0, 522.0, 523.0, 524.0 },633.0,661.0,689.0
    };
    mavlink_control_system_state_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.x_acc = packet_in.x_acc;
        packet1.y_acc = packet_in.y_acc;
        packet1.z_acc = packet_in.z_acc;
        packet1.x_vel = packet_in.x_vel;
        packet1.y_vel = packet_in.y_vel;
        packet1.z_vel = packet_in.z_vel;
        packet1.x_pos = packet_in.x_pos;
        packet1.y_pos = packet_in.y_pos;
        packet1.z_pos = packet_in.z_pos;
        packet1.airspeed = packet_in.airspeed;
        packet1.roll_rate = packet_in.roll_rate;
        packet1.pitch_rate = packet_in.pitch_rate;
        packet1.yaw_rate = packet_in.yaw_rate;
        
        mav_array_memcpy(packet1.vel_variance, packet_in.vel_variance, sizeof(float)*3);
        mav_array_memcpy(packet1.pos_variance, packet_in.pos_variance, sizeof(float)*3);
        mav_array_memcpy(packet1.q, packet_in.q, sizeof(float)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CONTROL_SYSTEM_STATE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CONTROL_SYSTEM_STATE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_control_system_state_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_control_system_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_control_system_state_pack(system_id, component_id, &msg , packet1.time_usec , packet1.x_acc , packet1.y_acc , packet1.z_acc , packet1.x_vel , packet1.y_vel , packet1.z_vel , packet1.x_pos , packet1.y_pos , packet1.z_pos , packet1.airspeed , packet1.vel_variance , packet1.pos_variance , packet1.q , packet1.roll_rate , packet1.pitch_rate , packet1.yaw_rate );
    mavlink_msg_control_system_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_control_system_state_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.x_acc , packet1.y_acc , packet1.z_acc , packet1.x_vel , packet1.y_vel , packet1.z_vel , packet1.x_pos , packet1.y_pos , packet1.z_pos , packet1.airspeed , packet1.vel_variance , packet1.pos_variance , packet1.q , packet1.roll_rate , packet1.pitch_rate , packet1.yaw_rate );
    mavlink_msg_control_system_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_control_system_state_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_control_system_state_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.x_acc , packet1.y_acc , packet1.z_acc , packet1.x_vel , packet1.y_vel , packet1.z_vel , packet1.x_pos , packet1.y_pos , packet1.z_pos , packet1.airspeed , packet1.vel_variance , packet1.pos_variance , packet1.q , packet1.roll_rate , packet1.pitch_rate , packet1.yaw_rate );
    mavlink_msg_control_system_state_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CONTROL_SYSTEM_STATE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CONTROL_SYSTEM_STATE) != NULL);
#endif
}

static void mavlink_test_battery_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_BATTERY_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_battery_status_t packet_in = {
        963497464,963497672,17651,{ 17755, 17756, 17757, 17758, 17759, 17760, 17761, 17762, 17763, 17764 },18795,101,168,235,46,963499336,125,{ 19367, 19368, 19369, 19370 },216,963500064
    };
    mavlink_battery_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.current_consumed = packet_in.current_consumed;
        packet1.energy_consumed = packet_in.energy_consumed;
        packet1.temperature = packet_in.temperature;
        packet1.current_battery = packet_in.current_battery;
        packet1.id = packet_in.id;
        packet1.battery_function = packet_in.battery_function;
        packet1.type = packet_in.type;
        packet1.battery_remaining = packet_in.battery_remaining;
        packet1.time_remaining = packet_in.time_remaining;
        packet1.charge_state = packet_in.charge_state;
        packet1.mode = packet_in.mode;
        packet1.fault_bitmask = packet_in.fault_bitmask;
        
        mav_array_memcpy(packet1.voltages, packet_in.voltages, sizeof(uint16_t)*10);
        mav_array_memcpy(packet1.voltages_ext, packet_in.voltages_ext, sizeof(uint16_t)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_BATTERY_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_BATTERY_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_battery_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_battery_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_battery_status_pack(system_id, component_id, &msg , packet1.id , packet1.battery_function , packet1.type , packet1.temperature , packet1.voltages , packet1.current_battery , packet1.current_consumed , packet1.energy_consumed , packet1.battery_remaining , packet1.time_remaining , packet1.charge_state , packet1.voltages_ext , packet1.mode , packet1.fault_bitmask );
    mavlink_msg_battery_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_battery_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.id , packet1.battery_function , packet1.type , packet1.temperature , packet1.voltages , packet1.current_battery , packet1.current_consumed , packet1.energy_consumed , packet1.battery_remaining , packet1.time_remaining , packet1.charge_state , packet1.voltages_ext , packet1.mode , packet1.fault_bitmask );
    mavlink_msg_battery_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_battery_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_battery_status_send(MAVLINK_COMM_1 , packet1.id , packet1.battery_function , packet1.type , packet1.temperature , packet1.voltages , packet1.current_battery , packet1.current_consumed , packet1.energy_consumed , packet1.battery_remaining , packet1.time_remaining , packet1.charge_state , packet1.voltages_ext , packet1.mode , packet1.fault_bitmask );
    mavlink_msg_battery_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("BATTERY_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_BATTERY_STATUS) != NULL);
#endif
}

static void mavlink_test_autopilot_version(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_AUTOPILOT_VERSION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_autopilot_version_t packet_in = {
        93372036854775807ULL,93372036854776311ULL,963498296,963498504,963498712,963498920,18899,19003,{ 113, 114, 115, 116, 117, 118, 119, 120 },{ 137, 138, 139, 140, 141, 142, 143, 144 },{ 161, 162, 163, 164, 165, 166, 167, 168 },{ 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202 }
    };
    mavlink_autopilot_version_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.capabilities = packet_in.capabilities;
        packet1.uid = packet_in.uid;
        packet1.flight_sw_version = packet_in.flight_sw_version;
        packet1.middleware_sw_version = packet_in.middleware_sw_version;
        packet1.os_sw_version = packet_in.os_sw_version;
        packet1.board_version = packet_in.board_version;
        packet1.vendor_id = packet_in.vendor_id;
        packet1.product_id = packet_in.product_id;
        
        mav_array_memcpy(packet1.flight_custom_version, packet_in.flight_custom_version, sizeof(uint8_t)*8);
        mav_array_memcpy(packet1.middleware_custom_version, packet_in.middleware_custom_version, sizeof(uint8_t)*8);
        mav_array_memcpy(packet1.os_custom_version, packet_in.os_custom_version, sizeof(uint8_t)*8);
        mav_array_memcpy(packet1.uid2, packet_in.uid2, sizeof(uint8_t)*18);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_AUTOPILOT_VERSION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_AUTOPILOT_VERSION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_autopilot_version_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_autopilot_version_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_autopilot_version_pack(system_id, component_id, &msg , packet1.capabilities , packet1.flight_sw_version , packet1.middleware_sw_version , packet1.os_sw_version , packet1.board_version , packet1.flight_custom_version , packet1.middleware_custom_version , packet1.os_custom_version , packet1.vendor_id , packet1.product_id , packet1.uid , packet1.uid2 );
    mavlink_msg_autopilot_version_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_autopilot_version_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.capabilities , packet1.flight_sw_version , packet1.middleware_sw_version , packet1.os_sw_version , packet1.board_version , packet1.flight_custom_version , packet1.middleware_custom_version , packet1.os_custom_version , packet1.vendor_id , packet1.product_id , packet1.uid , packet1.uid2 );
    mavlink_msg_autopilot_version_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_autopilot_version_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_autopilot_version_send(MAVLINK_COMM_1 , packet1.capabilities , packet1.flight_sw_version , packet1.middleware_sw_version , packet1.os_sw_version , packet1.board_version , packet1.flight_custom_version , packet1.middleware_custom_version , packet1.os_custom_version , packet1.vendor_id , packet1.product_id , packet1.uid , packet1.uid2 );
    mavlink_msg_autopilot_version_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("AUTOPILOT_VERSION") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_AUTOPILOT_VERSION) != NULL);
#endif
}

static void mavlink_test_landing_target(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LANDING_TARGET >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_landing_target_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,89,156,227.0,255.0,283.0,{ 311.0, 312.0, 313.0, 314.0 },51,118
    };
    mavlink_landing_target_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.angle_x = packet_in.angle_x;
        packet1.angle_y = packet_in.angle_y;
        packet1.distance = packet_in.distance;
        packet1.size_x = packet_in.size_x;
        packet1.size_y = packet_in.size_y;
        packet1.target_num = packet_in.target_num;
        packet1.frame = packet_in.frame;
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.type = packet_in.type;
        packet1.position_valid = packet_in.position_valid;
        
        mav_array_memcpy(packet1.q, packet_in.q, sizeof(float)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LANDING_TARGET_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LANDING_TARGET_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_landing_target_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_landing_target_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_landing_target_pack(system_id, component_id, &msg , packet1.time_usec , packet1.target_num , packet1.frame , packet1.angle_x , packet1.angle_y , packet1.distance , packet1.size_x , packet1.size_y , packet1.x , packet1.y , packet1.z , packet1.q , packet1.type , packet1.position_valid );
    mavlink_msg_landing_target_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_landing_target_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.target_num , packet1.frame , packet1.angle_x , packet1.angle_y , packet1.distance , packet1.size_x , packet1.size_y , packet1.x , packet1.y , packet1.z , packet1.q , packet1.type , packet1.position_valid );
    mavlink_msg_landing_target_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_landing_target_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_landing_target_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.target_num , packet1.frame , packet1.angle_x , packet1.angle_y , packet1.distance , packet1.size_x , packet1.size_y , packet1.x , packet1.y , packet1.z , packet1.q , packet1.type , packet1.position_valid );
    mavlink_msg_landing_target_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("LANDING_TARGET") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_LANDING_TARGET) != NULL);
#endif
}

static void mavlink_test_fence_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_FENCE_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_fence_status_t packet_in = {
        963497464,17443,151,218,29
    };
    mavlink_fence_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.breach_time = packet_in.breach_time;
        packet1.breach_count = packet_in.breach_count;
        packet1.breach_status = packet_in.breach_status;
        packet1.breach_type = packet_in.breach_type;
        packet1.breach_mitigation = packet_in.breach_mitigation;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_FENCE_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_FENCE_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_fence_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_fence_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_fence_status_pack(system_id, component_id, &msg , packet1.breach_status , packet1.breach_count , packet1.breach_type , packet1.breach_time , packet1.breach_mitigation );
    mavlink_msg_fence_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_fence_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.breach_status , packet1.breach_count , packet1.breach_type , packet1.breach_time , packet1.breach_mitigation );
    mavlink_msg_fence_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_fence_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_fence_status_send(MAVLINK_COMM_1 , packet1.breach_status , packet1.breach_count , packet1.breach_type , packet1.breach_time , packet1.breach_mitigation );
    mavlink_msg_fence_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("FENCE_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_FENCE_STATUS) != NULL);
#endif
}

static void mavlink_test_mag_cal_report(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MAG_CAL_REPORT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mag_cal_report_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,125,192,3,70,325.0,149,216,367.0
    };
    mavlink_mag_cal_report_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.fitness = packet_in.fitness;
        packet1.ofs_x = packet_in.ofs_x;
        packet1.ofs_y = packet_in.ofs_y;
        packet1.ofs_z = packet_in.ofs_z;
        packet1.diag_x = packet_in.diag_x;
        packet1.diag_y = packet_in.diag_y;
        packet1.diag_z = packet_in.diag_z;
        packet1.offdiag_x = packet_in.offdiag_x;
        packet1.offdiag_y = packet_in.offdiag_y;
        packet1.offdiag_z = packet_in.offdiag_z;
        packet1.compass_id = packet_in.compass_id;
        packet1.cal_mask = packet_in.cal_mask;
        packet1.cal_status = packet_in.cal_status;
        packet1.autosaved = packet_in.autosaved;
        packet1.orientation_confidence = packet_in.orientation_confidence;
        packet1.old_orientation = packet_in.old_orientation;
        packet1.new_orientation = packet_in.new_orientation;
        packet1.scale_factor = packet_in.scale_factor;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MAG_CAL_REPORT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MAG_CAL_REPORT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mag_cal_report_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mag_cal_report_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mag_cal_report_pack(system_id, component_id, &msg , packet1.compass_id , packet1.cal_mask , packet1.cal_status , packet1.autosaved , packet1.fitness , packet1.ofs_x , packet1.ofs_y , packet1.ofs_z , packet1.diag_x , packet1.diag_y , packet1.diag_z , packet1.offdiag_x , packet1.offdiag_y , packet1.offdiag_z , packet1.orientation_confidence , packet1.old_orientation , packet1.new_orientation , packet1.scale_factor );
    mavlink_msg_mag_cal_report_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mag_cal_report_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.compass_id , packet1.cal_mask , packet1.cal_status , packet1.autosaved , packet1.fitness , packet1.ofs_x , packet1.ofs_y , packet1.ofs_z , packet1.diag_x , packet1.diag_y , packet1.diag_z , packet1.offdiag_x , packet1.offdiag_y , packet1.offdiag_z , packet1.orientation_confidence , packet1.old_orientation , packet1.new_orientation , packet1.scale_factor );
    mavlink_msg_mag_cal_report_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mag_cal_report_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mag_cal_report_send(MAVLINK_COMM_1 , packet1.compass_id , packet1.cal_mask , packet1.cal_status , packet1.autosaved , packet1.fitness , packet1.ofs_x , packet1.ofs_y , packet1.ofs_z , packet1.diag_x , packet1.diag_y , packet1.diag_z , packet1.offdiag_x , packet1.offdiag_y , packet1.offdiag_z , packet1.orientation_confidence , packet1.old_orientation , packet1.new_orientation , packet1.scale_factor );
    mavlink_msg_mag_cal_report_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MAG_CAL_REPORT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MAG_CAL_REPORT) != NULL);
#endif
}

static void mavlink_test_efi_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_EFI_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_efi_status_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0,409.0,437.0,197,472.0,500.0
    };
    mavlink_efi_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.ecu_index = packet_in.ecu_index;
        packet1.rpm = packet_in.rpm;
        packet1.fuel_consumed = packet_in.fuel_consumed;
        packet1.fuel_flow = packet_in.fuel_flow;
        packet1.engine_load = packet_in.engine_load;
        packet1.throttle_position = packet_in.throttle_position;
        packet1.spark_dwell_time = packet_in.spark_dwell_time;
        packet1.barometric_pressure = packet_in.barometric_pressure;
        packet1.intake_manifold_pressure = packet_in.intake_manifold_pressure;
        packet1.intake_manifold_temperature = packet_in.intake_manifold_temperature;
        packet1.cylinder_head_temperature = packet_in.cylinder_head_temperature;
        packet1.ignition_timing = packet_in.ignition_timing;
        packet1.injection_time = packet_in.injection_time;
        packet1.exhaust_gas_temperature = packet_in.exhaust_gas_temperature;
        packet1.throttle_out = packet_in.throttle_out;
        packet1.pt_compensation = packet_in.pt_compensation;
        packet1.health = packet_in.health;
        packet1.ignition_voltage = packet_in.ignition_voltage;
        packet1.fuel_pressure = packet_in.fuel_pressure;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_EFI_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_EFI_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_efi_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_efi_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_efi_status_pack(system_id, component_id, &msg , packet1.health , packet1.ecu_index , packet1.rpm , packet1.fuel_consumed , packet1.fuel_flow , packet1.engine_load , packet1.throttle_position , packet1.spark_dwell_time , packet1.barometric_pressure , packet1.intake_manifold_pressure , packet1.intake_manifold_temperature , packet1.cylinder_head_temperature , packet1.ignition_timing , packet1.injection_time , packet1.exhaust_gas_temperature , packet1.throttle_out , packet1.pt_compensation , packet1.ignition_voltage , packet1.fuel_pressure );
    mavlink_msg_efi_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_efi_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.health , packet1.ecu_index , packet1.rpm , packet1.fuel_consumed , packet1.fuel_flow , packet1.engine_load , packet1.throttle_position , packet1.spark_dwell_time , packet1.barometric_pressure , packet1.intake_manifold_pressure , packet1.intake_manifold_temperature , packet1.cylinder_head_temperature , packet1.ignition_timing , packet1.injection_time , packet1.exhaust_gas_temperature , packet1.throttle_out , packet1.pt_compensation , packet1.ignition_voltage , packet1.fuel_pressure );
    mavlink_msg_efi_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_efi_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_efi_status_send(MAVLINK_COMM_1 , packet1.health , packet1.ecu_index , packet1.rpm , packet1.fuel_consumed , packet1.fuel_flow , packet1.engine_load , packet1.throttle_position , packet1.spark_dwell_time , packet1.barometric_pressure , packet1.intake_manifold_pressure , packet1.intake_manifold_temperature , packet1.cylinder_head_temperature , packet1.ignition_timing , packet1.injection_time , packet1.exhaust_gas_temperature , packet1.throttle_out , packet1.pt_compensation , packet1.ignition_voltage , packet1.fuel_pressure );
    mavlink_msg_efi_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("EFI_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_EFI_STATUS) != NULL);
#endif
}

static void mavlink_test_estimator_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ESTIMATOR_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_estimator_status_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,19315
    };
    mavlink_estimator_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.vel_ratio = packet_in.vel_ratio;
        packet1.pos_horiz_ratio = packet_in.pos_horiz_ratio;
        packet1.pos_vert_ratio = packet_in.pos_vert_ratio;
        packet1.mag_ratio = packet_in.mag_ratio;
        packet1.hagl_ratio = packet_in.hagl_ratio;
        packet1.tas_ratio = packet_in.tas_ratio;
        packet1.pos_horiz_accuracy = packet_in.pos_horiz_accuracy;
        packet1.pos_vert_accuracy = packet_in.pos_vert_accuracy;
        packet1.flags = packet_in.flags;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ESTIMATOR_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ESTIMATOR_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_estimator_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_estimator_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_estimator_status_pack(system_id, component_id, &msg , packet1.time_usec , packet1.flags , packet1.vel_ratio , packet1.pos_horiz_ratio , packet1.pos_vert_ratio , packet1.mag_ratio , packet1.hagl_ratio , packet1.tas_ratio , packet1.pos_horiz_accuracy , packet1.pos_vert_accuracy );
    mavlink_msg_estimator_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_estimator_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.flags , packet1.vel_ratio , packet1.pos_horiz_ratio , packet1.pos_vert_ratio , packet1.mag_ratio , packet1.hagl_ratio , packet1.tas_ratio , packet1.pos_horiz_accuracy , packet1.pos_vert_accuracy );
    mavlink_msg_estimator_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_estimator_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_estimator_status_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.flags , packet1.vel_ratio , packet1.pos_horiz_ratio , packet1.pos_vert_ratio , packet1.mag_ratio , packet1.hagl_ratio , packet1.tas_ratio , packet1.pos_horiz_accuracy , packet1.pos_vert_accuracy );
    mavlink_msg_estimator_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ESTIMATOR_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ESTIMATOR_STATUS) != NULL);
#endif
}

static void mavlink_test_wind_cov(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_WIND_COV >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_wind_cov_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0
    };
    mavlink_wind_cov_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.wind_x = packet_in.wind_x;
        packet1.wind_y = packet_in.wind_y;
        packet1.wind_z = packet_in.wind_z;
        packet1.var_horiz = packet_in.var_horiz;
        packet1.var_vert = packet_in.var_vert;
        packet1.wind_alt = packet_in.wind_alt;
        packet1.horiz_accuracy = packet_in.horiz_accuracy;
        packet1.vert_accuracy = packet_in.vert_accuracy;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_WIND_COV_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_WIND_COV_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wind_cov_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_wind_cov_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wind_cov_pack(system_id, component_id, &msg , packet1.time_usec , packet1.wind_x , packet1.wind_y , packet1.wind_z , packet1.var_horiz , packet1.var_vert , packet1.wind_alt , packet1.horiz_accuracy , packet1.vert_accuracy );
    mavlink_msg_wind_cov_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wind_cov_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.wind_x , packet1.wind_y , packet1.wind_z , packet1.var_horiz , packet1.var_vert , packet1.wind_alt , packet1.horiz_accuracy , packet1.vert_accuracy );
    mavlink_msg_wind_cov_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_wind_cov_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wind_cov_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.wind_x , packet1.wind_y , packet1.wind_z , packet1.var_horiz , packet1.var_vert , packet1.wind_alt , packet1.horiz_accuracy , packet1.vert_accuracy );
    mavlink_msg_wind_cov_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("WIND_COV") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_WIND_COV) != NULL);
#endif
}

static void mavlink_test_gps_input(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GPS_INPUT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_gps_input_t packet_in = {
        93372036854775807ULL,963497880,963498088,963498296,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0,20147,20251,185,252,63,20511
    };
    mavlink_gps_input_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.time_week_ms = packet_in.time_week_ms;
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        packet1.alt = packet_in.alt;
        packet1.hdop = packet_in.hdop;
        packet1.vdop = packet_in.vdop;
        packet1.vn = packet_in.vn;
        packet1.ve = packet_in.ve;
        packet1.vd = packet_in.vd;
        packet1.speed_accuracy = packet_in.speed_accuracy;
        packet1.horiz_accuracy = packet_in.horiz_accuracy;
        packet1.vert_accuracy = packet_in.vert_accuracy;
        packet1.ignore_flags = packet_in.ignore_flags;
        packet1.time_week = packet_in.time_week;
        packet1.gps_id = packet_in.gps_id;
        packet1.fix_type = packet_in.fix_type;
        packet1.satellites_visible = packet_in.satellites_visible;
        packet1.yaw = packet_in.yaw;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GPS_INPUT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GPS_INPUT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_input_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_gps_input_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_input_pack(system_id, component_id, &msg , packet1.time_usec , packet1.gps_id , packet1.ignore_flags , packet1.time_week_ms , packet1.time_week , packet1.fix_type , packet1.lat , packet1.lon , packet1.alt , packet1.hdop , packet1.vdop , packet1.vn , packet1.ve , packet1.vd , packet1.speed_accuracy , packet1.horiz_accuracy , packet1.vert_accuracy , packet1.satellites_visible , packet1.yaw );
    mavlink_msg_gps_input_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_input_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.gps_id , packet1.ignore_flags , packet1.time_week_ms , packet1.time_week , packet1.fix_type , packet1.lat , packet1.lon , packet1.alt , packet1.hdop , packet1.vdop , packet1.vn , packet1.ve , packet1.vd , packet1.speed_accuracy , packet1.horiz_accuracy , packet1.vert_accuracy , packet1.satellites_visible , packet1.yaw );
    mavlink_msg_gps_input_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_gps_input_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_input_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.gps_id , packet1.ignore_flags , packet1.time_week_ms , packet1.time_week , packet1.fix_type , packet1.lat , packet1.lon , packet1.alt , packet1.hdop , packet1.vdop , packet1.vn , packet1.ve , packet1.vd , packet1.speed_accuracy , packet1.horiz_accuracy , packet1.vert_accuracy , packet1.satellites_visible , packet1.yaw );
    mavlink_msg_gps_input_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("GPS_INPUT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_GPS_INPUT) != NULL);
#endif
}

static void mavlink_test_gps_rtcm_data(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GPS_RTCM_DATA >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_gps_rtcm_data_t packet_in = {
        5,72,{ 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62 }
    };
    mavlink_gps_rtcm_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.flags = packet_in.flags;
        packet1.len = packet_in.len;
        
        mav_array_memcpy(packet1.data, packet_in.data, sizeof(uint8_t)*180);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GPS_RTCM_DATA_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GPS_RTCM_DATA_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_rtcm_data_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_gps_rtcm_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_rtcm_data_pack(system_id, component_id, &msg , packet1.flags , packet1.len , packet1.data );
    mavlink_msg_gps_rtcm_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_rtcm_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.flags , packet1.len , packet1.data );
    mavlink_msg_gps_rtcm_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_gps_rtcm_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_rtcm_data_send(MAVLINK_COMM_1 , packet1.flags , packet1.len , packet1.data );
    mavlink_msg_gps_rtcm_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("GPS_RTCM_DATA") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_GPS_RTCM_DATA) != NULL);
#endif
}

static void mavlink_test_high_latency(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_HIGH_LATENCY >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_high_latency_t packet_in = {
        963497464,963497672,963497880,17859,17963,18067,18171,18275,18379,18483,211,22,89,156,223,34,101,168,235,46,113,180,247,58
    };
    mavlink_high_latency_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.custom_mode = packet_in.custom_mode;
        packet1.latitude = packet_in.latitude;
        packet1.longitude = packet_in.longitude;
        packet1.roll = packet_in.roll;
        packet1.pitch = packet_in.pitch;
        packet1.heading = packet_in.heading;
        packet1.heading_sp = packet_in.heading_sp;
        packet1.altitude_amsl = packet_in.altitude_amsl;
        packet1.altitude_sp = packet_in.altitude_sp;
        packet1.wp_distance = packet_in.wp_distance;
        packet1.base_mode = packet_in.base_mode;
        packet1.landed_state = packet_in.landed_state;
        packet1.throttle = packet_in.throttle;
        packet1.airspeed = packet_in.airspeed;
        packet1.airspeed_sp = packet_in.airspeed_sp;
        packet1.groundspeed = packet_in.groundspeed;
        packet1.climb_rate = packet_in.climb_rate;
        packet1.gps_nsat = packet_in.gps_nsat;
        packet1.gps_fix_type = packet_in.gps_fix_type;
        packet1.battery_remaining = packet_in.battery_remaining;
        packet1.temperature = packet_in.temperature;
        packet1.temperature_air = packet_in.temperature_air;
        packet1.failsafe = packet_in.failsafe;
        packet1.wp_num = packet_in.wp_num;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_HIGH_LATENCY_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_HIGH_LATENCY_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_high_latency_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_high_latency_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_high_latency_pack(system_id, component_id, &msg , packet1.base_mode , packet1.custom_mode , packet1.landed_state , packet1.roll , packet1.pitch , packet1.heading , packet1.throttle , packet1.heading_sp , packet1.latitude , packet1.longitude , packet1.altitude_amsl , packet1.altitude_sp , packet1.airspeed , packet1.airspeed_sp , packet1.groundspeed , packet1.climb_rate , packet1.gps_nsat , packet1.gps_fix_type , packet1.battery_remaining , packet1.temperature , packet1.temperature_air , packet1.failsafe , packet1.wp_num , packet1.wp_distance );
    mavlink_msg_high_latency_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_high_latency_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.base_mode , packet1.custom_mode , packet1.landed_state , packet1.roll , packet1.pitch , packet1.heading , packet1.throttle , packet1.heading_sp , packet1.latitude , packet1.longitude , packet1.altitude_amsl , packet1.altitude_sp , packet1.airspeed , packet1.airspeed_sp , packet1.groundspeed , packet1.climb_rate , packet1.gps_nsat , packet1.gps_fix_type , packet1.battery_remaining , packet1.temperature , packet1.temperature_air , packet1.failsafe , packet1.wp_num , packet1.wp_distance );
    mavlink_msg_high_latency_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_high_latency_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_high_latency_send(MAVLINK_COMM_1 , packet1.base_mode , packet1.custom_mode , packet1.landed_state , packet1.roll , packet1.pitch , packet1.heading , packet1.throttle , packet1.heading_sp , packet1.latitude , packet1.longitude , packet1.altitude_amsl , packet1.altitude_sp , packet1.airspeed , packet1.airspeed_sp , packet1.groundspeed , packet1.climb_rate , packet1.gps_nsat , packet1.gps_fix_type , packet1.battery_remaining , packet1.temperature , packet1.temperature_air , packet1.failsafe , packet1.wp_num , packet1.wp_distance );
    mavlink_msg_high_latency_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("HIGH_LATENCY") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_HIGH_LATENCY) != NULL);
#endif
}

static void mavlink_test_high_latency2(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_HIGH_LATENCY2 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_high_latency2_t packet_in = {
        963497464,963497672,963497880,17859,17963,18067,18171,18275,18379,77,144,211,22,89,156,223,34,101,168,235,46,113,180,247,58,125,192
    };
    mavlink_high_latency2_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.latitude = packet_in.latitude;
        packet1.longitude = packet_in.longitude;
        packet1.custom_mode = packet_in.custom_mode;
        packet1.altitude = packet_in.altitude;
        packet1.target_altitude = packet_in.target_altitude;
        packet1.target_distance = packet_in.target_distance;
        packet1.wp_num = packet_in.wp_num;
        packet1.failure_flags = packet_in.failure_flags;
        packet1.type = packet_in.type;
        packet1.autopilot = packet_in.autopilot;
        packet1.heading = packet_in.heading;
        packet1.target_heading = packet_in.target_heading;
        packet1.throttle = packet_in.throttle;
        packet1.airspeed = packet_in.airspeed;
        packet1.airspeed_sp = packet_in.airspeed_sp;
        packet1.groundspeed = packet_in.groundspeed;
        packet1.windspeed = packet_in.windspeed;
        packet1.wind_heading = packet_in.wind_heading;
        packet1.eph = packet_in.eph;
        packet1.epv = packet_in.epv;
        packet1.temperature_air = packet_in.temperature_air;
        packet1.climb_rate = packet_in.climb_rate;
        packet1.battery = packet_in.battery;
        packet1.custom0 = packet_in.custom0;
        packet1.custom1 = packet_in.custom1;
        packet1.custom2 = packet_in.custom2;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_HIGH_LATENCY2_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_HIGH_LATENCY2_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_high_latency2_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_high_latency2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_high_latency2_pack(system_id, component_id, &msg , packet1.timestamp , packet1.type , packet1.autopilot , packet1.custom_mode , packet1.latitude , packet1.longitude , packet1.altitude , packet1.target_altitude , packet1.heading , packet1.target_heading , packet1.target_distance , packet1.throttle , packet1.airspeed , packet1.airspeed_sp , packet1.groundspeed , packet1.windspeed , packet1.wind_heading , packet1.eph , packet1.epv , packet1.temperature_air , packet1.climb_rate , packet1.battery , packet1.wp_num , packet1.failure_flags , packet1.custom0 , packet1.custom1 , packet1.custom2 );
    mavlink_msg_high_latency2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_high_latency2_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.type , packet1.autopilot , packet1.custom_mode , packet1.latitude , packet1.longitude , packet1.altitude , packet1.target_altitude , packet1.heading , packet1.target_heading , packet1.target_distance , packet1.throttle , packet1.airspeed , packet1.airspeed_sp , packet1.groundspeed , packet1.windspeed , packet1.wind_heading , packet1.eph , packet1.epv , packet1.temperature_air , packet1.climb_rate , packet1.battery , packet1.wp_num , packet1.failure_flags , packet1.custom0 , packet1.custom1 , packet1.custom2 );
    mavlink_msg_high_latency2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_high_latency2_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_high_latency2_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.type , packet1.autopilot , packet1.custom_mode , packet1.latitude , packet1.longitude , packet1.altitude , packet1.target_altitude , packet1.heading , packet1.target_heading , packet1.target_distance , packet1.throttle , packet1.airspeed , packet1.airspeed_sp , packet1.groundspeed , packet1.windspeed , packet1.wind_heading , packet1.eph , packet1.epv , packet1.temperature_air , packet1.climb_rate , packet1.battery , packet1.wp_num , packet1.failure_flags , packet1.custom0 , packet1.custom1 , packet1.custom2 );
    mavlink_msg_high_latency2_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("HIGH_LATENCY2") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_HIGH_LATENCY2) != NULL);
#endif
}

static void mavlink_test_vibration(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_VIBRATION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_vibration_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,963498504,963498712,963498920
    };
    mavlink_vibration_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.vibration_x = packet_in.vibration_x;
        packet1.vibration_y = packet_in.vibration_y;
        packet1.vibration_z = packet_in.vibration_z;
        packet1.clipping_0 = packet_in.clipping_0;
        packet1.clipping_1 = packet_in.clipping_1;
        packet1.clipping_2 = packet_in.clipping_2;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_VIBRATION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_VIBRATION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_vibration_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_vibration_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_vibration_pack(system_id, component_id, &msg , packet1.time_usec , packet1.vibration_x , packet1.vibration_y , packet1.vibration_z , packet1.clipping_0 , packet1.clipping_1 , packet1.clipping_2 );
    mavlink_msg_vibration_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_vibration_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.vibration_x , packet1.vibration_y , packet1.vibration_z , packet1.clipping_0 , packet1.clipping_1 , packet1.clipping_2 );
    mavlink_msg_vibration_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_vibration_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_vibration_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.vibration_x , packet1.vibration_y , packet1.vibration_z , packet1.clipping_0 , packet1.clipping_1 , packet1.clipping_2 );
    mavlink_msg_vibration_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("VIBRATION") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_VIBRATION) != NULL);
#endif
}

static void mavlink_test_home_position(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_HOME_POSITION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_home_position_t packet_in = {
        963497464,963497672,963497880,101.0,129.0,157.0,{ 185.0, 186.0, 187.0, 188.0 },297.0,325.0,353.0,93372036854779083ULL
    };
    mavlink_home_position_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.latitude = packet_in.latitude;
        packet1.longitude = packet_in.longitude;
        packet1.altitude = packet_in.altitude;
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.approach_x = packet_in.approach_x;
        packet1.approach_y = packet_in.approach_y;
        packet1.approach_z = packet_in.approach_z;
        packet1.time_usec = packet_in.time_usec;
        
        mav_array_memcpy(packet1.q, packet_in.q, sizeof(float)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_HOME_POSITION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_HOME_POSITION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_home_position_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_home_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_home_position_pack(system_id, component_id, &msg , packet1.latitude , packet1.longitude , packet1.altitude , packet1.x , packet1.y , packet1.z , packet1.q , packet1.approach_x , packet1.approach_y , packet1.approach_z , packet1.time_usec );
    mavlink_msg_home_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_home_position_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.latitude , packet1.longitude , packet1.altitude , packet1.x , packet1.y , packet1.z , packet1.q , packet1.approach_x , packet1.approach_y , packet1.approach_z , packet1.time_usec );
    mavlink_msg_home_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_home_position_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_home_position_send(MAVLINK_COMM_1 , packet1.latitude , packet1.longitude , packet1.altitude , packet1.x , packet1.y , packet1.z , packet1.q , packet1.approach_x , packet1.approach_y , packet1.approach_z , packet1.time_usec );
    mavlink_msg_home_position_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("HOME_POSITION") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_HOME_POSITION) != NULL);
#endif
}

static void mavlink_test_set_home_position(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_HOME_POSITION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_home_position_t packet_in = {
        963497464,963497672,963497880,101.0,129.0,157.0,{ 185.0, 186.0, 187.0, 188.0 },297.0,325.0,353.0,161,93372036854779146ULL
    };
    mavlink_set_home_position_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.latitude = packet_in.latitude;
        packet1.longitude = packet_in.longitude;
        packet1.altitude = packet_in.altitude;
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.approach_x = packet_in.approach_x;
        packet1.approach_y = packet_in.approach_y;
        packet1.approach_z = packet_in.approach_z;
        packet1.target_system = packet_in.target_system;
        packet1.time_usec = packet_in.time_usec;
        
        mav_array_memcpy(packet1.q, packet_in.q, sizeof(float)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_HOME_POSITION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_HOME_POSITION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_home_position_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_home_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_home_position_pack(system_id, component_id, &msg , packet1.target_system , packet1.latitude , packet1.longitude , packet1.altitude , packet1.x , packet1.y , packet1.z , packet1.q , packet1.approach_x , packet1.approach_y , packet1.approach_z , packet1.time_usec );
    mavlink_msg_set_home_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_home_position_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.latitude , packet1.longitude , packet1.altitude , packet1.x , packet1.y , packet1.z , packet1.q , packet1.approach_x , packet1.approach_y , packet1.approach_z , packet1.time_usec );
    mavlink_msg_set_home_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_home_position_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_home_position_send(MAVLINK_COMM_1 , packet1.target_system , packet1.latitude , packet1.longitude , packet1.altitude , packet1.x , packet1.y , packet1.z , packet1.q , packet1.approach_x , packet1.approach_y , packet1.approach_z , packet1.time_usec );
    mavlink_msg_set_home_position_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SET_HOME_POSITION") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SET_HOME_POSITION) != NULL);
#endif
}

static void mavlink_test_message_interval(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MESSAGE_INTERVAL >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_message_interval_t packet_in = {
        963497464,17443
    };
    mavlink_message_interval_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.interval_us = packet_in.interval_us;
        packet1.message_id = packet_in.message_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MESSAGE_INTERVAL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MESSAGE_INTERVAL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_message_interval_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_message_interval_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_message_interval_pack(system_id, component_id, &msg , packet1.message_id , packet1.interval_us );
    mavlink_msg_message_interval_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_message_interval_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.message_id , packet1.interval_us );
    mavlink_msg_message_interval_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_message_interval_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_message_interval_send(MAVLINK_COMM_1 , packet1.message_id , packet1.interval_us );
    mavlink_msg_message_interval_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MESSAGE_INTERVAL") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MESSAGE_INTERVAL) != NULL);
#endif
}

static void mavlink_test_extended_sys_state(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_EXTENDED_SYS_STATE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_extended_sys_state_t packet_in = {
        5,72
    };
    mavlink_extended_sys_state_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.vtol_state = packet_in.vtol_state;
        packet1.landed_state = packet_in.landed_state;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_EXTENDED_SYS_STATE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_EXTENDED_SYS_STATE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_extended_sys_state_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_extended_sys_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_extended_sys_state_pack(system_id, component_id, &msg , packet1.vtol_state , packet1.landed_state );
    mavlink_msg_extended_sys_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_extended_sys_state_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.vtol_state , packet1.landed_state );
    mavlink_msg_extended_sys_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_extended_sys_state_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_extended_sys_state_send(MAVLINK_COMM_1 , packet1.vtol_state , packet1.landed_state );
    mavlink_msg_extended_sys_state_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("EXTENDED_SYS_STATE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_EXTENDED_SYS_STATE) != NULL);
#endif
}

static void mavlink_test_adsb_vehicle(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ADSB_VEHICLE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_adsb_vehicle_t packet_in = {
        963497464,963497672,963497880,963498088,18067,18171,18275,18379,18483,211,"BCDEFGHI",113,180
    };
    mavlink_adsb_vehicle_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.ICAO_address = packet_in.ICAO_address;
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        packet1.altitude = packet_in.altitude;
        packet1.heading = packet_in.heading;
        packet1.hor_velocity = packet_in.hor_velocity;
        packet1.ver_velocity = packet_in.ver_velocity;
        packet1.flags = packet_in.flags;
        packet1.squawk = packet_in.squawk;
        packet1.altitude_type = packet_in.altitude_type;
        packet1.emitter_type = packet_in.emitter_type;
        packet1.tslc = packet_in.tslc;
        
        mav_array_memcpy(packet1.callsign, packet_in.callsign, sizeof(char)*9);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ADSB_VEHICLE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ADSB_VEHICLE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adsb_vehicle_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_adsb_vehicle_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adsb_vehicle_pack(system_id, component_id, &msg , packet1.ICAO_address , packet1.lat , packet1.lon , packet1.altitude_type , packet1.altitude , packet1.heading , packet1.hor_velocity , packet1.ver_velocity , packet1.callsign , packet1.emitter_type , packet1.tslc , packet1.flags , packet1.squawk );
    mavlink_msg_adsb_vehicle_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adsb_vehicle_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.ICAO_address , packet1.lat , packet1.lon , packet1.altitude_type , packet1.altitude , packet1.heading , packet1.hor_velocity , packet1.ver_velocity , packet1.callsign , packet1.emitter_type , packet1.tslc , packet1.flags , packet1.squawk );
    mavlink_msg_adsb_vehicle_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_adsb_vehicle_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adsb_vehicle_send(MAVLINK_COMM_1 , packet1.ICAO_address , packet1.lat , packet1.lon , packet1.altitude_type , packet1.altitude , packet1.heading , packet1.hor_velocity , packet1.ver_velocity , packet1.callsign , packet1.emitter_type , packet1.tslc , packet1.flags , packet1.squawk );
    mavlink_msg_adsb_vehicle_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ADSB_VEHICLE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ADSB_VEHICLE) != NULL);
#endif
}

static void mavlink_test_collision(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_COLLISION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_collision_t packet_in = {
        963497464,45.0,73.0,101.0,53,120,187
    };
    mavlink_collision_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.id = packet_in.id;
        packet1.time_to_minimum_delta = packet_in.time_to_minimum_delta;
        packet1.altitude_minimum_delta = packet_in.altitude_minimum_delta;
        packet1.horizontal_minimum_delta = packet_in.horizontal_minimum_delta;
        packet1.src = packet_in.src;
        packet1.action = packet_in.action;
        packet1.threat_level = packet_in.threat_level;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_COLLISION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_COLLISION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_collision_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_collision_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_collision_pack(system_id, component_id, &msg , packet1.src , packet1.id , packet1.action , packet1.threat_level , packet1.time_to_minimum_delta , packet1.altitude_minimum_delta , packet1.horizontal_minimum_delta );
    mavlink_msg_collision_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_collision_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.src , packet1.id , packet1.action , packet1.threat_level , packet1.time_to_minimum_delta , packet1.altitude_minimum_delta , packet1.horizontal_minimum_delta );
    mavlink_msg_collision_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_collision_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_collision_send(MAVLINK_COMM_1 , packet1.src , packet1.id , packet1.action , packet1.threat_level , packet1.time_to_minimum_delta , packet1.altitude_minimum_delta , packet1.horizontal_minimum_delta );
    mavlink_msg_collision_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("COLLISION") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_COLLISION) != NULL);
#endif
}

static void mavlink_test_v2_extension(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_V2_EXTENSION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_v2_extension_t packet_in = {
        17235,139,206,17,{ 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76 }
    };
    mavlink_v2_extension_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.message_type = packet_in.message_type;
        packet1.target_network = packet_in.target_network;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        mav_array_memcpy(packet1.payload, packet_in.payload, sizeof(uint8_t)*249);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_V2_EXTENSION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_V2_EXTENSION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_v2_extension_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_v2_extension_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_v2_extension_pack(system_id, component_id, &msg , packet1.target_network , packet1.target_system , packet1.target_component , packet1.message_type , packet1.payload );
    mavlink_msg_v2_extension_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_v2_extension_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_network , packet1.target_system , packet1.target_component , packet1.message_type , packet1.payload );
    mavlink_msg_v2_extension_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_v2_extension_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_v2_extension_send(MAVLINK_COMM_1 , packet1.target_network , packet1.target_system , packet1.target_component , packet1.message_type , packet1.payload );
    mavlink_msg_v2_extension_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("V2_EXTENSION") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_V2_EXTENSION) != NULL);
#endif
}

static void mavlink_test_memory_vect(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MEMORY_VECT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_memory_vect_t packet_in = {
        17235,139,206,{ 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48 }
    };
    mavlink_memory_vect_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.address = packet_in.address;
        packet1.ver = packet_in.ver;
        packet1.type = packet_in.type;
        
        mav_array_memcpy(packet1.value, packet_in.value, sizeof(int8_t)*32);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MEMORY_VECT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MEMORY_VECT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_memory_vect_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_memory_vect_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_memory_vect_pack(system_id, component_id, &msg , packet1.address , packet1.ver , packet1.type , packet1.value );
    mavlink_msg_memory_vect_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_memory_vect_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.address , packet1.ver , packet1.type , packet1.value );
    mavlink_msg_memory_vect_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_memory_vect_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_memory_vect_send(MAVLINK_COMM_1 , packet1.address , packet1.ver , packet1.type , packet1.value );
    mavlink_msg_memory_vect_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MEMORY_VECT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MEMORY_VECT) != NULL);
#endif
}

static void mavlink_test_debug_vect(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DEBUG_VECT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_debug_vect_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,"UVWXYZABC"
    };
    mavlink_debug_vect_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        
        mav_array_memcpy(packet1.name, packet_in.name, sizeof(char)*10);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DEBUG_VECT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DEBUG_VECT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_debug_vect_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_debug_vect_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_debug_vect_pack(system_id, component_id, &msg , packet1.name , packet1.time_usec , packet1.x , packet1.y , packet1.z );
    mavlink_msg_debug_vect_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_debug_vect_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.name , packet1.time_usec , packet1.x , packet1.y , packet1.z );
    mavlink_msg_debug_vect_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_debug_vect_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_debug_vect_send(MAVLINK_COMM_1 , packet1.name , packet1.time_usec , packet1.x , packet1.y , packet1.z );
    mavlink_msg_debug_vect_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("DEBUG_VECT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_DEBUG_VECT) != NULL);
#endif
}

static void mavlink_test_named_value_float(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_NAMED_VALUE_FLOAT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_named_value_float_t packet_in = {
        963497464,45.0,"IJKLMNOPQ"
    };
    mavlink_named_value_float_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.value = packet_in.value;
        
        mav_array_memcpy(packet1.name, packet_in.name, sizeof(char)*10);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_named_value_float_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_named_value_float_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_named_value_float_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.name , packet1.value );
    mavlink_msg_named_value_float_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_named_value_float_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.name , packet1.value );
    mavlink_msg_named_value_float_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_named_value_float_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_named_value_float_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.name , packet1.value );
    mavlink_msg_named_value_float_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("NAMED_VALUE_FLOAT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_NAMED_VALUE_FLOAT) != NULL);
#endif
}

static void mavlink_test_named_value_int(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_NAMED_VALUE_INT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_named_value_int_t packet_in = {
        963497464,963497672,"IJKLMNOPQ"
    };
    mavlink_named_value_int_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.value = packet_in.value;
        
        mav_array_memcpy(packet1.name, packet_in.name, sizeof(char)*10);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_NAMED_VALUE_INT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_NAMED_VALUE_INT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_named_value_int_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_named_value_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_named_value_int_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.name , packet1.value );
    mavlink_msg_named_value_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_named_value_int_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.name , packet1.value );
    mavlink_msg_named_value_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_named_value_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_named_value_int_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.name , packet1.value );
    mavlink_msg_named_value_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("NAMED_VALUE_INT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_NAMED_VALUE_INT) != NULL);
#endif
}

static void mavlink_test_statustext(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_STATUSTEXT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_statustext_t packet_in = {
        5,"BCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWX",19887,228
    };
    mavlink_statustext_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.severity = packet_in.severity;
        packet1.id = packet_in.id;
        packet1.chunk_seq = packet_in.chunk_seq;
        
        mav_array_memcpy(packet1.text, packet_in.text, sizeof(char)*50);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_STATUSTEXT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_STATUSTEXT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_statustext_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_statustext_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_statustext_pack(system_id, component_id, &msg , packet1.severity , packet1.text , packet1.id , packet1.chunk_seq );
    mavlink_msg_statustext_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_statustext_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.severity , packet1.text , packet1.id , packet1.chunk_seq );
    mavlink_msg_statustext_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_statustext_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_statustext_send(MAVLINK_COMM_1 , packet1.severity , packet1.text , packet1.id , packet1.chunk_seq );
    mavlink_msg_statustext_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("STATUSTEXT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_STATUSTEXT) != NULL);
#endif
}

static void mavlink_test_debug(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DEBUG >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_debug_t packet_in = {
        963497464,45.0,29
    };
    mavlink_debug_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.value = packet_in.value;
        packet1.ind = packet_in.ind;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DEBUG_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DEBUG_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_debug_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_debug_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_debug_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.ind , packet1.value );
    mavlink_msg_debug_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_debug_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.ind , packet1.value );
    mavlink_msg_debug_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_debug_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_debug_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.ind , packet1.value );
    mavlink_msg_debug_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("DEBUG") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_DEBUG) != NULL);
#endif
}

static void mavlink_test_setup_signing(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SETUP_SIGNING >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_setup_signing_t packet_in = {
        93372036854775807ULL,29,96,{ 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194 }
    };
    mavlink_setup_signing_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.initial_timestamp = packet_in.initial_timestamp;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        mav_array_memcpy(packet1.secret_key, packet_in.secret_key, sizeof(uint8_t)*32);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SETUP_SIGNING_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SETUP_SIGNING_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_setup_signing_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_setup_signing_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_setup_signing_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.secret_key , packet1.initial_timestamp );
    mavlink_msg_setup_signing_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_setup_signing_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.secret_key , packet1.initial_timestamp );
    mavlink_msg_setup_signing_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_setup_signing_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_setup_signing_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.secret_key , packet1.initial_timestamp );
    mavlink_msg_setup_signing_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SETUP_SIGNING") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SETUP_SIGNING) != NULL);
#endif
}

static void mavlink_test_button_change(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_BUTTON_CHANGE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_button_change_t packet_in = {
        963497464,963497672,29
    };
    mavlink_button_change_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.last_change_ms = packet_in.last_change_ms;
        packet1.state = packet_in.state;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_BUTTON_CHANGE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_BUTTON_CHANGE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_button_change_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_button_change_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_button_change_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.last_change_ms , packet1.state );
    mavlink_msg_button_change_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_button_change_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.last_change_ms , packet1.state );
    mavlink_msg_button_change_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_button_change_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_button_change_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.last_change_ms , packet1.state );
    mavlink_msg_button_change_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("BUTTON_CHANGE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_BUTTON_CHANGE) != NULL);
#endif
}

static void mavlink_test_play_tune(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PLAY_TUNE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_play_tune_t packet_in = {
        5,72,"CDEFGHIJKLMNOPQRSTUVWXYZABCDE","GHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVW"
    };
    mavlink_play_tune_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        mav_array_memcpy(packet1.tune, packet_in.tune, sizeof(char)*30);
        mav_array_memcpy(packet1.tune2, packet_in.tune2, sizeof(char)*200);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PLAY_TUNE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PLAY_TUNE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_play_tune_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_play_tune_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_play_tune_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.tune , packet1.tune2 );
    mavlink_msg_play_tune_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_play_tune_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.tune , packet1.tune2 );
    mavlink_msg_play_tune_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_play_tune_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_play_tune_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.tune , packet1.tune2 );
    mavlink_msg_play_tune_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("PLAY_TUNE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_PLAY_TUNE) != NULL);
#endif
}

static void mavlink_test_camera_information(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CAMERA_INFORMATION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_camera_information_t packet_in = {
        963497464,963497672,73.0,101.0,129.0,963498504,18483,18587,18691,{ 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254 },{ 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94 },159,"RSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZ",134
    };
    mavlink_camera_information_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.firmware_version = packet_in.firmware_version;
        packet1.focal_length = packet_in.focal_length;
        packet1.sensor_size_h = packet_in.sensor_size_h;
        packet1.sensor_size_v = packet_in.sensor_size_v;
        packet1.flags = packet_in.flags;
        packet1.resolution_h = packet_in.resolution_h;
        packet1.resolution_v = packet_in.resolution_v;
        packet1.cam_definition_version = packet_in.cam_definition_version;
        packet1.lens_id = packet_in.lens_id;
        packet1.gimbal_device_id = packet_in.gimbal_device_id;
        
        mav_array_memcpy(packet1.vendor_name, packet_in.vendor_name, sizeof(uint8_t)*32);
        mav_array_memcpy(packet1.model_name, packet_in.model_name, sizeof(uint8_t)*32);
        mav_array_memcpy(packet1.cam_definition_uri, packet_in.cam_definition_uri, sizeof(char)*140);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CAMERA_INFORMATION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CAMERA_INFORMATION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_information_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_camera_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_information_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.vendor_name , packet1.model_name , packet1.firmware_version , packet1.focal_length , packet1.sensor_size_h , packet1.sensor_size_v , packet1.resolution_h , packet1.resolution_v , packet1.lens_id , packet1.flags , packet1.cam_definition_version , packet1.cam_definition_uri , packet1.gimbal_device_id );
    mavlink_msg_camera_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_information_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.vendor_name , packet1.model_name , packet1.firmware_version , packet1.focal_length , packet1.sensor_size_h , packet1.sensor_size_v , packet1.resolution_h , packet1.resolution_v , packet1.lens_id , packet1.flags , packet1.cam_definition_version , packet1.cam_definition_uri , packet1.gimbal_device_id );
    mavlink_msg_camera_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_camera_information_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_information_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.vendor_name , packet1.model_name , packet1.firmware_version , packet1.focal_length , packet1.sensor_size_h , packet1.sensor_size_v , packet1.resolution_h , packet1.resolution_v , packet1.lens_id , packet1.flags , packet1.cam_definition_version , packet1.cam_definition_uri , packet1.gimbal_device_id );
    mavlink_msg_camera_information_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CAMERA_INFORMATION") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CAMERA_INFORMATION) != NULL);
#endif
}

static void mavlink_test_camera_settings(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CAMERA_SETTINGS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_camera_settings_t packet_in = {
        963497464,17,52.0,80.0
    };
    mavlink_camera_settings_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.mode_id = packet_in.mode_id;
        packet1.zoomLevel = packet_in.zoomLevel;
        packet1.focusLevel = packet_in.focusLevel;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CAMERA_SETTINGS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CAMERA_SETTINGS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_settings_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_camera_settings_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_settings_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.mode_id , packet1.zoomLevel , packet1.focusLevel );
    mavlink_msg_camera_settings_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_settings_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.mode_id , packet1.zoomLevel , packet1.focusLevel );
    mavlink_msg_camera_settings_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_camera_settings_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_settings_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.mode_id , packet1.zoomLevel , packet1.focusLevel );
    mavlink_msg_camera_settings_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CAMERA_SETTINGS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CAMERA_SETTINGS) != NULL);
#endif
}

static void mavlink_test_storage_information(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_STORAGE_INFORMATION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_storage_information_t packet_in = {
        963497464,45.0,73.0,101.0,129.0,157.0,77,144,211,22,"CDEFGHIJKLMNOPQRSTUVWXYZABCDEFG"
    };
    mavlink_storage_information_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.total_capacity = packet_in.total_capacity;
        packet1.used_capacity = packet_in.used_capacity;
        packet1.available_capacity = packet_in.available_capacity;
        packet1.read_speed = packet_in.read_speed;
        packet1.write_speed = packet_in.write_speed;
        packet1.storage_id = packet_in.storage_id;
        packet1.storage_count = packet_in.storage_count;
        packet1.status = packet_in.status;
        packet1.type = packet_in.type;
        
        mav_array_memcpy(packet1.name, packet_in.name, sizeof(char)*32);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_STORAGE_INFORMATION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_STORAGE_INFORMATION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storage_information_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_storage_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storage_information_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.storage_id , packet1.storage_count , packet1.status , packet1.total_capacity , packet1.used_capacity , packet1.available_capacity , packet1.read_speed , packet1.write_speed , packet1.type , packet1.name );
    mavlink_msg_storage_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storage_information_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.storage_id , packet1.storage_count , packet1.status , packet1.total_capacity , packet1.used_capacity , packet1.available_capacity , packet1.read_speed , packet1.write_speed , packet1.type , packet1.name );
    mavlink_msg_storage_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_storage_information_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storage_information_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.storage_id , packet1.storage_count , packet1.status , packet1.total_capacity , packet1.used_capacity , packet1.available_capacity , packet1.read_speed , packet1.write_speed , packet1.type , packet1.name );
    mavlink_msg_storage_information_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("STORAGE_INFORMATION") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_STORAGE_INFORMATION) != NULL);
#endif
}

static void mavlink_test_camera_capture_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_camera_capture_status_t packet_in = {
        963497464,45.0,963497880,101.0,53,120,963498400
    };
    mavlink_camera_capture_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.image_interval = packet_in.image_interval;
        packet1.recording_time_ms = packet_in.recording_time_ms;
        packet1.available_capacity = packet_in.available_capacity;
        packet1.image_status = packet_in.image_status;
        packet1.video_status = packet_in.video_status;
        packet1.image_count = packet_in.image_count;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_capture_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_camera_capture_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_capture_status_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.image_status , packet1.video_status , packet1.image_interval , packet1.recording_time_ms , packet1.available_capacity , packet1.image_count );
    mavlink_msg_camera_capture_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_capture_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.image_status , packet1.video_status , packet1.image_interval , packet1.recording_time_ms , packet1.available_capacity , packet1.image_count );
    mavlink_msg_camera_capture_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_camera_capture_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_capture_status_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.image_status , packet1.video_status , packet1.image_interval , packet1.recording_time_ms , packet1.available_capacity , packet1.image_count );
    mavlink_msg_camera_capture_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CAMERA_CAPTURE_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS) != NULL);
#endif
}

static void mavlink_test_camera_image_captured(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_camera_image_captured_t packet_in = {
        93372036854775807ULL,963497880,963498088,963498296,963498504,963498712,{ 213.0, 214.0, 215.0, 216.0 },963499752,149,216,"YZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRST"
    };
    mavlink_camera_image_captured_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_utc = packet_in.time_utc;
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        packet1.alt = packet_in.alt;
        packet1.relative_alt = packet_in.relative_alt;
        packet1.image_index = packet_in.image_index;
        packet1.camera_id = packet_in.camera_id;
        packet1.capture_result = packet_in.capture_result;
        
        mav_array_memcpy(packet1.q, packet_in.q, sizeof(float)*4);
        mav_array_memcpy(packet1.file_url, packet_in.file_url, sizeof(char)*205);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_image_captured_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_camera_image_captured_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_image_captured_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.time_utc , packet1.camera_id , packet1.lat , packet1.lon , packet1.alt , packet1.relative_alt , packet1.q , packet1.image_index , packet1.capture_result , packet1.file_url );
    mavlink_msg_camera_image_captured_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_image_captured_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.time_utc , packet1.camera_id , packet1.lat , packet1.lon , packet1.alt , packet1.relative_alt , packet1.q , packet1.image_index , packet1.capture_result , packet1.file_url );
    mavlink_msg_camera_image_captured_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_camera_image_captured_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_image_captured_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.time_utc , packet1.camera_id , packet1.lat , packet1.lon , packet1.alt , packet1.relative_alt , packet1.q , packet1.image_index , packet1.capture_result , packet1.file_url );
    mavlink_msg_camera_image_captured_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CAMERA_IMAGE_CAPTURED") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED) != NULL);
#endif
}

static void mavlink_test_flight_information(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_FLIGHT_INFORMATION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_flight_information_t packet_in = {
        93372036854775807ULL,93372036854776311ULL,93372036854776815ULL,963498712
    };
    mavlink_flight_information_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.arming_time_utc = packet_in.arming_time_utc;
        packet1.takeoff_time_utc = packet_in.takeoff_time_utc;
        packet1.flight_uuid = packet_in.flight_uuid;
        packet1.time_boot_ms = packet_in.time_boot_ms;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_FLIGHT_INFORMATION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_FLIGHT_INFORMATION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_flight_information_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_flight_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_flight_information_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.arming_time_utc , packet1.takeoff_time_utc , packet1.flight_uuid );
    mavlink_msg_flight_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_flight_information_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.arming_time_utc , packet1.takeoff_time_utc , packet1.flight_uuid );
    mavlink_msg_flight_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_flight_information_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_flight_information_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.arming_time_utc , packet1.takeoff_time_utc , packet1.flight_uuid );
    mavlink_msg_flight_information_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("FLIGHT_INFORMATION") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_FLIGHT_INFORMATION) != NULL);
#endif
}

static void mavlink_test_mount_orientation(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MOUNT_ORIENTATION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mount_orientation_t packet_in = {
        963497464,45.0,73.0,101.0,129.0
    };
    mavlink_mount_orientation_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.roll = packet_in.roll;
        packet1.pitch = packet_in.pitch;
        packet1.yaw = packet_in.yaw;
        packet1.yaw_absolute = packet_in.yaw_absolute;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MOUNT_ORIENTATION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MOUNT_ORIENTATION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mount_orientation_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mount_orientation_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mount_orientation_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.roll , packet1.pitch , packet1.yaw , packet1.yaw_absolute );
    mavlink_msg_mount_orientation_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mount_orientation_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.roll , packet1.pitch , packet1.yaw , packet1.yaw_absolute );
    mavlink_msg_mount_orientation_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mount_orientation_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mount_orientation_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.roll , packet1.pitch , packet1.yaw , packet1.yaw_absolute );
    mavlink_msg_mount_orientation_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MOUNT_ORIENTATION") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MOUNT_ORIENTATION) != NULL);
#endif
}

static void mavlink_test_logging_data(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LOGGING_DATA >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_logging_data_t packet_in = {
        17235,139,206,17,84,{ 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143 }
    };
    mavlink_logging_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.sequence = packet_in.sequence;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.length = packet_in.length;
        packet1.first_message_offset = packet_in.first_message_offset;
        
        mav_array_memcpy(packet1.data, packet_in.data, sizeof(uint8_t)*249);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LOGGING_DATA_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LOGGING_DATA_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_logging_data_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_logging_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_logging_data_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.sequence , packet1.length , packet1.first_message_offset , packet1.data );
    mavlink_msg_logging_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_logging_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.sequence , packet1.length , packet1.first_message_offset , packet1.data );
    mavlink_msg_logging_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_logging_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_logging_data_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.sequence , packet1.length , packet1.first_message_offset , packet1.data );
    mavlink_msg_logging_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("LOGGING_DATA") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_LOGGING_DATA) != NULL);
#endif
}

static void mavlink_test_logging_data_acked(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LOGGING_DATA_ACKED >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_logging_data_acked_t packet_in = {
        17235,139,206,17,84,{ 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143 }
    };
    mavlink_logging_data_acked_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.sequence = packet_in.sequence;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.length = packet_in.length;
        packet1.first_message_offset = packet_in.first_message_offset;
        
        mav_array_memcpy(packet1.data, packet_in.data, sizeof(uint8_t)*249);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LOGGING_DATA_ACKED_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LOGGING_DATA_ACKED_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_logging_data_acked_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_logging_data_acked_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_logging_data_acked_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.sequence , packet1.length , packet1.first_message_offset , packet1.data );
    mavlink_msg_logging_data_acked_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_logging_data_acked_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.sequence , packet1.length , packet1.first_message_offset , packet1.data );
    mavlink_msg_logging_data_acked_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_logging_data_acked_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_logging_data_acked_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.sequence , packet1.length , packet1.first_message_offset , packet1.data );
    mavlink_msg_logging_data_acked_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("LOGGING_DATA_ACKED") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_LOGGING_DATA_ACKED) != NULL);
#endif
}

static void mavlink_test_logging_ack(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LOGGING_ACK >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_logging_ack_t packet_in = {
        17235,139,206
    };
    mavlink_logging_ack_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.sequence = packet_in.sequence;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LOGGING_ACK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LOGGING_ACK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_logging_ack_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_logging_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_logging_ack_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.sequence );
    mavlink_msg_logging_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_logging_ack_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.sequence );
    mavlink_msg_logging_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_logging_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_logging_ack_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.sequence );
    mavlink_msg_logging_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("LOGGING_ACK") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_LOGGING_ACK) != NULL);
#endif
}

static void mavlink_test_video_stream_information(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_video_stream_information_t packet_in = {
        17.0,963497672,17651,17755,17859,17963,18067,187,254,65,"VWXYZABCDEFGHIJKLMNOPQRSTUVWXYZ","BCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCD",196
    };
    mavlink_video_stream_information_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.framerate = packet_in.framerate;
        packet1.bitrate = packet_in.bitrate;
        packet1.flags = packet_in.flags;
        packet1.resolution_h = packet_in.resolution_h;
        packet1.resolution_v = packet_in.resolution_v;
        packet1.rotation = packet_in.rotation;
        packet1.hfov = packet_in.hfov;
        packet1.stream_id = packet_in.stream_id;
        packet1.count = packet_in.count;
        packet1.type = packet_in.type;
        packet1.encoding = packet_in.encoding;
        
        mav_array_memcpy(packet1.name, packet_in.name, sizeof(char)*32);
        mav_array_memcpy(packet1.uri, packet_in.uri, sizeof(char)*160);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_video_stream_information_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_video_stream_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_video_stream_information_pack(system_id, component_id, &msg , packet1.stream_id , packet1.count , packet1.type , packet1.flags , packet1.framerate , packet1.resolution_h , packet1.resolution_v , packet1.bitrate , packet1.rotation , packet1.hfov , packet1.name , packet1.uri , packet1.encoding );
    mavlink_msg_video_stream_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_video_stream_information_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.stream_id , packet1.count , packet1.type , packet1.flags , packet1.framerate , packet1.resolution_h , packet1.resolution_v , packet1.bitrate , packet1.rotation , packet1.hfov , packet1.name , packet1.uri , packet1.encoding );
    mavlink_msg_video_stream_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_video_stream_information_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_video_stream_information_send(MAVLINK_COMM_1 , packet1.stream_id , packet1.count , packet1.type , packet1.flags , packet1.framerate , packet1.resolution_h , packet1.resolution_v , packet1.bitrate , packet1.rotation , packet1.hfov , packet1.name , packet1.uri , packet1.encoding );
    mavlink_msg_video_stream_information_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("VIDEO_STREAM_INFORMATION") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION) != NULL);
#endif
}

static void mavlink_test_video_stream_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_VIDEO_STREAM_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_video_stream_status_t packet_in = {
        17.0,963497672,17651,17755,17859,17963,18067,187
    };
    mavlink_video_stream_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.framerate = packet_in.framerate;
        packet1.bitrate = packet_in.bitrate;
        packet1.flags = packet_in.flags;
        packet1.resolution_h = packet_in.resolution_h;
        packet1.resolution_v = packet_in.resolution_v;
        packet1.rotation = packet_in.rotation;
        packet1.hfov = packet_in.hfov;
        packet1.stream_id = packet_in.stream_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_VIDEO_STREAM_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_VIDEO_STREAM_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_video_stream_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_video_stream_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_video_stream_status_pack(system_id, component_id, &msg , packet1.stream_id , packet1.flags , packet1.framerate , packet1.resolution_h , packet1.resolution_v , packet1.bitrate , packet1.rotation , packet1.hfov );
    mavlink_msg_video_stream_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_video_stream_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.stream_id , packet1.flags , packet1.framerate , packet1.resolution_h , packet1.resolution_v , packet1.bitrate , packet1.rotation , packet1.hfov );
    mavlink_msg_video_stream_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_video_stream_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_video_stream_status_send(MAVLINK_COMM_1 , packet1.stream_id , packet1.flags , packet1.framerate , packet1.resolution_h , packet1.resolution_v , packet1.bitrate , packet1.rotation , packet1.hfov );
    mavlink_msg_video_stream_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("VIDEO_STREAM_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_VIDEO_STREAM_STATUS) != NULL);
#endif
}

static void mavlink_test_camera_fov_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CAMERA_FOV_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_camera_fov_status_t packet_in = {
        963497464,963497672,963497880,963498088,963498296,963498504,963498712,{ 213.0, 214.0, 215.0, 216.0 },325.0,353.0
    };
    mavlink_camera_fov_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.lat_camera = packet_in.lat_camera;
        packet1.lon_camera = packet_in.lon_camera;
        packet1.alt_camera = packet_in.alt_camera;
        packet1.lat_image = packet_in.lat_image;
        packet1.lon_image = packet_in.lon_image;
        packet1.alt_image = packet_in.alt_image;
        packet1.hfov = packet_in.hfov;
        packet1.vfov = packet_in.vfov;
        
        mav_array_memcpy(packet1.q, packet_in.q, sizeof(float)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CAMERA_FOV_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CAMERA_FOV_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_fov_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_camera_fov_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_fov_status_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.lat_camera , packet1.lon_camera , packet1.alt_camera , packet1.lat_image , packet1.lon_image , packet1.alt_image , packet1.q , packet1.hfov , packet1.vfov );
    mavlink_msg_camera_fov_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_fov_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.lat_camera , packet1.lon_camera , packet1.alt_camera , packet1.lat_image , packet1.lon_image , packet1.alt_image , packet1.q , packet1.hfov , packet1.vfov );
    mavlink_msg_camera_fov_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_camera_fov_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_fov_status_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.lat_camera , packet1.lon_camera , packet1.alt_camera , packet1.lat_image , packet1.lon_image , packet1.alt_image , packet1.q , packet1.hfov , packet1.vfov );
    mavlink_msg_camera_fov_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CAMERA_FOV_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CAMERA_FOV_STATUS) != NULL);
#endif
}

static void mavlink_test_camera_tracking_image_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_camera_tracking_image_status_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,185.0,89,156,223
    };
    mavlink_camera_tracking_image_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.point_x = packet_in.point_x;
        packet1.point_y = packet_in.point_y;
        packet1.radius = packet_in.radius;
        packet1.rec_top_x = packet_in.rec_top_x;
        packet1.rec_top_y = packet_in.rec_top_y;
        packet1.rec_bottom_x = packet_in.rec_bottom_x;
        packet1.rec_bottom_y = packet_in.rec_bottom_y;
        packet1.tracking_status = packet_in.tracking_status;
        packet1.tracking_mode = packet_in.tracking_mode;
        packet1.target_data = packet_in.target_data;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_tracking_image_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_camera_tracking_image_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_tracking_image_status_pack(system_id, component_id, &msg , packet1.tracking_status , packet1.tracking_mode , packet1.target_data , packet1.point_x , packet1.point_y , packet1.radius , packet1.rec_top_x , packet1.rec_top_y , packet1.rec_bottom_x , packet1.rec_bottom_y );
    mavlink_msg_camera_tracking_image_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_tracking_image_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.tracking_status , packet1.tracking_mode , packet1.target_data , packet1.point_x , packet1.point_y , packet1.radius , packet1.rec_top_x , packet1.rec_top_y , packet1.rec_bottom_x , packet1.rec_bottom_y );
    mavlink_msg_camera_tracking_image_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_camera_tracking_image_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_tracking_image_status_send(MAVLINK_COMM_1 , packet1.tracking_status , packet1.tracking_mode , packet1.target_data , packet1.point_x , packet1.point_y , packet1.radius , packet1.rec_top_x , packet1.rec_top_y , packet1.rec_bottom_x , packet1.rec_bottom_y );
    mavlink_msg_camera_tracking_image_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CAMERA_TRACKING_IMAGE_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS) != NULL);
#endif
}

static void mavlink_test_camera_tracking_geo_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_camera_tracking_geo_status_t packet_in = {
        963497464,963497672,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,149
    };
    mavlink_camera_tracking_geo_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        packet1.alt = packet_in.alt;
        packet1.h_acc = packet_in.h_acc;
        packet1.v_acc = packet_in.v_acc;
        packet1.vel_n = packet_in.vel_n;
        packet1.vel_e = packet_in.vel_e;
        packet1.vel_d = packet_in.vel_d;
        packet1.vel_acc = packet_in.vel_acc;
        packet1.dist = packet_in.dist;
        packet1.hdg = packet_in.hdg;
        packet1.hdg_acc = packet_in.hdg_acc;
        packet1.tracking_status = packet_in.tracking_status;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_tracking_geo_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_camera_tracking_geo_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_tracking_geo_status_pack(system_id, component_id, &msg , packet1.tracking_status , packet1.lat , packet1.lon , packet1.alt , packet1.h_acc , packet1.v_acc , packet1.vel_n , packet1.vel_e , packet1.vel_d , packet1.vel_acc , packet1.dist , packet1.hdg , packet1.hdg_acc );
    mavlink_msg_camera_tracking_geo_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_tracking_geo_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.tracking_status , packet1.lat , packet1.lon , packet1.alt , packet1.h_acc , packet1.v_acc , packet1.vel_n , packet1.vel_e , packet1.vel_d , packet1.vel_acc , packet1.dist , packet1.hdg , packet1.hdg_acc );
    mavlink_msg_camera_tracking_geo_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_camera_tracking_geo_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_tracking_geo_status_send(MAVLINK_COMM_1 , packet1.tracking_status , packet1.lat , packet1.lon , packet1.alt , packet1.h_acc , packet1.v_acc , packet1.vel_n , packet1.vel_e , packet1.vel_d , packet1.vel_acc , packet1.dist , packet1.hdg , packet1.hdg_acc );
    mavlink_msg_camera_tracking_geo_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CAMERA_TRACKING_GEO_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS) != NULL);
#endif
}

static void mavlink_test_camera_thermal_range(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_camera_thermal_range_t packet_in = {
        963497464,45.0,73.0,101.0,129.0,157.0,185.0,89,156
    };
    mavlink_camera_thermal_range_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.max = packet_in.max;
        packet1.max_point_x = packet_in.max_point_x;
        packet1.max_point_y = packet_in.max_point_y;
        packet1.min = packet_in.min;
        packet1.min_point_x = packet_in.min_point_x;
        packet1.min_point_y = packet_in.min_point_y;
        packet1.stream_id = packet_in.stream_id;
        packet1.camera_device_id = packet_in.camera_device_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_thermal_range_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_camera_thermal_range_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_thermal_range_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.stream_id , packet1.camera_device_id , packet1.max , packet1.max_point_x , packet1.max_point_y , packet1.min , packet1.min_point_x , packet1.min_point_y );
    mavlink_msg_camera_thermal_range_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_thermal_range_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.stream_id , packet1.camera_device_id , packet1.max , packet1.max_point_x , packet1.max_point_y , packet1.min , packet1.min_point_x , packet1.min_point_y );
    mavlink_msg_camera_thermal_range_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_camera_thermal_range_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_thermal_range_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.stream_id , packet1.camera_device_id , packet1.max , packet1.max_point_x , packet1.max_point_y , packet1.min , packet1.min_point_x , packet1.min_point_y );
    mavlink_msg_camera_thermal_range_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CAMERA_THERMAL_RANGE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE) != NULL);
#endif
}

static void mavlink_test_gimbal_manager_information(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_gimbal_manager_information_t packet_in = {
        963497464,963497672,73.0,101.0,129.0,157.0,185.0,213.0,101
    };
    mavlink_gimbal_manager_information_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.cap_flags = packet_in.cap_flags;
        packet1.roll_min = packet_in.roll_min;
        packet1.roll_max = packet_in.roll_max;
        packet1.pitch_min = packet_in.pitch_min;
        packet1.pitch_max = packet_in.pitch_max;
        packet1.yaw_min = packet_in.yaw_min;
        packet1.yaw_max = packet_in.yaw_max;
        packet1.gimbal_device_id = packet_in.gimbal_device_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_manager_information_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_gimbal_manager_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_manager_information_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.cap_flags , packet1.gimbal_device_id , packet1.roll_min , packet1.roll_max , packet1.pitch_min , packet1.pitch_max , packet1.yaw_min , packet1.yaw_max );
    mavlink_msg_gimbal_manager_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_manager_information_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.cap_flags , packet1.gimbal_device_id , packet1.roll_min , packet1.roll_max , packet1.pitch_min , packet1.pitch_max , packet1.yaw_min , packet1.yaw_max );
    mavlink_msg_gimbal_manager_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_gimbal_manager_information_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_manager_information_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.cap_flags , packet1.gimbal_device_id , packet1.roll_min , packet1.roll_max , packet1.pitch_min , packet1.pitch_max , packet1.yaw_min , packet1.yaw_max );
    mavlink_msg_gimbal_manager_information_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("GIMBAL_MANAGER_INFORMATION") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION) != NULL);
#endif
}

static void mavlink_test_gimbal_manager_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_gimbal_manager_status_t packet_in = {
        963497464,963497672,29,96,163,230,41
    };
    mavlink_gimbal_manager_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.flags = packet_in.flags;
        packet1.gimbal_device_id = packet_in.gimbal_device_id;
        packet1.primary_control_sysid = packet_in.primary_control_sysid;
        packet1.primary_control_compid = packet_in.primary_control_compid;
        packet1.secondary_control_sysid = packet_in.secondary_control_sysid;
        packet1.secondary_control_compid = packet_in.secondary_control_compid;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_manager_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_gimbal_manager_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_manager_status_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.flags , packet1.gimbal_device_id , packet1.primary_control_sysid , packet1.primary_control_compid , packet1.secondary_control_sysid , packet1.secondary_control_compid );
    mavlink_msg_gimbal_manager_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_manager_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.flags , packet1.gimbal_device_id , packet1.primary_control_sysid , packet1.primary_control_compid , packet1.secondary_control_sysid , packet1.secondary_control_compid );
    mavlink_msg_gimbal_manager_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_gimbal_manager_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_manager_status_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.flags , packet1.gimbal_device_id , packet1.primary_control_sysid , packet1.primary_control_compid , packet1.secondary_control_sysid , packet1.secondary_control_compid );
    mavlink_msg_gimbal_manager_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("GIMBAL_MANAGER_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS) != NULL);
#endif
}

static void mavlink_test_gimbal_manager_set_attitude(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_gimbal_manager_set_attitude_t packet_in = {
        963497464,{ 45.0, 46.0, 47.0, 48.0 },157.0,185.0,213.0,101,168,235
    };
    mavlink_gimbal_manager_set_attitude_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.flags = packet_in.flags;
        packet1.angular_velocity_x = packet_in.angular_velocity_x;
        packet1.angular_velocity_y = packet_in.angular_velocity_y;
        packet1.angular_velocity_z = packet_in.angular_velocity_z;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.gimbal_device_id = packet_in.gimbal_device_id;
        
        mav_array_memcpy(packet1.q, packet_in.q, sizeof(float)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_manager_set_attitude_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_gimbal_manager_set_attitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_manager_set_attitude_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.flags , packet1.gimbal_device_id , packet1.q , packet1.angular_velocity_x , packet1.angular_velocity_y , packet1.angular_velocity_z );
    mavlink_msg_gimbal_manager_set_attitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_manager_set_attitude_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.flags , packet1.gimbal_device_id , packet1.q , packet1.angular_velocity_x , packet1.angular_velocity_y , packet1.angular_velocity_z );
    mavlink_msg_gimbal_manager_set_attitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_gimbal_manager_set_attitude_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_manager_set_attitude_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.flags , packet1.gimbal_device_id , packet1.q , packet1.angular_velocity_x , packet1.angular_velocity_y , packet1.angular_velocity_z );
    mavlink_msg_gimbal_manager_set_attitude_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("GIMBAL_MANAGER_SET_ATTITUDE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE) != NULL);
#endif
}

static void mavlink_test_gimbal_device_information(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_gimbal_device_information_t packet_in = {
        93372036854775807ULL,963497880,963498088,963498296,157.0,185.0,213.0,241.0,269.0,297.0,19523,19627,"WXYZABCDEFGHIJKLMNOPQRSTUVWXYZA","CDEFGHIJKLMNOPQRSTUVWXYZABCDEFG","IJKLMNOPQRSTUVWXYZABCDEFGHIJKLM",181
    };
    mavlink_gimbal_device_information_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.uid = packet_in.uid;
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.firmware_version = packet_in.firmware_version;
        packet1.hardware_version = packet_in.hardware_version;
        packet1.roll_min = packet_in.roll_min;
        packet1.roll_max = packet_in.roll_max;
        packet1.pitch_min = packet_in.pitch_min;
        packet1.pitch_max = packet_in.pitch_max;
        packet1.yaw_min = packet_in.yaw_min;
        packet1.yaw_max = packet_in.yaw_max;
        packet1.cap_flags = packet_in.cap_flags;
        packet1.custom_cap_flags = packet_in.custom_cap_flags;
        packet1.gimbal_device_id = packet_in.gimbal_device_id;
        
        mav_array_memcpy(packet1.vendor_name, packet_in.vendor_name, sizeof(char)*32);
        mav_array_memcpy(packet1.model_name, packet_in.model_name, sizeof(char)*32);
        mav_array_memcpy(packet1.custom_name, packet_in.custom_name, sizeof(char)*32);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_device_information_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_gimbal_device_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_device_information_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.vendor_name , packet1.model_name , packet1.custom_name , packet1.firmware_version , packet1.hardware_version , packet1.uid , packet1.cap_flags , packet1.custom_cap_flags , packet1.roll_min , packet1.roll_max , packet1.pitch_min , packet1.pitch_max , packet1.yaw_min , packet1.yaw_max , packet1.gimbal_device_id );
    mavlink_msg_gimbal_device_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_device_information_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.vendor_name , packet1.model_name , packet1.custom_name , packet1.firmware_version , packet1.hardware_version , packet1.uid , packet1.cap_flags , packet1.custom_cap_flags , packet1.roll_min , packet1.roll_max , packet1.pitch_min , packet1.pitch_max , packet1.yaw_min , packet1.yaw_max , packet1.gimbal_device_id );
    mavlink_msg_gimbal_device_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_gimbal_device_information_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_device_information_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.vendor_name , packet1.model_name , packet1.custom_name , packet1.firmware_version , packet1.hardware_version , packet1.uid , packet1.cap_flags , packet1.custom_cap_flags , packet1.roll_min , packet1.roll_max , packet1.pitch_min , packet1.pitch_max , packet1.yaw_min , packet1.yaw_max , packet1.gimbal_device_id );
    mavlink_msg_gimbal_device_information_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("GIMBAL_DEVICE_INFORMATION") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION) != NULL);
#endif
}

static void mavlink_test_gimbal_device_set_attitude(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_gimbal_device_set_attitude_t packet_in = {
        { 17.0, 18.0, 19.0, 20.0 },129.0,157.0,185.0,18691,223,34
    };
    mavlink_gimbal_device_set_attitude_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.angular_velocity_x = packet_in.angular_velocity_x;
        packet1.angular_velocity_y = packet_in.angular_velocity_y;
        packet1.angular_velocity_z = packet_in.angular_velocity_z;
        packet1.flags = packet_in.flags;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        mav_array_memcpy(packet1.q, packet_in.q, sizeof(float)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_device_set_attitude_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_gimbal_device_set_attitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_device_set_attitude_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.flags , packet1.q , packet1.angular_velocity_x , packet1.angular_velocity_y , packet1.angular_velocity_z );
    mavlink_msg_gimbal_device_set_attitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_device_set_attitude_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.flags , packet1.q , packet1.angular_velocity_x , packet1.angular_velocity_y , packet1.angular_velocity_z );
    mavlink_msg_gimbal_device_set_attitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_gimbal_device_set_attitude_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_device_set_attitude_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.flags , packet1.q , packet1.angular_velocity_x , packet1.angular_velocity_y , packet1.angular_velocity_z );
    mavlink_msg_gimbal_device_set_attitude_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("GIMBAL_DEVICE_SET_ATTITUDE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE) != NULL);
#endif
}

static void mavlink_test_gimbal_device_attitude_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_gimbal_device_attitude_status_t packet_in = {
        963497464,{ 45.0, 46.0, 47.0, 48.0 },157.0,185.0,213.0,963499128,19107,247,58,297.0,325.0,149
    };
    mavlink_gimbal_device_attitude_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.angular_velocity_x = packet_in.angular_velocity_x;
        packet1.angular_velocity_y = packet_in.angular_velocity_y;
        packet1.angular_velocity_z = packet_in.angular_velocity_z;
        packet1.failure_flags = packet_in.failure_flags;
        packet1.flags = packet_in.flags;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.delta_yaw = packet_in.delta_yaw;
        packet1.delta_yaw_velocity = packet_in.delta_yaw_velocity;
        packet1.gimbal_device_id = packet_in.gimbal_device_id;
        
        mav_array_memcpy(packet1.q, packet_in.q, sizeof(float)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_device_attitude_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_gimbal_device_attitude_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_device_attitude_status_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.time_boot_ms , packet1.flags , packet1.q , packet1.angular_velocity_x , packet1.angular_velocity_y , packet1.angular_velocity_z , packet1.failure_flags , packet1.delta_yaw , packet1.delta_yaw_velocity , packet1.gimbal_device_id );
    mavlink_msg_gimbal_device_attitude_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_device_attitude_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.time_boot_ms , packet1.flags , packet1.q , packet1.angular_velocity_x , packet1.angular_velocity_y , packet1.angular_velocity_z , packet1.failure_flags , packet1.delta_yaw , packet1.delta_yaw_velocity , packet1.gimbal_device_id );
    mavlink_msg_gimbal_device_attitude_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_gimbal_device_attitude_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_device_attitude_status_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.time_boot_ms , packet1.flags , packet1.q , packet1.angular_velocity_x , packet1.angular_velocity_y , packet1.angular_velocity_z , packet1.failure_flags , packet1.delta_yaw , packet1.delta_yaw_velocity , packet1.gimbal_device_id );
    mavlink_msg_gimbal_device_attitude_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("GIMBAL_DEVICE_ATTITUDE_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS) != NULL);
#endif
}

static void mavlink_test_autopilot_state_for_gimbal_device(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_autopilot_state_for_gimbal_device_t packet_in = {
        93372036854775807ULL,{ 73.0, 74.0, 75.0, 76.0 },963498712,213.0,241.0,269.0,963499544,325.0,19731,27,94,161,388.0
    };
    mavlink_autopilot_state_for_gimbal_device_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_us = packet_in.time_boot_us;
        packet1.q_estimated_delay_us = packet_in.q_estimated_delay_us;
        packet1.vx = packet_in.vx;
        packet1.vy = packet_in.vy;
        packet1.vz = packet_in.vz;
        packet1.v_estimated_delay_us = packet_in.v_estimated_delay_us;
        packet1.feed_forward_angular_velocity_z = packet_in.feed_forward_angular_velocity_z;
        packet1.estimator_status = packet_in.estimator_status;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.landed_state = packet_in.landed_state;
        packet1.angular_velocity_z = packet_in.angular_velocity_z;
        
        mav_array_memcpy(packet1.q, packet_in.q, sizeof(float)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_autopilot_state_for_gimbal_device_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_autopilot_state_for_gimbal_device_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_autopilot_state_for_gimbal_device_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.time_boot_us , packet1.q , packet1.q_estimated_delay_us , packet1.vx , packet1.vy , packet1.vz , packet1.v_estimated_delay_us , packet1.feed_forward_angular_velocity_z , packet1.estimator_status , packet1.landed_state , packet1.angular_velocity_z );
    mavlink_msg_autopilot_state_for_gimbal_device_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_autopilot_state_for_gimbal_device_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.time_boot_us , packet1.q , packet1.q_estimated_delay_us , packet1.vx , packet1.vy , packet1.vz , packet1.v_estimated_delay_us , packet1.feed_forward_angular_velocity_z , packet1.estimator_status , packet1.landed_state , packet1.angular_velocity_z );
    mavlink_msg_autopilot_state_for_gimbal_device_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_autopilot_state_for_gimbal_device_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_autopilot_state_for_gimbal_device_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.time_boot_us , packet1.q , packet1.q_estimated_delay_us , packet1.vx , packet1.vy , packet1.vz , packet1.v_estimated_delay_us , packet1.feed_forward_angular_velocity_z , packet1.estimator_status , packet1.landed_state , packet1.angular_velocity_z );
    mavlink_msg_autopilot_state_for_gimbal_device_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("AUTOPILOT_STATE_FOR_GIMBAL_DEVICE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE) != NULL);
#endif
}

static void mavlink_test_gimbal_manager_set_pitchyaw(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_gimbal_manager_set_pitchyaw_t packet_in = {
        963497464,45.0,73.0,101.0,129.0,65,132,199
    };
    mavlink_gimbal_manager_set_pitchyaw_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.flags = packet_in.flags;
        packet1.pitch = packet_in.pitch;
        packet1.yaw = packet_in.yaw;
        packet1.pitch_rate = packet_in.pitch_rate;
        packet1.yaw_rate = packet_in.yaw_rate;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.gimbal_device_id = packet_in.gimbal_device_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_manager_set_pitchyaw_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_gimbal_manager_set_pitchyaw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_manager_set_pitchyaw_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.flags , packet1.gimbal_device_id , packet1.pitch , packet1.yaw , packet1.pitch_rate , packet1.yaw_rate );
    mavlink_msg_gimbal_manager_set_pitchyaw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_manager_set_pitchyaw_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.flags , packet1.gimbal_device_id , packet1.pitch , packet1.yaw , packet1.pitch_rate , packet1.yaw_rate );
    mavlink_msg_gimbal_manager_set_pitchyaw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_gimbal_manager_set_pitchyaw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_manager_set_pitchyaw_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.flags , packet1.gimbal_device_id , packet1.pitch , packet1.yaw , packet1.pitch_rate , packet1.yaw_rate );
    mavlink_msg_gimbal_manager_set_pitchyaw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("GIMBAL_MANAGER_SET_PITCHYAW") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW) != NULL);
#endif
}

static void mavlink_test_gimbal_manager_set_manual_control(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_MANUAL_CONTROL >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_gimbal_manager_set_manual_control_t packet_in = {
        963497464,45.0,73.0,101.0,129.0,65,132,199
    };
    mavlink_gimbal_manager_set_manual_control_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.flags = packet_in.flags;
        packet1.pitch = packet_in.pitch;
        packet1.yaw = packet_in.yaw;
        packet1.pitch_rate = packet_in.pitch_rate;
        packet1.yaw_rate = packet_in.yaw_rate;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.gimbal_device_id = packet_in.gimbal_device_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_MANUAL_CONTROL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_MANUAL_CONTROL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_manager_set_manual_control_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_gimbal_manager_set_manual_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_manager_set_manual_control_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.flags , packet1.gimbal_device_id , packet1.pitch , packet1.yaw , packet1.pitch_rate , packet1.yaw_rate );
    mavlink_msg_gimbal_manager_set_manual_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_manager_set_manual_control_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.flags , packet1.gimbal_device_id , packet1.pitch , packet1.yaw , packet1.pitch_rate , packet1.yaw_rate );
    mavlink_msg_gimbal_manager_set_manual_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_gimbal_manager_set_manual_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_manager_set_manual_control_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.flags , packet1.gimbal_device_id , packet1.pitch , packet1.yaw , packet1.pitch_rate , packet1.yaw_rate );
    mavlink_msg_gimbal_manager_set_manual_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("GIMBAL_MANAGER_SET_MANUAL_CONTROL") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_MANUAL_CONTROL) != NULL);
#endif
}

static void mavlink_test_wifi_config_ap(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_WIFI_CONFIG_AP >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_wifi_config_ap_t packet_in = {
        "ABCDEFGHIJKLMNOPQRSTUVWXYZABCDE","GHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQ"
    };
    mavlink_wifi_config_ap_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        mav_array_memcpy(packet1.ssid, packet_in.ssid, sizeof(char)*32);
        mav_array_memcpy(packet1.password, packet_in.password, sizeof(char)*64);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_WIFI_CONFIG_AP_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_WIFI_CONFIG_AP_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wifi_config_ap_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_wifi_config_ap_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wifi_config_ap_pack(system_id, component_id, &msg , packet1.ssid , packet1.password );
    mavlink_msg_wifi_config_ap_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wifi_config_ap_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.ssid , packet1.password );
    mavlink_msg_wifi_config_ap_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_wifi_config_ap_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wifi_config_ap_send(MAVLINK_COMM_1 , packet1.ssid , packet1.password );
    mavlink_msg_wifi_config_ap_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("WIFI_CONFIG_AP") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_WIFI_CONFIG_AP) != NULL);
#endif
}

static void mavlink_test_ais_vessel(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_AIS_VESSEL >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_ais_vessel_t packet_in = {
        963497464,963497672,963497880,17859,17963,18067,18171,18275,18379,18483,211,22,89,156,223,"FGHIJK","MNOPQRSTUVWXYZABCDE"
    };
    mavlink_ais_vessel_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.MMSI = packet_in.MMSI;
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        packet1.COG = packet_in.COG;
        packet1.heading = packet_in.heading;
        packet1.velocity = packet_in.velocity;
        packet1.dimension_bow = packet_in.dimension_bow;
        packet1.dimension_stern = packet_in.dimension_stern;
        packet1.tslc = packet_in.tslc;
        packet1.flags = packet_in.flags;
        packet1.turn_rate = packet_in.turn_rate;
        packet1.navigational_status = packet_in.navigational_status;
        packet1.type = packet_in.type;
        packet1.dimension_port = packet_in.dimension_port;
        packet1.dimension_starboard = packet_in.dimension_starboard;
        
        mav_array_memcpy(packet1.callsign, packet_in.callsign, sizeof(char)*7);
        mav_array_memcpy(packet1.name, packet_in.name, sizeof(char)*20);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_AIS_VESSEL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_AIS_VESSEL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ais_vessel_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_ais_vessel_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ais_vessel_pack(system_id, component_id, &msg , packet1.MMSI , packet1.lat , packet1.lon , packet1.COG , packet1.heading , packet1.velocity , packet1.turn_rate , packet1.navigational_status , packet1.type , packet1.dimension_bow , packet1.dimension_stern , packet1.dimension_port , packet1.dimension_starboard , packet1.callsign , packet1.name , packet1.tslc , packet1.flags );
    mavlink_msg_ais_vessel_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ais_vessel_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.MMSI , packet1.lat , packet1.lon , packet1.COG , packet1.heading , packet1.velocity , packet1.turn_rate , packet1.navigational_status , packet1.type , packet1.dimension_bow , packet1.dimension_stern , packet1.dimension_port , packet1.dimension_starboard , packet1.callsign , packet1.name , packet1.tslc , packet1.flags );
    mavlink_msg_ais_vessel_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_ais_vessel_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ais_vessel_send(MAVLINK_COMM_1 , packet1.MMSI , packet1.lat , packet1.lon , packet1.COG , packet1.heading , packet1.velocity , packet1.turn_rate , packet1.navigational_status , packet1.type , packet1.dimension_bow , packet1.dimension_stern , packet1.dimension_port , packet1.dimension_starboard , packet1.callsign , packet1.name , packet1.tslc , packet1.flags );
    mavlink_msg_ais_vessel_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("AIS_VESSEL") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_AIS_VESSEL) != NULL);
#endif
}

static void mavlink_test_uavcan_node_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_UAVCAN_NODE_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_uavcan_node_status_t packet_in = {
        93372036854775807ULL,963497880,17859,175,242,53
    };
    mavlink_uavcan_node_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.uptime_sec = packet_in.uptime_sec;
        packet1.vendor_specific_status_code = packet_in.vendor_specific_status_code;
        packet1.health = packet_in.health;
        packet1.mode = packet_in.mode;
        packet1.sub_mode = packet_in.sub_mode;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uavcan_node_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_uavcan_node_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uavcan_node_status_pack(system_id, component_id, &msg , packet1.time_usec , packet1.uptime_sec , packet1.health , packet1.mode , packet1.sub_mode , packet1.vendor_specific_status_code );
    mavlink_msg_uavcan_node_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uavcan_node_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.uptime_sec , packet1.health , packet1.mode , packet1.sub_mode , packet1.vendor_specific_status_code );
    mavlink_msg_uavcan_node_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_uavcan_node_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uavcan_node_status_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.uptime_sec , packet1.health , packet1.mode , packet1.sub_mode , packet1.vendor_specific_status_code );
    mavlink_msg_uavcan_node_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("UAVCAN_NODE_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_UAVCAN_NODE_STATUS) != NULL);
#endif
}

static void mavlink_test_uavcan_node_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_UAVCAN_NODE_INFO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_uavcan_node_info_t packet_in = {
        93372036854775807ULL,963497880,963498088,"QRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQ",37,104,{ 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186 },219,30
    };
    mavlink_uavcan_node_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.uptime_sec = packet_in.uptime_sec;
        packet1.sw_vcs_commit = packet_in.sw_vcs_commit;
        packet1.hw_version_major = packet_in.hw_version_major;
        packet1.hw_version_minor = packet_in.hw_version_minor;
        packet1.sw_version_major = packet_in.sw_version_major;
        packet1.sw_version_minor = packet_in.sw_version_minor;
        
        mav_array_memcpy(packet1.name, packet_in.name, sizeof(char)*80);
        mav_array_memcpy(packet1.hw_unique_id, packet_in.hw_unique_id, sizeof(uint8_t)*16);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_UAVCAN_NODE_INFO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_UAVCAN_NODE_INFO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uavcan_node_info_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_uavcan_node_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uavcan_node_info_pack(system_id, component_id, &msg , packet1.time_usec , packet1.uptime_sec , packet1.name , packet1.hw_version_major , packet1.hw_version_minor , packet1.hw_unique_id , packet1.sw_version_major , packet1.sw_version_minor , packet1.sw_vcs_commit );
    mavlink_msg_uavcan_node_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uavcan_node_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.uptime_sec , packet1.name , packet1.hw_version_major , packet1.hw_version_minor , packet1.hw_unique_id , packet1.sw_version_major , packet1.sw_version_minor , packet1.sw_vcs_commit );
    mavlink_msg_uavcan_node_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_uavcan_node_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uavcan_node_info_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.uptime_sec , packet1.name , packet1.hw_version_major , packet1.hw_version_minor , packet1.hw_unique_id , packet1.sw_version_major , packet1.sw_version_minor , packet1.sw_vcs_commit );
    mavlink_msg_uavcan_node_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("UAVCAN_NODE_INFO") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_UAVCAN_NODE_INFO) != NULL);
#endif
}

static void mavlink_test_param_ext_request_read(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_param_ext_request_read_t packet_in = {
        17235,139,206,"EFGHIJKLMNOPQRS"
    };
    mavlink_param_ext_request_read_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.param_index = packet_in.param_index;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        mav_array_memcpy(packet1.param_id, packet_in.param_id, sizeof(char)*16);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_ext_request_read_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_param_ext_request_read_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_ext_request_read_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.param_id , packet1.param_index );
    mavlink_msg_param_ext_request_read_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_ext_request_read_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.param_id , packet1.param_index );
    mavlink_msg_param_ext_request_read_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_param_ext_request_read_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_ext_request_read_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.param_id , packet1.param_index );
    mavlink_msg_param_ext_request_read_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("PARAM_EXT_REQUEST_READ") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ) != NULL);
#endif
}

static void mavlink_test_param_ext_request_list(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_param_ext_request_list_t packet_in = {
        5,72
    };
    mavlink_param_ext_request_list_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_ext_request_list_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_param_ext_request_list_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_ext_request_list_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component );
    mavlink_msg_param_ext_request_list_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_ext_request_list_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component );
    mavlink_msg_param_ext_request_list_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_param_ext_request_list_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_ext_request_list_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component );
    mavlink_msg_param_ext_request_list_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("PARAM_EXT_REQUEST_LIST") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST) != NULL);
#endif
}

static void mavlink_test_param_ext_value(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PARAM_EXT_VALUE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_param_ext_value_t packet_in = {
        17235,17339,"EFGHIJKLMNOPQRS","UVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQ",193
    };
    mavlink_param_ext_value_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.param_count = packet_in.param_count;
        packet1.param_index = packet_in.param_index;
        packet1.param_type = packet_in.param_type;
        
        mav_array_memcpy(packet1.param_id, packet_in.param_id, sizeof(char)*16);
        mav_array_memcpy(packet1.param_value, packet_in.param_value, sizeof(char)*128);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PARAM_EXT_VALUE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PARAM_EXT_VALUE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_ext_value_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_param_ext_value_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_ext_value_pack(system_id, component_id, &msg , packet1.param_id , packet1.param_value , packet1.param_type , packet1.param_count , packet1.param_index );
    mavlink_msg_param_ext_value_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_ext_value_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.param_id , packet1.param_value , packet1.param_type , packet1.param_count , packet1.param_index );
    mavlink_msg_param_ext_value_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_param_ext_value_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_ext_value_send(MAVLINK_COMM_1 , packet1.param_id , packet1.param_value , packet1.param_type , packet1.param_count , packet1.param_index );
    mavlink_msg_param_ext_value_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("PARAM_EXT_VALUE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_PARAM_EXT_VALUE) != NULL);
#endif
}

static void mavlink_test_param_ext_set(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PARAM_EXT_SET >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_param_ext_set_t packet_in = {
        5,72,"CDEFGHIJKLMNOPQ","STUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNO",59
    };
    mavlink_param_ext_set_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.param_type = packet_in.param_type;
        
        mav_array_memcpy(packet1.param_id, packet_in.param_id, sizeof(char)*16);
        mav_array_memcpy(packet1.param_value, packet_in.param_value, sizeof(char)*128);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PARAM_EXT_SET_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PARAM_EXT_SET_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_ext_set_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_param_ext_set_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_ext_set_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.param_id , packet1.param_value , packet1.param_type );
    mavlink_msg_param_ext_set_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_ext_set_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.param_id , packet1.param_value , packet1.param_type );
    mavlink_msg_param_ext_set_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_param_ext_set_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_ext_set_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.param_id , packet1.param_value , packet1.param_type );
    mavlink_msg_param_ext_set_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("PARAM_EXT_SET") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_PARAM_EXT_SET) != NULL);
#endif
}

static void mavlink_test_param_ext_ack(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PARAM_EXT_ACK >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_param_ext_ack_t packet_in = {
        "ABCDEFGHIJKLMNO","QRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLM",181,248
    };
    mavlink_param_ext_ack_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.param_type = packet_in.param_type;
        packet1.param_result = packet_in.param_result;
        
        mav_array_memcpy(packet1.param_id, packet_in.param_id, sizeof(char)*16);
        mav_array_memcpy(packet1.param_value, packet_in.param_value, sizeof(char)*128);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PARAM_EXT_ACK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PARAM_EXT_ACK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_ext_ack_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_param_ext_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_ext_ack_pack(system_id, component_id, &msg , packet1.param_id , packet1.param_value , packet1.param_type , packet1.param_result );
    mavlink_msg_param_ext_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_ext_ack_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.param_id , packet1.param_value , packet1.param_type , packet1.param_result );
    mavlink_msg_param_ext_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_param_ext_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_ext_ack_send(MAVLINK_COMM_1 , packet1.param_id , packet1.param_value , packet1.param_type , packet1.param_result );
    mavlink_msg_param_ext_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("PARAM_EXT_ACK") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_PARAM_EXT_ACK) != NULL);
#endif
}

static void mavlink_test_obstacle_distance(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_OBSTACLE_DISTANCE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_obstacle_distance_t packet_in = {
        93372036854775807ULL,{ 17651, 17652, 17653, 17654, 17655, 17656, 17657, 17658, 17659, 17660, 17661, 17662, 17663, 17664, 17665, 17666, 17667, 17668, 17669, 17670, 17671, 17672, 17673, 17674, 17675, 17676, 17677, 17678, 17679, 17680, 17681, 17682, 17683, 17684, 17685, 17686, 17687, 17688, 17689, 17690, 17691, 17692, 17693, 17694, 17695, 17696, 17697, 17698, 17699, 17700, 17701, 17702, 17703, 17704, 17705, 17706, 17707, 17708, 17709, 17710, 17711, 17712, 17713, 17714, 17715, 17716, 17717, 17718, 17719, 17720, 17721, 17722 },25139,25243,217,28,1123.0,1151.0,119
    };
    mavlink_obstacle_distance_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.min_distance = packet_in.min_distance;
        packet1.max_distance = packet_in.max_distance;
        packet1.sensor_type = packet_in.sensor_type;
        packet1.increment = packet_in.increment;
        packet1.increment_f = packet_in.increment_f;
        packet1.angle_offset = packet_in.angle_offset;
        packet1.frame = packet_in.frame;
        
        mav_array_memcpy(packet1.distances, packet_in.distances, sizeof(uint16_t)*72);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_OBSTACLE_DISTANCE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_OBSTACLE_DISTANCE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_obstacle_distance_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_obstacle_distance_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_obstacle_distance_pack(system_id, component_id, &msg , packet1.time_usec , packet1.sensor_type , packet1.distances , packet1.increment , packet1.min_distance , packet1.max_distance , packet1.increment_f , packet1.angle_offset , packet1.frame );
    mavlink_msg_obstacle_distance_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_obstacle_distance_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.sensor_type , packet1.distances , packet1.increment , packet1.min_distance , packet1.max_distance , packet1.increment_f , packet1.angle_offset , packet1.frame );
    mavlink_msg_obstacle_distance_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_obstacle_distance_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_obstacle_distance_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.sensor_type , packet1.distances , packet1.increment , packet1.min_distance , packet1.max_distance , packet1.increment_f , packet1.angle_offset , packet1.frame );
    mavlink_msg_obstacle_distance_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("OBSTACLE_DISTANCE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_OBSTACLE_DISTANCE) != NULL);
#endif
}

static void mavlink_test_odometry(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ODOMETRY >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_odometry_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,{ 157.0, 158.0, 159.0, 160.0 },269.0,297.0,325.0,353.0,381.0,409.0,{ 437.0, 438.0, 439.0, 440.0, 441.0, 442.0, 443.0, 444.0, 445.0, 446.0, 447.0, 448.0, 449.0, 450.0, 451.0, 452.0, 453.0, 454.0, 455.0, 456.0, 457.0 },{ 1025.0, 1026.0, 1027.0, 1028.0, 1029.0, 1030.0, 1031.0, 1032.0, 1033.0, 1034.0, 1035.0, 1036.0, 1037.0, 1038.0, 1039.0, 1040.0, 1041.0, 1042.0, 1043.0, 1044.0, 1045.0 },177,244,55,122,189
    };
    mavlink_odometry_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.vx = packet_in.vx;
        packet1.vy = packet_in.vy;
        packet1.vz = packet_in.vz;
        packet1.rollspeed = packet_in.rollspeed;
        packet1.pitchspeed = packet_in.pitchspeed;
        packet1.yawspeed = packet_in.yawspeed;
        packet1.frame_id = packet_in.frame_id;
        packet1.child_frame_id = packet_in.child_frame_id;
        packet1.reset_counter = packet_in.reset_counter;
        packet1.estimator_type = packet_in.estimator_type;
        packet1.quality = packet_in.quality;
        
        mav_array_memcpy(packet1.q, packet_in.q, sizeof(float)*4);
        mav_array_memcpy(packet1.pose_covariance, packet_in.pose_covariance, sizeof(float)*21);
        mav_array_memcpy(packet1.velocity_covariance, packet_in.velocity_covariance, sizeof(float)*21);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ODOMETRY_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ODOMETRY_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_odometry_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_odometry_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_odometry_pack(system_id, component_id, &msg , packet1.time_usec , packet1.frame_id , packet1.child_frame_id , packet1.x , packet1.y , packet1.z , packet1.q , packet1.vx , packet1.vy , packet1.vz , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed , packet1.pose_covariance , packet1.velocity_covariance , packet1.reset_counter , packet1.estimator_type , packet1.quality );
    mavlink_msg_odometry_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_odometry_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.frame_id , packet1.child_frame_id , packet1.x , packet1.y , packet1.z , packet1.q , packet1.vx , packet1.vy , packet1.vz , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed , packet1.pose_covariance , packet1.velocity_covariance , packet1.reset_counter , packet1.estimator_type , packet1.quality );
    mavlink_msg_odometry_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_odometry_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_odometry_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.frame_id , packet1.child_frame_id , packet1.x , packet1.y , packet1.z , packet1.q , packet1.vx , packet1.vy , packet1.vz , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed , packet1.pose_covariance , packet1.velocity_covariance , packet1.reset_counter , packet1.estimator_type , packet1.quality );
    mavlink_msg_odometry_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ODOMETRY") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ODOMETRY) != NULL);
#endif
}

static void mavlink_test_trajectory_representation_waypoints(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_trajectory_representation_waypoints_t packet_in = {
        93372036854775807ULL,{ 73.0, 74.0, 75.0, 76.0, 77.0 },{ 213.0, 214.0, 215.0, 216.0, 217.0 },{ 353.0, 354.0, 355.0, 356.0, 357.0 },{ 493.0, 494.0, 495.0, 496.0, 497.0 },{ 633.0, 634.0, 635.0, 636.0, 637.0 },{ 773.0, 774.0, 775.0, 776.0, 777.0 },{ 913.0, 914.0, 915.0, 916.0, 917.0 },{ 1053.0, 1054.0, 1055.0, 1056.0, 1057.0 },{ 1193.0, 1194.0, 1195.0, 1196.0, 1197.0 },{ 1333.0, 1334.0, 1335.0, 1336.0, 1337.0 },{ 1473.0, 1474.0, 1475.0, 1476.0, 1477.0 },{ 29091, 29092, 29093, 29094, 29095 },79
    };
    mavlink_trajectory_representation_waypoints_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.valid_points = packet_in.valid_points;
        
        mav_array_memcpy(packet1.pos_x, packet_in.pos_x, sizeof(float)*5);
        mav_array_memcpy(packet1.pos_y, packet_in.pos_y, sizeof(float)*5);
        mav_array_memcpy(packet1.pos_z, packet_in.pos_z, sizeof(float)*5);
        mav_array_memcpy(packet1.vel_x, packet_in.vel_x, sizeof(float)*5);
        mav_array_memcpy(packet1.vel_y, packet_in.vel_y, sizeof(float)*5);
        mav_array_memcpy(packet1.vel_z, packet_in.vel_z, sizeof(float)*5);
        mav_array_memcpy(packet1.acc_x, packet_in.acc_x, sizeof(float)*5);
        mav_array_memcpy(packet1.acc_y, packet_in.acc_y, sizeof(float)*5);
        mav_array_memcpy(packet1.acc_z, packet_in.acc_z, sizeof(float)*5);
        mav_array_memcpy(packet1.pos_yaw, packet_in.pos_yaw, sizeof(float)*5);
        mav_array_memcpy(packet1.vel_yaw, packet_in.vel_yaw, sizeof(float)*5);
        mav_array_memcpy(packet1.command, packet_in.command, sizeof(uint16_t)*5);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_trajectory_representation_waypoints_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_trajectory_representation_waypoints_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_trajectory_representation_waypoints_pack(system_id, component_id, &msg , packet1.time_usec , packet1.valid_points , packet1.pos_x , packet1.pos_y , packet1.pos_z , packet1.vel_x , packet1.vel_y , packet1.vel_z , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.pos_yaw , packet1.vel_yaw , packet1.command );
    mavlink_msg_trajectory_representation_waypoints_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_trajectory_representation_waypoints_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.valid_points , packet1.pos_x , packet1.pos_y , packet1.pos_z , packet1.vel_x , packet1.vel_y , packet1.vel_z , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.pos_yaw , packet1.vel_yaw , packet1.command );
    mavlink_msg_trajectory_representation_waypoints_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_trajectory_representation_waypoints_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_trajectory_representation_waypoints_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.valid_points , packet1.pos_x , packet1.pos_y , packet1.pos_z , packet1.vel_x , packet1.vel_y , packet1.vel_z , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.pos_yaw , packet1.vel_yaw , packet1.command );
    mavlink_msg_trajectory_representation_waypoints_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("TRAJECTORY_REPRESENTATION_WAYPOINTS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS) != NULL);
#endif
}

static void mavlink_test_trajectory_representation_bezier(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_BEZIER >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_trajectory_representation_bezier_t packet_in = {
        93372036854775807ULL,{ 73.0, 74.0, 75.0, 76.0, 77.0 },{ 213.0, 214.0, 215.0, 216.0, 217.0 },{ 353.0, 354.0, 355.0, 356.0, 357.0 },{ 493.0, 494.0, 495.0, 496.0, 497.0 },{ 633.0, 634.0, 635.0, 636.0, 637.0 },73
    };
    mavlink_trajectory_representation_bezier_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.valid_points = packet_in.valid_points;
        
        mav_array_memcpy(packet1.pos_x, packet_in.pos_x, sizeof(float)*5);
        mav_array_memcpy(packet1.pos_y, packet_in.pos_y, sizeof(float)*5);
        mav_array_memcpy(packet1.pos_z, packet_in.pos_z, sizeof(float)*5);
        mav_array_memcpy(packet1.delta, packet_in.delta, sizeof(float)*5);
        mav_array_memcpy(packet1.pos_yaw, packet_in.pos_yaw, sizeof(float)*5);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_BEZIER_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_BEZIER_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_trajectory_representation_bezier_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_trajectory_representation_bezier_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_trajectory_representation_bezier_pack(system_id, component_id, &msg , packet1.time_usec , packet1.valid_points , packet1.pos_x , packet1.pos_y , packet1.pos_z , packet1.delta , packet1.pos_yaw );
    mavlink_msg_trajectory_representation_bezier_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_trajectory_representation_bezier_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.valid_points , packet1.pos_x , packet1.pos_y , packet1.pos_z , packet1.delta , packet1.pos_yaw );
    mavlink_msg_trajectory_representation_bezier_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_trajectory_representation_bezier_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_trajectory_representation_bezier_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.valid_points , packet1.pos_x , packet1.pos_y , packet1.pos_z , packet1.delta , packet1.pos_yaw );
    mavlink_msg_trajectory_representation_bezier_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("TRAJECTORY_REPRESENTATION_BEZIER") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_BEZIER) != NULL);
#endif
}

static void mavlink_test_isbd_link_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ISBD_LINK_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_isbd_link_status_t packet_in = {
        93372036854775807ULL,93372036854776311ULL,18067,18171,65,132,199,10
    };
    mavlink_isbd_link_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.last_heartbeat = packet_in.last_heartbeat;
        packet1.failed_sessions = packet_in.failed_sessions;
        packet1.successful_sessions = packet_in.successful_sessions;
        packet1.signal_quality = packet_in.signal_quality;
        packet1.ring_pending = packet_in.ring_pending;
        packet1.tx_session_pending = packet_in.tx_session_pending;
        packet1.rx_session_pending = packet_in.rx_session_pending;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ISBD_LINK_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ISBD_LINK_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_isbd_link_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_isbd_link_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_isbd_link_status_pack(system_id, component_id, &msg , packet1.timestamp , packet1.last_heartbeat , packet1.failed_sessions , packet1.successful_sessions , packet1.signal_quality , packet1.ring_pending , packet1.tx_session_pending , packet1.rx_session_pending );
    mavlink_msg_isbd_link_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_isbd_link_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.last_heartbeat , packet1.failed_sessions , packet1.successful_sessions , packet1.signal_quality , packet1.ring_pending , packet1.tx_session_pending , packet1.rx_session_pending );
    mavlink_msg_isbd_link_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_isbd_link_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_isbd_link_status_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.last_heartbeat , packet1.failed_sessions , packet1.successful_sessions , packet1.signal_quality , packet1.ring_pending , packet1.tx_session_pending , packet1.rx_session_pending );
    mavlink_msg_isbd_link_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ISBD_LINK_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ISBD_LINK_STATUS) != NULL);
#endif
}

static void mavlink_test_raw_rpm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RAW_RPM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_raw_rpm_t packet_in = {
        17.0,17
    };
    mavlink_raw_rpm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.frequency = packet_in.frequency;
        packet1.index = packet_in.index;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_RAW_RPM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RAW_RPM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_raw_rpm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_raw_rpm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_raw_rpm_pack(system_id, component_id, &msg , packet1.index , packet1.frequency );
    mavlink_msg_raw_rpm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_raw_rpm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.index , packet1.frequency );
    mavlink_msg_raw_rpm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_raw_rpm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_raw_rpm_send(MAVLINK_COMM_1 , packet1.index , packet1.frequency );
    mavlink_msg_raw_rpm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("RAW_RPM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_RAW_RPM) != NULL);
#endif
}

static void mavlink_test_utm_global_position(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_UTM_GLOBAL_POSITION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_utm_global_position_t packet_in = {
        93372036854775807ULL,963497880,963498088,963498296,963498504,963498712,963498920,963499128,19107,19211,19315,19419,19523,19627,19731,{ 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44 },209,20
    };
    mavlink_utm_global_position_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time = packet_in.time;
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        packet1.alt = packet_in.alt;
        packet1.relative_alt = packet_in.relative_alt;
        packet1.next_lat = packet_in.next_lat;
        packet1.next_lon = packet_in.next_lon;
        packet1.next_alt = packet_in.next_alt;
        packet1.vx = packet_in.vx;
        packet1.vy = packet_in.vy;
        packet1.vz = packet_in.vz;
        packet1.h_acc = packet_in.h_acc;
        packet1.v_acc = packet_in.v_acc;
        packet1.vel_acc = packet_in.vel_acc;
        packet1.update_rate = packet_in.update_rate;
        packet1.flight_state = packet_in.flight_state;
        packet1.flags = packet_in.flags;
        
        mav_array_memcpy(packet1.uas_id, packet_in.uas_id, sizeof(uint8_t)*18);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_UTM_GLOBAL_POSITION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_UTM_GLOBAL_POSITION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_utm_global_position_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_utm_global_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_utm_global_position_pack(system_id, component_id, &msg , packet1.time , packet1.uas_id , packet1.lat , packet1.lon , packet1.alt , packet1.relative_alt , packet1.vx , packet1.vy , packet1.vz , packet1.h_acc , packet1.v_acc , packet1.vel_acc , packet1.next_lat , packet1.next_lon , packet1.next_alt , packet1.update_rate , packet1.flight_state , packet1.flags );
    mavlink_msg_utm_global_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_utm_global_position_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time , packet1.uas_id , packet1.lat , packet1.lon , packet1.alt , packet1.relative_alt , packet1.vx , packet1.vy , packet1.vz , packet1.h_acc , packet1.v_acc , packet1.vel_acc , packet1.next_lat , packet1.next_lon , packet1.next_alt , packet1.update_rate , packet1.flight_state , packet1.flags );
    mavlink_msg_utm_global_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_utm_global_position_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_utm_global_position_send(MAVLINK_COMM_1 , packet1.time , packet1.uas_id , packet1.lat , packet1.lon , packet1.alt , packet1.relative_alt , packet1.vx , packet1.vy , packet1.vz , packet1.h_acc , packet1.v_acc , packet1.vel_acc , packet1.next_lat , packet1.next_lon , packet1.next_alt , packet1.update_rate , packet1.flight_state , packet1.flags );
    mavlink_msg_utm_global_position_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("UTM_GLOBAL_POSITION") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_UTM_GLOBAL_POSITION) != NULL);
#endif
}

static void mavlink_test_debug_float_array(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_debug_float_array_t packet_in = {
        93372036854775807ULL,17651,"KLMNOPQRS",{ 157.0, 158.0, 159.0, 160.0, 161.0, 162.0, 163.0, 164.0, 165.0, 166.0, 167.0, 168.0, 169.0, 170.0, 171.0, 172.0, 173.0, 174.0, 175.0, 176.0, 177.0, 178.0, 179.0, 180.0, 181.0, 182.0, 183.0, 184.0, 185.0, 186.0, 187.0, 188.0, 189.0, 190.0, 191.0, 192.0, 193.0, 194.0, 195.0, 196.0, 197.0, 198.0, 199.0, 200.0, 201.0, 202.0, 203.0, 204.0, 205.0, 206.0, 207.0, 208.0, 209.0, 210.0, 211.0, 212.0, 213.0, 214.0 }
    };
    mavlink_debug_float_array_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.array_id = packet_in.array_id;
        
        mav_array_memcpy(packet1.name, packet_in.name, sizeof(char)*10);
        mav_array_memcpy(packet1.data, packet_in.data, sizeof(float)*58);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_debug_float_array_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_debug_float_array_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_debug_float_array_pack(system_id, component_id, &msg , packet1.time_usec , packet1.name , packet1.array_id , packet1.data );
    mavlink_msg_debug_float_array_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_debug_float_array_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.name , packet1.array_id , packet1.data );
    mavlink_msg_debug_float_array_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_debug_float_array_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_debug_float_array_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.name , packet1.array_id , packet1.data );
    mavlink_msg_debug_float_array_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("DEBUG_FLOAT_ARRAY") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY) != NULL);
#endif
}

static void mavlink_test_smart_battery_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SMART_BATTERY_INFO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_smart_battery_info_t packet_in = {
        963497464,963497672,17651,17755,17859,17963,18067,187,254,65,"VWXYZABCDEFGHIJ","LMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGH",21759,80,963502144,963502352,"UVWXYZABCD"
    };
    mavlink_smart_battery_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.capacity_full_specification = packet_in.capacity_full_specification;
        packet1.capacity_full = packet_in.capacity_full;
        packet1.cycle_count = packet_in.cycle_count;
        packet1.weight = packet_in.weight;
        packet1.discharge_minimum_voltage = packet_in.discharge_minimum_voltage;
        packet1.charging_minimum_voltage = packet_in.charging_minimum_voltage;
        packet1.resting_minimum_voltage = packet_in.resting_minimum_voltage;
        packet1.id = packet_in.id;
        packet1.battery_function = packet_in.battery_function;
        packet1.type = packet_in.type;
        packet1.charging_maximum_voltage = packet_in.charging_maximum_voltage;
        packet1.cells_in_series = packet_in.cells_in_series;
        packet1.discharge_maximum_current = packet_in.discharge_maximum_current;
        packet1.discharge_maximum_burst_current = packet_in.discharge_maximum_burst_current;
        
        mav_array_memcpy(packet1.serial_number, packet_in.serial_number, sizeof(char)*16);
        mav_array_memcpy(packet1.device_name, packet_in.device_name, sizeof(char)*50);
        mav_array_memcpy(packet1.manufacture_date, packet_in.manufacture_date, sizeof(char)*11);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SMART_BATTERY_INFO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SMART_BATTERY_INFO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_smart_battery_info_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_smart_battery_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_smart_battery_info_pack(system_id, component_id, &msg , packet1.id , packet1.battery_function , packet1.type , packet1.capacity_full_specification , packet1.capacity_full , packet1.cycle_count , packet1.serial_number , packet1.device_name , packet1.weight , packet1.discharge_minimum_voltage , packet1.charging_minimum_voltage , packet1.resting_minimum_voltage , packet1.charging_maximum_voltage , packet1.cells_in_series , packet1.discharge_maximum_current , packet1.discharge_maximum_burst_current , packet1.manufacture_date );
    mavlink_msg_smart_battery_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_smart_battery_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.id , packet1.battery_function , packet1.type , packet1.capacity_full_specification , packet1.capacity_full , packet1.cycle_count , packet1.serial_number , packet1.device_name , packet1.weight , packet1.discharge_minimum_voltage , packet1.charging_minimum_voltage , packet1.resting_minimum_voltage , packet1.charging_maximum_voltage , packet1.cells_in_series , packet1.discharge_maximum_current , packet1.discharge_maximum_burst_current , packet1.manufacture_date );
    mavlink_msg_smart_battery_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_smart_battery_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_smart_battery_info_send(MAVLINK_COMM_1 , packet1.id , packet1.battery_function , packet1.type , packet1.capacity_full_specification , packet1.capacity_full , packet1.cycle_count , packet1.serial_number , packet1.device_name , packet1.weight , packet1.discharge_minimum_voltage , packet1.charging_minimum_voltage , packet1.resting_minimum_voltage , packet1.charging_maximum_voltage , packet1.cells_in_series , packet1.discharge_maximum_current , packet1.discharge_maximum_burst_current , packet1.manufacture_date );
    mavlink_msg_smart_battery_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SMART_BATTERY_INFO") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SMART_BATTERY_INFO) != NULL);
#endif
}

static void mavlink_test_generator_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GENERATOR_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_generator_status_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,963498920,963499128,19107,19211,19315
    };
    mavlink_generator_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.status = packet_in.status;
        packet1.battery_current = packet_in.battery_current;
        packet1.load_current = packet_in.load_current;
        packet1.power_generated = packet_in.power_generated;
        packet1.bus_voltage = packet_in.bus_voltage;
        packet1.bat_current_setpoint = packet_in.bat_current_setpoint;
        packet1.runtime = packet_in.runtime;
        packet1.time_until_maintenance = packet_in.time_until_maintenance;
        packet1.generator_speed = packet_in.generator_speed;
        packet1.rectifier_temperature = packet_in.rectifier_temperature;
        packet1.generator_temperature = packet_in.generator_temperature;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GENERATOR_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GENERATOR_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_generator_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_generator_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_generator_status_pack(system_id, component_id, &msg , packet1.status , packet1.generator_speed , packet1.battery_current , packet1.load_current , packet1.power_generated , packet1.bus_voltage , packet1.rectifier_temperature , packet1.bat_current_setpoint , packet1.generator_temperature , packet1.runtime , packet1.time_until_maintenance );
    mavlink_msg_generator_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_generator_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.status , packet1.generator_speed , packet1.battery_current , packet1.load_current , packet1.power_generated , packet1.bus_voltage , packet1.rectifier_temperature , packet1.bat_current_setpoint , packet1.generator_temperature , packet1.runtime , packet1.time_until_maintenance );
    mavlink_msg_generator_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_generator_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_generator_status_send(MAVLINK_COMM_1 , packet1.status , packet1.generator_speed , packet1.battery_current , packet1.load_current , packet1.power_generated , packet1.bus_voltage , packet1.rectifier_temperature , packet1.bat_current_setpoint , packet1.generator_temperature , packet1.runtime , packet1.time_until_maintenance );
    mavlink_msg_generator_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("GENERATOR_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_GENERATOR_STATUS) != NULL);
#endif
}

static void mavlink_test_actuator_output_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_actuator_output_status_t packet_in = {
        93372036854775807ULL,963497880,{ 101.0, 102.0, 103.0, 104.0, 105.0, 106.0, 107.0, 108.0, 109.0, 110.0, 111.0, 112.0, 113.0, 114.0, 115.0, 116.0, 117.0, 118.0, 119.0, 120.0, 121.0, 122.0, 123.0, 124.0, 125.0, 126.0, 127.0, 128.0, 129.0, 130.0, 131.0, 132.0 }
    };
    mavlink_actuator_output_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.active = packet_in.active;
        
        mav_array_memcpy(packet1.actuator, packet_in.actuator, sizeof(float)*32);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_actuator_output_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_actuator_output_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_actuator_output_status_pack(system_id, component_id, &msg , packet1.time_usec , packet1.active , packet1.actuator );
    mavlink_msg_actuator_output_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_actuator_output_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.active , packet1.actuator );
    mavlink_msg_actuator_output_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_actuator_output_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_actuator_output_status_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.active , packet1.actuator );
    mavlink_msg_actuator_output_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ACTUATOR_OUTPUT_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS) != NULL);
#endif
}

static void mavlink_test_relay_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RELAY_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_relay_status_t packet_in = {
        963497464,17443,17547
    };
    mavlink_relay_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.on = packet_in.on;
        packet1.present = packet_in.present;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_RELAY_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RELAY_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_relay_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_relay_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_relay_status_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.on , packet1.present );
    mavlink_msg_relay_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_relay_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.on , packet1.present );
    mavlink_msg_relay_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_relay_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_relay_status_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.on , packet1.present );
    mavlink_msg_relay_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("RELAY_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_RELAY_STATUS) != NULL);
#endif
}

static void mavlink_test_tunnel(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TUNNEL >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_tunnel_t packet_in = {
        17235,139,206,17,{ 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211 }
    };
    mavlink_tunnel_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.payload_type = packet_in.payload_type;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.payload_length = packet_in.payload_length;
        
        mav_array_memcpy(packet1.payload, packet_in.payload, sizeof(uint8_t)*128);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TUNNEL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TUNNEL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tunnel_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_tunnel_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tunnel_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.payload_type , packet1.payload_length , packet1.payload );
    mavlink_msg_tunnel_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tunnel_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.payload_type , packet1.payload_length , packet1.payload );
    mavlink_msg_tunnel_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_tunnel_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tunnel_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.payload_type , packet1.payload_length , packet1.payload );
    mavlink_msg_tunnel_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("TUNNEL") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_TUNNEL) != NULL);
#endif
}

static void mavlink_test_can_frame(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CAN_FRAME >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_can_frame_t packet_in = {
        963497464,17,84,151,218,{ 29, 30, 31, 32, 33, 34, 35, 36 }
    };
    mavlink_can_frame_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.id = packet_in.id;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.bus = packet_in.bus;
        packet1.len = packet_in.len;
        
        mav_array_memcpy(packet1.data, packet_in.data, sizeof(uint8_t)*8);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CAN_FRAME_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CAN_FRAME_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_can_frame_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_can_frame_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_can_frame_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.bus , packet1.len , packet1.id , packet1.data );
    mavlink_msg_can_frame_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_can_frame_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.bus , packet1.len , packet1.id , packet1.data );
    mavlink_msg_can_frame_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_can_frame_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_can_frame_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.bus , packet1.len , packet1.id , packet1.data );
    mavlink_msg_can_frame_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CAN_FRAME") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CAN_FRAME) != NULL);
#endif
}

static void mavlink_test_canfd_frame(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CANFD_FRAME >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_canfd_frame_t packet_in = {
        963497464,17,84,151,218,{ 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92 }
    };
    mavlink_canfd_frame_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.id = packet_in.id;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.bus = packet_in.bus;
        packet1.len = packet_in.len;
        
        mav_array_memcpy(packet1.data, packet_in.data, sizeof(uint8_t)*64);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CANFD_FRAME_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CANFD_FRAME_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_canfd_frame_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_canfd_frame_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_canfd_frame_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.bus , packet1.len , packet1.id , packet1.data );
    mavlink_msg_canfd_frame_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_canfd_frame_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.bus , packet1.len , packet1.id , packet1.data );
    mavlink_msg_canfd_frame_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_canfd_frame_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_canfd_frame_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.bus , packet1.len , packet1.id , packet1.data );
    mavlink_msg_canfd_frame_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CANFD_FRAME") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CANFD_FRAME) != NULL);
#endif
}

static void mavlink_test_can_filter_modify(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CAN_FILTER_MODIFY >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_can_filter_modify_t packet_in = {
        { 17235, 17236, 17237, 17238, 17239, 17240, 17241, 17242, 17243, 17244, 17245, 17246, 17247, 17248, 17249, 17250 },101,168,235,46,113
    };
    mavlink_can_filter_modify_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.bus = packet_in.bus;
        packet1.operation = packet_in.operation;
        packet1.num_ids = packet_in.num_ids;
        
        mav_array_memcpy(packet1.ids, packet_in.ids, sizeof(uint16_t)*16);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CAN_FILTER_MODIFY_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CAN_FILTER_MODIFY_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_can_filter_modify_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_can_filter_modify_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_can_filter_modify_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.bus , packet1.operation , packet1.num_ids , packet1.ids );
    mavlink_msg_can_filter_modify_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_can_filter_modify_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.bus , packet1.operation , packet1.num_ids , packet1.ids );
    mavlink_msg_can_filter_modify_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_can_filter_modify_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_can_filter_modify_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.bus , packet1.operation , packet1.num_ids , packet1.ids );
    mavlink_msg_can_filter_modify_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CAN_FILTER_MODIFY") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CAN_FILTER_MODIFY) != NULL);
#endif
}

static void mavlink_test_wheel_distance(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_WHEEL_DISTANCE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_wheel_distance_t packet_in = {
        93372036854775807ULL,{ 179.0, 180.0, 181.0, 182.0, 183.0, 184.0, 185.0, 186.0, 187.0, 188.0, 189.0, 190.0, 191.0, 192.0, 193.0, 194.0 },157
    };
    mavlink_wheel_distance_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.count = packet_in.count;
        
        mav_array_memcpy(packet1.distance, packet_in.distance, sizeof(double)*16);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_WHEEL_DISTANCE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_WHEEL_DISTANCE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wheel_distance_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_wheel_distance_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wheel_distance_pack(system_id, component_id, &msg , packet1.time_usec , packet1.count , packet1.distance );
    mavlink_msg_wheel_distance_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wheel_distance_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.count , packet1.distance );
    mavlink_msg_wheel_distance_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_wheel_distance_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wheel_distance_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.count , packet1.distance );
    mavlink_msg_wheel_distance_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("WHEEL_DISTANCE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_WHEEL_DISTANCE) != NULL);
#endif
}

static void mavlink_test_winch_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_WINCH_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_winch_status_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,963498920,18899
    };
    mavlink_winch_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.line_length = packet_in.line_length;
        packet1.speed = packet_in.speed;
        packet1.tension = packet_in.tension;
        packet1.voltage = packet_in.voltage;
        packet1.current = packet_in.current;
        packet1.status = packet_in.status;
        packet1.temperature = packet_in.temperature;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_WINCH_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_WINCH_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_winch_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_winch_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_winch_status_pack(system_id, component_id, &msg , packet1.time_usec , packet1.line_length , packet1.speed , packet1.tension , packet1.voltage , packet1.current , packet1.temperature , packet1.status );
    mavlink_msg_winch_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_winch_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.line_length , packet1.speed , packet1.tension , packet1.voltage , packet1.current , packet1.temperature , packet1.status );
    mavlink_msg_winch_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_winch_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_winch_status_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.line_length , packet1.speed , packet1.tension , packet1.voltage , packet1.current , packet1.temperature , packet1.status );
    mavlink_msg_winch_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("WINCH_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_WINCH_STATUS) != NULL);
#endif
}

static void mavlink_test_open_drone_id_basic_id(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_OPEN_DRONE_ID_BASIC_ID >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_open_drone_id_basic_id_t packet_in = {
        5,72,{ 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158 },199,10,{ 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96 }
    };
    mavlink_open_drone_id_basic_id_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.id_type = packet_in.id_type;
        packet1.ua_type = packet_in.ua_type;
        
        mav_array_memcpy(packet1.id_or_mac, packet_in.id_or_mac, sizeof(uint8_t)*20);
        mav_array_memcpy(packet1.uas_id, packet_in.uas_id, sizeof(uint8_t)*20);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_OPEN_DRONE_ID_BASIC_ID_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_OPEN_DRONE_ID_BASIC_ID_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_basic_id_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_open_drone_id_basic_id_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_basic_id_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.id_or_mac , packet1.id_type , packet1.ua_type , packet1.uas_id );
    mavlink_msg_open_drone_id_basic_id_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_basic_id_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.id_or_mac , packet1.id_type , packet1.ua_type , packet1.uas_id );
    mavlink_msg_open_drone_id_basic_id_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_open_drone_id_basic_id_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_basic_id_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.id_or_mac , packet1.id_type , packet1.ua_type , packet1.uas_id );
    mavlink_msg_open_drone_id_basic_id_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("OPEN_DRONE_ID_BASIC_ID") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_OPEN_DRONE_ID_BASIC_ID) != NULL);
#endif
}

static void mavlink_test_open_drone_id_location(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_open_drone_id_location_t packet_in = {
        963497464,963497672,73.0,101.0,129.0,157.0,18483,18587,18691,223,34,{ 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120 },161,228,39,106,173,240,51
    };
    mavlink_open_drone_id_location_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.latitude = packet_in.latitude;
        packet1.longitude = packet_in.longitude;
        packet1.altitude_barometric = packet_in.altitude_barometric;
        packet1.altitude_geodetic = packet_in.altitude_geodetic;
        packet1.height = packet_in.height;
        packet1.timestamp = packet_in.timestamp;
        packet1.direction = packet_in.direction;
        packet1.speed_horizontal = packet_in.speed_horizontal;
        packet1.speed_vertical = packet_in.speed_vertical;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.status = packet_in.status;
        packet1.height_reference = packet_in.height_reference;
        packet1.horizontal_accuracy = packet_in.horizontal_accuracy;
        packet1.vertical_accuracy = packet_in.vertical_accuracy;
        packet1.barometer_accuracy = packet_in.barometer_accuracy;
        packet1.speed_accuracy = packet_in.speed_accuracy;
        packet1.timestamp_accuracy = packet_in.timestamp_accuracy;
        
        mav_array_memcpy(packet1.id_or_mac, packet_in.id_or_mac, sizeof(uint8_t)*20);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_location_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_open_drone_id_location_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_location_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.id_or_mac , packet1.status , packet1.direction , packet1.speed_horizontal , packet1.speed_vertical , packet1.latitude , packet1.longitude , packet1.altitude_barometric , packet1.altitude_geodetic , packet1.height_reference , packet1.height , packet1.horizontal_accuracy , packet1.vertical_accuracy , packet1.barometer_accuracy , packet1.speed_accuracy , packet1.timestamp , packet1.timestamp_accuracy );
    mavlink_msg_open_drone_id_location_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_location_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.id_or_mac , packet1.status , packet1.direction , packet1.speed_horizontal , packet1.speed_vertical , packet1.latitude , packet1.longitude , packet1.altitude_barometric , packet1.altitude_geodetic , packet1.height_reference , packet1.height , packet1.horizontal_accuracy , packet1.vertical_accuracy , packet1.barometer_accuracy , packet1.speed_accuracy , packet1.timestamp , packet1.timestamp_accuracy );
    mavlink_msg_open_drone_id_location_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_open_drone_id_location_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_location_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.id_or_mac , packet1.status , packet1.direction , packet1.speed_horizontal , packet1.speed_vertical , packet1.latitude , packet1.longitude , packet1.altitude_barometric , packet1.altitude_geodetic , packet1.height_reference , packet1.height , packet1.horizontal_accuracy , packet1.vertical_accuracy , packet1.barometer_accuracy , packet1.speed_accuracy , packet1.timestamp , packet1.timestamp_accuracy );
    mavlink_msg_open_drone_id_location_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("OPEN_DRONE_ID_LOCATION") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION) != NULL);
#endif
}

static void mavlink_test_open_drone_id_authentication(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_open_drone_id_authentication_t packet_in = {
        963497464,17,84,{ 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170 },211,22,89,156,{ 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245 }
    };
    mavlink_open_drone_id_authentication_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.authentication_type = packet_in.authentication_type;
        packet1.data_page = packet_in.data_page;
        packet1.last_page_index = packet_in.last_page_index;
        packet1.length = packet_in.length;
        
        mav_array_memcpy(packet1.id_or_mac, packet_in.id_or_mac, sizeof(uint8_t)*20);
        mav_array_memcpy(packet1.authentication_data, packet_in.authentication_data, sizeof(uint8_t)*23);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_authentication_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_open_drone_id_authentication_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_authentication_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.id_or_mac , packet1.authentication_type , packet1.data_page , packet1.last_page_index , packet1.length , packet1.timestamp , packet1.authentication_data );
    mavlink_msg_open_drone_id_authentication_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_authentication_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.id_or_mac , packet1.authentication_type , packet1.data_page , packet1.last_page_index , packet1.length , packet1.timestamp , packet1.authentication_data );
    mavlink_msg_open_drone_id_authentication_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_open_drone_id_authentication_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_authentication_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.id_or_mac , packet1.authentication_type , packet1.data_page , packet1.last_page_index , packet1.length , packet1.timestamp , packet1.authentication_data );
    mavlink_msg_open_drone_id_authentication_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("OPEN_DRONE_ID_AUTHENTICATION") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION) != NULL);
#endif
}

static void mavlink_test_open_drone_id_self_id(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_OPEN_DRONE_ID_SELF_ID >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_open_drone_id_self_id_t packet_in = {
        5,72,{ 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158 },199,"XYZABCDEFGHIJKLMNOPQRS"
    };
    mavlink_open_drone_id_self_id_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.description_type = packet_in.description_type;
        
        mav_array_memcpy(packet1.id_or_mac, packet_in.id_or_mac, sizeof(uint8_t)*20);
        mav_array_memcpy(packet1.description, packet_in.description, sizeof(char)*23);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_OPEN_DRONE_ID_SELF_ID_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_OPEN_DRONE_ID_SELF_ID_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_self_id_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_open_drone_id_self_id_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_self_id_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.id_or_mac , packet1.description_type , packet1.description );
    mavlink_msg_open_drone_id_self_id_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_self_id_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.id_or_mac , packet1.description_type , packet1.description );
    mavlink_msg_open_drone_id_self_id_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_open_drone_id_self_id_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_self_id_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.id_or_mac , packet1.description_type , packet1.description );
    mavlink_msg_open_drone_id_self_id_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("OPEN_DRONE_ID_SELF_ID") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_OPEN_DRONE_ID_SELF_ID) != NULL);
#endif
}

static void mavlink_test_open_drone_id_system(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_open_drone_id_system_t packet_in = {
        963497464,963497672,73.0,101.0,129.0,963498504,18483,18587,89,156,{ 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242 },27,94,161,228
    };
    mavlink_open_drone_id_system_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.operator_latitude = packet_in.operator_latitude;
        packet1.operator_longitude = packet_in.operator_longitude;
        packet1.area_ceiling = packet_in.area_ceiling;
        packet1.area_floor = packet_in.area_floor;
        packet1.operator_altitude_geo = packet_in.operator_altitude_geo;
        packet1.timestamp = packet_in.timestamp;
        packet1.area_count = packet_in.area_count;
        packet1.area_radius = packet_in.area_radius;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.operator_location_type = packet_in.operator_location_type;
        packet1.classification_type = packet_in.classification_type;
        packet1.category_eu = packet_in.category_eu;
        packet1.class_eu = packet_in.class_eu;
        
        mav_array_memcpy(packet1.id_or_mac, packet_in.id_or_mac, sizeof(uint8_t)*20);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_system_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_open_drone_id_system_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_system_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.id_or_mac , packet1.operator_location_type , packet1.classification_type , packet1.operator_latitude , packet1.operator_longitude , packet1.area_count , packet1.area_radius , packet1.area_ceiling , packet1.area_floor , packet1.category_eu , packet1.class_eu , packet1.operator_altitude_geo , packet1.timestamp );
    mavlink_msg_open_drone_id_system_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_system_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.id_or_mac , packet1.operator_location_type , packet1.classification_type , packet1.operator_latitude , packet1.operator_longitude , packet1.area_count , packet1.area_radius , packet1.area_ceiling , packet1.area_floor , packet1.category_eu , packet1.class_eu , packet1.operator_altitude_geo , packet1.timestamp );
    mavlink_msg_open_drone_id_system_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_open_drone_id_system_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_system_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.id_or_mac , packet1.operator_location_type , packet1.classification_type , packet1.operator_latitude , packet1.operator_longitude , packet1.area_count , packet1.area_radius , packet1.area_ceiling , packet1.area_floor , packet1.category_eu , packet1.class_eu , packet1.operator_altitude_geo , packet1.timestamp );
    mavlink_msg_open_drone_id_system_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("OPEN_DRONE_ID_SYSTEM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM) != NULL);
#endif
}

static void mavlink_test_open_drone_id_operator_id(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_open_drone_id_operator_id_t packet_in = {
        5,72,{ 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158 },199,"XYZABCDEFGHIJKLMNOP"
    };
    mavlink_open_drone_id_operator_id_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.operator_id_type = packet_in.operator_id_type;
        
        mav_array_memcpy(packet1.id_or_mac, packet_in.id_or_mac, sizeof(uint8_t)*20);
        mav_array_memcpy(packet1.operator_id, packet_in.operator_id, sizeof(char)*20);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_operator_id_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_open_drone_id_operator_id_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_operator_id_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.id_or_mac , packet1.operator_id_type , packet1.operator_id );
    mavlink_msg_open_drone_id_operator_id_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_operator_id_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.id_or_mac , packet1.operator_id_type , packet1.operator_id );
    mavlink_msg_open_drone_id_operator_id_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_open_drone_id_operator_id_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_operator_id_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.id_or_mac , packet1.operator_id_type , packet1.operator_id );
    mavlink_msg_open_drone_id_operator_id_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("OPEN_DRONE_ID_OPERATOR_ID") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID) != NULL);
#endif
}

static void mavlink_test_open_drone_id_arm_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_open_drone_id_arm_status_t packet_in = {
        5,"BCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWX"
    };
    mavlink_open_drone_id_arm_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.status = packet_in.status;
        
        mav_array_memcpy(packet1.error, packet_in.error, sizeof(char)*50);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_arm_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_open_drone_id_arm_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_arm_status_pack(system_id, component_id, &msg , packet1.status , packet1.error );
    mavlink_msg_open_drone_id_arm_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_arm_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.status , packet1.error );
    mavlink_msg_open_drone_id_arm_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_open_drone_id_arm_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_arm_status_send(MAVLINK_COMM_1 , packet1.status , packet1.error );
    mavlink_msg_open_drone_id_arm_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("OPEN_DRONE_ID_ARM_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS) != NULL);
#endif
}

static void mavlink_test_open_drone_id_message_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_open_drone_id_message_pack_t packet_in = {
        5,72,{ 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158 },199,10,{ 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45 }
    };
    mavlink_open_drone_id_message_pack_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.single_message_size = packet_in.single_message_size;
        packet1.msg_pack_size = packet_in.msg_pack_size;
        
        mav_array_memcpy(packet1.id_or_mac, packet_in.id_or_mac, sizeof(uint8_t)*20);
        mav_array_memcpy(packet1.messages, packet_in.messages, sizeof(uint8_t)*225);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_message_pack_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_open_drone_id_message_pack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_message_pack_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.id_or_mac , packet1.single_message_size , packet1.msg_pack_size , packet1.messages );
    mavlink_msg_open_drone_id_message_pack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_message_pack_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.id_or_mac , packet1.single_message_size , packet1.msg_pack_size , packet1.messages );
    mavlink_msg_open_drone_id_message_pack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_open_drone_id_message_pack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_message_pack_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.id_or_mac , packet1.single_message_size , packet1.msg_pack_size , packet1.messages );
    mavlink_msg_open_drone_id_message_pack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("OPEN_DRONE_ID_MESSAGE_PACK") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK) != NULL);
#endif
}

static void mavlink_test_open_drone_id_system_update(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_open_drone_id_system_update_t packet_in = {
        963497464,963497672,73.0,963498088,53,120
    };
    mavlink_open_drone_id_system_update_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.operator_latitude = packet_in.operator_latitude;
        packet1.operator_longitude = packet_in.operator_longitude;
        packet1.operator_altitude_geo = packet_in.operator_altitude_geo;
        packet1.timestamp = packet_in.timestamp;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_system_update_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_open_drone_id_system_update_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_system_update_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.operator_latitude , packet1.operator_longitude , packet1.operator_altitude_geo , packet1.timestamp );
    mavlink_msg_open_drone_id_system_update_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_system_update_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.operator_latitude , packet1.operator_longitude , packet1.operator_altitude_geo , packet1.timestamp );
    mavlink_msg_open_drone_id_system_update_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_open_drone_id_system_update_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_open_drone_id_system_update_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.operator_latitude , packet1.operator_longitude , packet1.operator_altitude_geo , packet1.timestamp );
    mavlink_msg_open_drone_id_system_update_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("OPEN_DRONE_ID_SYSTEM_UPDATE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE) != NULL);
#endif
}

static void mavlink_test_hygrometer_sensor(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_HYGROMETER_SENSOR >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_hygrometer_sensor_t packet_in = {
        17235,17339,17
    };
    mavlink_hygrometer_sensor_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.temperature = packet_in.temperature;
        packet1.humidity = packet_in.humidity;
        packet1.id = packet_in.id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_HYGROMETER_SENSOR_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_HYGROMETER_SENSOR_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hygrometer_sensor_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_hygrometer_sensor_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hygrometer_sensor_pack(system_id, component_id, &msg , packet1.id , packet1.temperature , packet1.humidity );
    mavlink_msg_hygrometer_sensor_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hygrometer_sensor_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.id , packet1.temperature , packet1.humidity );
    mavlink_msg_hygrometer_sensor_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_hygrometer_sensor_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hygrometer_sensor_send(MAVLINK_COMM_1 , packet1.id , packet1.temperature , packet1.humidity );
    mavlink_msg_hygrometer_sensor_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("HYGROMETER_SENSOR") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_HYGROMETER_SENSOR) != NULL);
#endif
}

static void mavlink_test_common(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_sys_status(system_id, component_id, last_msg);
    mavlink_test_system_time(system_id, component_id, last_msg);
    mavlink_test_ping(system_id, component_id, last_msg);
    mavlink_test_change_operator_control(system_id, component_id, last_msg);
    mavlink_test_change_operator_control_ack(system_id, component_id, last_msg);
    mavlink_test_auth_key(system_id, component_id, last_msg);
    mavlink_test_set_mode(system_id, component_id, last_msg);
    mavlink_test_param_request_read(system_id, component_id, last_msg);
    mavlink_test_param_request_list(system_id, component_id, last_msg);
    mavlink_test_param_value(system_id, component_id, last_msg);
    mavlink_test_param_set(system_id, component_id, last_msg);
    mavlink_test_gps_raw_int(system_id, component_id, last_msg);
    mavlink_test_gps_status(system_id, component_id, last_msg);
    mavlink_test_scaled_imu(system_id, component_id, last_msg);
    mavlink_test_raw_imu(system_id, component_id, last_msg);
    mavlink_test_raw_pressure(system_id, component_id, last_msg);
    mavlink_test_scaled_pressure(system_id, component_id, last_msg);
    mavlink_test_attitude(system_id, component_id, last_msg);
    mavlink_test_attitude_quaternion(system_id, component_id, last_msg);
    mavlink_test_local_position_ned(system_id, component_id, last_msg);
    mavlink_test_global_position_int(system_id, component_id, last_msg);
    mavlink_test_rc_channels_scaled(system_id, component_id, last_msg);
    mavlink_test_rc_channels_raw(system_id, component_id, last_msg);
    mavlink_test_servo_output_raw(system_id, component_id, last_msg);
    mavlink_test_mission_request_partial_list(system_id, component_id, last_msg);
    mavlink_test_mission_write_partial_list(system_id, component_id, last_msg);
    mavlink_test_mission_item(system_id, component_id, last_msg);
    mavlink_test_mission_request(system_id, component_id, last_msg);
    mavlink_test_mission_set_current(system_id, component_id, last_msg);
    mavlink_test_mission_current(system_id, component_id, last_msg);
    mavlink_test_mission_request_list(system_id, component_id, last_msg);
    mavlink_test_mission_count(system_id, component_id, last_msg);
    mavlink_test_mission_clear_all(system_id, component_id, last_msg);
    mavlink_test_mission_item_reached(system_id, component_id, last_msg);
    mavlink_test_mission_ack(system_id, component_id, last_msg);
    mavlink_test_set_gps_global_origin(system_id, component_id, last_msg);
    mavlink_test_gps_global_origin(system_id, component_id, last_msg);
    mavlink_test_param_map_rc(system_id, component_id, last_msg);
    mavlink_test_mission_request_int(system_id, component_id, last_msg);
    mavlink_test_safety_set_allowed_area(system_id, component_id, last_msg);
    mavlink_test_safety_allowed_area(system_id, component_id, last_msg);
    mavlink_test_attitude_quaternion_cov(system_id, component_id, last_msg);
    mavlink_test_nav_controller_output(system_id, component_id, last_msg);
    mavlink_test_global_position_int_cov(system_id, component_id, last_msg);
    mavlink_test_local_position_ned_cov(system_id, component_id, last_msg);
    mavlink_test_rc_channels(system_id, component_id, last_msg);
    mavlink_test_request_data_stream(system_id, component_id, last_msg);
    mavlink_test_data_stream(system_id, component_id, last_msg);
    mavlink_test_manual_control(system_id, component_id, last_msg);
    mavlink_test_rc_channels_override(system_id, component_id, last_msg);
    mavlink_test_mission_item_int(system_id, component_id, last_msg);
    mavlink_test_vfr_hud(system_id, component_id, last_msg);
    mavlink_test_command_int(system_id, component_id, last_msg);
    mavlink_test_command_long(system_id, component_id, last_msg);
    mavlink_test_command_ack(system_id, component_id, last_msg);
    mavlink_test_manual_setpoint(system_id, component_id, last_msg);
    mavlink_test_set_attitude_target(system_id, component_id, last_msg);
    mavlink_test_attitude_target(system_id, component_id, last_msg);
    mavlink_test_set_position_target_local_ned(system_id, component_id, last_msg);
    mavlink_test_position_target_local_ned(system_id, component_id, last_msg);
    mavlink_test_set_position_target_global_int(system_id, component_id, last_msg);
    mavlink_test_position_target_global_int(system_id, component_id, last_msg);
    mavlink_test_local_position_ned_system_global_offset(system_id, component_id, last_msg);
    mavlink_test_hil_state(system_id, component_id, last_msg);
    mavlink_test_hil_controls(system_id, component_id, last_msg);
    mavlink_test_hil_rc_inputs_raw(system_id, component_id, last_msg);
    mavlink_test_hil_actuator_controls(system_id, component_id, last_msg);
    mavlink_test_optical_flow(system_id, component_id, last_msg);
    mavlink_test_global_vision_position_estimate(system_id, component_id, last_msg);
    mavlink_test_vision_position_estimate(system_id, component_id, last_msg);
    mavlink_test_vision_speed_estimate(system_id, component_id, last_msg);
    mavlink_test_vicon_position_estimate(system_id, component_id, last_msg);
    mavlink_test_highres_imu(system_id, component_id, last_msg);
    mavlink_test_optical_flow_rad(system_id, component_id, last_msg);
    mavlink_test_hil_sensor(system_id, component_id, last_msg);
    mavlink_test_sim_state(system_id, component_id, last_msg);
    mavlink_test_radio_status(system_id, component_id, last_msg);
    mavlink_test_file_transfer_protocol(system_id, component_id, last_msg);
    mavlink_test_timesync(system_id, component_id, last_msg);
    mavlink_test_camera_trigger(system_id, component_id, last_msg);
    mavlink_test_hil_gps(system_id, component_id, last_msg);
    mavlink_test_hil_optical_flow(system_id, component_id, last_msg);
    mavlink_test_hil_state_quaternion(system_id, component_id, last_msg);
    mavlink_test_scaled_imu2(system_id, component_id, last_msg);
    mavlink_test_log_request_list(system_id, component_id, last_msg);
    mavlink_test_log_entry(system_id, component_id, last_msg);
    mavlink_test_log_request_data(system_id, component_id, last_msg);
    mavlink_test_log_data(system_id, component_id, last_msg);
    mavlink_test_log_erase(system_id, component_id, last_msg);
    mavlink_test_log_request_end(system_id, component_id, last_msg);
    mavlink_test_gps_inject_data(system_id, component_id, last_msg);
    mavlink_test_gps2_raw(system_id, component_id, last_msg);
    mavlink_test_power_status(system_id, component_id, last_msg);
    mavlink_test_serial_control(system_id, component_id, last_msg);
    mavlink_test_gps_rtk(system_id, component_id, last_msg);
    mavlink_test_gps2_rtk(system_id, component_id, last_msg);
    mavlink_test_scaled_imu3(system_id, component_id, last_msg);
    mavlink_test_data_transmission_handshake(system_id, component_id, last_msg);
    mavlink_test_encapsulated_data(system_id, component_id, last_msg);
    mavlink_test_distance_sensor(system_id, component_id, last_msg);
    mavlink_test_terrain_request(system_id, component_id, last_msg);
    mavlink_test_terrain_data(system_id, component_id, last_msg);
    mavlink_test_terrain_check(system_id, component_id, last_msg);
    mavlink_test_terrain_report(system_id, component_id, last_msg);
    mavlink_test_scaled_pressure2(system_id, component_id, last_msg);
    mavlink_test_att_pos_mocap(system_id, component_id, last_msg);
    mavlink_test_set_actuator_control_target(system_id, component_id, last_msg);
    mavlink_test_actuator_control_target(system_id, component_id, last_msg);
    mavlink_test_altitude(system_id, component_id, last_msg);
    mavlink_test_resource_request(system_id, component_id, last_msg);
    mavlink_test_scaled_pressure3(system_id, component_id, last_msg);
    mavlink_test_follow_target(system_id, component_id, last_msg);
    mavlink_test_control_system_state(system_id, component_id, last_msg);
    mavlink_test_battery_status(system_id, component_id, last_msg);
    mavlink_test_autopilot_version(system_id, component_id, last_msg);
    mavlink_test_landing_target(system_id, component_id, last_msg);
    mavlink_test_fence_status(system_id, component_id, last_msg);
    mavlink_test_mag_cal_report(system_id, component_id, last_msg);
    mavlink_test_efi_status(system_id, component_id, last_msg);
    mavlink_test_estimator_status(system_id, component_id, last_msg);
    mavlink_test_wind_cov(system_id, component_id, last_msg);
    mavlink_test_gps_input(system_id, component_id, last_msg);
    mavlink_test_gps_rtcm_data(system_id, component_id, last_msg);
    mavlink_test_high_latency(system_id, component_id, last_msg);
    mavlink_test_high_latency2(system_id, component_id, last_msg);
    mavlink_test_vibration(system_id, component_id, last_msg);
    mavlink_test_home_position(system_id, component_id, last_msg);
    mavlink_test_set_home_position(system_id, component_id, last_msg);
    mavlink_test_message_interval(system_id, component_id, last_msg);
    mavlink_test_extended_sys_state(system_id, component_id, last_msg);
    mavlink_test_adsb_vehicle(system_id, component_id, last_msg);
    mavlink_test_collision(system_id, component_id, last_msg);
    mavlink_test_v2_extension(system_id, component_id, last_msg);
    mavlink_test_memory_vect(system_id, component_id, last_msg);
    mavlink_test_debug_vect(system_id, component_id, last_msg);
    mavlink_test_named_value_float(system_id, component_id, last_msg);
    mavlink_test_named_value_int(system_id, component_id, last_msg);
    mavlink_test_statustext(system_id, component_id, last_msg);
    mavlink_test_debug(system_id, component_id, last_msg);
    mavlink_test_setup_signing(system_id, component_id, last_msg);
    mavlink_test_button_change(system_id, component_id, last_msg);
    mavlink_test_play_tune(system_id, component_id, last_msg);
    mavlink_test_camera_information(system_id, component_id, last_msg);
    mavlink_test_camera_settings(system_id, component_id, last_msg);
    mavlink_test_storage_information(system_id, component_id, last_msg);
    mavlink_test_camera_capture_status(system_id, component_id, last_msg);
    mavlink_test_camera_image_captured(system_id, component_id, last_msg);
    mavlink_test_flight_information(system_id, component_id, last_msg);
    mavlink_test_mount_orientation(system_id, component_id, last_msg);
    mavlink_test_logging_data(system_id, component_id, last_msg);
    mavlink_test_logging_data_acked(system_id, component_id, last_msg);
    mavlink_test_logging_ack(system_id, component_id, last_msg);
    mavlink_test_video_stream_information(system_id, component_id, last_msg);
    mavlink_test_video_stream_status(system_id, component_id, last_msg);
    mavlink_test_camera_fov_status(system_id, component_id, last_msg);
    mavlink_test_camera_tracking_image_status(system_id, component_id, last_msg);
    mavlink_test_camera_tracking_geo_status(system_id, component_id, last_msg);
    mavlink_test_camera_thermal_range(system_id, component_id, last_msg);
    mavlink_test_gimbal_manager_information(system_id, component_id, last_msg);
    mavlink_test_gimbal_manager_status(system_id, component_id, last_msg);
    mavlink_test_gimbal_manager_set_attitude(system_id, component_id, last_msg);
    mavlink_test_gimbal_device_information(system_id, component_id, last_msg);
    mavlink_test_gimbal_device_set_attitude(system_id, component_id, last_msg);
    mavlink_test_gimbal_device_attitude_status(system_id, component_id, last_msg);
    mavlink_test_autopilot_state_for_gimbal_device(system_id, component_id, last_msg);
    mavlink_test_gimbal_manager_set_pitchyaw(system_id, component_id, last_msg);
    mavlink_test_gimbal_manager_set_manual_control(system_id, component_id, last_msg);
    mavlink_test_wifi_config_ap(system_id, component_id, last_msg);
    mavlink_test_ais_vessel(system_id, component_id, last_msg);
    mavlink_test_uavcan_node_status(system_id, component_id, last_msg);
    mavlink_test_uavcan_node_info(system_id, component_id, last_msg);
    mavlink_test_param_ext_request_read(system_id, component_id, last_msg);
    mavlink_test_param_ext_request_list(system_id, component_id, last_msg);
    mavlink_test_param_ext_value(system_id, component_id, last_msg);
    mavlink_test_param_ext_set(system_id, component_id, last_msg);
    mavlink_test_param_ext_ack(system_id, component_id, last_msg);
    mavlink_test_obstacle_distance(system_id, component_id, last_msg);
    mavlink_test_odometry(system_id, component_id, last_msg);
    mavlink_test_trajectory_representation_waypoints(system_id, component_id, last_msg);
    mavlink_test_trajectory_representation_bezier(system_id, component_id, last_msg);
    mavlink_test_isbd_link_status(system_id, component_id, last_msg);
    mavlink_test_raw_rpm(system_id, component_id, last_msg);
    mavlink_test_utm_global_position(system_id, component_id, last_msg);
    mavlink_test_debug_float_array(system_id, component_id, last_msg);
    mavlink_test_smart_battery_info(system_id, component_id, last_msg);
    mavlink_test_generator_status(system_id, component_id, last_msg);
    mavlink_test_actuator_output_status(system_id, component_id, last_msg);
    mavlink_test_relay_status(system_id, component_id, last_msg);
    mavlink_test_tunnel(system_id, component_id, last_msg);
    mavlink_test_can_frame(system_id, component_id, last_msg);
    mavlink_test_canfd_frame(system_id, component_id, last_msg);
    mavlink_test_can_filter_modify(system_id, component_id, last_msg);
    mavlink_test_wheel_distance(system_id, component_id, last_msg);
    mavlink_test_winch_status(system_id, component_id, last_msg);
    mavlink_test_open_drone_id_basic_id(system_id, component_id, last_msg);
    mavlink_test_open_drone_id_location(system_id, component_id, last_msg);
    mavlink_test_open_drone_id_authentication(system_id, component_id, last_msg);
    mavlink_test_open_drone_id_self_id(system_id, component_id, last_msg);
    mavlink_test_open_drone_id_system(system_id, component_id, last_msg);
    mavlink_test_open_drone_id_operator_id(system_id, component_id, last_msg);
    mavlink_test_open_drone_id_arm_status(system_id, component_id, last_msg);
    mavlink_test_open_drone_id_message_pack(system_id, component_id, last_msg);
    mavlink_test_open_drone_id_system_update(system_id, component_id, last_msg);
    mavlink_test_hygrometer_sensor(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // COMMON_TESTSUITE_H
