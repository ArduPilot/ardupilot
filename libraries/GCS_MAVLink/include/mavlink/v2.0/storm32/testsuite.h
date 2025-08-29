/** @file
 *    @brief MAVLink comm protocol testsuite generated from storm32.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef STORM32_TESTSUITE_H
#define STORM32_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_ardupilotmega(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_storm32(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_ardupilotmega(system_id, component_id, last_msg);
    mavlink_test_storm32(system_id, component_id, last_msg);
}
#endif

#include "../ardupilotmega/testsuite.h"


static void mavlink_test_storm32_gimbal_device_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_storm32_gimbal_device_status_t packet_in = {
        963497464,{ 45.0, 46.0, 47.0, 48.0 },157.0,185.0,213.0,241.0,19107,19211,125,192
    };
    mavlink_storm32_gimbal_device_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.angular_velocity_x = packet_in.angular_velocity_x;
        packet1.angular_velocity_y = packet_in.angular_velocity_y;
        packet1.angular_velocity_z = packet_in.angular_velocity_z;
        packet1.yaw_absolute = packet_in.yaw_absolute;
        packet1.flags = packet_in.flags;
        packet1.failure_flags = packet_in.failure_flags;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        mav_array_memcpy(packet1.q, packet_in.q, sizeof(float)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storm32_gimbal_device_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_storm32_gimbal_device_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storm32_gimbal_device_status_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.time_boot_ms , packet1.flags , packet1.q , packet1.angular_velocity_x , packet1.angular_velocity_y , packet1.angular_velocity_z , packet1.yaw_absolute , packet1.failure_flags );
    mavlink_msg_storm32_gimbal_device_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storm32_gimbal_device_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.time_boot_ms , packet1.flags , packet1.q , packet1.angular_velocity_x , packet1.angular_velocity_y , packet1.angular_velocity_z , packet1.yaw_absolute , packet1.failure_flags );
    mavlink_msg_storm32_gimbal_device_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_storm32_gimbal_device_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storm32_gimbal_device_status_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.time_boot_ms , packet1.flags , packet1.q , packet1.angular_velocity_x , packet1.angular_velocity_y , packet1.angular_velocity_z , packet1.yaw_absolute , packet1.failure_flags );
    mavlink_msg_storm32_gimbal_device_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("STORM32_GIMBAL_DEVICE_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS) != NULL);
#endif
}

static void mavlink_test_storm32_gimbal_device_control(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_CONTROL >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_storm32_gimbal_device_control_t packet_in = {
        { 17.0, 18.0, 19.0, 20.0 },129.0,157.0,185.0,18691,223,34
    };
    mavlink_storm32_gimbal_device_control_t packet1, packet2;
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
           memset(MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_CONTROL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_CONTROL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storm32_gimbal_device_control_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_storm32_gimbal_device_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storm32_gimbal_device_control_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.flags , packet1.q , packet1.angular_velocity_x , packet1.angular_velocity_y , packet1.angular_velocity_z );
    mavlink_msg_storm32_gimbal_device_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storm32_gimbal_device_control_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.flags , packet1.q , packet1.angular_velocity_x , packet1.angular_velocity_y , packet1.angular_velocity_z );
    mavlink_msg_storm32_gimbal_device_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_storm32_gimbal_device_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storm32_gimbal_device_control_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.flags , packet1.q , packet1.angular_velocity_x , packet1.angular_velocity_y , packet1.angular_velocity_z );
    mavlink_msg_storm32_gimbal_device_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("STORM32_GIMBAL_DEVICE_CONTROL") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_CONTROL) != NULL);
#endif
}

static void mavlink_test_storm32_gimbal_manager_information(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_storm32_gimbal_manager_information_t packet_in = {
        963497464,963497672,73.0,101.0,129.0,157.0,185.0,213.0,101
    };
    mavlink_storm32_gimbal_manager_information_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.device_cap_flags = packet_in.device_cap_flags;
        packet1.manager_cap_flags = packet_in.manager_cap_flags;
        packet1.roll_min = packet_in.roll_min;
        packet1.roll_max = packet_in.roll_max;
        packet1.pitch_min = packet_in.pitch_min;
        packet1.pitch_max = packet_in.pitch_max;
        packet1.yaw_min = packet_in.yaw_min;
        packet1.yaw_max = packet_in.yaw_max;
        packet1.gimbal_id = packet_in.gimbal_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storm32_gimbal_manager_information_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_storm32_gimbal_manager_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storm32_gimbal_manager_information_pack(system_id, component_id, &msg , packet1.gimbal_id , packet1.device_cap_flags , packet1.manager_cap_flags , packet1.roll_min , packet1.roll_max , packet1.pitch_min , packet1.pitch_max , packet1.yaw_min , packet1.yaw_max );
    mavlink_msg_storm32_gimbal_manager_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storm32_gimbal_manager_information_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.gimbal_id , packet1.device_cap_flags , packet1.manager_cap_flags , packet1.roll_min , packet1.roll_max , packet1.pitch_min , packet1.pitch_max , packet1.yaw_min , packet1.yaw_max );
    mavlink_msg_storm32_gimbal_manager_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_storm32_gimbal_manager_information_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storm32_gimbal_manager_information_send(MAVLINK_COMM_1 , packet1.gimbal_id , packet1.device_cap_flags , packet1.manager_cap_flags , packet1.roll_min , packet1.roll_max , packet1.pitch_min , packet1.pitch_max , packet1.yaw_min , packet1.yaw_max );
    mavlink_msg_storm32_gimbal_manager_information_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("STORM32_GIMBAL_MANAGER_INFORMATION") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION) != NULL);
#endif
}

static void mavlink_test_storm32_gimbal_manager_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_storm32_gimbal_manager_status_t packet_in = {
        17235,17339,17,84,151
    };
    mavlink_storm32_gimbal_manager_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.device_flags = packet_in.device_flags;
        packet1.manager_flags = packet_in.manager_flags;
        packet1.gimbal_id = packet_in.gimbal_id;
        packet1.supervisor = packet_in.supervisor;
        packet1.profile = packet_in.profile;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storm32_gimbal_manager_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_storm32_gimbal_manager_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storm32_gimbal_manager_status_pack(system_id, component_id, &msg , packet1.gimbal_id , packet1.supervisor , packet1.device_flags , packet1.manager_flags , packet1.profile );
    mavlink_msg_storm32_gimbal_manager_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storm32_gimbal_manager_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.gimbal_id , packet1.supervisor , packet1.device_flags , packet1.manager_flags , packet1.profile );
    mavlink_msg_storm32_gimbal_manager_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_storm32_gimbal_manager_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storm32_gimbal_manager_status_send(MAVLINK_COMM_1 , packet1.gimbal_id , packet1.supervisor , packet1.device_flags , packet1.manager_flags , packet1.profile );
    mavlink_msg_storm32_gimbal_manager_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("STORM32_GIMBAL_MANAGER_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS) != NULL);
#endif
}

static void mavlink_test_storm32_gimbal_manager_control(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_storm32_gimbal_manager_control_t packet_in = {
        { 17.0, 18.0, 19.0, 20.0 },129.0,157.0,185.0,18691,18795,101,168,235,46
    };
    mavlink_storm32_gimbal_manager_control_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.angular_velocity_x = packet_in.angular_velocity_x;
        packet1.angular_velocity_y = packet_in.angular_velocity_y;
        packet1.angular_velocity_z = packet_in.angular_velocity_z;
        packet1.device_flags = packet_in.device_flags;
        packet1.manager_flags = packet_in.manager_flags;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.gimbal_id = packet_in.gimbal_id;
        packet1.client = packet_in.client;
        
        mav_array_memcpy(packet1.q, packet_in.q, sizeof(float)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storm32_gimbal_manager_control_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_storm32_gimbal_manager_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storm32_gimbal_manager_control_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.gimbal_id , packet1.client , packet1.device_flags , packet1.manager_flags , packet1.q , packet1.angular_velocity_x , packet1.angular_velocity_y , packet1.angular_velocity_z );
    mavlink_msg_storm32_gimbal_manager_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storm32_gimbal_manager_control_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.gimbal_id , packet1.client , packet1.device_flags , packet1.manager_flags , packet1.q , packet1.angular_velocity_x , packet1.angular_velocity_y , packet1.angular_velocity_z );
    mavlink_msg_storm32_gimbal_manager_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_storm32_gimbal_manager_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storm32_gimbal_manager_control_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.gimbal_id , packet1.client , packet1.device_flags , packet1.manager_flags , packet1.q , packet1.angular_velocity_x , packet1.angular_velocity_y , packet1.angular_velocity_z );
    mavlink_msg_storm32_gimbal_manager_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("STORM32_GIMBAL_MANAGER_CONTROL") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL) != NULL);
#endif
}

static void mavlink_test_storm32_gimbal_manager_control_pitchyaw(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_storm32_gimbal_manager_control_pitchyaw_t packet_in = {
        17.0,45.0,73.0,101.0,18067,18171,65,132,199,10
    };
    mavlink_storm32_gimbal_manager_control_pitchyaw_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.pitch = packet_in.pitch;
        packet1.yaw = packet_in.yaw;
        packet1.pitch_rate = packet_in.pitch_rate;
        packet1.yaw_rate = packet_in.yaw_rate;
        packet1.device_flags = packet_in.device_flags;
        packet1.manager_flags = packet_in.manager_flags;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.gimbal_id = packet_in.gimbal_id;
        packet1.client = packet_in.client;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storm32_gimbal_manager_control_pitchyaw_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_storm32_gimbal_manager_control_pitchyaw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storm32_gimbal_manager_control_pitchyaw_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.gimbal_id , packet1.client , packet1.device_flags , packet1.manager_flags , packet1.pitch , packet1.yaw , packet1.pitch_rate , packet1.yaw_rate );
    mavlink_msg_storm32_gimbal_manager_control_pitchyaw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storm32_gimbal_manager_control_pitchyaw_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.gimbal_id , packet1.client , packet1.device_flags , packet1.manager_flags , packet1.pitch , packet1.yaw , packet1.pitch_rate , packet1.yaw_rate );
    mavlink_msg_storm32_gimbal_manager_control_pitchyaw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_storm32_gimbal_manager_control_pitchyaw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storm32_gimbal_manager_control_pitchyaw_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.gimbal_id , packet1.client , packet1.device_flags , packet1.manager_flags , packet1.pitch , packet1.yaw , packet1.pitch_rate , packet1.yaw_rate );
    mavlink_msg_storm32_gimbal_manager_control_pitchyaw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW) != NULL);
#endif
}

static void mavlink_test_storm32_gimbal_manager_correct_roll(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_storm32_gimbal_manager_correct_roll_t packet_in = {
        17.0,17,84,151,218
    };
    mavlink_storm32_gimbal_manager_correct_roll_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.roll = packet_in.roll;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.gimbal_id = packet_in.gimbal_id;
        packet1.client = packet_in.client;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storm32_gimbal_manager_correct_roll_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_storm32_gimbal_manager_correct_roll_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storm32_gimbal_manager_correct_roll_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.gimbal_id , packet1.client , packet1.roll );
    mavlink_msg_storm32_gimbal_manager_correct_roll_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storm32_gimbal_manager_correct_roll_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.gimbal_id , packet1.client , packet1.roll );
    mavlink_msg_storm32_gimbal_manager_correct_roll_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_storm32_gimbal_manager_correct_roll_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storm32_gimbal_manager_correct_roll_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.gimbal_id , packet1.client , packet1.roll );
    mavlink_msg_storm32_gimbal_manager_correct_roll_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("STORM32_GIMBAL_MANAGER_CORRECT_ROLL") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL) != NULL);
#endif
}

static void mavlink_test_storm32_gimbal_manager_profile(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_storm32_gimbal_manager_profile_t packet_in = {
        5,72,139,206,{ 17, 18, 19, 20, 21, 22, 23, 24 },41,108,{ 175, 176, 177, 178, 179, 180, 181, 182 }
    };
    mavlink_storm32_gimbal_manager_profile_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.gimbal_id = packet_in.gimbal_id;
        packet1.profile = packet_in.profile;
        packet1.profile_flags = packet_in.profile_flags;
        packet1.rc_timeout = packet_in.rc_timeout;
        
        mav_array_memcpy(packet1.priorities, packet_in.priorities, sizeof(uint8_t)*8);
        mav_array_memcpy(packet1.timeouts, packet_in.timeouts, sizeof(uint8_t)*8);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storm32_gimbal_manager_profile_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_storm32_gimbal_manager_profile_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storm32_gimbal_manager_profile_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.gimbal_id , packet1.profile , packet1.priorities , packet1.profile_flags , packet1.rc_timeout , packet1.timeouts );
    mavlink_msg_storm32_gimbal_manager_profile_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storm32_gimbal_manager_profile_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.gimbal_id , packet1.profile , packet1.priorities , packet1.profile_flags , packet1.rc_timeout , packet1.timeouts );
    mavlink_msg_storm32_gimbal_manager_profile_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_storm32_gimbal_manager_profile_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_storm32_gimbal_manager_profile_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.gimbal_id , packet1.profile , packet1.priorities , packet1.profile_flags , packet1.rc_timeout , packet1.timeouts );
    mavlink_msg_storm32_gimbal_manager_profile_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("STORM32_GIMBAL_MANAGER_PROFILE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE) != NULL);
#endif
}

static void mavlink_test_qshot_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_QSHOT_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_qshot_status_t packet_in = {
        17235,17339
    };
    mavlink_qshot_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.mode = packet_in.mode;
        packet1.shot_state = packet_in.shot_state;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_QSHOT_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_QSHOT_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_qshot_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_qshot_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_qshot_status_pack(system_id, component_id, &msg , packet1.mode , packet1.shot_state );
    mavlink_msg_qshot_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_qshot_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.mode , packet1.shot_state );
    mavlink_msg_qshot_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_qshot_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_qshot_status_send(MAVLINK_COMM_1 , packet1.mode , packet1.shot_state );
    mavlink_msg_qshot_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("QSHOT_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_QSHOT_STATUS) != NULL);
#endif
}

static void mavlink_test_component_prearm_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_component_prearm_status_t packet_in = {
        963497464,963497672,29,96
    };
    mavlink_component_prearm_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.enabled_flags = packet_in.enabled_flags;
        packet1.fail_flags = packet_in.fail_flags;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_component_prearm_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_component_prearm_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_component_prearm_status_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.enabled_flags , packet1.fail_flags );
    mavlink_msg_component_prearm_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_component_prearm_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.enabled_flags , packet1.fail_flags );
    mavlink_msg_component_prearm_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_component_prearm_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_component_prearm_status_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.enabled_flags , packet1.fail_flags );
    mavlink_msg_component_prearm_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("COMPONENT_PREARM_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS) != NULL);
#endif
}

static void mavlink_test_storm32(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_storm32_gimbal_device_status(system_id, component_id, last_msg);
    mavlink_test_storm32_gimbal_device_control(system_id, component_id, last_msg);
    mavlink_test_storm32_gimbal_manager_information(system_id, component_id, last_msg);
    mavlink_test_storm32_gimbal_manager_status(system_id, component_id, last_msg);
    mavlink_test_storm32_gimbal_manager_control(system_id, component_id, last_msg);
    mavlink_test_storm32_gimbal_manager_control_pitchyaw(system_id, component_id, last_msg);
    mavlink_test_storm32_gimbal_manager_correct_roll(system_id, component_id, last_msg);
    mavlink_test_storm32_gimbal_manager_profile(system_id, component_id, last_msg);
    mavlink_test_qshot_status(system_id, component_id, last_msg);
    mavlink_test_component_prearm_status(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // STORM32_TESTSUITE_H
