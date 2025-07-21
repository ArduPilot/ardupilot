/** @file
 *    @brief MAVLink comm protocol testsuite generated from cubepilot.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef CUBEPILOT_TESTSUITE_H
#define CUBEPILOT_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_cubepilot(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_cubepilot(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_cubepilot_raw_rc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CUBEPILOT_RAW_RC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_cubepilot_raw_rc_t packet_in = {
        { 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36 }
    };
    mavlink_cubepilot_raw_rc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        mav_array_memcpy(packet1.rc_raw, packet_in.rc_raw, sizeof(uint8_t)*32);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_cubepilot_raw_rc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_cubepilot_raw_rc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_cubepilot_raw_rc_pack(system_id, component_id, &msg , packet1.rc_raw );
    mavlink_msg_cubepilot_raw_rc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_cubepilot_raw_rc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.rc_raw );
    mavlink_msg_cubepilot_raw_rc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_cubepilot_raw_rc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_cubepilot_raw_rc_send(MAVLINK_COMM_1 , packet1.rc_raw );
    mavlink_msg_cubepilot_raw_rc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CUBEPILOT_RAW_RC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CUBEPILOT_RAW_RC) != NULL);
#endif
}

static void mavlink_test_herelink_video_stream_information(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_herelink_video_stream_information_t packet_in = {
        17.0,963497672,17651,17755,17859,175,242,"QRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJK"
    };
    mavlink_herelink_video_stream_information_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.framerate = packet_in.framerate;
        packet1.bitrate = packet_in.bitrate;
        packet1.resolution_h = packet_in.resolution_h;
        packet1.resolution_v = packet_in.resolution_v;
        packet1.rotation = packet_in.rotation;
        packet1.camera_id = packet_in.camera_id;
        packet1.status = packet_in.status;
        
        mav_array_memcpy(packet1.uri, packet_in.uri, sizeof(char)*230);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_herelink_video_stream_information_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_herelink_video_stream_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_herelink_video_stream_information_pack(system_id, component_id, &msg , packet1.camera_id , packet1.status , packet1.framerate , packet1.resolution_h , packet1.resolution_v , packet1.bitrate , packet1.rotation , packet1.uri );
    mavlink_msg_herelink_video_stream_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_herelink_video_stream_information_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.camera_id , packet1.status , packet1.framerate , packet1.resolution_h , packet1.resolution_v , packet1.bitrate , packet1.rotation , packet1.uri );
    mavlink_msg_herelink_video_stream_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_herelink_video_stream_information_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_herelink_video_stream_information_send(MAVLINK_COMM_1 , packet1.camera_id , packet1.status , packet1.framerate , packet1.resolution_h , packet1.resolution_v , packet1.bitrate , packet1.rotation , packet1.uri );
    mavlink_msg_herelink_video_stream_information_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("HERELINK_VIDEO_STREAM_INFORMATION") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION) != NULL);
#endif
}

static void mavlink_test_herelink_telem(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_HERELINK_TELEM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_herelink_telem_t packet_in = {
        963497464,963497672,963497880,17859,17963,18067,187
    };
    mavlink_herelink_telem_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.rf_freq = packet_in.rf_freq;
        packet1.link_bw = packet_in.link_bw;
        packet1.link_rate = packet_in.link_rate;
        packet1.snr = packet_in.snr;
        packet1.cpu_temp = packet_in.cpu_temp;
        packet1.board_temp = packet_in.board_temp;
        packet1.rssi = packet_in.rssi;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_HERELINK_TELEM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_HERELINK_TELEM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_herelink_telem_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_herelink_telem_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_herelink_telem_pack(system_id, component_id, &msg , packet1.rssi , packet1.snr , packet1.rf_freq , packet1.link_bw , packet1.link_rate , packet1.cpu_temp , packet1.board_temp );
    mavlink_msg_herelink_telem_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_herelink_telem_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.rssi , packet1.snr , packet1.rf_freq , packet1.link_bw , packet1.link_rate , packet1.cpu_temp , packet1.board_temp );
    mavlink_msg_herelink_telem_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_herelink_telem_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_herelink_telem_send(MAVLINK_COMM_1 , packet1.rssi , packet1.snr , packet1.rf_freq , packet1.link_bw , packet1.link_rate , packet1.cpu_temp , packet1.board_temp );
    mavlink_msg_herelink_telem_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("HERELINK_TELEM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_HERELINK_TELEM) != NULL);
#endif
}

static void mavlink_test_cubepilot_firmware_update_start(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_cubepilot_firmware_update_start_t packet_in = {
        963497464,963497672,29,96
    };
    mavlink_cubepilot_firmware_update_start_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.size = packet_in.size;
        packet1.crc = packet_in.crc;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_cubepilot_firmware_update_start_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_cubepilot_firmware_update_start_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_cubepilot_firmware_update_start_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.size , packet1.crc );
    mavlink_msg_cubepilot_firmware_update_start_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_cubepilot_firmware_update_start_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.size , packet1.crc );
    mavlink_msg_cubepilot_firmware_update_start_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_cubepilot_firmware_update_start_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_cubepilot_firmware_update_start_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.size , packet1.crc );
    mavlink_msg_cubepilot_firmware_update_start_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CUBEPILOT_FIRMWARE_UPDATE_START") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START) != NULL);
#endif
}

static void mavlink_test_cubepilot_firmware_update_resp(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_cubepilot_firmware_update_resp_t packet_in = {
        963497464,17,84
    };
    mavlink_cubepilot_firmware_update_resp_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.offset = packet_in.offset;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_cubepilot_firmware_update_resp_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_cubepilot_firmware_update_resp_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_cubepilot_firmware_update_resp_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.offset );
    mavlink_msg_cubepilot_firmware_update_resp_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_cubepilot_firmware_update_resp_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.offset );
    mavlink_msg_cubepilot_firmware_update_resp_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_cubepilot_firmware_update_resp_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_cubepilot_firmware_update_resp_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.offset );
    mavlink_msg_cubepilot_firmware_update_resp_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CUBEPILOT_FIRMWARE_UPDATE_RESP") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP) != NULL);
#endif
}

static void mavlink_test_cubepilot(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_cubepilot_raw_rc(system_id, component_id, last_msg);
    mavlink_test_herelink_video_stream_information(system_id, component_id, last_msg);
    mavlink_test_herelink_telem(system_id, component_id, last_msg);
    mavlink_test_cubepilot_firmware_update_start(system_id, component_id, last_msg);
    mavlink_test_cubepilot_firmware_update_resp(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // CUBEPILOT_TESTSUITE_H
