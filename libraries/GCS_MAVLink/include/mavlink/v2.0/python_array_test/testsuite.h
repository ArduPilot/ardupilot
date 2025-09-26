/** @file
 *    @brief MAVLink comm protocol testsuite generated from python_array_test.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef PYTHON_ARRAY_TEST_TESTSUITE_H
#define PYTHON_ARRAY_TEST_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_python_array_test(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_python_array_test(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_array_test_0(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ARRAY_TEST_0 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_array_test_0_t packet_in = {
        { 963497464, 963497465, 963497466, 963497467 },{ 18067, 18068, 18069, 18070 },77,{ 144, 145, 146, 147 },{ 156, 157, 158, 159 }
    };
    mavlink_array_test_0_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.v1 = packet_in.v1;
        
        mav_array_memcpy(packet1.ar_u32, packet_in.ar_u32, sizeof(uint32_t)*4);
        mav_array_memcpy(packet1.ar_u16, packet_in.ar_u16, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.ar_i8, packet_in.ar_i8, sizeof(int8_t)*4);
        mav_array_memcpy(packet1.ar_u8, packet_in.ar_u8, sizeof(uint8_t)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ARRAY_TEST_0_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ARRAY_TEST_0_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_array_test_0_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_array_test_0_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_array_test_0_pack(system_id, component_id, &msg , packet1.v1 , packet1.ar_i8 , packet1.ar_u8 , packet1.ar_u16 , packet1.ar_u32 );
    mavlink_msg_array_test_0_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_array_test_0_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.v1 , packet1.ar_i8 , packet1.ar_u8 , packet1.ar_u16 , packet1.ar_u32 );
    mavlink_msg_array_test_0_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_array_test_0_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_array_test_0_send(MAVLINK_COMM_1 , packet1.v1 , packet1.ar_i8 , packet1.ar_u8 , packet1.ar_u16 , packet1.ar_u32 );
    mavlink_msg_array_test_0_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ARRAY_TEST_0") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ARRAY_TEST_0) != NULL);
#endif
}

static void mavlink_test_array_test_1(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ARRAY_TEST_1 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_array_test_1_t packet_in = {
        { 963497464, 963497465, 963497466, 963497467 }
    };
    mavlink_array_test_1_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        mav_array_memcpy(packet1.ar_u32, packet_in.ar_u32, sizeof(uint32_t)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ARRAY_TEST_1_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ARRAY_TEST_1_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_array_test_1_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_array_test_1_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_array_test_1_pack(system_id, component_id, &msg , packet1.ar_u32 );
    mavlink_msg_array_test_1_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_array_test_1_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.ar_u32 );
    mavlink_msg_array_test_1_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_array_test_1_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_array_test_1_send(MAVLINK_COMM_1 , packet1.ar_u32 );
    mavlink_msg_array_test_1_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ARRAY_TEST_1") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ARRAY_TEST_1) != NULL);
#endif
}

static void mavlink_test_array_test_3(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ARRAY_TEST_3 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_array_test_3_t packet_in = {
        { 963497464, 963497465, 963497466, 963497467 },53
    };
    mavlink_array_test_3_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.v = packet_in.v;
        
        mav_array_memcpy(packet1.ar_u32, packet_in.ar_u32, sizeof(uint32_t)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ARRAY_TEST_3_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ARRAY_TEST_3_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_array_test_3_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_array_test_3_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_array_test_3_pack(system_id, component_id, &msg , packet1.v , packet1.ar_u32 );
    mavlink_msg_array_test_3_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_array_test_3_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.v , packet1.ar_u32 );
    mavlink_msg_array_test_3_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_array_test_3_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_array_test_3_send(MAVLINK_COMM_1 , packet1.v , packet1.ar_u32 );
    mavlink_msg_array_test_3_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ARRAY_TEST_3") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ARRAY_TEST_3) != NULL);
#endif
}

static void mavlink_test_array_test_4(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ARRAY_TEST_4 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_array_test_4_t packet_in = {
        { 963497464, 963497465, 963497466, 963497467 },53
    };
    mavlink_array_test_4_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.v = packet_in.v;
        
        mav_array_memcpy(packet1.ar_u32, packet_in.ar_u32, sizeof(uint32_t)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ARRAY_TEST_4_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ARRAY_TEST_4_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_array_test_4_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_array_test_4_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_array_test_4_pack(system_id, component_id, &msg , packet1.ar_u32 , packet1.v );
    mavlink_msg_array_test_4_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_array_test_4_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.ar_u32 , packet1.v );
    mavlink_msg_array_test_4_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_array_test_4_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_array_test_4_send(MAVLINK_COMM_1 , packet1.ar_u32 , packet1.v );
    mavlink_msg_array_test_4_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ARRAY_TEST_4") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ARRAY_TEST_4) != NULL);
#endif
}

static void mavlink_test_array_test_5(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ARRAY_TEST_5 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_array_test_5_t packet_in = {
        "ABCD","FGHI"
    };
    mavlink_array_test_5_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        mav_array_memcpy(packet1.c1, packet_in.c1, sizeof(char)*5);
        mav_array_memcpy(packet1.c2, packet_in.c2, sizeof(char)*5);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ARRAY_TEST_5_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ARRAY_TEST_5_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_array_test_5_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_array_test_5_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_array_test_5_pack(system_id, component_id, &msg , packet1.c1 , packet1.c2 );
    mavlink_msg_array_test_5_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_array_test_5_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.c1 , packet1.c2 );
    mavlink_msg_array_test_5_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_array_test_5_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_array_test_5_send(MAVLINK_COMM_1 , packet1.c1 , packet1.c2 );
    mavlink_msg_array_test_5_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ARRAY_TEST_5") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ARRAY_TEST_5) != NULL);
#endif
}

static void mavlink_test_array_test_6(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ARRAY_TEST_6 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_array_test_6_t packet_in = {
        { 123.0, 124.0 },963498296,{ 963498504, 963498505 },{ 963498920, 963498921 },{ 269.0, 270.0 },19523,{ 19627, 19628 },{ 19835, 19836 },39,{ 106, 107 },{ 240, 241 },"HIJKLMNOPQRSTUVWXYZABCDEFGHIJKL"
    };
    mavlink_array_test_6_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.v3 = packet_in.v3;
        packet1.v2 = packet_in.v2;
        packet1.v1 = packet_in.v1;
        
        mav_array_memcpy(packet1.ar_d, packet_in.ar_d, sizeof(double)*2);
        mav_array_memcpy(packet1.ar_u32, packet_in.ar_u32, sizeof(uint32_t)*2);
        mav_array_memcpy(packet1.ar_i32, packet_in.ar_i32, sizeof(int32_t)*2);
        mav_array_memcpy(packet1.ar_f, packet_in.ar_f, sizeof(float)*2);
        mav_array_memcpy(packet1.ar_u16, packet_in.ar_u16, sizeof(uint16_t)*2);
        mav_array_memcpy(packet1.ar_i16, packet_in.ar_i16, sizeof(int16_t)*2);
        mav_array_memcpy(packet1.ar_u8, packet_in.ar_u8, sizeof(uint8_t)*2);
        mav_array_memcpy(packet1.ar_i8, packet_in.ar_i8, sizeof(int8_t)*2);
        mav_array_memcpy(packet1.ar_c, packet_in.ar_c, sizeof(char)*32);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ARRAY_TEST_6_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ARRAY_TEST_6_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_array_test_6_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_array_test_6_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_array_test_6_pack(system_id, component_id, &msg , packet1.v1 , packet1.v2 , packet1.v3 , packet1.ar_u32 , packet1.ar_i32 , packet1.ar_u16 , packet1.ar_i16 , packet1.ar_u8 , packet1.ar_i8 , packet1.ar_c , packet1.ar_d , packet1.ar_f );
    mavlink_msg_array_test_6_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_array_test_6_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.v1 , packet1.v2 , packet1.v3 , packet1.ar_u32 , packet1.ar_i32 , packet1.ar_u16 , packet1.ar_i16 , packet1.ar_u8 , packet1.ar_i8 , packet1.ar_c , packet1.ar_d , packet1.ar_f );
    mavlink_msg_array_test_6_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_array_test_6_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_array_test_6_send(MAVLINK_COMM_1 , packet1.v1 , packet1.v2 , packet1.v3 , packet1.ar_u32 , packet1.ar_i32 , packet1.ar_u16 , packet1.ar_i16 , packet1.ar_u8 , packet1.ar_i8 , packet1.ar_c , packet1.ar_d , packet1.ar_f );
    mavlink_msg_array_test_6_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ARRAY_TEST_6") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ARRAY_TEST_6) != NULL);
#endif
}

static void mavlink_test_array_test_7(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ARRAY_TEST_7 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_array_test_7_t packet_in = {
        { 123.0, 124.0 },{ 129.0, 130.0 },{ 963498712, 963498713 },{ 963499128, 963499129 },{ 19315, 19316 },{ 19523, 19524 },{ 149, 150 },{ 27, 28 },"ABCDEFGHIJKLMNOPQRSTUVWXYZABCDE"
    };
    mavlink_array_test_7_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        mav_array_memcpy(packet1.ar_d, packet_in.ar_d, sizeof(double)*2);
        mav_array_memcpy(packet1.ar_f, packet_in.ar_f, sizeof(float)*2);
        mav_array_memcpy(packet1.ar_u32, packet_in.ar_u32, sizeof(uint32_t)*2);
        mav_array_memcpy(packet1.ar_i32, packet_in.ar_i32, sizeof(int32_t)*2);
        mav_array_memcpy(packet1.ar_u16, packet_in.ar_u16, sizeof(uint16_t)*2);
        mav_array_memcpy(packet1.ar_i16, packet_in.ar_i16, sizeof(int16_t)*2);
        mav_array_memcpy(packet1.ar_u8, packet_in.ar_u8, sizeof(uint8_t)*2);
        mav_array_memcpy(packet1.ar_i8, packet_in.ar_i8, sizeof(int8_t)*2);
        mav_array_memcpy(packet1.ar_c, packet_in.ar_c, sizeof(char)*32);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ARRAY_TEST_7_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ARRAY_TEST_7_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_array_test_7_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_array_test_7_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_array_test_7_pack(system_id, component_id, &msg , packet1.ar_d , packet1.ar_f , packet1.ar_u32 , packet1.ar_i32 , packet1.ar_u16 , packet1.ar_i16 , packet1.ar_u8 , packet1.ar_i8 , packet1.ar_c );
    mavlink_msg_array_test_7_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_array_test_7_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.ar_d , packet1.ar_f , packet1.ar_u32 , packet1.ar_i32 , packet1.ar_u16 , packet1.ar_i16 , packet1.ar_u8 , packet1.ar_i8 , packet1.ar_c );
    mavlink_msg_array_test_7_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_array_test_7_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_array_test_7_send(MAVLINK_COMM_1 , packet1.ar_d , packet1.ar_f , packet1.ar_u32 , packet1.ar_i32 , packet1.ar_u16 , packet1.ar_i16 , packet1.ar_u8 , packet1.ar_i8 , packet1.ar_c );
    mavlink_msg_array_test_7_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ARRAY_TEST_7") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ARRAY_TEST_7) != NULL);
#endif
}

static void mavlink_test_array_test_8(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ARRAY_TEST_8 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_array_test_8_t packet_in = {
        { 123.0, 124.0 },963498296,{ 18275, 18276 }
    };
    mavlink_array_test_8_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.v3 = packet_in.v3;
        
        mav_array_memcpy(packet1.ar_d, packet_in.ar_d, sizeof(double)*2);
        mav_array_memcpy(packet1.ar_u16, packet_in.ar_u16, sizeof(uint16_t)*2);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ARRAY_TEST_8_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ARRAY_TEST_8_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_array_test_8_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_array_test_8_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_array_test_8_pack(system_id, component_id, &msg , packet1.v3 , packet1.ar_d , packet1.ar_u16 );
    mavlink_msg_array_test_8_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_array_test_8_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.v3 , packet1.ar_d , packet1.ar_u16 );
    mavlink_msg_array_test_8_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_array_test_8_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_array_test_8_send(MAVLINK_COMM_1 , packet1.v3 , packet1.ar_d , packet1.ar_u16 );
    mavlink_msg_array_test_8_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ARRAY_TEST_8") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ARRAY_TEST_8) != NULL);
#endif
}

static void mavlink_test_python_array_test(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_array_test_0(system_id, component_id, last_msg);
    mavlink_test_array_test_1(system_id, component_id, last_msg);
    mavlink_test_array_test_3(system_id, component_id, last_msg);
    mavlink_test_array_test_4(system_id, component_id, last_msg);
    mavlink_test_array_test_5(system_id, component_id, last_msg);
    mavlink_test_array_test_6(system_id, component_id, last_msg);
    mavlink_test_array_test_7(system_id, component_id, last_msg);
    mavlink_test_array_test_8(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // PYTHON_ARRAY_TEST_TESTSUITE_H
