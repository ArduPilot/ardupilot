/** @file
 *    @brief MAVLink comm protocol testsuite generated from test.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef TEST_TESTSUITE_H
#define TEST_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_test(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_test(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_test_types(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TEST_TYPES >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_test_types_t packet_in = {
        93372036854775807ULL,93372036854776311LL,235.0,{ 93372036854777319, 93372036854777320, 93372036854777321 },{ 93372036854778831, 93372036854778832, 93372036854778833 },{ 627.0, 628.0, 629.0 },963502456,963502664,745.0,{ 963503080, 963503081, 963503082 },{ 963503704, 963503705, 963503706 },{ 941.0, 942.0, 943.0 },24723,24827,{ 24931, 24932, 24933 },{ 25243, 25244, 25245 },'E',"FGHIJKLMN",198,9,{ 76, 77, 78 },{ 21, 22, 23 }
    };
    mavlink_test_types_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.u64 = packet_in.u64;
        packet1.s64 = packet_in.s64;
        packet1.d = packet_in.d;
        packet1.u32 = packet_in.u32;
        packet1.s32 = packet_in.s32;
        packet1.f = packet_in.f;
        packet1.u16 = packet_in.u16;
        packet1.s16 = packet_in.s16;
        packet1.c = packet_in.c;
        packet1.u8 = packet_in.u8;
        packet1.s8 = packet_in.s8;
        
        mav_array_memcpy(packet1.u64_array, packet_in.u64_array, sizeof(uint64_t)*3);
        mav_array_memcpy(packet1.s64_array, packet_in.s64_array, sizeof(int64_t)*3);
        mav_array_memcpy(packet1.d_array, packet_in.d_array, sizeof(double)*3);
        mav_array_memcpy(packet1.u32_array, packet_in.u32_array, sizeof(uint32_t)*3);
        mav_array_memcpy(packet1.s32_array, packet_in.s32_array, sizeof(int32_t)*3);
        mav_array_memcpy(packet1.f_array, packet_in.f_array, sizeof(float)*3);
        mav_array_memcpy(packet1.u16_array, packet_in.u16_array, sizeof(uint16_t)*3);
        mav_array_memcpy(packet1.s16_array, packet_in.s16_array, sizeof(int16_t)*3);
        mav_array_memcpy(packet1.s, packet_in.s, sizeof(char)*10);
        mav_array_memcpy(packet1.u8_array, packet_in.u8_array, sizeof(uint8_t)*3);
        mav_array_memcpy(packet1.s8_array, packet_in.s8_array, sizeof(int8_t)*3);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TEST_TYPES_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TEST_TYPES_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_test_types_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_test_types_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_test_types_pack(system_id, component_id, &msg , packet1.c , packet1.s , packet1.u8 , packet1.u16 , packet1.u32 , packet1.u64 , packet1.s8 , packet1.s16 , packet1.s32 , packet1.s64 , packet1.f , packet1.d , packet1.u8_array , packet1.u16_array , packet1.u32_array , packet1.u64_array , packet1.s8_array , packet1.s16_array , packet1.s32_array , packet1.s64_array , packet1.f_array , packet1.d_array );
    mavlink_msg_test_types_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_test_types_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.c , packet1.s , packet1.u8 , packet1.u16 , packet1.u32 , packet1.u64 , packet1.s8 , packet1.s16 , packet1.s32 , packet1.s64 , packet1.f , packet1.d , packet1.u8_array , packet1.u16_array , packet1.u32_array , packet1.u64_array , packet1.s8_array , packet1.s16_array , packet1.s32_array , packet1.s64_array , packet1.f_array , packet1.d_array );
    mavlink_msg_test_types_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_test_types_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_test_types_send(MAVLINK_COMM_1 , packet1.c , packet1.s , packet1.u8 , packet1.u16 , packet1.u32 , packet1.u64 , packet1.s8 , packet1.s16 , packet1.s32 , packet1.s64 , packet1.f , packet1.d , packet1.u8_array , packet1.u16_array , packet1.u32_array , packet1.u64_array , packet1.s8_array , packet1.s16_array , packet1.s32_array , packet1.s64_array , packet1.f_array , packet1.d_array );
    mavlink_msg_test_types_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("TEST_TYPES") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_TEST_TYPES) != NULL);
#endif
}

static void mavlink_test_test(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_test_types(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // TEST_TESTSUITE_H
