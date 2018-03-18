/** @file
 *    @brief MAVLink comm protocol testsuite generated from icarous.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef ICAROUS_TESTSUITE_H
#define ICAROUS_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_icarous(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_icarous(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_icarous_heartbeat(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ICAROUS_HEARTBEAT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_icarous_heartbeat_t packet_in = {
        5
    };
    mavlink_icarous_heartbeat_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.status = packet_in.status;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_icarous_heartbeat_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_icarous_heartbeat_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_icarous_heartbeat_pack(system_id, component_id, &msg , packet1.status );
    mavlink_msg_icarous_heartbeat_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_icarous_heartbeat_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.status );
    mavlink_msg_icarous_heartbeat_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_icarous_heartbeat_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_icarous_heartbeat_send(MAVLINK_COMM_1 , packet1.status );
    mavlink_msg_icarous_heartbeat_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_icarous_kinematic_bands(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_icarous_kinematic_bands_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,125,192,3,70,137,204
    };
    mavlink_icarous_kinematic_bands_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.min1 = packet_in.min1;
        packet1.max1 = packet_in.max1;
        packet1.min2 = packet_in.min2;
        packet1.max2 = packet_in.max2;
        packet1.min3 = packet_in.min3;
        packet1.max3 = packet_in.max3;
        packet1.min4 = packet_in.min4;
        packet1.max4 = packet_in.max4;
        packet1.min5 = packet_in.min5;
        packet1.max5 = packet_in.max5;
        packet1.numBands = packet_in.numBands;
        packet1.type1 = packet_in.type1;
        packet1.type2 = packet_in.type2;
        packet1.type3 = packet_in.type3;
        packet1.type4 = packet_in.type4;
        packet1.type5 = packet_in.type5;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_icarous_kinematic_bands_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_icarous_kinematic_bands_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_icarous_kinematic_bands_pack(system_id, component_id, &msg , packet1.numBands , packet1.type1 , packet1.min1 , packet1.max1 , packet1.type2 , packet1.min2 , packet1.max2 , packet1.type3 , packet1.min3 , packet1.max3 , packet1.type4 , packet1.min4 , packet1.max4 , packet1.type5 , packet1.min5 , packet1.max5 );
    mavlink_msg_icarous_kinematic_bands_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_icarous_kinematic_bands_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.numBands , packet1.type1 , packet1.min1 , packet1.max1 , packet1.type2 , packet1.min2 , packet1.max2 , packet1.type3 , packet1.min3 , packet1.max3 , packet1.type4 , packet1.min4 , packet1.max4 , packet1.type5 , packet1.min5 , packet1.max5 );
    mavlink_msg_icarous_kinematic_bands_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_icarous_kinematic_bands_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_icarous_kinematic_bands_send(MAVLINK_COMM_1 , packet1.numBands , packet1.type1 , packet1.min1 , packet1.max1 , packet1.type2 , packet1.min2 , packet1.max2 , packet1.type3 , packet1.min3 , packet1.max3 , packet1.type4 , packet1.min4 , packet1.max4 , packet1.type5 , packet1.min5 , packet1.max5 );
    mavlink_msg_icarous_kinematic_bands_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_icarous(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_icarous_heartbeat(system_id, component_id, last_msg);
    mavlink_test_icarous_kinematic_bands(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // ICAROUS_TESTSUITE_H
