/** @file
 *    @brief MAVLink comm protocol testsuite generated from ASLUAV.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef ASLUAV_TESTSUITE_H
#define ASLUAV_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_ASLUAV(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_ASLUAV(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_command_int_stamped(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_COMMAND_INT_STAMPED >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_command_int_stamped_t packet_in = {
        93372036854775807ULL,963497880,101.0,129.0,157.0,185.0,963498920,963499128,269.0,19315,3,70,137,204,15
    };
    mavlink_command_int_stamped_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.vehicle_timestamp = packet_in.vehicle_timestamp;
        packet1.utc_time = packet_in.utc_time;
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
           memset(MAVLINK_MSG_ID_COMMAND_INT_STAMPED_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_COMMAND_INT_STAMPED_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_int_stamped_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_command_int_stamped_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_int_stamped_pack(system_id, component_id, &msg , packet1.utc_time , packet1.vehicle_timestamp , packet1.target_system , packet1.target_component , packet1.frame , packet1.command , packet1.current , packet1.autocontinue , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.x , packet1.y , packet1.z );
    mavlink_msg_command_int_stamped_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_int_stamped_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.utc_time , packet1.vehicle_timestamp , packet1.target_system , packet1.target_component , packet1.frame , packet1.command , packet1.current , packet1.autocontinue , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.x , packet1.y , packet1.z );
    mavlink_msg_command_int_stamped_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_command_int_stamped_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_int_stamped_send(MAVLINK_COMM_1 , packet1.utc_time , packet1.vehicle_timestamp , packet1.target_system , packet1.target_component , packet1.frame , packet1.command , packet1.current , packet1.autocontinue , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.x , packet1.y , packet1.z );
    mavlink_msg_command_int_stamped_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("COMMAND_INT_STAMPED") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_COMMAND_INT_STAMPED) != NULL);
#endif
}

static void mavlink_test_command_long_stamped(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_COMMAND_LONG_STAMPED >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_command_long_stamped_t packet_in = {
        93372036854775807ULL,963497880,101.0,129.0,157.0,185.0,213.0,241.0,269.0,19315,3,70,137
    };
    mavlink_command_long_stamped_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.vehicle_timestamp = packet_in.vehicle_timestamp;
        packet1.utc_time = packet_in.utc_time;
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
           memset(MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_long_stamped_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_command_long_stamped_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_long_stamped_pack(system_id, component_id, &msg , packet1.utc_time , packet1.vehicle_timestamp , packet1.target_system , packet1.target_component , packet1.command , packet1.confirmation , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.param5 , packet1.param6 , packet1.param7 );
    mavlink_msg_command_long_stamped_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_long_stamped_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.utc_time , packet1.vehicle_timestamp , packet1.target_system , packet1.target_component , packet1.command , packet1.confirmation , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.param5 , packet1.param6 , packet1.param7 );
    mavlink_msg_command_long_stamped_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_command_long_stamped_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_long_stamped_send(MAVLINK_COMM_1 , packet1.utc_time , packet1.vehicle_timestamp , packet1.target_system , packet1.target_component , packet1.command , packet1.confirmation , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.param5 , packet1.param6 , packet1.param7 );
    mavlink_msg_command_long_stamped_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("COMMAND_LONG_STAMPED") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_COMMAND_LONG_STAMPED) != NULL);
#endif
}

static void mavlink_test_sens_power(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SENS_POWER >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_sens_power_t packet_in = {
        17.0,45.0,73.0,101.0
    };
    mavlink_sens_power_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.adc121_vspb_volt = packet_in.adc121_vspb_volt;
        packet1.adc121_cspb_amp = packet_in.adc121_cspb_amp;
        packet1.adc121_cs1_amp = packet_in.adc121_cs1_amp;
        packet1.adc121_cs2_amp = packet_in.adc121_cs2_amp;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SENS_POWER_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SENS_POWER_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sens_power_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_sens_power_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sens_power_pack(system_id, component_id, &msg , packet1.adc121_vspb_volt , packet1.adc121_cspb_amp , packet1.adc121_cs1_amp , packet1.adc121_cs2_amp );
    mavlink_msg_sens_power_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sens_power_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.adc121_vspb_volt , packet1.adc121_cspb_amp , packet1.adc121_cs1_amp , packet1.adc121_cs2_amp );
    mavlink_msg_sens_power_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_sens_power_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sens_power_send(MAVLINK_COMM_1 , packet1.adc121_vspb_volt , packet1.adc121_cspb_amp , packet1.adc121_cs1_amp , packet1.adc121_cs2_amp );
    mavlink_msg_sens_power_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SENS_POWER") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SENS_POWER) != NULL);
#endif
}

static void mavlink_test_sens_mppt(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SENS_MPPT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_sens_mppt_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,18899,19003,19107,247,58,125
    };
    mavlink_sens_mppt_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.mppt_timestamp = packet_in.mppt_timestamp;
        packet1.mppt1_volt = packet_in.mppt1_volt;
        packet1.mppt1_amp = packet_in.mppt1_amp;
        packet1.mppt2_volt = packet_in.mppt2_volt;
        packet1.mppt2_amp = packet_in.mppt2_amp;
        packet1.mppt3_volt = packet_in.mppt3_volt;
        packet1.mppt3_amp = packet_in.mppt3_amp;
        packet1.mppt1_pwm = packet_in.mppt1_pwm;
        packet1.mppt2_pwm = packet_in.mppt2_pwm;
        packet1.mppt3_pwm = packet_in.mppt3_pwm;
        packet1.mppt1_status = packet_in.mppt1_status;
        packet1.mppt2_status = packet_in.mppt2_status;
        packet1.mppt3_status = packet_in.mppt3_status;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SENS_MPPT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SENS_MPPT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sens_mppt_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_sens_mppt_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sens_mppt_pack(system_id, component_id, &msg , packet1.mppt_timestamp , packet1.mppt1_volt , packet1.mppt1_amp , packet1.mppt1_pwm , packet1.mppt1_status , packet1.mppt2_volt , packet1.mppt2_amp , packet1.mppt2_pwm , packet1.mppt2_status , packet1.mppt3_volt , packet1.mppt3_amp , packet1.mppt3_pwm , packet1.mppt3_status );
    mavlink_msg_sens_mppt_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sens_mppt_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.mppt_timestamp , packet1.mppt1_volt , packet1.mppt1_amp , packet1.mppt1_pwm , packet1.mppt1_status , packet1.mppt2_volt , packet1.mppt2_amp , packet1.mppt2_pwm , packet1.mppt2_status , packet1.mppt3_volt , packet1.mppt3_amp , packet1.mppt3_pwm , packet1.mppt3_status );
    mavlink_msg_sens_mppt_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_sens_mppt_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sens_mppt_send(MAVLINK_COMM_1 , packet1.mppt_timestamp , packet1.mppt1_volt , packet1.mppt1_amp , packet1.mppt1_pwm , packet1.mppt1_status , packet1.mppt2_volt , packet1.mppt2_amp , packet1.mppt2_pwm , packet1.mppt2_status , packet1.mppt3_volt , packet1.mppt3_amp , packet1.mppt3_pwm , packet1.mppt3_status );
    mavlink_msg_sens_mppt_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SENS_MPPT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SENS_MPPT) != NULL);
#endif
}

static void mavlink_test_aslctrl_data(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ASLCTRL_DATA >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_aslctrl_data_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0,409.0,437.0,465.0,493.0,521.0,549.0,577.0,605.0,633.0,661.0,37,104
    };
    mavlink_aslctrl_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.h = packet_in.h;
        packet1.hRef = packet_in.hRef;
        packet1.hRef_t = packet_in.hRef_t;
        packet1.PitchAngle = packet_in.PitchAngle;
        packet1.PitchAngleRef = packet_in.PitchAngleRef;
        packet1.q = packet_in.q;
        packet1.qRef = packet_in.qRef;
        packet1.uElev = packet_in.uElev;
        packet1.uThrot = packet_in.uThrot;
        packet1.uThrot2 = packet_in.uThrot2;
        packet1.nZ = packet_in.nZ;
        packet1.AirspeedRef = packet_in.AirspeedRef;
        packet1.YawAngle = packet_in.YawAngle;
        packet1.YawAngleRef = packet_in.YawAngleRef;
        packet1.RollAngle = packet_in.RollAngle;
        packet1.RollAngleRef = packet_in.RollAngleRef;
        packet1.p = packet_in.p;
        packet1.pRef = packet_in.pRef;
        packet1.r = packet_in.r;
        packet1.rRef = packet_in.rRef;
        packet1.uAil = packet_in.uAil;
        packet1.uRud = packet_in.uRud;
        packet1.aslctrl_mode = packet_in.aslctrl_mode;
        packet1.SpoilersEngaged = packet_in.SpoilersEngaged;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ASLCTRL_DATA_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ASLCTRL_DATA_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_aslctrl_data_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_aslctrl_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_aslctrl_data_pack(system_id, component_id, &msg , packet1.timestamp , packet1.aslctrl_mode , packet1.h , packet1.hRef , packet1.hRef_t , packet1.PitchAngle , packet1.PitchAngleRef , packet1.q , packet1.qRef , packet1.uElev , packet1.uThrot , packet1.uThrot2 , packet1.nZ , packet1.AirspeedRef , packet1.SpoilersEngaged , packet1.YawAngle , packet1.YawAngleRef , packet1.RollAngle , packet1.RollAngleRef , packet1.p , packet1.pRef , packet1.r , packet1.rRef , packet1.uAil , packet1.uRud );
    mavlink_msg_aslctrl_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_aslctrl_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.aslctrl_mode , packet1.h , packet1.hRef , packet1.hRef_t , packet1.PitchAngle , packet1.PitchAngleRef , packet1.q , packet1.qRef , packet1.uElev , packet1.uThrot , packet1.uThrot2 , packet1.nZ , packet1.AirspeedRef , packet1.SpoilersEngaged , packet1.YawAngle , packet1.YawAngleRef , packet1.RollAngle , packet1.RollAngleRef , packet1.p , packet1.pRef , packet1.r , packet1.rRef , packet1.uAil , packet1.uRud );
    mavlink_msg_aslctrl_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_aslctrl_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_aslctrl_data_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.aslctrl_mode , packet1.h , packet1.hRef , packet1.hRef_t , packet1.PitchAngle , packet1.PitchAngleRef , packet1.q , packet1.qRef , packet1.uElev , packet1.uThrot , packet1.uThrot2 , packet1.nZ , packet1.AirspeedRef , packet1.SpoilersEngaged , packet1.YawAngle , packet1.YawAngleRef , packet1.RollAngle , packet1.RollAngleRef , packet1.p , packet1.pRef , packet1.r , packet1.rRef , packet1.uAil , packet1.uRud );
    mavlink_msg_aslctrl_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ASLCTRL_DATA") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ASLCTRL_DATA) != NULL);
#endif
}

static void mavlink_test_aslctrl_debug(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ASLCTRL_DEBUG >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_aslctrl_debug_t packet_in = {
        963497464,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,113,180
    };
    mavlink_aslctrl_debug_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.i32_1 = packet_in.i32_1;
        packet1.f_1 = packet_in.f_1;
        packet1.f_2 = packet_in.f_2;
        packet1.f_3 = packet_in.f_3;
        packet1.f_4 = packet_in.f_4;
        packet1.f_5 = packet_in.f_5;
        packet1.f_6 = packet_in.f_6;
        packet1.f_7 = packet_in.f_7;
        packet1.f_8 = packet_in.f_8;
        packet1.i8_1 = packet_in.i8_1;
        packet1.i8_2 = packet_in.i8_2;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ASLCTRL_DEBUG_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ASLCTRL_DEBUG_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_aslctrl_debug_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_aslctrl_debug_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_aslctrl_debug_pack(system_id, component_id, &msg , packet1.i32_1 , packet1.i8_1 , packet1.i8_2 , packet1.f_1 , packet1.f_2 , packet1.f_3 , packet1.f_4 , packet1.f_5 , packet1.f_6 , packet1.f_7 , packet1.f_8 );
    mavlink_msg_aslctrl_debug_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_aslctrl_debug_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.i32_1 , packet1.i8_1 , packet1.i8_2 , packet1.f_1 , packet1.f_2 , packet1.f_3 , packet1.f_4 , packet1.f_5 , packet1.f_6 , packet1.f_7 , packet1.f_8 );
    mavlink_msg_aslctrl_debug_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_aslctrl_debug_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_aslctrl_debug_send(MAVLINK_COMM_1 , packet1.i32_1 , packet1.i8_1 , packet1.i8_2 , packet1.f_1 , packet1.f_2 , packet1.f_3 , packet1.f_4 , packet1.f_5 , packet1.f_6 , packet1.f_7 , packet1.f_8 );
    mavlink_msg_aslctrl_debug_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ASLCTRL_DEBUG") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ASLCTRL_DEBUG) != NULL);
#endif
}

static void mavlink_test_asluav_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ASLUAV_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_asluav_status_t packet_in = {
        17.0,17,84,{ 151, 152, 153, 154, 155, 156, 157, 158 }
    };
    mavlink_asluav_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.Motor_rpm = packet_in.Motor_rpm;
        packet1.LED_status = packet_in.LED_status;
        packet1.SATCOM_status = packet_in.SATCOM_status;
        
        mav_array_memcpy(packet1.Servo_status, packet_in.Servo_status, sizeof(uint8_t)*8);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ASLUAV_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ASLUAV_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_asluav_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_asluav_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_asluav_status_pack(system_id, component_id, &msg , packet1.LED_status , packet1.SATCOM_status , packet1.Servo_status , packet1.Motor_rpm );
    mavlink_msg_asluav_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_asluav_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.LED_status , packet1.SATCOM_status , packet1.Servo_status , packet1.Motor_rpm );
    mavlink_msg_asluav_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_asluav_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_asluav_status_send(MAVLINK_COMM_1 , packet1.LED_status , packet1.SATCOM_status , packet1.Servo_status , packet1.Motor_rpm );
    mavlink_msg_asluav_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ASLUAV_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ASLUAV_STATUS) != NULL);
#endif
}

static void mavlink_test_ekf_ext(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_EKF_EXT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_ekf_ext_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0
    };
    mavlink_ekf_ext_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.Windspeed = packet_in.Windspeed;
        packet1.WindDir = packet_in.WindDir;
        packet1.WindZ = packet_in.WindZ;
        packet1.Airspeed = packet_in.Airspeed;
        packet1.beta = packet_in.beta;
        packet1.alpha = packet_in.alpha;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_EKF_EXT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_EKF_EXT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ekf_ext_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_ekf_ext_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ekf_ext_pack(system_id, component_id, &msg , packet1.timestamp , packet1.Windspeed , packet1.WindDir , packet1.WindZ , packet1.Airspeed , packet1.beta , packet1.alpha );
    mavlink_msg_ekf_ext_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ekf_ext_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.Windspeed , packet1.WindDir , packet1.WindZ , packet1.Airspeed , packet1.beta , packet1.alpha );
    mavlink_msg_ekf_ext_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_ekf_ext_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ekf_ext_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.Windspeed , packet1.WindDir , packet1.WindZ , packet1.Airspeed , packet1.beta , packet1.alpha );
    mavlink_msg_ekf_ext_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("EKF_EXT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_EKF_EXT) != NULL);
#endif
}

static void mavlink_test_asl_obctrl(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ASL_OBCTRL >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_asl_obctrl_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,101
    };
    mavlink_asl_obctrl_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.uElev = packet_in.uElev;
        packet1.uThrot = packet_in.uThrot;
        packet1.uThrot2 = packet_in.uThrot2;
        packet1.uAilL = packet_in.uAilL;
        packet1.uAilR = packet_in.uAilR;
        packet1.uRud = packet_in.uRud;
        packet1.obctrl_status = packet_in.obctrl_status;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ASL_OBCTRL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ASL_OBCTRL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_asl_obctrl_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_asl_obctrl_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_asl_obctrl_pack(system_id, component_id, &msg , packet1.timestamp , packet1.uElev , packet1.uThrot , packet1.uThrot2 , packet1.uAilL , packet1.uAilR , packet1.uRud , packet1.obctrl_status );
    mavlink_msg_asl_obctrl_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_asl_obctrl_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.uElev , packet1.uThrot , packet1.uThrot2 , packet1.uAilL , packet1.uAilR , packet1.uRud , packet1.obctrl_status );
    mavlink_msg_asl_obctrl_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_asl_obctrl_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_asl_obctrl_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.uElev , packet1.uThrot , packet1.uThrot2 , packet1.uAilL , packet1.uAilR , packet1.uRud , packet1.obctrl_status );
    mavlink_msg_asl_obctrl_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ASL_OBCTRL") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ASL_OBCTRL) != NULL);
#endif
}

static void mavlink_test_sens_atmos(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SENS_ATMOS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_sens_atmos_t packet_in = {
        93372036854775807ULL,73.0,101.0
    };
    mavlink_sens_atmos_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.TempAmbient = packet_in.TempAmbient;
        packet1.Humidity = packet_in.Humidity;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SENS_ATMOS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SENS_ATMOS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sens_atmos_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_sens_atmos_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sens_atmos_pack(system_id, component_id, &msg , packet1.timestamp , packet1.TempAmbient , packet1.Humidity );
    mavlink_msg_sens_atmos_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sens_atmos_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.TempAmbient , packet1.Humidity );
    mavlink_msg_sens_atmos_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_sens_atmos_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sens_atmos_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.TempAmbient , packet1.Humidity );
    mavlink_msg_sens_atmos_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SENS_ATMOS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SENS_ATMOS) != NULL);
#endif
}

static void mavlink_test_sens_batmon(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SENS_BATMON >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_sens_batmon_t packet_in = {
        93372036854775807ULL,73.0,963498088,963498296,18275,18379,18483,18587,18691,18795,18899,19003,19107,19211,125
    };
    mavlink_sens_batmon_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.batmon_timestamp = packet_in.batmon_timestamp;
        packet1.temperature = packet_in.temperature;
        packet1.safetystatus = packet_in.safetystatus;
        packet1.operationstatus = packet_in.operationstatus;
        packet1.voltage = packet_in.voltage;
        packet1.current = packet_in.current;
        packet1.batterystatus = packet_in.batterystatus;
        packet1.serialnumber = packet_in.serialnumber;
        packet1.cellvoltage1 = packet_in.cellvoltage1;
        packet1.cellvoltage2 = packet_in.cellvoltage2;
        packet1.cellvoltage3 = packet_in.cellvoltage3;
        packet1.cellvoltage4 = packet_in.cellvoltage4;
        packet1.cellvoltage5 = packet_in.cellvoltage5;
        packet1.cellvoltage6 = packet_in.cellvoltage6;
        packet1.SoC = packet_in.SoC;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SENS_BATMON_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SENS_BATMON_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sens_batmon_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_sens_batmon_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sens_batmon_pack(system_id, component_id, &msg , packet1.batmon_timestamp , packet1.temperature , packet1.voltage , packet1.current , packet1.SoC , packet1.batterystatus , packet1.serialnumber , packet1.safetystatus , packet1.operationstatus , packet1.cellvoltage1 , packet1.cellvoltage2 , packet1.cellvoltage3 , packet1.cellvoltage4 , packet1.cellvoltage5 , packet1.cellvoltage6 );
    mavlink_msg_sens_batmon_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sens_batmon_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.batmon_timestamp , packet1.temperature , packet1.voltage , packet1.current , packet1.SoC , packet1.batterystatus , packet1.serialnumber , packet1.safetystatus , packet1.operationstatus , packet1.cellvoltage1 , packet1.cellvoltage2 , packet1.cellvoltage3 , packet1.cellvoltage4 , packet1.cellvoltage5 , packet1.cellvoltage6 );
    mavlink_msg_sens_batmon_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_sens_batmon_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sens_batmon_send(MAVLINK_COMM_1 , packet1.batmon_timestamp , packet1.temperature , packet1.voltage , packet1.current , packet1.SoC , packet1.batterystatus , packet1.serialnumber , packet1.safetystatus , packet1.operationstatus , packet1.cellvoltage1 , packet1.cellvoltage2 , packet1.cellvoltage3 , packet1.cellvoltage4 , packet1.cellvoltage5 , packet1.cellvoltage6 );
    mavlink_msg_sens_batmon_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SENS_BATMON") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SENS_BATMON) != NULL);
#endif
}

static void mavlink_test_fw_soaring_data(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_FW_SOARING_DATA >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_fw_soaring_data_t packet_in = {
        93372036854775807ULL,93372036854776311ULL,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0,409.0,437.0,465.0,493.0,521.0,549.0,577.0,605.0,633.0,661.0,689.0,49,116
    };
    mavlink_fw_soaring_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.timestampModeChanged = packet_in.timestampModeChanged;
        packet1.xW = packet_in.xW;
        packet1.xR = packet_in.xR;
        packet1.xLat = packet_in.xLat;
        packet1.xLon = packet_in.xLon;
        packet1.VarW = packet_in.VarW;
        packet1.VarR = packet_in.VarR;
        packet1.VarLat = packet_in.VarLat;
        packet1.VarLon = packet_in.VarLon;
        packet1.LoiterRadius = packet_in.LoiterRadius;
        packet1.LoiterDirection = packet_in.LoiterDirection;
        packet1.DistToSoarPoint = packet_in.DistToSoarPoint;
        packet1.vSinkExp = packet_in.vSinkExp;
        packet1.z1_LocalUpdraftSpeed = packet_in.z1_LocalUpdraftSpeed;
        packet1.z2_DeltaRoll = packet_in.z2_DeltaRoll;
        packet1.z1_exp = packet_in.z1_exp;
        packet1.z2_exp = packet_in.z2_exp;
        packet1.ThermalGSNorth = packet_in.ThermalGSNorth;
        packet1.ThermalGSEast = packet_in.ThermalGSEast;
        packet1.TSE_dot = packet_in.TSE_dot;
        packet1.DebugVar1 = packet_in.DebugVar1;
        packet1.DebugVar2 = packet_in.DebugVar2;
        packet1.ControlMode = packet_in.ControlMode;
        packet1.valid = packet_in.valid;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_FW_SOARING_DATA_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_FW_SOARING_DATA_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_fw_soaring_data_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_fw_soaring_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_fw_soaring_data_pack(system_id, component_id, &msg , packet1.timestamp , packet1.timestampModeChanged , packet1.xW , packet1.xR , packet1.xLat , packet1.xLon , packet1.VarW , packet1.VarR , packet1.VarLat , packet1.VarLon , packet1.LoiterRadius , packet1.LoiterDirection , packet1.DistToSoarPoint , packet1.vSinkExp , packet1.z1_LocalUpdraftSpeed , packet1.z2_DeltaRoll , packet1.z1_exp , packet1.z2_exp , packet1.ThermalGSNorth , packet1.ThermalGSEast , packet1.TSE_dot , packet1.DebugVar1 , packet1.DebugVar2 , packet1.ControlMode , packet1.valid );
    mavlink_msg_fw_soaring_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_fw_soaring_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.timestampModeChanged , packet1.xW , packet1.xR , packet1.xLat , packet1.xLon , packet1.VarW , packet1.VarR , packet1.VarLat , packet1.VarLon , packet1.LoiterRadius , packet1.LoiterDirection , packet1.DistToSoarPoint , packet1.vSinkExp , packet1.z1_LocalUpdraftSpeed , packet1.z2_DeltaRoll , packet1.z1_exp , packet1.z2_exp , packet1.ThermalGSNorth , packet1.ThermalGSEast , packet1.TSE_dot , packet1.DebugVar1 , packet1.DebugVar2 , packet1.ControlMode , packet1.valid );
    mavlink_msg_fw_soaring_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_fw_soaring_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_fw_soaring_data_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.timestampModeChanged , packet1.xW , packet1.xR , packet1.xLat , packet1.xLon , packet1.VarW , packet1.VarR , packet1.VarLat , packet1.VarLon , packet1.LoiterRadius , packet1.LoiterDirection , packet1.DistToSoarPoint , packet1.vSinkExp , packet1.z1_LocalUpdraftSpeed , packet1.z2_DeltaRoll , packet1.z1_exp , packet1.z2_exp , packet1.ThermalGSNorth , packet1.ThermalGSEast , packet1.TSE_dot , packet1.DebugVar1 , packet1.DebugVar2 , packet1.ControlMode , packet1.valid );
    mavlink_msg_fw_soaring_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("FW_SOARING_DATA") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_FW_SOARING_DATA) != NULL);
#endif
}

static void mavlink_test_sensorpod_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SENSORPOD_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_sensorpod_status_t packet_in = {
        93372036854775807ULL,17651,163,230,41,108,175,242
    };
    mavlink_sensorpod_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.free_space = packet_in.free_space;
        packet1.visensor_rate_1 = packet_in.visensor_rate_1;
        packet1.visensor_rate_2 = packet_in.visensor_rate_2;
        packet1.visensor_rate_3 = packet_in.visensor_rate_3;
        packet1.visensor_rate_4 = packet_in.visensor_rate_4;
        packet1.recording_nodes_count = packet_in.recording_nodes_count;
        packet1.cpu_temp = packet_in.cpu_temp;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SENSORPOD_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SENSORPOD_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensorpod_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_sensorpod_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensorpod_status_pack(system_id, component_id, &msg , packet1.timestamp , packet1.visensor_rate_1 , packet1.visensor_rate_2 , packet1.visensor_rate_3 , packet1.visensor_rate_4 , packet1.recording_nodes_count , packet1.cpu_temp , packet1.free_space );
    mavlink_msg_sensorpod_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensorpod_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.visensor_rate_1 , packet1.visensor_rate_2 , packet1.visensor_rate_3 , packet1.visensor_rate_4 , packet1.recording_nodes_count , packet1.cpu_temp , packet1.free_space );
    mavlink_msg_sensorpod_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_sensorpod_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensorpod_status_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.visensor_rate_1 , packet1.visensor_rate_2 , packet1.visensor_rate_3 , packet1.visensor_rate_4 , packet1.recording_nodes_count , packet1.cpu_temp , packet1.free_space );
    mavlink_msg_sensorpod_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SENSORPOD_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SENSORPOD_STATUS) != NULL);
#endif
}

static void mavlink_test_sens_power_board(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SENS_POWER_BOARD >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_sens_power_board_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,137,204
    };
    mavlink_sens_power_board_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.pwr_brd_system_volt = packet_in.pwr_brd_system_volt;
        packet1.pwr_brd_servo_volt = packet_in.pwr_brd_servo_volt;
        packet1.pwr_brd_digital_volt = packet_in.pwr_brd_digital_volt;
        packet1.pwr_brd_mot_l_amp = packet_in.pwr_brd_mot_l_amp;
        packet1.pwr_brd_mot_r_amp = packet_in.pwr_brd_mot_r_amp;
        packet1.pwr_brd_analog_amp = packet_in.pwr_brd_analog_amp;
        packet1.pwr_brd_digital_amp = packet_in.pwr_brd_digital_amp;
        packet1.pwr_brd_ext_amp = packet_in.pwr_brd_ext_amp;
        packet1.pwr_brd_aux_amp = packet_in.pwr_brd_aux_amp;
        packet1.pwr_brd_status = packet_in.pwr_brd_status;
        packet1.pwr_brd_led_status = packet_in.pwr_brd_led_status;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SENS_POWER_BOARD_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SENS_POWER_BOARD_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sens_power_board_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_sens_power_board_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sens_power_board_pack(system_id, component_id, &msg , packet1.timestamp , packet1.pwr_brd_status , packet1.pwr_brd_led_status , packet1.pwr_brd_system_volt , packet1.pwr_brd_servo_volt , packet1.pwr_brd_digital_volt , packet1.pwr_brd_mot_l_amp , packet1.pwr_brd_mot_r_amp , packet1.pwr_brd_analog_amp , packet1.pwr_brd_digital_amp , packet1.pwr_brd_ext_amp , packet1.pwr_brd_aux_amp );
    mavlink_msg_sens_power_board_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sens_power_board_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.pwr_brd_status , packet1.pwr_brd_led_status , packet1.pwr_brd_system_volt , packet1.pwr_brd_servo_volt , packet1.pwr_brd_digital_volt , packet1.pwr_brd_mot_l_amp , packet1.pwr_brd_mot_r_amp , packet1.pwr_brd_analog_amp , packet1.pwr_brd_digital_amp , packet1.pwr_brd_ext_amp , packet1.pwr_brd_aux_amp );
    mavlink_msg_sens_power_board_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_sens_power_board_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sens_power_board_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.pwr_brd_status , packet1.pwr_brd_led_status , packet1.pwr_brd_system_volt , packet1.pwr_brd_servo_volt , packet1.pwr_brd_digital_volt , packet1.pwr_brd_mot_l_amp , packet1.pwr_brd_mot_r_amp , packet1.pwr_brd_analog_amp , packet1.pwr_brd_digital_amp , packet1.pwr_brd_ext_amp , packet1.pwr_brd_aux_amp );
    mavlink_msg_sens_power_board_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SENS_POWER_BOARD") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SENS_POWER_BOARD) != NULL);
#endif
}

static void mavlink_test_gsm_link_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GSM_LINK_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_gsm_link_status_t packet_in = {
        93372036854775807ULL,29,96,163,230,41,108
    };
    mavlink_gsm_link_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.gsm_modem_type = packet_in.gsm_modem_type;
        packet1.gsm_link_type = packet_in.gsm_link_type;
        packet1.rssi = packet_in.rssi;
        packet1.rsrp_rscp = packet_in.rsrp_rscp;
        packet1.sinr_ecio = packet_in.sinr_ecio;
        packet1.rsrq = packet_in.rsrq;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GSM_LINK_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GSM_LINK_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gsm_link_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_gsm_link_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gsm_link_status_pack(system_id, component_id, &msg , packet1.timestamp , packet1.gsm_modem_type , packet1.gsm_link_type , packet1.rssi , packet1.rsrp_rscp , packet1.sinr_ecio , packet1.rsrq );
    mavlink_msg_gsm_link_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gsm_link_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.gsm_modem_type , packet1.gsm_link_type , packet1.rssi , packet1.rsrp_rscp , packet1.sinr_ecio , packet1.rsrq );
    mavlink_msg_gsm_link_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_gsm_link_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gsm_link_status_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.gsm_modem_type , packet1.gsm_link_type , packet1.rssi , packet1.rsrp_rscp , packet1.sinr_ecio , packet1.rsrq );
    mavlink_msg_gsm_link_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("GSM_LINK_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_GSM_LINK_STATUS) != NULL);
#endif
}

static void mavlink_test_satcom_link_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SATCOM_LINK_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_satcom_link_status_t packet_in = {
        93372036854775807ULL,93372036854776311ULL,18067,18171,65,132,199,10
    };
    mavlink_satcom_link_status_t packet1, packet2;
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
           memset(MAVLINK_MSG_ID_SATCOM_LINK_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SATCOM_LINK_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_satcom_link_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_satcom_link_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_satcom_link_status_pack(system_id, component_id, &msg , packet1.timestamp , packet1.last_heartbeat , packet1.failed_sessions , packet1.successful_sessions , packet1.signal_quality , packet1.ring_pending , packet1.tx_session_pending , packet1.rx_session_pending );
    mavlink_msg_satcom_link_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_satcom_link_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.last_heartbeat , packet1.failed_sessions , packet1.successful_sessions , packet1.signal_quality , packet1.ring_pending , packet1.tx_session_pending , packet1.rx_session_pending );
    mavlink_msg_satcom_link_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_satcom_link_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_satcom_link_status_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.last_heartbeat , packet1.failed_sessions , packet1.successful_sessions , packet1.signal_quality , packet1.ring_pending , packet1.tx_session_pending , packet1.rx_session_pending );
    mavlink_msg_satcom_link_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SATCOM_LINK_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SATCOM_LINK_STATUS) != NULL);
#endif
}

static void mavlink_test_sensor_airflow_angles(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_sensor_airflow_angles_t packet_in = {
        93372036854775807ULL,73.0,101.0,53,120
    };
    mavlink_sensor_airflow_angles_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.angleofattack = packet_in.angleofattack;
        packet1.sideslip = packet_in.sideslip;
        packet1.angleofattack_valid = packet_in.angleofattack_valid;
        packet1.sideslip_valid = packet_in.sideslip_valid;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensor_airflow_angles_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_sensor_airflow_angles_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensor_airflow_angles_pack(system_id, component_id, &msg , packet1.timestamp , packet1.angleofattack , packet1.angleofattack_valid , packet1.sideslip , packet1.sideslip_valid );
    mavlink_msg_sensor_airflow_angles_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensor_airflow_angles_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.angleofattack , packet1.angleofattack_valid , packet1.sideslip , packet1.sideslip_valid );
    mavlink_msg_sensor_airflow_angles_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_sensor_airflow_angles_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensor_airflow_angles_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.angleofattack , packet1.angleofattack_valid , packet1.sideslip , packet1.sideslip_valid );
    mavlink_msg_sensor_airflow_angles_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SENSOR_AIRFLOW_ANGLES") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES) != NULL);
#endif
}

static void mavlink_test_ASLUAV(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_command_int_stamped(system_id, component_id, last_msg);
    mavlink_test_command_long_stamped(system_id, component_id, last_msg);
    mavlink_test_sens_power(system_id, component_id, last_msg);
    mavlink_test_sens_mppt(system_id, component_id, last_msg);
    mavlink_test_aslctrl_data(system_id, component_id, last_msg);
    mavlink_test_aslctrl_debug(system_id, component_id, last_msg);
    mavlink_test_asluav_status(system_id, component_id, last_msg);
    mavlink_test_ekf_ext(system_id, component_id, last_msg);
    mavlink_test_asl_obctrl(system_id, component_id, last_msg);
    mavlink_test_sens_atmos(system_id, component_id, last_msg);
    mavlink_test_sens_batmon(system_id, component_id, last_msg);
    mavlink_test_fw_soaring_data(system_id, component_id, last_msg);
    mavlink_test_sensorpod_status(system_id, component_id, last_msg);
    mavlink_test_sens_power_board(system_id, component_id, last_msg);
    mavlink_test_gsm_link_status(system_id, component_id, last_msg);
    mavlink_test_satcom_link_status(system_id, component_id, last_msg);
    mavlink_test_sensor_airflow_angles(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // ASLUAV_TESTSUITE_H
