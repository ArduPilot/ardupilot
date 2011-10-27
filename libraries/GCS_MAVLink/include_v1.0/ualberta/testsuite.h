/** @file
 *	@brief MAVLink comm protocol testsuite generated from ualberta.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef UALBERTA_TESTSUITE_H
#define UALBERTA_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_ualberta(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_common(system_id, component_id, last_msg);
	mavlink_test_ualberta(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_nav_filter_bias(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_nav_filter_bias_t packet_in = {
		93372036854775807ULL,
	73.0,
	101.0,
	129.0,
	157.0,
	185.0,
	213.0,
	};
	mavlink_nav_filter_bias_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.usec = packet_in.usec;
        	packet1.accel_0 = packet_in.accel_0;
        	packet1.accel_1 = packet_in.accel_1;
        	packet1.accel_2 = packet_in.accel_2;
        	packet1.gyro_0 = packet_in.gyro_0;
        	packet1.gyro_1 = packet_in.gyro_1;
        	packet1.gyro_2 = packet_in.gyro_2;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_nav_filter_bias_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_nav_filter_bias_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_nav_filter_bias_pack(system_id, component_id, &msg , packet1.usec , packet1.accel_0 , packet1.accel_1 , packet1.accel_2 , packet1.gyro_0 , packet1.gyro_1 , packet1.gyro_2 );
	mavlink_msg_nav_filter_bias_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_nav_filter_bias_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.usec , packet1.accel_0 , packet1.accel_1 , packet1.accel_2 , packet1.gyro_0 , packet1.gyro_1 , packet1.gyro_2 );
	mavlink_msg_nav_filter_bias_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_nav_filter_bias_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_nav_filter_bias_send(MAVLINK_COMM_1 , packet1.usec , packet1.accel_0 , packet1.accel_1 , packet1.accel_2 , packet1.gyro_0 , packet1.gyro_1 , packet1.gyro_2 );
	mavlink_msg_nav_filter_bias_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_radio_calibration(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_radio_calibration_t packet_in = {
		{ 17235, 17236, 17237 },
	{ 17547, 17548, 17549 },
	{ 17859, 17860, 17861 },
	{ 18171, 18172 },
	{ 18379, 18380, 18381, 18382, 18383 },
	{ 18899, 18900, 18901, 18902, 18903 },
	};
	mavlink_radio_calibration_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        	memcpy(packet1.aileron, packet_in.aileron, sizeof(uint16_t)*3);
        	memcpy(packet1.elevator, packet_in.elevator, sizeof(uint16_t)*3);
        	memcpy(packet1.rudder, packet_in.rudder, sizeof(uint16_t)*3);
        	memcpy(packet1.gyro, packet_in.gyro, sizeof(uint16_t)*2);
        	memcpy(packet1.pitch, packet_in.pitch, sizeof(uint16_t)*5);
        	memcpy(packet1.throttle, packet_in.throttle, sizeof(uint16_t)*5);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_radio_calibration_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_radio_calibration_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_radio_calibration_pack(system_id, component_id, &msg , packet1.aileron , packet1.elevator , packet1.rudder , packet1.gyro , packet1.pitch , packet1.throttle );
	mavlink_msg_radio_calibration_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_radio_calibration_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.aileron , packet1.elevator , packet1.rudder , packet1.gyro , packet1.pitch , packet1.throttle );
	mavlink_msg_radio_calibration_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_radio_calibration_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_radio_calibration_send(MAVLINK_COMM_1 , packet1.aileron , packet1.elevator , packet1.rudder , packet1.gyro , packet1.pitch , packet1.throttle );
	mavlink_msg_radio_calibration_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_ualberta_sys_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_ualberta_sys_status_t packet_in = {
		5,
	72,
	139,
	};
	mavlink_ualberta_sys_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.mode = packet_in.mode;
        	packet1.nav_mode = packet_in.nav_mode;
        	packet1.pilot = packet_in.pilot;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ualberta_sys_status_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_ualberta_sys_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ualberta_sys_status_pack(system_id, component_id, &msg , packet1.mode , packet1.nav_mode , packet1.pilot );
	mavlink_msg_ualberta_sys_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ualberta_sys_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.mode , packet1.nav_mode , packet1.pilot );
	mavlink_msg_ualberta_sys_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_ualberta_sys_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ualberta_sys_status_send(MAVLINK_COMM_1 , packet1.mode , packet1.nav_mode , packet1.pilot );
	mavlink_msg_ualberta_sys_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_ualberta(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_nav_filter_bias(system_id, component_id, last_msg);
	mavlink_test_radio_calibration(system_id, component_id, last_msg);
	mavlink_test_ualberta_sys_status(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // UALBERTA_TESTSUITE_H
