/** @file
 *	@brief MAVLink comm protocol testsuite generated from sensesoar.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef SENSESOAR_TESTSUITE_H
#define SENSESOAR_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_sensesoar(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_common(system_id, component_id, last_msg);
	mavlink_test_sensesoar(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_obs_position(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_obs_position_t packet_in = {
		963497464,
	963497672,
	963497880,
	};
	mavlink_obs_position_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.lon = packet_in.lon;
        	packet1.lat = packet_in.lat;
        	packet1.alt = packet_in.alt;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obs_position_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_obs_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obs_position_pack(system_id, component_id, &msg , packet1.lon , packet1.lat , packet1.alt );
	mavlink_msg_obs_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obs_position_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.lon , packet1.lat , packet1.alt );
	mavlink_msg_obs_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_obs_position_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obs_position_send(MAVLINK_COMM_1 , packet1.lon , packet1.lat , packet1.alt );
	mavlink_msg_obs_position_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_obs_velocity(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_obs_velocity_t packet_in = {
		{ 17.0, 18.0, 19.0 },
	};
	mavlink_obs_velocity_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        	mav_array_memcpy(packet1.vel, packet_in.vel, sizeof(float)*3);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obs_velocity_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_obs_velocity_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obs_velocity_pack(system_id, component_id, &msg , packet1.vel );
	mavlink_msg_obs_velocity_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obs_velocity_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.vel );
	mavlink_msg_obs_velocity_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_obs_velocity_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obs_velocity_send(MAVLINK_COMM_1 , packet1.vel );
	mavlink_msg_obs_velocity_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_obs_attitude(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_obs_attitude_t packet_in = {
		{ 123.0, 124.0, 125.0, 126.0 },
	};
	mavlink_obs_attitude_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        	mav_array_memcpy(packet1.quat, packet_in.quat, sizeof(double)*4);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obs_attitude_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_obs_attitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obs_attitude_pack(system_id, component_id, &msg , packet1.quat );
	mavlink_msg_obs_attitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obs_attitude_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.quat );
	mavlink_msg_obs_attitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_obs_attitude_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obs_attitude_send(MAVLINK_COMM_1 , packet1.quat );
	mavlink_msg_obs_attitude_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_obs_wind(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_obs_wind_t packet_in = {
		{ 17.0, 18.0, 19.0 },
	};
	mavlink_obs_wind_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        	mav_array_memcpy(packet1.wind, packet_in.wind, sizeof(float)*3);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obs_wind_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_obs_wind_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obs_wind_pack(system_id, component_id, &msg , packet1.wind );
	mavlink_msg_obs_wind_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obs_wind_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.wind );
	mavlink_msg_obs_wind_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_obs_wind_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obs_wind_send(MAVLINK_COMM_1 , packet1.wind );
	mavlink_msg_obs_wind_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_obs_air_velocity(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_obs_air_velocity_t packet_in = {
		17.0,
	45.0,
	73.0,
	};
	mavlink_obs_air_velocity_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.magnitude = packet_in.magnitude;
        	packet1.aoa = packet_in.aoa;
        	packet1.slip = packet_in.slip;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obs_air_velocity_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_obs_air_velocity_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obs_air_velocity_pack(system_id, component_id, &msg , packet1.magnitude , packet1.aoa , packet1.slip );
	mavlink_msg_obs_air_velocity_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obs_air_velocity_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.magnitude , packet1.aoa , packet1.slip );
	mavlink_msg_obs_air_velocity_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_obs_air_velocity_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obs_air_velocity_send(MAVLINK_COMM_1 , packet1.magnitude , packet1.aoa , packet1.slip );
	mavlink_msg_obs_air_velocity_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_obs_bias(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_obs_bias_t packet_in = {
		{ 17.0, 18.0, 19.0 },
	{ 101.0, 102.0, 103.0 },
	};
	mavlink_obs_bias_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        	mav_array_memcpy(packet1.accBias, packet_in.accBias, sizeof(float)*3);
        	mav_array_memcpy(packet1.gyroBias, packet_in.gyroBias, sizeof(float)*3);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obs_bias_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_obs_bias_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obs_bias_pack(system_id, component_id, &msg , packet1.accBias , packet1.gyroBias );
	mavlink_msg_obs_bias_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obs_bias_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.accBias , packet1.gyroBias );
	mavlink_msg_obs_bias_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_obs_bias_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obs_bias_send(MAVLINK_COMM_1 , packet1.accBias , packet1.gyroBias );
	mavlink_msg_obs_bias_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_obs_qff(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_obs_qff_t packet_in = {
		17.0,
	};
	mavlink_obs_qff_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.qff = packet_in.qff;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obs_qff_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_obs_qff_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obs_qff_pack(system_id, component_id, &msg , packet1.qff );
	mavlink_msg_obs_qff_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obs_qff_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.qff );
	mavlink_msg_obs_qff_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_obs_qff_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obs_qff_send(MAVLINK_COMM_1 , packet1.qff );
	mavlink_msg_obs_qff_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_obs_air_temp(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_obs_air_temp_t packet_in = {
		17.0,
	};
	mavlink_obs_air_temp_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.airT = packet_in.airT;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obs_air_temp_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_obs_air_temp_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obs_air_temp_pack(system_id, component_id, &msg , packet1.airT );
	mavlink_msg_obs_air_temp_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obs_air_temp_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.airT );
	mavlink_msg_obs_air_temp_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_obs_air_temp_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obs_air_temp_send(MAVLINK_COMM_1 , packet1.airT );
	mavlink_msg_obs_air_temp_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_filt_rot_vel(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_filt_rot_vel_t packet_in = {
		{ 17.0, 18.0, 19.0 },
	};
	mavlink_filt_rot_vel_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        	mav_array_memcpy(packet1.rotVel, packet_in.rotVel, sizeof(float)*3);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_filt_rot_vel_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_filt_rot_vel_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_filt_rot_vel_pack(system_id, component_id, &msg , packet1.rotVel );
	mavlink_msg_filt_rot_vel_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_filt_rot_vel_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.rotVel );
	mavlink_msg_filt_rot_vel_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_filt_rot_vel_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_filt_rot_vel_send(MAVLINK_COMM_1 , packet1.rotVel );
	mavlink_msg_filt_rot_vel_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_llc_out(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_llc_out_t packet_in = {
		{ 17235, 17236, 17237, 17238 },
	{ 17651, 17652 },
	};
	mavlink_llc_out_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        	mav_array_memcpy(packet1.servoOut, packet_in.servoOut, sizeof(int16_t)*4);
        	mav_array_memcpy(packet1.MotorOut, packet_in.MotorOut, sizeof(int16_t)*2);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_llc_out_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_llc_out_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_llc_out_pack(system_id, component_id, &msg , packet1.servoOut , packet1.MotorOut );
	mavlink_msg_llc_out_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_llc_out_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.servoOut , packet1.MotorOut );
	mavlink_msg_llc_out_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_llc_out_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_llc_out_send(MAVLINK_COMM_1 , packet1.servoOut , packet1.MotorOut );
	mavlink_msg_llc_out_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_pm_elec(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_pm_elec_t packet_in = {
		17.0,
	45.0,
	{ 73.0, 74.0, 75.0 },
	};
	mavlink_pm_elec_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.PwCons = packet_in.PwCons;
        	packet1.BatStat = packet_in.BatStat;
        
        	mav_array_memcpy(packet1.PwGen, packet_in.PwGen, sizeof(float)*3);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pm_elec_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_pm_elec_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pm_elec_pack(system_id, component_id, &msg , packet1.PwCons , packet1.BatStat , packet1.PwGen );
	mavlink_msg_pm_elec_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pm_elec_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.PwCons , packet1.BatStat , packet1.PwGen );
	mavlink_msg_pm_elec_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_pm_elec_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pm_elec_send(MAVLINK_COMM_1 , packet1.PwCons , packet1.BatStat , packet1.PwGen );
	mavlink_msg_pm_elec_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_sys_stat(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_sys_stat_t packet_in = {
		5,
	72,
	139,
	206,
	};
	mavlink_sys_stat_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.gps = packet_in.gps;
        	packet1.act = packet_in.act;
        	packet1.mod = packet_in.mod;
        	packet1.commRssi = packet_in.commRssi;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sys_stat_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_sys_stat_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sys_stat_pack(system_id, component_id, &msg , packet1.gps , packet1.act , packet1.mod , packet1.commRssi );
	mavlink_msg_sys_stat_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sys_stat_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.gps , packet1.act , packet1.mod , packet1.commRssi );
	mavlink_msg_sys_stat_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_sys_stat_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sys_stat_send(MAVLINK_COMM_1 , packet1.gps , packet1.act , packet1.mod , packet1.commRssi );
	mavlink_msg_sys_stat_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_cmd_airspeed_chng(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_cmd_airspeed_chng_t packet_in = {
		17.0,
	17,
	};
	mavlink_cmd_airspeed_chng_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.spCmd = packet_in.spCmd;
        	packet1.target = packet_in.target;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_cmd_airspeed_chng_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_cmd_airspeed_chng_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_cmd_airspeed_chng_pack(system_id, component_id, &msg , packet1.target , packet1.spCmd );
	mavlink_msg_cmd_airspeed_chng_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_cmd_airspeed_chng_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target , packet1.spCmd );
	mavlink_msg_cmd_airspeed_chng_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_cmd_airspeed_chng_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_cmd_airspeed_chng_send(MAVLINK_COMM_1 , packet1.target , packet1.spCmd );
	mavlink_msg_cmd_airspeed_chng_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_cmd_airspeed_ack(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_cmd_airspeed_ack_t packet_in = {
		17.0,
	17,
	};
	mavlink_cmd_airspeed_ack_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.spCmd = packet_in.spCmd;
        	packet1.ack = packet_in.ack;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_cmd_airspeed_ack_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_cmd_airspeed_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_cmd_airspeed_ack_pack(system_id, component_id, &msg , packet1.spCmd , packet1.ack );
	mavlink_msg_cmd_airspeed_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_cmd_airspeed_ack_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.spCmd , packet1.ack );
	mavlink_msg_cmd_airspeed_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_cmd_airspeed_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_cmd_airspeed_ack_send(MAVLINK_COMM_1 , packet1.spCmd , packet1.ack );
	mavlink_msg_cmd_airspeed_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_sensesoar(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_obs_position(system_id, component_id, last_msg);
	mavlink_test_obs_velocity(system_id, component_id, last_msg);
	mavlink_test_obs_attitude(system_id, component_id, last_msg);
	mavlink_test_obs_wind(system_id, component_id, last_msg);
	mavlink_test_obs_air_velocity(system_id, component_id, last_msg);
	mavlink_test_obs_bias(system_id, component_id, last_msg);
	mavlink_test_obs_qff(system_id, component_id, last_msg);
	mavlink_test_obs_air_temp(system_id, component_id, last_msg);
	mavlink_test_filt_rot_vel(system_id, component_id, last_msg);
	mavlink_test_llc_out(system_id, component_id, last_msg);
	mavlink_test_pm_elec(system_id, component_id, last_msg);
	mavlink_test_sys_stat(system_id, component_id, last_msg);
	mavlink_test_cmd_airspeed_chng(system_id, component_id, last_msg);
	mavlink_test_cmd_airspeed_ack(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // SENSESOAR_TESTSUITE_H
