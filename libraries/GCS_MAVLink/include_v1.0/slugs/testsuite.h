/** @file
 *	@brief MAVLink comm protocol testsuite generated from slugs.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef SLUGS_TESTSUITE_H
#define SLUGS_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_slugs(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_common(system_id, component_id, last_msg);
	mavlink_test_slugs(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_cpu_load(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_cpu_load_t packet_in = {
		17235,
	139,
	206,
	};
	mavlink_cpu_load_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.batVolt = packet_in.batVolt;
        	packet1.sensLoad = packet_in.sensLoad;
        	packet1.ctrlLoad = packet_in.ctrlLoad;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_cpu_load_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_cpu_load_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_cpu_load_pack(system_id, component_id, &msg , packet1.sensLoad , packet1.ctrlLoad , packet1.batVolt );
	mavlink_msg_cpu_load_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_cpu_load_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.sensLoad , packet1.ctrlLoad , packet1.batVolt );
	mavlink_msg_cpu_load_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_cpu_load_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_cpu_load_send(MAVLINK_COMM_1 , packet1.sensLoad , packet1.ctrlLoad , packet1.batVolt );
	mavlink_msg_cpu_load_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_air_data(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_air_data_t packet_in = {
		17.0,
	45.0,
	17651,
	};
	mavlink_air_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.dynamicPressure = packet_in.dynamicPressure;
        	packet1.staticPressure = packet_in.staticPressure;
        	packet1.temperature = packet_in.temperature;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_air_data_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_air_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_air_data_pack(system_id, component_id, &msg , packet1.dynamicPressure , packet1.staticPressure , packet1.temperature );
	mavlink_msg_air_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_air_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.dynamicPressure , packet1.staticPressure , packet1.temperature );
	mavlink_msg_air_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_air_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_air_data_send(MAVLINK_COMM_1 , packet1.dynamicPressure , packet1.staticPressure , packet1.temperature );
	mavlink_msg_air_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_sensor_bias(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_sensor_bias_t packet_in = {
		17.0,
	45.0,
	73.0,
	101.0,
	129.0,
	157.0,
	};
	mavlink_sensor_bias_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.axBias = packet_in.axBias;
        	packet1.ayBias = packet_in.ayBias;
        	packet1.azBias = packet_in.azBias;
        	packet1.gxBias = packet_in.gxBias;
        	packet1.gyBias = packet_in.gyBias;
        	packet1.gzBias = packet_in.gzBias;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sensor_bias_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_sensor_bias_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sensor_bias_pack(system_id, component_id, &msg , packet1.axBias , packet1.ayBias , packet1.azBias , packet1.gxBias , packet1.gyBias , packet1.gzBias );
	mavlink_msg_sensor_bias_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sensor_bias_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.axBias , packet1.ayBias , packet1.azBias , packet1.gxBias , packet1.gyBias , packet1.gzBias );
	mavlink_msg_sensor_bias_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_sensor_bias_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sensor_bias_send(MAVLINK_COMM_1 , packet1.axBias , packet1.ayBias , packet1.azBias , packet1.gxBias , packet1.gyBias , packet1.gzBias );
	mavlink_msg_sensor_bias_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_diagnostic(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_diagnostic_t packet_in = {
		17.0,
	45.0,
	73.0,
	17859,
	17963,
	18067,
	};
	mavlink_diagnostic_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.diagFl1 = packet_in.diagFl1;
        	packet1.diagFl2 = packet_in.diagFl2;
        	packet1.diagFl3 = packet_in.diagFl3;
        	packet1.diagSh1 = packet_in.diagSh1;
        	packet1.diagSh2 = packet_in.diagSh2;
        	packet1.diagSh3 = packet_in.diagSh3;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_diagnostic_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_diagnostic_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_diagnostic_pack(system_id, component_id, &msg , packet1.diagFl1 , packet1.diagFl2 , packet1.diagFl3 , packet1.diagSh1 , packet1.diagSh2 , packet1.diagSh3 );
	mavlink_msg_diagnostic_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_diagnostic_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.diagFl1 , packet1.diagFl2 , packet1.diagFl3 , packet1.diagSh1 , packet1.diagSh2 , packet1.diagSh3 );
	mavlink_msg_diagnostic_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_diagnostic_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_diagnostic_send(MAVLINK_COMM_1 , packet1.diagFl1 , packet1.diagFl2 , packet1.diagFl3 , packet1.diagSh1 , packet1.diagSh2 , packet1.diagSh3 );
	mavlink_msg_diagnostic_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_slugs_navigation(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_slugs_navigation_t packet_in = {
		17.0,
	45.0,
	73.0,
	101.0,
	129.0,
	157.0,
	185.0,
	89,
	156,
	};
	mavlink_slugs_navigation_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.u_m = packet_in.u_m;
        	packet1.phi_c = packet_in.phi_c;
        	packet1.theta_c = packet_in.theta_c;
        	packet1.psiDot_c = packet_in.psiDot_c;
        	packet1.ay_body = packet_in.ay_body;
        	packet1.totalDist = packet_in.totalDist;
        	packet1.dist2Go = packet_in.dist2Go;
        	packet1.fromWP = packet_in.fromWP;
        	packet1.toWP = packet_in.toWP;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_slugs_navigation_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_slugs_navigation_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_slugs_navigation_pack(system_id, component_id, &msg , packet1.u_m , packet1.phi_c , packet1.theta_c , packet1.psiDot_c , packet1.ay_body , packet1.totalDist , packet1.dist2Go , packet1.fromWP , packet1.toWP );
	mavlink_msg_slugs_navigation_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_slugs_navigation_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.u_m , packet1.phi_c , packet1.theta_c , packet1.psiDot_c , packet1.ay_body , packet1.totalDist , packet1.dist2Go , packet1.fromWP , packet1.toWP );
	mavlink_msg_slugs_navigation_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_slugs_navigation_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_slugs_navigation_send(MAVLINK_COMM_1 , packet1.u_m , packet1.phi_c , packet1.theta_c , packet1.psiDot_c , packet1.ay_body , packet1.totalDist , packet1.dist2Go , packet1.fromWP , packet1.toWP );
	mavlink_msg_slugs_navigation_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_data_log(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_data_log_t packet_in = {
		17.0,
	45.0,
	73.0,
	101.0,
	129.0,
	157.0,
	};
	mavlink_data_log_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.fl_1 = packet_in.fl_1;
        	packet1.fl_2 = packet_in.fl_2;
        	packet1.fl_3 = packet_in.fl_3;
        	packet1.fl_4 = packet_in.fl_4;
        	packet1.fl_5 = packet_in.fl_5;
        	packet1.fl_6 = packet_in.fl_6;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_data_log_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_data_log_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_data_log_pack(system_id, component_id, &msg , packet1.fl_1 , packet1.fl_2 , packet1.fl_3 , packet1.fl_4 , packet1.fl_5 , packet1.fl_6 );
	mavlink_msg_data_log_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_data_log_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.fl_1 , packet1.fl_2 , packet1.fl_3 , packet1.fl_4 , packet1.fl_5 , packet1.fl_6 );
	mavlink_msg_data_log_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_data_log_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_data_log_send(MAVLINK_COMM_1 , packet1.fl_1 , packet1.fl_2 , packet1.fl_3 , packet1.fl_4 , packet1.fl_5 , packet1.fl_6 );
	mavlink_msg_data_log_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_gps_date_time(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_gps_date_time_t packet_in = {
		5,
	72,
	139,
	206,
	17,
	84,
	151,
	};
	mavlink_gps_date_time_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.year = packet_in.year;
        	packet1.month = packet_in.month;
        	packet1.day = packet_in.day;
        	packet1.hour = packet_in.hour;
        	packet1.min = packet_in.min;
        	packet1.sec = packet_in.sec;
        	packet1.visSat = packet_in.visSat;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_date_time_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_gps_date_time_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_date_time_pack(system_id, component_id, &msg , packet1.year , packet1.month , packet1.day , packet1.hour , packet1.min , packet1.sec , packet1.visSat );
	mavlink_msg_gps_date_time_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_date_time_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.year , packet1.month , packet1.day , packet1.hour , packet1.min , packet1.sec , packet1.visSat );
	mavlink_msg_gps_date_time_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_gps_date_time_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_date_time_send(MAVLINK_COMM_1 , packet1.year , packet1.month , packet1.day , packet1.hour , packet1.min , packet1.sec , packet1.visSat );
	mavlink_msg_gps_date_time_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_mid_lvl_cmds(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_mid_lvl_cmds_t packet_in = {
		17.0,
	45.0,
	73.0,
	41,
	};
	mavlink_mid_lvl_cmds_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.hCommand = packet_in.hCommand;
        	packet1.uCommand = packet_in.uCommand;
        	packet1.rCommand = packet_in.rCommand;
        	packet1.target = packet_in.target;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mid_lvl_cmds_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_mid_lvl_cmds_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mid_lvl_cmds_pack(system_id, component_id, &msg , packet1.target , packet1.hCommand , packet1.uCommand , packet1.rCommand );
	mavlink_msg_mid_lvl_cmds_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mid_lvl_cmds_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target , packet1.hCommand , packet1.uCommand , packet1.rCommand );
	mavlink_msg_mid_lvl_cmds_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_mid_lvl_cmds_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mid_lvl_cmds_send(MAVLINK_COMM_1 , packet1.target , packet1.hCommand , packet1.uCommand , packet1.rCommand );
	mavlink_msg_mid_lvl_cmds_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_ctrl_srfc_pt(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_ctrl_srfc_pt_t packet_in = {
		17235,
	139,
	};
	mavlink_ctrl_srfc_pt_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.bitfieldPt = packet_in.bitfieldPt;
        	packet1.target = packet_in.target;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ctrl_srfc_pt_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_ctrl_srfc_pt_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ctrl_srfc_pt_pack(system_id, component_id, &msg , packet1.target , packet1.bitfieldPt );
	mavlink_msg_ctrl_srfc_pt_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ctrl_srfc_pt_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target , packet1.bitfieldPt );
	mavlink_msg_ctrl_srfc_pt_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_ctrl_srfc_pt_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ctrl_srfc_pt_send(MAVLINK_COMM_1 , packet1.target , packet1.bitfieldPt );
	mavlink_msg_ctrl_srfc_pt_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_slugs_action(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_slugs_action_t packet_in = {
		17235,
	139,
	206,
	};
	mavlink_slugs_action_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.actionVal = packet_in.actionVal;
        	packet1.target = packet_in.target;
        	packet1.actionId = packet_in.actionId;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_slugs_action_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_slugs_action_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_slugs_action_pack(system_id, component_id, &msg , packet1.target , packet1.actionId , packet1.actionVal );
	mavlink_msg_slugs_action_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_slugs_action_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target , packet1.actionId , packet1.actionVal );
	mavlink_msg_slugs_action_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_slugs_action_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_slugs_action_send(MAVLINK_COMM_1 , packet1.target , packet1.actionId , packet1.actionVal );
	mavlink_msg_slugs_action_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_slugs(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_cpu_load(system_id, component_id, last_msg);
	mavlink_test_air_data(system_id, component_id, last_msg);
	mavlink_test_sensor_bias(system_id, component_id, last_msg);
	mavlink_test_diagnostic(system_id, component_id, last_msg);
	mavlink_test_slugs_navigation(system_id, component_id, last_msg);
	mavlink_test_data_log(system_id, component_id, last_msg);
	mavlink_test_gps_date_time(system_id, component_id, last_msg);
	mavlink_test_mid_lvl_cmds(system_id, component_id, last_msg);
	mavlink_test_ctrl_srfc_pt(system_id, component_id, last_msg);
	mavlink_test_slugs_action(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // SLUGS_TESTSUITE_H
