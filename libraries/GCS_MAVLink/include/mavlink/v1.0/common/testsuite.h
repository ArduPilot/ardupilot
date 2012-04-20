/** @file
 *	@brief MAVLink comm protocol testsuite generated from common.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef COMMON_TESTSUITE_H
#define COMMON_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

	mavlink_test_common(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_heartbeat(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_heartbeat_t packet_in = {
		963497464,
	17,
	84,
	151,
	218,
	3,
	};
	mavlink_heartbeat_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.custom_mode = packet_in.custom_mode;
        	packet1.type = packet_in.type;
        	packet1.autopilot = packet_in.autopilot;
        	packet1.base_mode = packet_in.base_mode;
        	packet1.system_status = packet_in.system_status;
        	packet1.mavlink_version = packet_in.mavlink_version;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_heartbeat_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_heartbeat_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_heartbeat_pack(system_id, component_id, &msg , packet1.type , packet1.autopilot , packet1.base_mode , packet1.custom_mode , packet1.system_status );
	mavlink_msg_heartbeat_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_heartbeat_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.type , packet1.autopilot , packet1.base_mode , packet1.custom_mode , packet1.system_status );
	mavlink_msg_heartbeat_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_heartbeat_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_heartbeat_send(MAVLINK_COMM_1 , packet1.type , packet1.autopilot , packet1.base_mode , packet1.custom_mode , packet1.system_status );
	mavlink_msg_heartbeat_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_sys_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_sys_status_t packet_in = {
		963497464,
	963497672,
	963497880,
	17859,
	17963,
	18067,
	18171,
	18275,
	18379,
	18483,
	18587,
	18691,
	223,
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
}

static void mavlink_test_system_time(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_system_time_t packet_in = {
		93372036854775807ULL,
	963497880,
	};
	mavlink_system_time_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_unix_usec = packet_in.time_unix_usec;
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        
        

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
}

static void mavlink_test_ping(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_ping_t packet_in = {
		93372036854775807ULL,
	963497880,
	41,
	108,
	};
	mavlink_ping_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_usec = packet_in.time_usec;
        	packet1.seq = packet_in.seq;
        	packet1.target_system = packet_in.target_system;
        	packet1.target_component = packet_in.target_component;
        
        

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
}

static void mavlink_test_change_operator_control(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_change_operator_control_t packet_in = {
		5,
	72,
	139,
	"DEFGHIJKLMNOPQRSTUVWXYZA",
	};
	mavlink_change_operator_control_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.target_system = packet_in.target_system;
        	packet1.control_request = packet_in.control_request;
        	packet1.version = packet_in.version;
        
        	mav_array_memcpy(packet1.passkey, packet_in.passkey, sizeof(char)*25);
        

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
}

static void mavlink_test_change_operator_control_ack(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_change_operator_control_ack_t packet_in = {
		5,
	72,
	139,
	};
	mavlink_change_operator_control_ack_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.gcs_system_id = packet_in.gcs_system_id;
        	packet1.control_request = packet_in.control_request;
        	packet1.ack = packet_in.ack;
        
        

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
}

static void mavlink_test_auth_key(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_auth_key_t packet_in = {
		"ABCDEFGHIJKLMNOPQRSTUVWXYZABCDE",
	};
	mavlink_auth_key_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        	mav_array_memcpy(packet1.key, packet_in.key, sizeof(char)*32);
        

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
}

static void mavlink_test_set_mode(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_set_mode_t packet_in = {
		963497464,
	17,
	84,
	};
	mavlink_set_mode_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.custom_mode = packet_in.custom_mode;
        	packet1.target_system = packet_in.target_system;
        	packet1.base_mode = packet_in.base_mode;
        
        

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
}

static void mavlink_test_param_request_read(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_param_request_read_t packet_in = {
		17235,
	139,
	206,
	"EFGHIJKLMNOPQRS",
	};
	mavlink_param_request_read_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.param_index = packet_in.param_index;
        	packet1.target_system = packet_in.target_system;
        	packet1.target_component = packet_in.target_component;
        
        	mav_array_memcpy(packet1.param_id, packet_in.param_id, sizeof(char)*16);
        

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
}

static void mavlink_test_param_request_list(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_param_request_list_t packet_in = {
		5,
	72,
	};
	mavlink_param_request_list_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.target_system = packet_in.target_system;
        	packet1.target_component = packet_in.target_component;
        
        

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
}

static void mavlink_test_param_value(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_param_value_t packet_in = {
		17.0,
	17443,
	17547,
	"IJKLMNOPQRSTUVW",
	77,
	};
	mavlink_param_value_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.param_value = packet_in.param_value;
        	packet1.param_count = packet_in.param_count;
        	packet1.param_index = packet_in.param_index;
        	packet1.param_type = packet_in.param_type;
        
        	mav_array_memcpy(packet1.param_id, packet_in.param_id, sizeof(char)*16);
        

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
}

static void mavlink_test_param_set(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_param_set_t packet_in = {
		17.0,
	17,
	84,
	"GHIJKLMNOPQRSTU",
	199,
	};
	mavlink_param_set_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.param_value = packet_in.param_value;
        	packet1.target_system = packet_in.target_system;
        	packet1.target_component = packet_in.target_component;
        	packet1.param_type = packet_in.param_type;
        
        	mav_array_memcpy(packet1.param_id, packet_in.param_id, sizeof(char)*16);
        

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
}

static void mavlink_test_gps_raw_int(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_gps_raw_int_t packet_in = {
		93372036854775807ULL,
	963497880,
	963498088,
	963498296,
	18275,
	18379,
	18483,
	18587,
	89,
	156,
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
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_raw_int_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_gps_raw_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_raw_int_pack(system_id, component_id, &msg , packet1.time_usec , packet1.fix_type , packet1.lat , packet1.lon , packet1.alt , packet1.eph , packet1.epv , packet1.vel , packet1.cog , packet1.satellites_visible );
	mavlink_msg_gps_raw_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_raw_int_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.fix_type , packet1.lat , packet1.lon , packet1.alt , packet1.eph , packet1.epv , packet1.vel , packet1.cog , packet1.satellites_visible );
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
	mavlink_msg_gps_raw_int_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.fix_type , packet1.lat , packet1.lon , packet1.alt , packet1.eph , packet1.epv , packet1.vel , packet1.cog , packet1.satellites_visible );
	mavlink_msg_gps_raw_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_gps_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_gps_status_t packet_in = {
		5,
	{ 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91 },
	{ 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151 },
	{ 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211 },
	{ 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 },
	{ 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75 },
	};
	mavlink_gps_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.satellites_visible = packet_in.satellites_visible;
        
        	mav_array_memcpy(packet1.satellite_prn, packet_in.satellite_prn, sizeof(uint8_t)*20);
        	mav_array_memcpy(packet1.satellite_used, packet_in.satellite_used, sizeof(uint8_t)*20);
        	mav_array_memcpy(packet1.satellite_elevation, packet_in.satellite_elevation, sizeof(uint8_t)*20);
        	mav_array_memcpy(packet1.satellite_azimuth, packet_in.satellite_azimuth, sizeof(uint8_t)*20);
        	mav_array_memcpy(packet1.satellite_snr, packet_in.satellite_snr, sizeof(uint8_t)*20);
        

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
}

static void mavlink_test_scaled_imu(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_scaled_imu_t packet_in = {
		963497464,
	17443,
	17547,
	17651,
	17755,
	17859,
	17963,
	18067,
	18171,
	18275,
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
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_scaled_imu_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_scaled_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_scaled_imu_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xmag , packet1.ymag , packet1.zmag );
	mavlink_msg_scaled_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_scaled_imu_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xmag , packet1.ymag , packet1.zmag );
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
	mavlink_msg_scaled_imu_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xmag , packet1.ymag , packet1.zmag );
	mavlink_msg_scaled_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_raw_imu(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_raw_imu_t packet_in = {
		93372036854775807ULL,
	17651,
	17755,
	17859,
	17963,
	18067,
	18171,
	18275,
	18379,
	18483,
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
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_raw_imu_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_raw_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_raw_imu_pack(system_id, component_id, &msg , packet1.time_usec , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xmag , packet1.ymag , packet1.zmag );
	mavlink_msg_raw_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_raw_imu_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xmag , packet1.ymag , packet1.zmag );
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
	mavlink_msg_raw_imu_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xmag , packet1.ymag , packet1.zmag );
	mavlink_msg_raw_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_raw_pressure(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_raw_pressure_t packet_in = {
		93372036854775807ULL,
	17651,
	17755,
	17859,
	17963,
	};
	mavlink_raw_pressure_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_usec = packet_in.time_usec;
        	packet1.press_abs = packet_in.press_abs;
        	packet1.press_diff1 = packet_in.press_diff1;
        	packet1.press_diff2 = packet_in.press_diff2;
        	packet1.temperature = packet_in.temperature;
        
        

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
}

static void mavlink_test_scaled_pressure(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_scaled_pressure_t packet_in = {
		963497464,
	45.0,
	73.0,
	17859,
	};
	mavlink_scaled_pressure_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        	packet1.press_abs = packet_in.press_abs;
        	packet1.press_diff = packet_in.press_diff;
        	packet1.temperature = packet_in.temperature;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_scaled_pressure_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_scaled_pressure_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_scaled_pressure_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.press_abs , packet1.press_diff , packet1.temperature );
	mavlink_msg_scaled_pressure_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_scaled_pressure_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.press_abs , packet1.press_diff , packet1.temperature );
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
	mavlink_msg_scaled_pressure_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.press_abs , packet1.press_diff , packet1.temperature );
	mavlink_msg_scaled_pressure_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_attitude(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_attitude_t packet_in = {
		963497464,
	45.0,
	73.0,
	101.0,
	129.0,
	157.0,
	185.0,
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
}

static void mavlink_test_attitude_quaternion(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_attitude_quaternion_t packet_in = {
		963497464,
	45.0,
	73.0,
	101.0,
	129.0,
	157.0,
	185.0,
	213.0,
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
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_attitude_quaternion_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_attitude_quaternion_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_attitude_quaternion_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.q1 , packet1.q2 , packet1.q3 , packet1.q4 , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed );
	mavlink_msg_attitude_quaternion_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_attitude_quaternion_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.q1 , packet1.q2 , packet1.q3 , packet1.q4 , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed );
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
	mavlink_msg_attitude_quaternion_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.q1 , packet1.q2 , packet1.q3 , packet1.q4 , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed );
	mavlink_msg_attitude_quaternion_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_local_position_ned(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_local_position_ned_t packet_in = {
		963497464,
	45.0,
	73.0,
	101.0,
	129.0,
	157.0,
	185.0,
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
}

static void mavlink_test_global_position_int(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_global_position_int_t packet_in = {
		963497464,
	963497672,
	963497880,
	963498088,
	963498296,
	18275,
	18379,
	18483,
	18587,
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
}

static void mavlink_test_rc_channels_scaled(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_rc_channels_scaled_t packet_in = {
		963497464,
	17443,
	17547,
	17651,
	17755,
	17859,
	17963,
	18067,
	18171,
	65,
	132,
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
}

static void mavlink_test_rc_channels_raw(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_rc_channels_raw_t packet_in = {
		963497464,
	17443,
	17547,
	17651,
	17755,
	17859,
	17963,
	18067,
	18171,
	65,
	132,
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
}

static void mavlink_test_servo_output_raw(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_servo_output_raw_t packet_in = {
		963497464,
	17443,
	17547,
	17651,
	17755,
	17859,
	17963,
	18067,
	18171,
	65,
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
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_servo_output_raw_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_servo_output_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_servo_output_raw_pack(system_id, component_id, &msg , packet1.time_usec , packet1.port , packet1.servo1_raw , packet1.servo2_raw , packet1.servo3_raw , packet1.servo4_raw , packet1.servo5_raw , packet1.servo6_raw , packet1.servo7_raw , packet1.servo8_raw );
	mavlink_msg_servo_output_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_servo_output_raw_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.port , packet1.servo1_raw , packet1.servo2_raw , packet1.servo3_raw , packet1.servo4_raw , packet1.servo5_raw , packet1.servo6_raw , packet1.servo7_raw , packet1.servo8_raw );
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
	mavlink_msg_servo_output_raw_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.port , packet1.servo1_raw , packet1.servo2_raw , packet1.servo3_raw , packet1.servo4_raw , packet1.servo5_raw , packet1.servo6_raw , packet1.servo7_raw , packet1.servo8_raw );
	mavlink_msg_servo_output_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_mission_request_partial_list(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_mission_request_partial_list_t packet_in = {
		17235,
	17339,
	17,
	84,
	};
	mavlink_mission_request_partial_list_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.start_index = packet_in.start_index;
        	packet1.end_index = packet_in.end_index;
        	packet1.target_system = packet_in.target_system;
        	packet1.target_component = packet_in.target_component;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_request_partial_list_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_mission_request_partial_list_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_request_partial_list_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.start_index , packet1.end_index );
	mavlink_msg_mission_request_partial_list_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_request_partial_list_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.start_index , packet1.end_index );
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
	mavlink_msg_mission_request_partial_list_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.start_index , packet1.end_index );
	mavlink_msg_mission_request_partial_list_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_mission_write_partial_list(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_mission_write_partial_list_t packet_in = {
		17235,
	17339,
	17,
	84,
	};
	mavlink_mission_write_partial_list_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.start_index = packet_in.start_index;
        	packet1.end_index = packet_in.end_index;
        	packet1.target_system = packet_in.target_system;
        	packet1.target_component = packet_in.target_component;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_write_partial_list_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_mission_write_partial_list_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_write_partial_list_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.start_index , packet1.end_index );
	mavlink_msg_mission_write_partial_list_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_write_partial_list_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.start_index , packet1.end_index );
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
	mavlink_msg_mission_write_partial_list_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.start_index , packet1.end_index );
	mavlink_msg_mission_write_partial_list_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_mission_item(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_mission_item_t packet_in = {
		17.0,
	45.0,
	73.0,
	101.0,
	129.0,
	157.0,
	185.0,
	18691,
	18795,
	101,
	168,
	235,
	46,
	113,
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
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_item_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_mission_item_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_item_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.seq , packet1.frame , packet1.command , packet1.current , packet1.autocontinue , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.x , packet1.y , packet1.z );
	mavlink_msg_mission_item_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_item_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.seq , packet1.frame , packet1.command , packet1.current , packet1.autocontinue , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.x , packet1.y , packet1.z );
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
	mavlink_msg_mission_item_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.seq , packet1.frame , packet1.command , packet1.current , packet1.autocontinue , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.x , packet1.y , packet1.z );
	mavlink_msg_mission_item_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_mission_request(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_mission_request_t packet_in = {
		17235,
	139,
	206,
	};
	mavlink_mission_request_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.seq = packet_in.seq;
        	packet1.target_system = packet_in.target_system;
        	packet1.target_component = packet_in.target_component;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_request_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_mission_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_request_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.seq );
	mavlink_msg_mission_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_request_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.seq );
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
	mavlink_msg_mission_request_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.seq );
	mavlink_msg_mission_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_mission_set_current(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_mission_set_current_t packet_in = {
		17235,
	139,
	206,
	};
	mavlink_mission_set_current_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.seq = packet_in.seq;
        	packet1.target_system = packet_in.target_system;
        	packet1.target_component = packet_in.target_component;
        
        

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
}

static void mavlink_test_mission_current(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_mission_current_t packet_in = {
		17235,
	};
	mavlink_mission_current_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.seq = packet_in.seq;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_current_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_mission_current_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_current_pack(system_id, component_id, &msg , packet1.seq );
	mavlink_msg_mission_current_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_current_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.seq );
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
	mavlink_msg_mission_current_send(MAVLINK_COMM_1 , packet1.seq );
	mavlink_msg_mission_current_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_mission_request_list(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_mission_request_list_t packet_in = {
		5,
	72,
	};
	mavlink_mission_request_list_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.target_system = packet_in.target_system;
        	packet1.target_component = packet_in.target_component;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_request_list_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_mission_request_list_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_request_list_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component );
	mavlink_msg_mission_request_list_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_request_list_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component );
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
	mavlink_msg_mission_request_list_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component );
	mavlink_msg_mission_request_list_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_mission_count(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_mission_count_t packet_in = {
		17235,
	139,
	206,
	};
	mavlink_mission_count_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.count = packet_in.count;
        	packet1.target_system = packet_in.target_system;
        	packet1.target_component = packet_in.target_component;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_count_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_mission_count_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_count_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.count );
	mavlink_msg_mission_count_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_count_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.count );
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
	mavlink_msg_mission_count_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.count );
	mavlink_msg_mission_count_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_mission_clear_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_mission_clear_all_t packet_in = {
		5,
	72,
	};
	mavlink_mission_clear_all_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.target_system = packet_in.target_system;
        	packet1.target_component = packet_in.target_component;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_clear_all_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_mission_clear_all_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_clear_all_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component );
	mavlink_msg_mission_clear_all_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_clear_all_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component );
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
	mavlink_msg_mission_clear_all_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component );
	mavlink_msg_mission_clear_all_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_mission_item_reached(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_mission_item_reached_t packet_in = {
		17235,
	};
	mavlink_mission_item_reached_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.seq = packet_in.seq;
        
        

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
}

static void mavlink_test_mission_ack(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_mission_ack_t packet_in = {
		5,
	72,
	139,
	};
	mavlink_mission_ack_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.target_system = packet_in.target_system;
        	packet1.target_component = packet_in.target_component;
        	packet1.type = packet_in.type;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_ack_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_mission_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_ack_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.type );
	mavlink_msg_mission_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_ack_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.type );
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
	mavlink_msg_mission_ack_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.type );
	mavlink_msg_mission_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_set_gps_global_origin(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_set_gps_global_origin_t packet_in = {
		963497464,
	963497672,
	963497880,
	41,
	};
	mavlink_set_gps_global_origin_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.latitude = packet_in.latitude;
        	packet1.longitude = packet_in.longitude;
        	packet1.altitude = packet_in.altitude;
        	packet1.target_system = packet_in.target_system;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_gps_global_origin_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_set_gps_global_origin_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_gps_global_origin_pack(system_id, component_id, &msg , packet1.target_system , packet1.latitude , packet1.longitude , packet1.altitude );
	mavlink_msg_set_gps_global_origin_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_gps_global_origin_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.latitude , packet1.longitude , packet1.altitude );
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
	mavlink_msg_set_gps_global_origin_send(MAVLINK_COMM_1 , packet1.target_system , packet1.latitude , packet1.longitude , packet1.altitude );
	mavlink_msg_set_gps_global_origin_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_gps_global_origin(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_gps_global_origin_t packet_in = {
		963497464,
	963497672,
	963497880,
	};
	mavlink_gps_global_origin_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.latitude = packet_in.latitude;
        	packet1.longitude = packet_in.longitude;
        	packet1.altitude = packet_in.altitude;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_global_origin_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_gps_global_origin_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_global_origin_pack(system_id, component_id, &msg , packet1.latitude , packet1.longitude , packet1.altitude );
	mavlink_msg_gps_global_origin_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_global_origin_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.latitude , packet1.longitude , packet1.altitude );
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
	mavlink_msg_gps_global_origin_send(MAVLINK_COMM_1 , packet1.latitude , packet1.longitude , packet1.altitude );
	mavlink_msg_gps_global_origin_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_set_local_position_setpoint(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_set_local_position_setpoint_t packet_in = {
		17.0,
	45.0,
	73.0,
	101.0,
	53,
	120,
	187,
	};
	mavlink_set_local_position_setpoint_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.x = packet_in.x;
        	packet1.y = packet_in.y;
        	packet1.z = packet_in.z;
        	packet1.yaw = packet_in.yaw;
        	packet1.target_system = packet_in.target_system;
        	packet1.target_component = packet_in.target_component;
        	packet1.coordinate_frame = packet_in.coordinate_frame;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_local_position_setpoint_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_set_local_position_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_local_position_setpoint_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.coordinate_frame , packet1.x , packet1.y , packet1.z , packet1.yaw );
	mavlink_msg_set_local_position_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_local_position_setpoint_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.coordinate_frame , packet1.x , packet1.y , packet1.z , packet1.yaw );
	mavlink_msg_set_local_position_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_set_local_position_setpoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_local_position_setpoint_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.coordinate_frame , packet1.x , packet1.y , packet1.z , packet1.yaw );
	mavlink_msg_set_local_position_setpoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_local_position_setpoint(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_local_position_setpoint_t packet_in = {
		17.0,
	45.0,
	73.0,
	101.0,
	53,
	};
	mavlink_local_position_setpoint_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.x = packet_in.x;
        	packet1.y = packet_in.y;
        	packet1.z = packet_in.z;
        	packet1.yaw = packet_in.yaw;
        	packet1.coordinate_frame = packet_in.coordinate_frame;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_local_position_setpoint_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_local_position_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_local_position_setpoint_pack(system_id, component_id, &msg , packet1.coordinate_frame , packet1.x , packet1.y , packet1.z , packet1.yaw );
	mavlink_msg_local_position_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_local_position_setpoint_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.coordinate_frame , packet1.x , packet1.y , packet1.z , packet1.yaw );
	mavlink_msg_local_position_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_local_position_setpoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_local_position_setpoint_send(MAVLINK_COMM_1 , packet1.coordinate_frame , packet1.x , packet1.y , packet1.z , packet1.yaw );
	mavlink_msg_local_position_setpoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_global_position_setpoint_int(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_global_position_setpoint_int_t packet_in = {
		963497464,
	963497672,
	963497880,
	17859,
	175,
	};
	mavlink_global_position_setpoint_int_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.latitude = packet_in.latitude;
        	packet1.longitude = packet_in.longitude;
        	packet1.altitude = packet_in.altitude;
        	packet1.yaw = packet_in.yaw;
        	packet1.coordinate_frame = packet_in.coordinate_frame;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_global_position_setpoint_int_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_global_position_setpoint_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_global_position_setpoint_int_pack(system_id, component_id, &msg , packet1.coordinate_frame , packet1.latitude , packet1.longitude , packet1.altitude , packet1.yaw );
	mavlink_msg_global_position_setpoint_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_global_position_setpoint_int_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.coordinate_frame , packet1.latitude , packet1.longitude , packet1.altitude , packet1.yaw );
	mavlink_msg_global_position_setpoint_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_global_position_setpoint_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_global_position_setpoint_int_send(MAVLINK_COMM_1 , packet1.coordinate_frame , packet1.latitude , packet1.longitude , packet1.altitude , packet1.yaw );
	mavlink_msg_global_position_setpoint_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_set_global_position_setpoint_int(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_set_global_position_setpoint_int_t packet_in = {
		963497464,
	963497672,
	963497880,
	17859,
	175,
	};
	mavlink_set_global_position_setpoint_int_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.latitude = packet_in.latitude;
        	packet1.longitude = packet_in.longitude;
        	packet1.altitude = packet_in.altitude;
        	packet1.yaw = packet_in.yaw;
        	packet1.coordinate_frame = packet_in.coordinate_frame;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_global_position_setpoint_int_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_set_global_position_setpoint_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_global_position_setpoint_int_pack(system_id, component_id, &msg , packet1.coordinate_frame , packet1.latitude , packet1.longitude , packet1.altitude , packet1.yaw );
	mavlink_msg_set_global_position_setpoint_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_global_position_setpoint_int_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.coordinate_frame , packet1.latitude , packet1.longitude , packet1.altitude , packet1.yaw );
	mavlink_msg_set_global_position_setpoint_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_set_global_position_setpoint_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_global_position_setpoint_int_send(MAVLINK_COMM_1 , packet1.coordinate_frame , packet1.latitude , packet1.longitude , packet1.altitude , packet1.yaw );
	mavlink_msg_set_global_position_setpoint_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_safety_set_allowed_area(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_safety_set_allowed_area_t packet_in = {
		17.0,
	45.0,
	73.0,
	101.0,
	129.0,
	157.0,
	77,
	144,
	211,
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
}

static void mavlink_test_safety_allowed_area(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_safety_allowed_area_t packet_in = {
		17.0,
	45.0,
	73.0,
	101.0,
	129.0,
	157.0,
	77,
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
}

static void mavlink_test_set_roll_pitch_yaw_thrust(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_set_roll_pitch_yaw_thrust_t packet_in = {
		17.0,
	45.0,
	73.0,
	101.0,
	53,
	120,
	};
	mavlink_set_roll_pitch_yaw_thrust_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.roll = packet_in.roll;
        	packet1.pitch = packet_in.pitch;
        	packet1.yaw = packet_in.yaw;
        	packet1.thrust = packet_in.thrust;
        	packet1.target_system = packet_in.target_system;
        	packet1.target_component = packet_in.target_component;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_roll_pitch_yaw_thrust_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_set_roll_pitch_yaw_thrust_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_roll_pitch_yaw_thrust_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust );
	mavlink_msg_set_roll_pitch_yaw_thrust_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_roll_pitch_yaw_thrust_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust );
	mavlink_msg_set_roll_pitch_yaw_thrust_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_set_roll_pitch_yaw_thrust_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_roll_pitch_yaw_thrust_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust );
	mavlink_msg_set_roll_pitch_yaw_thrust_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_set_roll_pitch_yaw_speed_thrust(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_set_roll_pitch_yaw_speed_thrust_t packet_in = {
		17.0,
	45.0,
	73.0,
	101.0,
	53,
	120,
	};
	mavlink_set_roll_pitch_yaw_speed_thrust_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.roll_speed = packet_in.roll_speed;
        	packet1.pitch_speed = packet_in.pitch_speed;
        	packet1.yaw_speed = packet_in.yaw_speed;
        	packet1.thrust = packet_in.thrust;
        	packet1.target_system = packet_in.target_system;
        	packet1.target_component = packet_in.target_component;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_roll_pitch_yaw_speed_thrust_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_set_roll_pitch_yaw_speed_thrust_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_roll_pitch_yaw_speed_thrust_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.roll_speed , packet1.pitch_speed , packet1.yaw_speed , packet1.thrust );
	mavlink_msg_set_roll_pitch_yaw_speed_thrust_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_roll_pitch_yaw_speed_thrust_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.roll_speed , packet1.pitch_speed , packet1.yaw_speed , packet1.thrust );
	mavlink_msg_set_roll_pitch_yaw_speed_thrust_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_set_roll_pitch_yaw_speed_thrust_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_roll_pitch_yaw_speed_thrust_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.roll_speed , packet1.pitch_speed , packet1.yaw_speed , packet1.thrust );
	mavlink_msg_set_roll_pitch_yaw_speed_thrust_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_roll_pitch_yaw_thrust_setpoint(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_roll_pitch_yaw_thrust_setpoint_t packet_in = {
		963497464,
	45.0,
	73.0,
	101.0,
	129.0,
	};
	mavlink_roll_pitch_yaw_thrust_setpoint_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        	packet1.roll = packet_in.roll;
        	packet1.pitch = packet_in.pitch;
        	packet1.yaw = packet_in.yaw;
        	packet1.thrust = packet_in.thrust;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_roll_pitch_yaw_thrust_setpoint_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_roll_pitch_yaw_thrust_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_roll_pitch_yaw_thrust_setpoint_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust );
	mavlink_msg_roll_pitch_yaw_thrust_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_roll_pitch_yaw_thrust_setpoint_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust );
	mavlink_msg_roll_pitch_yaw_thrust_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_roll_pitch_yaw_thrust_setpoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_roll_pitch_yaw_thrust_setpoint_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust );
	mavlink_msg_roll_pitch_yaw_thrust_setpoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_roll_pitch_yaw_speed_thrust_setpoint(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_roll_pitch_yaw_speed_thrust_setpoint_t packet_in = {
		963497464,
	45.0,
	73.0,
	101.0,
	129.0,
	};
	mavlink_roll_pitch_yaw_speed_thrust_setpoint_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        	packet1.roll_speed = packet_in.roll_speed;
        	packet1.pitch_speed = packet_in.pitch_speed;
        	packet1.yaw_speed = packet_in.yaw_speed;
        	packet1.thrust = packet_in.thrust;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.roll_speed , packet1.pitch_speed , packet1.yaw_speed , packet1.thrust );
	mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.roll_speed , packet1.pitch_speed , packet1.yaw_speed , packet1.thrust );
	mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.roll_speed , packet1.pitch_speed , packet1.yaw_speed , packet1.thrust );
	mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_set_quad_motors_setpoint(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_set_quad_motors_setpoint_t packet_in = {
		17235,
	17339,
	17443,
	17547,
	29,
	};
	mavlink_set_quad_motors_setpoint_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.motor_front_nw = packet_in.motor_front_nw;
        	packet1.motor_right_ne = packet_in.motor_right_ne;
        	packet1.motor_back_se = packet_in.motor_back_se;
        	packet1.motor_left_sw = packet_in.motor_left_sw;
        	packet1.target_system = packet_in.target_system;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_quad_motors_setpoint_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_set_quad_motors_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_quad_motors_setpoint_pack(system_id, component_id, &msg , packet1.target_system , packet1.motor_front_nw , packet1.motor_right_ne , packet1.motor_back_se , packet1.motor_left_sw );
	mavlink_msg_set_quad_motors_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_quad_motors_setpoint_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.motor_front_nw , packet1.motor_right_ne , packet1.motor_back_se , packet1.motor_left_sw );
	mavlink_msg_set_quad_motors_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_set_quad_motors_setpoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_quad_motors_setpoint_send(MAVLINK_COMM_1 , packet1.target_system , packet1.motor_front_nw , packet1.motor_right_ne , packet1.motor_back_se , packet1.motor_left_sw );
	mavlink_msg_set_quad_motors_setpoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_set_quad_swarm_roll_pitch_yaw_thrust(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t packet_in = {
		{ 17235, 17236, 17237, 17238, 17239, 17240 },
	{ 17859, 17860, 17861, 17862, 17863, 17864 },
	{ 18483, 18484, 18485, 18486, 18487, 18488 },
	{ 19107, 19108, 19109, 19110, 19111, 19112 },
	{ 149, 150, 151, 152, 153, 154 },
	};
	mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        	mav_array_memcpy(packet1.roll, packet_in.roll, sizeof(int16_t)*6);
        	mav_array_memcpy(packet1.pitch, packet_in.pitch, sizeof(int16_t)*6);
        	mav_array_memcpy(packet1.yaw, packet_in.yaw, sizeof(int16_t)*6);
        	mav_array_memcpy(packet1.thrust, packet_in.thrust, sizeof(uint16_t)*6);
        	mav_array_memcpy(packet1.target_systems, packet_in.target_systems, sizeof(uint8_t)*6);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_pack(system_id, component_id, &msg , packet1.target_systems , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust );
	mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_systems , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust );
	mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_send(MAVLINK_COMM_1 , packet1.target_systems , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust );
	mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_nav_controller_output(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_nav_controller_output_t packet_in = {
		17.0,
	45.0,
	73.0,
	101.0,
	129.0,
	18275,
	18379,
	18483,
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
}

static void mavlink_test_state_correction(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_state_correction_t packet_in = {
		17.0,
	45.0,
	73.0,
	101.0,
	129.0,
	157.0,
	185.0,
	213.0,
	241.0,
	};
	mavlink_state_correction_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.xErr = packet_in.xErr;
        	packet1.yErr = packet_in.yErr;
        	packet1.zErr = packet_in.zErr;
        	packet1.rollErr = packet_in.rollErr;
        	packet1.pitchErr = packet_in.pitchErr;
        	packet1.yawErr = packet_in.yawErr;
        	packet1.vxErr = packet_in.vxErr;
        	packet1.vyErr = packet_in.vyErr;
        	packet1.vzErr = packet_in.vzErr;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_state_correction_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_state_correction_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_state_correction_pack(system_id, component_id, &msg , packet1.xErr , packet1.yErr , packet1.zErr , packet1.rollErr , packet1.pitchErr , packet1.yawErr , packet1.vxErr , packet1.vyErr , packet1.vzErr );
	mavlink_msg_state_correction_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_state_correction_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.xErr , packet1.yErr , packet1.zErr , packet1.rollErr , packet1.pitchErr , packet1.yawErr , packet1.vxErr , packet1.vyErr , packet1.vzErr );
	mavlink_msg_state_correction_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_state_correction_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_state_correction_send(MAVLINK_COMM_1 , packet1.xErr , packet1.yErr , packet1.zErr , packet1.rollErr , packet1.pitchErr , packet1.yawErr , packet1.vxErr , packet1.vyErr , packet1.vzErr );
	mavlink_msg_state_correction_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_request_data_stream(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_request_data_stream_t packet_in = {
		17235,
	139,
	206,
	17,
	84,
	};
	mavlink_request_data_stream_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.req_message_rate = packet_in.req_message_rate;
        	packet1.target_system = packet_in.target_system;
        	packet1.target_component = packet_in.target_component;
        	packet1.req_stream_id = packet_in.req_stream_id;
        	packet1.start_stop = packet_in.start_stop;
        
        

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
}

static void mavlink_test_data_stream(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_data_stream_t packet_in = {
		17235,
	139,
	206,
	};
	mavlink_data_stream_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.message_rate = packet_in.message_rate;
        	packet1.stream_id = packet_in.stream_id;
        	packet1.on_off = packet_in.on_off;
        
        

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
}

static void mavlink_test_manual_control(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_manual_control_t packet_in = {
		17.0,
	45.0,
	73.0,
	101.0,
	53,
	120,
	187,
	254,
	65,
	};
	mavlink_manual_control_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.roll = packet_in.roll;
        	packet1.pitch = packet_in.pitch;
        	packet1.yaw = packet_in.yaw;
        	packet1.thrust = packet_in.thrust;
        	packet1.target = packet_in.target;
        	packet1.roll_manual = packet_in.roll_manual;
        	packet1.pitch_manual = packet_in.pitch_manual;
        	packet1.yaw_manual = packet_in.yaw_manual;
        	packet1.thrust_manual = packet_in.thrust_manual;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_manual_control_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_manual_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_manual_control_pack(system_id, component_id, &msg , packet1.target , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust , packet1.roll_manual , packet1.pitch_manual , packet1.yaw_manual , packet1.thrust_manual );
	mavlink_msg_manual_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_manual_control_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust , packet1.roll_manual , packet1.pitch_manual , packet1.yaw_manual , packet1.thrust_manual );
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
	mavlink_msg_manual_control_send(MAVLINK_COMM_1 , packet1.target , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust , packet1.roll_manual , packet1.pitch_manual , packet1.yaw_manual , packet1.thrust_manual );
	mavlink_msg_manual_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_rc_channels_override(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_rc_channels_override_t packet_in = {
		17235,
	17339,
	17443,
	17547,
	17651,
	17755,
	17859,
	17963,
	53,
	120,
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
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rc_channels_override_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_rc_channels_override_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rc_channels_override_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.chan1_raw , packet1.chan2_raw , packet1.chan3_raw , packet1.chan4_raw , packet1.chan5_raw , packet1.chan6_raw , packet1.chan7_raw , packet1.chan8_raw );
	mavlink_msg_rc_channels_override_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rc_channels_override_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.chan1_raw , packet1.chan2_raw , packet1.chan3_raw , packet1.chan4_raw , packet1.chan5_raw , packet1.chan6_raw , packet1.chan7_raw , packet1.chan8_raw );
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
	mavlink_msg_rc_channels_override_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.chan1_raw , packet1.chan2_raw , packet1.chan3_raw , packet1.chan4_raw , packet1.chan5_raw , packet1.chan6_raw , packet1.chan7_raw , packet1.chan8_raw );
	mavlink_msg_rc_channels_override_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_vfr_hud(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_vfr_hud_t packet_in = {
		17.0,
	45.0,
	73.0,
	101.0,
	18067,
	18171,
	};
	mavlink_vfr_hud_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.airspeed = packet_in.airspeed;
        	packet1.groundspeed = packet_in.groundspeed;
        	packet1.alt = packet_in.alt;
        	packet1.climb = packet_in.climb;
        	packet1.heading = packet_in.heading;
        	packet1.throttle = packet_in.throttle;
        
        

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
}

static void mavlink_test_command_long(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_command_long_t packet_in = {
		17.0,
	45.0,
	73.0,
	101.0,
	129.0,
	157.0,
	185.0,
	18691,
	223,
	34,
	101,
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
}

static void mavlink_test_command_ack(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_command_ack_t packet_in = {
		17235,
	139,
	};
	mavlink_command_ack_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.command = packet_in.command;
        	packet1.result = packet_in.result;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_command_ack_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_command_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_command_ack_pack(system_id, component_id, &msg , packet1.command , packet1.result );
	mavlink_msg_command_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_command_ack_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.command , packet1.result );
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
	mavlink_msg_command_ack_send(MAVLINK_COMM_1 , packet1.command , packet1.result );
	mavlink_msg_command_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_local_position_ned_system_global_offset(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_local_position_ned_system_global_offset_t packet_in = {
		963497464,
	45.0,
	73.0,
	101.0,
	129.0,
	157.0,
	185.0,
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
}

static void mavlink_test_hil_state(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_hil_state_t packet_in = {
		93372036854775807ULL,
	73.0,
	101.0,
	129.0,
	157.0,
	185.0,
	213.0,
	963499128,
	963499336,
	963499544,
	19523,
	19627,
	19731,
	19835,
	19939,
	20043,
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
}

static void mavlink_test_hil_controls(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_hil_controls_t packet_in = {
		93372036854775807ULL,
	73.0,
	101.0,
	129.0,
	157.0,
	185.0,
	213.0,
	241.0,
	269.0,
	125,
	192,
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
}

static void mavlink_test_hil_rc_inputs_raw(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_hil_rc_inputs_raw_t packet_in = {
		93372036854775807ULL,
	17651,
	17755,
	17859,
	17963,
	18067,
	18171,
	18275,
	18379,
	18483,
	18587,
	18691,
	18795,
	101,
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
}

static void mavlink_test_optical_flow(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_optical_flow_t packet_in = {
		93372036854775807ULL,
	73.0,
	101.0,
	129.0,
	18275,
	18379,
	77,
	144,
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
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_optical_flow_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_optical_flow_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_optical_flow_pack(system_id, component_id, &msg , packet1.time_usec , packet1.sensor_id , packet1.flow_x , packet1.flow_y , packet1.flow_comp_m_x , packet1.flow_comp_m_y , packet1.quality , packet1.ground_distance );
	mavlink_msg_optical_flow_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_optical_flow_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.sensor_id , packet1.flow_x , packet1.flow_y , packet1.flow_comp_m_x , packet1.flow_comp_m_y , packet1.quality , packet1.ground_distance );
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
	mavlink_msg_optical_flow_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.sensor_id , packet1.flow_x , packet1.flow_y , packet1.flow_comp_m_x , packet1.flow_comp_m_y , packet1.quality , packet1.ground_distance );
	mavlink_msg_optical_flow_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_global_vision_position_estimate(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_global_vision_position_estimate_t packet_in = {
		93372036854775807ULL,
	73.0,
	101.0,
	129.0,
	157.0,
	185.0,
	213.0,
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
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_global_vision_position_estimate_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_global_vision_position_estimate_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_global_vision_position_estimate_pack(system_id, component_id, &msg , packet1.usec , packet1.x , packet1.y , packet1.z , packet1.roll , packet1.pitch , packet1.yaw );
	mavlink_msg_global_vision_position_estimate_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_global_vision_position_estimate_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.usec , packet1.x , packet1.y , packet1.z , packet1.roll , packet1.pitch , packet1.yaw );
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
	mavlink_msg_global_vision_position_estimate_send(MAVLINK_COMM_1 , packet1.usec , packet1.x , packet1.y , packet1.z , packet1.roll , packet1.pitch , packet1.yaw );
	mavlink_msg_global_vision_position_estimate_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_vision_position_estimate(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_vision_position_estimate_t packet_in = {
		93372036854775807ULL,
	73.0,
	101.0,
	129.0,
	157.0,
	185.0,
	213.0,
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
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vision_position_estimate_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_vision_position_estimate_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vision_position_estimate_pack(system_id, component_id, &msg , packet1.usec , packet1.x , packet1.y , packet1.z , packet1.roll , packet1.pitch , packet1.yaw );
	mavlink_msg_vision_position_estimate_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vision_position_estimate_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.usec , packet1.x , packet1.y , packet1.z , packet1.roll , packet1.pitch , packet1.yaw );
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
	mavlink_msg_vision_position_estimate_send(MAVLINK_COMM_1 , packet1.usec , packet1.x , packet1.y , packet1.z , packet1.roll , packet1.pitch , packet1.yaw );
	mavlink_msg_vision_position_estimate_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_vision_speed_estimate(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_vision_speed_estimate_t packet_in = {
		93372036854775807ULL,
	73.0,
	101.0,
	129.0,
	};
	mavlink_vision_speed_estimate_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.usec = packet_in.usec;
        	packet1.x = packet_in.x;
        	packet1.y = packet_in.y;
        	packet1.z = packet_in.z;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vision_speed_estimate_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_vision_speed_estimate_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vision_speed_estimate_pack(system_id, component_id, &msg , packet1.usec , packet1.x , packet1.y , packet1.z );
	mavlink_msg_vision_speed_estimate_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vision_speed_estimate_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.usec , packet1.x , packet1.y , packet1.z );
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
	mavlink_msg_vision_speed_estimate_send(MAVLINK_COMM_1 , packet1.usec , packet1.x , packet1.y , packet1.z );
	mavlink_msg_vision_speed_estimate_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_vicon_position_estimate(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_vicon_position_estimate_t packet_in = {
		93372036854775807ULL,
	73.0,
	101.0,
	129.0,
	157.0,
	185.0,
	213.0,
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
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vicon_position_estimate_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_vicon_position_estimate_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vicon_position_estimate_pack(system_id, component_id, &msg , packet1.usec , packet1.x , packet1.y , packet1.z , packet1.roll , packet1.pitch , packet1.yaw );
	mavlink_msg_vicon_position_estimate_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vicon_position_estimate_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.usec , packet1.x , packet1.y , packet1.z , packet1.roll , packet1.pitch , packet1.yaw );
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
	mavlink_msg_vicon_position_estimate_send(MAVLINK_COMM_1 , packet1.usec , packet1.x , packet1.y , packet1.z , packet1.roll , packet1.pitch , packet1.yaw );
	mavlink_msg_vicon_position_estimate_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_memory_vect(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_memory_vect_t packet_in = {
		17235,
	139,
	206,
	{ 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48 },
	};
	mavlink_memory_vect_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.address = packet_in.address;
        	packet1.ver = packet_in.ver;
        	packet1.type = packet_in.type;
        
        	mav_array_memcpy(packet1.value, packet_in.value, sizeof(int8_t)*32);
        

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
}

static void mavlink_test_debug_vect(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_debug_vect_t packet_in = {
		93372036854775807ULL,
	73.0,
	101.0,
	129.0,
	"UVWXYZABC",
	};
	mavlink_debug_vect_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_usec = packet_in.time_usec;
        	packet1.x = packet_in.x;
        	packet1.y = packet_in.y;
        	packet1.z = packet_in.z;
        
        	mav_array_memcpy(packet1.name, packet_in.name, sizeof(char)*10);
        

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
}

static void mavlink_test_named_value_float(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_named_value_float_t packet_in = {
		963497464,
	45.0,
	"IJKLMNOPQ",
	};
	mavlink_named_value_float_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        	packet1.value = packet_in.value;
        
        	mav_array_memcpy(packet1.name, packet_in.name, sizeof(char)*10);
        

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
}

static void mavlink_test_named_value_int(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_named_value_int_t packet_in = {
		963497464,
	963497672,
	"IJKLMNOPQ",
	};
	mavlink_named_value_int_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        	packet1.value = packet_in.value;
        
        	mav_array_memcpy(packet1.name, packet_in.name, sizeof(char)*10);
        

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
}

static void mavlink_test_statustext(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_statustext_t packet_in = {
		5,
	"BCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWX",
	};
	mavlink_statustext_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.severity = packet_in.severity;
        
        	mav_array_memcpy(packet1.text, packet_in.text, sizeof(char)*50);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_statustext_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_statustext_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_statustext_pack(system_id, component_id, &msg , packet1.severity , packet1.text );
	mavlink_msg_statustext_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_statustext_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.severity , packet1.text );
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
	mavlink_msg_statustext_send(MAVLINK_COMM_1 , packet1.severity , packet1.text );
	mavlink_msg_statustext_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_debug(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_debug_t packet_in = {
		963497464,
	45.0,
	29,
	};
	mavlink_debug_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        	packet1.value = packet_in.value;
        	packet1.ind = packet_in.ind;
        
        

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
}

static void mavlink_test_common(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_heartbeat(system_id, component_id, last_msg);
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
	mavlink_test_set_local_position_setpoint(system_id, component_id, last_msg);
	mavlink_test_local_position_setpoint(system_id, component_id, last_msg);
	mavlink_test_global_position_setpoint_int(system_id, component_id, last_msg);
	mavlink_test_set_global_position_setpoint_int(system_id, component_id, last_msg);
	mavlink_test_safety_set_allowed_area(system_id, component_id, last_msg);
	mavlink_test_safety_allowed_area(system_id, component_id, last_msg);
	mavlink_test_set_roll_pitch_yaw_thrust(system_id, component_id, last_msg);
	mavlink_test_set_roll_pitch_yaw_speed_thrust(system_id, component_id, last_msg);
	mavlink_test_roll_pitch_yaw_thrust_setpoint(system_id, component_id, last_msg);
	mavlink_test_roll_pitch_yaw_speed_thrust_setpoint(system_id, component_id, last_msg);
	mavlink_test_set_quad_motors_setpoint(system_id, component_id, last_msg);
	mavlink_test_set_quad_swarm_roll_pitch_yaw_thrust(system_id, component_id, last_msg);
	mavlink_test_nav_controller_output(system_id, component_id, last_msg);
	mavlink_test_state_correction(system_id, component_id, last_msg);
	mavlink_test_request_data_stream(system_id, component_id, last_msg);
	mavlink_test_data_stream(system_id, component_id, last_msg);
	mavlink_test_manual_control(system_id, component_id, last_msg);
	mavlink_test_rc_channels_override(system_id, component_id, last_msg);
	mavlink_test_vfr_hud(system_id, component_id, last_msg);
	mavlink_test_command_long(system_id, component_id, last_msg);
	mavlink_test_command_ack(system_id, component_id, last_msg);
	mavlink_test_local_position_ned_system_global_offset(system_id, component_id, last_msg);
	mavlink_test_hil_state(system_id, component_id, last_msg);
	mavlink_test_hil_controls(system_id, component_id, last_msg);
	mavlink_test_hil_rc_inputs_raw(system_id, component_id, last_msg);
	mavlink_test_optical_flow(system_id, component_id, last_msg);
	mavlink_test_global_vision_position_estimate(system_id, component_id, last_msg);
	mavlink_test_vision_position_estimate(system_id, component_id, last_msg);
	mavlink_test_vision_speed_estimate(system_id, component_id, last_msg);
	mavlink_test_vicon_position_estimate(system_id, component_id, last_msg);
	mavlink_test_memory_vect(system_id, component_id, last_msg);
	mavlink_test_debug_vect(system_id, component_id, last_msg);
	mavlink_test_named_value_float(system_id, component_id, last_msg);
	mavlink_test_named_value_int(system_id, component_id, last_msg);
	mavlink_test_statustext(system_id, component_id, last_msg);
	mavlink_test_debug(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // COMMON_TESTSUITE_H
