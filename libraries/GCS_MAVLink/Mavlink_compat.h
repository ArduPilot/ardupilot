/*
  compatibility header during transition to MAVLink 1.0
 */

#ifdef MAVLINK10
// in MAVLink 1.0 'waypoint' becomes 'mission'. We can remove these
// mappings once we are not trying to support both protocols

#define MAVLINK_MSG_ID_WAYPOINT_CURRENT MAVLINK_MSG_ID_MISSION_CURRENT
#define MAVLINK_MSG_ID_WAYPOINT_CURRENT_LEN MAVLINK_MSG_ID_MISSION_CURRENT_LEN
#define mavlink_msg_waypoint_current_send mavlink_msg_mission_current_send
#define mavlink_msg_waypoint_current_decode mavlink_msg_mission_current_decode

#define MAVLINK_MSG_ID_WAYPOINT_COUNT MAVLINK_MSG_ID_MISSION_COUNT
#define MAVLINK_MSG_ID_WAYPOINT_COUNT_LEN MAVLINK_MSG_ID_MISSION_COUNT_LEN
#define mavlink_msg_waypoint_count_send mavlink_msg_mission_count_send
#define mavlink_msg_waypoint_count_decode mavlink_msg_mission_count_decode
#define mavlink_waypoint_count_t mavlink_mission_count_t

#define MAVLINK_MSG_ID_WAYPOINT_REQUEST MAVLINK_MSG_ID_MISSION_REQUEST
#define MAVLINK_MSG_ID_WAYPOINT_REQUEST_LEN MAVLINK_MSG_ID_MISSION_REQUEST_LEN
#define mavlink_msg_waypoint_request_send mavlink_msg_mission_request_send
#define mavlink_msg_waypoint_request_decode mavlink_msg_mission_request_decode
#define mavlink_waypoint_request_t mavlink_mission_request_t

#define MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST MAVLINK_MSG_ID_MISSION_REQUEST_LIST
#define MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST_LEN MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN
#define mavlink_msg_waypoint_request_list_send mavlink_msg_mission_request_list_send
#define mavlink_msg_waypoint_request_list_decode mavlink_msg_mission_request_list_decode
#define mavlink_waypoint_request_list_t mavlink_mission_request_list_t

#define MAVLINK_MSG_ID_WAYPOINT MAVLINK_MSG_ID_MISSION_ITEM
#define MAVLINK_MSG_ID_WAYPOINT_LEN MAVLINK_MSG_ID_MISSION_ITEM_LEN
#define mavlink_msg_waypoint_send mavlink_msg_mission_item_send
#define mavlink_msg_waypoint_decode mavlink_msg_mission_item_decode
#define mavlink_waypoint_t mavlink_mission_item_t

#define MAVLINK_MSG_ID_WAYPOINT_ACK MAVLINK_MSG_ID_MISSION_ACK
#define MAVLINK_MSG_ID_WAYPOINT_ACK_LEN MAVLINK_MSG_ID_MISSION_ACK_LEN
#define mavlink_msg_waypoint_ack_send mavlink_msg_mission_ack_send
#define mavlink_msg_waypoint_ack_decode mavlink_msg_mission_ack_decode
#define mavlink_waypoint_ack_t mavlink_mission_ack_t

#define MAVLINK_MSG_ID_WAYPOINT_CLEAR_ALL MAVLINK_MSG_ID_MISSION_CLEAR_ALL
#define MAVLINK_MSG_ID_WAYPOINT_CLEAR_ALL_LEN MAVLINK_MSG_ID_MISSION_CLEAR_ALL_LEN
#define mavlink_msg_waypoint_clear_all_send mavlink_msg_mission_clear_all_send
#define mavlink_msg_waypoint_clear_all_decode mavlink_msg_mission_clear_all_decode
#define mavlink_waypoint_clear_all_t mavlink_mission_clear_all_t

#define MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT MAVLINK_MSG_ID_MISSION_SET_CURRENT
#define MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT_LEN MAVLINK_MSG_ID_MISSION_SET_CURRENT_LEN
#define mavlink_msg_waypoint_set_current_send mavlink_msg_mission_set_current_send
#define mavlink_msg_waypoint_set_current_decode mavlink_msg_mission_set_current_decode
#define mavlink_waypoint_set_current_t mavlink_mission_set_current_t

static uint8_t mav_var_type(enum ap_var_type t)
{
    if (t == AP_PARAM_INT8) {
        return MAV_VAR_INT8;
    }
    if (t == AP_PARAM_INT16) {
        return MAV_VAR_INT16;
    }
    if (t == AP_PARAM_INT32) {
        return MAV_VAR_INT32;
    }
    // treat any others as float
    return MAV_VAR_FLOAT;
}

#define MAV_FIXED_WING MAV_TYPE_FIXED_WING

#else // MAVLINK10

static uint8_t mav_var_type(enum ap_var_type t)
{
	return 0;
}

#define MAV_MISSION_ACCEPTED 0
#define MAV_MISSION_UNSUPPORTED       1
#define MAV_MISSION_UNSUPPORTED_FRAME 1
#define MAV_MISSION_ERROR             1
#define MAV_MISSION_INVALID_SEQUENCE  1

/*
  some functions have some extra params in MAVLink 1.0
 */

static void mavlink_msg_global_position_int_send(mavlink_channel_t chan, uint32_t time_boot_ms, int32_t lat, int32_t lon,
						 int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy,
						 int16_t vz, uint16_t hdg)
{
	mavlink_msg_global_position_int_send(
		chan,
		lat,
		lon,
		alt,
		vx, vy, vz);
}

static void mavlink_msg_rc_channels_scaled_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t port,
						int16_t chan1_scaled, int16_t chan2_scaled, int16_t chan3_scaled,
						int16_t chan4_scaled, int16_t chan5_scaled, int16_t chan6_scaled,
						int16_t chan7_scaled, int16_t chan8_scaled, uint8_t rssi)
{
    mavlink_msg_rc_channels_scaled_send(
        chan,
	chan1_scaled,
	chan2_scaled,
	chan3_scaled,
	chan4_scaled,
	chan5_scaled,
	chan6_scaled,
	chan7_scaled,
	chan8_scaled,
	rssi);
}

static void mavlink_msg_rc_channels_raw_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t port,
					     uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw,
					     uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw,
					     uint16_t chan7_raw, uint16_t chan8_raw, uint8_t rssi)
{
	mavlink_msg_rc_channels_raw_send(
		chan,
		chan1_raw,
		chan2_raw,
		chan3_raw,
		chan4_raw,
		chan5_raw,
		chan6_raw,
		chan7_raw,
		chan8_raw,
		rssi);
}


static void mavlink_msg_servo_output_raw_send(mavlink_channel_t chan, uint32_t time_usec, uint8_t port,
					      uint16_t servo1_raw, uint16_t servo2_raw, uint16_t servo3_raw,
					      uint16_t servo4_raw, uint16_t servo5_raw, uint16_t servo6_raw,
					      uint16_t servo7_raw, uint16_t servo8_raw)
{
        mavlink_msg_servo_output_raw_send(
		chan,
		servo1_raw,
		servo2_raw,
		servo3_raw,
		servo4_raw,
		servo5_raw,
		servo6_raw,
		servo7_raw,
		servo8_raw);
}

static void mavlink_msg_statustext_send(mavlink_channel_t chan, uint8_t severity, const char *text)
{
        mavlink_msg_statustext_send(chan, severity, (const int8_t*) text);
}

static void mavlink_msg_param_value_send(mavlink_channel_t chan, const char *param_id,
					 float param_value, uint8_t param_type,
					 uint16_t param_count, uint16_t param_index)
{
	mavlink_msg_param_value_send(
		chan,
		(int8_t *)param_id,
		param_value,
		param_count,
		param_index);
}
#endif // MAVLINK10
