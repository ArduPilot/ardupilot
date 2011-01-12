// MESSAGE WAYPOINT_ACK PACKING

#define MAVLINK_MSG_ID_WAYPOINT_ACK 47

typedef struct __mavlink_waypoint_ack_t 
{
	uint8_t target_system; ///< System ID
	uint8_t target_component; ///< Component ID
	uint8_t type; ///< 0: OK, 1: Error

} mavlink_waypoint_ack_t;



/**
 * @brief Pack a waypoint_ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param type 0: OK, 1: Error
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_waypoint_ack_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint8_t type)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_WAYPOINT_ACK;

	i += put_uint8_t_by_index(target_system, i, msg->payload); // System ID
	i += put_uint8_t_by_index(target_component, i, msg->payload); // Component ID
	i += put_uint8_t_by_index(type, i, msg->payload); // 0: OK, 1: Error

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a waypoint_ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param type 0: OK, 1: Error
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_waypoint_ack_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint8_t type)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_WAYPOINT_ACK;

	i += put_uint8_t_by_index(target_system, i, msg->payload); // System ID
	i += put_uint8_t_by_index(target_component, i, msg->payload); // Component ID
	i += put_uint8_t_by_index(type, i, msg->payload); // 0: OK, 1: Error

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a waypoint_ack struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param waypoint_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_waypoint_ack_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_waypoint_ack_t* waypoint_ack)
{
	return mavlink_msg_waypoint_ack_pack(system_id, component_id, msg, waypoint_ack->target_system, waypoint_ack->target_component, waypoint_ack->type);
}

/**
 * @brief Send a waypoint_ack message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param type 0: OK, 1: Error
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_waypoint_ack_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t type)
{
	mavlink_message_t msg;
	mavlink_msg_waypoint_ack_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, target_system, target_component, type);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE WAYPOINT_ACK UNPACKING

/**
 * @brief Get field target_system from waypoint_ack message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_waypoint_ack_get_target_system(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field target_component from waypoint_ack message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_waypoint_ack_get_target_component(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t))[0];
}

/**
 * @brief Get field type from waypoint_ack message
 *
 * @return 0: OK, 1: Error
 */
static inline uint8_t mavlink_msg_waypoint_ack_get_type(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[0];
}

/**
 * @brief Decode a waypoint_ack message into a struct
 *
 * @param msg The message to decode
 * @param waypoint_ack C-struct to decode the message contents into
 */
static inline void mavlink_msg_waypoint_ack_decode(const mavlink_message_t* msg, mavlink_waypoint_ack_t* waypoint_ack)
{
	waypoint_ack->target_system = mavlink_msg_waypoint_ack_get_target_system(msg);
	waypoint_ack->target_component = mavlink_msg_waypoint_ack_get_target_component(msg);
	waypoint_ack->type = mavlink_msg_waypoint_ack_get_type(msg);
}
