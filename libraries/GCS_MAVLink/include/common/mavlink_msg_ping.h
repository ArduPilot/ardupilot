// MESSAGE PING PACKING

#define MAVLINK_MSG_ID_PING 3

typedef struct __mavlink_ping_t 
{
	uint32_t seq; ///< PING sequence
	uint8_t target_system; ///< 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
	uint8_t target_component; ///< 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
	uint64_t time; ///< Unix timestamp in microseconds

} mavlink_ping_t;



/**
 * @brief Pack a ping message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param seq PING sequence
 * @param target_system 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
 * @param target_component 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
 * @param time Unix timestamp in microseconds
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ping_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint32_t seq, uint8_t target_system, uint8_t target_component, uint64_t time)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_PING;

	i += put_uint32_t_by_index(seq, i, msg->payload); // PING sequence
	i += put_uint8_t_by_index(target_system, i, msg->payload); // 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
	i += put_uint8_t_by_index(target_component, i, msg->payload); // 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
	i += put_uint64_t_by_index(time, i, msg->payload); // Unix timestamp in microseconds

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a ping message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param seq PING sequence
 * @param target_system 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
 * @param target_component 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
 * @param time Unix timestamp in microseconds
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ping_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint32_t seq, uint8_t target_system, uint8_t target_component, uint64_t time)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_PING;

	i += put_uint32_t_by_index(seq, i, msg->payload); // PING sequence
	i += put_uint8_t_by_index(target_system, i, msg->payload); // 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
	i += put_uint8_t_by_index(target_component, i, msg->payload); // 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
	i += put_uint64_t_by_index(time, i, msg->payload); // Unix timestamp in microseconds

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a ping struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ping C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ping_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ping_t* ping)
{
	return mavlink_msg_ping_pack(system_id, component_id, msg, ping->seq, ping->target_system, ping->target_component, ping->time);
}

/**
 * @brief Send a ping message
 * @param chan MAVLink channel to send the message
 *
 * @param seq PING sequence
 * @param target_system 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
 * @param target_component 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
 * @param time Unix timestamp in microseconds
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ping_send(mavlink_channel_t chan, uint32_t seq, uint8_t target_system, uint8_t target_component, uint64_t time)
{
	mavlink_message_t msg;
	mavlink_msg_ping_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, seq, target_system, target_component, time);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE PING UNPACKING

/**
 * @brief Get field seq from ping message
 *
 * @return PING sequence
 */
static inline uint32_t mavlink_msg_ping_get_seq(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload)[0];
	r.b[2] = (msg->payload)[1];
	r.b[1] = (msg->payload)[2];
	r.b[0] = (msg->payload)[3];
	return (uint32_t)r.i;
}

/**
 * @brief Get field target_system from ping message
 *
 * @return 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
 */
static inline uint8_t mavlink_msg_ping_get_target_system(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint32_t))[0];
}

/**
 * @brief Get field target_component from ping message
 *
 * @return 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
 */
static inline uint8_t mavlink_msg_ping_get_target_component(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint32_t)+sizeof(uint8_t))[0];
}

/**
 * @brief Get field time from ping message
 *
 * @return Unix timestamp in microseconds
 */
static inline uint64_t mavlink_msg_ping_get_time(const mavlink_message_t* msg)
{
	generic_64bit r;
	r.b[7] = (msg->payload+sizeof(uint32_t)+sizeof(uint8_t)+sizeof(uint8_t))[0];
	r.b[6] = (msg->payload+sizeof(uint32_t)+sizeof(uint8_t)+sizeof(uint8_t))[1];
	r.b[5] = (msg->payload+sizeof(uint32_t)+sizeof(uint8_t)+sizeof(uint8_t))[2];
	r.b[4] = (msg->payload+sizeof(uint32_t)+sizeof(uint8_t)+sizeof(uint8_t))[3];
	r.b[3] = (msg->payload+sizeof(uint32_t)+sizeof(uint8_t)+sizeof(uint8_t))[4];
	r.b[2] = (msg->payload+sizeof(uint32_t)+sizeof(uint8_t)+sizeof(uint8_t))[5];
	r.b[1] = (msg->payload+sizeof(uint32_t)+sizeof(uint8_t)+sizeof(uint8_t))[6];
	r.b[0] = (msg->payload+sizeof(uint32_t)+sizeof(uint8_t)+sizeof(uint8_t))[7];
	return (uint64_t)r.ll;
}

/**
 * @brief Decode a ping message into a struct
 *
 * @param msg The message to decode
 * @param ping C-struct to decode the message contents into
 */
static inline void mavlink_msg_ping_decode(const mavlink_message_t* msg, mavlink_ping_t* ping)
{
	ping->seq = mavlink_msg_ping_get_seq(msg);
	ping->target_system = mavlink_msg_ping_get_target_system(msg);
	ping->target_component = mavlink_msg_ping_get_target_component(msg);
	ping->time = mavlink_msg_ping_get_time(msg);
}
