// MESSAGE WAYPOINT_REACHED PACKING

#define MAVLINK_MSG_ID_WAYPOINT_REACHED 46

typedef struct __mavlink_waypoint_reached_t 
{
	uint16_t seq; ///< Sequence

} mavlink_waypoint_reached_t;



/**
 * @brief Pack a waypoint_reached message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param seq Sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_waypoint_reached_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint16_t seq)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_WAYPOINT_REACHED;

	i += put_uint16_t_by_index(seq, i, msg->payload); // Sequence

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a waypoint_reached message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param seq Sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_waypoint_reached_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint16_t seq)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_WAYPOINT_REACHED;

	i += put_uint16_t_by_index(seq, i, msg->payload); // Sequence

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a waypoint_reached struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param waypoint_reached C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_waypoint_reached_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_waypoint_reached_t* waypoint_reached)
{
	return mavlink_msg_waypoint_reached_pack(system_id, component_id, msg, waypoint_reached->seq);
}

/**
 * @brief Send a waypoint_reached message
 * @param chan MAVLink channel to send the message
 *
 * @param seq Sequence
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_waypoint_reached_send(mavlink_channel_t chan, uint16_t seq)
{
	mavlink_message_t msg;
	mavlink_msg_waypoint_reached_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, seq);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE WAYPOINT_REACHED UNPACKING

/**
 * @brief Get field seq from waypoint_reached message
 *
 * @return Sequence
 */
static inline uint16_t mavlink_msg_waypoint_reached_get_seq(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload)[0];
	r.b[0] = (msg->payload)[1];
	return (uint16_t)r.s;
}

/**
 * @brief Decode a waypoint_reached message into a struct
 *
 * @param msg The message to decode
 * @param waypoint_reached C-struct to decode the message contents into
 */
static inline void mavlink_msg_waypoint_reached_decode(const mavlink_message_t* msg, mavlink_waypoint_reached_t* waypoint_reached)
{
	waypoint_reached->seq = mavlink_msg_waypoint_reached_get_seq(msg);
}
