// MESSAGE WAYPOINT_REACHED PACKING

#define MAVLINK_MSG_ID_WAYPOINT_REACHED 46

typedef struct __mavlink_waypoint_reached_t
{
 uint16_t seq; ///< Sequence
} mavlink_waypoint_reached_t;

#define MAVLINK_MSG_ID_WAYPOINT_REACHED_LEN 2
#define MAVLINK_MSG_ID_46_LEN 2



#define MAVLINK_MESSAGE_INFO_WAYPOINT_REACHED { \
	"WAYPOINT_REACHED", \
	1, \
	{  { "seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_waypoint_reached_t, seq) }, \
         } \
}


/**
 * @brief Pack a waypoint_reached message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param seq Sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_waypoint_reached_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[2];
	_mav_put_uint16_t(buf, 0, seq);

        memcpy(_MAV_PAYLOAD(msg), buf, 2);
#else
	mavlink_waypoint_reached_t packet;
	packet.seq = seq;

        memcpy(_MAV_PAYLOAD(msg), &packet, 2);
#endif

	msg->msgid = MAVLINK_MSG_ID_WAYPOINT_REACHED;
	return mavlink_finalize_message(msg, system_id, component_id, 2, 21);
}

/**
 * @brief Pack a waypoint_reached message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param seq Sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_waypoint_reached_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[2];
	_mav_put_uint16_t(buf, 0, seq);

        memcpy(_MAV_PAYLOAD(msg), buf, 2);
#else
	mavlink_waypoint_reached_t packet;
	packet.seq = seq;

        memcpy(_MAV_PAYLOAD(msg), &packet, 2);
#endif

	msg->msgid = MAVLINK_MSG_ID_WAYPOINT_REACHED;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 2, 21);
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
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[2];
	_mav_put_uint16_t(buf, 0, seq);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WAYPOINT_REACHED, buf, 2, 21);
#else
	mavlink_waypoint_reached_t packet;
	packet.seq = seq;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WAYPOINT_REACHED, (const char *)&packet, 2, 21);
#endif
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
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a waypoint_reached message into a struct
 *
 * @param msg The message to decode
 * @param waypoint_reached C-struct to decode the message contents into
 */
static inline void mavlink_msg_waypoint_reached_decode(const mavlink_message_t* msg, mavlink_waypoint_reached_t* waypoint_reached)
{
#if MAVLINK_NEED_BYTE_SWAP
	waypoint_reached->seq = mavlink_msg_waypoint_reached_get_seq(msg);
#else
	memcpy(waypoint_reached, _MAV_PAYLOAD(msg), 2);
#endif
}
