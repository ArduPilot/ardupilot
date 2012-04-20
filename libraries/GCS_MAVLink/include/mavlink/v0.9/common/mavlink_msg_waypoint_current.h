// MESSAGE WAYPOINT_CURRENT PACKING

#define MAVLINK_MSG_ID_WAYPOINT_CURRENT 42

typedef struct __mavlink_waypoint_current_t
{
 uint16_t seq; ///< Sequence
} mavlink_waypoint_current_t;

#define MAVLINK_MSG_ID_WAYPOINT_CURRENT_LEN 2
#define MAVLINK_MSG_ID_42_LEN 2



#define MAVLINK_MESSAGE_INFO_WAYPOINT_CURRENT { \
	"WAYPOINT_CURRENT", \
	1, \
	{  { "seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_waypoint_current_t, seq) }, \
         } \
}


/**
 * @brief Pack a waypoint_current message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param seq Sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_waypoint_current_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[2];
	_mav_put_uint16_t(buf, 0, seq);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 2);
#else
	mavlink_waypoint_current_t packet;
	packet.seq = seq;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 2);
#endif

	msg->msgid = MAVLINK_MSG_ID_WAYPOINT_CURRENT;
	return mavlink_finalize_message(msg, system_id, component_id, 2);
}

/**
 * @brief Pack a waypoint_current message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param seq Sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_waypoint_current_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[2];
	_mav_put_uint16_t(buf, 0, seq);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 2);
#else
	mavlink_waypoint_current_t packet;
	packet.seq = seq;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 2);
#endif

	msg->msgid = MAVLINK_MSG_ID_WAYPOINT_CURRENT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 2);
}

/**
 * @brief Encode a waypoint_current struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param waypoint_current C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_waypoint_current_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_waypoint_current_t* waypoint_current)
{
	return mavlink_msg_waypoint_current_pack(system_id, component_id, msg, waypoint_current->seq);
}

/**
 * @brief Send a waypoint_current message
 * @param chan MAVLink channel to send the message
 *
 * @param seq Sequence
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_waypoint_current_send(mavlink_channel_t chan, uint16_t seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[2];
	_mav_put_uint16_t(buf, 0, seq);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WAYPOINT_CURRENT, buf, 2);
#else
	mavlink_waypoint_current_t packet;
	packet.seq = seq;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WAYPOINT_CURRENT, (const char *)&packet, 2);
#endif
}

#endif

// MESSAGE WAYPOINT_CURRENT UNPACKING


/**
 * @brief Get field seq from waypoint_current message
 *
 * @return Sequence
 */
static inline uint16_t mavlink_msg_waypoint_current_get_seq(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a waypoint_current message into a struct
 *
 * @param msg The message to decode
 * @param waypoint_current C-struct to decode the message contents into
 */
static inline void mavlink_msg_waypoint_current_decode(const mavlink_message_t* msg, mavlink_waypoint_current_t* waypoint_current)
{
#if MAVLINK_NEED_BYTE_SWAP
	waypoint_current->seq = mavlink_msg_waypoint_current_get_seq(msg);
#else
	memcpy(waypoint_current, _MAV_PAYLOAD(msg), 2);
#endif
}
