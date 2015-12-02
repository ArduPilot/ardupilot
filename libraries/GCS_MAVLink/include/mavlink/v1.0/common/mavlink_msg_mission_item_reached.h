// MESSAGE MISSION_ITEM_REACHED PACKING

#define MAVLINK_MSG_ID_MISSION_ITEM_REACHED 46

typedef struct __mavlink_mission_item_reached_t
{
 uint16_t seq; /*< Sequence*/
} mavlink_mission_item_reached_t;

#define MAVLINK_MSG_ID_MISSION_ITEM_REACHED_LEN 2
#define MAVLINK_MSG_ID_46_LEN 2

#define MAVLINK_MSG_ID_MISSION_ITEM_REACHED_CRC 11
#define MAVLINK_MSG_ID_46_CRC 11



#define MAVLINK_MESSAGE_INFO_MISSION_ITEM_REACHED { \
	"MISSION_ITEM_REACHED", \
	1, \
	{  { "seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_mission_item_reached_t, seq) }, \
         } \
}


/**
 * @brief Pack a mission_item_reached message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param seq Sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_item_reached_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MISSION_ITEM_REACHED_LEN];
	_mav_put_uint16_t(buf, 0, seq);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_ITEM_REACHED_LEN);
#else
	mavlink_mission_item_reached_t packet;
	packet.seq = seq;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_ITEM_REACHED_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MISSION_ITEM_REACHED;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MISSION_ITEM_REACHED_LEN, MAVLINK_MSG_ID_MISSION_ITEM_REACHED_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MISSION_ITEM_REACHED_LEN);
#endif
}

/**
 * @brief Pack a mission_item_reached message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param seq Sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_item_reached_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MISSION_ITEM_REACHED_LEN];
	_mav_put_uint16_t(buf, 0, seq);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_ITEM_REACHED_LEN);
#else
	mavlink_mission_item_reached_t packet;
	packet.seq = seq;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_ITEM_REACHED_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MISSION_ITEM_REACHED;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MISSION_ITEM_REACHED_LEN, MAVLINK_MSG_ID_MISSION_ITEM_REACHED_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MISSION_ITEM_REACHED_LEN);
#endif
}

/**
 * @brief Encode a mission_item_reached struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_item_reached C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_item_reached_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mission_item_reached_t* mission_item_reached)
{
	return mavlink_msg_mission_item_reached_pack(system_id, component_id, msg, mission_item_reached->seq);
}

/**
 * @brief Encode a mission_item_reached struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_item_reached C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_item_reached_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mission_item_reached_t* mission_item_reached)
{
	return mavlink_msg_mission_item_reached_pack_chan(system_id, component_id, chan, msg, mission_item_reached->seq);
}

/**
 * @brief Send a mission_item_reached message
 * @param chan MAVLink channel to send the message
 *
 * @param seq Sequence
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mission_item_reached_send(mavlink_channel_t chan, uint16_t seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MISSION_ITEM_REACHED_LEN];
	_mav_put_uint16_t(buf, 0, seq);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_ITEM_REACHED, buf, MAVLINK_MSG_ID_MISSION_ITEM_REACHED_LEN, MAVLINK_MSG_ID_MISSION_ITEM_REACHED_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_ITEM_REACHED, buf, MAVLINK_MSG_ID_MISSION_ITEM_REACHED_LEN);
#endif
#else
	mavlink_mission_item_reached_t packet;
	packet.seq = seq;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_ITEM_REACHED, (const char *)&packet, MAVLINK_MSG_ID_MISSION_ITEM_REACHED_LEN, MAVLINK_MSG_ID_MISSION_ITEM_REACHED_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_ITEM_REACHED, (const char *)&packet, MAVLINK_MSG_ID_MISSION_ITEM_REACHED_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_MISSION_ITEM_REACHED_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mission_item_reached_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint16_t(buf, 0, seq);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_ITEM_REACHED, buf, MAVLINK_MSG_ID_MISSION_ITEM_REACHED_LEN, MAVLINK_MSG_ID_MISSION_ITEM_REACHED_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_ITEM_REACHED, buf, MAVLINK_MSG_ID_MISSION_ITEM_REACHED_LEN);
#endif
#else
	mavlink_mission_item_reached_t *packet = (mavlink_mission_item_reached_t *)msgbuf;
	packet->seq = seq;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_ITEM_REACHED, (const char *)packet, MAVLINK_MSG_ID_MISSION_ITEM_REACHED_LEN, MAVLINK_MSG_ID_MISSION_ITEM_REACHED_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_ITEM_REACHED, (const char *)packet, MAVLINK_MSG_ID_MISSION_ITEM_REACHED_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE MISSION_ITEM_REACHED UNPACKING


/**
 * @brief Get field seq from mission_item_reached message
 *
 * @return Sequence
 */
static inline uint16_t mavlink_msg_mission_item_reached_get_seq(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a mission_item_reached message into a struct
 *
 * @param msg The message to decode
 * @param mission_item_reached C-struct to decode the message contents into
 */
static inline void mavlink_msg_mission_item_reached_decode(const mavlink_message_t* msg, mavlink_mission_item_reached_t* mission_item_reached)
{
#if MAVLINK_NEED_BYTE_SWAP
	mission_item_reached->seq = mavlink_msg_mission_item_reached_get_seq(msg);
#else
	memcpy(mission_item_reached, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_MISSION_ITEM_REACHED_LEN);
#endif
}
