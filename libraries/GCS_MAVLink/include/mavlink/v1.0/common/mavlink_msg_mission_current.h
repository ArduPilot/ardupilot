// MESSAGE MISSION_CURRENT PACKING

#define MAVLINK_MSG_ID_MISSION_CURRENT 42

typedef struct __mavlink_mission_current_t
{
 uint16_t seq; ///< Sequence
} mavlink_mission_current_t;

#define MAVLINK_MSG_ID_MISSION_CURRENT_LEN 2
#define MAVLINK_MSG_ID_42_LEN 2

#define MAVLINK_MSG_ID_MISSION_CURRENT_CRC 28
#define MAVLINK_MSG_ID_42_CRC 28



#define MAVLINK_MESSAGE_INFO_MISSION_CURRENT { \
	"MISSION_CURRENT", \
	1, \
	{  { "seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_mission_current_t, seq) }, \
         } \
}


/**
 * @brief Pack a mission_current message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param seq Sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_current_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MISSION_CURRENT_LEN];
	_mav_put_uint16_t(buf, 0, seq);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_CURRENT_LEN);
#else
	mavlink_mission_current_t packet;
	packet.seq = seq;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_CURRENT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MISSION_CURRENT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MISSION_CURRENT_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MISSION_CURRENT_LEN);
#endif
}

/**
 * @brief Pack a mission_current message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param seq Sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_current_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MISSION_CURRENT_LEN];
	_mav_put_uint16_t(buf, 0, seq);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_CURRENT_LEN);
#else
	mavlink_mission_current_t packet;
	packet.seq = seq;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_CURRENT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MISSION_CURRENT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MISSION_CURRENT_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MISSION_CURRENT_LEN);
#endif
}

/**
 * @brief Encode a mission_current struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_current C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_current_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mission_current_t* mission_current)
{
	return mavlink_msg_mission_current_pack(system_id, component_id, msg, mission_current->seq);
}

/**
 * @brief Encode a mission_current struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_current C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_current_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mission_current_t* mission_current)
{
	return mavlink_msg_mission_current_pack_chan(system_id, component_id, chan, msg, mission_current->seq);
}

/**
 * @brief Send a mission_current message
 * @param chan MAVLink channel to send the message
 *
 * @param seq Sequence
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mission_current_send(mavlink_channel_t chan, uint16_t seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MISSION_CURRENT_LEN];
	_mav_put_uint16_t(buf, 0, seq);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CURRENT, buf, MAVLINK_MSG_ID_MISSION_CURRENT_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CURRENT, buf, MAVLINK_MSG_ID_MISSION_CURRENT_LEN);
#endif
#else
	mavlink_mission_current_t packet;
	packet.seq = seq;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CURRENT, (const char *)&packet, MAVLINK_MSG_ID_MISSION_CURRENT_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CURRENT, (const char *)&packet, MAVLINK_MSG_ID_MISSION_CURRENT_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_MISSION_CURRENT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mission_current_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint16_t(buf, 0, seq);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CURRENT, buf, MAVLINK_MSG_ID_MISSION_CURRENT_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CURRENT, buf, MAVLINK_MSG_ID_MISSION_CURRENT_LEN);
#endif
#else
	mavlink_mission_current_t *packet = (mavlink_mission_current_t *)msgbuf;
	packet->seq = seq;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CURRENT, (const char *)packet, MAVLINK_MSG_ID_MISSION_CURRENT_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CURRENT, (const char *)packet, MAVLINK_MSG_ID_MISSION_CURRENT_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE MISSION_CURRENT UNPACKING


/**
 * @brief Get field seq from mission_current message
 *
 * @return Sequence
 */
static inline uint16_t mavlink_msg_mission_current_get_seq(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a mission_current message into a struct
 *
 * @param msg The message to decode
 * @param mission_current C-struct to decode the message contents into
 */
static inline void mavlink_msg_mission_current_decode(const mavlink_message_t* msg, mavlink_mission_current_t* mission_current)
{
#if MAVLINK_NEED_BYTE_SWAP
	mission_current->seq = mavlink_msg_mission_current_get_seq(msg);
#else
	memcpy(mission_current, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_MISSION_CURRENT_LEN);
#endif
}
