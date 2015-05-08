// MESSAGE SAFETY_STATUS PACKING

#define MAVLINK_MSG_ID_SAFETY_STATUS 226

typedef struct __mavlink_safety_status_t
{
 uint64_t prearm; ///< Prearm
} mavlink_safety_status_t;

#define MAVLINK_MSG_ID_SAFETY_STATUS_LEN 8
#define MAVLINK_MSG_ID_226_LEN 8

#define MAVLINK_MSG_ID_SAFETY_STATUS_CRC 113
#define MAVLINK_MSG_ID_226_CRC 113



#define MAVLINK_MESSAGE_INFO_SAFETY_STATUS { \
	"SAFETY_STATUS", \
	1, \
	{  { "prearm", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_safety_status_t, prearm) }, \
         } \
}


/**
 * @brief Pack a safety_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param prearm Prearm
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_safety_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t prearm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SAFETY_STATUS_LEN];
	_mav_put_uint64_t(buf, 0, prearm);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SAFETY_STATUS_LEN);
#else
	mavlink_safety_status_t packet;
	packet.prearm = prearm;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SAFETY_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SAFETY_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SAFETY_STATUS_LEN, MAVLINK_MSG_ID_SAFETY_STATUS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SAFETY_STATUS_LEN);
#endif
}

/**
 * @brief Pack a safety_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param prearm Prearm
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_safety_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t prearm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SAFETY_STATUS_LEN];
	_mav_put_uint64_t(buf, 0, prearm);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SAFETY_STATUS_LEN);
#else
	mavlink_safety_status_t packet;
	packet.prearm = prearm;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SAFETY_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SAFETY_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SAFETY_STATUS_LEN, MAVLINK_MSG_ID_SAFETY_STATUS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SAFETY_STATUS_LEN);
#endif
}

/**
 * @brief Encode a safety_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param safety_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_safety_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_safety_status_t* safety_status)
{
	return mavlink_msg_safety_status_pack(system_id, component_id, msg, safety_status->prearm);
}

/**
 * @brief Encode a safety_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param safety_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_safety_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_safety_status_t* safety_status)
{
	return mavlink_msg_safety_status_pack_chan(system_id, component_id, chan, msg, safety_status->prearm);
}

/**
 * @brief Send a safety_status message
 * @param chan MAVLink channel to send the message
 *
 * @param prearm Prearm
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_safety_status_send(mavlink_channel_t chan, uint64_t prearm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SAFETY_STATUS_LEN];
	_mav_put_uint64_t(buf, 0, prearm);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SAFETY_STATUS, buf, MAVLINK_MSG_ID_SAFETY_STATUS_LEN, MAVLINK_MSG_ID_SAFETY_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SAFETY_STATUS, buf, MAVLINK_MSG_ID_SAFETY_STATUS_LEN);
#endif
#else
	mavlink_safety_status_t packet;
	packet.prearm = prearm;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SAFETY_STATUS, (const char *)&packet, MAVLINK_MSG_ID_SAFETY_STATUS_LEN, MAVLINK_MSG_ID_SAFETY_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SAFETY_STATUS, (const char *)&packet, MAVLINK_MSG_ID_SAFETY_STATUS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_SAFETY_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_safety_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t prearm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, prearm);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SAFETY_STATUS, buf, MAVLINK_MSG_ID_SAFETY_STATUS_LEN, MAVLINK_MSG_ID_SAFETY_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SAFETY_STATUS, buf, MAVLINK_MSG_ID_SAFETY_STATUS_LEN);
#endif
#else
	mavlink_safety_status_t *packet = (mavlink_safety_status_t *)msgbuf;
	packet->prearm = prearm;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SAFETY_STATUS, (const char *)packet, MAVLINK_MSG_ID_SAFETY_STATUS_LEN, MAVLINK_MSG_ID_SAFETY_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SAFETY_STATUS, (const char *)packet, MAVLINK_MSG_ID_SAFETY_STATUS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE SAFETY_STATUS UNPACKING


/**
 * @brief Get field prearm from safety_status message
 *
 * @return Prearm
 */
static inline uint64_t mavlink_msg_safety_status_get_prearm(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Decode a safety_status message into a struct
 *
 * @param msg The message to decode
 * @param safety_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_safety_status_decode(const mavlink_message_t* msg, mavlink_safety_status_t* safety_status)
{
#if MAVLINK_NEED_BYTE_SWAP
	safety_status->prearm = mavlink_msg_safety_status_get_prearm(msg);
#else
	memcpy(safety_status, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SAFETY_STATUS_LEN);
#endif
}
