// MESSAGE FENCED_FETCH_POINT PACKING

#define MAVLINK_MSG_ID_FENCED_FETCH_POINT 161

typedef struct __mavlink_fenced_fetch_point_t
{
 uint8_t idx; ///< point index (first point is 1, 0 is for return point)
} mavlink_fenced_fetch_point_t;

#define MAVLINK_MSG_ID_FENCED_FETCH_POINT_LEN 1
#define MAVLINK_MSG_ID_161_LEN 1



#define MAVLINK_MESSAGE_INFO_FENCED_FETCH_POINT { \
	"FENCED_FETCH_POINT", \
	1, \
	{  { "idx", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_fenced_fetch_point_t, idx) }, \
         } \
}


/**
 * @brief Pack a fenced_fetch_point message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param idx point index (first point is 1, 0 is for return point)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fenced_fetch_point_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t idx)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[1];
	_mav_put_uint8_t(buf, 0, idx);

        memcpy(_MAV_PAYLOAD(msg), buf, 1);
#else
	mavlink_fenced_fetch_point_t packet;
	packet.idx = idx;

        memcpy(_MAV_PAYLOAD(msg), &packet, 1);
#endif

	msg->msgid = MAVLINK_MSG_ID_FENCED_FETCH_POINT;
	return mavlink_finalize_message(msg, system_id, component_id, 1);
}

/**
 * @brief Pack a fenced_fetch_point message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param idx point index (first point is 1, 0 is for return point)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fenced_fetch_point_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t idx)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[1];
	_mav_put_uint8_t(buf, 0, idx);

        memcpy(_MAV_PAYLOAD(msg), buf, 1);
#else
	mavlink_fenced_fetch_point_t packet;
	packet.idx = idx;

        memcpy(_MAV_PAYLOAD(msg), &packet, 1);
#endif

	msg->msgid = MAVLINK_MSG_ID_FENCED_FETCH_POINT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 1);
}

/**
 * @brief Encode a fenced_fetch_point struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param fenced_fetch_point C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_fenced_fetch_point_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_fenced_fetch_point_t* fenced_fetch_point)
{
	return mavlink_msg_fenced_fetch_point_pack(system_id, component_id, msg, fenced_fetch_point->idx);
}

/**
 * @brief Send a fenced_fetch_point message
 * @param chan MAVLink channel to send the message
 *
 * @param idx point index (first point is 1, 0 is for return point)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_fenced_fetch_point_send(mavlink_channel_t chan, uint8_t idx)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[1];
	_mav_put_uint8_t(buf, 0, idx);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FENCED_FETCH_POINT, buf, 1);
#else
	mavlink_fenced_fetch_point_t packet;
	packet.idx = idx;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FENCED_FETCH_POINT, (const char *)&packet, 1);
#endif
}

#endif

// MESSAGE FENCED_FETCH_POINT UNPACKING


/**
 * @brief Get field idx from fenced_fetch_point message
 *
 * @return point index (first point is 1, 0 is for return point)
 */
static inline uint8_t mavlink_msg_fenced_fetch_point_get_idx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a fenced_fetch_point message into a struct
 *
 * @param msg The message to decode
 * @param fenced_fetch_point C-struct to decode the message contents into
 */
static inline void mavlink_msg_fenced_fetch_point_decode(const mavlink_message_t* msg, mavlink_fenced_fetch_point_t* fenced_fetch_point)
{
#if MAVLINK_NEED_BYTE_SWAP
	fenced_fetch_point->idx = mavlink_msg_fenced_fetch_point_get_idx(msg);
#else
	memcpy(fenced_fetch_point, _MAV_PAYLOAD(msg), 1);
#endif
}
