// MESSAGE FENCE_STATUS PACKING

#define MAVLINK_MSG_ID_FENCE_STATUS 162

typedef struct __mavlink_fence_status_t
{
 uint8_t breach_status; ///< 0 if currently inside fence, 1 if outside
 uint16_t breach_count; ///< number of fence breaches
 uint8_t breach_type; ///< last breach type (see FENCE_BREACH_* enum)
 uint32_t breach_time; ///< time of last breach in milliseconds since boot
} mavlink_fence_status_t;

#define MAVLINK_MSG_ID_FENCE_STATUS_LEN 8
#define MAVLINK_MSG_ID_162_LEN 8



#define MAVLINK_MESSAGE_INFO_FENCE_STATUS { \
	"FENCE_STATUS", \
	4, \
	{  { "breach_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_fence_status_t, breach_status) }, \
         { "breach_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 1, offsetof(mavlink_fence_status_t, breach_count) }, \
         { "breach_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_fence_status_t, breach_type) }, \
         { "breach_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_fence_status_t, breach_time) }, \
         } \
}


/**
 * @brief Pack a fence_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param breach_status 0 if currently inside fence, 1 if outside
 * @param breach_count number of fence breaches
 * @param breach_type last breach type (see FENCE_BREACH_* enum)
 * @param breach_time time of last breach in milliseconds since boot
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fence_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t breach_status, uint16_t breach_count, uint8_t breach_type, uint32_t breach_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[8];
	_mav_put_uint8_t(buf, 0, breach_status);
	_mav_put_uint16_t(buf, 1, breach_count);
	_mav_put_uint8_t(buf, 3, breach_type);
	_mav_put_uint32_t(buf, 4, breach_time);

        memcpy(_MAV_PAYLOAD(msg), buf, 8);
#else
	mavlink_fence_status_t packet;
	packet.breach_status = breach_status;
	packet.breach_count = breach_count;
	packet.breach_type = breach_type;
	packet.breach_time = breach_time;

        memcpy(_MAV_PAYLOAD(msg), &packet, 8);
#endif

	msg->msgid = MAVLINK_MSG_ID_FENCE_STATUS;
	return mavlink_finalize_message(msg, system_id, component_id, 8);
}

/**
 * @brief Pack a fence_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param breach_status 0 if currently inside fence, 1 if outside
 * @param breach_count number of fence breaches
 * @param breach_type last breach type (see FENCE_BREACH_* enum)
 * @param breach_time time of last breach in milliseconds since boot
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fence_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t breach_status,uint16_t breach_count,uint8_t breach_type,uint32_t breach_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[8];
	_mav_put_uint8_t(buf, 0, breach_status);
	_mav_put_uint16_t(buf, 1, breach_count);
	_mav_put_uint8_t(buf, 3, breach_type);
	_mav_put_uint32_t(buf, 4, breach_time);

        memcpy(_MAV_PAYLOAD(msg), buf, 8);
#else
	mavlink_fence_status_t packet;
	packet.breach_status = breach_status;
	packet.breach_count = breach_count;
	packet.breach_type = breach_type;
	packet.breach_time = breach_time;

        memcpy(_MAV_PAYLOAD(msg), &packet, 8);
#endif

	msg->msgid = MAVLINK_MSG_ID_FENCE_STATUS;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 8);
}

/**
 * @brief Encode a fence_status struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param fence_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_fence_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_fence_status_t* fence_status)
{
	return mavlink_msg_fence_status_pack(system_id, component_id, msg, fence_status->breach_status, fence_status->breach_count, fence_status->breach_type, fence_status->breach_time);
}

/**
 * @brief Send a fence_status message
 * @param chan MAVLink channel to send the message
 *
 * @param breach_status 0 if currently inside fence, 1 if outside
 * @param breach_count number of fence breaches
 * @param breach_type last breach type (see FENCE_BREACH_* enum)
 * @param breach_time time of last breach in milliseconds since boot
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_fence_status_send(mavlink_channel_t chan, uint8_t breach_status, uint16_t breach_count, uint8_t breach_type, uint32_t breach_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[8];
	_mav_put_uint8_t(buf, 0, breach_status);
	_mav_put_uint16_t(buf, 1, breach_count);
	_mav_put_uint8_t(buf, 3, breach_type);
	_mav_put_uint32_t(buf, 4, breach_time);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FENCE_STATUS, buf, 8);
#else
	mavlink_fence_status_t packet;
	packet.breach_status = breach_status;
	packet.breach_count = breach_count;
	packet.breach_type = breach_type;
	packet.breach_time = breach_time;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FENCE_STATUS, (const char *)&packet, 8);
#endif
}

#endif

// MESSAGE FENCE_STATUS UNPACKING


/**
 * @brief Get field breach_status from fence_status message
 *
 * @return 0 if currently inside fence, 1 if outside
 */
static inline uint8_t mavlink_msg_fence_status_get_breach_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field breach_count from fence_status message
 *
 * @return number of fence breaches
 */
static inline uint16_t mavlink_msg_fence_status_get_breach_count(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  1);
}

/**
 * @brief Get field breach_type from fence_status message
 *
 * @return last breach type (see FENCE_BREACH_* enum)
 */
static inline uint8_t mavlink_msg_fence_status_get_breach_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field breach_time from fence_status message
 *
 * @return time of last breach in milliseconds since boot
 */
static inline uint32_t mavlink_msg_fence_status_get_breach_time(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Decode a fence_status message into a struct
 *
 * @param msg The message to decode
 * @param fence_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_fence_status_decode(const mavlink_message_t* msg, mavlink_fence_status_t* fence_status)
{
#if MAVLINK_NEED_BYTE_SWAP
	fence_status->breach_status = mavlink_msg_fence_status_get_breach_status(msg);
	fence_status->breach_count = mavlink_msg_fence_status_get_breach_count(msg);
	fence_status->breach_type = mavlink_msg_fence_status_get_breach_type(msg);
	fence_status->breach_time = mavlink_msg_fence_status_get_breach_time(msg);
#else
	memcpy(fence_status, _MAV_PAYLOAD(msg), 8);
#endif
}
