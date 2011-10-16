// MESSAGE AP_TIMING PACKING

#define MAVLINK_MSG_ID_AP_TIMING 155

typedef struct __mavlink_ap_timing_t
{
 uint16_t min_50Hz_delta; ///< mininum delta for 50Hz loop
 uint16_t max_50Hz_delta; ///< maximum delta for 50Hz loop
 uint16_t min_200Hz_delta; ///< mininum delta for 200Hz loop
 uint16_t max_200Hz_delta; ///< maximum delta for 200Hz loop
} mavlink_ap_timing_t;

#define MAVLINK_MSG_ID_AP_TIMING_LEN 8
#define MAVLINK_MSG_ID_155_LEN 8



#define MAVLINK_MESSAGE_INFO_AP_TIMING { \
	"AP_TIMING", \
	4, \
	{  { "min_50Hz_delta", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_ap_timing_t, min_50Hz_delta) }, \
         { "max_50Hz_delta", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_ap_timing_t, max_50Hz_delta) }, \
         { "min_200Hz_delta", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_ap_timing_t, min_200Hz_delta) }, \
         { "max_200Hz_delta", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_ap_timing_t, max_200Hz_delta) }, \
         } \
}


/**
 * @brief Pack a ap_timing message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param min_50Hz_delta mininum delta for 50Hz loop
 * @param max_50Hz_delta maximum delta for 50Hz loop
 * @param min_200Hz_delta mininum delta for 200Hz loop
 * @param max_200Hz_delta maximum delta for 200Hz loop
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ap_timing_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t min_50Hz_delta, uint16_t max_50Hz_delta, uint16_t min_200Hz_delta, uint16_t max_200Hz_delta)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[8];
	_mav_put_uint16_t(buf, 0, min_50Hz_delta);
	_mav_put_uint16_t(buf, 2, max_50Hz_delta);
	_mav_put_uint16_t(buf, 4, min_200Hz_delta);
	_mav_put_uint16_t(buf, 6, max_200Hz_delta);

        memcpy(_MAV_PAYLOAD(msg), buf, 8);
#else
	mavlink_ap_timing_t packet;
	packet.min_50Hz_delta = min_50Hz_delta;
	packet.max_50Hz_delta = max_50Hz_delta;
	packet.min_200Hz_delta = min_200Hz_delta;
	packet.max_200Hz_delta = max_200Hz_delta;

        memcpy(_MAV_PAYLOAD(msg), &packet, 8);
#endif

	msg->msgid = MAVLINK_MSG_ID_AP_TIMING;
	return mavlink_finalize_message(msg, system_id, component_id, 8);
}

/**
 * @brief Pack a ap_timing message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param min_50Hz_delta mininum delta for 50Hz loop
 * @param max_50Hz_delta maximum delta for 50Hz loop
 * @param min_200Hz_delta mininum delta for 200Hz loop
 * @param max_200Hz_delta maximum delta for 200Hz loop
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ap_timing_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t min_50Hz_delta,uint16_t max_50Hz_delta,uint16_t min_200Hz_delta,uint16_t max_200Hz_delta)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[8];
	_mav_put_uint16_t(buf, 0, min_50Hz_delta);
	_mav_put_uint16_t(buf, 2, max_50Hz_delta);
	_mav_put_uint16_t(buf, 4, min_200Hz_delta);
	_mav_put_uint16_t(buf, 6, max_200Hz_delta);

        memcpy(_MAV_PAYLOAD(msg), buf, 8);
#else
	mavlink_ap_timing_t packet;
	packet.min_50Hz_delta = min_50Hz_delta;
	packet.max_50Hz_delta = max_50Hz_delta;
	packet.min_200Hz_delta = min_200Hz_delta;
	packet.max_200Hz_delta = max_200Hz_delta;

        memcpy(_MAV_PAYLOAD(msg), &packet, 8);
#endif

	msg->msgid = MAVLINK_MSG_ID_AP_TIMING;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 8);
}

/**
 * @brief Encode a ap_timing struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ap_timing C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ap_timing_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ap_timing_t* ap_timing)
{
	return mavlink_msg_ap_timing_pack(system_id, component_id, msg, ap_timing->min_50Hz_delta, ap_timing->max_50Hz_delta, ap_timing->min_200Hz_delta, ap_timing->max_200Hz_delta);
}

/**
 * @brief Send a ap_timing message
 * @param chan MAVLink channel to send the message
 *
 * @param min_50Hz_delta mininum delta for 50Hz loop
 * @param max_50Hz_delta maximum delta for 50Hz loop
 * @param min_200Hz_delta mininum delta for 200Hz loop
 * @param max_200Hz_delta maximum delta for 200Hz loop
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ap_timing_send(mavlink_channel_t chan, uint16_t min_50Hz_delta, uint16_t max_50Hz_delta, uint16_t min_200Hz_delta, uint16_t max_200Hz_delta)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[8];
	_mav_put_uint16_t(buf, 0, min_50Hz_delta);
	_mav_put_uint16_t(buf, 2, max_50Hz_delta);
	_mav_put_uint16_t(buf, 4, min_200Hz_delta);
	_mav_put_uint16_t(buf, 6, max_200Hz_delta);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AP_TIMING, buf, 8);
#else
	mavlink_ap_timing_t packet;
	packet.min_50Hz_delta = min_50Hz_delta;
	packet.max_50Hz_delta = max_50Hz_delta;
	packet.min_200Hz_delta = min_200Hz_delta;
	packet.max_200Hz_delta = max_200Hz_delta;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AP_TIMING, (const char *)&packet, 8);
#endif
}

#endif

// MESSAGE AP_TIMING UNPACKING


/**
 * @brief Get field min_50Hz_delta from ap_timing message
 *
 * @return mininum delta for 50Hz loop
 */
static inline uint16_t mavlink_msg_ap_timing_get_min_50Hz_delta(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field max_50Hz_delta from ap_timing message
 *
 * @return maximum delta for 50Hz loop
 */
static inline uint16_t mavlink_msg_ap_timing_get_max_50Hz_delta(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field min_200Hz_delta from ap_timing message
 *
 * @return mininum delta for 200Hz loop
 */
static inline uint16_t mavlink_msg_ap_timing_get_min_200Hz_delta(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field max_200Hz_delta from ap_timing message
 *
 * @return maximum delta for 200Hz loop
 */
static inline uint16_t mavlink_msg_ap_timing_get_max_200Hz_delta(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Decode a ap_timing message into a struct
 *
 * @param msg The message to decode
 * @param ap_timing C-struct to decode the message contents into
 */
static inline void mavlink_msg_ap_timing_decode(const mavlink_message_t* msg, mavlink_ap_timing_t* ap_timing)
{
#if MAVLINK_NEED_BYTE_SWAP
	ap_timing->min_50Hz_delta = mavlink_msg_ap_timing_get_min_50Hz_delta(msg);
	ap_timing->max_50Hz_delta = mavlink_msg_ap_timing_get_max_50Hz_delta(msg);
	ap_timing->min_200Hz_delta = mavlink_msg_ap_timing_get_min_200Hz_delta(msg);
	ap_timing->max_200Hz_delta = mavlink_msg_ap_timing_get_max_200Hz_delta(msg);
#else
	memcpy(ap_timing, _MAV_PAYLOAD(msg), 8);
#endif
}
