// MESSAGE FENCE_FETCH_POINT PACKING

#define MAVLINK_MSG_ID_FENCE_FETCH_POINT 161

typedef struct __mavlink_fence_fetch_point_t
{
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
 uint8_t idx; ///< point index (first point is 1, 0 is for return point)
} mavlink_fence_fetch_point_t;

#define MAVLINK_MSG_ID_FENCE_FETCH_POINT_LEN 3
#define MAVLINK_MSG_ID_161_LEN 3



#define MAVLINK_MESSAGE_INFO_FENCE_FETCH_POINT { \
	"FENCE_FETCH_POINT", \
	3, \
	{  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_fence_fetch_point_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_fence_fetch_point_t, target_component) }, \
         { "idx", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_fence_fetch_point_t, idx) }, \
         } \
}


/**
 * @brief Pack a fence_fetch_point message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param idx point index (first point is 1, 0 is for return point)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fence_fetch_point_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, uint8_t idx)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[3];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, idx);

        memcpy(_MAV_PAYLOAD(msg), buf, 3);
#else
	mavlink_fence_fetch_point_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.idx = idx;

        memcpy(_MAV_PAYLOAD(msg), &packet, 3);
#endif

	msg->msgid = MAVLINK_MSG_ID_FENCE_FETCH_POINT;
	return mavlink_finalize_message(msg, system_id, component_id, 3, 68);
}

/**
 * @brief Pack a fence_fetch_point message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param idx point index (first point is 1, 0 is for return point)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fence_fetch_point_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,uint8_t idx)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[3];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, idx);

        memcpy(_MAV_PAYLOAD(msg), buf, 3);
#else
	mavlink_fence_fetch_point_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.idx = idx;

        memcpy(_MAV_PAYLOAD(msg), &packet, 3);
#endif

	msg->msgid = MAVLINK_MSG_ID_FENCE_FETCH_POINT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 3, 68);
}

/**
 * @brief Encode a fence_fetch_point struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param fence_fetch_point C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_fence_fetch_point_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_fence_fetch_point_t* fence_fetch_point)
{
	return mavlink_msg_fence_fetch_point_pack(system_id, component_id, msg, fence_fetch_point->target_system, fence_fetch_point->target_component, fence_fetch_point->idx);
}

/**
 * @brief Send a fence_fetch_point message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param idx point index (first point is 1, 0 is for return point)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_fence_fetch_point_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t idx)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[3];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, idx);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FENCE_FETCH_POINT, buf, 3, 68);
#else
	mavlink_fence_fetch_point_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.idx = idx;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FENCE_FETCH_POINT, (const char *)&packet, 3, 68);
#endif
}

#endif

// MESSAGE FENCE_FETCH_POINT UNPACKING


/**
 * @brief Get field target_system from fence_fetch_point message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_fence_fetch_point_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field target_component from fence_fetch_point message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_fence_fetch_point_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field idx from fence_fetch_point message
 *
 * @return point index (first point is 1, 0 is for return point)
 */
static inline uint8_t mavlink_msg_fence_fetch_point_get_idx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a fence_fetch_point message into a struct
 *
 * @param msg The message to decode
 * @param fence_fetch_point C-struct to decode the message contents into
 */
static inline void mavlink_msg_fence_fetch_point_decode(const mavlink_message_t* msg, mavlink_fence_fetch_point_t* fence_fetch_point)
{
#if MAVLINK_NEED_BYTE_SWAP
	fence_fetch_point->target_system = mavlink_msg_fence_fetch_point_get_target_system(msg);
	fence_fetch_point->target_component = mavlink_msg_fence_fetch_point_get_target_component(msg);
	fence_fetch_point->idx = mavlink_msg_fence_fetch_point_get_idx(msg);
#else
	memcpy(fence_fetch_point, _MAV_PAYLOAD(msg), 3);
#endif
}
