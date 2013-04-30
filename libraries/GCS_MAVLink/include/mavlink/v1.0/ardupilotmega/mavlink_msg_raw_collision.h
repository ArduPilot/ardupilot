// MESSAGE RAW_COLLISION PACKING

#define MAVLINK_MSG_ID_RAW_COLLISION 175

typedef struct __mavlink_raw_collision_t
{
 int16_t bottom; ///< back distance in cm
 int16_t front; ///< front distance in cm
 int16_t right; ///< right distance in cm
 int16_t left; ///< left distance in cm
 int16_t back; ///< back distance in cm
 int16_t top; ///< top distance in cm
} mavlink_raw_collision_t;

#define MAVLINK_MSG_ID_RAW_COLLISION_LEN 12
#define MAVLINK_MSG_ID_175_LEN 12



#define MAVLINK_MESSAGE_INFO_RAW_COLLISION { \
	"RAW_COLLISION", \
	6, \
	{  { "bottom", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_raw_collision_t, bottom) }, \
         { "front", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_raw_collision_t, front) }, \
         { "right", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_raw_collision_t, right) }, \
         { "left", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_raw_collision_t, left) }, \
         { "back", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_raw_collision_t, back) }, \
         { "top", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_raw_collision_t, top) }, \
         } \
}


/**
 * @brief Pack a raw_collision message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param bottom back distance in cm
 * @param front front distance in cm
 * @param right right distance in cm
 * @param left left distance in cm
 * @param back back distance in cm
 * @param top top distance in cm
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_raw_collision_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int16_t bottom, int16_t front, int16_t right, int16_t left, int16_t back, int16_t top)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_int16_t(buf, 0, bottom);
	_mav_put_int16_t(buf, 2, front);
	_mav_put_int16_t(buf, 4, right);
	_mav_put_int16_t(buf, 6, left);
	_mav_put_int16_t(buf, 8, back);
	_mav_put_int16_t(buf, 10, top);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 12);
#else
	mavlink_raw_collision_t packet;
	packet.bottom = bottom;
	packet.front = front;
	packet.right = right;
	packet.left = left;
	packet.back = back;
	packet.top = top;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 12);
#endif

	msg->msgid = MAVLINK_MSG_ID_RAW_COLLISION;
	return mavlink_finalize_message(msg, system_id, component_id, 12, 158);
}

/**
 * @brief Pack a raw_collision message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param bottom back distance in cm
 * @param front front distance in cm
 * @param right right distance in cm
 * @param left left distance in cm
 * @param back back distance in cm
 * @param top top distance in cm
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_raw_collision_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int16_t bottom,int16_t front,int16_t right,int16_t left,int16_t back,int16_t top)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_int16_t(buf, 0, bottom);
	_mav_put_int16_t(buf, 2, front);
	_mav_put_int16_t(buf, 4, right);
	_mav_put_int16_t(buf, 6, left);
	_mav_put_int16_t(buf, 8, back);
	_mav_put_int16_t(buf, 10, top);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 12);
#else
	mavlink_raw_collision_t packet;
	packet.bottom = bottom;
	packet.front = front;
	packet.right = right;
	packet.left = left;
	packet.back = back;
	packet.top = top;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 12);
#endif

	msg->msgid = MAVLINK_MSG_ID_RAW_COLLISION;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 12, 158);
}

/**
 * @brief Encode a raw_collision struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param raw_collision C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_raw_collision_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_raw_collision_t* raw_collision)
{
	return mavlink_msg_raw_collision_pack(system_id, component_id, msg, raw_collision->bottom, raw_collision->front, raw_collision->right, raw_collision->left, raw_collision->back, raw_collision->top);
}

/**
 * @brief Send a raw_collision message
 * @param chan MAVLink channel to send the message
 *
 * @param bottom back distance in cm
 * @param front front distance in cm
 * @param right right distance in cm
 * @param left left distance in cm
 * @param back back distance in cm
 * @param top top distance in cm
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_raw_collision_send(mavlink_channel_t chan, int16_t bottom, int16_t front, int16_t right, int16_t left, int16_t back, int16_t top)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_int16_t(buf, 0, bottom);
	_mav_put_int16_t(buf, 2, front);
	_mav_put_int16_t(buf, 4, right);
	_mav_put_int16_t(buf, 6, left);
	_mav_put_int16_t(buf, 8, back);
	_mav_put_int16_t(buf, 10, top);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_COLLISION, buf, 12, 158);
#else
	mavlink_raw_collision_t packet;
	packet.bottom = bottom;
	packet.front = front;
	packet.right = right;
	packet.left = left;
	packet.back = back;
	packet.top = top;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_COLLISION, (const char *)&packet, 12, 158);
#endif
}

#endif

// MESSAGE RAW_COLLISION UNPACKING


/**
 * @brief Get field bottom from raw_collision message
 *
 * @return back distance in cm
 */
static inline int16_t mavlink_msg_raw_collision_get_bottom(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field front from raw_collision message
 *
 * @return front distance in cm
 */
static inline int16_t mavlink_msg_raw_collision_get_front(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Get field right from raw_collision message
 *
 * @return right distance in cm
 */
static inline int16_t mavlink_msg_raw_collision_get_right(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field left from raw_collision message
 *
 * @return left distance in cm
 */
static inline int16_t mavlink_msg_raw_collision_get_left(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  6);
}

/**
 * @brief Get field back from raw_collision message
 *
 * @return back distance in cm
 */
static inline int16_t mavlink_msg_raw_collision_get_back(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field top from raw_collision message
 *
 * @return top distance in cm
 */
static inline int16_t mavlink_msg_raw_collision_get_top(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Decode a raw_collision message into a struct
 *
 * @param msg The message to decode
 * @param raw_collision C-struct to decode the message contents into
 */
static inline void mavlink_msg_raw_collision_decode(const mavlink_message_t* msg, mavlink_raw_collision_t* raw_collision)
{
#if MAVLINK_NEED_BYTE_SWAP
	raw_collision->bottom = mavlink_msg_raw_collision_get_bottom(msg);
	raw_collision->front = mavlink_msg_raw_collision_get_front(msg);
	raw_collision->right = mavlink_msg_raw_collision_get_right(msg);
	raw_collision->left = mavlink_msg_raw_collision_get_left(msg);
	raw_collision->back = mavlink_msg_raw_collision_get_back(msg);
	raw_collision->top = mavlink_msg_raw_collision_get_top(msg);
#else
	memcpy(raw_collision, _MAV_PAYLOAD(msg), 12);
#endif
}
