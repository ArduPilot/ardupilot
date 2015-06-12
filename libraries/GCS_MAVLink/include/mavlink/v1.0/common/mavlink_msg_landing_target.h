// MESSAGE LANDING_TARGET PACKING

#define MAVLINK_MSG_ID_LANDING_TARGET 149

typedef struct __mavlink_landing_target_t
{
 float angle_x; ///< X-axis angular offset (in radians) of the target from the center of the image
 float angle_y; ///< Y-axis angular offset (in radians) of the target from the center of the image
 float distance; ///< Distance to the target from the vehicle in meters
 uint8_t target_num; ///< The ID of the target if multiple targets are present
 uint8_t frame; ///< MAV_FRAME enum specifying the whether the following feilds are earth-frame, body-frame, etc.
} mavlink_landing_target_t;

#define MAVLINK_MSG_ID_LANDING_TARGET_LEN 14
#define MAVLINK_MSG_ID_149_LEN 14

#define MAVLINK_MSG_ID_LANDING_TARGET_CRC 255
#define MAVLINK_MSG_ID_149_CRC 255



#define MAVLINK_MESSAGE_INFO_LANDING_TARGET { \
	"LANDING_TARGET", \
	5, \
	{  { "angle_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_landing_target_t, angle_x) }, \
         { "angle_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_landing_target_t, angle_y) }, \
         { "distance", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_landing_target_t, distance) }, \
         { "target_num", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_landing_target_t, target_num) }, \
         { "frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_landing_target_t, frame) }, \
         } \
}


/**
 * @brief Pack a landing_target message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_num The ID of the target if multiple targets are present
 * @param frame MAV_FRAME enum specifying the whether the following feilds are earth-frame, body-frame, etc.
 * @param angle_x X-axis angular offset (in radians) of the target from the center of the image
 * @param angle_y Y-axis angular offset (in radians) of the target from the center of the image
 * @param distance Distance to the target from the vehicle in meters
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_landing_target_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_num, uint8_t frame, float angle_x, float angle_y, float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_LANDING_TARGET_LEN];
	_mav_put_float(buf, 0, angle_x);
	_mav_put_float(buf, 4, angle_y);
	_mav_put_float(buf, 8, distance);
	_mav_put_uint8_t(buf, 12, target_num);
	_mav_put_uint8_t(buf, 13, frame);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LANDING_TARGET_LEN);
#else
	mavlink_landing_target_t packet;
	packet.angle_x = angle_x;
	packet.angle_y = angle_y;
	packet.distance = distance;
	packet.target_num = target_num;
	packet.frame = frame;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LANDING_TARGET_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_LANDING_TARGET;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LANDING_TARGET_LEN, MAVLINK_MSG_ID_LANDING_TARGET_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LANDING_TARGET_LEN);
#endif
}

/**
 * @brief Pack a landing_target message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_num The ID of the target if multiple targets are present
 * @param frame MAV_FRAME enum specifying the whether the following feilds are earth-frame, body-frame, etc.
 * @param angle_x X-axis angular offset (in radians) of the target from the center of the image
 * @param angle_y Y-axis angular offset (in radians) of the target from the center of the image
 * @param distance Distance to the target from the vehicle in meters
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_landing_target_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_num,uint8_t frame,float angle_x,float angle_y,float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_LANDING_TARGET_LEN];
	_mav_put_float(buf, 0, angle_x);
	_mav_put_float(buf, 4, angle_y);
	_mav_put_float(buf, 8, distance);
	_mav_put_uint8_t(buf, 12, target_num);
	_mav_put_uint8_t(buf, 13, frame);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LANDING_TARGET_LEN);
#else
	mavlink_landing_target_t packet;
	packet.angle_x = angle_x;
	packet.angle_y = angle_y;
	packet.distance = distance;
	packet.target_num = target_num;
	packet.frame = frame;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LANDING_TARGET_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_LANDING_TARGET;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LANDING_TARGET_LEN, MAVLINK_MSG_ID_LANDING_TARGET_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LANDING_TARGET_LEN);
#endif
}

/**
 * @brief Encode a landing_target struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param landing_target C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_landing_target_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_landing_target_t* landing_target)
{
	return mavlink_msg_landing_target_pack(system_id, component_id, msg, landing_target->target_num, landing_target->frame, landing_target->angle_x, landing_target->angle_y, landing_target->distance);
}

/**
 * @brief Encode a landing_target struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param landing_target C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_landing_target_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_landing_target_t* landing_target)
{
	return mavlink_msg_landing_target_pack_chan(system_id, component_id, chan, msg, landing_target->target_num, landing_target->frame, landing_target->angle_x, landing_target->angle_y, landing_target->distance);
}

/**
 * @brief Send a landing_target message
 * @param chan MAVLink channel to send the message
 *
 * @param target_num The ID of the target if multiple targets are present
 * @param frame MAV_FRAME enum specifying the whether the following feilds are earth-frame, body-frame, etc.
 * @param angle_x X-axis angular offset (in radians) of the target from the center of the image
 * @param angle_y Y-axis angular offset (in radians) of the target from the center of the image
 * @param distance Distance to the target from the vehicle in meters
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_landing_target_send(mavlink_channel_t chan, uint8_t target_num, uint8_t frame, float angle_x, float angle_y, float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_LANDING_TARGET_LEN];
	_mav_put_float(buf, 0, angle_x);
	_mav_put_float(buf, 4, angle_y);
	_mav_put_float(buf, 8, distance);
	_mav_put_uint8_t(buf, 12, target_num);
	_mav_put_uint8_t(buf, 13, frame);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LANDING_TARGET, buf, MAVLINK_MSG_ID_LANDING_TARGET_LEN, MAVLINK_MSG_ID_LANDING_TARGET_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LANDING_TARGET, buf, MAVLINK_MSG_ID_LANDING_TARGET_LEN);
#endif
#else
	mavlink_landing_target_t packet;
	packet.angle_x = angle_x;
	packet.angle_y = angle_y;
	packet.distance = distance;
	packet.target_num = target_num;
	packet.frame = frame;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LANDING_TARGET, (const char *)&packet, MAVLINK_MSG_ID_LANDING_TARGET_LEN, MAVLINK_MSG_ID_LANDING_TARGET_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LANDING_TARGET, (const char *)&packet, MAVLINK_MSG_ID_LANDING_TARGET_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_LANDING_TARGET_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_landing_target_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_num, uint8_t frame, float angle_x, float angle_y, float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, angle_x);
	_mav_put_float(buf, 4, angle_y);
	_mav_put_float(buf, 8, distance);
	_mav_put_uint8_t(buf, 12, target_num);
	_mav_put_uint8_t(buf, 13, frame);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LANDING_TARGET, buf, MAVLINK_MSG_ID_LANDING_TARGET_LEN, MAVLINK_MSG_ID_LANDING_TARGET_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LANDING_TARGET, buf, MAVLINK_MSG_ID_LANDING_TARGET_LEN);
#endif
#else
	mavlink_landing_target_t *packet = (mavlink_landing_target_t *)msgbuf;
	packet->angle_x = angle_x;
	packet->angle_y = angle_y;
	packet->distance = distance;
	packet->target_num = target_num;
	packet->frame = frame;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LANDING_TARGET, (const char *)packet, MAVLINK_MSG_ID_LANDING_TARGET_LEN, MAVLINK_MSG_ID_LANDING_TARGET_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LANDING_TARGET, (const char *)packet, MAVLINK_MSG_ID_LANDING_TARGET_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE LANDING_TARGET UNPACKING


/**
 * @brief Get field target_num from landing_target message
 *
 * @return The ID of the target if multiple targets are present
 */
static inline uint8_t mavlink_msg_landing_target_get_target_num(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field frame from landing_target message
 *
 * @return MAV_FRAME enum specifying the whether the following feilds are earth-frame, body-frame, etc.
 */
static inline uint8_t mavlink_msg_landing_target_get_frame(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field angle_x from landing_target message
 *
 * @return X-axis angular offset (in radians) of the target from the center of the image
 */
static inline float mavlink_msg_landing_target_get_angle_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field angle_y from landing_target message
 *
 * @return Y-axis angular offset (in radians) of the target from the center of the image
 */
static inline float mavlink_msg_landing_target_get_angle_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field distance from landing_target message
 *
 * @return Distance to the target from the vehicle in meters
 */
static inline float mavlink_msg_landing_target_get_distance(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a landing_target message into a struct
 *
 * @param msg The message to decode
 * @param landing_target C-struct to decode the message contents into
 */
static inline void mavlink_msg_landing_target_decode(const mavlink_message_t* msg, mavlink_landing_target_t* landing_target)
{
#if MAVLINK_NEED_BYTE_SWAP
	landing_target->angle_x = mavlink_msg_landing_target_get_angle_x(msg);
	landing_target->angle_y = mavlink_msg_landing_target_get_angle_y(msg);
	landing_target->distance = mavlink_msg_landing_target_get_distance(msg);
	landing_target->target_num = mavlink_msg_landing_target_get_target_num(msg);
	landing_target->frame = mavlink_msg_landing_target_get_frame(msg);
#else
	memcpy(landing_target, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_LANDING_TARGET_LEN);
#endif
}
