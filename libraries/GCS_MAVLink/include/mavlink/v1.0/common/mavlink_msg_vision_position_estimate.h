// MESSAGE VISION_POSITION_ESTIMATE PACKING

#define MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE 102

typedef struct __mavlink_vision_position_estimate_t
{
 uint64_t usec; ///< Timestamp (microseconds, synced to UNIX time or since system boot)
 float x; ///< Global X position
 float y; ///< Global Y position
 float z; ///< Global Z position
 float roll; ///< Roll angle in rad
 float pitch; ///< Pitch angle in rad
 float yaw; ///< Yaw angle in rad
} mavlink_vision_position_estimate_t;

#define MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN 32
#define MAVLINK_MSG_ID_102_LEN 32

#define MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_CRC 158
#define MAVLINK_MSG_ID_102_CRC 158



#define MAVLINK_MESSAGE_INFO_VISION_POSITION_ESTIMATE { \
	"VISION_POSITION_ESTIMATE", \
	7, \
	{  { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_vision_position_estimate_t, usec) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_vision_position_estimate_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_vision_position_estimate_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_vision_position_estimate_t, z) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_vision_position_estimate_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_vision_position_estimate_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_vision_position_estimate_t, yaw) }, \
         } \
}


/**
 * @brief Pack a vision_position_estimate message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec Timestamp (microseconds, synced to UNIX time or since system boot)
 * @param x Global X position
 * @param y Global Y position
 * @param z Global Z position
 * @param roll Roll angle in rad
 * @param pitch Pitch angle in rad
 * @param yaw Yaw angle in rad
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vision_position_estimate_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t usec, float x, float y, float z, float roll, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_float(buf, 8, x);
	_mav_put_float(buf, 12, y);
	_mav_put_float(buf, 16, z);
	_mav_put_float(buf, 20, roll);
	_mav_put_float(buf, 24, pitch);
	_mav_put_float(buf, 28, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN);
#else
	mavlink_vision_position_estimate_t packet;
	packet.usec = usec;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN);
#endif
}

/**
 * @brief Pack a vision_position_estimate message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec Timestamp (microseconds, synced to UNIX time or since system boot)
 * @param x Global X position
 * @param y Global Y position
 * @param z Global Z position
 * @param roll Roll angle in rad
 * @param pitch Pitch angle in rad
 * @param yaw Yaw angle in rad
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vision_position_estimate_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t usec,float x,float y,float z,float roll,float pitch,float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_float(buf, 8, x);
	_mav_put_float(buf, 12, y);
	_mav_put_float(buf, 16, z);
	_mav_put_float(buf, 20, roll);
	_mav_put_float(buf, 24, pitch);
	_mav_put_float(buf, 28, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN);
#else
	mavlink_vision_position_estimate_t packet;
	packet.usec = usec;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN);
#endif
}

/**
 * @brief Encode a vision_position_estimate struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vision_position_estimate C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vision_position_estimate_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_vision_position_estimate_t* vision_position_estimate)
{
	return mavlink_msg_vision_position_estimate_pack(system_id, component_id, msg, vision_position_estimate->usec, vision_position_estimate->x, vision_position_estimate->y, vision_position_estimate->z, vision_position_estimate->roll, vision_position_estimate->pitch, vision_position_estimate->yaw);
}

/**
 * @brief Encode a vision_position_estimate struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vision_position_estimate C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vision_position_estimate_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_vision_position_estimate_t* vision_position_estimate)
{
	return mavlink_msg_vision_position_estimate_pack_chan(system_id, component_id, chan, msg, vision_position_estimate->usec, vision_position_estimate->x, vision_position_estimate->y, vision_position_estimate->z, vision_position_estimate->roll, vision_position_estimate->pitch, vision_position_estimate->yaw);
}

/**
 * @brief Send a vision_position_estimate message
 * @param chan MAVLink channel to send the message
 *
 * @param usec Timestamp (microseconds, synced to UNIX time or since system boot)
 * @param x Global X position
 * @param y Global Y position
 * @param z Global Z position
 * @param roll Roll angle in rad
 * @param pitch Pitch angle in rad
 * @param yaw Yaw angle in rad
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_vision_position_estimate_send(mavlink_channel_t chan, uint64_t usec, float x, float y, float z, float roll, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_float(buf, 8, x);
	_mav_put_float(buf, 12, y);
	_mav_put_float(buf, 16, z);
	_mav_put_float(buf, 20, roll);
	_mav_put_float(buf, 24, pitch);
	_mav_put_float(buf, 28, yaw);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE, buf, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE, buf, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN);
#endif
#else
	mavlink_vision_position_estimate_t packet;
	packet.usec = usec;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE, (const char *)&packet, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE, (const char *)&packet, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_vision_position_estimate_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t usec, float x, float y, float z, float roll, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_float(buf, 8, x);
	_mav_put_float(buf, 12, y);
	_mav_put_float(buf, 16, z);
	_mav_put_float(buf, 20, roll);
	_mav_put_float(buf, 24, pitch);
	_mav_put_float(buf, 28, yaw);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE, buf, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE, buf, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN);
#endif
#else
	mavlink_vision_position_estimate_t *packet = (mavlink_vision_position_estimate_t *)msgbuf;
	packet->usec = usec;
	packet->x = x;
	packet->y = y;
	packet->z = z;
	packet->roll = roll;
	packet->pitch = pitch;
	packet->yaw = yaw;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE, (const char *)packet, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE, (const char *)packet, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE VISION_POSITION_ESTIMATE UNPACKING


/**
 * @brief Get field usec from vision_position_estimate message
 *
 * @return Timestamp (microseconds, synced to UNIX time or since system boot)
 */
static inline uint64_t mavlink_msg_vision_position_estimate_get_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field x from vision_position_estimate message
 *
 * @return Global X position
 */
static inline float mavlink_msg_vision_position_estimate_get_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field y from vision_position_estimate message
 *
 * @return Global Y position
 */
static inline float mavlink_msg_vision_position_estimate_get_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field z from vision_position_estimate message
 *
 * @return Global Z position
 */
static inline float mavlink_msg_vision_position_estimate_get_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field roll from vision_position_estimate message
 *
 * @return Roll angle in rad
 */
static inline float mavlink_msg_vision_position_estimate_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field pitch from vision_position_estimate message
 *
 * @return Pitch angle in rad
 */
static inline float mavlink_msg_vision_position_estimate_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field yaw from vision_position_estimate message
 *
 * @return Yaw angle in rad
 */
static inline float mavlink_msg_vision_position_estimate_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a vision_position_estimate message into a struct
 *
 * @param msg The message to decode
 * @param vision_position_estimate C-struct to decode the message contents into
 */
static inline void mavlink_msg_vision_position_estimate_decode(const mavlink_message_t* msg, mavlink_vision_position_estimate_t* vision_position_estimate)
{
#if MAVLINK_NEED_BYTE_SWAP
	vision_position_estimate->usec = mavlink_msg_vision_position_estimate_get_usec(msg);
	vision_position_estimate->x = mavlink_msg_vision_position_estimate_get_x(msg);
	vision_position_estimate->y = mavlink_msg_vision_position_estimate_get_y(msg);
	vision_position_estimate->z = mavlink_msg_vision_position_estimate_get_z(msg);
	vision_position_estimate->roll = mavlink_msg_vision_position_estimate_get_roll(msg);
	vision_position_estimate->pitch = mavlink_msg_vision_position_estimate_get_pitch(msg);
	vision_position_estimate->yaw = mavlink_msg_vision_position_estimate_get_yaw(msg);
#else
	memcpy(vision_position_estimate, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN);
#endif
}
