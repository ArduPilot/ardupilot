// MESSAGE ATTITUDE PACKING

#define MAVLINK_MSG_ID_ATTITUDE 30

typedef struct __mavlink_attitude_t
{
 uint64_t usec; ///< Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 float roll; ///< Roll angle (rad)
 float pitch; ///< Pitch angle (rad)
 float yaw; ///< Yaw angle (rad)
 float rollspeed; ///< Roll angular speed (rad/s)
 float pitchspeed; ///< Pitch angular speed (rad/s)
 float yawspeed; ///< Yaw angular speed (rad/s)
} mavlink_attitude_t;

#define MAVLINK_MSG_ID_ATTITUDE_LEN 32
#define MAVLINK_MSG_ID_30_LEN 32



#define MAVLINK_MESSAGE_INFO_ATTITUDE { \
	"ATTITUDE", \
	7, \
	{  { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_attitude_t, usec) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_attitude_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_attitude_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_attitude_t, yaw) }, \
         { "rollspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_attitude_t, rollspeed) }, \
         { "pitchspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_attitude_t, pitchspeed) }, \
         { "yawspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_attitude_t, yawspeed) }, \
         } \
}


/**
 * @brief Pack a attitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param roll Roll angle (rad)
 * @param pitch Pitch angle (rad)
 * @param yaw Yaw angle (rad)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t usec, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[32];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_float(buf, 8, roll);
	_mav_put_float(buf, 12, pitch);
	_mav_put_float(buf, 16, yaw);
	_mav_put_float(buf, 20, rollspeed);
	_mav_put_float(buf, 24, pitchspeed);
	_mav_put_float(buf, 28, yawspeed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 32);
#else
	mavlink_attitude_t packet;
	packet.usec = usec;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.rollspeed = rollspeed;
	packet.pitchspeed = pitchspeed;
	packet.yawspeed = yawspeed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 32);
#endif

	msg->msgid = MAVLINK_MSG_ID_ATTITUDE;
	return mavlink_finalize_message(msg, system_id, component_id, 32);
}

/**
 * @brief Pack a attitude message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param roll Roll angle (rad)
 * @param pitch Pitch angle (rad)
 * @param yaw Yaw angle (rad)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t usec,float roll,float pitch,float yaw,float rollspeed,float pitchspeed,float yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[32];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_float(buf, 8, roll);
	_mav_put_float(buf, 12, pitch);
	_mav_put_float(buf, 16, yaw);
	_mav_put_float(buf, 20, rollspeed);
	_mav_put_float(buf, 24, pitchspeed);
	_mav_put_float(buf, 28, yawspeed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 32);
#else
	mavlink_attitude_t packet;
	packet.usec = usec;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.rollspeed = rollspeed;
	packet.pitchspeed = pitchspeed;
	packet.yawspeed = yawspeed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 32);
#endif

	msg->msgid = MAVLINK_MSG_ID_ATTITUDE;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 32);
}

/**
 * @brief Encode a attitude struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_attitude_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_attitude_t* attitude)
{
	return mavlink_msg_attitude_pack(system_id, component_id, msg, attitude->usec, attitude->roll, attitude->pitch, attitude->yaw, attitude->rollspeed, attitude->pitchspeed, attitude->yawspeed);
}

/**
 * @brief Send a attitude message
 * @param chan MAVLink channel to send the message
 *
 * @param usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param roll Roll angle (rad)
 * @param pitch Pitch angle (rad)
 * @param yaw Yaw angle (rad)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_attitude_send(mavlink_channel_t chan, uint64_t usec, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[32];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_float(buf, 8, roll);
	_mav_put_float(buf, 12, pitch);
	_mav_put_float(buf, 16, yaw);
	_mav_put_float(buf, 20, rollspeed);
	_mav_put_float(buf, 24, pitchspeed);
	_mav_put_float(buf, 28, yawspeed);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE, buf, 32);
#else
	mavlink_attitude_t packet;
	packet.usec = usec;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.rollspeed = rollspeed;
	packet.pitchspeed = pitchspeed;
	packet.yawspeed = yawspeed;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE, (const char *)&packet, 32);
#endif
}

#endif

// MESSAGE ATTITUDE UNPACKING


/**
 * @brief Get field usec from attitude message
 *
 * @return Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 */
static inline uint64_t mavlink_msg_attitude_get_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field roll from attitude message
 *
 * @return Roll angle (rad)
 */
static inline float mavlink_msg_attitude_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field pitch from attitude message
 *
 * @return Pitch angle (rad)
 */
static inline float mavlink_msg_attitude_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field yaw from attitude message
 *
 * @return Yaw angle (rad)
 */
static inline float mavlink_msg_attitude_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field rollspeed from attitude message
 *
 * @return Roll angular speed (rad/s)
 */
static inline float mavlink_msg_attitude_get_rollspeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field pitchspeed from attitude message
 *
 * @return Pitch angular speed (rad/s)
 */
static inline float mavlink_msg_attitude_get_pitchspeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field yawspeed from attitude message
 *
 * @return Yaw angular speed (rad/s)
 */
static inline float mavlink_msg_attitude_get_yawspeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a attitude message into a struct
 *
 * @param msg The message to decode
 * @param attitude C-struct to decode the message contents into
 */
static inline void mavlink_msg_attitude_decode(const mavlink_message_t* msg, mavlink_attitude_t* attitude)
{
#if MAVLINK_NEED_BYTE_SWAP
	attitude->usec = mavlink_msg_attitude_get_usec(msg);
	attitude->roll = mavlink_msg_attitude_get_roll(msg);
	attitude->pitch = mavlink_msg_attitude_get_pitch(msg);
	attitude->yaw = mavlink_msg_attitude_get_yaw(msg);
	attitude->rollspeed = mavlink_msg_attitude_get_rollspeed(msg);
	attitude->pitchspeed = mavlink_msg_attitude_get_pitchspeed(msg);
	attitude->yawspeed = mavlink_msg_attitude_get_yawspeed(msg);
#else
	memcpy(attitude, _MAV_PAYLOAD(msg), 32);
#endif
}
