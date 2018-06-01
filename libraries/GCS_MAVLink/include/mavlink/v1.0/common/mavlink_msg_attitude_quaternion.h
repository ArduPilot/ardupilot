// MESSAGE ATTITUDE_QUATERNION PACKING

#define MAVLINK_MSG_ID_ATTITUDE_QUATERNION 31

typedef struct __mavlink_attitude_quaternion_t
{
 uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot)*/
 float q1; /*< Quaternion component 1, w (1 in null-rotation)*/
 float q2; /*< Quaternion component 2, x (0 in null-rotation)*/
 float q3; /*< Quaternion component 3, y (0 in null-rotation)*/
 float q4; /*< Quaternion component 4, z (0 in null-rotation)*/
 float rollspeed; /*< Roll angular speed (rad/s)*/
 float pitchspeed; /*< Pitch angular speed (rad/s)*/
 float yawspeed; /*< Yaw angular speed (rad/s)*/
} mavlink_attitude_quaternion_t;

#define MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN 32
#define MAVLINK_MSG_ID_31_LEN 32

#define MAVLINK_MSG_ID_ATTITUDE_QUATERNION_CRC 246
#define MAVLINK_MSG_ID_31_CRC 246



#define MAVLINK_MESSAGE_INFO_ATTITUDE_QUATERNION { \
	"ATTITUDE_QUATERNION", \
	8, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_attitude_quaternion_t, time_boot_ms) }, \
         { "q1", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_attitude_quaternion_t, q1) }, \
         { "q2", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_attitude_quaternion_t, q2) }, \
         { "q3", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_attitude_quaternion_t, q3) }, \
         { "q4", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_attitude_quaternion_t, q4) }, \
         { "rollspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_attitude_quaternion_t, rollspeed) }, \
         { "pitchspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_attitude_quaternion_t, pitchspeed) }, \
         { "yawspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_attitude_quaternion_t, yawspeed) }, \
         } \
}


/**
 * @brief Pack a attitude_quaternion message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param q1 Quaternion component 1, w (1 in null-rotation)
 * @param q2 Quaternion component 2, x (0 in null-rotation)
 * @param q3 Quaternion component 3, y (0 in null-rotation)
 * @param q4 Quaternion component 4, z (0 in null-rotation)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude_quaternion_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, float q1, float q2, float q3, float q4, float rollspeed, float pitchspeed, float yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, q1);
	_mav_put_float(buf, 8, q2);
	_mav_put_float(buf, 12, q3);
	_mav_put_float(buf, 16, q4);
	_mav_put_float(buf, 20, rollspeed);
	_mav_put_float(buf, 24, pitchspeed);
	_mav_put_float(buf, 28, yawspeed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN);
#else
	mavlink_attitude_quaternion_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.q1 = q1;
	packet.q2 = q2;
	packet.q3 = q3;
	packet.q4 = q4;
	packet.rollspeed = rollspeed;
	packet.pitchspeed = pitchspeed;
	packet.yawspeed = yawspeed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ATTITUDE_QUATERNION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN);
#endif
}

/**
 * @brief Pack a attitude_quaternion message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param q1 Quaternion component 1, w (1 in null-rotation)
 * @param q2 Quaternion component 2, x (0 in null-rotation)
 * @param q3 Quaternion component 3, y (0 in null-rotation)
 * @param q4 Quaternion component 4, z (0 in null-rotation)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude_quaternion_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,float q1,float q2,float q3,float q4,float rollspeed,float pitchspeed,float yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, q1);
	_mav_put_float(buf, 8, q2);
	_mav_put_float(buf, 12, q3);
	_mav_put_float(buf, 16, q4);
	_mav_put_float(buf, 20, rollspeed);
	_mav_put_float(buf, 24, pitchspeed);
	_mav_put_float(buf, 28, yawspeed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN);
#else
	mavlink_attitude_quaternion_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.q1 = q1;
	packet.q2 = q2;
	packet.q3 = q3;
	packet.q4 = q4;
	packet.rollspeed = rollspeed;
	packet.pitchspeed = pitchspeed;
	packet.yawspeed = yawspeed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ATTITUDE_QUATERNION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN);
#endif
}

/**
 * @brief Encode a attitude_quaternion struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param attitude_quaternion C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_attitude_quaternion_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_attitude_quaternion_t* attitude_quaternion)
{
	return mavlink_msg_attitude_quaternion_pack(system_id, component_id, msg, attitude_quaternion->time_boot_ms, attitude_quaternion->q1, attitude_quaternion->q2, attitude_quaternion->q3, attitude_quaternion->q4, attitude_quaternion->rollspeed, attitude_quaternion->pitchspeed, attitude_quaternion->yawspeed);
}

/**
 * @brief Encode a attitude_quaternion struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param attitude_quaternion C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_attitude_quaternion_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_attitude_quaternion_t* attitude_quaternion)
{
	return mavlink_msg_attitude_quaternion_pack_chan(system_id, component_id, chan, msg, attitude_quaternion->time_boot_ms, attitude_quaternion->q1, attitude_quaternion->q2, attitude_quaternion->q3, attitude_quaternion->q4, attitude_quaternion->rollspeed, attitude_quaternion->pitchspeed, attitude_quaternion->yawspeed);
}

/**
 * @brief Send a attitude_quaternion message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param q1 Quaternion component 1, w (1 in null-rotation)
 * @param q2 Quaternion component 2, x (0 in null-rotation)
 * @param q3 Quaternion component 3, y (0 in null-rotation)
 * @param q4 Quaternion component 4, z (0 in null-rotation)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_attitude_quaternion_send(mavlink_channel_t chan, uint32_t time_boot_ms, float q1, float q2, float q3, float q4, float rollspeed, float pitchspeed, float yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, q1);
	_mav_put_float(buf, 8, q2);
	_mav_put_float(buf, 12, q3);
	_mav_put_float(buf, 16, q4);
	_mav_put_float(buf, 20, rollspeed);
	_mav_put_float(buf, 24, pitchspeed);
	_mav_put_float(buf, 28, yawspeed);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_QUATERNION, buf, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_QUATERNION, buf, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN);
#endif
#else
	mavlink_attitude_quaternion_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.q1 = q1;
	packet.q2 = q2;
	packet.q3 = q3;
	packet.q4 = q4;
	packet.rollspeed = rollspeed;
	packet.pitchspeed = pitchspeed;
	packet.yawspeed = yawspeed;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_QUATERNION, (const char *)&packet, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_QUATERNION, (const char *)&packet, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_attitude_quaternion_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, float q1, float q2, float q3, float q4, float rollspeed, float pitchspeed, float yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, q1);
	_mav_put_float(buf, 8, q2);
	_mav_put_float(buf, 12, q3);
	_mav_put_float(buf, 16, q4);
	_mav_put_float(buf, 20, rollspeed);
	_mav_put_float(buf, 24, pitchspeed);
	_mav_put_float(buf, 28, yawspeed);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_QUATERNION, buf, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_QUATERNION, buf, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN);
#endif
#else
	mavlink_attitude_quaternion_t *packet = (mavlink_attitude_quaternion_t *)msgbuf;
	packet->time_boot_ms = time_boot_ms;
	packet->q1 = q1;
	packet->q2 = q2;
	packet->q3 = q3;
	packet->q4 = q4;
	packet->rollspeed = rollspeed;
	packet->pitchspeed = pitchspeed;
	packet->yawspeed = yawspeed;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_QUATERNION, (const char *)packet, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_QUATERNION, (const char *)packet, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE ATTITUDE_QUATERNION UNPACKING


/**
 * @brief Get field time_boot_ms from attitude_quaternion message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_attitude_quaternion_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field q1 from attitude_quaternion message
 *
 * @return Quaternion component 1, w (1 in null-rotation)
 */
static inline float mavlink_msg_attitude_quaternion_get_q1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field q2 from attitude_quaternion message
 *
 * @return Quaternion component 2, x (0 in null-rotation)
 */
static inline float mavlink_msg_attitude_quaternion_get_q2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field q3 from attitude_quaternion message
 *
 * @return Quaternion component 3, y (0 in null-rotation)
 */
static inline float mavlink_msg_attitude_quaternion_get_q3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field q4 from attitude_quaternion message
 *
 * @return Quaternion component 4, z (0 in null-rotation)
 */
static inline float mavlink_msg_attitude_quaternion_get_q4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field rollspeed from attitude_quaternion message
 *
 * @return Roll angular speed (rad/s)
 */
static inline float mavlink_msg_attitude_quaternion_get_rollspeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field pitchspeed from attitude_quaternion message
 *
 * @return Pitch angular speed (rad/s)
 */
static inline float mavlink_msg_attitude_quaternion_get_pitchspeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field yawspeed from attitude_quaternion message
 *
 * @return Yaw angular speed (rad/s)
 */
static inline float mavlink_msg_attitude_quaternion_get_yawspeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a attitude_quaternion message into a struct
 *
 * @param msg The message to decode
 * @param attitude_quaternion C-struct to decode the message contents into
 */
static inline void mavlink_msg_attitude_quaternion_decode(const mavlink_message_t* msg, mavlink_attitude_quaternion_t* attitude_quaternion)
{
#if MAVLINK_NEED_BYTE_SWAP
	attitude_quaternion->time_boot_ms = mavlink_msg_attitude_quaternion_get_time_boot_ms(msg);
	attitude_quaternion->q1 = mavlink_msg_attitude_quaternion_get_q1(msg);
	attitude_quaternion->q2 = mavlink_msg_attitude_quaternion_get_q2(msg);
	attitude_quaternion->q3 = mavlink_msg_attitude_quaternion_get_q3(msg);
	attitude_quaternion->q4 = mavlink_msg_attitude_quaternion_get_q4(msg);
	attitude_quaternion->rollspeed = mavlink_msg_attitude_quaternion_get_rollspeed(msg);
	attitude_quaternion->pitchspeed = mavlink_msg_attitude_quaternion_get_pitchspeed(msg);
	attitude_quaternion->yawspeed = mavlink_msg_attitude_quaternion_get_yawspeed(msg);
#else
	memcpy(attitude_quaternion, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN);
#endif
}
