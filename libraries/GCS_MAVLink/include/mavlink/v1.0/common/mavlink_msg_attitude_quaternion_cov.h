// MESSAGE ATTITUDE_QUATERNION_COV PACKING

#define MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV 61

typedef struct __mavlink_attitude_quaternion_cov_t
{
 uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot)*/
 float q[4]; /*< Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)*/
 float rollspeed; /*< Roll angular speed (rad/s)*/
 float pitchspeed; /*< Pitch angular speed (rad/s)*/
 float yawspeed; /*< Yaw angular speed (rad/s)*/
 float covariance[9]; /*< Attitude covariance*/
} mavlink_attitude_quaternion_cov_t;

#define MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN 68
#define MAVLINK_MSG_ID_61_LEN 68

#define MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_CRC 153
#define MAVLINK_MSG_ID_61_CRC 153

#define MAVLINK_MSG_ATTITUDE_QUATERNION_COV_FIELD_Q_LEN 4
#define MAVLINK_MSG_ATTITUDE_QUATERNION_COV_FIELD_COVARIANCE_LEN 9

#define MAVLINK_MESSAGE_INFO_ATTITUDE_QUATERNION_COV { \
	"ATTITUDE_QUATERNION_COV", \
	6, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_attitude_quaternion_cov_t, time_boot_ms) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 4, 4, offsetof(mavlink_attitude_quaternion_cov_t, q) }, \
         { "rollspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_attitude_quaternion_cov_t, rollspeed) }, \
         { "pitchspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_attitude_quaternion_cov_t, pitchspeed) }, \
         { "yawspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_attitude_quaternion_cov_t, yawspeed) }, \
         { "covariance", NULL, MAVLINK_TYPE_FLOAT, 9, 32, offsetof(mavlink_attitude_quaternion_cov_t, covariance) }, \
         } \
}


/**
 * @brief Pack a attitude_quaternion_cov message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param q Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @param covariance Attitude covariance
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude_quaternion_cov_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, const float *q, float rollspeed, float pitchspeed, float yawspeed, const float *covariance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 20, rollspeed);
	_mav_put_float(buf, 24, pitchspeed);
	_mav_put_float(buf, 28, yawspeed);
	_mav_put_float_array(buf, 4, q, 4);
	_mav_put_float_array(buf, 32, covariance, 9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN);
#else
	mavlink_attitude_quaternion_cov_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.rollspeed = rollspeed;
	packet.pitchspeed = pitchspeed;
	packet.yawspeed = yawspeed;
	mav_array_memcpy(packet.q, q, sizeof(float)*4);
	mav_array_memcpy(packet.covariance, covariance, sizeof(float)*9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN);
#endif
}

/**
 * @brief Pack a attitude_quaternion_cov message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param q Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @param covariance Attitude covariance
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude_quaternion_cov_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,const float *q,float rollspeed,float pitchspeed,float yawspeed,const float *covariance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 20, rollspeed);
	_mav_put_float(buf, 24, pitchspeed);
	_mav_put_float(buf, 28, yawspeed);
	_mav_put_float_array(buf, 4, q, 4);
	_mav_put_float_array(buf, 32, covariance, 9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN);
#else
	mavlink_attitude_quaternion_cov_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.rollspeed = rollspeed;
	packet.pitchspeed = pitchspeed;
	packet.yawspeed = yawspeed;
	mav_array_memcpy(packet.q, q, sizeof(float)*4);
	mav_array_memcpy(packet.covariance, covariance, sizeof(float)*9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN);
#endif
}

/**
 * @brief Encode a attitude_quaternion_cov struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param attitude_quaternion_cov C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_attitude_quaternion_cov_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_attitude_quaternion_cov_t* attitude_quaternion_cov)
{
	return mavlink_msg_attitude_quaternion_cov_pack(system_id, component_id, msg, attitude_quaternion_cov->time_boot_ms, attitude_quaternion_cov->q, attitude_quaternion_cov->rollspeed, attitude_quaternion_cov->pitchspeed, attitude_quaternion_cov->yawspeed, attitude_quaternion_cov->covariance);
}

/**
 * @brief Encode a attitude_quaternion_cov struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param attitude_quaternion_cov C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_attitude_quaternion_cov_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_attitude_quaternion_cov_t* attitude_quaternion_cov)
{
	return mavlink_msg_attitude_quaternion_cov_pack_chan(system_id, component_id, chan, msg, attitude_quaternion_cov->time_boot_ms, attitude_quaternion_cov->q, attitude_quaternion_cov->rollspeed, attitude_quaternion_cov->pitchspeed, attitude_quaternion_cov->yawspeed, attitude_quaternion_cov->covariance);
}

/**
 * @brief Send a attitude_quaternion_cov message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param q Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @param covariance Attitude covariance
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_attitude_quaternion_cov_send(mavlink_channel_t chan, uint32_t time_boot_ms, const float *q, float rollspeed, float pitchspeed, float yawspeed, const float *covariance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 20, rollspeed);
	_mav_put_float(buf, 24, pitchspeed);
	_mav_put_float(buf, 28, yawspeed);
	_mav_put_float_array(buf, 4, q, 4);
	_mav_put_float_array(buf, 32, covariance, 9);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV, buf, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV, buf, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN);
#endif
#else
	mavlink_attitude_quaternion_cov_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.rollspeed = rollspeed;
	packet.pitchspeed = pitchspeed;
	packet.yawspeed = yawspeed;
	mav_array_memcpy(packet.q, q, sizeof(float)*4);
	mav_array_memcpy(packet.covariance, covariance, sizeof(float)*9);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV, (const char *)&packet, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV, (const char *)&packet, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_attitude_quaternion_cov_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, const float *q, float rollspeed, float pitchspeed, float yawspeed, const float *covariance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 20, rollspeed);
	_mav_put_float(buf, 24, pitchspeed);
	_mav_put_float(buf, 28, yawspeed);
	_mav_put_float_array(buf, 4, q, 4);
	_mav_put_float_array(buf, 32, covariance, 9);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV, buf, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV, buf, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN);
#endif
#else
	mavlink_attitude_quaternion_cov_t *packet = (mavlink_attitude_quaternion_cov_t *)msgbuf;
	packet->time_boot_ms = time_boot_ms;
	packet->rollspeed = rollspeed;
	packet->pitchspeed = pitchspeed;
	packet->yawspeed = yawspeed;
	mav_array_memcpy(packet->q, q, sizeof(float)*4);
	mav_array_memcpy(packet->covariance, covariance, sizeof(float)*9);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV, (const char *)packet, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV, (const char *)packet, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE ATTITUDE_QUATERNION_COV UNPACKING


/**
 * @brief Get field time_boot_ms from attitude_quaternion_cov message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_attitude_quaternion_cov_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field q from attitude_quaternion_cov message
 *
 * @return Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
 */
static inline uint16_t mavlink_msg_attitude_quaternion_cov_get_q(const mavlink_message_t* msg, float *q)
{
	return _MAV_RETURN_float_array(msg, q, 4,  4);
}

/**
 * @brief Get field rollspeed from attitude_quaternion_cov message
 *
 * @return Roll angular speed (rad/s)
 */
static inline float mavlink_msg_attitude_quaternion_cov_get_rollspeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field pitchspeed from attitude_quaternion_cov message
 *
 * @return Pitch angular speed (rad/s)
 */
static inline float mavlink_msg_attitude_quaternion_cov_get_pitchspeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field yawspeed from attitude_quaternion_cov message
 *
 * @return Yaw angular speed (rad/s)
 */
static inline float mavlink_msg_attitude_quaternion_cov_get_yawspeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field covariance from attitude_quaternion_cov message
 *
 * @return Attitude covariance
 */
static inline uint16_t mavlink_msg_attitude_quaternion_cov_get_covariance(const mavlink_message_t* msg, float *covariance)
{
	return _MAV_RETURN_float_array(msg, covariance, 9,  32);
}

/**
 * @brief Decode a attitude_quaternion_cov message into a struct
 *
 * @param msg The message to decode
 * @param attitude_quaternion_cov C-struct to decode the message contents into
 */
static inline void mavlink_msg_attitude_quaternion_cov_decode(const mavlink_message_t* msg, mavlink_attitude_quaternion_cov_t* attitude_quaternion_cov)
{
#if MAVLINK_NEED_BYTE_SWAP
	attitude_quaternion_cov->time_boot_ms = mavlink_msg_attitude_quaternion_cov_get_time_boot_ms(msg);
	mavlink_msg_attitude_quaternion_cov_get_q(msg, attitude_quaternion_cov->q);
	attitude_quaternion_cov->rollspeed = mavlink_msg_attitude_quaternion_cov_get_rollspeed(msg);
	attitude_quaternion_cov->pitchspeed = mavlink_msg_attitude_quaternion_cov_get_pitchspeed(msg);
	attitude_quaternion_cov->yawspeed = mavlink_msg_attitude_quaternion_cov_get_yawspeed(msg);
	mavlink_msg_attitude_quaternion_cov_get_covariance(msg, attitude_quaternion_cov->covariance);
#else
	memcpy(attitude_quaternion_cov, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN);
#endif
}
