// MESSAGE ATTITUDE_TARGET PACKING

#define MAVLINK_MSG_ID_ATTITUDE_TARGET 83

typedef struct __mavlink_attitude_target_t
{
 uint32_t time_boot_ms; ///< Timestamp in milliseconds since system boot
 float q[4]; ///< Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
 float body_roll_rate; ///< Body roll rate in radians per second
 float body_pitch_rate; ///< Body roll rate in radians per second
 float body_yaw_rate; ///< Body roll rate in radians per second
 float thrust; ///< Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
 uint8_t type_mask; ///< Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 7: reserved, bit 8: attitude
} mavlink_attitude_target_t;

#define MAVLINK_MSG_ID_ATTITUDE_TARGET_LEN 37
#define MAVLINK_MSG_ID_83_LEN 37

#define MAVLINK_MSG_ID_ATTITUDE_TARGET_CRC 22
#define MAVLINK_MSG_ID_83_CRC 22

#define MAVLINK_MSG_ATTITUDE_TARGET_FIELD_Q_LEN 4

#define MAVLINK_MESSAGE_INFO_ATTITUDE_TARGET { \
	"ATTITUDE_TARGET", \
	7, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_attitude_target_t, time_boot_ms) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 4, 4, offsetof(mavlink_attitude_target_t, q) }, \
         { "body_roll_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_attitude_target_t, body_roll_rate) }, \
         { "body_pitch_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_attitude_target_t, body_pitch_rate) }, \
         { "body_yaw_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_attitude_target_t, body_yaw_rate) }, \
         { "thrust", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_attitude_target_t, thrust) }, \
         { "type_mask", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_attitude_target_t, type_mask) }, \
         } \
}


/**
 * @brief Pack a attitude_target message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp in milliseconds since system boot
 * @param type_mask Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 7: reserved, bit 8: attitude
 * @param q Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
 * @param body_roll_rate Body roll rate in radians per second
 * @param body_pitch_rate Body roll rate in radians per second
 * @param body_yaw_rate Body roll rate in radians per second
 * @param thrust Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude_target_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, uint8_t type_mask, const float *q, float body_roll_rate, float body_pitch_rate, float body_yaw_rate, float thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ATTITUDE_TARGET_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 20, body_roll_rate);
	_mav_put_float(buf, 24, body_pitch_rate);
	_mav_put_float(buf, 28, body_yaw_rate);
	_mav_put_float(buf, 32, thrust);
	_mav_put_uint8_t(buf, 36, type_mask);
	_mav_put_float_array(buf, 4, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATTITUDE_TARGET_LEN);
#else
	mavlink_attitude_target_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.body_roll_rate = body_roll_rate;
	packet.body_pitch_rate = body_pitch_rate;
	packet.body_yaw_rate = body_yaw_rate;
	packet.thrust = thrust;
	packet.type_mask = type_mask;
	mav_array_memcpy(packet.q, q, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATTITUDE_TARGET_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ATTITUDE_TARGET;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ATTITUDE_TARGET_LEN, MAVLINK_MSG_ID_ATTITUDE_TARGET_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ATTITUDE_TARGET_LEN);
#endif
}

/**
 * @brief Pack a attitude_target message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp in milliseconds since system boot
 * @param type_mask Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 7: reserved, bit 8: attitude
 * @param q Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
 * @param body_roll_rate Body roll rate in radians per second
 * @param body_pitch_rate Body roll rate in radians per second
 * @param body_yaw_rate Body roll rate in radians per second
 * @param thrust Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude_target_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,uint8_t type_mask,const float *q,float body_roll_rate,float body_pitch_rate,float body_yaw_rate,float thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ATTITUDE_TARGET_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 20, body_roll_rate);
	_mav_put_float(buf, 24, body_pitch_rate);
	_mav_put_float(buf, 28, body_yaw_rate);
	_mav_put_float(buf, 32, thrust);
	_mav_put_uint8_t(buf, 36, type_mask);
	_mav_put_float_array(buf, 4, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATTITUDE_TARGET_LEN);
#else
	mavlink_attitude_target_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.body_roll_rate = body_roll_rate;
	packet.body_pitch_rate = body_pitch_rate;
	packet.body_yaw_rate = body_yaw_rate;
	packet.thrust = thrust;
	packet.type_mask = type_mask;
	mav_array_memcpy(packet.q, q, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATTITUDE_TARGET_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ATTITUDE_TARGET;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ATTITUDE_TARGET_LEN, MAVLINK_MSG_ID_ATTITUDE_TARGET_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ATTITUDE_TARGET_LEN);
#endif
}

/**
 * @brief Encode a attitude_target struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param attitude_target C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_attitude_target_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_attitude_target_t* attitude_target)
{
	return mavlink_msg_attitude_target_pack(system_id, component_id, msg, attitude_target->time_boot_ms, attitude_target->type_mask, attitude_target->q, attitude_target->body_roll_rate, attitude_target->body_pitch_rate, attitude_target->body_yaw_rate, attitude_target->thrust);
}

/**
 * @brief Encode a attitude_target struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param attitude_target C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_attitude_target_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_attitude_target_t* attitude_target)
{
	return mavlink_msg_attitude_target_pack_chan(system_id, component_id, chan, msg, attitude_target->time_boot_ms, attitude_target->type_mask, attitude_target->q, attitude_target->body_roll_rate, attitude_target->body_pitch_rate, attitude_target->body_yaw_rate, attitude_target->thrust);
}

/**
 * @brief Send a attitude_target message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp in milliseconds since system boot
 * @param type_mask Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 7: reserved, bit 8: attitude
 * @param q Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
 * @param body_roll_rate Body roll rate in radians per second
 * @param body_pitch_rate Body roll rate in radians per second
 * @param body_yaw_rate Body roll rate in radians per second
 * @param thrust Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_attitude_target_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t type_mask, const float *q, float body_roll_rate, float body_pitch_rate, float body_yaw_rate, float thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ATTITUDE_TARGET_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 20, body_roll_rate);
	_mav_put_float(buf, 24, body_pitch_rate);
	_mav_put_float(buf, 28, body_yaw_rate);
	_mav_put_float(buf, 32, thrust);
	_mav_put_uint8_t(buf, 36, type_mask);
	_mav_put_float_array(buf, 4, q, 4);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_TARGET, buf, MAVLINK_MSG_ID_ATTITUDE_TARGET_LEN, MAVLINK_MSG_ID_ATTITUDE_TARGET_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_TARGET, buf, MAVLINK_MSG_ID_ATTITUDE_TARGET_LEN);
#endif
#else
	mavlink_attitude_target_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.body_roll_rate = body_roll_rate;
	packet.body_pitch_rate = body_pitch_rate;
	packet.body_yaw_rate = body_yaw_rate;
	packet.thrust = thrust;
	packet.type_mask = type_mask;
	mav_array_memcpy(packet.q, q, sizeof(float)*4);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_TARGET, (const char *)&packet, MAVLINK_MSG_ID_ATTITUDE_TARGET_LEN, MAVLINK_MSG_ID_ATTITUDE_TARGET_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_TARGET, (const char *)&packet, MAVLINK_MSG_ID_ATTITUDE_TARGET_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_ATTITUDE_TARGET_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_attitude_target_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint8_t type_mask, const float *q, float body_roll_rate, float body_pitch_rate, float body_yaw_rate, float thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 20, body_roll_rate);
	_mav_put_float(buf, 24, body_pitch_rate);
	_mav_put_float(buf, 28, body_yaw_rate);
	_mav_put_float(buf, 32, thrust);
	_mav_put_uint8_t(buf, 36, type_mask);
	_mav_put_float_array(buf, 4, q, 4);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_TARGET, buf, MAVLINK_MSG_ID_ATTITUDE_TARGET_LEN, MAVLINK_MSG_ID_ATTITUDE_TARGET_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_TARGET, buf, MAVLINK_MSG_ID_ATTITUDE_TARGET_LEN);
#endif
#else
	mavlink_attitude_target_t *packet = (mavlink_attitude_target_t *)msgbuf;
	packet->time_boot_ms = time_boot_ms;
	packet->body_roll_rate = body_roll_rate;
	packet->body_pitch_rate = body_pitch_rate;
	packet->body_yaw_rate = body_yaw_rate;
	packet->thrust = thrust;
	packet->type_mask = type_mask;
	mav_array_memcpy(packet->q, q, sizeof(float)*4);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_TARGET, (const char *)packet, MAVLINK_MSG_ID_ATTITUDE_TARGET_LEN, MAVLINK_MSG_ID_ATTITUDE_TARGET_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_TARGET, (const char *)packet, MAVLINK_MSG_ID_ATTITUDE_TARGET_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE ATTITUDE_TARGET UNPACKING


/**
 * @brief Get field time_boot_ms from attitude_target message
 *
 * @return Timestamp in milliseconds since system boot
 */
static inline uint32_t mavlink_msg_attitude_target_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field type_mask from attitude_target message
 *
 * @return Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 7: reserved, bit 8: attitude
 */
static inline uint8_t mavlink_msg_attitude_target_get_type_mask(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Get field q from attitude_target message
 *
 * @return Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
 */
static inline uint16_t mavlink_msg_attitude_target_get_q(const mavlink_message_t* msg, float *q)
{
	return _MAV_RETURN_float_array(msg, q, 4,  4);
}

/**
 * @brief Get field body_roll_rate from attitude_target message
 *
 * @return Body roll rate in radians per second
 */
static inline float mavlink_msg_attitude_target_get_body_roll_rate(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field body_pitch_rate from attitude_target message
 *
 * @return Body roll rate in radians per second
 */
static inline float mavlink_msg_attitude_target_get_body_pitch_rate(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field body_yaw_rate from attitude_target message
 *
 * @return Body roll rate in radians per second
 */
static inline float mavlink_msg_attitude_target_get_body_yaw_rate(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field thrust from attitude_target message
 *
 * @return Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
 */
static inline float mavlink_msg_attitude_target_get_thrust(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Decode a attitude_target message into a struct
 *
 * @param msg The message to decode
 * @param attitude_target C-struct to decode the message contents into
 */
static inline void mavlink_msg_attitude_target_decode(const mavlink_message_t* msg, mavlink_attitude_target_t* attitude_target)
{
#if MAVLINK_NEED_BYTE_SWAP
	attitude_target->time_boot_ms = mavlink_msg_attitude_target_get_time_boot_ms(msg);
	mavlink_msg_attitude_target_get_q(msg, attitude_target->q);
	attitude_target->body_roll_rate = mavlink_msg_attitude_target_get_body_roll_rate(msg);
	attitude_target->body_pitch_rate = mavlink_msg_attitude_target_get_body_pitch_rate(msg);
	attitude_target->body_yaw_rate = mavlink_msg_attitude_target_get_body_yaw_rate(msg);
	attitude_target->thrust = mavlink_msg_attitude_target_get_thrust(msg);
	attitude_target->type_mask = mavlink_msg_attitude_target_get_type_mask(msg);
#else
	memcpy(attitude_target, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ATTITUDE_TARGET_LEN);
#endif
}
