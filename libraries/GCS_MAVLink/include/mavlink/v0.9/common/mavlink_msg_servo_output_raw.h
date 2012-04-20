// MESSAGE SERVO_OUTPUT_RAW PACKING

#define MAVLINK_MSG_ID_SERVO_OUTPUT_RAW 37

typedef struct __mavlink_servo_output_raw_t
{
 uint16_t servo1_raw; ///< Servo output 1 value, in microseconds
 uint16_t servo2_raw; ///< Servo output 2 value, in microseconds
 uint16_t servo3_raw; ///< Servo output 3 value, in microseconds
 uint16_t servo4_raw; ///< Servo output 4 value, in microseconds
 uint16_t servo5_raw; ///< Servo output 5 value, in microseconds
 uint16_t servo6_raw; ///< Servo output 6 value, in microseconds
 uint16_t servo7_raw; ///< Servo output 7 value, in microseconds
 uint16_t servo8_raw; ///< Servo output 8 value, in microseconds
} mavlink_servo_output_raw_t;

#define MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN 16
#define MAVLINK_MSG_ID_37_LEN 16



#define MAVLINK_MESSAGE_INFO_SERVO_OUTPUT_RAW { \
	"SERVO_OUTPUT_RAW", \
	8, \
	{  { "servo1_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_servo_output_raw_t, servo1_raw) }, \
         { "servo2_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_servo_output_raw_t, servo2_raw) }, \
         { "servo3_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_servo_output_raw_t, servo3_raw) }, \
         { "servo4_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_servo_output_raw_t, servo4_raw) }, \
         { "servo5_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_servo_output_raw_t, servo5_raw) }, \
         { "servo6_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_servo_output_raw_t, servo6_raw) }, \
         { "servo7_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_servo_output_raw_t, servo7_raw) }, \
         { "servo8_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_servo_output_raw_t, servo8_raw) }, \
         } \
}


/**
 * @brief Pack a servo_output_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param servo1_raw Servo output 1 value, in microseconds
 * @param servo2_raw Servo output 2 value, in microseconds
 * @param servo3_raw Servo output 3 value, in microseconds
 * @param servo4_raw Servo output 4 value, in microseconds
 * @param servo5_raw Servo output 5 value, in microseconds
 * @param servo6_raw Servo output 6 value, in microseconds
 * @param servo7_raw Servo output 7 value, in microseconds
 * @param servo8_raw Servo output 8 value, in microseconds
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_servo_output_raw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t servo1_raw, uint16_t servo2_raw, uint16_t servo3_raw, uint16_t servo4_raw, uint16_t servo5_raw, uint16_t servo6_raw, uint16_t servo7_raw, uint16_t servo8_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[16];
	_mav_put_uint16_t(buf, 0, servo1_raw);
	_mav_put_uint16_t(buf, 2, servo2_raw);
	_mav_put_uint16_t(buf, 4, servo3_raw);
	_mav_put_uint16_t(buf, 6, servo4_raw);
	_mav_put_uint16_t(buf, 8, servo5_raw);
	_mav_put_uint16_t(buf, 10, servo6_raw);
	_mav_put_uint16_t(buf, 12, servo7_raw);
	_mav_put_uint16_t(buf, 14, servo8_raw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 16);
#else
	mavlink_servo_output_raw_t packet;
	packet.servo1_raw = servo1_raw;
	packet.servo2_raw = servo2_raw;
	packet.servo3_raw = servo3_raw;
	packet.servo4_raw = servo4_raw;
	packet.servo5_raw = servo5_raw;
	packet.servo6_raw = servo6_raw;
	packet.servo7_raw = servo7_raw;
	packet.servo8_raw = servo8_raw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 16);
#endif

	msg->msgid = MAVLINK_MSG_ID_SERVO_OUTPUT_RAW;
	return mavlink_finalize_message(msg, system_id, component_id, 16);
}

/**
 * @brief Pack a servo_output_raw message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param servo1_raw Servo output 1 value, in microseconds
 * @param servo2_raw Servo output 2 value, in microseconds
 * @param servo3_raw Servo output 3 value, in microseconds
 * @param servo4_raw Servo output 4 value, in microseconds
 * @param servo5_raw Servo output 5 value, in microseconds
 * @param servo6_raw Servo output 6 value, in microseconds
 * @param servo7_raw Servo output 7 value, in microseconds
 * @param servo8_raw Servo output 8 value, in microseconds
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_servo_output_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t servo1_raw,uint16_t servo2_raw,uint16_t servo3_raw,uint16_t servo4_raw,uint16_t servo5_raw,uint16_t servo6_raw,uint16_t servo7_raw,uint16_t servo8_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[16];
	_mav_put_uint16_t(buf, 0, servo1_raw);
	_mav_put_uint16_t(buf, 2, servo2_raw);
	_mav_put_uint16_t(buf, 4, servo3_raw);
	_mav_put_uint16_t(buf, 6, servo4_raw);
	_mav_put_uint16_t(buf, 8, servo5_raw);
	_mav_put_uint16_t(buf, 10, servo6_raw);
	_mav_put_uint16_t(buf, 12, servo7_raw);
	_mav_put_uint16_t(buf, 14, servo8_raw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 16);
#else
	mavlink_servo_output_raw_t packet;
	packet.servo1_raw = servo1_raw;
	packet.servo2_raw = servo2_raw;
	packet.servo3_raw = servo3_raw;
	packet.servo4_raw = servo4_raw;
	packet.servo5_raw = servo5_raw;
	packet.servo6_raw = servo6_raw;
	packet.servo7_raw = servo7_raw;
	packet.servo8_raw = servo8_raw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 16);
#endif

	msg->msgid = MAVLINK_MSG_ID_SERVO_OUTPUT_RAW;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 16);
}

/**
 * @brief Encode a servo_output_raw struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param servo_output_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_servo_output_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_servo_output_raw_t* servo_output_raw)
{
	return mavlink_msg_servo_output_raw_pack(system_id, component_id, msg, servo_output_raw->servo1_raw, servo_output_raw->servo2_raw, servo_output_raw->servo3_raw, servo_output_raw->servo4_raw, servo_output_raw->servo5_raw, servo_output_raw->servo6_raw, servo_output_raw->servo7_raw, servo_output_raw->servo8_raw);
}

/**
 * @brief Send a servo_output_raw message
 * @param chan MAVLink channel to send the message
 *
 * @param servo1_raw Servo output 1 value, in microseconds
 * @param servo2_raw Servo output 2 value, in microseconds
 * @param servo3_raw Servo output 3 value, in microseconds
 * @param servo4_raw Servo output 4 value, in microseconds
 * @param servo5_raw Servo output 5 value, in microseconds
 * @param servo6_raw Servo output 6 value, in microseconds
 * @param servo7_raw Servo output 7 value, in microseconds
 * @param servo8_raw Servo output 8 value, in microseconds
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_servo_output_raw_send(mavlink_channel_t chan, uint16_t servo1_raw, uint16_t servo2_raw, uint16_t servo3_raw, uint16_t servo4_raw, uint16_t servo5_raw, uint16_t servo6_raw, uint16_t servo7_raw, uint16_t servo8_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[16];
	_mav_put_uint16_t(buf, 0, servo1_raw);
	_mav_put_uint16_t(buf, 2, servo2_raw);
	_mav_put_uint16_t(buf, 4, servo3_raw);
	_mav_put_uint16_t(buf, 6, servo4_raw);
	_mav_put_uint16_t(buf, 8, servo5_raw);
	_mav_put_uint16_t(buf, 10, servo6_raw);
	_mav_put_uint16_t(buf, 12, servo7_raw);
	_mav_put_uint16_t(buf, 14, servo8_raw);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, buf, 16);
#else
	mavlink_servo_output_raw_t packet;
	packet.servo1_raw = servo1_raw;
	packet.servo2_raw = servo2_raw;
	packet.servo3_raw = servo3_raw;
	packet.servo4_raw = servo4_raw;
	packet.servo5_raw = servo5_raw;
	packet.servo6_raw = servo6_raw;
	packet.servo7_raw = servo7_raw;
	packet.servo8_raw = servo8_raw;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, (const char *)&packet, 16);
#endif
}

#endif

// MESSAGE SERVO_OUTPUT_RAW UNPACKING


/**
 * @brief Get field servo1_raw from servo_output_raw message
 *
 * @return Servo output 1 value, in microseconds
 */
static inline uint16_t mavlink_msg_servo_output_raw_get_servo1_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field servo2_raw from servo_output_raw message
 *
 * @return Servo output 2 value, in microseconds
 */
static inline uint16_t mavlink_msg_servo_output_raw_get_servo2_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field servo3_raw from servo_output_raw message
 *
 * @return Servo output 3 value, in microseconds
 */
static inline uint16_t mavlink_msg_servo_output_raw_get_servo3_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field servo4_raw from servo_output_raw message
 *
 * @return Servo output 4 value, in microseconds
 */
static inline uint16_t mavlink_msg_servo_output_raw_get_servo4_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field servo5_raw from servo_output_raw message
 *
 * @return Servo output 5 value, in microseconds
 */
static inline uint16_t mavlink_msg_servo_output_raw_get_servo5_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field servo6_raw from servo_output_raw message
 *
 * @return Servo output 6 value, in microseconds
 */
static inline uint16_t mavlink_msg_servo_output_raw_get_servo6_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Get field servo7_raw from servo_output_raw message
 *
 * @return Servo output 7 value, in microseconds
 */
static inline uint16_t mavlink_msg_servo_output_raw_get_servo7_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field servo8_raw from servo_output_raw message
 *
 * @return Servo output 8 value, in microseconds
 */
static inline uint16_t mavlink_msg_servo_output_raw_get_servo8_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  14);
}

/**
 * @brief Decode a servo_output_raw message into a struct
 *
 * @param msg The message to decode
 * @param servo_output_raw C-struct to decode the message contents into
 */
static inline void mavlink_msg_servo_output_raw_decode(const mavlink_message_t* msg, mavlink_servo_output_raw_t* servo_output_raw)
{
#if MAVLINK_NEED_BYTE_SWAP
	servo_output_raw->servo1_raw = mavlink_msg_servo_output_raw_get_servo1_raw(msg);
	servo_output_raw->servo2_raw = mavlink_msg_servo_output_raw_get_servo2_raw(msg);
	servo_output_raw->servo3_raw = mavlink_msg_servo_output_raw_get_servo3_raw(msg);
	servo_output_raw->servo4_raw = mavlink_msg_servo_output_raw_get_servo4_raw(msg);
	servo_output_raw->servo5_raw = mavlink_msg_servo_output_raw_get_servo5_raw(msg);
	servo_output_raw->servo6_raw = mavlink_msg_servo_output_raw_get_servo6_raw(msg);
	servo_output_raw->servo7_raw = mavlink_msg_servo_output_raw_get_servo7_raw(msg);
	servo_output_raw->servo8_raw = mavlink_msg_servo_output_raw_get_servo8_raw(msg);
#else
	memcpy(servo_output_raw, _MAV_PAYLOAD(msg), 16);
#endif
}
