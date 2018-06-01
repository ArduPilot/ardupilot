// MESSAGE GIMBAL_HOME_OFFSET_CALIBRATION_RESULT PACKING

#define MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT 205

typedef struct __mavlink_gimbal_home_offset_calibration_result_t
{
 uint8_t calibration_result; /*< The result of the home offset calibration*/
} mavlink_gimbal_home_offset_calibration_result_t;

#define MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT_LEN 1
#define MAVLINK_MSG_ID_205_LEN 1

#define MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT_CRC 63
#define MAVLINK_MSG_ID_205_CRC 63



#define MAVLINK_MESSAGE_INFO_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT { \
	"GIMBAL_HOME_OFFSET_CALIBRATION_RESULT", \
	1, \
	{  { "calibration_result", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_gimbal_home_offset_calibration_result_t, calibration_result) }, \
         } \
}


/**
 * @brief Pack a gimbal_home_offset_calibration_result message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param calibration_result The result of the home offset calibration
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_home_offset_calibration_result_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t calibration_result)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT_LEN];
	_mav_put_uint8_t(buf, 0, calibration_result);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT_LEN);
#else
	mavlink_gimbal_home_offset_calibration_result_t packet;
	packet.calibration_result = calibration_result;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT_LEN, MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT_LEN);
#endif
}

/**
 * @brief Pack a gimbal_home_offset_calibration_result message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param calibration_result The result of the home offset calibration
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_home_offset_calibration_result_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t calibration_result)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT_LEN];
	_mav_put_uint8_t(buf, 0, calibration_result);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT_LEN);
#else
	mavlink_gimbal_home_offset_calibration_result_t packet;
	packet.calibration_result = calibration_result;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT_LEN, MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT_LEN);
#endif
}

/**
 * @brief Encode a gimbal_home_offset_calibration_result struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_home_offset_calibration_result C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_home_offset_calibration_result_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gimbal_home_offset_calibration_result_t* gimbal_home_offset_calibration_result)
{
	return mavlink_msg_gimbal_home_offset_calibration_result_pack(system_id, component_id, msg, gimbal_home_offset_calibration_result->calibration_result);
}

/**
 * @brief Encode a gimbal_home_offset_calibration_result struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_home_offset_calibration_result C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_home_offset_calibration_result_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gimbal_home_offset_calibration_result_t* gimbal_home_offset_calibration_result)
{
	return mavlink_msg_gimbal_home_offset_calibration_result_pack_chan(system_id, component_id, chan, msg, gimbal_home_offset_calibration_result->calibration_result);
}

/**
 * @brief Send a gimbal_home_offset_calibration_result message
 * @param chan MAVLink channel to send the message
 *
 * @param calibration_result The result of the home offset calibration
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gimbal_home_offset_calibration_result_send(mavlink_channel_t chan, uint8_t calibration_result)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT_LEN];
	_mav_put_uint8_t(buf, 0, calibration_result);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT, buf, MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT_LEN, MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT, buf, MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT_LEN);
#endif
#else
	mavlink_gimbal_home_offset_calibration_result_t packet;
	packet.calibration_result = calibration_result;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT, (const char *)&packet, MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT_LEN, MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT, (const char *)&packet, MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gimbal_home_offset_calibration_result_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t calibration_result)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, calibration_result);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT, buf, MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT_LEN, MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT, buf, MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT_LEN);
#endif
#else
	mavlink_gimbal_home_offset_calibration_result_t *packet = (mavlink_gimbal_home_offset_calibration_result_t *)msgbuf;
	packet->calibration_result = calibration_result;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT, (const char *)packet, MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT_LEN, MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT, (const char *)packet, MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE GIMBAL_HOME_OFFSET_CALIBRATION_RESULT UNPACKING


/**
 * @brief Get field calibration_result from gimbal_home_offset_calibration_result message
 *
 * @return The result of the home offset calibration
 */
static inline uint8_t mavlink_msg_gimbal_home_offset_calibration_result_get_calibration_result(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a gimbal_home_offset_calibration_result message into a struct
 *
 * @param msg The message to decode
 * @param gimbal_home_offset_calibration_result C-struct to decode the message contents into
 */
static inline void mavlink_msg_gimbal_home_offset_calibration_result_decode(const mavlink_message_t* msg, mavlink_gimbal_home_offset_calibration_result_t* gimbal_home_offset_calibration_result)
{
#if MAVLINK_NEED_BYTE_SWAP
	gimbal_home_offset_calibration_result->calibration_result = mavlink_msg_gimbal_home_offset_calibration_result_get_calibration_result(msg);
#else
	memcpy(gimbal_home_offset_calibration_result, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT_LEN);
#endif
}
