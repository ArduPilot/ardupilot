// MESSAGE PREARM_CHECK_FAILURE_CODE PACKING

#define MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE 247

typedef struct __mavlink_prearm_check_failure_code_t
{
 uint32_t code; /*< failure code (autopilot specific, corresponding text should be looked up in Arming.xml)*/
 uint8_t check_id; /*< corresponds to bit position of PREARM_CHECK_REPORT's enabled_checks field*/
} mavlink_prearm_check_failure_code_t;

#define MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE_LEN 5
#define MAVLINK_MSG_ID_247_LEN 5

#define MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE_CRC 140
#define MAVLINK_MSG_ID_247_CRC 140



#define MAVLINK_MESSAGE_INFO_PREARM_CHECK_FAILURE_CODE { \
	"PREARM_CHECK_FAILURE_CODE", \
	2, \
	{  { "code", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_prearm_check_failure_code_t, code) }, \
         { "check_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_prearm_check_failure_code_t, check_id) }, \
         } \
}


/**
 * @brief Pack a prearm_check_failure_code message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param check_id corresponds to bit position of PREARM_CHECK_REPORT's enabled_checks field
 * @param code failure code (autopilot specific, corresponding text should be looked up in Arming.xml)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_prearm_check_failure_code_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t check_id, uint32_t code)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE_LEN];
	_mav_put_uint32_t(buf, 0, code);
	_mav_put_uint8_t(buf, 4, check_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE_LEN);
#else
	mavlink_prearm_check_failure_code_t packet;
	packet.code = code;
	packet.check_id = check_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE_LEN, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE_LEN);
#endif
}

/**
 * @brief Pack a prearm_check_failure_code message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param check_id corresponds to bit position of PREARM_CHECK_REPORT's enabled_checks field
 * @param code failure code (autopilot specific, corresponding text should be looked up in Arming.xml)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_prearm_check_failure_code_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t check_id,uint32_t code)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE_LEN];
	_mav_put_uint32_t(buf, 0, code);
	_mav_put_uint8_t(buf, 4, check_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE_LEN);
#else
	mavlink_prearm_check_failure_code_t packet;
	packet.code = code;
	packet.check_id = check_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE_LEN, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE_LEN);
#endif
}

/**
 * @brief Encode a prearm_check_failure_code struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param prearm_check_failure_code C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_prearm_check_failure_code_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_prearm_check_failure_code_t* prearm_check_failure_code)
{
	return mavlink_msg_prearm_check_failure_code_pack(system_id, component_id, msg, prearm_check_failure_code->check_id, prearm_check_failure_code->code);
}

/**
 * @brief Encode a prearm_check_failure_code struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param prearm_check_failure_code C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_prearm_check_failure_code_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_prearm_check_failure_code_t* prearm_check_failure_code)
{
	return mavlink_msg_prearm_check_failure_code_pack_chan(system_id, component_id, chan, msg, prearm_check_failure_code->check_id, prearm_check_failure_code->code);
}

/**
 * @brief Send a prearm_check_failure_code message
 * @param chan MAVLink channel to send the message
 *
 * @param check_id corresponds to bit position of PREARM_CHECK_REPORT's enabled_checks field
 * @param code failure code (autopilot specific, corresponding text should be looked up in Arming.xml)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_prearm_check_failure_code_send(mavlink_channel_t chan, uint8_t check_id, uint32_t code)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE_LEN];
	_mav_put_uint32_t(buf, 0, code);
	_mav_put_uint8_t(buf, 4, check_id);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE, buf, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE_LEN, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE, buf, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE_LEN);
#endif
#else
	mavlink_prearm_check_failure_code_t packet;
	packet.code = code;
	packet.check_id = check_id;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE, (const char *)&packet, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE_LEN, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE, (const char *)&packet, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_prearm_check_failure_code_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t check_id, uint32_t code)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, code);
	_mav_put_uint8_t(buf, 4, check_id);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE, buf, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE_LEN, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE, buf, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE_LEN);
#endif
#else
	mavlink_prearm_check_failure_code_t *packet = (mavlink_prearm_check_failure_code_t *)msgbuf;
	packet->code = code;
	packet->check_id = check_id;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE, (const char *)packet, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE_LEN, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE, (const char *)packet, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE PREARM_CHECK_FAILURE_CODE UNPACKING


/**
 * @brief Get field check_id from prearm_check_failure_code message
 *
 * @return corresponds to bit position of PREARM_CHECK_REPORT's enabled_checks field
 */
static inline uint8_t mavlink_msg_prearm_check_failure_code_get_check_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field code from prearm_check_failure_code message
 *
 * @return failure code (autopilot specific, corresponding text should be looked up in Arming.xml)
 */
static inline uint32_t mavlink_msg_prearm_check_failure_code_get_code(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Decode a prearm_check_failure_code message into a struct
 *
 * @param msg The message to decode
 * @param prearm_check_failure_code C-struct to decode the message contents into
 */
static inline void mavlink_msg_prearm_check_failure_code_decode(const mavlink_message_t* msg, mavlink_prearm_check_failure_code_t* prearm_check_failure_code)
{
#if MAVLINK_NEED_BYTE_SWAP
	prearm_check_failure_code->code = mavlink_msg_prearm_check_failure_code_get_code(msg);
	prearm_check_failure_code->check_id = mavlink_msg_prearm_check_failure_code_get_check_id(msg);
#else
	memcpy(prearm_check_failure_code, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_CODE_LEN);
#endif
}
