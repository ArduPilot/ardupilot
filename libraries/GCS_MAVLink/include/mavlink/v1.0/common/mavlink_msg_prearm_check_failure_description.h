// MESSAGE PREARM_CHECK_FAILURE_DESCRIPTION PACKING

#define MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION 247

typedef struct __mavlink_prearm_check_failure_description_t
{
 uint32_t code; /*< failure code (autopilot specific, may allow GCS to recognise failure without parsing text string)*/
 uint8_t subsystem; /*< subsystem that has failed pre-arm check*/
 char description[50]; /*< text description of the failure*/
} mavlink_prearm_check_failure_description_t;

#define MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION_LEN 55
#define MAVLINK_MSG_ID_247_LEN 55

#define MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION_CRC 169
#define MAVLINK_MSG_ID_247_CRC 169

#define MAVLINK_MSG_PREARM_CHECK_FAILURE_DESCRIPTION_FIELD_DESCRIPTION_LEN 50

#define MAVLINK_MESSAGE_INFO_PREARM_CHECK_FAILURE_DESCRIPTION { \
	"PREARM_CHECK_FAILURE_DESCRIPTION", \
	3, \
	{  { "code", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_prearm_check_failure_description_t, code) }, \
         { "subsystem", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_prearm_check_failure_description_t, subsystem) }, \
         { "description", NULL, MAVLINK_TYPE_CHAR, 50, 5, offsetof(mavlink_prearm_check_failure_description_t, description) }, \
         } \
}


/**
 * @brief Pack a prearm_check_failure_description message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param subsystem subsystem that has failed pre-arm check
 * @param code failure code (autopilot specific, may allow GCS to recognise failure without parsing text string)
 * @param description text description of the failure
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_prearm_check_failure_description_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t subsystem, uint32_t code, const char *description)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION_LEN];
	_mav_put_uint32_t(buf, 0, code);
	_mav_put_uint8_t(buf, 4, subsystem);
	_mav_put_char_array(buf, 5, description, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION_LEN);
#else
	mavlink_prearm_check_failure_description_t packet;
	packet.code = code;
	packet.subsystem = subsystem;
	mav_array_memcpy(packet.description, description, sizeof(char)*50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION_LEN, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION_LEN);
#endif
}

/**
 * @brief Pack a prearm_check_failure_description message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param subsystem subsystem that has failed pre-arm check
 * @param code failure code (autopilot specific, may allow GCS to recognise failure without parsing text string)
 * @param description text description of the failure
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_prearm_check_failure_description_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t subsystem,uint32_t code,const char *description)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION_LEN];
	_mav_put_uint32_t(buf, 0, code);
	_mav_put_uint8_t(buf, 4, subsystem);
	_mav_put_char_array(buf, 5, description, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION_LEN);
#else
	mavlink_prearm_check_failure_description_t packet;
	packet.code = code;
	packet.subsystem = subsystem;
	mav_array_memcpy(packet.description, description, sizeof(char)*50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION_LEN, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION_LEN);
#endif
}

/**
 * @brief Encode a prearm_check_failure_description struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param prearm_check_failure_description C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_prearm_check_failure_description_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_prearm_check_failure_description_t* prearm_check_failure_description)
{
	return mavlink_msg_prearm_check_failure_description_pack(system_id, component_id, msg, prearm_check_failure_description->subsystem, prearm_check_failure_description->code, prearm_check_failure_description->description);
}

/**
 * @brief Encode a prearm_check_failure_description struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param prearm_check_failure_description C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_prearm_check_failure_description_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_prearm_check_failure_description_t* prearm_check_failure_description)
{
	return mavlink_msg_prearm_check_failure_description_pack_chan(system_id, component_id, chan, msg, prearm_check_failure_description->subsystem, prearm_check_failure_description->code, prearm_check_failure_description->description);
}

/**
 * @brief Send a prearm_check_failure_description message
 * @param chan MAVLink channel to send the message
 *
 * @param subsystem subsystem that has failed pre-arm check
 * @param code failure code (autopilot specific, may allow GCS to recognise failure without parsing text string)
 * @param description text description of the failure
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_prearm_check_failure_description_send(mavlink_channel_t chan, uint8_t subsystem, uint32_t code, const char *description)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION_LEN];
	_mav_put_uint32_t(buf, 0, code);
	_mav_put_uint8_t(buf, 4, subsystem);
	_mav_put_char_array(buf, 5, description, 50);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION, buf, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION_LEN, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION, buf, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION_LEN);
#endif
#else
	mavlink_prearm_check_failure_description_t packet;
	packet.code = code;
	packet.subsystem = subsystem;
	mav_array_memcpy(packet.description, description, sizeof(char)*50);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION, (const char *)&packet, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION_LEN, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION, (const char *)&packet, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_prearm_check_failure_description_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t subsystem, uint32_t code, const char *description)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, code);
	_mav_put_uint8_t(buf, 4, subsystem);
	_mav_put_char_array(buf, 5, description, 50);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION, buf, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION_LEN, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION, buf, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION_LEN);
#endif
#else
	mavlink_prearm_check_failure_description_t *packet = (mavlink_prearm_check_failure_description_t *)msgbuf;
	packet->code = code;
	packet->subsystem = subsystem;
	mav_array_memcpy(packet->description, description, sizeof(char)*50);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION, (const char *)packet, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION_LEN, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION, (const char *)packet, MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE PREARM_CHECK_FAILURE_DESCRIPTION UNPACKING


/**
 * @brief Get field subsystem from prearm_check_failure_description message
 *
 * @return subsystem that has failed pre-arm check
 */
static inline uint8_t mavlink_msg_prearm_check_failure_description_get_subsystem(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field code from prearm_check_failure_description message
 *
 * @return failure code (autopilot specific, may allow GCS to recognise failure without parsing text string)
 */
static inline uint32_t mavlink_msg_prearm_check_failure_description_get_code(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field description from prearm_check_failure_description message
 *
 * @return text description of the failure
 */
static inline uint16_t mavlink_msg_prearm_check_failure_description_get_description(const mavlink_message_t* msg, char *description)
{
	return _MAV_RETURN_char_array(msg, description, 50,  5);
}

/**
 * @brief Decode a prearm_check_failure_description message into a struct
 *
 * @param msg The message to decode
 * @param prearm_check_failure_description C-struct to decode the message contents into
 */
static inline void mavlink_msg_prearm_check_failure_description_decode(const mavlink_message_t* msg, mavlink_prearm_check_failure_description_t* prearm_check_failure_description)
{
#if MAVLINK_NEED_BYTE_SWAP
	prearm_check_failure_description->code = mavlink_msg_prearm_check_failure_description_get_code(msg);
	prearm_check_failure_description->subsystem = mavlink_msg_prearm_check_failure_description_get_subsystem(msg);
	mavlink_msg_prearm_check_failure_description_get_description(msg, prearm_check_failure_description->description);
#else
	memcpy(prearm_check_failure_description, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_PREARM_CHECK_FAILURE_DESCRIPTION_LEN);
#endif
}
