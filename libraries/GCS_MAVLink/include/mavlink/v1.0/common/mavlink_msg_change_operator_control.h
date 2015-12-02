// MESSAGE CHANGE_OPERATOR_CONTROL PACKING

#define MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL 5

typedef struct __mavlink_change_operator_control_t
{
 uint8_t target_system; /*< System the GCS requests control for*/
 uint8_t control_request; /*< 0: request control of this MAV, 1: Release control of this MAV*/
 uint8_t version; /*< 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch.*/
 char passkey[25]; /*< Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and "!?,.-"*/
} mavlink_change_operator_control_t;

#define MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN 28
#define MAVLINK_MSG_ID_5_LEN 28

#define MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_CRC 217
#define MAVLINK_MSG_ID_5_CRC 217

#define MAVLINK_MSG_CHANGE_OPERATOR_CONTROL_FIELD_PASSKEY_LEN 25

#define MAVLINK_MESSAGE_INFO_CHANGE_OPERATOR_CONTROL { \
	"CHANGE_OPERATOR_CONTROL", \
	4, \
	{  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_change_operator_control_t, target_system) }, \
         { "control_request", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_change_operator_control_t, control_request) }, \
         { "version", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_change_operator_control_t, version) }, \
         { "passkey", NULL, MAVLINK_TYPE_CHAR, 25, 3, offsetof(mavlink_change_operator_control_t, passkey) }, \
         } \
}


/**
 * @brief Pack a change_operator_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System the GCS requests control for
 * @param control_request 0: request control of this MAV, 1: Release control of this MAV
 * @param version 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch.
 * @param passkey Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and "!?,.-"
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_change_operator_control_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t control_request, uint8_t version, const char *passkey)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, control_request);
	_mav_put_uint8_t(buf, 2, version);
	_mav_put_char_array(buf, 3, passkey, 25);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN);
#else
	mavlink_change_operator_control_t packet;
	packet.target_system = target_system;
	packet.control_request = control_request;
	packet.version = version;
	mav_array_memcpy(packet.passkey, passkey, sizeof(char)*25);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN, MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN);
#endif
}

/**
 * @brief Pack a change_operator_control message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System the GCS requests control for
 * @param control_request 0: request control of this MAV, 1: Release control of this MAV
 * @param version 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch.
 * @param passkey Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and "!?,.-"
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_change_operator_control_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t control_request,uint8_t version,const char *passkey)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, control_request);
	_mav_put_uint8_t(buf, 2, version);
	_mav_put_char_array(buf, 3, passkey, 25);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN);
#else
	mavlink_change_operator_control_t packet;
	packet.target_system = target_system;
	packet.control_request = control_request;
	packet.version = version;
	mav_array_memcpy(packet.passkey, passkey, sizeof(char)*25);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN, MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN);
#endif
}

/**
 * @brief Encode a change_operator_control struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param change_operator_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_change_operator_control_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_change_operator_control_t* change_operator_control)
{
	return mavlink_msg_change_operator_control_pack(system_id, component_id, msg, change_operator_control->target_system, change_operator_control->control_request, change_operator_control->version, change_operator_control->passkey);
}

/**
 * @brief Encode a change_operator_control struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param change_operator_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_change_operator_control_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_change_operator_control_t* change_operator_control)
{
	return mavlink_msg_change_operator_control_pack_chan(system_id, component_id, chan, msg, change_operator_control->target_system, change_operator_control->control_request, change_operator_control->version, change_operator_control->passkey);
}

/**
 * @brief Send a change_operator_control message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System the GCS requests control for
 * @param control_request 0: request control of this MAV, 1: Release control of this MAV
 * @param version 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch.
 * @param passkey Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and "!?,.-"
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_change_operator_control_send(mavlink_channel_t chan, uint8_t target_system, uint8_t control_request, uint8_t version, const char *passkey)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, control_request);
	_mav_put_uint8_t(buf, 2, version);
	_mav_put_char_array(buf, 3, passkey, 25);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL, buf, MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN, MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL, buf, MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN);
#endif
#else
	mavlink_change_operator_control_t packet;
	packet.target_system = target_system;
	packet.control_request = control_request;
	packet.version = version;
	mav_array_memcpy(packet.passkey, passkey, sizeof(char)*25);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL, (const char *)&packet, MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN, MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL, (const char *)&packet, MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_change_operator_control_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t control_request, uint8_t version, const char *passkey)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, control_request);
	_mav_put_uint8_t(buf, 2, version);
	_mav_put_char_array(buf, 3, passkey, 25);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL, buf, MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN, MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL, buf, MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN);
#endif
#else
	mavlink_change_operator_control_t *packet = (mavlink_change_operator_control_t *)msgbuf;
	packet->target_system = target_system;
	packet->control_request = control_request;
	packet->version = version;
	mav_array_memcpy(packet->passkey, passkey, sizeof(char)*25);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL, (const char *)packet, MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN, MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL, (const char *)packet, MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE CHANGE_OPERATOR_CONTROL UNPACKING


/**
 * @brief Get field target_system from change_operator_control message
 *
 * @return System the GCS requests control for
 */
static inline uint8_t mavlink_msg_change_operator_control_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field control_request from change_operator_control message
 *
 * @return 0: request control of this MAV, 1: Release control of this MAV
 */
static inline uint8_t mavlink_msg_change_operator_control_get_control_request(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field version from change_operator_control message
 *
 * @return 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch.
 */
static inline uint8_t mavlink_msg_change_operator_control_get_version(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field passkey from change_operator_control message
 *
 * @return Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and "!?,.-"
 */
static inline uint16_t mavlink_msg_change_operator_control_get_passkey(const mavlink_message_t* msg, char *passkey)
{
	return _MAV_RETURN_char_array(msg, passkey, 25,  3);
}

/**
 * @brief Decode a change_operator_control message into a struct
 *
 * @param msg The message to decode
 * @param change_operator_control C-struct to decode the message contents into
 */
static inline void mavlink_msg_change_operator_control_decode(const mavlink_message_t* msg, mavlink_change_operator_control_t* change_operator_control)
{
#if MAVLINK_NEED_BYTE_SWAP
	change_operator_control->target_system = mavlink_msg_change_operator_control_get_target_system(msg);
	change_operator_control->control_request = mavlink_msg_change_operator_control_get_control_request(msg);
	change_operator_control->version = mavlink_msg_change_operator_control_get_version(msg);
	mavlink_msg_change_operator_control_get_passkey(msg, change_operator_control->passkey);
#else
	memcpy(change_operator_control, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN);
#endif
}
