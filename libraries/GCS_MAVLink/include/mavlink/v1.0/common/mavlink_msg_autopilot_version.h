// MESSAGE AUTOPILOT_VERSION PACKING

#define MAVLINK_MSG_ID_AUTOPILOT_VERSION 148

typedef struct __mavlink_autopilot_version_t
{
 uint64_t capabilities; ///< bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum)
 uint32_t version; ///< Firmware version number
 uint8_t custom_version[8]; ///< Custom version field, commonly the first 8 bytes (16 characters) of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
} mavlink_autopilot_version_t;

#define MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN 20
#define MAVLINK_MSG_ID_148_LEN 20

#define MAVLINK_MSG_ID_AUTOPILOT_VERSION_CRC 49
#define MAVLINK_MSG_ID_148_CRC 49

#define MAVLINK_MSG_AUTOPILOT_VERSION_FIELD_CUSTOM_VERSION_LEN 8

#define MAVLINK_MESSAGE_INFO_AUTOPILOT_VERSION { \
	"AUTOPILOT_VERSION", \
	3, \
	{  { "capabilities", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_autopilot_version_t, capabilities) }, \
         { "version", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_autopilot_version_t, version) }, \
         { "custom_version", NULL, MAVLINK_TYPE_UINT8_T, 8, 12, offsetof(mavlink_autopilot_version_t, custom_version) }, \
         } \
}


/**
 * @brief Pack a autopilot_version message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param capabilities bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum)
 * @param version Firmware version number
 * @param custom_version Custom version field, commonly the first 8 bytes (16 characters) of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_autopilot_version_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t capabilities, uint32_t version, const uint8_t *custom_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN];
	_mav_put_uint64_t(buf, 0, capabilities);
	_mav_put_uint32_t(buf, 8, version);
	_mav_put_uint8_t_array(buf, 12, custom_version, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN);
#else
	mavlink_autopilot_version_t packet;
	packet.capabilities = capabilities;
	packet.version = version;
	mav_array_memcpy(packet.custom_version, custom_version, sizeof(uint8_t)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_AUTOPILOT_VERSION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN, MAVLINK_MSG_ID_AUTOPILOT_VERSION_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN);
#endif
}

/**
 * @brief Pack a autopilot_version message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param capabilities bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum)
 * @param version Firmware version number
 * @param custom_version Custom version field, commonly the first 8 bytes (16 characters) of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_autopilot_version_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t capabilities,uint32_t version,const uint8_t *custom_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN];
	_mav_put_uint64_t(buf, 0, capabilities);
	_mav_put_uint32_t(buf, 8, version);
	_mav_put_uint8_t_array(buf, 12, custom_version, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN);
#else
	mavlink_autopilot_version_t packet;
	packet.capabilities = capabilities;
	packet.version = version;
	mav_array_memcpy(packet.custom_version, custom_version, sizeof(uint8_t)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_AUTOPILOT_VERSION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN, MAVLINK_MSG_ID_AUTOPILOT_VERSION_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN);
#endif
}

/**
 * @brief Encode a autopilot_version struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param autopilot_version C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_autopilot_version_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_autopilot_version_t* autopilot_version)
{
	return mavlink_msg_autopilot_version_pack(system_id, component_id, msg, autopilot_version->capabilities, autopilot_version->version, autopilot_version->custom_version);
}

/**
 * @brief Encode a autopilot_version struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param autopilot_version C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_autopilot_version_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_autopilot_version_t* autopilot_version)
{
	return mavlink_msg_autopilot_version_pack_chan(system_id, component_id, chan, msg, autopilot_version->capabilities, autopilot_version->version, autopilot_version->custom_version);
}

/**
 * @brief Send a autopilot_version message
 * @param chan MAVLink channel to send the message
 *
 * @param capabilities bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum)
 * @param version Firmware version number
 * @param custom_version Custom version field, commonly the first 8 bytes (16 characters) of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_autopilot_version_send(mavlink_channel_t chan, uint64_t capabilities, uint32_t version, const uint8_t *custom_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN];
	_mav_put_uint64_t(buf, 0, capabilities);
	_mav_put_uint32_t(buf, 8, version);
	_mav_put_uint8_t_array(buf, 12, custom_version, 8);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTOPILOT_VERSION, buf, MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN, MAVLINK_MSG_ID_AUTOPILOT_VERSION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTOPILOT_VERSION, buf, MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN);
#endif
#else
	mavlink_autopilot_version_t packet;
	packet.capabilities = capabilities;
	packet.version = version;
	mav_array_memcpy(packet.custom_version, custom_version, sizeof(uint8_t)*8);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTOPILOT_VERSION, (const char *)&packet, MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN, MAVLINK_MSG_ID_AUTOPILOT_VERSION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTOPILOT_VERSION, (const char *)&packet, MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_autopilot_version_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t capabilities, uint32_t version, const uint8_t *custom_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, capabilities);
	_mav_put_uint32_t(buf, 8, version);
	_mav_put_uint8_t_array(buf, 12, custom_version, 8);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTOPILOT_VERSION, buf, MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN, MAVLINK_MSG_ID_AUTOPILOT_VERSION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTOPILOT_VERSION, buf, MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN);
#endif
#else
	mavlink_autopilot_version_t *packet = (mavlink_autopilot_version_t *)msgbuf;
	packet->capabilities = capabilities;
	packet->version = version;
	mav_array_memcpy(packet->custom_version, custom_version, sizeof(uint8_t)*8);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTOPILOT_VERSION, (const char *)packet, MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN, MAVLINK_MSG_ID_AUTOPILOT_VERSION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTOPILOT_VERSION, (const char *)packet, MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE AUTOPILOT_VERSION UNPACKING


/**
 * @brief Get field capabilities from autopilot_version message
 *
 * @return bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum)
 */
static inline uint64_t mavlink_msg_autopilot_version_get_capabilities(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field version from autopilot_version message
 *
 * @return Firmware version number
 */
static inline uint32_t mavlink_msg_autopilot_version_get_version(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field custom_version from autopilot_version message
 *
 * @return Custom version field, commonly the first 8 bytes (16 characters) of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
 */
static inline uint16_t mavlink_msg_autopilot_version_get_custom_version(const mavlink_message_t* msg, uint8_t *custom_version)
{
	return _MAV_RETURN_uint8_t_array(msg, custom_version, 8,  12);
}

/**
 * @brief Decode a autopilot_version message into a struct
 *
 * @param msg The message to decode
 * @param autopilot_version C-struct to decode the message contents into
 */
static inline void mavlink_msg_autopilot_version_decode(const mavlink_message_t* msg, mavlink_autopilot_version_t* autopilot_version)
{
#if MAVLINK_NEED_BYTE_SWAP
	autopilot_version->capabilities = mavlink_msg_autopilot_version_get_capabilities(msg);
	autopilot_version->version = mavlink_msg_autopilot_version_get_version(msg);
	mavlink_msg_autopilot_version_get_custom_version(msg, autopilot_version->custom_version);
#else
	memcpy(autopilot_version, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN);
#endif
}
