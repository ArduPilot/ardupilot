// MESSAGE AUTOPILOT_VERSION PACKING

#define MAVLINK_MSG_ID_AUTOPILOT_VERSION 148

typedef struct __mavlink_autopilot_version_t
{
 uint64_t capabilities; /*< bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum)*/
 uint64_t uid; /*< UID if provided by hardware*/
 uint32_t flight_sw_version; /*< Firmware version number*/
 uint32_t middleware_sw_version; /*< Middleware version number*/
 uint32_t os_sw_version; /*< Operating system version number*/
 uint32_t board_version; /*< HW / board version (last 8 bytes should be silicon ID, if any)*/
 uint16_t vendor_id; /*< ID of the board vendor*/
 uint16_t product_id; /*< ID of the product*/
 uint8_t flight_custom_version[8]; /*< Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.*/
 uint8_t middleware_custom_version[8]; /*< Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.*/
 uint8_t os_custom_version[8]; /*< Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.*/
} mavlink_autopilot_version_t;

#define MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN 60
#define MAVLINK_MSG_ID_148_LEN 60

#define MAVLINK_MSG_ID_AUTOPILOT_VERSION_CRC 178
#define MAVLINK_MSG_ID_148_CRC 178

#define MAVLINK_MSG_AUTOPILOT_VERSION_FIELD_FLIGHT_CUSTOM_VERSION_LEN 8
#define MAVLINK_MSG_AUTOPILOT_VERSION_FIELD_MIDDLEWARE_CUSTOM_VERSION_LEN 8
#define MAVLINK_MSG_AUTOPILOT_VERSION_FIELD_OS_CUSTOM_VERSION_LEN 8

#define MAVLINK_MESSAGE_INFO_AUTOPILOT_VERSION { \
	"AUTOPILOT_VERSION", \
	11, \
	{  { "capabilities", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_autopilot_version_t, capabilities) }, \
         { "uid", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_autopilot_version_t, uid) }, \
         { "flight_sw_version", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_autopilot_version_t, flight_sw_version) }, \
         { "middleware_sw_version", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_autopilot_version_t, middleware_sw_version) }, \
         { "os_sw_version", NULL, MAVLINK_TYPE_UINT32_T, 0, 24, offsetof(mavlink_autopilot_version_t, os_sw_version) }, \
         { "board_version", NULL, MAVLINK_TYPE_UINT32_T, 0, 28, offsetof(mavlink_autopilot_version_t, board_version) }, \
         { "vendor_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_autopilot_version_t, vendor_id) }, \
         { "product_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 34, offsetof(mavlink_autopilot_version_t, product_id) }, \
         { "flight_custom_version", NULL, MAVLINK_TYPE_UINT8_T, 8, 36, offsetof(mavlink_autopilot_version_t, flight_custom_version) }, \
         { "middleware_custom_version", NULL, MAVLINK_TYPE_UINT8_T, 8, 44, offsetof(mavlink_autopilot_version_t, middleware_custom_version) }, \
         { "os_custom_version", NULL, MAVLINK_TYPE_UINT8_T, 8, 52, offsetof(mavlink_autopilot_version_t, os_custom_version) }, \
         } \
}


/**
 * @brief Pack a autopilot_version message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param capabilities bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum)
 * @param flight_sw_version Firmware version number
 * @param middleware_sw_version Middleware version number
 * @param os_sw_version Operating system version number
 * @param board_version HW / board version (last 8 bytes should be silicon ID, if any)
 * @param flight_custom_version Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
 * @param middleware_custom_version Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
 * @param os_custom_version Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
 * @param vendor_id ID of the board vendor
 * @param product_id ID of the product
 * @param uid UID if provided by hardware
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_autopilot_version_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t capabilities, uint32_t flight_sw_version, uint32_t middleware_sw_version, uint32_t os_sw_version, uint32_t board_version, const uint8_t *flight_custom_version, const uint8_t *middleware_custom_version, const uint8_t *os_custom_version, uint16_t vendor_id, uint16_t product_id, uint64_t uid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN];
	_mav_put_uint64_t(buf, 0, capabilities);
	_mav_put_uint64_t(buf, 8, uid);
	_mav_put_uint32_t(buf, 16, flight_sw_version);
	_mav_put_uint32_t(buf, 20, middleware_sw_version);
	_mav_put_uint32_t(buf, 24, os_sw_version);
	_mav_put_uint32_t(buf, 28, board_version);
	_mav_put_uint16_t(buf, 32, vendor_id);
	_mav_put_uint16_t(buf, 34, product_id);
	_mav_put_uint8_t_array(buf, 36, flight_custom_version, 8);
	_mav_put_uint8_t_array(buf, 44, middleware_custom_version, 8);
	_mav_put_uint8_t_array(buf, 52, os_custom_version, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN);
#else
	mavlink_autopilot_version_t packet;
	packet.capabilities = capabilities;
	packet.uid = uid;
	packet.flight_sw_version = flight_sw_version;
	packet.middleware_sw_version = middleware_sw_version;
	packet.os_sw_version = os_sw_version;
	packet.board_version = board_version;
	packet.vendor_id = vendor_id;
	packet.product_id = product_id;
	mav_array_memcpy(packet.flight_custom_version, flight_custom_version, sizeof(uint8_t)*8);
	mav_array_memcpy(packet.middleware_custom_version, middleware_custom_version, sizeof(uint8_t)*8);
	mav_array_memcpy(packet.os_custom_version, os_custom_version, sizeof(uint8_t)*8);
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
 * @param flight_sw_version Firmware version number
 * @param middleware_sw_version Middleware version number
 * @param os_sw_version Operating system version number
 * @param board_version HW / board version (last 8 bytes should be silicon ID, if any)
 * @param flight_custom_version Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
 * @param middleware_custom_version Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
 * @param os_custom_version Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
 * @param vendor_id ID of the board vendor
 * @param product_id ID of the product
 * @param uid UID if provided by hardware
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_autopilot_version_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t capabilities,uint32_t flight_sw_version,uint32_t middleware_sw_version,uint32_t os_sw_version,uint32_t board_version,const uint8_t *flight_custom_version,const uint8_t *middleware_custom_version,const uint8_t *os_custom_version,uint16_t vendor_id,uint16_t product_id,uint64_t uid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN];
	_mav_put_uint64_t(buf, 0, capabilities);
	_mav_put_uint64_t(buf, 8, uid);
	_mav_put_uint32_t(buf, 16, flight_sw_version);
	_mav_put_uint32_t(buf, 20, middleware_sw_version);
	_mav_put_uint32_t(buf, 24, os_sw_version);
	_mav_put_uint32_t(buf, 28, board_version);
	_mav_put_uint16_t(buf, 32, vendor_id);
	_mav_put_uint16_t(buf, 34, product_id);
	_mav_put_uint8_t_array(buf, 36, flight_custom_version, 8);
	_mav_put_uint8_t_array(buf, 44, middleware_custom_version, 8);
	_mav_put_uint8_t_array(buf, 52, os_custom_version, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN);
#else
	mavlink_autopilot_version_t packet;
	packet.capabilities = capabilities;
	packet.uid = uid;
	packet.flight_sw_version = flight_sw_version;
	packet.middleware_sw_version = middleware_sw_version;
	packet.os_sw_version = os_sw_version;
	packet.board_version = board_version;
	packet.vendor_id = vendor_id;
	packet.product_id = product_id;
	mav_array_memcpy(packet.flight_custom_version, flight_custom_version, sizeof(uint8_t)*8);
	mav_array_memcpy(packet.middleware_custom_version, middleware_custom_version, sizeof(uint8_t)*8);
	mav_array_memcpy(packet.os_custom_version, os_custom_version, sizeof(uint8_t)*8);
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
	return mavlink_msg_autopilot_version_pack(system_id, component_id, msg, autopilot_version->capabilities, autopilot_version->flight_sw_version, autopilot_version->middleware_sw_version, autopilot_version->os_sw_version, autopilot_version->board_version, autopilot_version->flight_custom_version, autopilot_version->middleware_custom_version, autopilot_version->os_custom_version, autopilot_version->vendor_id, autopilot_version->product_id, autopilot_version->uid);
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
	return mavlink_msg_autopilot_version_pack_chan(system_id, component_id, chan, msg, autopilot_version->capabilities, autopilot_version->flight_sw_version, autopilot_version->middleware_sw_version, autopilot_version->os_sw_version, autopilot_version->board_version, autopilot_version->flight_custom_version, autopilot_version->middleware_custom_version, autopilot_version->os_custom_version, autopilot_version->vendor_id, autopilot_version->product_id, autopilot_version->uid);
}

/**
 * @brief Send a autopilot_version message
 * @param chan MAVLink channel to send the message
 *
 * @param capabilities bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum)
 * @param flight_sw_version Firmware version number
 * @param middleware_sw_version Middleware version number
 * @param os_sw_version Operating system version number
 * @param board_version HW / board version (last 8 bytes should be silicon ID, if any)
 * @param flight_custom_version Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
 * @param middleware_custom_version Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
 * @param os_custom_version Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
 * @param vendor_id ID of the board vendor
 * @param product_id ID of the product
 * @param uid UID if provided by hardware
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_autopilot_version_send(mavlink_channel_t chan, uint64_t capabilities, uint32_t flight_sw_version, uint32_t middleware_sw_version, uint32_t os_sw_version, uint32_t board_version, const uint8_t *flight_custom_version, const uint8_t *middleware_custom_version, const uint8_t *os_custom_version, uint16_t vendor_id, uint16_t product_id, uint64_t uid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN];
	_mav_put_uint64_t(buf, 0, capabilities);
	_mav_put_uint64_t(buf, 8, uid);
	_mav_put_uint32_t(buf, 16, flight_sw_version);
	_mav_put_uint32_t(buf, 20, middleware_sw_version);
	_mav_put_uint32_t(buf, 24, os_sw_version);
	_mav_put_uint32_t(buf, 28, board_version);
	_mav_put_uint16_t(buf, 32, vendor_id);
	_mav_put_uint16_t(buf, 34, product_id);
	_mav_put_uint8_t_array(buf, 36, flight_custom_version, 8);
	_mav_put_uint8_t_array(buf, 44, middleware_custom_version, 8);
	_mav_put_uint8_t_array(buf, 52, os_custom_version, 8);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTOPILOT_VERSION, buf, MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN, MAVLINK_MSG_ID_AUTOPILOT_VERSION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTOPILOT_VERSION, buf, MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN);
#endif
#else
	mavlink_autopilot_version_t packet;
	packet.capabilities = capabilities;
	packet.uid = uid;
	packet.flight_sw_version = flight_sw_version;
	packet.middleware_sw_version = middleware_sw_version;
	packet.os_sw_version = os_sw_version;
	packet.board_version = board_version;
	packet.vendor_id = vendor_id;
	packet.product_id = product_id;
	mav_array_memcpy(packet.flight_custom_version, flight_custom_version, sizeof(uint8_t)*8);
	mav_array_memcpy(packet.middleware_custom_version, middleware_custom_version, sizeof(uint8_t)*8);
	mav_array_memcpy(packet.os_custom_version, os_custom_version, sizeof(uint8_t)*8);
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
static inline void mavlink_msg_autopilot_version_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t capabilities, uint32_t flight_sw_version, uint32_t middleware_sw_version, uint32_t os_sw_version, uint32_t board_version, const uint8_t *flight_custom_version, const uint8_t *middleware_custom_version, const uint8_t *os_custom_version, uint16_t vendor_id, uint16_t product_id, uint64_t uid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, capabilities);
	_mav_put_uint64_t(buf, 8, uid);
	_mav_put_uint32_t(buf, 16, flight_sw_version);
	_mav_put_uint32_t(buf, 20, middleware_sw_version);
	_mav_put_uint32_t(buf, 24, os_sw_version);
	_mav_put_uint32_t(buf, 28, board_version);
	_mav_put_uint16_t(buf, 32, vendor_id);
	_mav_put_uint16_t(buf, 34, product_id);
	_mav_put_uint8_t_array(buf, 36, flight_custom_version, 8);
	_mav_put_uint8_t_array(buf, 44, middleware_custom_version, 8);
	_mav_put_uint8_t_array(buf, 52, os_custom_version, 8);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTOPILOT_VERSION, buf, MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN, MAVLINK_MSG_ID_AUTOPILOT_VERSION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTOPILOT_VERSION, buf, MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN);
#endif
#else
	mavlink_autopilot_version_t *packet = (mavlink_autopilot_version_t *)msgbuf;
	packet->capabilities = capabilities;
	packet->uid = uid;
	packet->flight_sw_version = flight_sw_version;
	packet->middleware_sw_version = middleware_sw_version;
	packet->os_sw_version = os_sw_version;
	packet->board_version = board_version;
	packet->vendor_id = vendor_id;
	packet->product_id = product_id;
	mav_array_memcpy(packet->flight_custom_version, flight_custom_version, sizeof(uint8_t)*8);
	mav_array_memcpy(packet->middleware_custom_version, middleware_custom_version, sizeof(uint8_t)*8);
	mav_array_memcpy(packet->os_custom_version, os_custom_version, sizeof(uint8_t)*8);
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
 * @brief Get field flight_sw_version from autopilot_version message
 *
 * @return Firmware version number
 */
static inline uint32_t mavlink_msg_autopilot_version_get_flight_sw_version(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  16);
}

/**
 * @brief Get field middleware_sw_version from autopilot_version message
 *
 * @return Middleware version number
 */
static inline uint32_t mavlink_msg_autopilot_version_get_middleware_sw_version(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  20);
}

/**
 * @brief Get field os_sw_version from autopilot_version message
 *
 * @return Operating system version number
 */
static inline uint32_t mavlink_msg_autopilot_version_get_os_sw_version(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  24);
}

/**
 * @brief Get field board_version from autopilot_version message
 *
 * @return HW / board version (last 8 bytes should be silicon ID, if any)
 */
static inline uint32_t mavlink_msg_autopilot_version_get_board_version(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  28);
}

/**
 * @brief Get field flight_custom_version from autopilot_version message
 *
 * @return Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
 */
static inline uint16_t mavlink_msg_autopilot_version_get_flight_custom_version(const mavlink_message_t* msg, uint8_t *flight_custom_version)
{
	return _MAV_RETURN_uint8_t_array(msg, flight_custom_version, 8,  36);
}

/**
 * @brief Get field middleware_custom_version from autopilot_version message
 *
 * @return Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
 */
static inline uint16_t mavlink_msg_autopilot_version_get_middleware_custom_version(const mavlink_message_t* msg, uint8_t *middleware_custom_version)
{
	return _MAV_RETURN_uint8_t_array(msg, middleware_custom_version, 8,  44);
}

/**
 * @brief Get field os_custom_version from autopilot_version message
 *
 * @return Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
 */
static inline uint16_t mavlink_msg_autopilot_version_get_os_custom_version(const mavlink_message_t* msg, uint8_t *os_custom_version)
{
	return _MAV_RETURN_uint8_t_array(msg, os_custom_version, 8,  52);
}

/**
 * @brief Get field vendor_id from autopilot_version message
 *
 * @return ID of the board vendor
 */
static inline uint16_t mavlink_msg_autopilot_version_get_vendor_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  32);
}

/**
 * @brief Get field product_id from autopilot_version message
 *
 * @return ID of the product
 */
static inline uint16_t mavlink_msg_autopilot_version_get_product_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  34);
}

/**
 * @brief Get field uid from autopilot_version message
 *
 * @return UID if provided by hardware
 */
static inline uint64_t mavlink_msg_autopilot_version_get_uid(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  8);
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
	autopilot_version->uid = mavlink_msg_autopilot_version_get_uid(msg);
	autopilot_version->flight_sw_version = mavlink_msg_autopilot_version_get_flight_sw_version(msg);
	autopilot_version->middleware_sw_version = mavlink_msg_autopilot_version_get_middleware_sw_version(msg);
	autopilot_version->os_sw_version = mavlink_msg_autopilot_version_get_os_sw_version(msg);
	autopilot_version->board_version = mavlink_msg_autopilot_version_get_board_version(msg);
	autopilot_version->vendor_id = mavlink_msg_autopilot_version_get_vendor_id(msg);
	autopilot_version->product_id = mavlink_msg_autopilot_version_get_product_id(msg);
	mavlink_msg_autopilot_version_get_flight_custom_version(msg, autopilot_version->flight_custom_version);
	mavlink_msg_autopilot_version_get_middleware_custom_version(msg, autopilot_version->middleware_custom_version);
	mavlink_msg_autopilot_version_get_os_custom_version(msg, autopilot_version->os_custom_version);
#else
	memcpy(autopilot_version, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN);
#endif
}
