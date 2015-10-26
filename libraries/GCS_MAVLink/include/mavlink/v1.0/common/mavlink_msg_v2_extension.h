// MESSAGE V2_EXTENSION PACKING

#define MAVLINK_MSG_ID_V2_EXTENSION 248

typedef struct __mavlink_v2_extension_t
{
 uint16_t message_type; ///< A code that identifies the software component that understands this message (analogous to usb device classes or mime type strings).  If this code is less than 32768, it is considered a 'registered' protocol extension and the corresponding entry should be added to https://github.com/mavlink/mavlink/extension-message-ids.xml.  Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...). Message_types greater than 32767 are considered local experiments and should not be checked in to any widely distributed codebase.
 uint8_t target_network; ///< Network ID (0 for broadcast)
 uint8_t target_system; ///< System ID (0 for broadcast)
 uint8_t target_component; ///< Component ID (0 for broadcast)
 uint8_t payload[249]; ///< Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields.  The entire content of this block is opaque unless you understand any the encoding message_type.  The particular encoding used can be extension specific and might not always be documented as part of the mavlink specification.
} mavlink_v2_extension_t;

#define MAVLINK_MSG_ID_V2_EXTENSION_LEN 254
#define MAVLINK_MSG_ID_248_LEN 254

#define MAVLINK_MSG_ID_V2_EXTENSION_CRC 8
#define MAVLINK_MSG_ID_248_CRC 8

#define MAVLINK_MSG_V2_EXTENSION_FIELD_PAYLOAD_LEN 249

#define MAVLINK_MESSAGE_INFO_V2_EXTENSION { \
	"V2_EXTENSION", \
	5, \
	{  { "message_type", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_v2_extension_t, message_type) }, \
         { "target_network", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_v2_extension_t, target_network) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_v2_extension_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_v2_extension_t, target_component) }, \
         { "payload", NULL, MAVLINK_TYPE_UINT8_T, 249, 5, offsetof(mavlink_v2_extension_t, payload) }, \
         } \
}


/**
 * @brief Pack a v2_extension message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_network Network ID (0 for broadcast)
 * @param target_system System ID (0 for broadcast)
 * @param target_component Component ID (0 for broadcast)
 * @param message_type A code that identifies the software component that understands this message (analogous to usb device classes or mime type strings).  If this code is less than 32768, it is considered a 'registered' protocol extension and the corresponding entry should be added to https://github.com/mavlink/mavlink/extension-message-ids.xml.  Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...). Message_types greater than 32767 are considered local experiments and should not be checked in to any widely distributed codebase.
 * @param payload Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields.  The entire content of this block is opaque unless you understand any the encoding message_type.  The particular encoding used can be extension specific and might not always be documented as part of the mavlink specification.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_v2_extension_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_network, uint8_t target_system, uint8_t target_component, uint16_t message_type, const uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_V2_EXTENSION_LEN];
	_mav_put_uint16_t(buf, 0, message_type);
	_mav_put_uint8_t(buf, 2, target_network);
	_mav_put_uint8_t(buf, 3, target_system);
	_mav_put_uint8_t(buf, 4, target_component);
	_mav_put_uint8_t_array(buf, 5, payload, 249);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_V2_EXTENSION_LEN);
#else
	mavlink_v2_extension_t packet;
	packet.message_type = message_type;
	packet.target_network = target_network;
	packet.target_system = target_system;
	packet.target_component = target_component;
	mav_array_memcpy(packet.payload, payload, sizeof(uint8_t)*249);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_V2_EXTENSION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_V2_EXTENSION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_V2_EXTENSION_LEN, MAVLINK_MSG_ID_V2_EXTENSION_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_V2_EXTENSION_LEN);
#endif
}

/**
 * @brief Pack a v2_extension message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_network Network ID (0 for broadcast)
 * @param target_system System ID (0 for broadcast)
 * @param target_component Component ID (0 for broadcast)
 * @param message_type A code that identifies the software component that understands this message (analogous to usb device classes or mime type strings).  If this code is less than 32768, it is considered a 'registered' protocol extension and the corresponding entry should be added to https://github.com/mavlink/mavlink/extension-message-ids.xml.  Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...). Message_types greater than 32767 are considered local experiments and should not be checked in to any widely distributed codebase.
 * @param payload Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields.  The entire content of this block is opaque unless you understand any the encoding message_type.  The particular encoding used can be extension specific and might not always be documented as part of the mavlink specification.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_v2_extension_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_network,uint8_t target_system,uint8_t target_component,uint16_t message_type,const uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_V2_EXTENSION_LEN];
	_mav_put_uint16_t(buf, 0, message_type);
	_mav_put_uint8_t(buf, 2, target_network);
	_mav_put_uint8_t(buf, 3, target_system);
	_mav_put_uint8_t(buf, 4, target_component);
	_mav_put_uint8_t_array(buf, 5, payload, 249);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_V2_EXTENSION_LEN);
#else
	mavlink_v2_extension_t packet;
	packet.message_type = message_type;
	packet.target_network = target_network;
	packet.target_system = target_system;
	packet.target_component = target_component;
	mav_array_memcpy(packet.payload, payload, sizeof(uint8_t)*249);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_V2_EXTENSION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_V2_EXTENSION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_V2_EXTENSION_LEN, MAVLINK_MSG_ID_V2_EXTENSION_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_V2_EXTENSION_LEN);
#endif
}

/**
 * @brief Encode a v2_extension struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param v2_extension C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_v2_extension_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_v2_extension_t* v2_extension)
{
	return mavlink_msg_v2_extension_pack(system_id, component_id, msg, v2_extension->target_network, v2_extension->target_system, v2_extension->target_component, v2_extension->message_type, v2_extension->payload);
}

/**
 * @brief Encode a v2_extension struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param v2_extension C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_v2_extension_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_v2_extension_t* v2_extension)
{
	return mavlink_msg_v2_extension_pack_chan(system_id, component_id, chan, msg, v2_extension->target_network, v2_extension->target_system, v2_extension->target_component, v2_extension->message_type, v2_extension->payload);
}

/**
 * @brief Send a v2_extension message
 * @param chan MAVLink channel to send the message
 *
 * @param target_network Network ID (0 for broadcast)
 * @param target_system System ID (0 for broadcast)
 * @param target_component Component ID (0 for broadcast)
 * @param message_type A code that identifies the software component that understands this message (analogous to usb device classes or mime type strings).  If this code is less than 32768, it is considered a 'registered' protocol extension and the corresponding entry should be added to https://github.com/mavlink/mavlink/extension-message-ids.xml.  Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...). Message_types greater than 32767 are considered local experiments and should not be checked in to any widely distributed codebase.
 * @param payload Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields.  The entire content of this block is opaque unless you understand any the encoding message_type.  The particular encoding used can be extension specific and might not always be documented as part of the mavlink specification.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_v2_extension_send(mavlink_channel_t chan, uint8_t target_network, uint8_t target_system, uint8_t target_component, uint16_t message_type, const uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_V2_EXTENSION_LEN];
	_mav_put_uint16_t(buf, 0, message_type);
	_mav_put_uint8_t(buf, 2, target_network);
	_mav_put_uint8_t(buf, 3, target_system);
	_mav_put_uint8_t(buf, 4, target_component);
	_mav_put_uint8_t_array(buf, 5, payload, 249);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_V2_EXTENSION, buf, MAVLINK_MSG_ID_V2_EXTENSION_LEN, MAVLINK_MSG_ID_V2_EXTENSION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_V2_EXTENSION, buf, MAVLINK_MSG_ID_V2_EXTENSION_LEN);
#endif
#else
	mavlink_v2_extension_t packet;
	packet.message_type = message_type;
	packet.target_network = target_network;
	packet.target_system = target_system;
	packet.target_component = target_component;
	mav_array_memcpy(packet.payload, payload, sizeof(uint8_t)*249);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_V2_EXTENSION, (const char *)&packet, MAVLINK_MSG_ID_V2_EXTENSION_LEN, MAVLINK_MSG_ID_V2_EXTENSION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_V2_EXTENSION, (const char *)&packet, MAVLINK_MSG_ID_V2_EXTENSION_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_V2_EXTENSION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_v2_extension_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_network, uint8_t target_system, uint8_t target_component, uint16_t message_type, const uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint16_t(buf, 0, message_type);
	_mav_put_uint8_t(buf, 2, target_network);
	_mav_put_uint8_t(buf, 3, target_system);
	_mav_put_uint8_t(buf, 4, target_component);
	_mav_put_uint8_t_array(buf, 5, payload, 249);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_V2_EXTENSION, buf, MAVLINK_MSG_ID_V2_EXTENSION_LEN, MAVLINK_MSG_ID_V2_EXTENSION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_V2_EXTENSION, buf, MAVLINK_MSG_ID_V2_EXTENSION_LEN);
#endif
#else
	mavlink_v2_extension_t *packet = (mavlink_v2_extension_t *)msgbuf;
	packet->message_type = message_type;
	packet->target_network = target_network;
	packet->target_system = target_system;
	packet->target_component = target_component;
	mav_array_memcpy(packet->payload, payload, sizeof(uint8_t)*249);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_V2_EXTENSION, (const char *)packet, MAVLINK_MSG_ID_V2_EXTENSION_LEN, MAVLINK_MSG_ID_V2_EXTENSION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_V2_EXTENSION, (const char *)packet, MAVLINK_MSG_ID_V2_EXTENSION_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE V2_EXTENSION UNPACKING


/**
 * @brief Get field target_network from v2_extension message
 *
 * @return Network ID (0 for broadcast)
 */
static inline uint8_t mavlink_msg_v2_extension_get_target_network(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field target_system from v2_extension message
 *
 * @return System ID (0 for broadcast)
 */
static inline uint8_t mavlink_msg_v2_extension_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field target_component from v2_extension message
 *
 * @return Component ID (0 for broadcast)
 */
static inline uint8_t mavlink_msg_v2_extension_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field message_type from v2_extension message
 *
 * @return A code that identifies the software component that understands this message (analogous to usb device classes or mime type strings).  If this code is less than 32768, it is considered a 'registered' protocol extension and the corresponding entry should be added to https://github.com/mavlink/mavlink/extension-message-ids.xml.  Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...). Message_types greater than 32767 are considered local experiments and should not be checked in to any widely distributed codebase.
 */
static inline uint16_t mavlink_msg_v2_extension_get_message_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field payload from v2_extension message
 *
 * @return Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields.  The entire content of this block is opaque unless you understand any the encoding message_type.  The particular encoding used can be extension specific and might not always be documented as part of the mavlink specification.
 */
static inline uint16_t mavlink_msg_v2_extension_get_payload(const mavlink_message_t* msg, uint8_t *payload)
{
	return _MAV_RETURN_uint8_t_array(msg, payload, 249,  5);
}

/**
 * @brief Decode a v2_extension message into a struct
 *
 * @param msg The message to decode
 * @param v2_extension C-struct to decode the message contents into
 */
static inline void mavlink_msg_v2_extension_decode(const mavlink_message_t* msg, mavlink_v2_extension_t* v2_extension)
{
#if MAVLINK_NEED_BYTE_SWAP
	v2_extension->message_type = mavlink_msg_v2_extension_get_message_type(msg);
	v2_extension->target_network = mavlink_msg_v2_extension_get_target_network(msg);
	v2_extension->target_system = mavlink_msg_v2_extension_get_target_system(msg);
	v2_extension->target_component = mavlink_msg_v2_extension_get_target_component(msg);
	mavlink_msg_v2_extension_get_payload(msg, v2_extension->payload);
#else
	memcpy(v2_extension, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_V2_EXTENSION_LEN);
#endif
}
