// MESSAGE DATA_TRANSMISSION_HANDSHAKE PACKING

#define MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE 130

typedef struct __mavlink_data_transmission_handshake_t
{
 uint32_t size; ///< total data size in bytes (set on ACK only)
 uint16_t width; ///< Width of a matrix or image
 uint16_t height; ///< Height of a matrix or image
 uint16_t packets; ///< number of packets beeing sent (set on ACK only)
 uint8_t type; ///< type of requested/acknowledged data (as defined in ENUM DATA_TYPES in mavlink/include/mavlink_types.h)
 uint8_t payload; ///< payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on ACK only)
 uint8_t jpg_quality; ///< JPEG quality out of [1,100]
} mavlink_data_transmission_handshake_t;

#define MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_LEN 13
#define MAVLINK_MSG_ID_130_LEN 13

#define MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_CRC 29
#define MAVLINK_MSG_ID_130_CRC 29



#define MAVLINK_MESSAGE_INFO_DATA_TRANSMISSION_HANDSHAKE { \
	"DATA_TRANSMISSION_HANDSHAKE", \
	7, \
	{  { "size", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_data_transmission_handshake_t, size) }, \
         { "width", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_data_transmission_handshake_t, width) }, \
         { "height", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_data_transmission_handshake_t, height) }, \
         { "packets", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_data_transmission_handshake_t, packets) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_data_transmission_handshake_t, type) }, \
         { "payload", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_data_transmission_handshake_t, payload) }, \
         { "jpg_quality", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_data_transmission_handshake_t, jpg_quality) }, \
         } \
}


/**
 * @brief Pack a data_transmission_handshake message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param type type of requested/acknowledged data (as defined in ENUM DATA_TYPES in mavlink/include/mavlink_types.h)
 * @param size total data size in bytes (set on ACK only)
 * @param width Width of a matrix or image
 * @param height Height of a matrix or image
 * @param packets number of packets beeing sent (set on ACK only)
 * @param payload payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on ACK only)
 * @param jpg_quality JPEG quality out of [1,100]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_data_transmission_handshake_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t type, uint32_t size, uint16_t width, uint16_t height, uint16_t packets, uint8_t payload, uint8_t jpg_quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_LEN];
	_mav_put_uint32_t(buf, 0, size);
	_mav_put_uint16_t(buf, 4, width);
	_mav_put_uint16_t(buf, 6, height);
	_mav_put_uint16_t(buf, 8, packets);
	_mav_put_uint8_t(buf, 10, type);
	_mav_put_uint8_t(buf, 11, payload);
	_mav_put_uint8_t(buf, 12, jpg_quality);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_LEN);
#else
	mavlink_data_transmission_handshake_t packet;
	packet.size = size;
	packet.width = width;
	packet.height = height;
	packet.packets = packets;
	packet.type = type;
	packet.payload = payload;
	packet.jpg_quality = jpg_quality;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_LEN, MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_LEN);
#endif
}

/**
 * @brief Pack a data_transmission_handshake message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param type type of requested/acknowledged data (as defined in ENUM DATA_TYPES in mavlink/include/mavlink_types.h)
 * @param size total data size in bytes (set on ACK only)
 * @param width Width of a matrix or image
 * @param height Height of a matrix or image
 * @param packets number of packets beeing sent (set on ACK only)
 * @param payload payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on ACK only)
 * @param jpg_quality JPEG quality out of [1,100]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_data_transmission_handshake_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t type,uint32_t size,uint16_t width,uint16_t height,uint16_t packets,uint8_t payload,uint8_t jpg_quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_LEN];
	_mav_put_uint32_t(buf, 0, size);
	_mav_put_uint16_t(buf, 4, width);
	_mav_put_uint16_t(buf, 6, height);
	_mav_put_uint16_t(buf, 8, packets);
	_mav_put_uint8_t(buf, 10, type);
	_mav_put_uint8_t(buf, 11, payload);
	_mav_put_uint8_t(buf, 12, jpg_quality);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_LEN);
#else
	mavlink_data_transmission_handshake_t packet;
	packet.size = size;
	packet.width = width;
	packet.height = height;
	packet.packets = packets;
	packet.type = type;
	packet.payload = payload;
	packet.jpg_quality = jpg_quality;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_LEN, MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_LEN);
#endif
}

/**
 * @brief Encode a data_transmission_handshake struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param data_transmission_handshake C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_data_transmission_handshake_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_data_transmission_handshake_t* data_transmission_handshake)
{
	return mavlink_msg_data_transmission_handshake_pack(system_id, component_id, msg, data_transmission_handshake->type, data_transmission_handshake->size, data_transmission_handshake->width, data_transmission_handshake->height, data_transmission_handshake->packets, data_transmission_handshake->payload, data_transmission_handshake->jpg_quality);
}

/**
 * @brief Encode a data_transmission_handshake struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param data_transmission_handshake C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_data_transmission_handshake_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_data_transmission_handshake_t* data_transmission_handshake)
{
	return mavlink_msg_data_transmission_handshake_pack_chan(system_id, component_id, chan, msg, data_transmission_handshake->type, data_transmission_handshake->size, data_transmission_handshake->width, data_transmission_handshake->height, data_transmission_handshake->packets, data_transmission_handshake->payload, data_transmission_handshake->jpg_quality);
}

/**
 * @brief Send a data_transmission_handshake message
 * @param chan MAVLink channel to send the message
 *
 * @param type type of requested/acknowledged data (as defined in ENUM DATA_TYPES in mavlink/include/mavlink_types.h)
 * @param size total data size in bytes (set on ACK only)
 * @param width Width of a matrix or image
 * @param height Height of a matrix or image
 * @param packets number of packets beeing sent (set on ACK only)
 * @param payload payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on ACK only)
 * @param jpg_quality JPEG quality out of [1,100]
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_data_transmission_handshake_send(mavlink_channel_t chan, uint8_t type, uint32_t size, uint16_t width, uint16_t height, uint16_t packets, uint8_t payload, uint8_t jpg_quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_LEN];
	_mav_put_uint32_t(buf, 0, size);
	_mav_put_uint16_t(buf, 4, width);
	_mav_put_uint16_t(buf, 6, height);
	_mav_put_uint16_t(buf, 8, packets);
	_mav_put_uint8_t(buf, 10, type);
	_mav_put_uint8_t(buf, 11, payload);
	_mav_put_uint8_t(buf, 12, jpg_quality);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE, buf, MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_LEN, MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE, buf, MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_LEN);
#endif
#else
	mavlink_data_transmission_handshake_t packet;
	packet.size = size;
	packet.width = width;
	packet.height = height;
	packet.packets = packets;
	packet.type = type;
	packet.payload = payload;
	packet.jpg_quality = jpg_quality;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE, (const char *)&packet, MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_LEN, MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE, (const char *)&packet, MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_data_transmission_handshake_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t type, uint32_t size, uint16_t width, uint16_t height, uint16_t packets, uint8_t payload, uint8_t jpg_quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, size);
	_mav_put_uint16_t(buf, 4, width);
	_mav_put_uint16_t(buf, 6, height);
	_mav_put_uint16_t(buf, 8, packets);
	_mav_put_uint8_t(buf, 10, type);
	_mav_put_uint8_t(buf, 11, payload);
	_mav_put_uint8_t(buf, 12, jpg_quality);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE, buf, MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_LEN, MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE, buf, MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_LEN);
#endif
#else
	mavlink_data_transmission_handshake_t *packet = (mavlink_data_transmission_handshake_t *)msgbuf;
	packet->size = size;
	packet->width = width;
	packet->height = height;
	packet->packets = packets;
	packet->type = type;
	packet->payload = payload;
	packet->jpg_quality = jpg_quality;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE, (const char *)packet, MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_LEN, MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE, (const char *)packet, MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE DATA_TRANSMISSION_HANDSHAKE UNPACKING


/**
 * @brief Get field type from data_transmission_handshake message
 *
 * @return type of requested/acknowledged data (as defined in ENUM DATA_TYPES in mavlink/include/mavlink_types.h)
 */
static inline uint8_t mavlink_msg_data_transmission_handshake_get_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field size from data_transmission_handshake message
 *
 * @return total data size in bytes (set on ACK only)
 */
static inline uint32_t mavlink_msg_data_transmission_handshake_get_size(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field width from data_transmission_handshake message
 *
 * @return Width of a matrix or image
 */
static inline uint16_t mavlink_msg_data_transmission_handshake_get_width(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field height from data_transmission_handshake message
 *
 * @return Height of a matrix or image
 */
static inline uint16_t mavlink_msg_data_transmission_handshake_get_height(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field packets from data_transmission_handshake message
 *
 * @return number of packets beeing sent (set on ACK only)
 */
static inline uint16_t mavlink_msg_data_transmission_handshake_get_packets(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field payload from data_transmission_handshake message
 *
 * @return payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on ACK only)
 */
static inline uint8_t mavlink_msg_data_transmission_handshake_get_payload(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field jpg_quality from data_transmission_handshake message
 *
 * @return JPEG quality out of [1,100]
 */
static inline uint8_t mavlink_msg_data_transmission_handshake_get_jpg_quality(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Decode a data_transmission_handshake message into a struct
 *
 * @param msg The message to decode
 * @param data_transmission_handshake C-struct to decode the message contents into
 */
static inline void mavlink_msg_data_transmission_handshake_decode(const mavlink_message_t* msg, mavlink_data_transmission_handshake_t* data_transmission_handshake)
{
#if MAVLINK_NEED_BYTE_SWAP
	data_transmission_handshake->size = mavlink_msg_data_transmission_handshake_get_size(msg);
	data_transmission_handshake->width = mavlink_msg_data_transmission_handshake_get_width(msg);
	data_transmission_handshake->height = mavlink_msg_data_transmission_handshake_get_height(msg);
	data_transmission_handshake->packets = mavlink_msg_data_transmission_handshake_get_packets(msg);
	data_transmission_handshake->type = mavlink_msg_data_transmission_handshake_get_type(msg);
	data_transmission_handshake->payload = mavlink_msg_data_transmission_handshake_get_payload(msg);
	data_transmission_handshake->jpg_quality = mavlink_msg_data_transmission_handshake_get_jpg_quality(msg);
#else
	memcpy(data_transmission_handshake, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_LEN);
#endif
}
