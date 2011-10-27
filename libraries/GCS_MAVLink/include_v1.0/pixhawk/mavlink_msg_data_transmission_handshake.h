// MESSAGE DATA_TRANSMISSION_HANDSHAKE PACKING

#define MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE 193

typedef struct __mavlink_data_transmission_handshake_t
{
 uint32_t size; ///< total data size in bytes (set on ACK only)
 uint8_t type; ///< type of requested/acknowledged data (as defined in ENUM DATA_TYPES in mavlink/include/mavlink_types.h)
 uint8_t packets; ///< number of packets beeing sent (set on ACK only)
 uint8_t payload; ///< payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on ACK only)
 uint8_t jpg_quality; ///< JPEG quality out of [1,100]
} mavlink_data_transmission_handshake_t;

#define MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_LEN 8
#define MAVLINK_MSG_ID_193_LEN 8



#define MAVLINK_MESSAGE_INFO_DATA_TRANSMISSION_HANDSHAKE { \
	"DATA_TRANSMISSION_HANDSHAKE", \
	5, \
	{  { "size", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_data_transmission_handshake_t, size) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_data_transmission_handshake_t, type) }, \
         { "packets", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_data_transmission_handshake_t, packets) }, \
         { "payload", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_data_transmission_handshake_t, payload) }, \
         { "jpg_quality", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_data_transmission_handshake_t, jpg_quality) }, \
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
 * @param packets number of packets beeing sent (set on ACK only)
 * @param payload payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on ACK only)
 * @param jpg_quality JPEG quality out of [1,100]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_data_transmission_handshake_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t type, uint32_t size, uint8_t packets, uint8_t payload, uint8_t jpg_quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[8];
	_mav_put_uint32_t(buf, 0, size);
	_mav_put_uint8_t(buf, 4, type);
	_mav_put_uint8_t(buf, 5, packets);
	_mav_put_uint8_t(buf, 6, payload);
	_mav_put_uint8_t(buf, 7, jpg_quality);

        memcpy(_MAV_PAYLOAD(msg), buf, 8);
#else
	mavlink_data_transmission_handshake_t packet;
	packet.size = size;
	packet.type = type;
	packet.packets = packets;
	packet.payload = payload;
	packet.jpg_quality = jpg_quality;

        memcpy(_MAV_PAYLOAD(msg), &packet, 8);
#endif

	msg->msgid = MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE;
	return mavlink_finalize_message(msg, system_id, component_id, 8, 148);
}

/**
 * @brief Pack a data_transmission_handshake message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param type type of requested/acknowledged data (as defined in ENUM DATA_TYPES in mavlink/include/mavlink_types.h)
 * @param size total data size in bytes (set on ACK only)
 * @param packets number of packets beeing sent (set on ACK only)
 * @param payload payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on ACK only)
 * @param jpg_quality JPEG quality out of [1,100]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_data_transmission_handshake_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t type,uint32_t size,uint8_t packets,uint8_t payload,uint8_t jpg_quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[8];
	_mav_put_uint32_t(buf, 0, size);
	_mav_put_uint8_t(buf, 4, type);
	_mav_put_uint8_t(buf, 5, packets);
	_mav_put_uint8_t(buf, 6, payload);
	_mav_put_uint8_t(buf, 7, jpg_quality);

        memcpy(_MAV_PAYLOAD(msg), buf, 8);
#else
	mavlink_data_transmission_handshake_t packet;
	packet.size = size;
	packet.type = type;
	packet.packets = packets;
	packet.payload = payload;
	packet.jpg_quality = jpg_quality;

        memcpy(_MAV_PAYLOAD(msg), &packet, 8);
#endif

	msg->msgid = MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 8, 148);
}

/**
 * @brief Encode a data_transmission_handshake struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param data_transmission_handshake C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_data_transmission_handshake_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_data_transmission_handshake_t* data_transmission_handshake)
{
	return mavlink_msg_data_transmission_handshake_pack(system_id, component_id, msg, data_transmission_handshake->type, data_transmission_handshake->size, data_transmission_handshake->packets, data_transmission_handshake->payload, data_transmission_handshake->jpg_quality);
}

/**
 * @brief Send a data_transmission_handshake message
 * @param chan MAVLink channel to send the message
 *
 * @param type type of requested/acknowledged data (as defined in ENUM DATA_TYPES in mavlink/include/mavlink_types.h)
 * @param size total data size in bytes (set on ACK only)
 * @param packets number of packets beeing sent (set on ACK only)
 * @param payload payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on ACK only)
 * @param jpg_quality JPEG quality out of [1,100]
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_data_transmission_handshake_send(mavlink_channel_t chan, uint8_t type, uint32_t size, uint8_t packets, uint8_t payload, uint8_t jpg_quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[8];
	_mav_put_uint32_t(buf, 0, size);
	_mav_put_uint8_t(buf, 4, type);
	_mav_put_uint8_t(buf, 5, packets);
	_mav_put_uint8_t(buf, 6, payload);
	_mav_put_uint8_t(buf, 7, jpg_quality);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE, buf, 8, 148);
#else
	mavlink_data_transmission_handshake_t packet;
	packet.size = size;
	packet.type = type;
	packet.packets = packets;
	packet.payload = payload;
	packet.jpg_quality = jpg_quality;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE, (const char *)&packet, 8, 148);
#endif
}

#endif

// MESSAGE DATA_TRANSMISSION_HANDSHAKE UNPACKING


/**
 * @brief Get field type from data_transmission_handshake message
 *
 * @return type of requested/acknowledged data (as defined in ENUM DATA_TYPES in mavlink/include/mavlink_types.h)
 */
static inline uint8_t mavlink_msg_data_transmission_handshake_get_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
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
 * @brief Get field packets from data_transmission_handshake message
 *
 * @return number of packets beeing sent (set on ACK only)
 */
static inline uint8_t mavlink_msg_data_transmission_handshake_get_packets(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field payload from data_transmission_handshake message
 *
 * @return payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on ACK only)
 */
static inline uint8_t mavlink_msg_data_transmission_handshake_get_payload(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field jpg_quality from data_transmission_handshake message
 *
 * @return JPEG quality out of [1,100]
 */
static inline uint8_t mavlink_msg_data_transmission_handshake_get_jpg_quality(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  7);
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
	data_transmission_handshake->type = mavlink_msg_data_transmission_handshake_get_type(msg);
	data_transmission_handshake->packets = mavlink_msg_data_transmission_handshake_get_packets(msg);
	data_transmission_handshake->payload = mavlink_msg_data_transmission_handshake_get_payload(msg);
	data_transmission_handshake->jpg_quality = mavlink_msg_data_transmission_handshake_get_jpg_quality(msg);
#else
	memcpy(data_transmission_handshake, _MAV_PAYLOAD(msg), 8);
#endif
}
