// MESSAGE DATA_TRANSMISSION_HANDSHAKE PACKING

#define MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE 170

typedef struct __mavlink_data_transmission_handshake_t 
{
	uint8_t type; ///< type of requested/acknowledged data (as defined in ENUM DATA_TYPES in mavlink/include/mavlink_types.h)
	uint32_t size; ///< total data size in bytes (set on ACK only)
	uint8_t packets; ///< number of packets beeing sent (set on ACK only)
	uint8_t payload; ///< payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on ACK only)
	uint8_t jpg_quality; ///< JPEG quality out of [1,100]

} mavlink_data_transmission_handshake_t;



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
static inline uint16_t mavlink_msg_data_transmission_handshake_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t type, uint32_t size, uint8_t packets, uint8_t payload, uint8_t jpg_quality)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE;

	i += put_uint8_t_by_index(type, i, msg->payload); // type of requested/acknowledged data (as defined in ENUM DATA_TYPES in mavlink/include/mavlink_types.h)
	i += put_uint32_t_by_index(size, i, msg->payload); // total data size in bytes (set on ACK only)
	i += put_uint8_t_by_index(packets, i, msg->payload); // number of packets beeing sent (set on ACK only)
	i += put_uint8_t_by_index(payload, i, msg->payload); // payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on ACK only)
	i += put_uint8_t_by_index(jpg_quality, i, msg->payload); // JPEG quality out of [1,100]

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a data_transmission_handshake message
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
static inline uint16_t mavlink_msg_data_transmission_handshake_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint8_t type, uint32_t size, uint8_t packets, uint8_t payload, uint8_t jpg_quality)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE;

	i += put_uint8_t_by_index(type, i, msg->payload); // type of requested/acknowledged data (as defined in ENUM DATA_TYPES in mavlink/include/mavlink_types.h)
	i += put_uint32_t_by_index(size, i, msg->payload); // total data size in bytes (set on ACK only)
	i += put_uint8_t_by_index(packets, i, msg->payload); // number of packets beeing sent (set on ACK only)
	i += put_uint8_t_by_index(payload, i, msg->payload); // payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on ACK only)
	i += put_uint8_t_by_index(jpg_quality, i, msg->payload); // JPEG quality out of [1,100]

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
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
	mavlink_message_t msg;
	mavlink_msg_data_transmission_handshake_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, type, size, packets, payload, jpg_quality);
	mavlink_send_uart(chan, &msg);
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
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field size from data_transmission_handshake message
 *
 * @return total data size in bytes (set on ACK only)
 */
static inline uint32_t mavlink_msg_data_transmission_handshake_get_size(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t))[3];
	return (uint32_t)r.i;
}

/**
 * @brief Get field packets from data_transmission_handshake message
 *
 * @return number of packets beeing sent (set on ACK only)
 */
static inline uint8_t mavlink_msg_data_transmission_handshake_get_packets(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t)+sizeof(uint32_t))[0];
}

/**
 * @brief Get field payload from data_transmission_handshake message
 *
 * @return payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on ACK only)
 */
static inline uint8_t mavlink_msg_data_transmission_handshake_get_payload(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint8_t))[0];
}

/**
 * @brief Get field jpg_quality from data_transmission_handshake message
 *
 * @return JPEG quality out of [1,100]
 */
static inline uint8_t mavlink_msg_data_transmission_handshake_get_jpg_quality(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint8_t)+sizeof(uint8_t))[0];
}

/**
 * @brief Decode a data_transmission_handshake message into a struct
 *
 * @param msg The message to decode
 * @param data_transmission_handshake C-struct to decode the message contents into
 */
static inline void mavlink_msg_data_transmission_handshake_decode(const mavlink_message_t* msg, mavlink_data_transmission_handshake_t* data_transmission_handshake)
{
	data_transmission_handshake->type = mavlink_msg_data_transmission_handshake_get_type(msg);
	data_transmission_handshake->size = mavlink_msg_data_transmission_handshake_get_size(msg);
	data_transmission_handshake->packets = mavlink_msg_data_transmission_handshake_get_packets(msg);
	data_transmission_handshake->payload = mavlink_msg_data_transmission_handshake_get_payload(msg);
	data_transmission_handshake->jpg_quality = mavlink_msg_data_transmission_handshake_get_jpg_quality(msg);
}
