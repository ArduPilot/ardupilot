// MESSAGE GET_IMAGE_ACK PACKING

#define MAVLINK_MSG_ID_GET_IMAGE_ACK 170

typedef struct __mavlink_get_image_ack_t 
{
	uint16_t size; ///< image size in bytes (65000 byte max)
	uint8_t packets; ///< number of packets beeing sent
	uint8_t payload; ///< image payload size (normally 254 byte)
	uint8_t quality; ///< JPEG quality out of [0,100]

} mavlink_get_image_ack_t;



/**
 * @brief Send a get_image_ack message
 *
 * @param size image size in bytes (65000 byte max)
 * @param packets number of packets beeing sent
 * @param payload image payload size (normally 254 byte)
 * @param quality JPEG quality out of [0,100]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_get_image_ack_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint16_t size, uint8_t packets, uint8_t payload, uint8_t quality)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_GET_IMAGE_ACK;

	i += put_uint16_t_by_index(size, i, msg->payload); //image size in bytes (65000 byte max)
	i += put_uint8_t_by_index(packets, i, msg->payload); //number of packets beeing sent
	i += put_uint8_t_by_index(payload, i, msg->payload); //image payload size (normally 254 byte)
	i += put_uint8_t_by_index(quality, i, msg->payload); //JPEG quality out of [0,100]

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

static inline uint16_t mavlink_msg_get_image_ack_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_get_image_ack_t* get_image_ack)
{
	return mavlink_msg_get_image_ack_pack(system_id, component_id, msg, get_image_ack->size, get_image_ack->packets, get_image_ack->payload, get_image_ack->quality);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_get_image_ack_send(mavlink_channel_t chan, uint16_t size, uint8_t packets, uint8_t payload, uint8_t quality)
{
	mavlink_message_t msg;
	mavlink_msg_get_image_ack_pack(mavlink_system.sysid, mavlink_system.compid, &msg, size, packets, payload, quality);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE GET_IMAGE_ACK UNPACKING

/**
 * @brief Get field size from get_image_ack message
 *
 * @return image size in bytes (65000 byte max)
 */
static inline uint16_t mavlink_msg_get_image_ack_get_size(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload)[0];
	r.b[0] = (msg->payload)[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field packets from get_image_ack message
 *
 * @return number of packets beeing sent
 */
static inline uint8_t mavlink_msg_get_image_ack_get_packets(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint16_t))[0];
}

/**
 * @brief Get field payload from get_image_ack message
 *
 * @return image payload size (normally 254 byte)
 */
static inline uint8_t mavlink_msg_get_image_ack_get_payload(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint16_t)+sizeof(uint8_t))[0];
}

/**
 * @brief Get field quality from get_image_ack message
 *
 * @return JPEG quality out of [0,100]
 */
static inline uint8_t mavlink_msg_get_image_ack_get_quality(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t))[0];
}

static inline void mavlink_msg_get_image_ack_decode(const mavlink_message_t* msg, mavlink_get_image_ack_t* get_image_ack)
{
	get_image_ack->size = mavlink_msg_get_image_ack_get_size(msg);
	get_image_ack->packets = mavlink_msg_get_image_ack_get_packets(msg);
	get_image_ack->payload = mavlink_msg_get_image_ack_get_payload(msg);
	get_image_ack->quality = mavlink_msg_get_image_ack_get_quality(msg);
}
