// MESSAGE ENCAPSULATED_IMAGE PACKING

#define MAVLINK_MSG_ID_ENCAPSULATED_IMAGE 171

typedef struct __mavlink_encapsulated_image_t 
{
	uint8_t seqnr; ///< sequence number (starting with 0 on every transmission)
	uint8_t data[254]; ///< image data bytes

} mavlink_encapsulated_image_t;

#define MAVLINK_MSG_ENCAPSULATED_IMAGE_FIELD_DATA_LEN 254


/**
 * @brief Send a encapsulated_image message
 *
 * @param seqnr sequence number (starting with 0 on every transmission)
 * @param data image data bytes
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_encapsulated_image_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t seqnr, const uint8_t* data)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_ENCAPSULATED_IMAGE;

	i += put_uint8_t_by_index(seqnr, i, msg->payload); //sequence number (starting with 0 on every transmission)
	i += put_array_by_index((int8_t*)data, sizeof(uint8_t)*254, i, msg->payload); //image data bytes

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

static inline uint16_t mavlink_msg_encapsulated_image_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_encapsulated_image_t* encapsulated_image)
{
	return mavlink_msg_encapsulated_image_pack(system_id, component_id, msg, encapsulated_image->seqnr, encapsulated_image->data);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_encapsulated_image_send(mavlink_channel_t chan, uint8_t seqnr, const uint8_t* data)
{
	mavlink_message_t msg;
	mavlink_msg_encapsulated_image_pack(mavlink_system.sysid, mavlink_system.compid, &msg, seqnr, data);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE ENCAPSULATED_IMAGE UNPACKING

/**
 * @brief Get field seqnr from encapsulated_image message
 *
 * @return sequence number (starting with 0 on every transmission)
 */
static inline uint8_t mavlink_msg_encapsulated_image_get_seqnr(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field data from encapsulated_image message
 *
 * @return image data bytes
 */
static inline uint16_t mavlink_msg_encapsulated_image_get_data(const mavlink_message_t* msg, uint8_t* r_data)
{

	memcpy(r_data, msg->payload+sizeof(uint8_t), sizeof(uint8_t)*254);
	return sizeof(uint8_t)*254;
}

static inline void mavlink_msg_encapsulated_image_decode(const mavlink_message_t* msg, mavlink_encapsulated_image_t* encapsulated_image)
{
	encapsulated_image->seqnr = mavlink_msg_encapsulated_image_get_seqnr(msg);
	mavlink_msg_encapsulated_image_get_data(msg, encapsulated_image->data);
}
