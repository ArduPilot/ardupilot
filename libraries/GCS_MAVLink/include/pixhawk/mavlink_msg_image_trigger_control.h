// MESSAGE IMAGE_TRIGGER_CONTROL PACKING

#define MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL 102

typedef struct __mavlink_image_trigger_control_t 
{
	uint8_t enable; ///< 0 to disable, 1 to enable

} mavlink_image_trigger_control_t;



/**
 * @brief Send a image_trigger_control message
 *
 * @param enable 0 to disable, 1 to enable
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_image_trigger_control_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t enable)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL;

	i += put_uint8_t_by_index(enable, i, msg->payload); //0 to disable, 1 to enable

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

static inline uint16_t mavlink_msg_image_trigger_control_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_image_trigger_control_t* image_trigger_control)
{
	return mavlink_msg_image_trigger_control_pack(system_id, component_id, msg, image_trigger_control->enable);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_image_trigger_control_send(mavlink_channel_t chan, uint8_t enable)
{
	mavlink_message_t msg;
	mavlink_msg_image_trigger_control_pack(mavlink_system.sysid, mavlink_system.compid, &msg, enable);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE IMAGE_TRIGGER_CONTROL UNPACKING

/**
 * @brief Get field enable from image_trigger_control message
 *
 * @return 0 to disable, 1 to enable
 */
static inline uint8_t mavlink_msg_image_trigger_control_get_enable(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

static inline void mavlink_msg_image_trigger_control_decode(const mavlink_message_t* msg, mavlink_image_trigger_control_t* image_trigger_control)
{
	image_trigger_control->enable = mavlink_msg_image_trigger_control_get_enable(msg);
}
