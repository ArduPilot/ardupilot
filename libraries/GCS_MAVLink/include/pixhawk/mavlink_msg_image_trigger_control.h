// MESSAGE IMAGE_TRIGGER_CONTROL PACKING

#define MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL 102

typedef struct __mavlink_image_trigger_control_t 
{
	uint8_t enable; ///< 0 to disable, 1 to enable

} mavlink_image_trigger_control_t;



/**
 * @brief Pack a image_trigger_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param enable 0 to disable, 1 to enable
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_image_trigger_control_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t enable)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL;

	i += put_uint8_t_by_index(enable, i, msg->payload); // 0 to disable, 1 to enable

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a image_trigger_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param enable 0 to disable, 1 to enable
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_image_trigger_control_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint8_t enable)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL;

	i += put_uint8_t_by_index(enable, i, msg->payload); // 0 to disable, 1 to enable

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a image_trigger_control struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param image_trigger_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_image_trigger_control_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_image_trigger_control_t* image_trigger_control)
{
	return mavlink_msg_image_trigger_control_pack(system_id, component_id, msg, image_trigger_control->enable);
}

/**
 * @brief Send a image_trigger_control message
 * @param chan MAVLink channel to send the message
 *
 * @param enable 0 to disable, 1 to enable
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_image_trigger_control_send(mavlink_channel_t chan, uint8_t enable)
{
	mavlink_message_t msg;
	mavlink_msg_image_trigger_control_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, enable);
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

/**
 * @brief Decode a image_trigger_control message into a struct
 *
 * @param msg The message to decode
 * @param image_trigger_control C-struct to decode the message contents into
 */
static inline void mavlink_msg_image_trigger_control_decode(const mavlink_message_t* msg, mavlink_image_trigger_control_t* image_trigger_control)
{
	image_trigger_control->enable = mavlink_msg_image_trigger_control_get_enable(msg);
}
