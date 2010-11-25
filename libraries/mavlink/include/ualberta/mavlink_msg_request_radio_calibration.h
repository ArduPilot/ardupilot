// MESSAGE REQUEST_RADIO_CALIBRATION PACKING

#define MAVLINK_MSG_ID_REQUEST_RADIO_CALIBRATION 83

typedef struct __mavlink_request_radio_calibration_t 
{
	uint8_t unused; ///< Unused field.  Included to prevent compile time warnings

} mavlink_request_radio_calibration_t;



/**
 * @brief Send a request_radio_calibration message
 *
 * @param unused Unused field.  Included to prevent compile time warnings
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_request_radio_calibration_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t unused)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_REQUEST_RADIO_CALIBRATION;

	i += put_uint8_t_by_index(unused, i, msg->payload); //Unused field.  Included to prevent compile time warnings

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

static inline uint16_t mavlink_msg_request_radio_calibration_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_request_radio_calibration_t* request_radio_calibration)
{
	return mavlink_msg_request_radio_calibration_pack(system_id, component_id, msg, request_radio_calibration->unused);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_request_radio_calibration_send(mavlink_channel_t chan, uint8_t unused)
{
	mavlink_message_t msg;
	mavlink_msg_request_radio_calibration_pack(mavlink_system.sysid, mavlink_system.compid, &msg, unused);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE REQUEST_RADIO_CALIBRATION UNPACKING

/**
 * @brief Get field unused from request_radio_calibration message
 *
 * @return Unused field.  Included to prevent compile time warnings
 */
static inline uint8_t mavlink_msg_request_radio_calibration_get_unused(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

static inline void mavlink_msg_request_radio_calibration_decode(const mavlink_message_t* msg, mavlink_request_radio_calibration_t* request_radio_calibration)
{
	request_radio_calibration->unused = mavlink_msg_request_radio_calibration_get_unused(msg);
}
