// MESSAGE REQUEST_RC_CHANNELS PACKING

#define MAVLINK_MSG_ID_REQUEST_RC_CHANNELS 221

typedef struct __mavlink_request_rc_channels_t 
{
	uint8_t enabled; ///< True: start sending data; False: stop sending data

} mavlink_request_rc_channels_t;



/**
 * @brief Send a request_rc_channels message
 *
 * @param enabled True: start sending data; False: stop sending data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_request_rc_channels_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t enabled)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_REQUEST_RC_CHANNELS;

	i += put_uint8_t_by_index(enabled, i, msg->payload); //True: start sending data; False: stop sending data

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

static inline uint16_t mavlink_msg_request_rc_channels_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_request_rc_channels_t* request_rc_channels)
{
	return mavlink_msg_request_rc_channels_pack(system_id, component_id, msg, request_rc_channels->enabled);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_request_rc_channels_send(mavlink_channel_t chan, uint8_t enabled)
{
	mavlink_message_t msg;
	mavlink_msg_request_rc_channels_pack(mavlink_system.sysid, mavlink_system.compid, &msg, enabled);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE REQUEST_RC_CHANNELS UNPACKING

/**
 * @brief Get field enabled from request_rc_channels message
 *
 * @return True: start sending data; False: stop sending data
 */
static inline uint8_t mavlink_msg_request_rc_channels_get_enabled(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

static inline void mavlink_msg_request_rc_channels_decode(const mavlink_message_t* msg, mavlink_request_rc_channels_t* request_rc_channels)
{
	request_rc_channels->enabled = mavlink_msg_request_rc_channels_get_enabled(msg);
}
