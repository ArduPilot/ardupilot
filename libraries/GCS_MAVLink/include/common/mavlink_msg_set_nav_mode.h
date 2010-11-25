// MESSAGE SET_NAV_MODE PACKING

#define MAVLINK_MSG_ID_SET_NAV_MODE 12

typedef struct __mavlink_set_nav_mode_t 
{
	uint8_t target; ///< The system setting the mode
	uint8_t nav_mode; ///< The new navigation mode

} mavlink_set_nav_mode_t;



/**
 * @brief Send a set_nav_mode message
 *
 * @param target The system setting the mode
 * @param nav_mode The new navigation mode
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_nav_mode_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target, uint8_t nav_mode)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_SET_NAV_MODE;

	i += put_uint8_t_by_index(target, i, msg->payload); //The system setting the mode
	i += put_uint8_t_by_index(nav_mode, i, msg->payload); //The new navigation mode

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

static inline uint16_t mavlink_msg_set_nav_mode_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_nav_mode_t* set_nav_mode)
{
	return mavlink_msg_set_nav_mode_pack(system_id, component_id, msg, set_nav_mode->target, set_nav_mode->nav_mode);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_nav_mode_send(mavlink_channel_t chan, uint8_t target, uint8_t nav_mode)
{
	mavlink_message_t msg;
	mavlink_msg_set_nav_mode_pack(mavlink_system.sysid, mavlink_system.compid, &msg, target, nav_mode);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE SET_NAV_MODE UNPACKING

/**
 * @brief Get field target from set_nav_mode message
 *
 * @return The system setting the mode
 */
static inline uint8_t mavlink_msg_set_nav_mode_get_target(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field nav_mode from set_nav_mode message
 *
 * @return The new navigation mode
 */
static inline uint8_t mavlink_msg_set_nav_mode_get_nav_mode(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t))[0];
}

static inline void mavlink_msg_set_nav_mode_decode(const mavlink_message_t* msg, mavlink_set_nav_mode_t* set_nav_mode)
{
	set_nav_mode->target = mavlink_msg_set_nav_mode_get_target(msg);
	set_nav_mode->nav_mode = mavlink_msg_set_nav_mode_get_nav_mode(msg);
}
