// MESSAGE SLUGS_ACTION PACKING

#define MAVLINK_MSG_ID_SLUGS_ACTION 183

typedef struct __mavlink_slugs_action_t 
{
	uint8_t target; ///< The system reporting the action
	uint8_t actionId; ///< Action ID. See apDefinitions.h in the SLUGS /clib directory for the ID names
	uint16_t actionVal; ///< Value associated with the action

} mavlink_slugs_action_t;



/**
 * @brief Send a slugs_action message
 *
 * @param target The system reporting the action
 * @param actionId Action ID. See apDefinitions.h in the SLUGS /clib directory for the ID names
 * @param actionVal Value associated with the action
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_slugs_action_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target, uint8_t actionId, uint16_t actionVal)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_SLUGS_ACTION;

	i += put_uint8_t_by_index(target, i, msg->payload); //The system reporting the action
	i += put_uint8_t_by_index(actionId, i, msg->payload); //Action ID. See apDefinitions.h in the SLUGS /clib directory for the ID names
	i += put_uint16_t_by_index(actionVal, i, msg->payload); //Value associated with the action

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

static inline uint16_t mavlink_msg_slugs_action_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_slugs_action_t* slugs_action)
{
	return mavlink_msg_slugs_action_pack(system_id, component_id, msg, slugs_action->target, slugs_action->actionId, slugs_action->actionVal);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_slugs_action_send(mavlink_channel_t chan, uint8_t target, uint8_t actionId, uint16_t actionVal)
{
	mavlink_message_t msg;
	mavlink_msg_slugs_action_pack(mavlink_system.sysid, mavlink_system.compid, &msg, target, actionId, actionVal);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE SLUGS_ACTION UNPACKING

/**
 * @brief Get field target from slugs_action message
 *
 * @return The system reporting the action
 */
static inline uint8_t mavlink_msg_slugs_action_get_target(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field actionId from slugs_action message
 *
 * @return Action ID. See apDefinitions.h in the SLUGS /clib directory for the ID names
 */
static inline uint8_t mavlink_msg_slugs_action_get_actionId(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t))[0];
}

/**
 * @brief Get field actionVal from slugs_action message
 *
 * @return Value associated with the action
 */
static inline uint16_t mavlink_msg_slugs_action_get_actionVal(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[0];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[1];
	return (uint16_t)r.s;
}

static inline void mavlink_msg_slugs_action_decode(const mavlink_message_t* msg, mavlink_slugs_action_t* slugs_action)
{
	slugs_action->target = mavlink_msg_slugs_action_get_target(msg);
	slugs_action->actionId = mavlink_msg_slugs_action_get_actionId(msg);
	slugs_action->actionVal = mavlink_msg_slugs_action_get_actionVal(msg);
}
