// MESSAGE SLUGS_ACTION PACKING

#define MAVLINK_MSG_ID_SLUGS_ACTION 183

typedef struct __mavlink_slugs_action_t
{
 uint16_t actionVal; ///< Value associated with the action
 uint8_t target; ///< The system reporting the action
 uint8_t actionId; ///< Action ID. See apDefinitions.h in the SLUGS /clib directory for the ID names
} mavlink_slugs_action_t;

#define MAVLINK_MSG_ID_SLUGS_ACTION_LEN 4
#define MAVLINK_MSG_ID_183_LEN 4



#define MAVLINK_MESSAGE_INFO_SLUGS_ACTION { \
	"SLUGS_ACTION", \
	3, \
	{  { "actionVal", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_slugs_action_t, actionVal) }, \
         { "target", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_slugs_action_t, target) }, \
         { "actionId", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_slugs_action_t, actionId) }, \
         } \
}


/**
 * @brief Pack a slugs_action message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target The system reporting the action
 * @param actionId Action ID. See apDefinitions.h in the SLUGS /clib directory for the ID names
 * @param actionVal Value associated with the action
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_slugs_action_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target, uint8_t actionId, uint16_t actionVal)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_uint16_t(buf, 0, actionVal);
	_mav_put_uint8_t(buf, 2, target);
	_mav_put_uint8_t(buf, 3, actionId);

        memcpy(_MAV_PAYLOAD(msg), buf, 4);
#else
	mavlink_slugs_action_t packet;
	packet.actionVal = actionVal;
	packet.target = target;
	packet.actionId = actionId;

        memcpy(_MAV_PAYLOAD(msg), &packet, 4);
#endif

	msg->msgid = MAVLINK_MSG_ID_SLUGS_ACTION;
	return mavlink_finalize_message(msg, system_id, component_id, 4, 65);
}

/**
 * @brief Pack a slugs_action message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target The system reporting the action
 * @param actionId Action ID. See apDefinitions.h in the SLUGS /clib directory for the ID names
 * @param actionVal Value associated with the action
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_slugs_action_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target,uint8_t actionId,uint16_t actionVal)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_uint16_t(buf, 0, actionVal);
	_mav_put_uint8_t(buf, 2, target);
	_mav_put_uint8_t(buf, 3, actionId);

        memcpy(_MAV_PAYLOAD(msg), buf, 4);
#else
	mavlink_slugs_action_t packet;
	packet.actionVal = actionVal;
	packet.target = target;
	packet.actionId = actionId;

        memcpy(_MAV_PAYLOAD(msg), &packet, 4);
#endif

	msg->msgid = MAVLINK_MSG_ID_SLUGS_ACTION;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 4, 65);
}

/**
 * @brief Encode a slugs_action struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param slugs_action C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_slugs_action_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_slugs_action_t* slugs_action)
{
	return mavlink_msg_slugs_action_pack(system_id, component_id, msg, slugs_action->target, slugs_action->actionId, slugs_action->actionVal);
}

/**
 * @brief Send a slugs_action message
 * @param chan MAVLink channel to send the message
 *
 * @param target The system reporting the action
 * @param actionId Action ID. See apDefinitions.h in the SLUGS /clib directory for the ID names
 * @param actionVal Value associated with the action
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_slugs_action_send(mavlink_channel_t chan, uint8_t target, uint8_t actionId, uint16_t actionVal)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_uint16_t(buf, 0, actionVal);
	_mav_put_uint8_t(buf, 2, target);
	_mav_put_uint8_t(buf, 3, actionId);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SLUGS_ACTION, buf, 4, 65);
#else
	mavlink_slugs_action_t packet;
	packet.actionVal = actionVal;
	packet.target = target;
	packet.actionId = actionId;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SLUGS_ACTION, (const char *)&packet, 4, 65);
#endif
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
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field actionId from slugs_action message
 *
 * @return Action ID. See apDefinitions.h in the SLUGS /clib directory for the ID names
 */
static inline uint8_t mavlink_msg_slugs_action_get_actionId(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field actionVal from slugs_action message
 *
 * @return Value associated with the action
 */
static inline uint16_t mavlink_msg_slugs_action_get_actionVal(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a slugs_action message into a struct
 *
 * @param msg The message to decode
 * @param slugs_action C-struct to decode the message contents into
 */
static inline void mavlink_msg_slugs_action_decode(const mavlink_message_t* msg, mavlink_slugs_action_t* slugs_action)
{
#if MAVLINK_NEED_BYTE_SWAP
	slugs_action->actionVal = mavlink_msg_slugs_action_get_actionVal(msg);
	slugs_action->target = mavlink_msg_slugs_action_get_target(msg);
	slugs_action->actionId = mavlink_msg_slugs_action_get_actionId(msg);
#else
	memcpy(slugs_action, _MAV_PAYLOAD(msg), 4);
#endif
}
