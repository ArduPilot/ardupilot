// MESSAGE CMD_AIRSPEED_CHNG PACKING

#define MAVLINK_MSG_ID_CMD_AIRSPEED_CHNG 192

typedef struct __mavlink_cmd_airspeed_chng_t
{
 float spCmd; ///< commanded airspeed
 uint8_t target; ///< Target ID
} mavlink_cmd_airspeed_chng_t;

#define MAVLINK_MSG_ID_CMD_AIRSPEED_CHNG_LEN 5
#define MAVLINK_MSG_ID_192_LEN 5



#define MAVLINK_MESSAGE_INFO_CMD_AIRSPEED_CHNG { \
	"CMD_AIRSPEED_CHNG", \
	2, \
	{  { "spCmd", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_cmd_airspeed_chng_t, spCmd) }, \
         { "target", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_cmd_airspeed_chng_t, target) }, \
         } \
}


/**
 * @brief Pack a cmd_airspeed_chng message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target Target ID
 * @param spCmd commanded airspeed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cmd_airspeed_chng_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target, float spCmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[5];
	_mav_put_float(buf, 0, spCmd);
	_mav_put_uint8_t(buf, 4, target);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 5);
#else
	mavlink_cmd_airspeed_chng_t packet;
	packet.spCmd = spCmd;
	packet.target = target;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 5);
#endif

	msg->msgid = MAVLINK_MSG_ID_CMD_AIRSPEED_CHNG;
	return mavlink_finalize_message(msg, system_id, component_id, 5, 209);
}

/**
 * @brief Pack a cmd_airspeed_chng message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target Target ID
 * @param spCmd commanded airspeed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cmd_airspeed_chng_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target,float spCmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[5];
	_mav_put_float(buf, 0, spCmd);
	_mav_put_uint8_t(buf, 4, target);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 5);
#else
	mavlink_cmd_airspeed_chng_t packet;
	packet.spCmd = spCmd;
	packet.target = target;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 5);
#endif

	msg->msgid = MAVLINK_MSG_ID_CMD_AIRSPEED_CHNG;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 5, 209);
}

/**
 * @brief Encode a cmd_airspeed_chng struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param cmd_airspeed_chng C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cmd_airspeed_chng_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_cmd_airspeed_chng_t* cmd_airspeed_chng)
{
	return mavlink_msg_cmd_airspeed_chng_pack(system_id, component_id, msg, cmd_airspeed_chng->target, cmd_airspeed_chng->spCmd);
}

/**
 * @brief Send a cmd_airspeed_chng message
 * @param chan MAVLink channel to send the message
 *
 * @param target Target ID
 * @param spCmd commanded airspeed
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_cmd_airspeed_chng_send(mavlink_channel_t chan, uint8_t target, float spCmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[5];
	_mav_put_float(buf, 0, spCmd);
	_mav_put_uint8_t(buf, 4, target);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CMD_AIRSPEED_CHNG, buf, 5, 209);
#else
	mavlink_cmd_airspeed_chng_t packet;
	packet.spCmd = spCmd;
	packet.target = target;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CMD_AIRSPEED_CHNG, (const char *)&packet, 5, 209);
#endif
}

#endif

// MESSAGE CMD_AIRSPEED_CHNG UNPACKING


/**
 * @brief Get field target from cmd_airspeed_chng message
 *
 * @return Target ID
 */
static inline uint8_t mavlink_msg_cmd_airspeed_chng_get_target(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field spCmd from cmd_airspeed_chng message
 *
 * @return commanded airspeed
 */
static inline float mavlink_msg_cmd_airspeed_chng_get_spCmd(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a cmd_airspeed_chng message into a struct
 *
 * @param msg The message to decode
 * @param cmd_airspeed_chng C-struct to decode the message contents into
 */
static inline void mavlink_msg_cmd_airspeed_chng_decode(const mavlink_message_t* msg, mavlink_cmd_airspeed_chng_t* cmd_airspeed_chng)
{
#if MAVLINK_NEED_BYTE_SWAP
	cmd_airspeed_chng->spCmd = mavlink_msg_cmd_airspeed_chng_get_spCmd(msg);
	cmd_airspeed_chng->target = mavlink_msg_cmd_airspeed_chng_get_target(msg);
#else
	memcpy(cmd_airspeed_chng, _MAV_PAYLOAD(msg), 5);
#endif
}
