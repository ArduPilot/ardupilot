// MESSAGE CMD_AIRSPEED_ACK PACKING

#define MAVLINK_MSG_ID_CMD_AIRSPEED_ACK 194

typedef struct __mavlink_cmd_airspeed_ack_t
{
 float spCmd; ///< commanded airspeed
 uint8_t ack; ///< 0:ack, 1:nack
} mavlink_cmd_airspeed_ack_t;

#define MAVLINK_MSG_ID_CMD_AIRSPEED_ACK_LEN 5
#define MAVLINK_MSG_ID_194_LEN 5



#define MAVLINK_MESSAGE_INFO_CMD_AIRSPEED_ACK { \
	"CMD_AIRSPEED_ACK", \
	2, \
	{  { "spCmd", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_cmd_airspeed_ack_t, spCmd) }, \
         { "ack", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_cmd_airspeed_ack_t, ack) }, \
         } \
}


/**
 * @brief Pack a cmd_airspeed_ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param spCmd commanded airspeed
 * @param ack 0:ack, 1:nack
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cmd_airspeed_ack_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float spCmd, uint8_t ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[5];
	_mav_put_float(buf, 0, spCmd);
	_mav_put_uint8_t(buf, 4, ack);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 5);
#else
	mavlink_cmd_airspeed_ack_t packet;
	packet.spCmd = spCmd;
	packet.ack = ack;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 5);
#endif

	msg->msgid = MAVLINK_MSG_ID_CMD_AIRSPEED_ACK;
	return mavlink_finalize_message(msg, system_id, component_id, 5, 243);
}

/**
 * @brief Pack a cmd_airspeed_ack message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param spCmd commanded airspeed
 * @param ack 0:ack, 1:nack
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cmd_airspeed_ack_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float spCmd,uint8_t ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[5];
	_mav_put_float(buf, 0, spCmd);
	_mav_put_uint8_t(buf, 4, ack);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 5);
#else
	mavlink_cmd_airspeed_ack_t packet;
	packet.spCmd = spCmd;
	packet.ack = ack;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 5);
#endif

	msg->msgid = MAVLINK_MSG_ID_CMD_AIRSPEED_ACK;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 5, 243);
}

/**
 * @brief Encode a cmd_airspeed_ack struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param cmd_airspeed_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cmd_airspeed_ack_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_cmd_airspeed_ack_t* cmd_airspeed_ack)
{
	return mavlink_msg_cmd_airspeed_ack_pack(system_id, component_id, msg, cmd_airspeed_ack->spCmd, cmd_airspeed_ack->ack);
}

/**
 * @brief Send a cmd_airspeed_ack message
 * @param chan MAVLink channel to send the message
 *
 * @param spCmd commanded airspeed
 * @param ack 0:ack, 1:nack
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_cmd_airspeed_ack_send(mavlink_channel_t chan, float spCmd, uint8_t ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[5];
	_mav_put_float(buf, 0, spCmd);
	_mav_put_uint8_t(buf, 4, ack);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CMD_AIRSPEED_ACK, buf, 5, 243);
#else
	mavlink_cmd_airspeed_ack_t packet;
	packet.spCmd = spCmd;
	packet.ack = ack;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CMD_AIRSPEED_ACK, (const char *)&packet, 5, 243);
#endif
}

#endif

// MESSAGE CMD_AIRSPEED_ACK UNPACKING


/**
 * @brief Get field spCmd from cmd_airspeed_ack message
 *
 * @return commanded airspeed
 */
static inline float mavlink_msg_cmd_airspeed_ack_get_spCmd(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field ack from cmd_airspeed_ack message
 *
 * @return 0:ack, 1:nack
 */
static inline uint8_t mavlink_msg_cmd_airspeed_ack_get_ack(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Decode a cmd_airspeed_ack message into a struct
 *
 * @param msg The message to decode
 * @param cmd_airspeed_ack C-struct to decode the message contents into
 */
static inline void mavlink_msg_cmd_airspeed_ack_decode(const mavlink_message_t* msg, mavlink_cmd_airspeed_ack_t* cmd_airspeed_ack)
{
#if MAVLINK_NEED_BYTE_SWAP
	cmd_airspeed_ack->spCmd = mavlink_msg_cmd_airspeed_ack_get_spCmd(msg);
	cmd_airspeed_ack->ack = mavlink_msg_cmd_airspeed_ack_get_ack(msg);
#else
	memcpy(cmd_airspeed_ack, _MAV_PAYLOAD(msg), 5);
#endif
}
