// MESSAGE GOPRO_COMMAND PACKING

#define MAVLINK_MSG_ID_GOPRO_COMMAND 217

typedef struct __mavlink_gopro_command_t
{
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
 uint8_t gp_cmd_name_1; ///< First character of the 2 character GoPro command
 uint8_t gp_cmd_name_2; ///< Second character of the 2 character GoPro command
 uint8_t gp_cmd_parm; ///< Parameter for the command
} mavlink_gopro_command_t;

#define MAVLINK_MSG_ID_GOPRO_COMMAND_LEN 5
#define MAVLINK_MSG_ID_217_LEN 5

#define MAVLINK_MSG_ID_GOPRO_COMMAND_CRC 43
#define MAVLINK_MSG_ID_217_CRC 43



#define MAVLINK_MESSAGE_INFO_GOPRO_COMMAND { \
	"GOPRO_COMMAND", \
	5, \
	{  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_gopro_command_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_gopro_command_t, target_component) }, \
         { "gp_cmd_name_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_gopro_command_t, gp_cmd_name_1) }, \
         { "gp_cmd_name_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_gopro_command_t, gp_cmd_name_2) }, \
         { "gp_cmd_parm", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_gopro_command_t, gp_cmd_parm) }, \
         } \
}


/**
 * @brief Pack a gopro_command message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param gp_cmd_name_1 First character of the 2 character GoPro command
 * @param gp_cmd_name_2 Second character of the 2 character GoPro command
 * @param gp_cmd_parm Parameter for the command
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gopro_command_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, uint8_t gp_cmd_name_1, uint8_t gp_cmd_name_2, uint8_t gp_cmd_parm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GOPRO_COMMAND_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, gp_cmd_name_1);
	_mav_put_uint8_t(buf, 3, gp_cmd_name_2);
	_mav_put_uint8_t(buf, 4, gp_cmd_parm);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GOPRO_COMMAND_LEN);
#else
	mavlink_gopro_command_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.gp_cmd_name_1 = gp_cmd_name_1;
	packet.gp_cmd_name_2 = gp_cmd_name_2;
	packet.gp_cmd_parm = gp_cmd_parm;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GOPRO_COMMAND_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GOPRO_COMMAND;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GOPRO_COMMAND_LEN, MAVLINK_MSG_ID_GOPRO_COMMAND_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GOPRO_COMMAND_LEN);
#endif
}

/**
 * @brief Pack a gopro_command message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param gp_cmd_name_1 First character of the 2 character GoPro command
 * @param gp_cmd_name_2 Second character of the 2 character GoPro command
 * @param gp_cmd_parm Parameter for the command
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gopro_command_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,uint8_t gp_cmd_name_1,uint8_t gp_cmd_name_2,uint8_t gp_cmd_parm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GOPRO_COMMAND_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, gp_cmd_name_1);
	_mav_put_uint8_t(buf, 3, gp_cmd_name_2);
	_mav_put_uint8_t(buf, 4, gp_cmd_parm);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GOPRO_COMMAND_LEN);
#else
	mavlink_gopro_command_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.gp_cmd_name_1 = gp_cmd_name_1;
	packet.gp_cmd_name_2 = gp_cmd_name_2;
	packet.gp_cmd_parm = gp_cmd_parm;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GOPRO_COMMAND_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GOPRO_COMMAND;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GOPRO_COMMAND_LEN, MAVLINK_MSG_ID_GOPRO_COMMAND_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GOPRO_COMMAND_LEN);
#endif
}

/**
 * @brief Encode a gopro_command struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gopro_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gopro_command_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gopro_command_t* gopro_command)
{
	return mavlink_msg_gopro_command_pack(system_id, component_id, msg, gopro_command->target_system, gopro_command->target_component, gopro_command->gp_cmd_name_1, gopro_command->gp_cmd_name_2, gopro_command->gp_cmd_parm);
}

/**
 * @brief Encode a gopro_command struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gopro_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gopro_command_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gopro_command_t* gopro_command)
{
	return mavlink_msg_gopro_command_pack_chan(system_id, component_id, chan, msg, gopro_command->target_system, gopro_command->target_component, gopro_command->gp_cmd_name_1, gopro_command->gp_cmd_name_2, gopro_command->gp_cmd_parm);
}

/**
 * @brief Send a gopro_command message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param gp_cmd_name_1 First character of the 2 character GoPro command
 * @param gp_cmd_name_2 Second character of the 2 character GoPro command
 * @param gp_cmd_parm Parameter for the command
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gopro_command_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t gp_cmd_name_1, uint8_t gp_cmd_name_2, uint8_t gp_cmd_parm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GOPRO_COMMAND_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, gp_cmd_name_1);
	_mav_put_uint8_t(buf, 3, gp_cmd_name_2);
	_mav_put_uint8_t(buf, 4, gp_cmd_parm);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GOPRO_COMMAND, buf, MAVLINK_MSG_ID_GOPRO_COMMAND_LEN, MAVLINK_MSG_ID_GOPRO_COMMAND_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GOPRO_COMMAND, buf, MAVLINK_MSG_ID_GOPRO_COMMAND_LEN);
#endif
#else
	mavlink_gopro_command_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.gp_cmd_name_1 = gp_cmd_name_1;
	packet.gp_cmd_name_2 = gp_cmd_name_2;
	packet.gp_cmd_parm = gp_cmd_parm;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GOPRO_COMMAND, (const char *)&packet, MAVLINK_MSG_ID_GOPRO_COMMAND_LEN, MAVLINK_MSG_ID_GOPRO_COMMAND_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GOPRO_COMMAND, (const char *)&packet, MAVLINK_MSG_ID_GOPRO_COMMAND_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_GOPRO_COMMAND_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gopro_command_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t gp_cmd_name_1, uint8_t gp_cmd_name_2, uint8_t gp_cmd_parm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, gp_cmd_name_1);
	_mav_put_uint8_t(buf, 3, gp_cmd_name_2);
	_mav_put_uint8_t(buf, 4, gp_cmd_parm);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GOPRO_COMMAND, buf, MAVLINK_MSG_ID_GOPRO_COMMAND_LEN, MAVLINK_MSG_ID_GOPRO_COMMAND_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GOPRO_COMMAND, buf, MAVLINK_MSG_ID_GOPRO_COMMAND_LEN);
#endif
#else
	mavlink_gopro_command_t *packet = (mavlink_gopro_command_t *)msgbuf;
	packet->target_system = target_system;
	packet->target_component = target_component;
	packet->gp_cmd_name_1 = gp_cmd_name_1;
	packet->gp_cmd_name_2 = gp_cmd_name_2;
	packet->gp_cmd_parm = gp_cmd_parm;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GOPRO_COMMAND, (const char *)packet, MAVLINK_MSG_ID_GOPRO_COMMAND_LEN, MAVLINK_MSG_ID_GOPRO_COMMAND_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GOPRO_COMMAND, (const char *)packet, MAVLINK_MSG_ID_GOPRO_COMMAND_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE GOPRO_COMMAND UNPACKING


/**
 * @brief Get field target_system from gopro_command message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_gopro_command_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field target_component from gopro_command message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_gopro_command_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field gp_cmd_name_1 from gopro_command message
 *
 * @return First character of the 2 character GoPro command
 */
static inline uint8_t mavlink_msg_gopro_command_get_gp_cmd_name_1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field gp_cmd_name_2 from gopro_command message
 *
 * @return Second character of the 2 character GoPro command
 */
static inline uint8_t mavlink_msg_gopro_command_get_gp_cmd_name_2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field gp_cmd_parm from gopro_command message
 *
 * @return Parameter for the command
 */
static inline uint8_t mavlink_msg_gopro_command_get_gp_cmd_parm(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Decode a gopro_command message into a struct
 *
 * @param msg The message to decode
 * @param gopro_command C-struct to decode the message contents into
 */
static inline void mavlink_msg_gopro_command_decode(const mavlink_message_t* msg, mavlink_gopro_command_t* gopro_command)
{
#if MAVLINK_NEED_BYTE_SWAP
	gopro_command->target_system = mavlink_msg_gopro_command_get_target_system(msg);
	gopro_command->target_component = mavlink_msg_gopro_command_get_target_component(msg);
	gopro_command->gp_cmd_name_1 = mavlink_msg_gopro_command_get_gp_cmd_name_1(msg);
	gopro_command->gp_cmd_name_2 = mavlink_msg_gopro_command_get_gp_cmd_name_2(msg);
	gopro_command->gp_cmd_parm = mavlink_msg_gopro_command_get_gp_cmd_parm(msg);
#else
	memcpy(gopro_command, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_GOPRO_COMMAND_LEN);
#endif
}
