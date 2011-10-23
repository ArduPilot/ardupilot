// MESSAGE MID_LVL_CMDS PACKING

#define MAVLINK_MSG_ID_MID_LVL_CMDS 180

typedef struct __mavlink_mid_lvl_cmds_t
{
 float hCommand; ///< Commanded Airspeed
 float uCommand; ///< Log value 2 
 float rCommand; ///< Log value 3 
 uint8_t target; ///< The system setting the commands
} mavlink_mid_lvl_cmds_t;

#define MAVLINK_MSG_ID_MID_LVL_CMDS_LEN 13
#define MAVLINK_MSG_ID_180_LEN 13



#define MAVLINK_MESSAGE_INFO_MID_LVL_CMDS { \
	"MID_LVL_CMDS", \
	4, \
	{  { "hCommand", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_mid_lvl_cmds_t, hCommand) }, \
         { "uCommand", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_mid_lvl_cmds_t, uCommand) }, \
         { "rCommand", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_mid_lvl_cmds_t, rCommand) }, \
         { "target", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_mid_lvl_cmds_t, target) }, \
         } \
}


/**
 * @brief Pack a mid_lvl_cmds message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target The system setting the commands
 * @param hCommand Commanded Airspeed
 * @param uCommand Log value 2 
 * @param rCommand Log value 3 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mid_lvl_cmds_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target, float hCommand, float uCommand, float rCommand)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[13];
	_mav_put_float(buf, 0, hCommand);
	_mav_put_float(buf, 4, uCommand);
	_mav_put_float(buf, 8, rCommand);
	_mav_put_uint8_t(buf, 12, target);

        memcpy(_MAV_PAYLOAD(msg), buf, 13);
#else
	mavlink_mid_lvl_cmds_t packet;
	packet.hCommand = hCommand;
	packet.uCommand = uCommand;
	packet.rCommand = rCommand;
	packet.target = target;

        memcpy(_MAV_PAYLOAD(msg), &packet, 13);
#endif

	msg->msgid = MAVLINK_MSG_ID_MID_LVL_CMDS;
	return mavlink_finalize_message(msg, system_id, component_id, 13, 146);
}

/**
 * @brief Pack a mid_lvl_cmds message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target The system setting the commands
 * @param hCommand Commanded Airspeed
 * @param uCommand Log value 2 
 * @param rCommand Log value 3 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mid_lvl_cmds_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target,float hCommand,float uCommand,float rCommand)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[13];
	_mav_put_float(buf, 0, hCommand);
	_mav_put_float(buf, 4, uCommand);
	_mav_put_float(buf, 8, rCommand);
	_mav_put_uint8_t(buf, 12, target);

        memcpy(_MAV_PAYLOAD(msg), buf, 13);
#else
	mavlink_mid_lvl_cmds_t packet;
	packet.hCommand = hCommand;
	packet.uCommand = uCommand;
	packet.rCommand = rCommand;
	packet.target = target;

        memcpy(_MAV_PAYLOAD(msg), &packet, 13);
#endif

	msg->msgid = MAVLINK_MSG_ID_MID_LVL_CMDS;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 13, 146);
}

/**
 * @brief Encode a mid_lvl_cmds struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mid_lvl_cmds C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mid_lvl_cmds_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mid_lvl_cmds_t* mid_lvl_cmds)
{
	return mavlink_msg_mid_lvl_cmds_pack(system_id, component_id, msg, mid_lvl_cmds->target, mid_lvl_cmds->hCommand, mid_lvl_cmds->uCommand, mid_lvl_cmds->rCommand);
}

/**
 * @brief Send a mid_lvl_cmds message
 * @param chan MAVLink channel to send the message
 *
 * @param target The system setting the commands
 * @param hCommand Commanded Airspeed
 * @param uCommand Log value 2 
 * @param rCommand Log value 3 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mid_lvl_cmds_send(mavlink_channel_t chan, uint8_t target, float hCommand, float uCommand, float rCommand)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[13];
	_mav_put_float(buf, 0, hCommand);
	_mav_put_float(buf, 4, uCommand);
	_mav_put_float(buf, 8, rCommand);
	_mav_put_uint8_t(buf, 12, target);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MID_LVL_CMDS, buf, 13, 146);
#else
	mavlink_mid_lvl_cmds_t packet;
	packet.hCommand = hCommand;
	packet.uCommand = uCommand;
	packet.rCommand = rCommand;
	packet.target = target;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MID_LVL_CMDS, (const char *)&packet, 13, 146);
#endif
}

#endif

// MESSAGE MID_LVL_CMDS UNPACKING


/**
 * @brief Get field target from mid_lvl_cmds message
 *
 * @return The system setting the commands
 */
static inline uint8_t mavlink_msg_mid_lvl_cmds_get_target(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field hCommand from mid_lvl_cmds message
 *
 * @return Commanded Airspeed
 */
static inline float mavlink_msg_mid_lvl_cmds_get_hCommand(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field uCommand from mid_lvl_cmds message
 *
 * @return Log value 2 
 */
static inline float mavlink_msg_mid_lvl_cmds_get_uCommand(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field rCommand from mid_lvl_cmds message
 *
 * @return Log value 3 
 */
static inline float mavlink_msg_mid_lvl_cmds_get_rCommand(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a mid_lvl_cmds message into a struct
 *
 * @param msg The message to decode
 * @param mid_lvl_cmds C-struct to decode the message contents into
 */
static inline void mavlink_msg_mid_lvl_cmds_decode(const mavlink_message_t* msg, mavlink_mid_lvl_cmds_t* mid_lvl_cmds)
{
#if MAVLINK_NEED_BYTE_SWAP
	mid_lvl_cmds->hCommand = mavlink_msg_mid_lvl_cmds_get_hCommand(msg);
	mid_lvl_cmds->uCommand = mavlink_msg_mid_lvl_cmds_get_uCommand(msg);
	mid_lvl_cmds->rCommand = mavlink_msg_mid_lvl_cmds_get_rCommand(msg);
	mid_lvl_cmds->target = mavlink_msg_mid_lvl_cmds_get_target(msg);
#else
	memcpy(mid_lvl_cmds, _MAV_PAYLOAD(msg), 13);
#endif
}
