// MESSAGE MID_LVL_CMDS PACKING

#define MAVLINK_MSG_ID_MID_LVL_CMDS 180

typedef struct __mavlink_mid_lvl_cmds_t 
{
	uint8_t target; ///< The system setting the commands
	float hCommand; ///< Commanded Airspeed
	float uCommand; ///< Log value 2 
	float rCommand; ///< Log value 3 

} mavlink_mid_lvl_cmds_t;



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
static inline uint16_t mavlink_msg_mid_lvl_cmds_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target, float hCommand, float uCommand, float rCommand)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_MID_LVL_CMDS;

	i += put_uint8_t_by_index(target, i, msg->payload); // The system setting the commands
	i += put_float_by_index(hCommand, i, msg->payload); // Commanded Airspeed
	i += put_float_by_index(uCommand, i, msg->payload); // Log value 2 
	i += put_float_by_index(rCommand, i, msg->payload); // Log value 3 

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a mid_lvl_cmds message
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
static inline uint16_t mavlink_msg_mid_lvl_cmds_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint8_t target, float hCommand, float uCommand, float rCommand)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_MID_LVL_CMDS;

	i += put_uint8_t_by_index(target, i, msg->payload); // The system setting the commands
	i += put_float_by_index(hCommand, i, msg->payload); // Commanded Airspeed
	i += put_float_by_index(uCommand, i, msg->payload); // Log value 2 
	i += put_float_by_index(rCommand, i, msg->payload); // Log value 3 

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
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
	mavlink_message_t msg;
	mavlink_msg_mid_lvl_cmds_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, target, hCommand, uCommand, rCommand);
	mavlink_send_uart(chan, &msg);
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
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field hCommand from mid_lvl_cmds message
 *
 * @return Commanded Airspeed
 */
static inline float mavlink_msg_mid_lvl_cmds_get_hCommand(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t))[3];
	return (float)r.f;
}

/**
 * @brief Get field uCommand from mid_lvl_cmds message
 *
 * @return Log value 2 
 */
static inline float mavlink_msg_mid_lvl_cmds_get_uCommand(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field rCommand from mid_lvl_cmds message
 *
 * @return Log value 3 
 */
static inline float mavlink_msg_mid_lvl_cmds_get_rCommand(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Decode a mid_lvl_cmds message into a struct
 *
 * @param msg The message to decode
 * @param mid_lvl_cmds C-struct to decode the message contents into
 */
static inline void mavlink_msg_mid_lvl_cmds_decode(const mavlink_message_t* msg, mavlink_mid_lvl_cmds_t* mid_lvl_cmds)
{
	mid_lvl_cmds->target = mavlink_msg_mid_lvl_cmds_get_target(msg);
	mid_lvl_cmds->hCommand = mavlink_msg_mid_lvl_cmds_get_hCommand(msg);
	mid_lvl_cmds->uCommand = mavlink_msg_mid_lvl_cmds_get_uCommand(msg);
	mid_lvl_cmds->rCommand = mavlink_msg_mid_lvl_cmds_get_rCommand(msg);
}
