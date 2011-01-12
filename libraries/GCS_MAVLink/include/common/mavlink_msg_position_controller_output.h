// MESSAGE POSITION_CONTROLLER_OUTPUT PACKING

#define MAVLINK_MSG_ID_POSITION_CONTROLLER_OUTPUT 61

typedef struct __mavlink_position_controller_output_t 
{
	uint8_t enabled; ///< 1: enabled, 0: disabled
	int8_t x; ///< Position x: -128: -100%, 127: +100%
	int8_t y; ///< Position y: -128: -100%, 127: +100%
	int8_t z; ///< Position z: -128: -100%, 127: +100%
	int8_t yaw; ///< Position yaw: -128: -100%, 127: +100%

} mavlink_position_controller_output_t;



/**
 * @brief Pack a position_controller_output message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param enabled 1: enabled, 0: disabled
 * @param x Position x: -128: -100%, 127: +100%
 * @param y Position y: -128: -100%, 127: +100%
 * @param z Position z: -128: -100%, 127: +100%
 * @param yaw Position yaw: -128: -100%, 127: +100%
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_position_controller_output_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t enabled, int8_t x, int8_t y, int8_t z, int8_t yaw)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_POSITION_CONTROLLER_OUTPUT;

	i += put_uint8_t_by_index(enabled, i, msg->payload); // 1: enabled, 0: disabled
	i += put_int8_t_by_index(x, i, msg->payload); // Position x: -128: -100%, 127: +100%
	i += put_int8_t_by_index(y, i, msg->payload); // Position y: -128: -100%, 127: +100%
	i += put_int8_t_by_index(z, i, msg->payload); // Position z: -128: -100%, 127: +100%
	i += put_int8_t_by_index(yaw, i, msg->payload); // Position yaw: -128: -100%, 127: +100%

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a position_controller_output message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param enabled 1: enabled, 0: disabled
 * @param x Position x: -128: -100%, 127: +100%
 * @param y Position y: -128: -100%, 127: +100%
 * @param z Position z: -128: -100%, 127: +100%
 * @param yaw Position yaw: -128: -100%, 127: +100%
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_position_controller_output_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint8_t enabled, int8_t x, int8_t y, int8_t z, int8_t yaw)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_POSITION_CONTROLLER_OUTPUT;

	i += put_uint8_t_by_index(enabled, i, msg->payload); // 1: enabled, 0: disabled
	i += put_int8_t_by_index(x, i, msg->payload); // Position x: -128: -100%, 127: +100%
	i += put_int8_t_by_index(y, i, msg->payload); // Position y: -128: -100%, 127: +100%
	i += put_int8_t_by_index(z, i, msg->payload); // Position z: -128: -100%, 127: +100%
	i += put_int8_t_by_index(yaw, i, msg->payload); // Position yaw: -128: -100%, 127: +100%

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a position_controller_output struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param position_controller_output C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_position_controller_output_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_position_controller_output_t* position_controller_output)
{
	return mavlink_msg_position_controller_output_pack(system_id, component_id, msg, position_controller_output->enabled, position_controller_output->x, position_controller_output->y, position_controller_output->z, position_controller_output->yaw);
}

/**
 * @brief Send a position_controller_output message
 * @param chan MAVLink channel to send the message
 *
 * @param enabled 1: enabled, 0: disabled
 * @param x Position x: -128: -100%, 127: +100%
 * @param y Position y: -128: -100%, 127: +100%
 * @param z Position z: -128: -100%, 127: +100%
 * @param yaw Position yaw: -128: -100%, 127: +100%
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_position_controller_output_send(mavlink_channel_t chan, uint8_t enabled, int8_t x, int8_t y, int8_t z, int8_t yaw)
{
	mavlink_message_t msg;
	mavlink_msg_position_controller_output_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, enabled, x, y, z, yaw);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE POSITION_CONTROLLER_OUTPUT UNPACKING

/**
 * @brief Get field enabled from position_controller_output message
 *
 * @return 1: enabled, 0: disabled
 */
static inline uint8_t mavlink_msg_position_controller_output_get_enabled(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field x from position_controller_output message
 *
 * @return Position x: -128: -100%, 127: +100%
 */
static inline int8_t mavlink_msg_position_controller_output_get_x(const mavlink_message_t* msg)
{
	return (int8_t)(msg->payload+sizeof(uint8_t))[0];
}

/**
 * @brief Get field y from position_controller_output message
 *
 * @return Position y: -128: -100%, 127: +100%
 */
static inline int8_t mavlink_msg_position_controller_output_get_y(const mavlink_message_t* msg)
{
	return (int8_t)(msg->payload+sizeof(uint8_t)+sizeof(int8_t))[0];
}

/**
 * @brief Get field z from position_controller_output message
 *
 * @return Position z: -128: -100%, 127: +100%
 */
static inline int8_t mavlink_msg_position_controller_output_get_z(const mavlink_message_t* msg)
{
	return (int8_t)(msg->payload+sizeof(uint8_t)+sizeof(int8_t)+sizeof(int8_t))[0];
}

/**
 * @brief Get field yaw from position_controller_output message
 *
 * @return Position yaw: -128: -100%, 127: +100%
 */
static inline int8_t mavlink_msg_position_controller_output_get_yaw(const mavlink_message_t* msg)
{
	return (int8_t)(msg->payload+sizeof(uint8_t)+sizeof(int8_t)+sizeof(int8_t)+sizeof(int8_t))[0];
}

/**
 * @brief Decode a position_controller_output message into a struct
 *
 * @param msg The message to decode
 * @param position_controller_output C-struct to decode the message contents into
 */
static inline void mavlink_msg_position_controller_output_decode(const mavlink_message_t* msg, mavlink_position_controller_output_t* position_controller_output)
{
	position_controller_output->enabled = mavlink_msg_position_controller_output_get_enabled(msg);
	position_controller_output->x = mavlink_msg_position_controller_output_get_x(msg);
	position_controller_output->y = mavlink_msg_position_controller_output_get_y(msg);
	position_controller_output->z = mavlink_msg_position_controller_output_get_z(msg);
	position_controller_output->yaw = mavlink_msg_position_controller_output_get_yaw(msg);
}
