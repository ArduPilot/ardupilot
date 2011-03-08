// MESSAGE COMMAND PACKING

#define MAVLINK_MSG_ID_COMMAND 75

typedef struct __mavlink_command_t 
{
	uint8_t target_system; ///< System which should execute the command
	uint8_t target_component; ///< Component which should execute the command, 0 for all components
	uint8_t command; ///< Command ID, as defined by MAV_CMD enum.
	uint8_t confirmation; ///< 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
	float param1; ///< Parameter 1, as defined by MAV_CMD enum.
	float param2; ///< Parameter 2, as defined by MAV_CMD enum.
	float param3; ///< Parameter 3, as defined by MAV_CMD enum.
	float param4; ///< Parameter 4, as defined by MAV_CMD enum.

} mavlink_command_t;



/**
 * @brief Pack a command message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System which should execute the command
 * @param target_component Component which should execute the command, 0 for all components
 * @param command Command ID, as defined by MAV_CMD enum.
 * @param confirmation 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 * @param param1 Parameter 1, as defined by MAV_CMD enum.
 * @param param2 Parameter 2, as defined by MAV_CMD enum.
 * @param param3 Parameter 3, as defined by MAV_CMD enum.
 * @param param4 Parameter 4, as defined by MAV_CMD enum.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_command_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint8_t command, uint8_t confirmation, float param1, float param2, float param3, float param4)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_COMMAND;

	i += put_uint8_t_by_index(target_system, i, msg->payload); // System which should execute the command
	i += put_uint8_t_by_index(target_component, i, msg->payload); // Component which should execute the command, 0 for all components
	i += put_uint8_t_by_index(command, i, msg->payload); // Command ID, as defined by MAV_CMD enum.
	i += put_uint8_t_by_index(confirmation, i, msg->payload); // 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
	i += put_float_by_index(param1, i, msg->payload); // Parameter 1, as defined by MAV_CMD enum.
	i += put_float_by_index(param2, i, msg->payload); // Parameter 2, as defined by MAV_CMD enum.
	i += put_float_by_index(param3, i, msg->payload); // Parameter 3, as defined by MAV_CMD enum.
	i += put_float_by_index(param4, i, msg->payload); // Parameter 4, as defined by MAV_CMD enum.

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a command message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System which should execute the command
 * @param target_component Component which should execute the command, 0 for all components
 * @param command Command ID, as defined by MAV_CMD enum.
 * @param confirmation 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 * @param param1 Parameter 1, as defined by MAV_CMD enum.
 * @param param2 Parameter 2, as defined by MAV_CMD enum.
 * @param param3 Parameter 3, as defined by MAV_CMD enum.
 * @param param4 Parameter 4, as defined by MAV_CMD enum.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_command_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint8_t command, uint8_t confirmation, float param1, float param2, float param3, float param4)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_COMMAND;

	i += put_uint8_t_by_index(target_system, i, msg->payload); // System which should execute the command
	i += put_uint8_t_by_index(target_component, i, msg->payload); // Component which should execute the command, 0 for all components
	i += put_uint8_t_by_index(command, i, msg->payload); // Command ID, as defined by MAV_CMD enum.
	i += put_uint8_t_by_index(confirmation, i, msg->payload); // 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
	i += put_float_by_index(param1, i, msg->payload); // Parameter 1, as defined by MAV_CMD enum.
	i += put_float_by_index(param2, i, msg->payload); // Parameter 2, as defined by MAV_CMD enum.
	i += put_float_by_index(param3, i, msg->payload); // Parameter 3, as defined by MAV_CMD enum.
	i += put_float_by_index(param4, i, msg->payload); // Parameter 4, as defined by MAV_CMD enum.

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a command struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_command_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_command_t* command)
{
	return mavlink_msg_command_pack(system_id, component_id, msg, command->target_system, command->target_component, command->command, command->confirmation, command->param1, command->param2, command->param3, command->param4);
}

/**
 * @brief Send a command message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System which should execute the command
 * @param target_component Component which should execute the command, 0 for all components
 * @param command Command ID, as defined by MAV_CMD enum.
 * @param confirmation 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 * @param param1 Parameter 1, as defined by MAV_CMD enum.
 * @param param2 Parameter 2, as defined by MAV_CMD enum.
 * @param param3 Parameter 3, as defined by MAV_CMD enum.
 * @param param4 Parameter 4, as defined by MAV_CMD enum.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_command_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t command, uint8_t confirmation, float param1, float param2, float param3, float param4)
{
	mavlink_message_t msg;
	mavlink_msg_command_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, target_system, target_component, command, confirmation, param1, param2, param3, param4);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE COMMAND UNPACKING

/**
 * @brief Get field target_system from command message
 *
 * @return System which should execute the command
 */
static inline uint8_t mavlink_msg_command_get_target_system(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field target_component from command message
 *
 * @return Component which should execute the command, 0 for all components
 */
static inline uint8_t mavlink_msg_command_get_target_component(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t))[0];
}

/**
 * @brief Get field command from command message
 *
 * @return Command ID, as defined by MAV_CMD enum.
 */
static inline uint8_t mavlink_msg_command_get_command(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[0];
}

/**
 * @brief Get field confirmation from command message
 *
 * @return 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 */
static inline uint8_t mavlink_msg_command_get_confirmation(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t))[0];
}

/**
 * @brief Get field param1 from command message
 *
 * @return Parameter 1, as defined by MAV_CMD enum.
 */
static inline float mavlink_msg_command_get_param1(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t))[3];
	return (float)r.f;
}

/**
 * @brief Get field param2 from command message
 *
 * @return Parameter 2, as defined by MAV_CMD enum.
 */
static inline float mavlink_msg_command_get_param2(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field param3 from command message
 *
 * @return Parameter 3, as defined by MAV_CMD enum.
 */
static inline float mavlink_msg_command_get_param3(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field param4 from command message
 *
 * @return Parameter 4, as defined by MAV_CMD enum.
 */
static inline float mavlink_msg_command_get_param4(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Decode a command message into a struct
 *
 * @param msg The message to decode
 * @param command C-struct to decode the message contents into
 */
static inline void mavlink_msg_command_decode(const mavlink_message_t* msg, mavlink_command_t* command)
{
	command->target_system = mavlink_msg_command_get_target_system(msg);
	command->target_component = mavlink_msg_command_get_target_component(msg);
	command->command = mavlink_msg_command_get_command(msg);
	command->confirmation = mavlink_msg_command_get_confirmation(msg);
	command->param1 = mavlink_msg_command_get_param1(msg);
	command->param2 = mavlink_msg_command_get_param2(msg);
	command->param3 = mavlink_msg_command_get_param3(msg);
	command->param4 = mavlink_msg_command_get_param4(msg);
}
