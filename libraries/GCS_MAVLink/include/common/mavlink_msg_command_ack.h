// MESSAGE COMMAND_ACK PACKING

#define MAVLINK_MSG_ID_COMMAND_ACK 76

typedef struct __mavlink_command_ack_t 
{
	float command; ///< Current airspeed in m/s
	float result; ///< 1: Action ACCEPTED and EXECUTED, 1: Action TEMPORARY REJECTED/DENIED, 2: Action PERMANENTLY DENIED, 3: Action UNKNOWN/UNSUPPORTED, 4: Requesting CONFIRMATION

} mavlink_command_ack_t;



/**
 * @brief Pack a command_ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param command Current airspeed in m/s
 * @param result 1: Action ACCEPTED and EXECUTED, 1: Action TEMPORARY REJECTED/DENIED, 2: Action PERMANENTLY DENIED, 3: Action UNKNOWN/UNSUPPORTED, 4: Requesting CONFIRMATION
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_command_ack_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, float command, float result)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_COMMAND_ACK;

	i += put_float_by_index(command, i, msg->payload); // Current airspeed in m/s
	i += put_float_by_index(result, i, msg->payload); // 1: Action ACCEPTED and EXECUTED, 1: Action TEMPORARY REJECTED/DENIED, 2: Action PERMANENTLY DENIED, 3: Action UNKNOWN/UNSUPPORTED, 4: Requesting CONFIRMATION

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a command_ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param command Current airspeed in m/s
 * @param result 1: Action ACCEPTED and EXECUTED, 1: Action TEMPORARY REJECTED/DENIED, 2: Action PERMANENTLY DENIED, 3: Action UNKNOWN/UNSUPPORTED, 4: Requesting CONFIRMATION
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_command_ack_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, float command, float result)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_COMMAND_ACK;

	i += put_float_by_index(command, i, msg->payload); // Current airspeed in m/s
	i += put_float_by_index(result, i, msg->payload); // 1: Action ACCEPTED and EXECUTED, 1: Action TEMPORARY REJECTED/DENIED, 2: Action PERMANENTLY DENIED, 3: Action UNKNOWN/UNSUPPORTED, 4: Requesting CONFIRMATION

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a command_ack struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param command_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_command_ack_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_command_ack_t* command_ack)
{
	return mavlink_msg_command_ack_pack(system_id, component_id, msg, command_ack->command, command_ack->result);
}

/**
 * @brief Send a command_ack message
 * @param chan MAVLink channel to send the message
 *
 * @param command Current airspeed in m/s
 * @param result 1: Action ACCEPTED and EXECUTED, 1: Action TEMPORARY REJECTED/DENIED, 2: Action PERMANENTLY DENIED, 3: Action UNKNOWN/UNSUPPORTED, 4: Requesting CONFIRMATION
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_command_ack_send(mavlink_channel_t chan, float command, float result)
{
	mavlink_message_t msg;
	mavlink_msg_command_ack_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, command, result);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE COMMAND_ACK UNPACKING

/**
 * @brief Get field command from command_ack message
 *
 * @return Current airspeed in m/s
 */
static inline float mavlink_msg_command_ack_get_command(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload)[0];
	r.b[2] = (msg->payload)[1];
	r.b[1] = (msg->payload)[2];
	r.b[0] = (msg->payload)[3];
	return (float)r.f;
}

/**
 * @brief Get field result from command_ack message
 *
 * @return 1: Action ACCEPTED and EXECUTED, 1: Action TEMPORARY REJECTED/DENIED, 2: Action PERMANENTLY DENIED, 3: Action UNKNOWN/UNSUPPORTED, 4: Requesting CONFIRMATION
 */
static inline float mavlink_msg_command_ack_get_result(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Decode a command_ack message into a struct
 *
 * @param msg The message to decode
 * @param command_ack C-struct to decode the message contents into
 */
static inline void mavlink_msg_command_ack_decode(const mavlink_message_t* msg, mavlink_command_ack_t* command_ack)
{
	command_ack->command = mavlink_msg_command_ack_get_command(msg);
	command_ack->result = mavlink_msg_command_ack_get_result(msg);
}
