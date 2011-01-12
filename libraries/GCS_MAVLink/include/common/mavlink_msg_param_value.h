// MESSAGE PARAM_VALUE PACKING

#define MAVLINK_MSG_ID_PARAM_VALUE 22

typedef struct __mavlink_param_value_t 
{
	int8_t param_id[15]; ///< Onboard parameter id
	float param_value; ///< Onboard parameter value
	uint16_t param_count; ///< Total number of onboard parameters
	uint16_t param_index; ///< Index of this onboard parameter

} mavlink_param_value_t;

#define MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN 15


/**
 * @brief Pack a param_value message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param param_id Onboard parameter id
 * @param param_value Onboard parameter value
 * @param param_count Total number of onboard parameters
 * @param param_index Index of this onboard parameter
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_param_value_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const int8_t* param_id, float param_value, uint16_t param_count, uint16_t param_index)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_PARAM_VALUE;

	i += put_array_by_index(param_id, 15, i, msg->payload); // Onboard parameter id
	i += put_float_by_index(param_value, i, msg->payload); // Onboard parameter value
	i += put_uint16_t_by_index(param_count, i, msg->payload); // Total number of onboard parameters
	i += put_uint16_t_by_index(param_index, i, msg->payload); // Index of this onboard parameter

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a param_value message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param param_id Onboard parameter id
 * @param param_value Onboard parameter value
 * @param param_count Total number of onboard parameters
 * @param param_index Index of this onboard parameter
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_param_value_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const int8_t* param_id, float param_value, uint16_t param_count, uint16_t param_index)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_PARAM_VALUE;

	i += put_array_by_index(param_id, 15, i, msg->payload); // Onboard parameter id
	i += put_float_by_index(param_value, i, msg->payload); // Onboard parameter value
	i += put_uint16_t_by_index(param_count, i, msg->payload); // Total number of onboard parameters
	i += put_uint16_t_by_index(param_index, i, msg->payload); // Index of this onboard parameter

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a param_value struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param param_value C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_param_value_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_param_value_t* param_value)
{
	return mavlink_msg_param_value_pack(system_id, component_id, msg, param_value->param_id, param_value->param_value, param_value->param_count, param_value->param_index);
}

/**
 * @brief Send a param_value message
 * @param chan MAVLink channel to send the message
 *
 * @param param_id Onboard parameter id
 * @param param_value Onboard parameter value
 * @param param_count Total number of onboard parameters
 * @param param_index Index of this onboard parameter
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_param_value_send(mavlink_channel_t chan, const int8_t* param_id, float param_value, uint16_t param_count, uint16_t param_index)
{
	mavlink_message_t msg;
	mavlink_msg_param_value_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, param_id, param_value, param_count, param_index);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE PARAM_VALUE UNPACKING

/**
 * @brief Get field param_id from param_value message
 *
 * @return Onboard parameter id
 */
static inline uint16_t mavlink_msg_param_value_get_param_id(const mavlink_message_t* msg, int8_t* r_data)
{

	memcpy(r_data, msg->payload, 15);
	return 15;
}

/**
 * @brief Get field param_value from param_value message
 *
 * @return Onboard parameter value
 */
static inline float mavlink_msg_param_value_get_param_value(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+15)[0];
	r.b[2] = (msg->payload+15)[1];
	r.b[1] = (msg->payload+15)[2];
	r.b[0] = (msg->payload+15)[3];
	return (float)r.f;
}

/**
 * @brief Get field param_count from param_value message
 *
 * @return Total number of onboard parameters
 */
static inline uint16_t mavlink_msg_param_value_get_param_count(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+15+sizeof(float))[0];
	r.b[0] = (msg->payload+15+sizeof(float))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field param_index from param_value message
 *
 * @return Index of this onboard parameter
 */
static inline uint16_t mavlink_msg_param_value_get_param_index(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+15+sizeof(float)+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+15+sizeof(float)+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Decode a param_value message into a struct
 *
 * @param msg The message to decode
 * @param param_value C-struct to decode the message contents into
 */
static inline void mavlink_msg_param_value_decode(const mavlink_message_t* msg, mavlink_param_value_t* param_value)
{
	mavlink_msg_param_value_get_param_id(msg, param_value->param_id);
	param_value->param_value = mavlink_msg_param_value_get_param_value(msg);
	param_value->param_count = mavlink_msg_param_value_get_param_count(msg);
	param_value->param_index = mavlink_msg_param_value_get_param_index(msg);
}
