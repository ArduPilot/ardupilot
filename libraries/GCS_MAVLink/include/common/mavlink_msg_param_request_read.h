// MESSAGE PARAM_REQUEST_READ PACKING

#define MAVLINK_MSG_ID_PARAM_REQUEST_READ 20

typedef struct __mavlink_param_request_read_t 
{
	uint8_t target_system; ///< System ID
	uint8_t target_component; ///< Component ID
	int8_t param_id[15]; ///< Onboard parameter id
	uint16_t param_index; ///< Parameter index

} mavlink_param_request_read_t;

#define MAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN 15


/**
 * @brief Pack a param_request_read message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param param_id Onboard parameter id
 * @param param_index Parameter index
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_param_request_read_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, const int8_t* param_id, uint16_t param_index)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_PARAM_REQUEST_READ;

	i += put_uint8_t_by_index(target_system, i, msg->payload); // System ID
	i += put_uint8_t_by_index(target_component, i, msg->payload); // Component ID
	i += put_array_by_index(param_id, 15, i, msg->payload); // Onboard parameter id
	i += put_uint16_t_by_index(param_index, i, msg->payload); // Parameter index

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a param_request_read message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param param_id Onboard parameter id
 * @param param_index Parameter index
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_param_request_read_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, const int8_t* param_id, uint16_t param_index)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_PARAM_REQUEST_READ;

	i += put_uint8_t_by_index(target_system, i, msg->payload); // System ID
	i += put_uint8_t_by_index(target_component, i, msg->payload); // Component ID
	i += put_array_by_index(param_id, 15, i, msg->payload); // Onboard parameter id
	i += put_uint16_t_by_index(param_index, i, msg->payload); // Parameter index

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a param_request_read struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param param_request_read C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_param_request_read_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_param_request_read_t* param_request_read)
{
	return mavlink_msg_param_request_read_pack(system_id, component_id, msg, param_request_read->target_system, param_request_read->target_component, param_request_read->param_id, param_request_read->param_index);
}

/**
 * @brief Send a param_request_read message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param param_id Onboard parameter id
 * @param param_index Parameter index
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_param_request_read_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, const int8_t* param_id, uint16_t param_index)
{
	mavlink_message_t msg;
	mavlink_msg_param_request_read_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, target_system, target_component, param_id, param_index);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE PARAM_REQUEST_READ UNPACKING

/**
 * @brief Get field target_system from param_request_read message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_param_request_read_get_target_system(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field target_component from param_request_read message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_param_request_read_get_target_component(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t))[0];
}

/**
 * @brief Get field param_id from param_request_read message
 *
 * @return Onboard parameter id
 */
static inline uint16_t mavlink_msg_param_request_read_get_param_id(const mavlink_message_t* msg, int8_t* r_data)
{

	memcpy(r_data, msg->payload+sizeof(uint8_t)+sizeof(uint8_t), 15);
	return 15;
}

/**
 * @brief Get field param_index from param_request_read message
 *
 * @return Parameter index
 */
static inline uint16_t mavlink_msg_param_request_read_get_param_index(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+15)[0];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+15)[1];
	return (uint16_t)r.s;
}

/**
 * @brief Decode a param_request_read message into a struct
 *
 * @param msg The message to decode
 * @param param_request_read C-struct to decode the message contents into
 */
static inline void mavlink_msg_param_request_read_decode(const mavlink_message_t* msg, mavlink_param_request_read_t* param_request_read)
{
	param_request_read->target_system = mavlink_msg_param_request_read_get_target_system(msg);
	param_request_read->target_component = mavlink_msg_param_request_read_get_target_component(msg);
	mavlink_msg_param_request_read_get_param_id(msg, param_request_read->param_id);
	param_request_read->param_index = mavlink_msg_param_request_read_get_param_index(msg);
}
