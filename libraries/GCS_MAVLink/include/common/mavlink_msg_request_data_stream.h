// MESSAGE REQUEST_DATA_STREAM PACKING

#define MAVLINK_MSG_ID_REQUEST_DATA_STREAM 66

typedef struct __mavlink_request_data_stream_t 
{
	uint8_t target_system; ///< The target requested to send the message stream.
	uint8_t target_component; ///< The target requested to send the message stream.
	uint8_t req_stream_id; ///< The ID of the requested message type
	uint16_t req_message_rate; ///< The requested interval between two messages of this type
	uint8_t start_stop; ///< 1 to start sending, 0 to stop sending.

} mavlink_request_data_stream_t;



/**
 * @brief Pack a request_data_stream message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system The target requested to send the message stream.
 * @param target_component The target requested to send the message stream.
 * @param req_stream_id The ID of the requested message type
 * @param req_message_rate The requested interval between two messages of this type
 * @param start_stop 1 to start sending, 0 to stop sending.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_request_data_stream_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, uint16_t req_message_rate, uint8_t start_stop)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_REQUEST_DATA_STREAM;

	i += put_uint8_t_by_index(target_system, i, msg->payload); // The target requested to send the message stream.
	i += put_uint8_t_by_index(target_component, i, msg->payload); // The target requested to send the message stream.
	i += put_uint8_t_by_index(req_stream_id, i, msg->payload); // The ID of the requested message type
	i += put_uint16_t_by_index(req_message_rate, i, msg->payload); // The requested interval between two messages of this type
	i += put_uint8_t_by_index(start_stop, i, msg->payload); // 1 to start sending, 0 to stop sending.

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a request_data_stream message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system The target requested to send the message stream.
 * @param target_component The target requested to send the message stream.
 * @param req_stream_id The ID of the requested message type
 * @param req_message_rate The requested interval between two messages of this type
 * @param start_stop 1 to start sending, 0 to stop sending.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_request_data_stream_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, uint16_t req_message_rate, uint8_t start_stop)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_REQUEST_DATA_STREAM;

	i += put_uint8_t_by_index(target_system, i, msg->payload); // The target requested to send the message stream.
	i += put_uint8_t_by_index(target_component, i, msg->payload); // The target requested to send the message stream.
	i += put_uint8_t_by_index(req_stream_id, i, msg->payload); // The ID of the requested message type
	i += put_uint16_t_by_index(req_message_rate, i, msg->payload); // The requested interval between two messages of this type
	i += put_uint8_t_by_index(start_stop, i, msg->payload); // 1 to start sending, 0 to stop sending.

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a request_data_stream struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param request_data_stream C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_request_data_stream_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_request_data_stream_t* request_data_stream)
{
	return mavlink_msg_request_data_stream_pack(system_id, component_id, msg, request_data_stream->target_system, request_data_stream->target_component, request_data_stream->req_stream_id, request_data_stream->req_message_rate, request_data_stream->start_stop);
}

/**
 * @brief Send a request_data_stream message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system The target requested to send the message stream.
 * @param target_component The target requested to send the message stream.
 * @param req_stream_id The ID of the requested message type
 * @param req_message_rate The requested interval between two messages of this type
 * @param start_stop 1 to start sending, 0 to stop sending.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_request_data_stream_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, uint16_t req_message_rate, uint8_t start_stop)
{
	mavlink_message_t msg;
	mavlink_msg_request_data_stream_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, target_system, target_component, req_stream_id, req_message_rate, start_stop);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE REQUEST_DATA_STREAM UNPACKING

/**
 * @brief Get field target_system from request_data_stream message
 *
 * @return The target requested to send the message stream.
 */
static inline uint8_t mavlink_msg_request_data_stream_get_target_system(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field target_component from request_data_stream message
 *
 * @return The target requested to send the message stream.
 */
static inline uint8_t mavlink_msg_request_data_stream_get_target_component(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t))[0];
}

/**
 * @brief Get field req_stream_id from request_data_stream message
 *
 * @return The ID of the requested message type
 */
static inline uint8_t mavlink_msg_request_data_stream_get_req_stream_id(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[0];
}

/**
 * @brief Get field req_message_rate from request_data_stream message
 *
 * @return The requested interval between two messages of this type
 */
static inline uint16_t mavlink_msg_request_data_stream_get_req_message_rate(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t))[0];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field start_stop from request_data_stream message
 *
 * @return 1 to start sending, 0 to stop sending.
 */
static inline uint8_t mavlink_msg_request_data_stream_get_start_stop(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t))[0];
}

/**
 * @brief Decode a request_data_stream message into a struct
 *
 * @param msg The message to decode
 * @param request_data_stream C-struct to decode the message contents into
 */
static inline void mavlink_msg_request_data_stream_decode(const mavlink_message_t* msg, mavlink_request_data_stream_t* request_data_stream)
{
	request_data_stream->target_system = mavlink_msg_request_data_stream_get_target_system(msg);
	request_data_stream->target_component = mavlink_msg_request_data_stream_get_target_component(msg);
	request_data_stream->req_stream_id = mavlink_msg_request_data_stream_get_req_stream_id(msg);
	request_data_stream->req_message_rate = mavlink_msg_request_data_stream_get_req_message_rate(msg);
	request_data_stream->start_stop = mavlink_msg_request_data_stream_get_start_stop(msg);
}
