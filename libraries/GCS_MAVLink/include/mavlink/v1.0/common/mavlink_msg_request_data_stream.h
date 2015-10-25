// MESSAGE REQUEST_DATA_STREAM PACKING

#define MAVLINK_MSG_ID_REQUEST_DATA_STREAM 66

typedef struct __mavlink_request_data_stream_t
{
 uint16_t req_message_rate; ///< The requested interval between two messages of this type
 uint8_t target_system; ///< The target requested to send the message stream.
 uint8_t target_component; ///< The target requested to send the message stream.
 uint8_t req_stream_id; ///< The ID of the requested data stream
 uint8_t start_stop; ///< 1 to start sending, 0 to stop sending.
} mavlink_request_data_stream_t;

#define MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN 6
#define MAVLINK_MSG_ID_66_LEN 6

#define MAVLINK_MSG_ID_REQUEST_DATA_STREAM_CRC 148
#define MAVLINK_MSG_ID_66_CRC 148



#define MAVLINK_MESSAGE_INFO_REQUEST_DATA_STREAM { \
	"REQUEST_DATA_STREAM", \
	5, \
	{  { "req_message_rate", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_request_data_stream_t, req_message_rate) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_request_data_stream_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_request_data_stream_t, target_component) }, \
         { "req_stream_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_request_data_stream_t, req_stream_id) }, \
         { "start_stop", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_request_data_stream_t, start_stop) }, \
         } \
}


/**
 * @brief Pack a request_data_stream message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system The target requested to send the message stream.
 * @param target_component The target requested to send the message stream.
 * @param req_stream_id The ID of the requested data stream
 * @param req_message_rate The requested interval between two messages of this type
 * @param start_stop 1 to start sending, 0 to stop sending.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_request_data_stream_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, uint16_t req_message_rate, uint8_t start_stop)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN];
	_mav_put_uint16_t(buf, 0, req_message_rate);
	_mav_put_uint8_t(buf, 2, target_system);
	_mav_put_uint8_t(buf, 3, target_component);
	_mav_put_uint8_t(buf, 4, req_stream_id);
	_mav_put_uint8_t(buf, 5, start_stop);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN);
#else
	mavlink_request_data_stream_t packet;
	packet.req_message_rate = req_message_rate;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.req_stream_id = req_stream_id;
	packet.start_stop = start_stop;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_REQUEST_DATA_STREAM;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN, MAVLINK_MSG_ID_REQUEST_DATA_STREAM_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN);
#endif
}

/**
 * @brief Pack a request_data_stream message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system The target requested to send the message stream.
 * @param target_component The target requested to send the message stream.
 * @param req_stream_id The ID of the requested data stream
 * @param req_message_rate The requested interval between two messages of this type
 * @param start_stop 1 to start sending, 0 to stop sending.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_request_data_stream_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,uint8_t req_stream_id,uint16_t req_message_rate,uint8_t start_stop)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN];
	_mav_put_uint16_t(buf, 0, req_message_rate);
	_mav_put_uint8_t(buf, 2, target_system);
	_mav_put_uint8_t(buf, 3, target_component);
	_mav_put_uint8_t(buf, 4, req_stream_id);
	_mav_put_uint8_t(buf, 5, start_stop);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN);
#else
	mavlink_request_data_stream_t packet;
	packet.req_message_rate = req_message_rate;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.req_stream_id = req_stream_id;
	packet.start_stop = start_stop;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_REQUEST_DATA_STREAM;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN, MAVLINK_MSG_ID_REQUEST_DATA_STREAM_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN);
#endif
}

/**
 * @brief Encode a request_data_stream struct
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
 * @brief Encode a request_data_stream struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param request_data_stream C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_request_data_stream_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_request_data_stream_t* request_data_stream)
{
	return mavlink_msg_request_data_stream_pack_chan(system_id, component_id, chan, msg, request_data_stream->target_system, request_data_stream->target_component, request_data_stream->req_stream_id, request_data_stream->req_message_rate, request_data_stream->start_stop);
}

/**
 * @brief Send a request_data_stream message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system The target requested to send the message stream.
 * @param target_component The target requested to send the message stream.
 * @param req_stream_id The ID of the requested data stream
 * @param req_message_rate The requested interval between two messages of this type
 * @param start_stop 1 to start sending, 0 to stop sending.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_request_data_stream_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, uint16_t req_message_rate, uint8_t start_stop)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN];
	_mav_put_uint16_t(buf, 0, req_message_rate);
	_mav_put_uint8_t(buf, 2, target_system);
	_mav_put_uint8_t(buf, 3, target_component);
	_mav_put_uint8_t(buf, 4, req_stream_id);
	_mav_put_uint8_t(buf, 5, start_stop);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_DATA_STREAM, buf, MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN, MAVLINK_MSG_ID_REQUEST_DATA_STREAM_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_DATA_STREAM, buf, MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN);
#endif
#else
	mavlink_request_data_stream_t packet;
	packet.req_message_rate = req_message_rate;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.req_stream_id = req_stream_id;
	packet.start_stop = start_stop;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_DATA_STREAM, (const char *)&packet, MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN, MAVLINK_MSG_ID_REQUEST_DATA_STREAM_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_DATA_STREAM, (const char *)&packet, MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_request_data_stream_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, uint16_t req_message_rate, uint8_t start_stop)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint16_t(buf, 0, req_message_rate);
	_mav_put_uint8_t(buf, 2, target_system);
	_mav_put_uint8_t(buf, 3, target_component);
	_mav_put_uint8_t(buf, 4, req_stream_id);
	_mav_put_uint8_t(buf, 5, start_stop);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_DATA_STREAM, buf, MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN, MAVLINK_MSG_ID_REQUEST_DATA_STREAM_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_DATA_STREAM, buf, MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN);
#endif
#else
	mavlink_request_data_stream_t *packet = (mavlink_request_data_stream_t *)msgbuf;
	packet->req_message_rate = req_message_rate;
	packet->target_system = target_system;
	packet->target_component = target_component;
	packet->req_stream_id = req_stream_id;
	packet->start_stop = start_stop;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_DATA_STREAM, (const char *)packet, MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN, MAVLINK_MSG_ID_REQUEST_DATA_STREAM_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_DATA_STREAM, (const char *)packet, MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE REQUEST_DATA_STREAM UNPACKING


/**
 * @brief Get field target_system from request_data_stream message
 *
 * @return The target requested to send the message stream.
 */
static inline uint8_t mavlink_msg_request_data_stream_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field target_component from request_data_stream message
 *
 * @return The target requested to send the message stream.
 */
static inline uint8_t mavlink_msg_request_data_stream_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field req_stream_id from request_data_stream message
 *
 * @return The ID of the requested data stream
 */
static inline uint8_t mavlink_msg_request_data_stream_get_req_stream_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field req_message_rate from request_data_stream message
 *
 * @return The requested interval between two messages of this type
 */
static inline uint16_t mavlink_msg_request_data_stream_get_req_message_rate(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field start_stop from request_data_stream message
 *
 * @return 1 to start sending, 0 to stop sending.
 */
static inline uint8_t mavlink_msg_request_data_stream_get_start_stop(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Decode a request_data_stream message into a struct
 *
 * @param msg The message to decode
 * @param request_data_stream C-struct to decode the message contents into
 */
static inline void mavlink_msg_request_data_stream_decode(const mavlink_message_t* msg, mavlink_request_data_stream_t* request_data_stream)
{
#if MAVLINK_NEED_BYTE_SWAP
	request_data_stream->req_message_rate = mavlink_msg_request_data_stream_get_req_message_rate(msg);
	request_data_stream->target_system = mavlink_msg_request_data_stream_get_target_system(msg);
	request_data_stream->target_component = mavlink_msg_request_data_stream_get_target_component(msg);
	request_data_stream->req_stream_id = mavlink_msg_request_data_stream_get_req_stream_id(msg);
	request_data_stream->start_stop = mavlink_msg_request_data_stream_get_start_stop(msg);
#else
	memcpy(request_data_stream, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN);
#endif
}
