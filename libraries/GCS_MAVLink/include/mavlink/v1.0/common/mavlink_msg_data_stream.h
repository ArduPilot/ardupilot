// MESSAGE DATA_STREAM PACKING

#define MAVLINK_MSG_ID_DATA_STREAM 67

typedef struct __mavlink_data_stream_t
{
 uint16_t message_rate; ///< The requested interval between two messages of this type
 uint8_t stream_id; ///< The ID of the requested data stream
 uint8_t on_off; ///< 1 stream is enabled, 0 stream is stopped.
} mavlink_data_stream_t;

#define MAVLINK_MSG_ID_DATA_STREAM_LEN 4
#define MAVLINK_MSG_ID_67_LEN 4

#define MAVLINK_MSG_ID_DATA_STREAM_CRC 21
#define MAVLINK_MSG_ID_67_CRC 21



#define MAVLINK_MESSAGE_INFO_DATA_STREAM { \
	"DATA_STREAM", \
	3, \
	{  { "message_rate", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_data_stream_t, message_rate) }, \
         { "stream_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_data_stream_t, stream_id) }, \
         { "on_off", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_data_stream_t, on_off) }, \
         } \
}


/**
 * @brief Pack a data_stream message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param stream_id The ID of the requested data stream
 * @param message_rate The requested interval between two messages of this type
 * @param on_off 1 stream is enabled, 0 stream is stopped.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_data_stream_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t stream_id, uint16_t message_rate, uint8_t on_off)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_DATA_STREAM_LEN];
	_mav_put_uint16_t(buf, 0, message_rate);
	_mav_put_uint8_t(buf, 2, stream_id);
	_mav_put_uint8_t(buf, 3, on_off);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DATA_STREAM_LEN);
#else
	mavlink_data_stream_t packet;
	packet.message_rate = message_rate;
	packet.stream_id = stream_id;
	packet.on_off = on_off;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DATA_STREAM_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_DATA_STREAM;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DATA_STREAM_LEN, MAVLINK_MSG_ID_DATA_STREAM_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DATA_STREAM_LEN);
#endif
}

/**
 * @brief Pack a data_stream message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param stream_id The ID of the requested data stream
 * @param message_rate The requested interval between two messages of this type
 * @param on_off 1 stream is enabled, 0 stream is stopped.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_data_stream_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t stream_id,uint16_t message_rate,uint8_t on_off)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_DATA_STREAM_LEN];
	_mav_put_uint16_t(buf, 0, message_rate);
	_mav_put_uint8_t(buf, 2, stream_id);
	_mav_put_uint8_t(buf, 3, on_off);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DATA_STREAM_LEN);
#else
	mavlink_data_stream_t packet;
	packet.message_rate = message_rate;
	packet.stream_id = stream_id;
	packet.on_off = on_off;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DATA_STREAM_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_DATA_STREAM;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DATA_STREAM_LEN, MAVLINK_MSG_ID_DATA_STREAM_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DATA_STREAM_LEN);
#endif
}

/**
 * @brief Encode a data_stream struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param data_stream C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_data_stream_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_data_stream_t* data_stream)
{
	return mavlink_msg_data_stream_pack(system_id, component_id, msg, data_stream->stream_id, data_stream->message_rate, data_stream->on_off);
}

/**
 * @brief Encode a data_stream struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param data_stream C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_data_stream_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_data_stream_t* data_stream)
{
	return mavlink_msg_data_stream_pack_chan(system_id, component_id, chan, msg, data_stream->stream_id, data_stream->message_rate, data_stream->on_off);
}

/**
 * @brief Send a data_stream message
 * @param chan MAVLink channel to send the message
 *
 * @param stream_id The ID of the requested data stream
 * @param message_rate The requested interval between two messages of this type
 * @param on_off 1 stream is enabled, 0 stream is stopped.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_data_stream_send(mavlink_channel_t chan, uint8_t stream_id, uint16_t message_rate, uint8_t on_off)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_DATA_STREAM_LEN];
	_mav_put_uint16_t(buf, 0, message_rate);
	_mav_put_uint8_t(buf, 2, stream_id);
	_mav_put_uint8_t(buf, 3, on_off);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA_STREAM, buf, MAVLINK_MSG_ID_DATA_STREAM_LEN, MAVLINK_MSG_ID_DATA_STREAM_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA_STREAM, buf, MAVLINK_MSG_ID_DATA_STREAM_LEN);
#endif
#else
	mavlink_data_stream_t packet;
	packet.message_rate = message_rate;
	packet.stream_id = stream_id;
	packet.on_off = on_off;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA_STREAM, (const char *)&packet, MAVLINK_MSG_ID_DATA_STREAM_LEN, MAVLINK_MSG_ID_DATA_STREAM_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA_STREAM, (const char *)&packet, MAVLINK_MSG_ID_DATA_STREAM_LEN);
#endif
#endif
}

#endif

// MESSAGE DATA_STREAM UNPACKING


/**
 * @brief Get field stream_id from data_stream message
 *
 * @return The ID of the requested data stream
 */
static inline uint8_t mavlink_msg_data_stream_get_stream_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field message_rate from data_stream message
 *
 * @return The requested interval between two messages of this type
 */
static inline uint16_t mavlink_msg_data_stream_get_message_rate(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field on_off from data_stream message
 *
 * @return 1 stream is enabled, 0 stream is stopped.
 */
static inline uint8_t mavlink_msg_data_stream_get_on_off(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Decode a data_stream message into a struct
 *
 * @param msg The message to decode
 * @param data_stream C-struct to decode the message contents into
 */
static inline void mavlink_msg_data_stream_decode(const mavlink_message_t* msg, mavlink_data_stream_t* data_stream)
{
#if MAVLINK_NEED_BYTE_SWAP
	data_stream->message_rate = mavlink_msg_data_stream_get_message_rate(msg);
	data_stream->stream_id = mavlink_msg_data_stream_get_stream_id(msg);
	data_stream->on_off = mavlink_msg_data_stream_get_on_off(msg);
#else
	memcpy(data_stream, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_DATA_STREAM_LEN);
#endif
}
