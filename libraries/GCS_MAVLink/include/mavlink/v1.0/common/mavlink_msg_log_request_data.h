// MESSAGE LOG_REQUEST_DATA PACKING

#define MAVLINK_MSG_ID_LOG_REQUEST_DATA 119

typedef struct __mavlink_log_request_data_t
{
 uint32_t ofs; ///< Offset into the log
 uint32_t count; ///< Number of bytes
 uint16_t id; ///< Log id (from LOG_ENTRY reply)
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
} mavlink_log_request_data_t;

#define MAVLINK_MSG_ID_LOG_REQUEST_DATA_LEN 12
#define MAVLINK_MSG_ID_119_LEN 12

#define MAVLINK_MSG_ID_LOG_REQUEST_DATA_CRC 116
#define MAVLINK_MSG_ID_119_CRC 116



#define MAVLINK_MESSAGE_INFO_LOG_REQUEST_DATA { \
	"LOG_REQUEST_DATA", \
	5, \
	{  { "ofs", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_log_request_data_t, ofs) }, \
         { "count", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_log_request_data_t, count) }, \
         { "id", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_log_request_data_t, id) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_log_request_data_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_log_request_data_t, target_component) }, \
         } \
}


/**
 * @brief Pack a log_request_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param id Log id (from LOG_ENTRY reply)
 * @param ofs Offset into the log
 * @param count Number of bytes
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_log_request_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, uint16_t id, uint32_t ofs, uint32_t count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_LOG_REQUEST_DATA_LEN];
	_mav_put_uint32_t(buf, 0, ofs);
	_mav_put_uint32_t(buf, 4, count);
	_mav_put_uint16_t(buf, 8, id);
	_mav_put_uint8_t(buf, 10, target_system);
	_mav_put_uint8_t(buf, 11, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LOG_REQUEST_DATA_LEN);
#else
	mavlink_log_request_data_t packet;
	packet.ofs = ofs;
	packet.count = count;
	packet.id = id;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LOG_REQUEST_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_LOG_REQUEST_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LOG_REQUEST_DATA_LEN, MAVLINK_MSG_ID_LOG_REQUEST_DATA_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LOG_REQUEST_DATA_LEN);
#endif
}

/**
 * @brief Pack a log_request_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param id Log id (from LOG_ENTRY reply)
 * @param ofs Offset into the log
 * @param count Number of bytes
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_log_request_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,uint16_t id,uint32_t ofs,uint32_t count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_LOG_REQUEST_DATA_LEN];
	_mav_put_uint32_t(buf, 0, ofs);
	_mav_put_uint32_t(buf, 4, count);
	_mav_put_uint16_t(buf, 8, id);
	_mav_put_uint8_t(buf, 10, target_system);
	_mav_put_uint8_t(buf, 11, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LOG_REQUEST_DATA_LEN);
#else
	mavlink_log_request_data_t packet;
	packet.ofs = ofs;
	packet.count = count;
	packet.id = id;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LOG_REQUEST_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_LOG_REQUEST_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LOG_REQUEST_DATA_LEN, MAVLINK_MSG_ID_LOG_REQUEST_DATA_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LOG_REQUEST_DATA_LEN);
#endif
}

/**
 * @brief Encode a log_request_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param log_request_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_log_request_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_log_request_data_t* log_request_data)
{
	return mavlink_msg_log_request_data_pack(system_id, component_id, msg, log_request_data->target_system, log_request_data->target_component, log_request_data->id, log_request_data->ofs, log_request_data->count);
}

/**
 * @brief Encode a log_request_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param log_request_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_log_request_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_log_request_data_t* log_request_data)
{
	return mavlink_msg_log_request_data_pack_chan(system_id, component_id, chan, msg, log_request_data->target_system, log_request_data->target_component, log_request_data->id, log_request_data->ofs, log_request_data->count);
}

/**
 * @brief Send a log_request_data message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param id Log id (from LOG_ENTRY reply)
 * @param ofs Offset into the log
 * @param count Number of bytes
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_log_request_data_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint16_t id, uint32_t ofs, uint32_t count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_LOG_REQUEST_DATA_LEN];
	_mav_put_uint32_t(buf, 0, ofs);
	_mav_put_uint32_t(buf, 4, count);
	_mav_put_uint16_t(buf, 8, id);
	_mav_put_uint8_t(buf, 10, target_system);
	_mav_put_uint8_t(buf, 11, target_component);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOG_REQUEST_DATA, buf, MAVLINK_MSG_ID_LOG_REQUEST_DATA_LEN, MAVLINK_MSG_ID_LOG_REQUEST_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOG_REQUEST_DATA, buf, MAVLINK_MSG_ID_LOG_REQUEST_DATA_LEN);
#endif
#else
	mavlink_log_request_data_t packet;
	packet.ofs = ofs;
	packet.count = count;
	packet.id = id;
	packet.target_system = target_system;
	packet.target_component = target_component;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOG_REQUEST_DATA, (const char *)&packet, MAVLINK_MSG_ID_LOG_REQUEST_DATA_LEN, MAVLINK_MSG_ID_LOG_REQUEST_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOG_REQUEST_DATA, (const char *)&packet, MAVLINK_MSG_ID_LOG_REQUEST_DATA_LEN);
#endif
#endif
}

#endif

// MESSAGE LOG_REQUEST_DATA UNPACKING


/**
 * @brief Get field target_system from log_request_data message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_log_request_data_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field target_component from log_request_data message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_log_request_data_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field id from log_request_data message
 *
 * @return Log id (from LOG_ENTRY reply)
 */
static inline uint16_t mavlink_msg_log_request_data_get_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field ofs from log_request_data message
 *
 * @return Offset into the log
 */
static inline uint32_t mavlink_msg_log_request_data_get_ofs(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field count from log_request_data message
 *
 * @return Number of bytes
 */
static inline uint32_t mavlink_msg_log_request_data_get_count(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Decode a log_request_data message into a struct
 *
 * @param msg The message to decode
 * @param log_request_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_log_request_data_decode(const mavlink_message_t* msg, mavlink_log_request_data_t* log_request_data)
{
#if MAVLINK_NEED_BYTE_SWAP
	log_request_data->ofs = mavlink_msg_log_request_data_get_ofs(msg);
	log_request_data->count = mavlink_msg_log_request_data_get_count(msg);
	log_request_data->id = mavlink_msg_log_request_data_get_id(msg);
	log_request_data->target_system = mavlink_msg_log_request_data_get_target_system(msg);
	log_request_data->target_component = mavlink_msg_log_request_data_get_target_component(msg);
#else
	memcpy(log_request_data, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_LOG_REQUEST_DATA_LEN);
#endif
}
