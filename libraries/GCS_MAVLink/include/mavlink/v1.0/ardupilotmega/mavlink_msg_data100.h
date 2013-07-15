// MESSAGE DATA100 PACKING

#define MAVLINK_MSG_ID_DATA100 171

typedef struct __mavlink_data100_t
{
 uint8_t type; ///< data type
 uint8_t len; ///< data length
 uint8_t data[100]; ///< raw data
} mavlink_data100_t;

#define MAVLINK_MSG_ID_DATA100_LEN 102
#define MAVLINK_MSG_ID_171_LEN 102

#define MAVLINK_MSG_DATA100_FIELD_DATA_LEN 100

#define MAVLINK_MESSAGE_INFO_DATA100 { \
	"DATA100", \
	3, \
	{  { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_data100_t, type) }, \
         { "len", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_data100_t, len) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 100, 2, offsetof(mavlink_data100_t, data) }, \
         } \
}


/**
 * @brief Pack a data100 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param type data type
 * @param len data length
 * @param data raw data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_data100_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t type, uint8_t len, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[102];
	_mav_put_uint8_t(buf, 0, type);
	_mav_put_uint8_t(buf, 1, len);
	_mav_put_uint8_t_array(buf, 2, data, 100);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 102);
#else
	mavlink_data100_t packet;
	packet.type = type;
	packet.len = len;
	mav_array_memcpy(packet.data, data, sizeof(uint8_t)*100);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 102);
#endif

	msg->msgid = MAVLINK_MSG_ID_DATA100;
	return mavlink_finalize_message(msg, system_id, component_id, 102, 179);
}

/**
 * @brief Pack a data100 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param type data type
 * @param len data length
 * @param data raw data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_data100_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t type,uint8_t len,const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[102];
	_mav_put_uint8_t(buf, 0, type);
	_mav_put_uint8_t(buf, 1, len);
	_mav_put_uint8_t_array(buf, 2, data, 100);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 102);
#else
	mavlink_data100_t packet;
	packet.type = type;
	packet.len = len;
	mav_array_memcpy(packet.data, data, sizeof(uint8_t)*100);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 102);
#endif

	msg->msgid = MAVLINK_MSG_ID_DATA100;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 102, 179);
}

/**
 * @brief Encode a data100 struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param data100 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_data100_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_data100_t* data100)
{
	return mavlink_msg_data100_pack(system_id, component_id, msg, data100->type, data100->len, data100->data);
}

/**
 * @brief Send a data100 message
 * @param chan MAVLink channel to send the message
 *
 * @param type data type
 * @param len data length
 * @param data raw data
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_data100_send(mavlink_channel_t chan, uint8_t type, uint8_t len, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[102];
	_mav_put_uint8_t(buf, 0, type);
	_mav_put_uint8_t(buf, 1, len);
	_mav_put_uint8_t_array(buf, 2, data, 100);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA100, buf, 102, 179);
#else
	mavlink_data100_t packet;
	packet.type = type;
	packet.len = len;
	mav_array_memcpy(packet.data, data, sizeof(uint8_t)*100);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA100, (const char *)&packet, 102, 179);
#endif
}

#endif

// MESSAGE DATA100 UNPACKING


/**
 * @brief Get field type from data100 message
 *
 * @return data type
 */
static inline uint8_t mavlink_msg_data100_get_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field len from data100 message
 *
 * @return data length
 */
static inline uint8_t mavlink_msg_data100_get_len(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field data from data100 message
 *
 * @return raw data
 */
static inline uint16_t mavlink_msg_data100_get_data(const mavlink_message_t* msg, uint8_t *data)
{
	return _MAV_RETURN_uint8_t_array(msg, data, 100,  2);
}

/**
 * @brief Decode a data100 message into a struct
 *
 * @param msg The message to decode
 * @param data100 C-struct to decode the message contents into
 */
static inline void mavlink_msg_data100_decode(const mavlink_message_t* msg, mavlink_data100_t* data100)
{
#if MAVLINK_NEED_BYTE_SWAP
	data100->type = mavlink_msg_data100_get_type(msg);
	data100->len = mavlink_msg_data100_get_len(msg);
	mavlink_msg_data100_get_data(msg, data100->data);
#else
	memcpy(data100, _MAV_PAYLOAD(msg), 102);
#endif
}
