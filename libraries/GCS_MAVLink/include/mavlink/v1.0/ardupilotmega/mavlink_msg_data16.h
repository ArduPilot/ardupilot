// MESSAGE DATA16 PACKING

#define MAVLINK_MSG_ID_DATA16 169

typedef struct __mavlink_data16_t
{
 uint8_t type; /*< data type*/
 uint8_t len; /*< data length*/
 uint8_t data[16]; /*< raw data*/
} mavlink_data16_t;

#define MAVLINK_MSG_ID_DATA16_LEN 18
#define MAVLINK_MSG_ID_169_LEN 18

#define MAVLINK_MSG_ID_DATA16_CRC 234
#define MAVLINK_MSG_ID_169_CRC 234

#define MAVLINK_MSG_DATA16_FIELD_DATA_LEN 16

#define MAVLINK_MESSAGE_INFO_DATA16 { \
	"DATA16", \
	3, \
	{  { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_data16_t, type) }, \
         { "len", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_data16_t, len) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 16, 2, offsetof(mavlink_data16_t, data) }, \
         } \
}


/**
 * @brief Pack a data16 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param type data type
 * @param len data length
 * @param data raw data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_data16_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t type, uint8_t len, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_DATA16_LEN];
	_mav_put_uint8_t(buf, 0, type);
	_mav_put_uint8_t(buf, 1, len);
	_mav_put_uint8_t_array(buf, 2, data, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DATA16_LEN);
#else
	mavlink_data16_t packet;
	packet.type = type;
	packet.len = len;
	mav_array_memcpy(packet.data, data, sizeof(uint8_t)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DATA16_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_DATA16;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DATA16_LEN, MAVLINK_MSG_ID_DATA16_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DATA16_LEN);
#endif
}

/**
 * @brief Pack a data16 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param type data type
 * @param len data length
 * @param data raw data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_data16_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t type,uint8_t len,const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_DATA16_LEN];
	_mav_put_uint8_t(buf, 0, type);
	_mav_put_uint8_t(buf, 1, len);
	_mav_put_uint8_t_array(buf, 2, data, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DATA16_LEN);
#else
	mavlink_data16_t packet;
	packet.type = type;
	packet.len = len;
	mav_array_memcpy(packet.data, data, sizeof(uint8_t)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DATA16_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_DATA16;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DATA16_LEN, MAVLINK_MSG_ID_DATA16_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DATA16_LEN);
#endif
}

/**
 * @brief Encode a data16 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param data16 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_data16_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_data16_t* data16)
{
	return mavlink_msg_data16_pack(system_id, component_id, msg, data16->type, data16->len, data16->data);
}

/**
 * @brief Encode a data16 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param data16 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_data16_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_data16_t* data16)
{
	return mavlink_msg_data16_pack_chan(system_id, component_id, chan, msg, data16->type, data16->len, data16->data);
}

/**
 * @brief Send a data16 message
 * @param chan MAVLink channel to send the message
 *
 * @param type data type
 * @param len data length
 * @param data raw data
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_data16_send(mavlink_channel_t chan, uint8_t type, uint8_t len, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_DATA16_LEN];
	_mav_put_uint8_t(buf, 0, type);
	_mav_put_uint8_t(buf, 1, len);
	_mav_put_uint8_t_array(buf, 2, data, 16);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA16, buf, MAVLINK_MSG_ID_DATA16_LEN, MAVLINK_MSG_ID_DATA16_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA16, buf, MAVLINK_MSG_ID_DATA16_LEN);
#endif
#else
	mavlink_data16_t packet;
	packet.type = type;
	packet.len = len;
	mav_array_memcpy(packet.data, data, sizeof(uint8_t)*16);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA16, (const char *)&packet, MAVLINK_MSG_ID_DATA16_LEN, MAVLINK_MSG_ID_DATA16_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA16, (const char *)&packet, MAVLINK_MSG_ID_DATA16_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_DATA16_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_data16_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t type, uint8_t len, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, type);
	_mav_put_uint8_t(buf, 1, len);
	_mav_put_uint8_t_array(buf, 2, data, 16);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA16, buf, MAVLINK_MSG_ID_DATA16_LEN, MAVLINK_MSG_ID_DATA16_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA16, buf, MAVLINK_MSG_ID_DATA16_LEN);
#endif
#else
	mavlink_data16_t *packet = (mavlink_data16_t *)msgbuf;
	packet->type = type;
	packet->len = len;
	mav_array_memcpy(packet->data, data, sizeof(uint8_t)*16);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA16, (const char *)packet, MAVLINK_MSG_ID_DATA16_LEN, MAVLINK_MSG_ID_DATA16_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA16, (const char *)packet, MAVLINK_MSG_ID_DATA16_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE DATA16 UNPACKING


/**
 * @brief Get field type from data16 message
 *
 * @return data type
 */
static inline uint8_t mavlink_msg_data16_get_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field len from data16 message
 *
 * @return data length
 */
static inline uint8_t mavlink_msg_data16_get_len(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field data from data16 message
 *
 * @return raw data
 */
static inline uint16_t mavlink_msg_data16_get_data(const mavlink_message_t* msg, uint8_t *data)
{
	return _MAV_RETURN_uint8_t_array(msg, data, 16,  2);
}

/**
 * @brief Decode a data16 message into a struct
 *
 * @param msg The message to decode
 * @param data16 C-struct to decode the message contents into
 */
static inline void mavlink_msg_data16_decode(const mavlink_message_t* msg, mavlink_data16_t* data16)
{
#if MAVLINK_NEED_BYTE_SWAP
	data16->type = mavlink_msg_data16_get_type(msg);
	data16->len = mavlink_msg_data16_get_len(msg);
	mavlink_msg_data16_get_data(msg, data16->data);
#else
	memcpy(data16, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_DATA16_LEN);
#endif
}
