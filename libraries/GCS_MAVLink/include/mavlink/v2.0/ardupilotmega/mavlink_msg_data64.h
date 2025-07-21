#pragma once
// MESSAGE DATA64 PACKING

#define MAVLINK_MSG_ID_DATA64 171


typedef struct __mavlink_data64_t {
 uint8_t type; /*<  Data type.*/
 uint8_t len; /*< [bytes] Data length.*/
 uint8_t data[64]; /*<  Raw data.*/
} mavlink_data64_t;

#define MAVLINK_MSG_ID_DATA64_LEN 66
#define MAVLINK_MSG_ID_DATA64_MIN_LEN 66
#define MAVLINK_MSG_ID_171_LEN 66
#define MAVLINK_MSG_ID_171_MIN_LEN 66

#define MAVLINK_MSG_ID_DATA64_CRC 181
#define MAVLINK_MSG_ID_171_CRC 181

#define MAVLINK_MSG_DATA64_FIELD_DATA_LEN 64

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DATA64 { \
    171, \
    "DATA64", \
    3, \
    {  { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_data64_t, type) }, \
         { "len", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_data64_t, len) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 64, 2, offsetof(mavlink_data64_t, data) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DATA64 { \
    "DATA64", \
    3, \
    {  { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_data64_t, type) }, \
         { "len", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_data64_t, len) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 64, 2, offsetof(mavlink_data64_t, data) }, \
         } \
}
#endif

/**
 * @brief Pack a data64 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param type  Data type.
 * @param len [bytes] Data length.
 * @param data  Raw data.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_data64_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t type, uint8_t len, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DATA64_LEN];
    _mav_put_uint8_t(buf, 0, type);
    _mav_put_uint8_t(buf, 1, len);
    _mav_put_uint8_t_array(buf, 2, data, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DATA64_LEN);
#else
    mavlink_data64_t packet;
    packet.type = type;
    packet.len = len;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DATA64_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DATA64;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DATA64_MIN_LEN, MAVLINK_MSG_ID_DATA64_LEN, MAVLINK_MSG_ID_DATA64_CRC);
}

/**
 * @brief Pack a data64 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param type  Data type.
 * @param len [bytes] Data length.
 * @param data  Raw data.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_data64_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t type, uint8_t len, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DATA64_LEN];
    _mav_put_uint8_t(buf, 0, type);
    _mav_put_uint8_t(buf, 1, len);
    _mav_put_uint8_t_array(buf, 2, data, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DATA64_LEN);
#else
    mavlink_data64_t packet;
    packet.type = type;
    packet.len = len;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DATA64_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DATA64;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DATA64_MIN_LEN, MAVLINK_MSG_ID_DATA64_LEN, MAVLINK_MSG_ID_DATA64_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DATA64_MIN_LEN, MAVLINK_MSG_ID_DATA64_LEN);
#endif
}

/**
 * @brief Pack a data64 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param type  Data type.
 * @param len [bytes] Data length.
 * @param data  Raw data.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_data64_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t type,uint8_t len,const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DATA64_LEN];
    _mav_put_uint8_t(buf, 0, type);
    _mav_put_uint8_t(buf, 1, len);
    _mav_put_uint8_t_array(buf, 2, data, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DATA64_LEN);
#else
    mavlink_data64_t packet;
    packet.type = type;
    packet.len = len;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DATA64_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DATA64;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DATA64_MIN_LEN, MAVLINK_MSG_ID_DATA64_LEN, MAVLINK_MSG_ID_DATA64_CRC);
}

/**
 * @brief Encode a data64 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param data64 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_data64_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_data64_t* data64)
{
    return mavlink_msg_data64_pack(system_id, component_id, msg, data64->type, data64->len, data64->data);
}

/**
 * @brief Encode a data64 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param data64 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_data64_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_data64_t* data64)
{
    return mavlink_msg_data64_pack_chan(system_id, component_id, chan, msg, data64->type, data64->len, data64->data);
}

/**
 * @brief Encode a data64 struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param data64 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_data64_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_data64_t* data64)
{
    return mavlink_msg_data64_pack_status(system_id, component_id, _status, msg,  data64->type, data64->len, data64->data);
}

/**
 * @brief Send a data64 message
 * @param chan MAVLink channel to send the message
 *
 * @param type  Data type.
 * @param len [bytes] Data length.
 * @param data  Raw data.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_data64_send(mavlink_channel_t chan, uint8_t type, uint8_t len, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DATA64_LEN];
    _mav_put_uint8_t(buf, 0, type);
    _mav_put_uint8_t(buf, 1, len);
    _mav_put_uint8_t_array(buf, 2, data, 64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA64, buf, MAVLINK_MSG_ID_DATA64_MIN_LEN, MAVLINK_MSG_ID_DATA64_LEN, MAVLINK_MSG_ID_DATA64_CRC);
#else
    mavlink_data64_t packet;
    packet.type = type;
    packet.len = len;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA64, (const char *)&packet, MAVLINK_MSG_ID_DATA64_MIN_LEN, MAVLINK_MSG_ID_DATA64_LEN, MAVLINK_MSG_ID_DATA64_CRC);
#endif
}

/**
 * @brief Send a data64 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_data64_send_struct(mavlink_channel_t chan, const mavlink_data64_t* data64)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_data64_send(chan, data64->type, data64->len, data64->data);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA64, (const char *)data64, MAVLINK_MSG_ID_DATA64_MIN_LEN, MAVLINK_MSG_ID_DATA64_LEN, MAVLINK_MSG_ID_DATA64_CRC);
#endif
}

#if MAVLINK_MSG_ID_DATA64_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_data64_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t type, uint8_t len, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, type);
    _mav_put_uint8_t(buf, 1, len);
    _mav_put_uint8_t_array(buf, 2, data, 64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA64, buf, MAVLINK_MSG_ID_DATA64_MIN_LEN, MAVLINK_MSG_ID_DATA64_LEN, MAVLINK_MSG_ID_DATA64_CRC);
#else
    mavlink_data64_t *packet = (mavlink_data64_t *)msgbuf;
    packet->type = type;
    packet->len = len;
    mav_array_memcpy(packet->data, data, sizeof(uint8_t)*64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA64, (const char *)packet, MAVLINK_MSG_ID_DATA64_MIN_LEN, MAVLINK_MSG_ID_DATA64_LEN, MAVLINK_MSG_ID_DATA64_CRC);
#endif
}
#endif

#endif

// MESSAGE DATA64 UNPACKING


/**
 * @brief Get field type from data64 message
 *
 * @return  Data type.
 */
static inline uint8_t mavlink_msg_data64_get_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field len from data64 message
 *
 * @return [bytes] Data length.
 */
static inline uint8_t mavlink_msg_data64_get_len(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field data from data64 message
 *
 * @return  Raw data.
 */
static inline uint16_t mavlink_msg_data64_get_data(const mavlink_message_t* msg, uint8_t *data)
{
    return _MAV_RETURN_uint8_t_array(msg, data, 64,  2);
}

/**
 * @brief Decode a data64 message into a struct
 *
 * @param msg The message to decode
 * @param data64 C-struct to decode the message contents into
 */
static inline void mavlink_msg_data64_decode(const mavlink_message_t* msg, mavlink_data64_t* data64)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    data64->type = mavlink_msg_data64_get_type(msg);
    data64->len = mavlink_msg_data64_get_len(msg);
    mavlink_msg_data64_get_data(msg, data64->data);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DATA64_LEN? msg->len : MAVLINK_MSG_ID_DATA64_LEN;
        memset(data64, 0, MAVLINK_MSG_ID_DATA64_LEN);
    memcpy(data64, _MAV_PAYLOAD(msg), len);
#endif
}
