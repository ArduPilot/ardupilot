#pragma once
// MESSAGE DATA_STREAM PACKING

#define MAVLINK_MSG_ID_DATA_STREAM 67


typedef struct __mavlink_data_stream_t {
 uint16_t message_rate; /*< [Hz] The message rate*/
 uint8_t stream_id; /*<  The ID of the requested data stream*/
 uint8_t on_off; /*<  1 stream is enabled, 0 stream is stopped.*/
} mavlink_data_stream_t;

#define MAVLINK_MSG_ID_DATA_STREAM_LEN 4
#define MAVLINK_MSG_ID_DATA_STREAM_MIN_LEN 4
#define MAVLINK_MSG_ID_67_LEN 4
#define MAVLINK_MSG_ID_67_MIN_LEN 4

#define MAVLINK_MSG_ID_DATA_STREAM_CRC 21
#define MAVLINK_MSG_ID_67_CRC 21



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DATA_STREAM { \
    67, \
    "DATA_STREAM", \
    3, \
    {  { "stream_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_data_stream_t, stream_id) }, \
         { "message_rate", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_data_stream_t, message_rate) }, \
         { "on_off", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_data_stream_t, on_off) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DATA_STREAM { \
    "DATA_STREAM", \
    3, \
    {  { "stream_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_data_stream_t, stream_id) }, \
         { "message_rate", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_data_stream_t, message_rate) }, \
         { "on_off", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_data_stream_t, on_off) }, \
         } \
}
#endif

/**
 * @brief Pack a data_stream message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param stream_id  The ID of the requested data stream
 * @param message_rate [Hz] The message rate
 * @param on_off  1 stream is enabled, 0 stream is stopped.
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
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DATA_STREAM_MIN_LEN, MAVLINK_MSG_ID_DATA_STREAM_LEN, MAVLINK_MSG_ID_DATA_STREAM_CRC);
}

/**
 * @brief Pack a data_stream message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param stream_id  The ID of the requested data stream
 * @param message_rate [Hz] The message rate
 * @param on_off  1 stream is enabled, 0 stream is stopped.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_data_stream_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
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
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DATA_STREAM_MIN_LEN, MAVLINK_MSG_ID_DATA_STREAM_LEN, MAVLINK_MSG_ID_DATA_STREAM_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DATA_STREAM_MIN_LEN, MAVLINK_MSG_ID_DATA_STREAM_LEN);
#endif
}

/**
 * @brief Pack a data_stream message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param stream_id  The ID of the requested data stream
 * @param message_rate [Hz] The message rate
 * @param on_off  1 stream is enabled, 0 stream is stopped.
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
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DATA_STREAM_MIN_LEN, MAVLINK_MSG_ID_DATA_STREAM_LEN, MAVLINK_MSG_ID_DATA_STREAM_CRC);
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
 * @brief Encode a data_stream struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param data_stream C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_data_stream_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_data_stream_t* data_stream)
{
    return mavlink_msg_data_stream_pack_status(system_id, component_id, _status, msg,  data_stream->stream_id, data_stream->message_rate, data_stream->on_off);
}

/**
 * @brief Send a data_stream message
 * @param chan MAVLink channel to send the message
 *
 * @param stream_id  The ID of the requested data stream
 * @param message_rate [Hz] The message rate
 * @param on_off  1 stream is enabled, 0 stream is stopped.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_data_stream_send(mavlink_channel_t chan, uint8_t stream_id, uint16_t message_rate, uint8_t on_off)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DATA_STREAM_LEN];
    _mav_put_uint16_t(buf, 0, message_rate);
    _mav_put_uint8_t(buf, 2, stream_id);
    _mav_put_uint8_t(buf, 3, on_off);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA_STREAM, buf, MAVLINK_MSG_ID_DATA_STREAM_MIN_LEN, MAVLINK_MSG_ID_DATA_STREAM_LEN, MAVLINK_MSG_ID_DATA_STREAM_CRC);
#else
    mavlink_data_stream_t packet;
    packet.message_rate = message_rate;
    packet.stream_id = stream_id;
    packet.on_off = on_off;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA_STREAM, (const char *)&packet, MAVLINK_MSG_ID_DATA_STREAM_MIN_LEN, MAVLINK_MSG_ID_DATA_STREAM_LEN, MAVLINK_MSG_ID_DATA_STREAM_CRC);
#endif
}

/**
 * @brief Send a data_stream message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_data_stream_send_struct(mavlink_channel_t chan, const mavlink_data_stream_t* data_stream)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_data_stream_send(chan, data_stream->stream_id, data_stream->message_rate, data_stream->on_off);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA_STREAM, (const char *)data_stream, MAVLINK_MSG_ID_DATA_STREAM_MIN_LEN, MAVLINK_MSG_ID_DATA_STREAM_LEN, MAVLINK_MSG_ID_DATA_STREAM_CRC);
#endif
}

#if MAVLINK_MSG_ID_DATA_STREAM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_data_stream_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t stream_id, uint16_t message_rate, uint8_t on_off)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, message_rate);
    _mav_put_uint8_t(buf, 2, stream_id);
    _mav_put_uint8_t(buf, 3, on_off);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA_STREAM, buf, MAVLINK_MSG_ID_DATA_STREAM_MIN_LEN, MAVLINK_MSG_ID_DATA_STREAM_LEN, MAVLINK_MSG_ID_DATA_STREAM_CRC);
#else
    mavlink_data_stream_t *packet = (mavlink_data_stream_t *)msgbuf;
    packet->message_rate = message_rate;
    packet->stream_id = stream_id;
    packet->on_off = on_off;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA_STREAM, (const char *)packet, MAVLINK_MSG_ID_DATA_STREAM_MIN_LEN, MAVLINK_MSG_ID_DATA_STREAM_LEN, MAVLINK_MSG_ID_DATA_STREAM_CRC);
#endif
}
#endif

#endif

// MESSAGE DATA_STREAM UNPACKING


/**
 * @brief Get field stream_id from data_stream message
 *
 * @return  The ID of the requested data stream
 */
static inline uint8_t mavlink_msg_data_stream_get_stream_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field message_rate from data_stream message
 *
 * @return [Hz] The message rate
 */
static inline uint16_t mavlink_msg_data_stream_get_message_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field on_off from data_stream message
 *
 * @return  1 stream is enabled, 0 stream is stopped.
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
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    data_stream->message_rate = mavlink_msg_data_stream_get_message_rate(msg);
    data_stream->stream_id = mavlink_msg_data_stream_get_stream_id(msg);
    data_stream->on_off = mavlink_msg_data_stream_get_on_off(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DATA_STREAM_LEN? msg->len : MAVLINK_MSG_ID_DATA_STREAM_LEN;
        memset(data_stream, 0, MAVLINK_MSG_ID_DATA_STREAM_LEN);
    memcpy(data_stream, _MAV_PAYLOAD(msg), len);
#endif
}
