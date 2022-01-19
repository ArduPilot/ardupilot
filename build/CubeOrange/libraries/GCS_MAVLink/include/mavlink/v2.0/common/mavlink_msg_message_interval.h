#pragma once
// MESSAGE MESSAGE_INTERVAL PACKING

#define MAVLINK_MSG_ID_MESSAGE_INTERVAL 244


typedef struct __mavlink_message_interval_t {
 int32_t interval_us; /*< [us] The interval between two messages. A value of -1 indicates this stream is disabled, 0 indicates it is not available, > 0 indicates the interval at which it is sent.*/
 uint16_t message_id; /*<  The ID of the requested MAVLink message. v1.0 is limited to 254 messages.*/
} mavlink_message_interval_t;

#define MAVLINK_MSG_ID_MESSAGE_INTERVAL_LEN 6
#define MAVLINK_MSG_ID_MESSAGE_INTERVAL_MIN_LEN 6
#define MAVLINK_MSG_ID_244_LEN 6
#define MAVLINK_MSG_ID_244_MIN_LEN 6

#define MAVLINK_MSG_ID_MESSAGE_INTERVAL_CRC 95
#define MAVLINK_MSG_ID_244_CRC 95



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MESSAGE_INTERVAL { \
    244, \
    "MESSAGE_INTERVAL", \
    2, \
    {  { "message_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_message_interval_t, message_id) }, \
         { "interval_us", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_message_interval_t, interval_us) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MESSAGE_INTERVAL { \
    "MESSAGE_INTERVAL", \
    2, \
    {  { "message_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_message_interval_t, message_id) }, \
         { "interval_us", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_message_interval_t, interval_us) }, \
         } \
}
#endif

/**
 * @brief Pack a message_interval message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param message_id  The ID of the requested MAVLink message. v1.0 is limited to 254 messages.
 * @param interval_us [us] The interval between two messages. A value of -1 indicates this stream is disabled, 0 indicates it is not available, > 0 indicates the interval at which it is sent.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_message_interval_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t message_id, int32_t interval_us)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MESSAGE_INTERVAL_LEN];
    _mav_put_int32_t(buf, 0, interval_us);
    _mav_put_uint16_t(buf, 4, message_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MESSAGE_INTERVAL_LEN);
#else
    mavlink_message_interval_t packet;
    packet.interval_us = interval_us;
    packet.message_id = message_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MESSAGE_INTERVAL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MESSAGE_INTERVAL;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MESSAGE_INTERVAL_MIN_LEN, MAVLINK_MSG_ID_MESSAGE_INTERVAL_LEN, MAVLINK_MSG_ID_MESSAGE_INTERVAL_CRC);
}

/**
 * @brief Pack a message_interval message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param message_id  The ID of the requested MAVLink message. v1.0 is limited to 254 messages.
 * @param interval_us [us] The interval between two messages. A value of -1 indicates this stream is disabled, 0 indicates it is not available, > 0 indicates the interval at which it is sent.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_message_interval_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t message_id,int32_t interval_us)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MESSAGE_INTERVAL_LEN];
    _mav_put_int32_t(buf, 0, interval_us);
    _mav_put_uint16_t(buf, 4, message_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MESSAGE_INTERVAL_LEN);
#else
    mavlink_message_interval_t packet;
    packet.interval_us = interval_us;
    packet.message_id = message_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MESSAGE_INTERVAL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MESSAGE_INTERVAL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MESSAGE_INTERVAL_MIN_LEN, MAVLINK_MSG_ID_MESSAGE_INTERVAL_LEN, MAVLINK_MSG_ID_MESSAGE_INTERVAL_CRC);
}

/**
 * @brief Encode a message_interval struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param message_interval C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_message_interval_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_message_interval_t* message_interval)
{
    return mavlink_msg_message_interval_pack(system_id, component_id, msg, message_interval->message_id, message_interval->interval_us);
}

/**
 * @brief Encode a message_interval struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param message_interval C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_message_interval_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_message_interval_t* message_interval)
{
    return mavlink_msg_message_interval_pack_chan(system_id, component_id, chan, msg, message_interval->message_id, message_interval->interval_us);
}

/**
 * @brief Send a message_interval message
 * @param chan MAVLink channel to send the message
 *
 * @param message_id  The ID of the requested MAVLink message. v1.0 is limited to 254 messages.
 * @param interval_us [us] The interval between two messages. A value of -1 indicates this stream is disabled, 0 indicates it is not available, > 0 indicates the interval at which it is sent.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_message_interval_send(mavlink_channel_t chan, uint16_t message_id, int32_t interval_us)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MESSAGE_INTERVAL_LEN];
    _mav_put_int32_t(buf, 0, interval_us);
    _mav_put_uint16_t(buf, 4, message_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MESSAGE_INTERVAL, buf, MAVLINK_MSG_ID_MESSAGE_INTERVAL_MIN_LEN, MAVLINK_MSG_ID_MESSAGE_INTERVAL_LEN, MAVLINK_MSG_ID_MESSAGE_INTERVAL_CRC);
#else
    mavlink_message_interval_t packet;
    packet.interval_us = interval_us;
    packet.message_id = message_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MESSAGE_INTERVAL, (const char *)&packet, MAVLINK_MSG_ID_MESSAGE_INTERVAL_MIN_LEN, MAVLINK_MSG_ID_MESSAGE_INTERVAL_LEN, MAVLINK_MSG_ID_MESSAGE_INTERVAL_CRC);
#endif
}

/**
 * @brief Send a message_interval message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_message_interval_send_struct(mavlink_channel_t chan, const mavlink_message_interval_t* message_interval)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_message_interval_send(chan, message_interval->message_id, message_interval->interval_us);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MESSAGE_INTERVAL, (const char *)message_interval, MAVLINK_MSG_ID_MESSAGE_INTERVAL_MIN_LEN, MAVLINK_MSG_ID_MESSAGE_INTERVAL_LEN, MAVLINK_MSG_ID_MESSAGE_INTERVAL_CRC);
#endif
}

#if MAVLINK_MSG_ID_MESSAGE_INTERVAL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_message_interval_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t message_id, int32_t interval_us)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, interval_us);
    _mav_put_uint16_t(buf, 4, message_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MESSAGE_INTERVAL, buf, MAVLINK_MSG_ID_MESSAGE_INTERVAL_MIN_LEN, MAVLINK_MSG_ID_MESSAGE_INTERVAL_LEN, MAVLINK_MSG_ID_MESSAGE_INTERVAL_CRC);
#else
    mavlink_message_interval_t *packet = (mavlink_message_interval_t *)msgbuf;
    packet->interval_us = interval_us;
    packet->message_id = message_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MESSAGE_INTERVAL, (const char *)packet, MAVLINK_MSG_ID_MESSAGE_INTERVAL_MIN_LEN, MAVLINK_MSG_ID_MESSAGE_INTERVAL_LEN, MAVLINK_MSG_ID_MESSAGE_INTERVAL_CRC);
#endif
}
#endif

#endif

// MESSAGE MESSAGE_INTERVAL UNPACKING


/**
 * @brief Get field message_id from message_interval message
 *
 * @return  The ID of the requested MAVLink message. v1.0 is limited to 254 messages.
 */
static inline uint16_t mavlink_msg_message_interval_get_message_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field interval_us from message_interval message
 *
 * @return [us] The interval between two messages. A value of -1 indicates this stream is disabled, 0 indicates it is not available, > 0 indicates the interval at which it is sent.
 */
static inline int32_t mavlink_msg_message_interval_get_interval_us(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Decode a message_interval message into a struct
 *
 * @param msg The message to decode
 * @param message_interval C-struct to decode the message contents into
 */
static inline void mavlink_msg_message_interval_decode(const mavlink_message_t* msg, mavlink_message_interval_t* message_interval)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    message_interval->interval_us = mavlink_msg_message_interval_get_interval_us(msg);
    message_interval->message_id = mavlink_msg_message_interval_get_message_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MESSAGE_INTERVAL_LEN? msg->len : MAVLINK_MSG_ID_MESSAGE_INTERVAL_LEN;
        memset(message_interval, 0, MAVLINK_MSG_ID_MESSAGE_INTERVAL_LEN);
    memcpy(message_interval, _MAV_PAYLOAD(msg), len);
#endif
}
