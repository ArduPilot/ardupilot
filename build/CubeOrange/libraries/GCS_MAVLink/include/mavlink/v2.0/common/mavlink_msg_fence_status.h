#pragma once
// MESSAGE FENCE_STATUS PACKING

#define MAVLINK_MSG_ID_FENCE_STATUS 162


typedef struct __mavlink_fence_status_t {
 uint32_t breach_time; /*< [ms] Time (since boot) of last breach.*/
 uint16_t breach_count; /*<  Number of fence breaches.*/
 uint8_t breach_status; /*<  Breach status (0 if currently inside fence, 1 if outside).*/
 uint8_t breach_type; /*<  Last breach type.*/
 uint8_t breach_mitigation; /*<  Active action to prevent fence breach*/
} mavlink_fence_status_t;

#define MAVLINK_MSG_ID_FENCE_STATUS_LEN 9
#define MAVLINK_MSG_ID_FENCE_STATUS_MIN_LEN 8
#define MAVLINK_MSG_ID_162_LEN 9
#define MAVLINK_MSG_ID_162_MIN_LEN 8

#define MAVLINK_MSG_ID_FENCE_STATUS_CRC 189
#define MAVLINK_MSG_ID_162_CRC 189



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FENCE_STATUS { \
    162, \
    "FENCE_STATUS", \
    5, \
    {  { "breach_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_fence_status_t, breach_status) }, \
         { "breach_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_fence_status_t, breach_count) }, \
         { "breach_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_fence_status_t, breach_type) }, \
         { "breach_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_fence_status_t, breach_time) }, \
         { "breach_mitigation", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_fence_status_t, breach_mitigation) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FENCE_STATUS { \
    "FENCE_STATUS", \
    5, \
    {  { "breach_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_fence_status_t, breach_status) }, \
         { "breach_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_fence_status_t, breach_count) }, \
         { "breach_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_fence_status_t, breach_type) }, \
         { "breach_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_fence_status_t, breach_time) }, \
         { "breach_mitigation", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_fence_status_t, breach_mitigation) }, \
         } \
}
#endif

/**
 * @brief Pack a fence_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param breach_status  Breach status (0 if currently inside fence, 1 if outside).
 * @param breach_count  Number of fence breaches.
 * @param breach_type  Last breach type.
 * @param breach_time [ms] Time (since boot) of last breach.
 * @param breach_mitigation  Active action to prevent fence breach
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fence_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t breach_status, uint16_t breach_count, uint8_t breach_type, uint32_t breach_time, uint8_t breach_mitigation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FENCE_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, breach_time);
    _mav_put_uint16_t(buf, 4, breach_count);
    _mav_put_uint8_t(buf, 6, breach_status);
    _mav_put_uint8_t(buf, 7, breach_type);
    _mav_put_uint8_t(buf, 8, breach_mitigation);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FENCE_STATUS_LEN);
#else
    mavlink_fence_status_t packet;
    packet.breach_time = breach_time;
    packet.breach_count = breach_count;
    packet.breach_status = breach_status;
    packet.breach_type = breach_type;
    packet.breach_mitigation = breach_mitigation;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FENCE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FENCE_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FENCE_STATUS_MIN_LEN, MAVLINK_MSG_ID_FENCE_STATUS_LEN, MAVLINK_MSG_ID_FENCE_STATUS_CRC);
}

/**
 * @brief Pack a fence_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param breach_status  Breach status (0 if currently inside fence, 1 if outside).
 * @param breach_count  Number of fence breaches.
 * @param breach_type  Last breach type.
 * @param breach_time [ms] Time (since boot) of last breach.
 * @param breach_mitigation  Active action to prevent fence breach
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fence_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t breach_status,uint16_t breach_count,uint8_t breach_type,uint32_t breach_time,uint8_t breach_mitigation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FENCE_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, breach_time);
    _mav_put_uint16_t(buf, 4, breach_count);
    _mav_put_uint8_t(buf, 6, breach_status);
    _mav_put_uint8_t(buf, 7, breach_type);
    _mav_put_uint8_t(buf, 8, breach_mitigation);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FENCE_STATUS_LEN);
#else
    mavlink_fence_status_t packet;
    packet.breach_time = breach_time;
    packet.breach_count = breach_count;
    packet.breach_status = breach_status;
    packet.breach_type = breach_type;
    packet.breach_mitigation = breach_mitigation;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FENCE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FENCE_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FENCE_STATUS_MIN_LEN, MAVLINK_MSG_ID_FENCE_STATUS_LEN, MAVLINK_MSG_ID_FENCE_STATUS_CRC);
}

/**
 * @brief Encode a fence_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param fence_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_fence_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_fence_status_t* fence_status)
{
    return mavlink_msg_fence_status_pack(system_id, component_id, msg, fence_status->breach_status, fence_status->breach_count, fence_status->breach_type, fence_status->breach_time, fence_status->breach_mitigation);
}

/**
 * @brief Encode a fence_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param fence_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_fence_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_fence_status_t* fence_status)
{
    return mavlink_msg_fence_status_pack_chan(system_id, component_id, chan, msg, fence_status->breach_status, fence_status->breach_count, fence_status->breach_type, fence_status->breach_time, fence_status->breach_mitigation);
}

/**
 * @brief Send a fence_status message
 * @param chan MAVLink channel to send the message
 *
 * @param breach_status  Breach status (0 if currently inside fence, 1 if outside).
 * @param breach_count  Number of fence breaches.
 * @param breach_type  Last breach type.
 * @param breach_time [ms] Time (since boot) of last breach.
 * @param breach_mitigation  Active action to prevent fence breach
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_fence_status_send(mavlink_channel_t chan, uint8_t breach_status, uint16_t breach_count, uint8_t breach_type, uint32_t breach_time, uint8_t breach_mitigation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FENCE_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, breach_time);
    _mav_put_uint16_t(buf, 4, breach_count);
    _mav_put_uint8_t(buf, 6, breach_status);
    _mav_put_uint8_t(buf, 7, breach_type);
    _mav_put_uint8_t(buf, 8, breach_mitigation);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FENCE_STATUS, buf, MAVLINK_MSG_ID_FENCE_STATUS_MIN_LEN, MAVLINK_MSG_ID_FENCE_STATUS_LEN, MAVLINK_MSG_ID_FENCE_STATUS_CRC);
#else
    mavlink_fence_status_t packet;
    packet.breach_time = breach_time;
    packet.breach_count = breach_count;
    packet.breach_status = breach_status;
    packet.breach_type = breach_type;
    packet.breach_mitigation = breach_mitigation;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FENCE_STATUS, (const char *)&packet, MAVLINK_MSG_ID_FENCE_STATUS_MIN_LEN, MAVLINK_MSG_ID_FENCE_STATUS_LEN, MAVLINK_MSG_ID_FENCE_STATUS_CRC);
#endif
}

/**
 * @brief Send a fence_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_fence_status_send_struct(mavlink_channel_t chan, const mavlink_fence_status_t* fence_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_fence_status_send(chan, fence_status->breach_status, fence_status->breach_count, fence_status->breach_type, fence_status->breach_time, fence_status->breach_mitigation);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FENCE_STATUS, (const char *)fence_status, MAVLINK_MSG_ID_FENCE_STATUS_MIN_LEN, MAVLINK_MSG_ID_FENCE_STATUS_LEN, MAVLINK_MSG_ID_FENCE_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_FENCE_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_fence_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t breach_status, uint16_t breach_count, uint8_t breach_type, uint32_t breach_time, uint8_t breach_mitigation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, breach_time);
    _mav_put_uint16_t(buf, 4, breach_count);
    _mav_put_uint8_t(buf, 6, breach_status);
    _mav_put_uint8_t(buf, 7, breach_type);
    _mav_put_uint8_t(buf, 8, breach_mitigation);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FENCE_STATUS, buf, MAVLINK_MSG_ID_FENCE_STATUS_MIN_LEN, MAVLINK_MSG_ID_FENCE_STATUS_LEN, MAVLINK_MSG_ID_FENCE_STATUS_CRC);
#else
    mavlink_fence_status_t *packet = (mavlink_fence_status_t *)msgbuf;
    packet->breach_time = breach_time;
    packet->breach_count = breach_count;
    packet->breach_status = breach_status;
    packet->breach_type = breach_type;
    packet->breach_mitigation = breach_mitigation;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FENCE_STATUS, (const char *)packet, MAVLINK_MSG_ID_FENCE_STATUS_MIN_LEN, MAVLINK_MSG_ID_FENCE_STATUS_LEN, MAVLINK_MSG_ID_FENCE_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE FENCE_STATUS UNPACKING


/**
 * @brief Get field breach_status from fence_status message
 *
 * @return  Breach status (0 if currently inside fence, 1 if outside).
 */
static inline uint8_t mavlink_msg_fence_status_get_breach_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field breach_count from fence_status message
 *
 * @return  Number of fence breaches.
 */
static inline uint16_t mavlink_msg_fence_status_get_breach_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field breach_type from fence_status message
 *
 * @return  Last breach type.
 */
static inline uint8_t mavlink_msg_fence_status_get_breach_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field breach_time from fence_status message
 *
 * @return [ms] Time (since boot) of last breach.
 */
static inline uint32_t mavlink_msg_fence_status_get_breach_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field breach_mitigation from fence_status message
 *
 * @return  Active action to prevent fence breach
 */
static inline uint8_t mavlink_msg_fence_status_get_breach_mitigation(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Decode a fence_status message into a struct
 *
 * @param msg The message to decode
 * @param fence_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_fence_status_decode(const mavlink_message_t* msg, mavlink_fence_status_t* fence_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    fence_status->breach_time = mavlink_msg_fence_status_get_breach_time(msg);
    fence_status->breach_count = mavlink_msg_fence_status_get_breach_count(msg);
    fence_status->breach_status = mavlink_msg_fence_status_get_breach_status(msg);
    fence_status->breach_type = mavlink_msg_fence_status_get_breach_type(msg);
    fence_status->breach_mitigation = mavlink_msg_fence_status_get_breach_mitigation(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FENCE_STATUS_LEN? msg->len : MAVLINK_MSG_ID_FENCE_STATUS_LEN;
        memset(fence_status, 0, MAVLINK_MSG_ID_FENCE_STATUS_LEN);
    memcpy(fence_status, _MAV_PAYLOAD(msg), len);
#endif
}
