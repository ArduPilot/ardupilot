#pragma once
// MESSAGE RALLY_FETCH_POINT PACKING

#define MAVLINK_MSG_ID_RALLY_FETCH_POINT 176


typedef struct __mavlink_rally_fetch_point_t {
 uint8_t target_system; /*<  System ID.*/
 uint8_t target_component; /*<  Component ID.*/
 uint8_t idx; /*<  Point index (first point is 0).*/
} mavlink_rally_fetch_point_t;

#define MAVLINK_MSG_ID_RALLY_FETCH_POINT_LEN 3
#define MAVLINK_MSG_ID_RALLY_FETCH_POINT_MIN_LEN 3
#define MAVLINK_MSG_ID_176_LEN 3
#define MAVLINK_MSG_ID_176_MIN_LEN 3

#define MAVLINK_MSG_ID_RALLY_FETCH_POINT_CRC 234
#define MAVLINK_MSG_ID_176_CRC 234



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RALLY_FETCH_POINT { \
    176, \
    "RALLY_FETCH_POINT", \
    3, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_rally_fetch_point_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_rally_fetch_point_t, target_component) }, \
         { "idx", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_rally_fetch_point_t, idx) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RALLY_FETCH_POINT { \
    "RALLY_FETCH_POINT", \
    3, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_rally_fetch_point_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_rally_fetch_point_t, target_component) }, \
         { "idx", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_rally_fetch_point_t, idx) }, \
         } \
}
#endif

/**
 * @brief Pack a rally_fetch_point message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param idx  Point index (first point is 0).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rally_fetch_point_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t idx)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RALLY_FETCH_POINT_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, idx);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RALLY_FETCH_POINT_LEN);
#else
    mavlink_rally_fetch_point_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.idx = idx;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RALLY_FETCH_POINT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RALLY_FETCH_POINT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RALLY_FETCH_POINT_MIN_LEN, MAVLINK_MSG_ID_RALLY_FETCH_POINT_LEN, MAVLINK_MSG_ID_RALLY_FETCH_POINT_CRC);
}

/**
 * @brief Pack a rally_fetch_point message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param idx  Point index (first point is 0).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rally_fetch_point_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t idx)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RALLY_FETCH_POINT_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, idx);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RALLY_FETCH_POINT_LEN);
#else
    mavlink_rally_fetch_point_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.idx = idx;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RALLY_FETCH_POINT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RALLY_FETCH_POINT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_RALLY_FETCH_POINT_MIN_LEN, MAVLINK_MSG_ID_RALLY_FETCH_POINT_LEN, MAVLINK_MSG_ID_RALLY_FETCH_POINT_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_RALLY_FETCH_POINT_MIN_LEN, MAVLINK_MSG_ID_RALLY_FETCH_POINT_LEN);
#endif
}

/**
 * @brief Pack a rally_fetch_point message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param idx  Point index (first point is 0).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rally_fetch_point_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint8_t idx)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RALLY_FETCH_POINT_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, idx);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RALLY_FETCH_POINT_LEN);
#else
    mavlink_rally_fetch_point_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.idx = idx;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RALLY_FETCH_POINT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RALLY_FETCH_POINT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RALLY_FETCH_POINT_MIN_LEN, MAVLINK_MSG_ID_RALLY_FETCH_POINT_LEN, MAVLINK_MSG_ID_RALLY_FETCH_POINT_CRC);
}

/**
 * @brief Encode a rally_fetch_point struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rally_fetch_point C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rally_fetch_point_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rally_fetch_point_t* rally_fetch_point)
{
    return mavlink_msg_rally_fetch_point_pack(system_id, component_id, msg, rally_fetch_point->target_system, rally_fetch_point->target_component, rally_fetch_point->idx);
}

/**
 * @brief Encode a rally_fetch_point struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rally_fetch_point C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rally_fetch_point_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rally_fetch_point_t* rally_fetch_point)
{
    return mavlink_msg_rally_fetch_point_pack_chan(system_id, component_id, chan, msg, rally_fetch_point->target_system, rally_fetch_point->target_component, rally_fetch_point->idx);
}

/**
 * @brief Encode a rally_fetch_point struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param rally_fetch_point C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rally_fetch_point_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_rally_fetch_point_t* rally_fetch_point)
{
    return mavlink_msg_rally_fetch_point_pack_status(system_id, component_id, _status, msg,  rally_fetch_point->target_system, rally_fetch_point->target_component, rally_fetch_point->idx);
}

/**
 * @brief Send a rally_fetch_point message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param idx  Point index (first point is 0).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rally_fetch_point_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t idx)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RALLY_FETCH_POINT_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, idx);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RALLY_FETCH_POINT, buf, MAVLINK_MSG_ID_RALLY_FETCH_POINT_MIN_LEN, MAVLINK_MSG_ID_RALLY_FETCH_POINT_LEN, MAVLINK_MSG_ID_RALLY_FETCH_POINT_CRC);
#else
    mavlink_rally_fetch_point_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.idx = idx;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RALLY_FETCH_POINT, (const char *)&packet, MAVLINK_MSG_ID_RALLY_FETCH_POINT_MIN_LEN, MAVLINK_MSG_ID_RALLY_FETCH_POINT_LEN, MAVLINK_MSG_ID_RALLY_FETCH_POINT_CRC);
#endif
}

/**
 * @brief Send a rally_fetch_point message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_rally_fetch_point_send_struct(mavlink_channel_t chan, const mavlink_rally_fetch_point_t* rally_fetch_point)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_rally_fetch_point_send(chan, rally_fetch_point->target_system, rally_fetch_point->target_component, rally_fetch_point->idx);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RALLY_FETCH_POINT, (const char *)rally_fetch_point, MAVLINK_MSG_ID_RALLY_FETCH_POINT_MIN_LEN, MAVLINK_MSG_ID_RALLY_FETCH_POINT_LEN, MAVLINK_MSG_ID_RALLY_FETCH_POINT_CRC);
#endif
}

#if MAVLINK_MSG_ID_RALLY_FETCH_POINT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rally_fetch_point_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t idx)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, idx);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RALLY_FETCH_POINT, buf, MAVLINK_MSG_ID_RALLY_FETCH_POINT_MIN_LEN, MAVLINK_MSG_ID_RALLY_FETCH_POINT_LEN, MAVLINK_MSG_ID_RALLY_FETCH_POINT_CRC);
#else
    mavlink_rally_fetch_point_t *packet = (mavlink_rally_fetch_point_t *)msgbuf;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->idx = idx;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RALLY_FETCH_POINT, (const char *)packet, MAVLINK_MSG_ID_RALLY_FETCH_POINT_MIN_LEN, MAVLINK_MSG_ID_RALLY_FETCH_POINT_LEN, MAVLINK_MSG_ID_RALLY_FETCH_POINT_CRC);
#endif
}
#endif

#endif

// MESSAGE RALLY_FETCH_POINT UNPACKING


/**
 * @brief Get field target_system from rally_fetch_point message
 *
 * @return  System ID.
 */
static inline uint8_t mavlink_msg_rally_fetch_point_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field target_component from rally_fetch_point message
 *
 * @return  Component ID.
 */
static inline uint8_t mavlink_msg_rally_fetch_point_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field idx from rally_fetch_point message
 *
 * @return  Point index (first point is 0).
 */
static inline uint8_t mavlink_msg_rally_fetch_point_get_idx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a rally_fetch_point message into a struct
 *
 * @param msg The message to decode
 * @param rally_fetch_point C-struct to decode the message contents into
 */
static inline void mavlink_msg_rally_fetch_point_decode(const mavlink_message_t* msg, mavlink_rally_fetch_point_t* rally_fetch_point)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    rally_fetch_point->target_system = mavlink_msg_rally_fetch_point_get_target_system(msg);
    rally_fetch_point->target_component = mavlink_msg_rally_fetch_point_get_target_component(msg);
    rally_fetch_point->idx = mavlink_msg_rally_fetch_point_get_idx(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RALLY_FETCH_POINT_LEN? msg->len : MAVLINK_MSG_ID_RALLY_FETCH_POINT_LEN;
        memset(rally_fetch_point, 0, MAVLINK_MSG_ID_RALLY_FETCH_POINT_LEN);
    memcpy(rally_fetch_point, _MAV_PAYLOAD(msg), len);
#endif
}
