#pragma once
// MESSAGE FENCE_POINT PACKING

#define MAVLINK_MSG_ID_FENCE_POINT 160

MAVPACKED(
typedef struct __mavlink_fence_point_t {
 float lat; /*< Latitude of point*/
 float lng; /*< Longitude of point*/
 uint8_t target_system; /*< System ID*/
 uint8_t target_component; /*< Component ID*/
 uint8_t idx; /*< point index (first point is 1, 0 is for return point)*/
 uint8_t count; /*< total number of points (for sanity checking)*/
}) mavlink_fence_point_t;

#define MAVLINK_MSG_ID_FENCE_POINT_LEN 12
#define MAVLINK_MSG_ID_FENCE_POINT_MIN_LEN 12
#define MAVLINK_MSG_ID_160_LEN 12
#define MAVLINK_MSG_ID_160_MIN_LEN 12

#define MAVLINK_MSG_ID_FENCE_POINT_CRC 78
#define MAVLINK_MSG_ID_160_CRC 78



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FENCE_POINT { \
    160, \
    "FENCE_POINT", \
    6, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_fence_point_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_fence_point_t, target_component) }, \
         { "idx", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_fence_point_t, idx) }, \
         { "count", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_fence_point_t, count) }, \
         { "lat", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_fence_point_t, lat) }, \
         { "lng", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_fence_point_t, lng) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FENCE_POINT { \
    "FENCE_POINT", \
    6, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_fence_point_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_fence_point_t, target_component) }, \
         { "idx", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_fence_point_t, idx) }, \
         { "count", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_fence_point_t, count) }, \
         { "lat", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_fence_point_t, lat) }, \
         { "lng", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_fence_point_t, lng) }, \
         } \
}
#endif

/**
 * @brief Pack a fence_point message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param idx point index (first point is 1, 0 is for return point)
 * @param count total number of points (for sanity checking)
 * @param lat Latitude of point
 * @param lng Longitude of point
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fence_point_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t idx, uint8_t count, float lat, float lng)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FENCE_POINT_LEN];
    _mav_put_float(buf, 0, lat);
    _mav_put_float(buf, 4, lng);
    _mav_put_uint8_t(buf, 8, target_system);
    _mav_put_uint8_t(buf, 9, target_component);
    _mav_put_uint8_t(buf, 10, idx);
    _mav_put_uint8_t(buf, 11, count);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FENCE_POINT_LEN);
#else
    mavlink_fence_point_t packet;
    packet.lat = lat;
    packet.lng = lng;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.idx = idx;
    packet.count = count;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FENCE_POINT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FENCE_POINT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FENCE_POINT_MIN_LEN, MAVLINK_MSG_ID_FENCE_POINT_LEN, MAVLINK_MSG_ID_FENCE_POINT_CRC);
}

/**
 * @brief Pack a fence_point message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param idx point index (first point is 1, 0 is for return point)
 * @param count total number of points (for sanity checking)
 * @param lat Latitude of point
 * @param lng Longitude of point
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fence_point_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint8_t idx,uint8_t count,float lat,float lng)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FENCE_POINT_LEN];
    _mav_put_float(buf, 0, lat);
    _mav_put_float(buf, 4, lng);
    _mav_put_uint8_t(buf, 8, target_system);
    _mav_put_uint8_t(buf, 9, target_component);
    _mav_put_uint8_t(buf, 10, idx);
    _mav_put_uint8_t(buf, 11, count);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FENCE_POINT_LEN);
#else
    mavlink_fence_point_t packet;
    packet.lat = lat;
    packet.lng = lng;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.idx = idx;
    packet.count = count;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FENCE_POINT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FENCE_POINT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FENCE_POINT_MIN_LEN, MAVLINK_MSG_ID_FENCE_POINT_LEN, MAVLINK_MSG_ID_FENCE_POINT_CRC);
}

/**
 * @brief Encode a fence_point struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param fence_point C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_fence_point_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_fence_point_t* fence_point)
{
    return mavlink_msg_fence_point_pack(system_id, component_id, msg, fence_point->target_system, fence_point->target_component, fence_point->idx, fence_point->count, fence_point->lat, fence_point->lng);
}

/**
 * @brief Encode a fence_point struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param fence_point C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_fence_point_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_fence_point_t* fence_point)
{
    return mavlink_msg_fence_point_pack_chan(system_id, component_id, chan, msg, fence_point->target_system, fence_point->target_component, fence_point->idx, fence_point->count, fence_point->lat, fence_point->lng);
}

/**
 * @brief Send a fence_point message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param idx point index (first point is 1, 0 is for return point)
 * @param count total number of points (for sanity checking)
 * @param lat Latitude of point
 * @param lng Longitude of point
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_fence_point_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t idx, uint8_t count, float lat, float lng)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FENCE_POINT_LEN];
    _mav_put_float(buf, 0, lat);
    _mav_put_float(buf, 4, lng);
    _mav_put_uint8_t(buf, 8, target_system);
    _mav_put_uint8_t(buf, 9, target_component);
    _mav_put_uint8_t(buf, 10, idx);
    _mav_put_uint8_t(buf, 11, count);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FENCE_POINT, buf, MAVLINK_MSG_ID_FENCE_POINT_MIN_LEN, MAVLINK_MSG_ID_FENCE_POINT_LEN, MAVLINK_MSG_ID_FENCE_POINT_CRC);
#else
    mavlink_fence_point_t packet;
    packet.lat = lat;
    packet.lng = lng;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.idx = idx;
    packet.count = count;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FENCE_POINT, (const char *)&packet, MAVLINK_MSG_ID_FENCE_POINT_MIN_LEN, MAVLINK_MSG_ID_FENCE_POINT_LEN, MAVLINK_MSG_ID_FENCE_POINT_CRC);
#endif
}

/**
 * @brief Send a fence_point message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_fence_point_send_struct(mavlink_channel_t chan, const mavlink_fence_point_t* fence_point)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_fence_point_send(chan, fence_point->target_system, fence_point->target_component, fence_point->idx, fence_point->count, fence_point->lat, fence_point->lng);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FENCE_POINT, (const char *)fence_point, MAVLINK_MSG_ID_FENCE_POINT_MIN_LEN, MAVLINK_MSG_ID_FENCE_POINT_LEN, MAVLINK_MSG_ID_FENCE_POINT_CRC);
#endif
}

#if MAVLINK_MSG_ID_FENCE_POINT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_fence_point_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t idx, uint8_t count, float lat, float lng)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, lat);
    _mav_put_float(buf, 4, lng);
    _mav_put_uint8_t(buf, 8, target_system);
    _mav_put_uint8_t(buf, 9, target_component);
    _mav_put_uint8_t(buf, 10, idx);
    _mav_put_uint8_t(buf, 11, count);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FENCE_POINT, buf, MAVLINK_MSG_ID_FENCE_POINT_MIN_LEN, MAVLINK_MSG_ID_FENCE_POINT_LEN, MAVLINK_MSG_ID_FENCE_POINT_CRC);
#else
    mavlink_fence_point_t *packet = (mavlink_fence_point_t *)msgbuf;
    packet->lat = lat;
    packet->lng = lng;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->idx = idx;
    packet->count = count;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FENCE_POINT, (const char *)packet, MAVLINK_MSG_ID_FENCE_POINT_MIN_LEN, MAVLINK_MSG_ID_FENCE_POINT_LEN, MAVLINK_MSG_ID_FENCE_POINT_CRC);
#endif
}
#endif

#endif

// MESSAGE FENCE_POINT UNPACKING


/**
 * @brief Get field target_system from fence_point message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_fence_point_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field target_component from fence_point message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_fence_point_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field idx from fence_point message
 *
 * @return point index (first point is 1, 0 is for return point)
 */
static inline uint8_t mavlink_msg_fence_point_get_idx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field count from fence_point message
 *
 * @return total number of points (for sanity checking)
 */
static inline uint8_t mavlink_msg_fence_point_get_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field lat from fence_point message
 *
 * @return Latitude of point
 */
static inline float mavlink_msg_fence_point_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field lng from fence_point message
 *
 * @return Longitude of point
 */
static inline float mavlink_msg_fence_point_get_lng(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a fence_point message into a struct
 *
 * @param msg The message to decode
 * @param fence_point C-struct to decode the message contents into
 */
static inline void mavlink_msg_fence_point_decode(const mavlink_message_t* msg, mavlink_fence_point_t* fence_point)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    fence_point->lat = mavlink_msg_fence_point_get_lat(msg);
    fence_point->lng = mavlink_msg_fence_point_get_lng(msg);
    fence_point->target_system = mavlink_msg_fence_point_get_target_system(msg);
    fence_point->target_component = mavlink_msg_fence_point_get_target_component(msg);
    fence_point->idx = mavlink_msg_fence_point_get_idx(msg);
    fence_point->count = mavlink_msg_fence_point_get_count(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FENCE_POINT_LEN? msg->len : MAVLINK_MSG_ID_FENCE_POINT_LEN;
        memset(fence_point, 0, MAVLINK_MSG_ID_FENCE_POINT_LEN);
    memcpy(fence_point, _MAV_PAYLOAD(msg), len);
#endif
}
