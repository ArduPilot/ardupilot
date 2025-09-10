#pragma once
// MESSAGE RALLY_POINT PACKING

#define MAVLINK_MSG_ID_RALLY_POINT 175


typedef struct __mavlink_rally_point_t {
 int32_t lat; /*< [degE7] Latitude of point.*/
 int32_t lng; /*< [degE7] Longitude of point.*/
 int16_t alt; /*< [m] Transit / loiter altitude relative to home.*/
 int16_t break_alt; /*< [m] Break altitude relative to home.*/
 uint16_t land_dir; /*< [cdeg] Heading to aim for when landing.*/
 uint8_t target_system; /*<  System ID.*/
 uint8_t target_component; /*<  Component ID.*/
 uint8_t idx; /*<  Point index (first point is 0).*/
 uint8_t count; /*<  Total number of points (for sanity checking).*/
 uint8_t flags; /*<  Configuration flags.*/
} mavlink_rally_point_t;

#define MAVLINK_MSG_ID_RALLY_POINT_LEN 19
#define MAVLINK_MSG_ID_RALLY_POINT_MIN_LEN 19
#define MAVLINK_MSG_ID_175_LEN 19
#define MAVLINK_MSG_ID_175_MIN_LEN 19

#define MAVLINK_MSG_ID_RALLY_POINT_CRC 138
#define MAVLINK_MSG_ID_175_CRC 138



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RALLY_POINT { \
    175, \
    "RALLY_POINT", \
    10, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_rally_point_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_rally_point_t, target_component) }, \
         { "idx", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_rally_point_t, idx) }, \
         { "count", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_rally_point_t, count) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_rally_point_t, lat) }, \
         { "lng", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_rally_point_t, lng) }, \
         { "alt", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_rally_point_t, alt) }, \
         { "break_alt", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_rally_point_t, break_alt) }, \
         { "land_dir", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_rally_point_t, land_dir) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_rally_point_t, flags) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RALLY_POINT { \
    "RALLY_POINT", \
    10, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_rally_point_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_rally_point_t, target_component) }, \
         { "idx", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_rally_point_t, idx) }, \
         { "count", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_rally_point_t, count) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_rally_point_t, lat) }, \
         { "lng", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_rally_point_t, lng) }, \
         { "alt", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_rally_point_t, alt) }, \
         { "break_alt", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_rally_point_t, break_alt) }, \
         { "land_dir", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_rally_point_t, land_dir) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_rally_point_t, flags) }, \
         } \
}
#endif

/**
 * @brief Pack a rally_point message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param idx  Point index (first point is 0).
 * @param count  Total number of points (for sanity checking).
 * @param lat [degE7] Latitude of point.
 * @param lng [degE7] Longitude of point.
 * @param alt [m] Transit / loiter altitude relative to home.
 * @param break_alt [m] Break altitude relative to home.
 * @param land_dir [cdeg] Heading to aim for when landing.
 * @param flags  Configuration flags.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rally_point_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t idx, uint8_t count, int32_t lat, int32_t lng, int16_t alt, int16_t break_alt, uint16_t land_dir, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RALLY_POINT_LEN];
    _mav_put_int32_t(buf, 0, lat);
    _mav_put_int32_t(buf, 4, lng);
    _mav_put_int16_t(buf, 8, alt);
    _mav_put_int16_t(buf, 10, break_alt);
    _mav_put_uint16_t(buf, 12, land_dir);
    _mav_put_uint8_t(buf, 14, target_system);
    _mav_put_uint8_t(buf, 15, target_component);
    _mav_put_uint8_t(buf, 16, idx);
    _mav_put_uint8_t(buf, 17, count);
    _mav_put_uint8_t(buf, 18, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RALLY_POINT_LEN);
#else
    mavlink_rally_point_t packet;
    packet.lat = lat;
    packet.lng = lng;
    packet.alt = alt;
    packet.break_alt = break_alt;
    packet.land_dir = land_dir;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.idx = idx;
    packet.count = count;
    packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RALLY_POINT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RALLY_POINT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RALLY_POINT_MIN_LEN, MAVLINK_MSG_ID_RALLY_POINT_LEN, MAVLINK_MSG_ID_RALLY_POINT_CRC);
}

/**
 * @brief Pack a rally_point message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param idx  Point index (first point is 0).
 * @param count  Total number of points (for sanity checking).
 * @param lat [degE7] Latitude of point.
 * @param lng [degE7] Longitude of point.
 * @param alt [m] Transit / loiter altitude relative to home.
 * @param break_alt [m] Break altitude relative to home.
 * @param land_dir [cdeg] Heading to aim for when landing.
 * @param flags  Configuration flags.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rally_point_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t idx, uint8_t count, int32_t lat, int32_t lng, int16_t alt, int16_t break_alt, uint16_t land_dir, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RALLY_POINT_LEN];
    _mav_put_int32_t(buf, 0, lat);
    _mav_put_int32_t(buf, 4, lng);
    _mav_put_int16_t(buf, 8, alt);
    _mav_put_int16_t(buf, 10, break_alt);
    _mav_put_uint16_t(buf, 12, land_dir);
    _mav_put_uint8_t(buf, 14, target_system);
    _mav_put_uint8_t(buf, 15, target_component);
    _mav_put_uint8_t(buf, 16, idx);
    _mav_put_uint8_t(buf, 17, count);
    _mav_put_uint8_t(buf, 18, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RALLY_POINT_LEN);
#else
    mavlink_rally_point_t packet;
    packet.lat = lat;
    packet.lng = lng;
    packet.alt = alt;
    packet.break_alt = break_alt;
    packet.land_dir = land_dir;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.idx = idx;
    packet.count = count;
    packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RALLY_POINT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RALLY_POINT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_RALLY_POINT_MIN_LEN, MAVLINK_MSG_ID_RALLY_POINT_LEN, MAVLINK_MSG_ID_RALLY_POINT_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_RALLY_POINT_MIN_LEN, MAVLINK_MSG_ID_RALLY_POINT_LEN);
#endif
}

/**
 * @brief Pack a rally_point message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param idx  Point index (first point is 0).
 * @param count  Total number of points (for sanity checking).
 * @param lat [degE7] Latitude of point.
 * @param lng [degE7] Longitude of point.
 * @param alt [m] Transit / loiter altitude relative to home.
 * @param break_alt [m] Break altitude relative to home.
 * @param land_dir [cdeg] Heading to aim for when landing.
 * @param flags  Configuration flags.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rally_point_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint8_t idx,uint8_t count,int32_t lat,int32_t lng,int16_t alt,int16_t break_alt,uint16_t land_dir,uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RALLY_POINT_LEN];
    _mav_put_int32_t(buf, 0, lat);
    _mav_put_int32_t(buf, 4, lng);
    _mav_put_int16_t(buf, 8, alt);
    _mav_put_int16_t(buf, 10, break_alt);
    _mav_put_uint16_t(buf, 12, land_dir);
    _mav_put_uint8_t(buf, 14, target_system);
    _mav_put_uint8_t(buf, 15, target_component);
    _mav_put_uint8_t(buf, 16, idx);
    _mav_put_uint8_t(buf, 17, count);
    _mav_put_uint8_t(buf, 18, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RALLY_POINT_LEN);
#else
    mavlink_rally_point_t packet;
    packet.lat = lat;
    packet.lng = lng;
    packet.alt = alt;
    packet.break_alt = break_alt;
    packet.land_dir = land_dir;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.idx = idx;
    packet.count = count;
    packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RALLY_POINT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RALLY_POINT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RALLY_POINT_MIN_LEN, MAVLINK_MSG_ID_RALLY_POINT_LEN, MAVLINK_MSG_ID_RALLY_POINT_CRC);
}

/**
 * @brief Encode a rally_point struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rally_point C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rally_point_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rally_point_t* rally_point)
{
    return mavlink_msg_rally_point_pack(system_id, component_id, msg, rally_point->target_system, rally_point->target_component, rally_point->idx, rally_point->count, rally_point->lat, rally_point->lng, rally_point->alt, rally_point->break_alt, rally_point->land_dir, rally_point->flags);
}

/**
 * @brief Encode a rally_point struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rally_point C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rally_point_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rally_point_t* rally_point)
{
    return mavlink_msg_rally_point_pack_chan(system_id, component_id, chan, msg, rally_point->target_system, rally_point->target_component, rally_point->idx, rally_point->count, rally_point->lat, rally_point->lng, rally_point->alt, rally_point->break_alt, rally_point->land_dir, rally_point->flags);
}

/**
 * @brief Encode a rally_point struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param rally_point C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rally_point_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_rally_point_t* rally_point)
{
    return mavlink_msg_rally_point_pack_status(system_id, component_id, _status, msg,  rally_point->target_system, rally_point->target_component, rally_point->idx, rally_point->count, rally_point->lat, rally_point->lng, rally_point->alt, rally_point->break_alt, rally_point->land_dir, rally_point->flags);
}

/**
 * @brief Send a rally_point message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param idx  Point index (first point is 0).
 * @param count  Total number of points (for sanity checking).
 * @param lat [degE7] Latitude of point.
 * @param lng [degE7] Longitude of point.
 * @param alt [m] Transit / loiter altitude relative to home.
 * @param break_alt [m] Break altitude relative to home.
 * @param land_dir [cdeg] Heading to aim for when landing.
 * @param flags  Configuration flags.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rally_point_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t idx, uint8_t count, int32_t lat, int32_t lng, int16_t alt, int16_t break_alt, uint16_t land_dir, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RALLY_POINT_LEN];
    _mav_put_int32_t(buf, 0, lat);
    _mav_put_int32_t(buf, 4, lng);
    _mav_put_int16_t(buf, 8, alt);
    _mav_put_int16_t(buf, 10, break_alt);
    _mav_put_uint16_t(buf, 12, land_dir);
    _mav_put_uint8_t(buf, 14, target_system);
    _mav_put_uint8_t(buf, 15, target_component);
    _mav_put_uint8_t(buf, 16, idx);
    _mav_put_uint8_t(buf, 17, count);
    _mav_put_uint8_t(buf, 18, flags);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RALLY_POINT, buf, MAVLINK_MSG_ID_RALLY_POINT_MIN_LEN, MAVLINK_MSG_ID_RALLY_POINT_LEN, MAVLINK_MSG_ID_RALLY_POINT_CRC);
#else
    mavlink_rally_point_t packet;
    packet.lat = lat;
    packet.lng = lng;
    packet.alt = alt;
    packet.break_alt = break_alt;
    packet.land_dir = land_dir;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.idx = idx;
    packet.count = count;
    packet.flags = flags;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RALLY_POINT, (const char *)&packet, MAVLINK_MSG_ID_RALLY_POINT_MIN_LEN, MAVLINK_MSG_ID_RALLY_POINT_LEN, MAVLINK_MSG_ID_RALLY_POINT_CRC);
#endif
}

/**
 * @brief Send a rally_point message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_rally_point_send_struct(mavlink_channel_t chan, const mavlink_rally_point_t* rally_point)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_rally_point_send(chan, rally_point->target_system, rally_point->target_component, rally_point->idx, rally_point->count, rally_point->lat, rally_point->lng, rally_point->alt, rally_point->break_alt, rally_point->land_dir, rally_point->flags);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RALLY_POINT, (const char *)rally_point, MAVLINK_MSG_ID_RALLY_POINT_MIN_LEN, MAVLINK_MSG_ID_RALLY_POINT_LEN, MAVLINK_MSG_ID_RALLY_POINT_CRC);
#endif
}

#if MAVLINK_MSG_ID_RALLY_POINT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rally_point_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t idx, uint8_t count, int32_t lat, int32_t lng, int16_t alt, int16_t break_alt, uint16_t land_dir, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, lat);
    _mav_put_int32_t(buf, 4, lng);
    _mav_put_int16_t(buf, 8, alt);
    _mav_put_int16_t(buf, 10, break_alt);
    _mav_put_uint16_t(buf, 12, land_dir);
    _mav_put_uint8_t(buf, 14, target_system);
    _mav_put_uint8_t(buf, 15, target_component);
    _mav_put_uint8_t(buf, 16, idx);
    _mav_put_uint8_t(buf, 17, count);
    _mav_put_uint8_t(buf, 18, flags);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RALLY_POINT, buf, MAVLINK_MSG_ID_RALLY_POINT_MIN_LEN, MAVLINK_MSG_ID_RALLY_POINT_LEN, MAVLINK_MSG_ID_RALLY_POINT_CRC);
#else
    mavlink_rally_point_t *packet = (mavlink_rally_point_t *)msgbuf;
    packet->lat = lat;
    packet->lng = lng;
    packet->alt = alt;
    packet->break_alt = break_alt;
    packet->land_dir = land_dir;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->idx = idx;
    packet->count = count;
    packet->flags = flags;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RALLY_POINT, (const char *)packet, MAVLINK_MSG_ID_RALLY_POINT_MIN_LEN, MAVLINK_MSG_ID_RALLY_POINT_LEN, MAVLINK_MSG_ID_RALLY_POINT_CRC);
#endif
}
#endif

#endif

// MESSAGE RALLY_POINT UNPACKING


/**
 * @brief Get field target_system from rally_point message
 *
 * @return  System ID.
 */
static inline uint8_t mavlink_msg_rally_point_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field target_component from rally_point message
 *
 * @return  Component ID.
 */
static inline uint8_t mavlink_msg_rally_point_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  15);
}

/**
 * @brief Get field idx from rally_point message
 *
 * @return  Point index (first point is 0).
 */
static inline uint8_t mavlink_msg_rally_point_get_idx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field count from rally_point message
 *
 * @return  Total number of points (for sanity checking).
 */
static inline uint8_t mavlink_msg_rally_point_get_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field lat from rally_point message
 *
 * @return [degE7] Latitude of point.
 */
static inline int32_t mavlink_msg_rally_point_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field lng from rally_point message
 *
 * @return [degE7] Longitude of point.
 */
static inline int32_t mavlink_msg_rally_point_get_lng(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field alt from rally_point message
 *
 * @return [m] Transit / loiter altitude relative to home.
 */
static inline int16_t mavlink_msg_rally_point_get_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field break_alt from rally_point message
 *
 * @return [m] Break altitude relative to home.
 */
static inline int16_t mavlink_msg_rally_point_get_break_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Get field land_dir from rally_point message
 *
 * @return [cdeg] Heading to aim for when landing.
 */
static inline uint16_t mavlink_msg_rally_point_get_land_dir(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field flags from rally_point message
 *
 * @return  Configuration flags.
 */
static inline uint8_t mavlink_msg_rally_point_get_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Decode a rally_point message into a struct
 *
 * @param msg The message to decode
 * @param rally_point C-struct to decode the message contents into
 */
static inline void mavlink_msg_rally_point_decode(const mavlink_message_t* msg, mavlink_rally_point_t* rally_point)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    rally_point->lat = mavlink_msg_rally_point_get_lat(msg);
    rally_point->lng = mavlink_msg_rally_point_get_lng(msg);
    rally_point->alt = mavlink_msg_rally_point_get_alt(msg);
    rally_point->break_alt = mavlink_msg_rally_point_get_break_alt(msg);
    rally_point->land_dir = mavlink_msg_rally_point_get_land_dir(msg);
    rally_point->target_system = mavlink_msg_rally_point_get_target_system(msg);
    rally_point->target_component = mavlink_msg_rally_point_get_target_component(msg);
    rally_point->idx = mavlink_msg_rally_point_get_idx(msg);
    rally_point->count = mavlink_msg_rally_point_get_count(msg);
    rally_point->flags = mavlink_msg_rally_point_get_flags(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RALLY_POINT_LEN? msg->len : MAVLINK_MSG_ID_RALLY_POINT_LEN;
        memset(rally_point, 0, MAVLINK_MSG_ID_RALLY_POINT_LEN);
    memcpy(rally_point, _MAV_PAYLOAD(msg), len);
#endif
}
