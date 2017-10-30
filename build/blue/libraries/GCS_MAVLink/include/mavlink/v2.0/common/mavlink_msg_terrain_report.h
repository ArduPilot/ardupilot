#pragma once
// MESSAGE TERRAIN_REPORT PACKING

#define MAVLINK_MSG_ID_TERRAIN_REPORT 136

MAVPACKED(
typedef struct __mavlink_terrain_report_t {
 int32_t lat; /*< Latitude (degrees *10^7)*/
 int32_t lon; /*< Longitude (degrees *10^7)*/
 float terrain_height; /*< Terrain height in meters AMSL*/
 float current_height; /*< Current vehicle height above lat/lon terrain height (meters)*/
 uint16_t spacing; /*< grid spacing (zero if terrain at this location unavailable)*/
 uint16_t pending; /*< Number of 4x4 terrain blocks waiting to be received or read from disk*/
 uint16_t loaded; /*< Number of 4x4 terrain blocks in memory*/
}) mavlink_terrain_report_t;

#define MAVLINK_MSG_ID_TERRAIN_REPORT_LEN 22
#define MAVLINK_MSG_ID_TERRAIN_REPORT_MIN_LEN 22
#define MAVLINK_MSG_ID_136_LEN 22
#define MAVLINK_MSG_ID_136_MIN_LEN 22

#define MAVLINK_MSG_ID_TERRAIN_REPORT_CRC 1
#define MAVLINK_MSG_ID_136_CRC 1



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TERRAIN_REPORT { \
    136, \
    "TERRAIN_REPORT", \
    7, \
    {  { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_terrain_report_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_terrain_report_t, lon) }, \
         { "spacing", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_terrain_report_t, spacing) }, \
         { "terrain_height", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_terrain_report_t, terrain_height) }, \
         { "current_height", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_terrain_report_t, current_height) }, \
         { "pending", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_terrain_report_t, pending) }, \
         { "loaded", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_terrain_report_t, loaded) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TERRAIN_REPORT { \
    "TERRAIN_REPORT", \
    7, \
    {  { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_terrain_report_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_terrain_report_t, lon) }, \
         { "spacing", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_terrain_report_t, spacing) }, \
         { "terrain_height", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_terrain_report_t, terrain_height) }, \
         { "current_height", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_terrain_report_t, current_height) }, \
         { "pending", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_terrain_report_t, pending) }, \
         { "loaded", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_terrain_report_t, loaded) }, \
         } \
}
#endif

/**
 * @brief Pack a terrain_report message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param lat Latitude (degrees *10^7)
 * @param lon Longitude (degrees *10^7)
 * @param spacing grid spacing (zero if terrain at this location unavailable)
 * @param terrain_height Terrain height in meters AMSL
 * @param current_height Current vehicle height above lat/lon terrain height (meters)
 * @param pending Number of 4x4 terrain blocks waiting to be received or read from disk
 * @param loaded Number of 4x4 terrain blocks in memory
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_terrain_report_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t lat, int32_t lon, uint16_t spacing, float terrain_height, float current_height, uint16_t pending, uint16_t loaded)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TERRAIN_REPORT_LEN];
    _mav_put_int32_t(buf, 0, lat);
    _mav_put_int32_t(buf, 4, lon);
    _mav_put_float(buf, 8, terrain_height);
    _mav_put_float(buf, 12, current_height);
    _mav_put_uint16_t(buf, 16, spacing);
    _mav_put_uint16_t(buf, 18, pending);
    _mav_put_uint16_t(buf, 20, loaded);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TERRAIN_REPORT_LEN);
#else
    mavlink_terrain_report_t packet;
    packet.lat = lat;
    packet.lon = lon;
    packet.terrain_height = terrain_height;
    packet.current_height = current_height;
    packet.spacing = spacing;
    packet.pending = pending;
    packet.loaded = loaded;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TERRAIN_REPORT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TERRAIN_REPORT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TERRAIN_REPORT_MIN_LEN, MAVLINK_MSG_ID_TERRAIN_REPORT_LEN, MAVLINK_MSG_ID_TERRAIN_REPORT_CRC);
}

/**
 * @brief Pack a terrain_report message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lat Latitude (degrees *10^7)
 * @param lon Longitude (degrees *10^7)
 * @param spacing grid spacing (zero if terrain at this location unavailable)
 * @param terrain_height Terrain height in meters AMSL
 * @param current_height Current vehicle height above lat/lon terrain height (meters)
 * @param pending Number of 4x4 terrain blocks waiting to be received or read from disk
 * @param loaded Number of 4x4 terrain blocks in memory
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_terrain_report_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t lat,int32_t lon,uint16_t spacing,float terrain_height,float current_height,uint16_t pending,uint16_t loaded)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TERRAIN_REPORT_LEN];
    _mav_put_int32_t(buf, 0, lat);
    _mav_put_int32_t(buf, 4, lon);
    _mav_put_float(buf, 8, terrain_height);
    _mav_put_float(buf, 12, current_height);
    _mav_put_uint16_t(buf, 16, spacing);
    _mav_put_uint16_t(buf, 18, pending);
    _mav_put_uint16_t(buf, 20, loaded);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TERRAIN_REPORT_LEN);
#else
    mavlink_terrain_report_t packet;
    packet.lat = lat;
    packet.lon = lon;
    packet.terrain_height = terrain_height;
    packet.current_height = current_height;
    packet.spacing = spacing;
    packet.pending = pending;
    packet.loaded = loaded;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TERRAIN_REPORT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TERRAIN_REPORT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TERRAIN_REPORT_MIN_LEN, MAVLINK_MSG_ID_TERRAIN_REPORT_LEN, MAVLINK_MSG_ID_TERRAIN_REPORT_CRC);
}

/**
 * @brief Encode a terrain_report struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param terrain_report C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_terrain_report_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_terrain_report_t* terrain_report)
{
    return mavlink_msg_terrain_report_pack(system_id, component_id, msg, terrain_report->lat, terrain_report->lon, terrain_report->spacing, terrain_report->terrain_height, terrain_report->current_height, terrain_report->pending, terrain_report->loaded);
}

/**
 * @brief Encode a terrain_report struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param terrain_report C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_terrain_report_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_terrain_report_t* terrain_report)
{
    return mavlink_msg_terrain_report_pack_chan(system_id, component_id, chan, msg, terrain_report->lat, terrain_report->lon, terrain_report->spacing, terrain_report->terrain_height, terrain_report->current_height, terrain_report->pending, terrain_report->loaded);
}

/**
 * @brief Send a terrain_report message
 * @param chan MAVLink channel to send the message
 *
 * @param lat Latitude (degrees *10^7)
 * @param lon Longitude (degrees *10^7)
 * @param spacing grid spacing (zero if terrain at this location unavailable)
 * @param terrain_height Terrain height in meters AMSL
 * @param current_height Current vehicle height above lat/lon terrain height (meters)
 * @param pending Number of 4x4 terrain blocks waiting to be received or read from disk
 * @param loaded Number of 4x4 terrain blocks in memory
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_terrain_report_send(mavlink_channel_t chan, int32_t lat, int32_t lon, uint16_t spacing, float terrain_height, float current_height, uint16_t pending, uint16_t loaded)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TERRAIN_REPORT_LEN];
    _mav_put_int32_t(buf, 0, lat);
    _mav_put_int32_t(buf, 4, lon);
    _mav_put_float(buf, 8, terrain_height);
    _mav_put_float(buf, 12, current_height);
    _mav_put_uint16_t(buf, 16, spacing);
    _mav_put_uint16_t(buf, 18, pending);
    _mav_put_uint16_t(buf, 20, loaded);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TERRAIN_REPORT, buf, MAVLINK_MSG_ID_TERRAIN_REPORT_MIN_LEN, MAVLINK_MSG_ID_TERRAIN_REPORT_LEN, MAVLINK_MSG_ID_TERRAIN_REPORT_CRC);
#else
    mavlink_terrain_report_t packet;
    packet.lat = lat;
    packet.lon = lon;
    packet.terrain_height = terrain_height;
    packet.current_height = current_height;
    packet.spacing = spacing;
    packet.pending = pending;
    packet.loaded = loaded;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TERRAIN_REPORT, (const char *)&packet, MAVLINK_MSG_ID_TERRAIN_REPORT_MIN_LEN, MAVLINK_MSG_ID_TERRAIN_REPORT_LEN, MAVLINK_MSG_ID_TERRAIN_REPORT_CRC);
#endif
}

/**
 * @brief Send a terrain_report message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_terrain_report_send_struct(mavlink_channel_t chan, const mavlink_terrain_report_t* terrain_report)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_terrain_report_send(chan, terrain_report->lat, terrain_report->lon, terrain_report->spacing, terrain_report->terrain_height, terrain_report->current_height, terrain_report->pending, terrain_report->loaded);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TERRAIN_REPORT, (const char *)terrain_report, MAVLINK_MSG_ID_TERRAIN_REPORT_MIN_LEN, MAVLINK_MSG_ID_TERRAIN_REPORT_LEN, MAVLINK_MSG_ID_TERRAIN_REPORT_CRC);
#endif
}

#if MAVLINK_MSG_ID_TERRAIN_REPORT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_terrain_report_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t lat, int32_t lon, uint16_t spacing, float terrain_height, float current_height, uint16_t pending, uint16_t loaded)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, lat);
    _mav_put_int32_t(buf, 4, lon);
    _mav_put_float(buf, 8, terrain_height);
    _mav_put_float(buf, 12, current_height);
    _mav_put_uint16_t(buf, 16, spacing);
    _mav_put_uint16_t(buf, 18, pending);
    _mav_put_uint16_t(buf, 20, loaded);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TERRAIN_REPORT, buf, MAVLINK_MSG_ID_TERRAIN_REPORT_MIN_LEN, MAVLINK_MSG_ID_TERRAIN_REPORT_LEN, MAVLINK_MSG_ID_TERRAIN_REPORT_CRC);
#else
    mavlink_terrain_report_t *packet = (mavlink_terrain_report_t *)msgbuf;
    packet->lat = lat;
    packet->lon = lon;
    packet->terrain_height = terrain_height;
    packet->current_height = current_height;
    packet->spacing = spacing;
    packet->pending = pending;
    packet->loaded = loaded;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TERRAIN_REPORT, (const char *)packet, MAVLINK_MSG_ID_TERRAIN_REPORT_MIN_LEN, MAVLINK_MSG_ID_TERRAIN_REPORT_LEN, MAVLINK_MSG_ID_TERRAIN_REPORT_CRC);
#endif
}
#endif

#endif

// MESSAGE TERRAIN_REPORT UNPACKING


/**
 * @brief Get field lat from terrain_report message
 *
 * @return Latitude (degrees *10^7)
 */
static inline int32_t mavlink_msg_terrain_report_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field lon from terrain_report message
 *
 * @return Longitude (degrees *10^7)
 */
static inline int32_t mavlink_msg_terrain_report_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field spacing from terrain_report message
 *
 * @return grid spacing (zero if terrain at this location unavailable)
 */
static inline uint16_t mavlink_msg_terrain_report_get_spacing(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field terrain_height from terrain_report message
 *
 * @return Terrain height in meters AMSL
 */
static inline float mavlink_msg_terrain_report_get_terrain_height(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field current_height from terrain_report message
 *
 * @return Current vehicle height above lat/lon terrain height (meters)
 */
static inline float mavlink_msg_terrain_report_get_current_height(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field pending from terrain_report message
 *
 * @return Number of 4x4 terrain blocks waiting to be received or read from disk
 */
static inline uint16_t mavlink_msg_terrain_report_get_pending(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  18);
}

/**
 * @brief Get field loaded from terrain_report message
 *
 * @return Number of 4x4 terrain blocks in memory
 */
static inline uint16_t mavlink_msg_terrain_report_get_loaded(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Decode a terrain_report message into a struct
 *
 * @param msg The message to decode
 * @param terrain_report C-struct to decode the message contents into
 */
static inline void mavlink_msg_terrain_report_decode(const mavlink_message_t* msg, mavlink_terrain_report_t* terrain_report)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    terrain_report->lat = mavlink_msg_terrain_report_get_lat(msg);
    terrain_report->lon = mavlink_msg_terrain_report_get_lon(msg);
    terrain_report->terrain_height = mavlink_msg_terrain_report_get_terrain_height(msg);
    terrain_report->current_height = mavlink_msg_terrain_report_get_current_height(msg);
    terrain_report->spacing = mavlink_msg_terrain_report_get_spacing(msg);
    terrain_report->pending = mavlink_msg_terrain_report_get_pending(msg);
    terrain_report->loaded = mavlink_msg_terrain_report_get_loaded(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TERRAIN_REPORT_LEN? msg->len : MAVLINK_MSG_ID_TERRAIN_REPORT_LEN;
        memset(terrain_report, 0, MAVLINK_MSG_ID_TERRAIN_REPORT_LEN);
    memcpy(terrain_report, _MAV_PAYLOAD(msg), len);
#endif
}
