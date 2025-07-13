#pragma once
// MESSAGE TERRAIN_REQUEST PACKING

#define MAVLINK_MSG_ID_TERRAIN_REQUEST 133


typedef struct __mavlink_terrain_request_t {
 uint64_t mask; /*<  Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits)*/
 int32_t lat; /*< [degE7] Latitude of SW corner of first grid*/
 int32_t lon; /*< [degE7] Longitude of SW corner of first grid*/
 uint16_t grid_spacing; /*< [m] Grid spacing*/
} mavlink_terrain_request_t;

#define MAVLINK_MSG_ID_TERRAIN_REQUEST_LEN 18
#define MAVLINK_MSG_ID_TERRAIN_REQUEST_MIN_LEN 18
#define MAVLINK_MSG_ID_133_LEN 18
#define MAVLINK_MSG_ID_133_MIN_LEN 18

#define MAVLINK_MSG_ID_TERRAIN_REQUEST_CRC 6
#define MAVLINK_MSG_ID_133_CRC 6



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TERRAIN_REQUEST { \
    133, \
    "TERRAIN_REQUEST", \
    4, \
    {  { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_terrain_request_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_terrain_request_t, lon) }, \
         { "grid_spacing", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_terrain_request_t, grid_spacing) }, \
         { "mask", "0x%07x", MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_terrain_request_t, mask) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TERRAIN_REQUEST { \
    "TERRAIN_REQUEST", \
    4, \
    {  { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_terrain_request_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_terrain_request_t, lon) }, \
         { "grid_spacing", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_terrain_request_t, grid_spacing) }, \
         { "mask", "0x%07x", MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_terrain_request_t, mask) }, \
         } \
}
#endif

/**
 * @brief Pack a terrain_request message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param lat [degE7] Latitude of SW corner of first grid
 * @param lon [degE7] Longitude of SW corner of first grid
 * @param grid_spacing [m] Grid spacing
 * @param mask  Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_terrain_request_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t lat, int32_t lon, uint16_t grid_spacing, uint64_t mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TERRAIN_REQUEST_LEN];
    _mav_put_uint64_t(buf, 0, mask);
    _mav_put_int32_t(buf, 8, lat);
    _mav_put_int32_t(buf, 12, lon);
    _mav_put_uint16_t(buf, 16, grid_spacing);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TERRAIN_REQUEST_LEN);
#else
    mavlink_terrain_request_t packet;
    packet.mask = mask;
    packet.lat = lat;
    packet.lon = lon;
    packet.grid_spacing = grid_spacing;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TERRAIN_REQUEST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TERRAIN_REQUEST;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TERRAIN_REQUEST_MIN_LEN, MAVLINK_MSG_ID_TERRAIN_REQUEST_LEN, MAVLINK_MSG_ID_TERRAIN_REQUEST_CRC);
}

/**
 * @brief Pack a terrain_request message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param lat [degE7] Latitude of SW corner of first grid
 * @param lon [degE7] Longitude of SW corner of first grid
 * @param grid_spacing [m] Grid spacing
 * @param mask  Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_terrain_request_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               int32_t lat, int32_t lon, uint16_t grid_spacing, uint64_t mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TERRAIN_REQUEST_LEN];
    _mav_put_uint64_t(buf, 0, mask);
    _mav_put_int32_t(buf, 8, lat);
    _mav_put_int32_t(buf, 12, lon);
    _mav_put_uint16_t(buf, 16, grid_spacing);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TERRAIN_REQUEST_LEN);
#else
    mavlink_terrain_request_t packet;
    packet.mask = mask;
    packet.lat = lat;
    packet.lon = lon;
    packet.grid_spacing = grid_spacing;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TERRAIN_REQUEST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TERRAIN_REQUEST;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TERRAIN_REQUEST_MIN_LEN, MAVLINK_MSG_ID_TERRAIN_REQUEST_LEN, MAVLINK_MSG_ID_TERRAIN_REQUEST_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TERRAIN_REQUEST_MIN_LEN, MAVLINK_MSG_ID_TERRAIN_REQUEST_LEN);
#endif
}

/**
 * @brief Pack a terrain_request message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lat [degE7] Latitude of SW corner of first grid
 * @param lon [degE7] Longitude of SW corner of first grid
 * @param grid_spacing [m] Grid spacing
 * @param mask  Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_terrain_request_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t lat,int32_t lon,uint16_t grid_spacing,uint64_t mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TERRAIN_REQUEST_LEN];
    _mav_put_uint64_t(buf, 0, mask);
    _mav_put_int32_t(buf, 8, lat);
    _mav_put_int32_t(buf, 12, lon);
    _mav_put_uint16_t(buf, 16, grid_spacing);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TERRAIN_REQUEST_LEN);
#else
    mavlink_terrain_request_t packet;
    packet.mask = mask;
    packet.lat = lat;
    packet.lon = lon;
    packet.grid_spacing = grid_spacing;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TERRAIN_REQUEST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TERRAIN_REQUEST;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TERRAIN_REQUEST_MIN_LEN, MAVLINK_MSG_ID_TERRAIN_REQUEST_LEN, MAVLINK_MSG_ID_TERRAIN_REQUEST_CRC);
}

/**
 * @brief Encode a terrain_request struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param terrain_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_terrain_request_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_terrain_request_t* terrain_request)
{
    return mavlink_msg_terrain_request_pack(system_id, component_id, msg, terrain_request->lat, terrain_request->lon, terrain_request->grid_spacing, terrain_request->mask);
}

/**
 * @brief Encode a terrain_request struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param terrain_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_terrain_request_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_terrain_request_t* terrain_request)
{
    return mavlink_msg_terrain_request_pack_chan(system_id, component_id, chan, msg, terrain_request->lat, terrain_request->lon, terrain_request->grid_spacing, terrain_request->mask);
}

/**
 * @brief Encode a terrain_request struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param terrain_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_terrain_request_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_terrain_request_t* terrain_request)
{
    return mavlink_msg_terrain_request_pack_status(system_id, component_id, _status, msg,  terrain_request->lat, terrain_request->lon, terrain_request->grid_spacing, terrain_request->mask);
}

/**
 * @brief Send a terrain_request message
 * @param chan MAVLink channel to send the message
 *
 * @param lat [degE7] Latitude of SW corner of first grid
 * @param lon [degE7] Longitude of SW corner of first grid
 * @param grid_spacing [m] Grid spacing
 * @param mask  Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_terrain_request_send(mavlink_channel_t chan, int32_t lat, int32_t lon, uint16_t grid_spacing, uint64_t mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TERRAIN_REQUEST_LEN];
    _mav_put_uint64_t(buf, 0, mask);
    _mav_put_int32_t(buf, 8, lat);
    _mav_put_int32_t(buf, 12, lon);
    _mav_put_uint16_t(buf, 16, grid_spacing);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TERRAIN_REQUEST, buf, MAVLINK_MSG_ID_TERRAIN_REQUEST_MIN_LEN, MAVLINK_MSG_ID_TERRAIN_REQUEST_LEN, MAVLINK_MSG_ID_TERRAIN_REQUEST_CRC);
#else
    mavlink_terrain_request_t packet;
    packet.mask = mask;
    packet.lat = lat;
    packet.lon = lon;
    packet.grid_spacing = grid_spacing;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TERRAIN_REQUEST, (const char *)&packet, MAVLINK_MSG_ID_TERRAIN_REQUEST_MIN_LEN, MAVLINK_MSG_ID_TERRAIN_REQUEST_LEN, MAVLINK_MSG_ID_TERRAIN_REQUEST_CRC);
#endif
}

/**
 * @brief Send a terrain_request message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_terrain_request_send_struct(mavlink_channel_t chan, const mavlink_terrain_request_t* terrain_request)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_terrain_request_send(chan, terrain_request->lat, terrain_request->lon, terrain_request->grid_spacing, terrain_request->mask);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TERRAIN_REQUEST, (const char *)terrain_request, MAVLINK_MSG_ID_TERRAIN_REQUEST_MIN_LEN, MAVLINK_MSG_ID_TERRAIN_REQUEST_LEN, MAVLINK_MSG_ID_TERRAIN_REQUEST_CRC);
#endif
}

#if MAVLINK_MSG_ID_TERRAIN_REQUEST_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_terrain_request_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t lat, int32_t lon, uint16_t grid_spacing, uint64_t mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, mask);
    _mav_put_int32_t(buf, 8, lat);
    _mav_put_int32_t(buf, 12, lon);
    _mav_put_uint16_t(buf, 16, grid_spacing);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TERRAIN_REQUEST, buf, MAVLINK_MSG_ID_TERRAIN_REQUEST_MIN_LEN, MAVLINK_MSG_ID_TERRAIN_REQUEST_LEN, MAVLINK_MSG_ID_TERRAIN_REQUEST_CRC);
#else
    mavlink_terrain_request_t *packet = (mavlink_terrain_request_t *)msgbuf;
    packet->mask = mask;
    packet->lat = lat;
    packet->lon = lon;
    packet->grid_spacing = grid_spacing;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TERRAIN_REQUEST, (const char *)packet, MAVLINK_MSG_ID_TERRAIN_REQUEST_MIN_LEN, MAVLINK_MSG_ID_TERRAIN_REQUEST_LEN, MAVLINK_MSG_ID_TERRAIN_REQUEST_CRC);
#endif
}
#endif

#endif

// MESSAGE TERRAIN_REQUEST UNPACKING


/**
 * @brief Get field lat from terrain_request message
 *
 * @return [degE7] Latitude of SW corner of first grid
 */
static inline int32_t mavlink_msg_terrain_request_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field lon from terrain_request message
 *
 * @return [degE7] Longitude of SW corner of first grid
 */
static inline int32_t mavlink_msg_terrain_request_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field grid_spacing from terrain_request message
 *
 * @return [m] Grid spacing
 */
static inline uint16_t mavlink_msg_terrain_request_get_grid_spacing(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field mask from terrain_request message
 *
 * @return  Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits)
 */
static inline uint64_t mavlink_msg_terrain_request_get_mask(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Decode a terrain_request message into a struct
 *
 * @param msg The message to decode
 * @param terrain_request C-struct to decode the message contents into
 */
static inline void mavlink_msg_terrain_request_decode(const mavlink_message_t* msg, mavlink_terrain_request_t* terrain_request)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    terrain_request->mask = mavlink_msg_terrain_request_get_mask(msg);
    terrain_request->lat = mavlink_msg_terrain_request_get_lat(msg);
    terrain_request->lon = mavlink_msg_terrain_request_get_lon(msg);
    terrain_request->grid_spacing = mavlink_msg_terrain_request_get_grid_spacing(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TERRAIN_REQUEST_LEN? msg->len : MAVLINK_MSG_ID_TERRAIN_REQUEST_LEN;
        memset(terrain_request, 0, MAVLINK_MSG_ID_TERRAIN_REQUEST_LEN);
    memcpy(terrain_request, _MAV_PAYLOAD(msg), len);
#endif
}
