#pragma once
// MESSAGE TERRAIN_CHECK PACKING

#define MAVLINK_MSG_ID_TERRAIN_CHECK 135


typedef struct __mavlink_terrain_check_t {
 int32_t lat; /*< [degE7] Latitude*/
 int32_t lon; /*< [degE7] Longitude*/
} mavlink_terrain_check_t;

#define MAVLINK_MSG_ID_TERRAIN_CHECK_LEN 8
#define MAVLINK_MSG_ID_TERRAIN_CHECK_MIN_LEN 8
#define MAVLINK_MSG_ID_135_LEN 8
#define MAVLINK_MSG_ID_135_MIN_LEN 8

#define MAVLINK_MSG_ID_TERRAIN_CHECK_CRC 203
#define MAVLINK_MSG_ID_135_CRC 203



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TERRAIN_CHECK { \
    135, \
    "TERRAIN_CHECK", \
    2, \
    {  { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_terrain_check_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_terrain_check_t, lon) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TERRAIN_CHECK { \
    "TERRAIN_CHECK", \
    2, \
    {  { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_terrain_check_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_terrain_check_t, lon) }, \
         } \
}
#endif

/**
 * @brief Pack a terrain_check message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param lat [degE7] Latitude
 * @param lon [degE7] Longitude
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_terrain_check_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t lat, int32_t lon)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TERRAIN_CHECK_LEN];
    _mav_put_int32_t(buf, 0, lat);
    _mav_put_int32_t(buf, 4, lon);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TERRAIN_CHECK_LEN);
#else
    mavlink_terrain_check_t packet;
    packet.lat = lat;
    packet.lon = lon;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TERRAIN_CHECK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TERRAIN_CHECK;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TERRAIN_CHECK_MIN_LEN, MAVLINK_MSG_ID_TERRAIN_CHECK_LEN, MAVLINK_MSG_ID_TERRAIN_CHECK_CRC);
}

/**
 * @brief Pack a terrain_check message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param lat [degE7] Latitude
 * @param lon [degE7] Longitude
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_terrain_check_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               int32_t lat, int32_t lon)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TERRAIN_CHECK_LEN];
    _mav_put_int32_t(buf, 0, lat);
    _mav_put_int32_t(buf, 4, lon);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TERRAIN_CHECK_LEN);
#else
    mavlink_terrain_check_t packet;
    packet.lat = lat;
    packet.lon = lon;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TERRAIN_CHECK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TERRAIN_CHECK;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TERRAIN_CHECK_MIN_LEN, MAVLINK_MSG_ID_TERRAIN_CHECK_LEN, MAVLINK_MSG_ID_TERRAIN_CHECK_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TERRAIN_CHECK_MIN_LEN, MAVLINK_MSG_ID_TERRAIN_CHECK_LEN);
#endif
}

/**
 * @brief Pack a terrain_check message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lat [degE7] Latitude
 * @param lon [degE7] Longitude
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_terrain_check_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t lat,int32_t lon)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TERRAIN_CHECK_LEN];
    _mav_put_int32_t(buf, 0, lat);
    _mav_put_int32_t(buf, 4, lon);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TERRAIN_CHECK_LEN);
#else
    mavlink_terrain_check_t packet;
    packet.lat = lat;
    packet.lon = lon;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TERRAIN_CHECK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TERRAIN_CHECK;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TERRAIN_CHECK_MIN_LEN, MAVLINK_MSG_ID_TERRAIN_CHECK_LEN, MAVLINK_MSG_ID_TERRAIN_CHECK_CRC);
}

/**
 * @brief Encode a terrain_check struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param terrain_check C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_terrain_check_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_terrain_check_t* terrain_check)
{
    return mavlink_msg_terrain_check_pack(system_id, component_id, msg, terrain_check->lat, terrain_check->lon);
}

/**
 * @brief Encode a terrain_check struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param terrain_check C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_terrain_check_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_terrain_check_t* terrain_check)
{
    return mavlink_msg_terrain_check_pack_chan(system_id, component_id, chan, msg, terrain_check->lat, terrain_check->lon);
}

/**
 * @brief Encode a terrain_check struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param terrain_check C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_terrain_check_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_terrain_check_t* terrain_check)
{
    return mavlink_msg_terrain_check_pack_status(system_id, component_id, _status, msg,  terrain_check->lat, terrain_check->lon);
}

/**
 * @brief Send a terrain_check message
 * @param chan MAVLink channel to send the message
 *
 * @param lat [degE7] Latitude
 * @param lon [degE7] Longitude
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_terrain_check_send(mavlink_channel_t chan, int32_t lat, int32_t lon)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TERRAIN_CHECK_LEN];
    _mav_put_int32_t(buf, 0, lat);
    _mav_put_int32_t(buf, 4, lon);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TERRAIN_CHECK, buf, MAVLINK_MSG_ID_TERRAIN_CHECK_MIN_LEN, MAVLINK_MSG_ID_TERRAIN_CHECK_LEN, MAVLINK_MSG_ID_TERRAIN_CHECK_CRC);
#else
    mavlink_terrain_check_t packet;
    packet.lat = lat;
    packet.lon = lon;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TERRAIN_CHECK, (const char *)&packet, MAVLINK_MSG_ID_TERRAIN_CHECK_MIN_LEN, MAVLINK_MSG_ID_TERRAIN_CHECK_LEN, MAVLINK_MSG_ID_TERRAIN_CHECK_CRC);
#endif
}

/**
 * @brief Send a terrain_check message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_terrain_check_send_struct(mavlink_channel_t chan, const mavlink_terrain_check_t* terrain_check)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_terrain_check_send(chan, terrain_check->lat, terrain_check->lon);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TERRAIN_CHECK, (const char *)terrain_check, MAVLINK_MSG_ID_TERRAIN_CHECK_MIN_LEN, MAVLINK_MSG_ID_TERRAIN_CHECK_LEN, MAVLINK_MSG_ID_TERRAIN_CHECK_CRC);
#endif
}

#if MAVLINK_MSG_ID_TERRAIN_CHECK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_terrain_check_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t lat, int32_t lon)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, lat);
    _mav_put_int32_t(buf, 4, lon);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TERRAIN_CHECK, buf, MAVLINK_MSG_ID_TERRAIN_CHECK_MIN_LEN, MAVLINK_MSG_ID_TERRAIN_CHECK_LEN, MAVLINK_MSG_ID_TERRAIN_CHECK_CRC);
#else
    mavlink_terrain_check_t *packet = (mavlink_terrain_check_t *)msgbuf;
    packet->lat = lat;
    packet->lon = lon;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TERRAIN_CHECK, (const char *)packet, MAVLINK_MSG_ID_TERRAIN_CHECK_MIN_LEN, MAVLINK_MSG_ID_TERRAIN_CHECK_LEN, MAVLINK_MSG_ID_TERRAIN_CHECK_CRC);
#endif
}
#endif

#endif

// MESSAGE TERRAIN_CHECK UNPACKING


/**
 * @brief Get field lat from terrain_check message
 *
 * @return [degE7] Latitude
 */
static inline int32_t mavlink_msg_terrain_check_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field lon from terrain_check message
 *
 * @return [degE7] Longitude
 */
static inline int32_t mavlink_msg_terrain_check_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Decode a terrain_check message into a struct
 *
 * @param msg The message to decode
 * @param terrain_check C-struct to decode the message contents into
 */
static inline void mavlink_msg_terrain_check_decode(const mavlink_message_t* msg, mavlink_terrain_check_t* terrain_check)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    terrain_check->lat = mavlink_msg_terrain_check_get_lat(msg);
    terrain_check->lon = mavlink_msg_terrain_check_get_lon(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TERRAIN_CHECK_LEN? msg->len : MAVLINK_MSG_ID_TERRAIN_CHECK_LEN;
        memset(terrain_check, 0, MAVLINK_MSG_ID_TERRAIN_CHECK_LEN);
    memcpy(terrain_check, _MAV_PAYLOAD(msg), len);
#endif
}
