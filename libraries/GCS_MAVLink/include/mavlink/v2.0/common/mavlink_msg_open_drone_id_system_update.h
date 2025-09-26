#pragma once
// MESSAGE OPEN_DRONE_ID_SYSTEM_UPDATE PACKING

#define MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE 12919


typedef struct __mavlink_open_drone_id_system_update_t {
 int32_t operator_latitude; /*< [degE7] Latitude of the operator. If unknown: 0 (both Lat/Lon).*/
 int32_t operator_longitude; /*< [degE7] Longitude of the operator. If unknown: 0 (both Lat/Lon).*/
 float operator_altitude_geo; /*< [m] Geodetic altitude of the operator relative to WGS84. If unknown: -1000 m.*/
 uint32_t timestamp; /*< [s] 32 bit Unix Timestamp in seconds since 00:00:00 01/01/2019.*/
 uint8_t target_system; /*<  System ID (0 for broadcast).*/
 uint8_t target_component; /*<  Component ID (0 for broadcast).*/
} mavlink_open_drone_id_system_update_t;

#define MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_LEN 18
#define MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_MIN_LEN 18
#define MAVLINK_MSG_ID_12919_LEN 18
#define MAVLINK_MSG_ID_12919_MIN_LEN 18

#define MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_CRC 7
#define MAVLINK_MSG_ID_12919_CRC 7



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_OPEN_DRONE_ID_SYSTEM_UPDATE { \
    12919, \
    "OPEN_DRONE_ID_SYSTEM_UPDATE", \
    6, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_open_drone_id_system_update_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_open_drone_id_system_update_t, target_component) }, \
         { "operator_latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_open_drone_id_system_update_t, operator_latitude) }, \
         { "operator_longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_open_drone_id_system_update_t, operator_longitude) }, \
         { "operator_altitude_geo", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_open_drone_id_system_update_t, operator_altitude_geo) }, \
         { "timestamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_open_drone_id_system_update_t, timestamp) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_OPEN_DRONE_ID_SYSTEM_UPDATE { \
    "OPEN_DRONE_ID_SYSTEM_UPDATE", \
    6, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_open_drone_id_system_update_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_open_drone_id_system_update_t, target_component) }, \
         { "operator_latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_open_drone_id_system_update_t, operator_latitude) }, \
         { "operator_longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_open_drone_id_system_update_t, operator_longitude) }, \
         { "operator_altitude_geo", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_open_drone_id_system_update_t, operator_altitude_geo) }, \
         { "timestamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_open_drone_id_system_update_t, timestamp) }, \
         } \
}
#endif

/**
 * @brief Pack a open_drone_id_system_update message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID (0 for broadcast).
 * @param target_component  Component ID (0 for broadcast).
 * @param operator_latitude [degE7] Latitude of the operator. If unknown: 0 (both Lat/Lon).
 * @param operator_longitude [degE7] Longitude of the operator. If unknown: 0 (both Lat/Lon).
 * @param operator_altitude_geo [m] Geodetic altitude of the operator relative to WGS84. If unknown: -1000 m.
 * @param timestamp [s] 32 bit Unix Timestamp in seconds since 00:00:00 01/01/2019.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_open_drone_id_system_update_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, int32_t operator_latitude, int32_t operator_longitude, float operator_altitude_geo, uint32_t timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_LEN];
    _mav_put_int32_t(buf, 0, operator_latitude);
    _mav_put_int32_t(buf, 4, operator_longitude);
    _mav_put_float(buf, 8, operator_altitude_geo);
    _mav_put_uint32_t(buf, 12, timestamp);
    _mav_put_uint8_t(buf, 16, target_system);
    _mav_put_uint8_t(buf, 17, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_LEN);
#else
    mavlink_open_drone_id_system_update_t packet;
    packet.operator_latitude = operator_latitude;
    packet.operator_longitude = operator_longitude;
    packet.operator_altitude_geo = operator_altitude_geo;
    packet.timestamp = timestamp;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_CRC);
}

/**
 * @brief Pack a open_drone_id_system_update message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID (0 for broadcast).
 * @param target_component  Component ID (0 for broadcast).
 * @param operator_latitude [degE7] Latitude of the operator. If unknown: 0 (both Lat/Lon).
 * @param operator_longitude [degE7] Longitude of the operator. If unknown: 0 (both Lat/Lon).
 * @param operator_altitude_geo [m] Geodetic altitude of the operator relative to WGS84. If unknown: -1000 m.
 * @param timestamp [s] 32 bit Unix Timestamp in seconds since 00:00:00 01/01/2019.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_open_drone_id_system_update_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, int32_t operator_latitude, int32_t operator_longitude, float operator_altitude_geo, uint32_t timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_LEN];
    _mav_put_int32_t(buf, 0, operator_latitude);
    _mav_put_int32_t(buf, 4, operator_longitude);
    _mav_put_float(buf, 8, operator_altitude_geo);
    _mav_put_uint32_t(buf, 12, timestamp);
    _mav_put_uint8_t(buf, 16, target_system);
    _mav_put_uint8_t(buf, 17, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_LEN);
#else
    mavlink_open_drone_id_system_update_t packet;
    packet.operator_latitude = operator_latitude;
    packet.operator_longitude = operator_longitude;
    packet.operator_altitude_geo = operator_altitude_geo;
    packet.timestamp = timestamp;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_LEN);
#endif
}

/**
 * @brief Pack a open_drone_id_system_update message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID (0 for broadcast).
 * @param target_component  Component ID (0 for broadcast).
 * @param operator_latitude [degE7] Latitude of the operator. If unknown: 0 (both Lat/Lon).
 * @param operator_longitude [degE7] Longitude of the operator. If unknown: 0 (both Lat/Lon).
 * @param operator_altitude_geo [m] Geodetic altitude of the operator relative to WGS84. If unknown: -1000 m.
 * @param timestamp [s] 32 bit Unix Timestamp in seconds since 00:00:00 01/01/2019.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_open_drone_id_system_update_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,int32_t operator_latitude,int32_t operator_longitude,float operator_altitude_geo,uint32_t timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_LEN];
    _mav_put_int32_t(buf, 0, operator_latitude);
    _mav_put_int32_t(buf, 4, operator_longitude);
    _mav_put_float(buf, 8, operator_altitude_geo);
    _mav_put_uint32_t(buf, 12, timestamp);
    _mav_put_uint8_t(buf, 16, target_system);
    _mav_put_uint8_t(buf, 17, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_LEN);
#else
    mavlink_open_drone_id_system_update_t packet;
    packet.operator_latitude = operator_latitude;
    packet.operator_longitude = operator_longitude;
    packet.operator_altitude_geo = operator_altitude_geo;
    packet.timestamp = timestamp;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_CRC);
}

/**
 * @brief Encode a open_drone_id_system_update struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param open_drone_id_system_update C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_open_drone_id_system_update_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_open_drone_id_system_update_t* open_drone_id_system_update)
{
    return mavlink_msg_open_drone_id_system_update_pack(system_id, component_id, msg, open_drone_id_system_update->target_system, open_drone_id_system_update->target_component, open_drone_id_system_update->operator_latitude, open_drone_id_system_update->operator_longitude, open_drone_id_system_update->operator_altitude_geo, open_drone_id_system_update->timestamp);
}

/**
 * @brief Encode a open_drone_id_system_update struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param open_drone_id_system_update C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_open_drone_id_system_update_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_open_drone_id_system_update_t* open_drone_id_system_update)
{
    return mavlink_msg_open_drone_id_system_update_pack_chan(system_id, component_id, chan, msg, open_drone_id_system_update->target_system, open_drone_id_system_update->target_component, open_drone_id_system_update->operator_latitude, open_drone_id_system_update->operator_longitude, open_drone_id_system_update->operator_altitude_geo, open_drone_id_system_update->timestamp);
}

/**
 * @brief Encode a open_drone_id_system_update struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param open_drone_id_system_update C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_open_drone_id_system_update_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_open_drone_id_system_update_t* open_drone_id_system_update)
{
    return mavlink_msg_open_drone_id_system_update_pack_status(system_id, component_id, _status, msg,  open_drone_id_system_update->target_system, open_drone_id_system_update->target_component, open_drone_id_system_update->operator_latitude, open_drone_id_system_update->operator_longitude, open_drone_id_system_update->operator_altitude_geo, open_drone_id_system_update->timestamp);
}

/**
 * @brief Send a open_drone_id_system_update message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID (0 for broadcast).
 * @param target_component  Component ID (0 for broadcast).
 * @param operator_latitude [degE7] Latitude of the operator. If unknown: 0 (both Lat/Lon).
 * @param operator_longitude [degE7] Longitude of the operator. If unknown: 0 (both Lat/Lon).
 * @param operator_altitude_geo [m] Geodetic altitude of the operator relative to WGS84. If unknown: -1000 m.
 * @param timestamp [s] 32 bit Unix Timestamp in seconds since 00:00:00 01/01/2019.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_open_drone_id_system_update_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, int32_t operator_latitude, int32_t operator_longitude, float operator_altitude_geo, uint32_t timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_LEN];
    _mav_put_int32_t(buf, 0, operator_latitude);
    _mav_put_int32_t(buf, 4, operator_longitude);
    _mav_put_float(buf, 8, operator_altitude_geo);
    _mav_put_uint32_t(buf, 12, timestamp);
    _mav_put_uint8_t(buf, 16, target_system);
    _mav_put_uint8_t(buf, 17, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE, buf, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_CRC);
#else
    mavlink_open_drone_id_system_update_t packet;
    packet.operator_latitude = operator_latitude;
    packet.operator_longitude = operator_longitude;
    packet.operator_altitude_geo = operator_altitude_geo;
    packet.timestamp = timestamp;
    packet.target_system = target_system;
    packet.target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE, (const char *)&packet, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_CRC);
#endif
}

/**
 * @brief Send a open_drone_id_system_update message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_open_drone_id_system_update_send_struct(mavlink_channel_t chan, const mavlink_open_drone_id_system_update_t* open_drone_id_system_update)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_open_drone_id_system_update_send(chan, open_drone_id_system_update->target_system, open_drone_id_system_update->target_component, open_drone_id_system_update->operator_latitude, open_drone_id_system_update->operator_longitude, open_drone_id_system_update->operator_altitude_geo, open_drone_id_system_update->timestamp);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE, (const char *)open_drone_id_system_update, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_CRC);
#endif
}

#if MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_open_drone_id_system_update_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, int32_t operator_latitude, int32_t operator_longitude, float operator_altitude_geo, uint32_t timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, operator_latitude);
    _mav_put_int32_t(buf, 4, operator_longitude);
    _mav_put_float(buf, 8, operator_altitude_geo);
    _mav_put_uint32_t(buf, 12, timestamp);
    _mav_put_uint8_t(buf, 16, target_system);
    _mav_put_uint8_t(buf, 17, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE, buf, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_CRC);
#else
    mavlink_open_drone_id_system_update_t *packet = (mavlink_open_drone_id_system_update_t *)msgbuf;
    packet->operator_latitude = operator_latitude;
    packet->operator_longitude = operator_longitude;
    packet->operator_altitude_geo = operator_altitude_geo;
    packet->timestamp = timestamp;
    packet->target_system = target_system;
    packet->target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE, (const char *)packet, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_CRC);
#endif
}
#endif

#endif

// MESSAGE OPEN_DRONE_ID_SYSTEM_UPDATE UNPACKING


/**
 * @brief Get field target_system from open_drone_id_system_update message
 *
 * @return  System ID (0 for broadcast).
 */
static inline uint8_t mavlink_msg_open_drone_id_system_update_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field target_component from open_drone_id_system_update message
 *
 * @return  Component ID (0 for broadcast).
 */
static inline uint8_t mavlink_msg_open_drone_id_system_update_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field operator_latitude from open_drone_id_system_update message
 *
 * @return [degE7] Latitude of the operator. If unknown: 0 (both Lat/Lon).
 */
static inline int32_t mavlink_msg_open_drone_id_system_update_get_operator_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field operator_longitude from open_drone_id_system_update message
 *
 * @return [degE7] Longitude of the operator. If unknown: 0 (both Lat/Lon).
 */
static inline int32_t mavlink_msg_open_drone_id_system_update_get_operator_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field operator_altitude_geo from open_drone_id_system_update message
 *
 * @return [m] Geodetic altitude of the operator relative to WGS84. If unknown: -1000 m.
 */
static inline float mavlink_msg_open_drone_id_system_update_get_operator_altitude_geo(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field timestamp from open_drone_id_system_update message
 *
 * @return [s] 32 bit Unix Timestamp in seconds since 00:00:00 01/01/2019.
 */
static inline uint32_t mavlink_msg_open_drone_id_system_update_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Decode a open_drone_id_system_update message into a struct
 *
 * @param msg The message to decode
 * @param open_drone_id_system_update C-struct to decode the message contents into
 */
static inline void mavlink_msg_open_drone_id_system_update_decode(const mavlink_message_t* msg, mavlink_open_drone_id_system_update_t* open_drone_id_system_update)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    open_drone_id_system_update->operator_latitude = mavlink_msg_open_drone_id_system_update_get_operator_latitude(msg);
    open_drone_id_system_update->operator_longitude = mavlink_msg_open_drone_id_system_update_get_operator_longitude(msg);
    open_drone_id_system_update->operator_altitude_geo = mavlink_msg_open_drone_id_system_update_get_operator_altitude_geo(msg);
    open_drone_id_system_update->timestamp = mavlink_msg_open_drone_id_system_update_get_timestamp(msg);
    open_drone_id_system_update->target_system = mavlink_msg_open_drone_id_system_update_get_target_system(msg);
    open_drone_id_system_update->target_component = mavlink_msg_open_drone_id_system_update_get_target_component(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_LEN? msg->len : MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_LEN;
        memset(open_drone_id_system_update, 0, MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE_LEN);
    memcpy(open_drone_id_system_update, _MAV_PAYLOAD(msg), len);
#endif
}
