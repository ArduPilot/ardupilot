#pragma once
// MESSAGE GPS_GLOBAL_ORIGIN PACKING

#define MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN 49

MAVPACKED(
typedef struct __mavlink_gps_global_origin_t {
 int32_t latitude; /*< Latitude (WGS84), in degrees * 1E7*/
 int32_t longitude; /*< Longitude (WGS84), in degrees * 1E7*/
 int32_t altitude; /*< Altitude (AMSL), in meters * 1000 (positive for up)*/
}) mavlink_gps_global_origin_t;

#define MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_LEN 12
#define MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_MIN_LEN 12
#define MAVLINK_MSG_ID_49_LEN 12
#define MAVLINK_MSG_ID_49_MIN_LEN 12

#define MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_CRC 39
#define MAVLINK_MSG_ID_49_CRC 39



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GPS_GLOBAL_ORIGIN { \
    49, \
    "GPS_GLOBAL_ORIGIN", \
    3, \
    {  { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_gps_global_origin_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_gps_global_origin_t, longitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_gps_global_origin_t, altitude) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GPS_GLOBAL_ORIGIN { \
    "GPS_GLOBAL_ORIGIN", \
    3, \
    {  { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_gps_global_origin_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_gps_global_origin_t, longitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_gps_global_origin_t, altitude) }, \
         } \
}
#endif

/**
 * @brief Pack a gps_global_origin message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param latitude Latitude (WGS84), in degrees * 1E7
 * @param longitude Longitude (WGS84), in degrees * 1E7
 * @param altitude Altitude (AMSL), in meters * 1000 (positive for up)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_global_origin_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t latitude, int32_t longitude, int32_t altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_LEN];
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_int32_t(buf, 8, altitude);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_LEN);
#else
    mavlink_gps_global_origin_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_MIN_LEN, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_LEN, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_CRC);
}

/**
 * @brief Pack a gps_global_origin message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param latitude Latitude (WGS84), in degrees * 1E7
 * @param longitude Longitude (WGS84), in degrees * 1E7
 * @param altitude Altitude (AMSL), in meters * 1000 (positive for up)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_global_origin_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t latitude,int32_t longitude,int32_t altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_LEN];
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_int32_t(buf, 8, altitude);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_LEN);
#else
    mavlink_gps_global_origin_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_MIN_LEN, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_LEN, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_CRC);
}

/**
 * @brief Encode a gps_global_origin struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gps_global_origin C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_global_origin_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gps_global_origin_t* gps_global_origin)
{
    return mavlink_msg_gps_global_origin_pack(system_id, component_id, msg, gps_global_origin->latitude, gps_global_origin->longitude, gps_global_origin->altitude);
}

/**
 * @brief Encode a gps_global_origin struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gps_global_origin C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_global_origin_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gps_global_origin_t* gps_global_origin)
{
    return mavlink_msg_gps_global_origin_pack_chan(system_id, component_id, chan, msg, gps_global_origin->latitude, gps_global_origin->longitude, gps_global_origin->altitude);
}

/**
 * @brief Send a gps_global_origin message
 * @param chan MAVLink channel to send the message
 *
 * @param latitude Latitude (WGS84), in degrees * 1E7
 * @param longitude Longitude (WGS84), in degrees * 1E7
 * @param altitude Altitude (AMSL), in meters * 1000 (positive for up)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gps_global_origin_send(mavlink_channel_t chan, int32_t latitude, int32_t longitude, int32_t altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_LEN];
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_int32_t(buf, 8, altitude);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN, buf, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_MIN_LEN, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_LEN, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_CRC);
#else
    mavlink_gps_global_origin_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN, (const char *)&packet, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_MIN_LEN, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_LEN, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_CRC);
#endif
}

/**
 * @brief Send a gps_global_origin message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gps_global_origin_send_struct(mavlink_channel_t chan, const mavlink_gps_global_origin_t* gps_global_origin)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gps_global_origin_send(chan, gps_global_origin->latitude, gps_global_origin->longitude, gps_global_origin->altitude);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN, (const char *)gps_global_origin, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_MIN_LEN, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_LEN, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_CRC);
#endif
}

#if MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gps_global_origin_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t latitude, int32_t longitude, int32_t altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_int32_t(buf, 8, altitude);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN, buf, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_MIN_LEN, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_LEN, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_CRC);
#else
    mavlink_gps_global_origin_t *packet = (mavlink_gps_global_origin_t *)msgbuf;
    packet->latitude = latitude;
    packet->longitude = longitude;
    packet->altitude = altitude;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN, (const char *)packet, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_MIN_LEN, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_LEN, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_CRC);
#endif
}
#endif

#endif

// MESSAGE GPS_GLOBAL_ORIGIN UNPACKING


/**
 * @brief Get field latitude from gps_global_origin message
 *
 * @return Latitude (WGS84), in degrees * 1E7
 */
static inline int32_t mavlink_msg_gps_global_origin_get_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field longitude from gps_global_origin message
 *
 * @return Longitude (WGS84), in degrees * 1E7
 */
static inline int32_t mavlink_msg_gps_global_origin_get_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field altitude from gps_global_origin message
 *
 * @return Altitude (AMSL), in meters * 1000 (positive for up)
 */
static inline int32_t mavlink_msg_gps_global_origin_get_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Decode a gps_global_origin message into a struct
 *
 * @param msg The message to decode
 * @param gps_global_origin C-struct to decode the message contents into
 */
static inline void mavlink_msg_gps_global_origin_decode(const mavlink_message_t* msg, mavlink_gps_global_origin_t* gps_global_origin)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gps_global_origin->latitude = mavlink_msg_gps_global_origin_get_latitude(msg);
    gps_global_origin->longitude = mavlink_msg_gps_global_origin_get_longitude(msg);
    gps_global_origin->altitude = mavlink_msg_gps_global_origin_get_altitude(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_LEN? msg->len : MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_LEN;
        memset(gps_global_origin, 0, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_LEN);
    memcpy(gps_global_origin, _MAV_PAYLOAD(msg), len);
#endif
}
