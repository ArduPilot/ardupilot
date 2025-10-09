#pragma once
// MESSAGE AHRS2 PACKING

#define MAVLINK_MSG_ID_AHRS2 178


typedef struct __mavlink_ahrs2_t {
 float roll; /*< [rad] Roll angle.*/
 float pitch; /*< [rad] Pitch angle.*/
 float yaw; /*< [rad] Yaw angle.*/
 float altitude; /*< [m] Altitude (MSL).*/
 int32_t lat; /*< [degE7] Latitude.*/
 int32_t lng; /*< [degE7] Longitude.*/
} mavlink_ahrs2_t;

#define MAVLINK_MSG_ID_AHRS2_LEN 24
#define MAVLINK_MSG_ID_AHRS2_MIN_LEN 24
#define MAVLINK_MSG_ID_178_LEN 24
#define MAVLINK_MSG_ID_178_MIN_LEN 24

#define MAVLINK_MSG_ID_AHRS2_CRC 47
#define MAVLINK_MSG_ID_178_CRC 47



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AHRS2 { \
    178, \
    "AHRS2", \
    6, \
    {  { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_ahrs2_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_ahrs2_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ahrs2_t, yaw) }, \
         { "altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ahrs2_t, altitude) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_ahrs2_t, lat) }, \
         { "lng", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_ahrs2_t, lng) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AHRS2 { \
    "AHRS2", \
    6, \
    {  { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_ahrs2_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_ahrs2_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ahrs2_t, yaw) }, \
         { "altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ahrs2_t, altitude) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_ahrs2_t, lat) }, \
         { "lng", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_ahrs2_t, lng) }, \
         } \
}
#endif

/**
 * @brief Pack a ahrs2 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param roll [rad] Roll angle.
 * @param pitch [rad] Pitch angle.
 * @param yaw [rad] Yaw angle.
 * @param altitude [m] Altitude (MSL).
 * @param lat [degE7] Latitude.
 * @param lng [degE7] Longitude.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ahrs2_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float roll, float pitch, float yaw, float altitude, int32_t lat, int32_t lng)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AHRS2_LEN];
    _mav_put_float(buf, 0, roll);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);
    _mav_put_float(buf, 12, altitude);
    _mav_put_int32_t(buf, 16, lat);
    _mav_put_int32_t(buf, 20, lng);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AHRS2_LEN);
#else
    mavlink_ahrs2_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.altitude = altitude;
    packet.lat = lat;
    packet.lng = lng;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AHRS2_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AHRS2;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AHRS2_MIN_LEN, MAVLINK_MSG_ID_AHRS2_LEN, MAVLINK_MSG_ID_AHRS2_CRC);
}

/**
 * @brief Pack a ahrs2 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param roll [rad] Roll angle.
 * @param pitch [rad] Pitch angle.
 * @param yaw [rad] Yaw angle.
 * @param altitude [m] Altitude (MSL).
 * @param lat [degE7] Latitude.
 * @param lng [degE7] Longitude.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ahrs2_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float roll, float pitch, float yaw, float altitude, int32_t lat, int32_t lng)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AHRS2_LEN];
    _mav_put_float(buf, 0, roll);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);
    _mav_put_float(buf, 12, altitude);
    _mav_put_int32_t(buf, 16, lat);
    _mav_put_int32_t(buf, 20, lng);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AHRS2_LEN);
#else
    mavlink_ahrs2_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.altitude = altitude;
    packet.lat = lat;
    packet.lng = lng;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AHRS2_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AHRS2;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AHRS2_MIN_LEN, MAVLINK_MSG_ID_AHRS2_LEN, MAVLINK_MSG_ID_AHRS2_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AHRS2_MIN_LEN, MAVLINK_MSG_ID_AHRS2_LEN);
#endif
}

/**
 * @brief Pack a ahrs2 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param roll [rad] Roll angle.
 * @param pitch [rad] Pitch angle.
 * @param yaw [rad] Yaw angle.
 * @param altitude [m] Altitude (MSL).
 * @param lat [degE7] Latitude.
 * @param lng [degE7] Longitude.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ahrs2_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float roll,float pitch,float yaw,float altitude,int32_t lat,int32_t lng)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AHRS2_LEN];
    _mav_put_float(buf, 0, roll);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);
    _mav_put_float(buf, 12, altitude);
    _mav_put_int32_t(buf, 16, lat);
    _mav_put_int32_t(buf, 20, lng);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AHRS2_LEN);
#else
    mavlink_ahrs2_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.altitude = altitude;
    packet.lat = lat;
    packet.lng = lng;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AHRS2_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AHRS2;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AHRS2_MIN_LEN, MAVLINK_MSG_ID_AHRS2_LEN, MAVLINK_MSG_ID_AHRS2_CRC);
}

/**
 * @brief Encode a ahrs2 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ahrs2 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ahrs2_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ahrs2_t* ahrs2)
{
    return mavlink_msg_ahrs2_pack(system_id, component_id, msg, ahrs2->roll, ahrs2->pitch, ahrs2->yaw, ahrs2->altitude, ahrs2->lat, ahrs2->lng);
}

/**
 * @brief Encode a ahrs2 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ahrs2 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ahrs2_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ahrs2_t* ahrs2)
{
    return mavlink_msg_ahrs2_pack_chan(system_id, component_id, chan, msg, ahrs2->roll, ahrs2->pitch, ahrs2->yaw, ahrs2->altitude, ahrs2->lat, ahrs2->lng);
}

/**
 * @brief Encode a ahrs2 struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param ahrs2 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ahrs2_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_ahrs2_t* ahrs2)
{
    return mavlink_msg_ahrs2_pack_status(system_id, component_id, _status, msg,  ahrs2->roll, ahrs2->pitch, ahrs2->yaw, ahrs2->altitude, ahrs2->lat, ahrs2->lng);
}

/**
 * @brief Send a ahrs2 message
 * @param chan MAVLink channel to send the message
 *
 * @param roll [rad] Roll angle.
 * @param pitch [rad] Pitch angle.
 * @param yaw [rad] Yaw angle.
 * @param altitude [m] Altitude (MSL).
 * @param lat [degE7] Latitude.
 * @param lng [degE7] Longitude.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ahrs2_send(mavlink_channel_t chan, float roll, float pitch, float yaw, float altitude, int32_t lat, int32_t lng)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AHRS2_LEN];
    _mav_put_float(buf, 0, roll);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);
    _mav_put_float(buf, 12, altitude);
    _mav_put_int32_t(buf, 16, lat);
    _mav_put_int32_t(buf, 20, lng);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AHRS2, buf, MAVLINK_MSG_ID_AHRS2_MIN_LEN, MAVLINK_MSG_ID_AHRS2_LEN, MAVLINK_MSG_ID_AHRS2_CRC);
#else
    mavlink_ahrs2_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.altitude = altitude;
    packet.lat = lat;
    packet.lng = lng;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AHRS2, (const char *)&packet, MAVLINK_MSG_ID_AHRS2_MIN_LEN, MAVLINK_MSG_ID_AHRS2_LEN, MAVLINK_MSG_ID_AHRS2_CRC);
#endif
}

/**
 * @brief Send a ahrs2 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ahrs2_send_struct(mavlink_channel_t chan, const mavlink_ahrs2_t* ahrs2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ahrs2_send(chan, ahrs2->roll, ahrs2->pitch, ahrs2->yaw, ahrs2->altitude, ahrs2->lat, ahrs2->lng);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AHRS2, (const char *)ahrs2, MAVLINK_MSG_ID_AHRS2_MIN_LEN, MAVLINK_MSG_ID_AHRS2_LEN, MAVLINK_MSG_ID_AHRS2_CRC);
#endif
}

#if MAVLINK_MSG_ID_AHRS2_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ahrs2_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float roll, float pitch, float yaw, float altitude, int32_t lat, int32_t lng)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, roll);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);
    _mav_put_float(buf, 12, altitude);
    _mav_put_int32_t(buf, 16, lat);
    _mav_put_int32_t(buf, 20, lng);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AHRS2, buf, MAVLINK_MSG_ID_AHRS2_MIN_LEN, MAVLINK_MSG_ID_AHRS2_LEN, MAVLINK_MSG_ID_AHRS2_CRC);
#else
    mavlink_ahrs2_t *packet = (mavlink_ahrs2_t *)msgbuf;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;
    packet->altitude = altitude;
    packet->lat = lat;
    packet->lng = lng;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AHRS2, (const char *)packet, MAVLINK_MSG_ID_AHRS2_MIN_LEN, MAVLINK_MSG_ID_AHRS2_LEN, MAVLINK_MSG_ID_AHRS2_CRC);
#endif
}
#endif

#endif

// MESSAGE AHRS2 UNPACKING


/**
 * @brief Get field roll from ahrs2 message
 *
 * @return [rad] Roll angle.
 */
static inline float mavlink_msg_ahrs2_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pitch from ahrs2 message
 *
 * @return [rad] Pitch angle.
 */
static inline float mavlink_msg_ahrs2_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field yaw from ahrs2 message
 *
 * @return [rad] Yaw angle.
 */
static inline float mavlink_msg_ahrs2_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field altitude from ahrs2 message
 *
 * @return [m] Altitude (MSL).
 */
static inline float mavlink_msg_ahrs2_get_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field lat from ahrs2 message
 *
 * @return [degE7] Latitude.
 */
static inline int32_t mavlink_msg_ahrs2_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field lng from ahrs2 message
 *
 * @return [degE7] Longitude.
 */
static inline int32_t mavlink_msg_ahrs2_get_lng(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  20);
}

/**
 * @brief Decode a ahrs2 message into a struct
 *
 * @param msg The message to decode
 * @param ahrs2 C-struct to decode the message contents into
 */
static inline void mavlink_msg_ahrs2_decode(const mavlink_message_t* msg, mavlink_ahrs2_t* ahrs2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    ahrs2->roll = mavlink_msg_ahrs2_get_roll(msg);
    ahrs2->pitch = mavlink_msg_ahrs2_get_pitch(msg);
    ahrs2->yaw = mavlink_msg_ahrs2_get_yaw(msg);
    ahrs2->altitude = mavlink_msg_ahrs2_get_altitude(msg);
    ahrs2->lat = mavlink_msg_ahrs2_get_lat(msg);
    ahrs2->lng = mavlink_msg_ahrs2_get_lng(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AHRS2_LEN? msg->len : MAVLINK_MSG_ID_AHRS2_LEN;
        memset(ahrs2, 0, MAVLINK_MSG_ID_AHRS2_LEN);
    memcpy(ahrs2, _MAV_PAYLOAD(msg), len);
#endif
}
