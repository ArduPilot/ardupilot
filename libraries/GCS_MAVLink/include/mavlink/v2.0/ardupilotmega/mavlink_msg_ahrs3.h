#pragma once
// MESSAGE AHRS3 PACKING

#define MAVLINK_MSG_ID_AHRS3 182


typedef struct __mavlink_ahrs3_t {
 float roll; /*< [rad] Roll angle.*/
 float pitch; /*< [rad] Pitch angle.*/
 float yaw; /*< [rad] Yaw angle.*/
 float altitude; /*< [m] Altitude (MSL).*/
 int32_t lat; /*< [degE7] Latitude.*/
 int32_t lng; /*< [degE7] Longitude.*/
 float v1; /*<  Test variable1.*/
 float v2; /*<  Test variable2.*/
 float v3; /*<  Test variable3.*/
 float v4; /*<  Test variable4.*/
} mavlink_ahrs3_t;

#define MAVLINK_MSG_ID_AHRS3_LEN 40
#define MAVLINK_MSG_ID_AHRS3_MIN_LEN 40
#define MAVLINK_MSG_ID_182_LEN 40
#define MAVLINK_MSG_ID_182_MIN_LEN 40

#define MAVLINK_MSG_ID_AHRS3_CRC 229
#define MAVLINK_MSG_ID_182_CRC 229



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AHRS3 { \
    182, \
    "AHRS3", \
    10, \
    {  { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_ahrs3_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_ahrs3_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ahrs3_t, yaw) }, \
         { "altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ahrs3_t, altitude) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_ahrs3_t, lat) }, \
         { "lng", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_ahrs3_t, lng) }, \
         { "v1", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_ahrs3_t, v1) }, \
         { "v2", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_ahrs3_t, v2) }, \
         { "v3", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_ahrs3_t, v3) }, \
         { "v4", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_ahrs3_t, v4) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AHRS3 { \
    "AHRS3", \
    10, \
    {  { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_ahrs3_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_ahrs3_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ahrs3_t, yaw) }, \
         { "altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ahrs3_t, altitude) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_ahrs3_t, lat) }, \
         { "lng", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_ahrs3_t, lng) }, \
         { "v1", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_ahrs3_t, v1) }, \
         { "v2", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_ahrs3_t, v2) }, \
         { "v3", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_ahrs3_t, v3) }, \
         { "v4", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_ahrs3_t, v4) }, \
         } \
}
#endif

/**
 * @brief Pack a ahrs3 message
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
 * @param v1  Test variable1.
 * @param v2  Test variable2.
 * @param v3  Test variable3.
 * @param v4  Test variable4.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ahrs3_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float roll, float pitch, float yaw, float altitude, int32_t lat, int32_t lng, float v1, float v2, float v3, float v4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AHRS3_LEN];
    _mav_put_float(buf, 0, roll);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);
    _mav_put_float(buf, 12, altitude);
    _mav_put_int32_t(buf, 16, lat);
    _mav_put_int32_t(buf, 20, lng);
    _mav_put_float(buf, 24, v1);
    _mav_put_float(buf, 28, v2);
    _mav_put_float(buf, 32, v3);
    _mav_put_float(buf, 36, v4);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AHRS3_LEN);
#else
    mavlink_ahrs3_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.altitude = altitude;
    packet.lat = lat;
    packet.lng = lng;
    packet.v1 = v1;
    packet.v2 = v2;
    packet.v3 = v3;
    packet.v4 = v4;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AHRS3_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AHRS3;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AHRS3_MIN_LEN, MAVLINK_MSG_ID_AHRS3_LEN, MAVLINK_MSG_ID_AHRS3_CRC);
}

/**
 * @brief Pack a ahrs3 message
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
 * @param v1  Test variable1.
 * @param v2  Test variable2.
 * @param v3  Test variable3.
 * @param v4  Test variable4.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ahrs3_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float roll, float pitch, float yaw, float altitude, int32_t lat, int32_t lng, float v1, float v2, float v3, float v4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AHRS3_LEN];
    _mav_put_float(buf, 0, roll);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);
    _mav_put_float(buf, 12, altitude);
    _mav_put_int32_t(buf, 16, lat);
    _mav_put_int32_t(buf, 20, lng);
    _mav_put_float(buf, 24, v1);
    _mav_put_float(buf, 28, v2);
    _mav_put_float(buf, 32, v3);
    _mav_put_float(buf, 36, v4);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AHRS3_LEN);
#else
    mavlink_ahrs3_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.altitude = altitude;
    packet.lat = lat;
    packet.lng = lng;
    packet.v1 = v1;
    packet.v2 = v2;
    packet.v3 = v3;
    packet.v4 = v4;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AHRS3_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AHRS3;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AHRS3_MIN_LEN, MAVLINK_MSG_ID_AHRS3_LEN, MAVLINK_MSG_ID_AHRS3_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AHRS3_MIN_LEN, MAVLINK_MSG_ID_AHRS3_LEN);
#endif
}

/**
 * @brief Pack a ahrs3 message on a channel
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
 * @param v1  Test variable1.
 * @param v2  Test variable2.
 * @param v3  Test variable3.
 * @param v4  Test variable4.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ahrs3_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float roll,float pitch,float yaw,float altitude,int32_t lat,int32_t lng,float v1,float v2,float v3,float v4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AHRS3_LEN];
    _mav_put_float(buf, 0, roll);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);
    _mav_put_float(buf, 12, altitude);
    _mav_put_int32_t(buf, 16, lat);
    _mav_put_int32_t(buf, 20, lng);
    _mav_put_float(buf, 24, v1);
    _mav_put_float(buf, 28, v2);
    _mav_put_float(buf, 32, v3);
    _mav_put_float(buf, 36, v4);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AHRS3_LEN);
#else
    mavlink_ahrs3_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.altitude = altitude;
    packet.lat = lat;
    packet.lng = lng;
    packet.v1 = v1;
    packet.v2 = v2;
    packet.v3 = v3;
    packet.v4 = v4;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AHRS3_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AHRS3;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AHRS3_MIN_LEN, MAVLINK_MSG_ID_AHRS3_LEN, MAVLINK_MSG_ID_AHRS3_CRC);
}

/**
 * @brief Encode a ahrs3 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ahrs3 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ahrs3_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ahrs3_t* ahrs3)
{
    return mavlink_msg_ahrs3_pack(system_id, component_id, msg, ahrs3->roll, ahrs3->pitch, ahrs3->yaw, ahrs3->altitude, ahrs3->lat, ahrs3->lng, ahrs3->v1, ahrs3->v2, ahrs3->v3, ahrs3->v4);
}

/**
 * @brief Encode a ahrs3 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ahrs3 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ahrs3_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ahrs3_t* ahrs3)
{
    return mavlink_msg_ahrs3_pack_chan(system_id, component_id, chan, msg, ahrs3->roll, ahrs3->pitch, ahrs3->yaw, ahrs3->altitude, ahrs3->lat, ahrs3->lng, ahrs3->v1, ahrs3->v2, ahrs3->v3, ahrs3->v4);
}

/**
 * @brief Encode a ahrs3 struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param ahrs3 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ahrs3_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_ahrs3_t* ahrs3)
{
    return mavlink_msg_ahrs3_pack_status(system_id, component_id, _status, msg,  ahrs3->roll, ahrs3->pitch, ahrs3->yaw, ahrs3->altitude, ahrs3->lat, ahrs3->lng, ahrs3->v1, ahrs3->v2, ahrs3->v3, ahrs3->v4);
}

/**
 * @brief Send a ahrs3 message
 * @param chan MAVLink channel to send the message
 *
 * @param roll [rad] Roll angle.
 * @param pitch [rad] Pitch angle.
 * @param yaw [rad] Yaw angle.
 * @param altitude [m] Altitude (MSL).
 * @param lat [degE7] Latitude.
 * @param lng [degE7] Longitude.
 * @param v1  Test variable1.
 * @param v2  Test variable2.
 * @param v3  Test variable3.
 * @param v4  Test variable4.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ahrs3_send(mavlink_channel_t chan, float roll, float pitch, float yaw, float altitude, int32_t lat, int32_t lng, float v1, float v2, float v3, float v4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AHRS3_LEN];
    _mav_put_float(buf, 0, roll);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);
    _mav_put_float(buf, 12, altitude);
    _mav_put_int32_t(buf, 16, lat);
    _mav_put_int32_t(buf, 20, lng);
    _mav_put_float(buf, 24, v1);
    _mav_put_float(buf, 28, v2);
    _mav_put_float(buf, 32, v3);
    _mav_put_float(buf, 36, v4);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AHRS3, buf, MAVLINK_MSG_ID_AHRS3_MIN_LEN, MAVLINK_MSG_ID_AHRS3_LEN, MAVLINK_MSG_ID_AHRS3_CRC);
#else
    mavlink_ahrs3_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.altitude = altitude;
    packet.lat = lat;
    packet.lng = lng;
    packet.v1 = v1;
    packet.v2 = v2;
    packet.v3 = v3;
    packet.v4 = v4;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AHRS3, (const char *)&packet, MAVLINK_MSG_ID_AHRS3_MIN_LEN, MAVLINK_MSG_ID_AHRS3_LEN, MAVLINK_MSG_ID_AHRS3_CRC);
#endif
}

/**
 * @brief Send a ahrs3 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ahrs3_send_struct(mavlink_channel_t chan, const mavlink_ahrs3_t* ahrs3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ahrs3_send(chan, ahrs3->roll, ahrs3->pitch, ahrs3->yaw, ahrs3->altitude, ahrs3->lat, ahrs3->lng, ahrs3->v1, ahrs3->v2, ahrs3->v3, ahrs3->v4);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AHRS3, (const char *)ahrs3, MAVLINK_MSG_ID_AHRS3_MIN_LEN, MAVLINK_MSG_ID_AHRS3_LEN, MAVLINK_MSG_ID_AHRS3_CRC);
#endif
}

#if MAVLINK_MSG_ID_AHRS3_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ahrs3_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float roll, float pitch, float yaw, float altitude, int32_t lat, int32_t lng, float v1, float v2, float v3, float v4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, roll);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);
    _mav_put_float(buf, 12, altitude);
    _mav_put_int32_t(buf, 16, lat);
    _mav_put_int32_t(buf, 20, lng);
    _mav_put_float(buf, 24, v1);
    _mav_put_float(buf, 28, v2);
    _mav_put_float(buf, 32, v3);
    _mav_put_float(buf, 36, v4);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AHRS3, buf, MAVLINK_MSG_ID_AHRS3_MIN_LEN, MAVLINK_MSG_ID_AHRS3_LEN, MAVLINK_MSG_ID_AHRS3_CRC);
#else
    mavlink_ahrs3_t *packet = (mavlink_ahrs3_t *)msgbuf;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;
    packet->altitude = altitude;
    packet->lat = lat;
    packet->lng = lng;
    packet->v1 = v1;
    packet->v2 = v2;
    packet->v3 = v3;
    packet->v4 = v4;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AHRS3, (const char *)packet, MAVLINK_MSG_ID_AHRS3_MIN_LEN, MAVLINK_MSG_ID_AHRS3_LEN, MAVLINK_MSG_ID_AHRS3_CRC);
#endif
}
#endif

#endif

// MESSAGE AHRS3 UNPACKING


/**
 * @brief Get field roll from ahrs3 message
 *
 * @return [rad] Roll angle.
 */
static inline float mavlink_msg_ahrs3_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pitch from ahrs3 message
 *
 * @return [rad] Pitch angle.
 */
static inline float mavlink_msg_ahrs3_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field yaw from ahrs3 message
 *
 * @return [rad] Yaw angle.
 */
static inline float mavlink_msg_ahrs3_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field altitude from ahrs3 message
 *
 * @return [m] Altitude (MSL).
 */
static inline float mavlink_msg_ahrs3_get_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field lat from ahrs3 message
 *
 * @return [degE7] Latitude.
 */
static inline int32_t mavlink_msg_ahrs3_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field lng from ahrs3 message
 *
 * @return [degE7] Longitude.
 */
static inline int32_t mavlink_msg_ahrs3_get_lng(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  20);
}

/**
 * @brief Get field v1 from ahrs3 message
 *
 * @return  Test variable1.
 */
static inline float mavlink_msg_ahrs3_get_v1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field v2 from ahrs3 message
 *
 * @return  Test variable2.
 */
static inline float mavlink_msg_ahrs3_get_v2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field v3 from ahrs3 message
 *
 * @return  Test variable3.
 */
static inline float mavlink_msg_ahrs3_get_v3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field v4 from ahrs3 message
 *
 * @return  Test variable4.
 */
static inline float mavlink_msg_ahrs3_get_v4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Decode a ahrs3 message into a struct
 *
 * @param msg The message to decode
 * @param ahrs3 C-struct to decode the message contents into
 */
static inline void mavlink_msg_ahrs3_decode(const mavlink_message_t* msg, mavlink_ahrs3_t* ahrs3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    ahrs3->roll = mavlink_msg_ahrs3_get_roll(msg);
    ahrs3->pitch = mavlink_msg_ahrs3_get_pitch(msg);
    ahrs3->yaw = mavlink_msg_ahrs3_get_yaw(msg);
    ahrs3->altitude = mavlink_msg_ahrs3_get_altitude(msg);
    ahrs3->lat = mavlink_msg_ahrs3_get_lat(msg);
    ahrs3->lng = mavlink_msg_ahrs3_get_lng(msg);
    ahrs3->v1 = mavlink_msg_ahrs3_get_v1(msg);
    ahrs3->v2 = mavlink_msg_ahrs3_get_v2(msg);
    ahrs3->v3 = mavlink_msg_ahrs3_get_v3(msg);
    ahrs3->v4 = mavlink_msg_ahrs3_get_v4(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AHRS3_LEN? msg->len : MAVLINK_MSG_ID_AHRS3_LEN;
        memset(ahrs3, 0, MAVLINK_MSG_ID_AHRS3_LEN);
    memcpy(ahrs3, _MAV_PAYLOAD(msg), len);
#endif
}
