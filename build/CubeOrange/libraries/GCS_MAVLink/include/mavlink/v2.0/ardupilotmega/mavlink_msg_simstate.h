#pragma once
// MESSAGE SIMSTATE PACKING

#define MAVLINK_MSG_ID_SIMSTATE 164


typedef struct __mavlink_simstate_t {
 float roll; /*< [rad] Roll angle.*/
 float pitch; /*< [rad] Pitch angle.*/
 float yaw; /*< [rad] Yaw angle.*/
 float xacc; /*< [m/s/s] X acceleration.*/
 float yacc; /*< [m/s/s] Y acceleration.*/
 float zacc; /*< [m/s/s] Z acceleration.*/
 float xgyro; /*< [rad/s] Angular speed around X axis.*/
 float ygyro; /*< [rad/s] Angular speed around Y axis.*/
 float zgyro; /*< [rad/s] Angular speed around Z axis.*/
 int32_t lat; /*< [degE7] Latitude.*/
 int32_t lng; /*< [degE7] Longitude.*/
} mavlink_simstate_t;

#define MAVLINK_MSG_ID_SIMSTATE_LEN 44
#define MAVLINK_MSG_ID_SIMSTATE_MIN_LEN 44
#define MAVLINK_MSG_ID_164_LEN 44
#define MAVLINK_MSG_ID_164_MIN_LEN 44

#define MAVLINK_MSG_ID_SIMSTATE_CRC 154
#define MAVLINK_MSG_ID_164_CRC 154



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SIMSTATE { \
    164, \
    "SIMSTATE", \
    11, \
    {  { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_simstate_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_simstate_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_simstate_t, yaw) }, \
         { "xacc", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_simstate_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_simstate_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_simstate_t, zacc) }, \
         { "xgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_simstate_t, xgyro) }, \
         { "ygyro", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_simstate_t, ygyro) }, \
         { "zgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_simstate_t, zgyro) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 36, offsetof(mavlink_simstate_t, lat) }, \
         { "lng", NULL, MAVLINK_TYPE_INT32_T, 0, 40, offsetof(mavlink_simstate_t, lng) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SIMSTATE { \
    "SIMSTATE", \
    11, \
    {  { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_simstate_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_simstate_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_simstate_t, yaw) }, \
         { "xacc", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_simstate_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_simstate_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_simstate_t, zacc) }, \
         { "xgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_simstate_t, xgyro) }, \
         { "ygyro", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_simstate_t, ygyro) }, \
         { "zgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_simstate_t, zgyro) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 36, offsetof(mavlink_simstate_t, lat) }, \
         { "lng", NULL, MAVLINK_TYPE_INT32_T, 0, 40, offsetof(mavlink_simstate_t, lng) }, \
         } \
}
#endif

/**
 * @brief Pack a simstate message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param roll [rad] Roll angle.
 * @param pitch [rad] Pitch angle.
 * @param yaw [rad] Yaw angle.
 * @param xacc [m/s/s] X acceleration.
 * @param yacc [m/s/s] Y acceleration.
 * @param zacc [m/s/s] Z acceleration.
 * @param xgyro [rad/s] Angular speed around X axis.
 * @param ygyro [rad/s] Angular speed around Y axis.
 * @param zgyro [rad/s] Angular speed around Z axis.
 * @param lat [degE7] Latitude.
 * @param lng [degE7] Longitude.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_simstate_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, int32_t lat, int32_t lng)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SIMSTATE_LEN];
    _mav_put_float(buf, 0, roll);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);
    _mav_put_float(buf, 12, xacc);
    _mav_put_float(buf, 16, yacc);
    _mav_put_float(buf, 20, zacc);
    _mav_put_float(buf, 24, xgyro);
    _mav_put_float(buf, 28, ygyro);
    _mav_put_float(buf, 32, zgyro);
    _mav_put_int32_t(buf, 36, lat);
    _mav_put_int32_t(buf, 40, lng);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SIMSTATE_LEN);
#else
    mavlink_simstate_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.xacc = xacc;
    packet.yacc = yacc;
    packet.zacc = zacc;
    packet.xgyro = xgyro;
    packet.ygyro = ygyro;
    packet.zgyro = zgyro;
    packet.lat = lat;
    packet.lng = lng;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SIMSTATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SIMSTATE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SIMSTATE_MIN_LEN, MAVLINK_MSG_ID_SIMSTATE_LEN, MAVLINK_MSG_ID_SIMSTATE_CRC);
}

/**
 * @brief Pack a simstate message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param roll [rad] Roll angle.
 * @param pitch [rad] Pitch angle.
 * @param yaw [rad] Yaw angle.
 * @param xacc [m/s/s] X acceleration.
 * @param yacc [m/s/s] Y acceleration.
 * @param zacc [m/s/s] Z acceleration.
 * @param xgyro [rad/s] Angular speed around X axis.
 * @param ygyro [rad/s] Angular speed around Y axis.
 * @param zgyro [rad/s] Angular speed around Z axis.
 * @param lat [degE7] Latitude.
 * @param lng [degE7] Longitude.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_simstate_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float roll,float pitch,float yaw,float xacc,float yacc,float zacc,float xgyro,float ygyro,float zgyro,int32_t lat,int32_t lng)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SIMSTATE_LEN];
    _mav_put_float(buf, 0, roll);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);
    _mav_put_float(buf, 12, xacc);
    _mav_put_float(buf, 16, yacc);
    _mav_put_float(buf, 20, zacc);
    _mav_put_float(buf, 24, xgyro);
    _mav_put_float(buf, 28, ygyro);
    _mav_put_float(buf, 32, zgyro);
    _mav_put_int32_t(buf, 36, lat);
    _mav_put_int32_t(buf, 40, lng);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SIMSTATE_LEN);
#else
    mavlink_simstate_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.xacc = xacc;
    packet.yacc = yacc;
    packet.zacc = zacc;
    packet.xgyro = xgyro;
    packet.ygyro = ygyro;
    packet.zgyro = zgyro;
    packet.lat = lat;
    packet.lng = lng;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SIMSTATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SIMSTATE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SIMSTATE_MIN_LEN, MAVLINK_MSG_ID_SIMSTATE_LEN, MAVLINK_MSG_ID_SIMSTATE_CRC);
}

/**
 * @brief Encode a simstate struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param simstate C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_simstate_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_simstate_t* simstate)
{
    return mavlink_msg_simstate_pack(system_id, component_id, msg, simstate->roll, simstate->pitch, simstate->yaw, simstate->xacc, simstate->yacc, simstate->zacc, simstate->xgyro, simstate->ygyro, simstate->zgyro, simstate->lat, simstate->lng);
}

/**
 * @brief Encode a simstate struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param simstate C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_simstate_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_simstate_t* simstate)
{
    return mavlink_msg_simstate_pack_chan(system_id, component_id, chan, msg, simstate->roll, simstate->pitch, simstate->yaw, simstate->xacc, simstate->yacc, simstate->zacc, simstate->xgyro, simstate->ygyro, simstate->zgyro, simstate->lat, simstate->lng);
}

/**
 * @brief Send a simstate message
 * @param chan MAVLink channel to send the message
 *
 * @param roll [rad] Roll angle.
 * @param pitch [rad] Pitch angle.
 * @param yaw [rad] Yaw angle.
 * @param xacc [m/s/s] X acceleration.
 * @param yacc [m/s/s] Y acceleration.
 * @param zacc [m/s/s] Z acceleration.
 * @param xgyro [rad/s] Angular speed around X axis.
 * @param ygyro [rad/s] Angular speed around Y axis.
 * @param zgyro [rad/s] Angular speed around Z axis.
 * @param lat [degE7] Latitude.
 * @param lng [degE7] Longitude.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_simstate_send(mavlink_channel_t chan, float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, int32_t lat, int32_t lng)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SIMSTATE_LEN];
    _mav_put_float(buf, 0, roll);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);
    _mav_put_float(buf, 12, xacc);
    _mav_put_float(buf, 16, yacc);
    _mav_put_float(buf, 20, zacc);
    _mav_put_float(buf, 24, xgyro);
    _mav_put_float(buf, 28, ygyro);
    _mav_put_float(buf, 32, zgyro);
    _mav_put_int32_t(buf, 36, lat);
    _mav_put_int32_t(buf, 40, lng);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SIMSTATE, buf, MAVLINK_MSG_ID_SIMSTATE_MIN_LEN, MAVLINK_MSG_ID_SIMSTATE_LEN, MAVLINK_MSG_ID_SIMSTATE_CRC);
#else
    mavlink_simstate_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.xacc = xacc;
    packet.yacc = yacc;
    packet.zacc = zacc;
    packet.xgyro = xgyro;
    packet.ygyro = ygyro;
    packet.zgyro = zgyro;
    packet.lat = lat;
    packet.lng = lng;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SIMSTATE, (const char *)&packet, MAVLINK_MSG_ID_SIMSTATE_MIN_LEN, MAVLINK_MSG_ID_SIMSTATE_LEN, MAVLINK_MSG_ID_SIMSTATE_CRC);
#endif
}

/**
 * @brief Send a simstate message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_simstate_send_struct(mavlink_channel_t chan, const mavlink_simstate_t* simstate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_simstate_send(chan, simstate->roll, simstate->pitch, simstate->yaw, simstate->xacc, simstate->yacc, simstate->zacc, simstate->xgyro, simstate->ygyro, simstate->zgyro, simstate->lat, simstate->lng);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SIMSTATE, (const char *)simstate, MAVLINK_MSG_ID_SIMSTATE_MIN_LEN, MAVLINK_MSG_ID_SIMSTATE_LEN, MAVLINK_MSG_ID_SIMSTATE_CRC);
#endif
}

#if MAVLINK_MSG_ID_SIMSTATE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_simstate_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, int32_t lat, int32_t lng)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, roll);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);
    _mav_put_float(buf, 12, xacc);
    _mav_put_float(buf, 16, yacc);
    _mav_put_float(buf, 20, zacc);
    _mav_put_float(buf, 24, xgyro);
    _mav_put_float(buf, 28, ygyro);
    _mav_put_float(buf, 32, zgyro);
    _mav_put_int32_t(buf, 36, lat);
    _mav_put_int32_t(buf, 40, lng);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SIMSTATE, buf, MAVLINK_MSG_ID_SIMSTATE_MIN_LEN, MAVLINK_MSG_ID_SIMSTATE_LEN, MAVLINK_MSG_ID_SIMSTATE_CRC);
#else
    mavlink_simstate_t *packet = (mavlink_simstate_t *)msgbuf;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;
    packet->xacc = xacc;
    packet->yacc = yacc;
    packet->zacc = zacc;
    packet->xgyro = xgyro;
    packet->ygyro = ygyro;
    packet->zgyro = zgyro;
    packet->lat = lat;
    packet->lng = lng;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SIMSTATE, (const char *)packet, MAVLINK_MSG_ID_SIMSTATE_MIN_LEN, MAVLINK_MSG_ID_SIMSTATE_LEN, MAVLINK_MSG_ID_SIMSTATE_CRC);
#endif
}
#endif

#endif

// MESSAGE SIMSTATE UNPACKING


/**
 * @brief Get field roll from simstate message
 *
 * @return [rad] Roll angle.
 */
static inline float mavlink_msg_simstate_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pitch from simstate message
 *
 * @return [rad] Pitch angle.
 */
static inline float mavlink_msg_simstate_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field yaw from simstate message
 *
 * @return [rad] Yaw angle.
 */
static inline float mavlink_msg_simstate_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field xacc from simstate message
 *
 * @return [m/s/s] X acceleration.
 */
static inline float mavlink_msg_simstate_get_xacc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field yacc from simstate message
 *
 * @return [m/s/s] Y acceleration.
 */
static inline float mavlink_msg_simstate_get_yacc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field zacc from simstate message
 *
 * @return [m/s/s] Z acceleration.
 */
static inline float mavlink_msg_simstate_get_zacc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field xgyro from simstate message
 *
 * @return [rad/s] Angular speed around X axis.
 */
static inline float mavlink_msg_simstate_get_xgyro(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field ygyro from simstate message
 *
 * @return [rad/s] Angular speed around Y axis.
 */
static inline float mavlink_msg_simstate_get_ygyro(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field zgyro from simstate message
 *
 * @return [rad/s] Angular speed around Z axis.
 */
static inline float mavlink_msg_simstate_get_zgyro(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field lat from simstate message
 *
 * @return [degE7] Latitude.
 */
static inline int32_t mavlink_msg_simstate_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  36);
}

/**
 * @brief Get field lng from simstate message
 *
 * @return [degE7] Longitude.
 */
static inline int32_t mavlink_msg_simstate_get_lng(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  40);
}

/**
 * @brief Decode a simstate message into a struct
 *
 * @param msg The message to decode
 * @param simstate C-struct to decode the message contents into
 */
static inline void mavlink_msg_simstate_decode(const mavlink_message_t* msg, mavlink_simstate_t* simstate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    simstate->roll = mavlink_msg_simstate_get_roll(msg);
    simstate->pitch = mavlink_msg_simstate_get_pitch(msg);
    simstate->yaw = mavlink_msg_simstate_get_yaw(msg);
    simstate->xacc = mavlink_msg_simstate_get_xacc(msg);
    simstate->yacc = mavlink_msg_simstate_get_yacc(msg);
    simstate->zacc = mavlink_msg_simstate_get_zacc(msg);
    simstate->xgyro = mavlink_msg_simstate_get_xgyro(msg);
    simstate->ygyro = mavlink_msg_simstate_get_ygyro(msg);
    simstate->zgyro = mavlink_msg_simstate_get_zgyro(msg);
    simstate->lat = mavlink_msg_simstate_get_lat(msg);
    simstate->lng = mavlink_msg_simstate_get_lng(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SIMSTATE_LEN? msg->len : MAVLINK_MSG_ID_SIMSTATE_LEN;
        memset(simstate, 0, MAVLINK_MSG_ID_SIMSTATE_LEN);
    memcpy(simstate, _MAV_PAYLOAD(msg), len);
#endif
}
