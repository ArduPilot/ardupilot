#pragma once
// MESSAGE WIND_COV PACKING

#define MAVLINK_MSG_ID_WIND_COV 231

MAVPACKED(
typedef struct __mavlink_wind_cov_t {
 uint64_t time_usec; /*< Timestamp (micros since boot or Unix epoch)*/
 float wind_x; /*< Wind in X (NED) direction in m/s*/
 float wind_y; /*< Wind in Y (NED) direction in m/s*/
 float wind_z; /*< Wind in Z (NED) direction in m/s*/
 float var_horiz; /*< Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate.*/
 float var_vert; /*< Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate.*/
 float wind_alt; /*< AMSL altitude (m) this measurement was taken at*/
 float horiz_accuracy; /*< Horizontal speed 1-STD accuracy*/
 float vert_accuracy; /*< Vertical speed 1-STD accuracy*/
}) mavlink_wind_cov_t;

#define MAVLINK_MSG_ID_WIND_COV_LEN 40
#define MAVLINK_MSG_ID_WIND_COV_MIN_LEN 40
#define MAVLINK_MSG_ID_231_LEN 40
#define MAVLINK_MSG_ID_231_MIN_LEN 40

#define MAVLINK_MSG_ID_WIND_COV_CRC 105
#define MAVLINK_MSG_ID_231_CRC 105



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_WIND_COV { \
    231, \
    "WIND_COV", \
    9, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_wind_cov_t, time_usec) }, \
         { "wind_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_wind_cov_t, wind_x) }, \
         { "wind_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_wind_cov_t, wind_y) }, \
         { "wind_z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_wind_cov_t, wind_z) }, \
         { "var_horiz", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_wind_cov_t, var_horiz) }, \
         { "var_vert", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_wind_cov_t, var_vert) }, \
         { "wind_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_wind_cov_t, wind_alt) }, \
         { "horiz_accuracy", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_wind_cov_t, horiz_accuracy) }, \
         { "vert_accuracy", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_wind_cov_t, vert_accuracy) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_WIND_COV { \
    "WIND_COV", \
    9, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_wind_cov_t, time_usec) }, \
         { "wind_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_wind_cov_t, wind_x) }, \
         { "wind_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_wind_cov_t, wind_y) }, \
         { "wind_z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_wind_cov_t, wind_z) }, \
         { "var_horiz", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_wind_cov_t, var_horiz) }, \
         { "var_vert", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_wind_cov_t, var_vert) }, \
         { "wind_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_wind_cov_t, wind_alt) }, \
         { "horiz_accuracy", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_wind_cov_t, horiz_accuracy) }, \
         { "vert_accuracy", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_wind_cov_t, vert_accuracy) }, \
         } \
}
#endif

/**
 * @brief Pack a wind_cov message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param wind_x Wind in X (NED) direction in m/s
 * @param wind_y Wind in Y (NED) direction in m/s
 * @param wind_z Wind in Z (NED) direction in m/s
 * @param var_horiz Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate.
 * @param var_vert Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate.
 * @param wind_alt AMSL altitude (m) this measurement was taken at
 * @param horiz_accuracy Horizontal speed 1-STD accuracy
 * @param vert_accuracy Vertical speed 1-STD accuracy
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_wind_cov_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, float wind_x, float wind_y, float wind_z, float var_horiz, float var_vert, float wind_alt, float horiz_accuracy, float vert_accuracy)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WIND_COV_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, wind_x);
    _mav_put_float(buf, 12, wind_y);
    _mav_put_float(buf, 16, wind_z);
    _mav_put_float(buf, 20, var_horiz);
    _mav_put_float(buf, 24, var_vert);
    _mav_put_float(buf, 28, wind_alt);
    _mav_put_float(buf, 32, horiz_accuracy);
    _mav_put_float(buf, 36, vert_accuracy);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WIND_COV_LEN);
#else
    mavlink_wind_cov_t packet;
    packet.time_usec = time_usec;
    packet.wind_x = wind_x;
    packet.wind_y = wind_y;
    packet.wind_z = wind_z;
    packet.var_horiz = var_horiz;
    packet.var_vert = var_vert;
    packet.wind_alt = wind_alt;
    packet.horiz_accuracy = horiz_accuracy;
    packet.vert_accuracy = vert_accuracy;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WIND_COV_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WIND_COV;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_WIND_COV_MIN_LEN, MAVLINK_MSG_ID_WIND_COV_LEN, MAVLINK_MSG_ID_WIND_COV_CRC);
}

/**
 * @brief Pack a wind_cov message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param wind_x Wind in X (NED) direction in m/s
 * @param wind_y Wind in Y (NED) direction in m/s
 * @param wind_z Wind in Z (NED) direction in m/s
 * @param var_horiz Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate.
 * @param var_vert Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate.
 * @param wind_alt AMSL altitude (m) this measurement was taken at
 * @param horiz_accuracy Horizontal speed 1-STD accuracy
 * @param vert_accuracy Vertical speed 1-STD accuracy
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_wind_cov_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,float wind_x,float wind_y,float wind_z,float var_horiz,float var_vert,float wind_alt,float horiz_accuracy,float vert_accuracy)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WIND_COV_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, wind_x);
    _mav_put_float(buf, 12, wind_y);
    _mav_put_float(buf, 16, wind_z);
    _mav_put_float(buf, 20, var_horiz);
    _mav_put_float(buf, 24, var_vert);
    _mav_put_float(buf, 28, wind_alt);
    _mav_put_float(buf, 32, horiz_accuracy);
    _mav_put_float(buf, 36, vert_accuracy);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WIND_COV_LEN);
#else
    mavlink_wind_cov_t packet;
    packet.time_usec = time_usec;
    packet.wind_x = wind_x;
    packet.wind_y = wind_y;
    packet.wind_z = wind_z;
    packet.var_horiz = var_horiz;
    packet.var_vert = var_vert;
    packet.wind_alt = wind_alt;
    packet.horiz_accuracy = horiz_accuracy;
    packet.vert_accuracy = vert_accuracy;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WIND_COV_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WIND_COV;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_WIND_COV_MIN_LEN, MAVLINK_MSG_ID_WIND_COV_LEN, MAVLINK_MSG_ID_WIND_COV_CRC);
}

/**
 * @brief Encode a wind_cov struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param wind_cov C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_wind_cov_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_wind_cov_t* wind_cov)
{
    return mavlink_msg_wind_cov_pack(system_id, component_id, msg, wind_cov->time_usec, wind_cov->wind_x, wind_cov->wind_y, wind_cov->wind_z, wind_cov->var_horiz, wind_cov->var_vert, wind_cov->wind_alt, wind_cov->horiz_accuracy, wind_cov->vert_accuracy);
}

/**
 * @brief Encode a wind_cov struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param wind_cov C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_wind_cov_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_wind_cov_t* wind_cov)
{
    return mavlink_msg_wind_cov_pack_chan(system_id, component_id, chan, msg, wind_cov->time_usec, wind_cov->wind_x, wind_cov->wind_y, wind_cov->wind_z, wind_cov->var_horiz, wind_cov->var_vert, wind_cov->wind_alt, wind_cov->horiz_accuracy, wind_cov->vert_accuracy);
}

/**
 * @brief Send a wind_cov message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param wind_x Wind in X (NED) direction in m/s
 * @param wind_y Wind in Y (NED) direction in m/s
 * @param wind_z Wind in Z (NED) direction in m/s
 * @param var_horiz Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate.
 * @param var_vert Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate.
 * @param wind_alt AMSL altitude (m) this measurement was taken at
 * @param horiz_accuracy Horizontal speed 1-STD accuracy
 * @param vert_accuracy Vertical speed 1-STD accuracy
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_wind_cov_send(mavlink_channel_t chan, uint64_t time_usec, float wind_x, float wind_y, float wind_z, float var_horiz, float var_vert, float wind_alt, float horiz_accuracy, float vert_accuracy)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WIND_COV_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, wind_x);
    _mav_put_float(buf, 12, wind_y);
    _mav_put_float(buf, 16, wind_z);
    _mav_put_float(buf, 20, var_horiz);
    _mav_put_float(buf, 24, var_vert);
    _mav_put_float(buf, 28, wind_alt);
    _mav_put_float(buf, 32, horiz_accuracy);
    _mav_put_float(buf, 36, vert_accuracy);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WIND_COV, buf, MAVLINK_MSG_ID_WIND_COV_MIN_LEN, MAVLINK_MSG_ID_WIND_COV_LEN, MAVLINK_MSG_ID_WIND_COV_CRC);
#else
    mavlink_wind_cov_t packet;
    packet.time_usec = time_usec;
    packet.wind_x = wind_x;
    packet.wind_y = wind_y;
    packet.wind_z = wind_z;
    packet.var_horiz = var_horiz;
    packet.var_vert = var_vert;
    packet.wind_alt = wind_alt;
    packet.horiz_accuracy = horiz_accuracy;
    packet.vert_accuracy = vert_accuracy;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WIND_COV, (const char *)&packet, MAVLINK_MSG_ID_WIND_COV_MIN_LEN, MAVLINK_MSG_ID_WIND_COV_LEN, MAVLINK_MSG_ID_WIND_COV_CRC);
#endif
}

/**
 * @brief Send a wind_cov message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_wind_cov_send_struct(mavlink_channel_t chan, const mavlink_wind_cov_t* wind_cov)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_wind_cov_send(chan, wind_cov->time_usec, wind_cov->wind_x, wind_cov->wind_y, wind_cov->wind_z, wind_cov->var_horiz, wind_cov->var_vert, wind_cov->wind_alt, wind_cov->horiz_accuracy, wind_cov->vert_accuracy);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WIND_COV, (const char *)wind_cov, MAVLINK_MSG_ID_WIND_COV_MIN_LEN, MAVLINK_MSG_ID_WIND_COV_LEN, MAVLINK_MSG_ID_WIND_COV_CRC);
#endif
}

#if MAVLINK_MSG_ID_WIND_COV_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_wind_cov_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, float wind_x, float wind_y, float wind_z, float var_horiz, float var_vert, float wind_alt, float horiz_accuracy, float vert_accuracy)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, wind_x);
    _mav_put_float(buf, 12, wind_y);
    _mav_put_float(buf, 16, wind_z);
    _mav_put_float(buf, 20, var_horiz);
    _mav_put_float(buf, 24, var_vert);
    _mav_put_float(buf, 28, wind_alt);
    _mav_put_float(buf, 32, horiz_accuracy);
    _mav_put_float(buf, 36, vert_accuracy);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WIND_COV, buf, MAVLINK_MSG_ID_WIND_COV_MIN_LEN, MAVLINK_MSG_ID_WIND_COV_LEN, MAVLINK_MSG_ID_WIND_COV_CRC);
#else
    mavlink_wind_cov_t *packet = (mavlink_wind_cov_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->wind_x = wind_x;
    packet->wind_y = wind_y;
    packet->wind_z = wind_z;
    packet->var_horiz = var_horiz;
    packet->var_vert = var_vert;
    packet->wind_alt = wind_alt;
    packet->horiz_accuracy = horiz_accuracy;
    packet->vert_accuracy = vert_accuracy;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WIND_COV, (const char *)packet, MAVLINK_MSG_ID_WIND_COV_MIN_LEN, MAVLINK_MSG_ID_WIND_COV_LEN, MAVLINK_MSG_ID_WIND_COV_CRC);
#endif
}
#endif

#endif

// MESSAGE WIND_COV UNPACKING


/**
 * @brief Get field time_usec from wind_cov message
 *
 * @return Timestamp (micros since boot or Unix epoch)
 */
static inline uint64_t mavlink_msg_wind_cov_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field wind_x from wind_cov message
 *
 * @return Wind in X (NED) direction in m/s
 */
static inline float mavlink_msg_wind_cov_get_wind_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field wind_y from wind_cov message
 *
 * @return Wind in Y (NED) direction in m/s
 */
static inline float mavlink_msg_wind_cov_get_wind_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field wind_z from wind_cov message
 *
 * @return Wind in Z (NED) direction in m/s
 */
static inline float mavlink_msg_wind_cov_get_wind_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field var_horiz from wind_cov message
 *
 * @return Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate.
 */
static inline float mavlink_msg_wind_cov_get_var_horiz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field var_vert from wind_cov message
 *
 * @return Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate.
 */
static inline float mavlink_msg_wind_cov_get_var_vert(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field wind_alt from wind_cov message
 *
 * @return AMSL altitude (m) this measurement was taken at
 */
static inline float mavlink_msg_wind_cov_get_wind_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field horiz_accuracy from wind_cov message
 *
 * @return Horizontal speed 1-STD accuracy
 */
static inline float mavlink_msg_wind_cov_get_horiz_accuracy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field vert_accuracy from wind_cov message
 *
 * @return Vertical speed 1-STD accuracy
 */
static inline float mavlink_msg_wind_cov_get_vert_accuracy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Decode a wind_cov message into a struct
 *
 * @param msg The message to decode
 * @param wind_cov C-struct to decode the message contents into
 */
static inline void mavlink_msg_wind_cov_decode(const mavlink_message_t* msg, mavlink_wind_cov_t* wind_cov)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    wind_cov->time_usec = mavlink_msg_wind_cov_get_time_usec(msg);
    wind_cov->wind_x = mavlink_msg_wind_cov_get_wind_x(msg);
    wind_cov->wind_y = mavlink_msg_wind_cov_get_wind_y(msg);
    wind_cov->wind_z = mavlink_msg_wind_cov_get_wind_z(msg);
    wind_cov->var_horiz = mavlink_msg_wind_cov_get_var_horiz(msg);
    wind_cov->var_vert = mavlink_msg_wind_cov_get_var_vert(msg);
    wind_cov->wind_alt = mavlink_msg_wind_cov_get_wind_alt(msg);
    wind_cov->horiz_accuracy = mavlink_msg_wind_cov_get_horiz_accuracy(msg);
    wind_cov->vert_accuracy = mavlink_msg_wind_cov_get_vert_accuracy(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_WIND_COV_LEN? msg->len : MAVLINK_MSG_ID_WIND_COV_LEN;
        memset(wind_cov, 0, MAVLINK_MSG_ID_WIND_COV_LEN);
    memcpy(wind_cov, _MAV_PAYLOAD(msg), len);
#endif
}
