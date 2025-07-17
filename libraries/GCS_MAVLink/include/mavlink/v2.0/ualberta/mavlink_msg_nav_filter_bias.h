#pragma once
// MESSAGE NAV_FILTER_BIAS PACKING

#define MAVLINK_MSG_ID_NAV_FILTER_BIAS 220


typedef struct __mavlink_nav_filter_bias_t {
 uint64_t usec; /*<  Timestamp (microseconds)*/
 float accel_0; /*<  b_f[0]*/
 float accel_1; /*<  b_f[1]*/
 float accel_2; /*<  b_f[2]*/
 float gyro_0; /*<  b_f[0]*/
 float gyro_1; /*<  b_f[1]*/
 float gyro_2; /*<  b_f[2]*/
} mavlink_nav_filter_bias_t;

#define MAVLINK_MSG_ID_NAV_FILTER_BIAS_LEN 32
#define MAVLINK_MSG_ID_NAV_FILTER_BIAS_MIN_LEN 32
#define MAVLINK_MSG_ID_220_LEN 32
#define MAVLINK_MSG_ID_220_MIN_LEN 32

#define MAVLINK_MSG_ID_NAV_FILTER_BIAS_CRC 34
#define MAVLINK_MSG_ID_220_CRC 34



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_NAV_FILTER_BIAS { \
    220, \
    "NAV_FILTER_BIAS", \
    7, \
    {  { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_nav_filter_bias_t, usec) }, \
         { "accel_0", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_nav_filter_bias_t, accel_0) }, \
         { "accel_1", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_nav_filter_bias_t, accel_1) }, \
         { "accel_2", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_nav_filter_bias_t, accel_2) }, \
         { "gyro_0", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_nav_filter_bias_t, gyro_0) }, \
         { "gyro_1", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_nav_filter_bias_t, gyro_1) }, \
         { "gyro_2", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_nav_filter_bias_t, gyro_2) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_NAV_FILTER_BIAS { \
    "NAV_FILTER_BIAS", \
    7, \
    {  { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_nav_filter_bias_t, usec) }, \
         { "accel_0", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_nav_filter_bias_t, accel_0) }, \
         { "accel_1", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_nav_filter_bias_t, accel_1) }, \
         { "accel_2", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_nav_filter_bias_t, accel_2) }, \
         { "gyro_0", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_nav_filter_bias_t, gyro_0) }, \
         { "gyro_1", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_nav_filter_bias_t, gyro_1) }, \
         { "gyro_2", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_nav_filter_bias_t, gyro_2) }, \
         } \
}
#endif

/**
 * @brief Pack a nav_filter_bias message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec  Timestamp (microseconds)
 * @param accel_0  b_f[0]
 * @param accel_1  b_f[1]
 * @param accel_2  b_f[2]
 * @param gyro_0  b_f[0]
 * @param gyro_1  b_f[1]
 * @param gyro_2  b_f[2]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_nav_filter_bias_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t usec, float accel_0, float accel_1, float accel_2, float gyro_0, float gyro_1, float gyro_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAV_FILTER_BIAS_LEN];
    _mav_put_uint64_t(buf, 0, usec);
    _mav_put_float(buf, 8, accel_0);
    _mav_put_float(buf, 12, accel_1);
    _mav_put_float(buf, 16, accel_2);
    _mav_put_float(buf, 20, gyro_0);
    _mav_put_float(buf, 24, gyro_1);
    _mav_put_float(buf, 28, gyro_2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAV_FILTER_BIAS_LEN);
#else
    mavlink_nav_filter_bias_t packet;
    packet.usec = usec;
    packet.accel_0 = accel_0;
    packet.accel_1 = accel_1;
    packet.accel_2 = accel_2;
    packet.gyro_0 = gyro_0;
    packet.gyro_1 = gyro_1;
    packet.gyro_2 = gyro_2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAV_FILTER_BIAS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAV_FILTER_BIAS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NAV_FILTER_BIAS_MIN_LEN, MAVLINK_MSG_ID_NAV_FILTER_BIAS_LEN, MAVLINK_MSG_ID_NAV_FILTER_BIAS_CRC);
}

/**
 * @brief Pack a nav_filter_bias message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec  Timestamp (microseconds)
 * @param accel_0  b_f[0]
 * @param accel_1  b_f[1]
 * @param accel_2  b_f[2]
 * @param gyro_0  b_f[0]
 * @param gyro_1  b_f[1]
 * @param gyro_2  b_f[2]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_nav_filter_bias_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t usec, float accel_0, float accel_1, float accel_2, float gyro_0, float gyro_1, float gyro_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAV_FILTER_BIAS_LEN];
    _mav_put_uint64_t(buf, 0, usec);
    _mav_put_float(buf, 8, accel_0);
    _mav_put_float(buf, 12, accel_1);
    _mav_put_float(buf, 16, accel_2);
    _mav_put_float(buf, 20, gyro_0);
    _mav_put_float(buf, 24, gyro_1);
    _mav_put_float(buf, 28, gyro_2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAV_FILTER_BIAS_LEN);
#else
    mavlink_nav_filter_bias_t packet;
    packet.usec = usec;
    packet.accel_0 = accel_0;
    packet.accel_1 = accel_1;
    packet.accel_2 = accel_2;
    packet.gyro_0 = gyro_0;
    packet.gyro_1 = gyro_1;
    packet.gyro_2 = gyro_2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAV_FILTER_BIAS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAV_FILTER_BIAS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_NAV_FILTER_BIAS_MIN_LEN, MAVLINK_MSG_ID_NAV_FILTER_BIAS_LEN, MAVLINK_MSG_ID_NAV_FILTER_BIAS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_NAV_FILTER_BIAS_MIN_LEN, MAVLINK_MSG_ID_NAV_FILTER_BIAS_LEN);
#endif
}

/**
 * @brief Pack a nav_filter_bias message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec  Timestamp (microseconds)
 * @param accel_0  b_f[0]
 * @param accel_1  b_f[1]
 * @param accel_2  b_f[2]
 * @param gyro_0  b_f[0]
 * @param gyro_1  b_f[1]
 * @param gyro_2  b_f[2]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_nav_filter_bias_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t usec,float accel_0,float accel_1,float accel_2,float gyro_0,float gyro_1,float gyro_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAV_FILTER_BIAS_LEN];
    _mav_put_uint64_t(buf, 0, usec);
    _mav_put_float(buf, 8, accel_0);
    _mav_put_float(buf, 12, accel_1);
    _mav_put_float(buf, 16, accel_2);
    _mav_put_float(buf, 20, gyro_0);
    _mav_put_float(buf, 24, gyro_1);
    _mav_put_float(buf, 28, gyro_2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAV_FILTER_BIAS_LEN);
#else
    mavlink_nav_filter_bias_t packet;
    packet.usec = usec;
    packet.accel_0 = accel_0;
    packet.accel_1 = accel_1;
    packet.accel_2 = accel_2;
    packet.gyro_0 = gyro_0;
    packet.gyro_1 = gyro_1;
    packet.gyro_2 = gyro_2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAV_FILTER_BIAS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAV_FILTER_BIAS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NAV_FILTER_BIAS_MIN_LEN, MAVLINK_MSG_ID_NAV_FILTER_BIAS_LEN, MAVLINK_MSG_ID_NAV_FILTER_BIAS_CRC);
}

/**
 * @brief Encode a nav_filter_bias struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param nav_filter_bias C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_nav_filter_bias_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_nav_filter_bias_t* nav_filter_bias)
{
    return mavlink_msg_nav_filter_bias_pack(system_id, component_id, msg, nav_filter_bias->usec, nav_filter_bias->accel_0, nav_filter_bias->accel_1, nav_filter_bias->accel_2, nav_filter_bias->gyro_0, nav_filter_bias->gyro_1, nav_filter_bias->gyro_2);
}

/**
 * @brief Encode a nav_filter_bias struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param nav_filter_bias C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_nav_filter_bias_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_nav_filter_bias_t* nav_filter_bias)
{
    return mavlink_msg_nav_filter_bias_pack_chan(system_id, component_id, chan, msg, nav_filter_bias->usec, nav_filter_bias->accel_0, nav_filter_bias->accel_1, nav_filter_bias->accel_2, nav_filter_bias->gyro_0, nav_filter_bias->gyro_1, nav_filter_bias->gyro_2);
}

/**
 * @brief Encode a nav_filter_bias struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param nav_filter_bias C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_nav_filter_bias_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_nav_filter_bias_t* nav_filter_bias)
{
    return mavlink_msg_nav_filter_bias_pack_status(system_id, component_id, _status, msg,  nav_filter_bias->usec, nav_filter_bias->accel_0, nav_filter_bias->accel_1, nav_filter_bias->accel_2, nav_filter_bias->gyro_0, nav_filter_bias->gyro_1, nav_filter_bias->gyro_2);
}

/**
 * @brief Send a nav_filter_bias message
 * @param chan MAVLink channel to send the message
 *
 * @param usec  Timestamp (microseconds)
 * @param accel_0  b_f[0]
 * @param accel_1  b_f[1]
 * @param accel_2  b_f[2]
 * @param gyro_0  b_f[0]
 * @param gyro_1  b_f[1]
 * @param gyro_2  b_f[2]
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_nav_filter_bias_send(mavlink_channel_t chan, uint64_t usec, float accel_0, float accel_1, float accel_2, float gyro_0, float gyro_1, float gyro_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAV_FILTER_BIAS_LEN];
    _mav_put_uint64_t(buf, 0, usec);
    _mav_put_float(buf, 8, accel_0);
    _mav_put_float(buf, 12, accel_1);
    _mav_put_float(buf, 16, accel_2);
    _mav_put_float(buf, 20, gyro_0);
    _mav_put_float(buf, 24, gyro_1);
    _mav_put_float(buf, 28, gyro_2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAV_FILTER_BIAS, buf, MAVLINK_MSG_ID_NAV_FILTER_BIAS_MIN_LEN, MAVLINK_MSG_ID_NAV_FILTER_BIAS_LEN, MAVLINK_MSG_ID_NAV_FILTER_BIAS_CRC);
#else
    mavlink_nav_filter_bias_t packet;
    packet.usec = usec;
    packet.accel_0 = accel_0;
    packet.accel_1 = accel_1;
    packet.accel_2 = accel_2;
    packet.gyro_0 = gyro_0;
    packet.gyro_1 = gyro_1;
    packet.gyro_2 = gyro_2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAV_FILTER_BIAS, (const char *)&packet, MAVLINK_MSG_ID_NAV_FILTER_BIAS_MIN_LEN, MAVLINK_MSG_ID_NAV_FILTER_BIAS_LEN, MAVLINK_MSG_ID_NAV_FILTER_BIAS_CRC);
#endif
}

/**
 * @brief Send a nav_filter_bias message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_nav_filter_bias_send_struct(mavlink_channel_t chan, const mavlink_nav_filter_bias_t* nav_filter_bias)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_nav_filter_bias_send(chan, nav_filter_bias->usec, nav_filter_bias->accel_0, nav_filter_bias->accel_1, nav_filter_bias->accel_2, nav_filter_bias->gyro_0, nav_filter_bias->gyro_1, nav_filter_bias->gyro_2);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAV_FILTER_BIAS, (const char *)nav_filter_bias, MAVLINK_MSG_ID_NAV_FILTER_BIAS_MIN_LEN, MAVLINK_MSG_ID_NAV_FILTER_BIAS_LEN, MAVLINK_MSG_ID_NAV_FILTER_BIAS_CRC);
#endif
}

#if MAVLINK_MSG_ID_NAV_FILTER_BIAS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_nav_filter_bias_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t usec, float accel_0, float accel_1, float accel_2, float gyro_0, float gyro_1, float gyro_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, usec);
    _mav_put_float(buf, 8, accel_0);
    _mav_put_float(buf, 12, accel_1);
    _mav_put_float(buf, 16, accel_2);
    _mav_put_float(buf, 20, gyro_0);
    _mav_put_float(buf, 24, gyro_1);
    _mav_put_float(buf, 28, gyro_2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAV_FILTER_BIAS, buf, MAVLINK_MSG_ID_NAV_FILTER_BIAS_MIN_LEN, MAVLINK_MSG_ID_NAV_FILTER_BIAS_LEN, MAVLINK_MSG_ID_NAV_FILTER_BIAS_CRC);
#else
    mavlink_nav_filter_bias_t *packet = (mavlink_nav_filter_bias_t *)msgbuf;
    packet->usec = usec;
    packet->accel_0 = accel_0;
    packet->accel_1 = accel_1;
    packet->accel_2 = accel_2;
    packet->gyro_0 = gyro_0;
    packet->gyro_1 = gyro_1;
    packet->gyro_2 = gyro_2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAV_FILTER_BIAS, (const char *)packet, MAVLINK_MSG_ID_NAV_FILTER_BIAS_MIN_LEN, MAVLINK_MSG_ID_NAV_FILTER_BIAS_LEN, MAVLINK_MSG_ID_NAV_FILTER_BIAS_CRC);
#endif
}
#endif

#endif

// MESSAGE NAV_FILTER_BIAS UNPACKING


/**
 * @brief Get field usec from nav_filter_bias message
 *
 * @return  Timestamp (microseconds)
 */
static inline uint64_t mavlink_msg_nav_filter_bias_get_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field accel_0 from nav_filter_bias message
 *
 * @return  b_f[0]
 */
static inline float mavlink_msg_nav_filter_bias_get_accel_0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field accel_1 from nav_filter_bias message
 *
 * @return  b_f[1]
 */
static inline float mavlink_msg_nav_filter_bias_get_accel_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field accel_2 from nav_filter_bias message
 *
 * @return  b_f[2]
 */
static inline float mavlink_msg_nav_filter_bias_get_accel_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field gyro_0 from nav_filter_bias message
 *
 * @return  b_f[0]
 */
static inline float mavlink_msg_nav_filter_bias_get_gyro_0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field gyro_1 from nav_filter_bias message
 *
 * @return  b_f[1]
 */
static inline float mavlink_msg_nav_filter_bias_get_gyro_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field gyro_2 from nav_filter_bias message
 *
 * @return  b_f[2]
 */
static inline float mavlink_msg_nav_filter_bias_get_gyro_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a nav_filter_bias message into a struct
 *
 * @param msg The message to decode
 * @param nav_filter_bias C-struct to decode the message contents into
 */
static inline void mavlink_msg_nav_filter_bias_decode(const mavlink_message_t* msg, mavlink_nav_filter_bias_t* nav_filter_bias)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    nav_filter_bias->usec = mavlink_msg_nav_filter_bias_get_usec(msg);
    nav_filter_bias->accel_0 = mavlink_msg_nav_filter_bias_get_accel_0(msg);
    nav_filter_bias->accel_1 = mavlink_msg_nav_filter_bias_get_accel_1(msg);
    nav_filter_bias->accel_2 = mavlink_msg_nav_filter_bias_get_accel_2(msg);
    nav_filter_bias->gyro_0 = mavlink_msg_nav_filter_bias_get_gyro_0(msg);
    nav_filter_bias->gyro_1 = mavlink_msg_nav_filter_bias_get_gyro_1(msg);
    nav_filter_bias->gyro_2 = mavlink_msg_nav_filter_bias_get_gyro_2(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_NAV_FILTER_BIAS_LEN? msg->len : MAVLINK_MSG_ID_NAV_FILTER_BIAS_LEN;
        memset(nav_filter_bias, 0, MAVLINK_MSG_ID_NAV_FILTER_BIAS_LEN);
    memcpy(nav_filter_bias, _MAV_PAYLOAD(msg), len);
#endif
}
