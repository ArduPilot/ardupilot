#pragma once
// MESSAGE ATT_POS_MOCAP PACKING

#define MAVLINK_MSG_ID_ATT_POS_MOCAP 138


typedef struct __mavlink_att_pos_mocap_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 float q[4]; /*<  Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)*/
 float x; /*< [m] X position (NED)*/
 float y; /*< [m] Y position (NED)*/
 float z; /*< [m] Z position (NED)*/
 float covariance[21]; /*<  Row-major representation of a pose 6x6 cross-covariance matrix upper right triangle (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.*/
} mavlink_att_pos_mocap_t;

#define MAVLINK_MSG_ID_ATT_POS_MOCAP_LEN 120
#define MAVLINK_MSG_ID_ATT_POS_MOCAP_MIN_LEN 36
#define MAVLINK_MSG_ID_138_LEN 120
#define MAVLINK_MSG_ID_138_MIN_LEN 36

#define MAVLINK_MSG_ID_ATT_POS_MOCAP_CRC 109
#define MAVLINK_MSG_ID_138_CRC 109

#define MAVLINK_MSG_ATT_POS_MOCAP_FIELD_Q_LEN 4
#define MAVLINK_MSG_ATT_POS_MOCAP_FIELD_COVARIANCE_LEN 21

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ATT_POS_MOCAP { \
    138, \
    "ATT_POS_MOCAP", \
    6, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_att_pos_mocap_t, time_usec) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 4, 8, offsetof(mavlink_att_pos_mocap_t, q) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_att_pos_mocap_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_att_pos_mocap_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_att_pos_mocap_t, z) }, \
         { "covariance", NULL, MAVLINK_TYPE_FLOAT, 21, 36, offsetof(mavlink_att_pos_mocap_t, covariance) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ATT_POS_MOCAP { \
    "ATT_POS_MOCAP", \
    6, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_att_pos_mocap_t, time_usec) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 4, 8, offsetof(mavlink_att_pos_mocap_t, q) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_att_pos_mocap_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_att_pos_mocap_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_att_pos_mocap_t, z) }, \
         { "covariance", NULL, MAVLINK_TYPE_FLOAT, 21, 36, offsetof(mavlink_att_pos_mocap_t, covariance) }, \
         } \
}
#endif

/**
 * @brief Pack a att_pos_mocap message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param q  Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
 * @param x [m] X position (NED)
 * @param y [m] Y position (NED)
 * @param z [m] Z position (NED)
 * @param covariance  Row-major representation of a pose 6x6 cross-covariance matrix upper right triangle (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_att_pos_mocap_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, const float *q, float x, float y, float z, const float *covariance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATT_POS_MOCAP_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 24, x);
    _mav_put_float(buf, 28, y);
    _mav_put_float(buf, 32, z);
    _mav_put_float_array(buf, 8, q, 4);
    _mav_put_float_array(buf, 36, covariance, 21);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATT_POS_MOCAP_LEN);
#else
    mavlink_att_pos_mocap_t packet;
    packet.time_usec = time_usec;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
    mav_array_memcpy(packet.covariance, covariance, sizeof(float)*21);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATT_POS_MOCAP_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ATT_POS_MOCAP;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ATT_POS_MOCAP_MIN_LEN, MAVLINK_MSG_ID_ATT_POS_MOCAP_LEN, MAVLINK_MSG_ID_ATT_POS_MOCAP_CRC);
}

/**
 * @brief Pack a att_pos_mocap message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param q  Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
 * @param x [m] X position (NED)
 * @param y [m] Y position (NED)
 * @param z [m] Z position (NED)
 * @param covariance  Row-major representation of a pose 6x6 cross-covariance matrix upper right triangle (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_att_pos_mocap_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,const float *q,float x,float y,float z,const float *covariance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATT_POS_MOCAP_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 24, x);
    _mav_put_float(buf, 28, y);
    _mav_put_float(buf, 32, z);
    _mav_put_float_array(buf, 8, q, 4);
    _mav_put_float_array(buf, 36, covariance, 21);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATT_POS_MOCAP_LEN);
#else
    mavlink_att_pos_mocap_t packet;
    packet.time_usec = time_usec;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
    mav_array_memcpy(packet.covariance, covariance, sizeof(float)*21);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATT_POS_MOCAP_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ATT_POS_MOCAP;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ATT_POS_MOCAP_MIN_LEN, MAVLINK_MSG_ID_ATT_POS_MOCAP_LEN, MAVLINK_MSG_ID_ATT_POS_MOCAP_CRC);
}

/**
 * @brief Encode a att_pos_mocap struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param att_pos_mocap C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_att_pos_mocap_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_att_pos_mocap_t* att_pos_mocap)
{
    return mavlink_msg_att_pos_mocap_pack(system_id, component_id, msg, att_pos_mocap->time_usec, att_pos_mocap->q, att_pos_mocap->x, att_pos_mocap->y, att_pos_mocap->z, att_pos_mocap->covariance);
}

/**
 * @brief Encode a att_pos_mocap struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param att_pos_mocap C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_att_pos_mocap_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_att_pos_mocap_t* att_pos_mocap)
{
    return mavlink_msg_att_pos_mocap_pack_chan(system_id, component_id, chan, msg, att_pos_mocap->time_usec, att_pos_mocap->q, att_pos_mocap->x, att_pos_mocap->y, att_pos_mocap->z, att_pos_mocap->covariance);
}

/**
 * @brief Send a att_pos_mocap message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param q  Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
 * @param x [m] X position (NED)
 * @param y [m] Y position (NED)
 * @param z [m] Z position (NED)
 * @param covariance  Row-major representation of a pose 6x6 cross-covariance matrix upper right triangle (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_att_pos_mocap_send(mavlink_channel_t chan, uint64_t time_usec, const float *q, float x, float y, float z, const float *covariance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATT_POS_MOCAP_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 24, x);
    _mav_put_float(buf, 28, y);
    _mav_put_float(buf, 32, z);
    _mav_put_float_array(buf, 8, q, 4);
    _mav_put_float_array(buf, 36, covariance, 21);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATT_POS_MOCAP, buf, MAVLINK_MSG_ID_ATT_POS_MOCAP_MIN_LEN, MAVLINK_MSG_ID_ATT_POS_MOCAP_LEN, MAVLINK_MSG_ID_ATT_POS_MOCAP_CRC);
#else
    mavlink_att_pos_mocap_t packet;
    packet.time_usec = time_usec;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
    mav_array_memcpy(packet.covariance, covariance, sizeof(float)*21);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATT_POS_MOCAP, (const char *)&packet, MAVLINK_MSG_ID_ATT_POS_MOCAP_MIN_LEN, MAVLINK_MSG_ID_ATT_POS_MOCAP_LEN, MAVLINK_MSG_ID_ATT_POS_MOCAP_CRC);
#endif
}

/**
 * @brief Send a att_pos_mocap message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_att_pos_mocap_send_struct(mavlink_channel_t chan, const mavlink_att_pos_mocap_t* att_pos_mocap)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_att_pos_mocap_send(chan, att_pos_mocap->time_usec, att_pos_mocap->q, att_pos_mocap->x, att_pos_mocap->y, att_pos_mocap->z, att_pos_mocap->covariance);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATT_POS_MOCAP, (const char *)att_pos_mocap, MAVLINK_MSG_ID_ATT_POS_MOCAP_MIN_LEN, MAVLINK_MSG_ID_ATT_POS_MOCAP_LEN, MAVLINK_MSG_ID_ATT_POS_MOCAP_CRC);
#endif
}

#if MAVLINK_MSG_ID_ATT_POS_MOCAP_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_att_pos_mocap_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, const float *q, float x, float y, float z, const float *covariance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 24, x);
    _mav_put_float(buf, 28, y);
    _mav_put_float(buf, 32, z);
    _mav_put_float_array(buf, 8, q, 4);
    _mav_put_float_array(buf, 36, covariance, 21);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATT_POS_MOCAP, buf, MAVLINK_MSG_ID_ATT_POS_MOCAP_MIN_LEN, MAVLINK_MSG_ID_ATT_POS_MOCAP_LEN, MAVLINK_MSG_ID_ATT_POS_MOCAP_CRC);
#else
    mavlink_att_pos_mocap_t *packet = (mavlink_att_pos_mocap_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    mav_array_memcpy(packet->q, q, sizeof(float)*4);
    mav_array_memcpy(packet->covariance, covariance, sizeof(float)*21);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATT_POS_MOCAP, (const char *)packet, MAVLINK_MSG_ID_ATT_POS_MOCAP_MIN_LEN, MAVLINK_MSG_ID_ATT_POS_MOCAP_LEN, MAVLINK_MSG_ID_ATT_POS_MOCAP_CRC);
#endif
}
#endif

#endif

// MESSAGE ATT_POS_MOCAP UNPACKING


/**
 * @brief Get field time_usec from att_pos_mocap message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
static inline uint64_t mavlink_msg_att_pos_mocap_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field q from att_pos_mocap message
 *
 * @return  Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
 */
static inline uint16_t mavlink_msg_att_pos_mocap_get_q(const mavlink_message_t* msg, float *q)
{
    return _MAV_RETURN_float_array(msg, q, 4,  8);
}

/**
 * @brief Get field x from att_pos_mocap message
 *
 * @return [m] X position (NED)
 */
static inline float mavlink_msg_att_pos_mocap_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field y from att_pos_mocap message
 *
 * @return [m] Y position (NED)
 */
static inline float mavlink_msg_att_pos_mocap_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field z from att_pos_mocap message
 *
 * @return [m] Z position (NED)
 */
static inline float mavlink_msg_att_pos_mocap_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field covariance from att_pos_mocap message
 *
 * @return  Row-major representation of a pose 6x6 cross-covariance matrix upper right triangle (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.
 */
static inline uint16_t mavlink_msg_att_pos_mocap_get_covariance(const mavlink_message_t* msg, float *covariance)
{
    return _MAV_RETURN_float_array(msg, covariance, 21,  36);
}

/**
 * @brief Decode a att_pos_mocap message into a struct
 *
 * @param msg The message to decode
 * @param att_pos_mocap C-struct to decode the message contents into
 */
static inline void mavlink_msg_att_pos_mocap_decode(const mavlink_message_t* msg, mavlink_att_pos_mocap_t* att_pos_mocap)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    att_pos_mocap->time_usec = mavlink_msg_att_pos_mocap_get_time_usec(msg);
    mavlink_msg_att_pos_mocap_get_q(msg, att_pos_mocap->q);
    att_pos_mocap->x = mavlink_msg_att_pos_mocap_get_x(msg);
    att_pos_mocap->y = mavlink_msg_att_pos_mocap_get_y(msg);
    att_pos_mocap->z = mavlink_msg_att_pos_mocap_get_z(msg);
    mavlink_msg_att_pos_mocap_get_covariance(msg, att_pos_mocap->covariance);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ATT_POS_MOCAP_LEN? msg->len : MAVLINK_MSG_ID_ATT_POS_MOCAP_LEN;
        memset(att_pos_mocap, 0, MAVLINK_MSG_ID_ATT_POS_MOCAP_LEN);
    memcpy(att_pos_mocap, _MAV_PAYLOAD(msg), len);
#endif
}
