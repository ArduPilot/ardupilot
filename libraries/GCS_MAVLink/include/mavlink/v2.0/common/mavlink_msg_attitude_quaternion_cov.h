#pragma once
// MESSAGE ATTITUDE_QUATERNION_COV PACKING

#define MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV 61


typedef struct __mavlink_attitude_quaternion_cov_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 float q[4]; /*<  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)*/
 float rollspeed; /*< [rad/s] Roll angular speed*/
 float pitchspeed; /*< [rad/s] Pitch angular speed*/
 float yawspeed; /*< [rad/s] Yaw angular speed*/
 float covariance[9]; /*<  Row-major representation of a 3x3 attitude covariance matrix (states: roll, pitch, yaw; first three entries are the first ROW, next three entries are the second row, etc.). If unknown, assign NaN value to first element in the array.*/
} mavlink_attitude_quaternion_cov_t;

#define MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN 72
#define MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_MIN_LEN 72
#define MAVLINK_MSG_ID_61_LEN 72
#define MAVLINK_MSG_ID_61_MIN_LEN 72

#define MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_CRC 167
#define MAVLINK_MSG_ID_61_CRC 167

#define MAVLINK_MSG_ATTITUDE_QUATERNION_COV_FIELD_Q_LEN 4
#define MAVLINK_MSG_ATTITUDE_QUATERNION_COV_FIELD_COVARIANCE_LEN 9

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ATTITUDE_QUATERNION_COV { \
    61, \
    "ATTITUDE_QUATERNION_COV", \
    6, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_attitude_quaternion_cov_t, time_usec) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 4, 8, offsetof(mavlink_attitude_quaternion_cov_t, q) }, \
         { "rollspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_attitude_quaternion_cov_t, rollspeed) }, \
         { "pitchspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_attitude_quaternion_cov_t, pitchspeed) }, \
         { "yawspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_attitude_quaternion_cov_t, yawspeed) }, \
         { "covariance", NULL, MAVLINK_TYPE_FLOAT, 9, 36, offsetof(mavlink_attitude_quaternion_cov_t, covariance) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ATTITUDE_QUATERNION_COV { \
    "ATTITUDE_QUATERNION_COV", \
    6, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_attitude_quaternion_cov_t, time_usec) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 4, 8, offsetof(mavlink_attitude_quaternion_cov_t, q) }, \
         { "rollspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_attitude_quaternion_cov_t, rollspeed) }, \
         { "pitchspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_attitude_quaternion_cov_t, pitchspeed) }, \
         { "yawspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_attitude_quaternion_cov_t, yawspeed) }, \
         { "covariance", NULL, MAVLINK_TYPE_FLOAT, 9, 36, offsetof(mavlink_attitude_quaternion_cov_t, covariance) }, \
         } \
}
#endif

/**
 * @brief Pack a attitude_quaternion_cov message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param q  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
 * @param rollspeed [rad/s] Roll angular speed
 * @param pitchspeed [rad/s] Pitch angular speed
 * @param yawspeed [rad/s] Yaw angular speed
 * @param covariance  Row-major representation of a 3x3 attitude covariance matrix (states: roll, pitch, yaw; first three entries are the first ROW, next three entries are the second row, etc.). If unknown, assign NaN value to first element in the array.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude_quaternion_cov_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, const float *q, float rollspeed, float pitchspeed, float yawspeed, const float *covariance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 24, rollspeed);
    _mav_put_float(buf, 28, pitchspeed);
    _mav_put_float(buf, 32, yawspeed);
    _mav_put_float_array(buf, 8, q, 4);
    _mav_put_float_array(buf, 36, covariance, 9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN);
#else
    mavlink_attitude_quaternion_cov_t packet;
    packet.time_usec = time_usec;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
    mav_array_memcpy(packet.covariance, covariance, sizeof(float)*9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_CRC);
}

/**
 * @brief Pack a attitude_quaternion_cov message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param q  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
 * @param rollspeed [rad/s] Roll angular speed
 * @param pitchspeed [rad/s] Pitch angular speed
 * @param yawspeed [rad/s] Yaw angular speed
 * @param covariance  Row-major representation of a 3x3 attitude covariance matrix (states: roll, pitch, yaw; first three entries are the first ROW, next three entries are the second row, etc.). If unknown, assign NaN value to first element in the array.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude_quaternion_cov_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t time_usec, const float *q, float rollspeed, float pitchspeed, float yawspeed, const float *covariance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 24, rollspeed);
    _mav_put_float(buf, 28, pitchspeed);
    _mav_put_float(buf, 32, yawspeed);
    _mav_put_float_array(buf, 8, q, 4);
    _mav_put_float_array(buf, 36, covariance, 9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN);
#else
    mavlink_attitude_quaternion_cov_t packet;
    packet.time_usec = time_usec;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
    mav_array_memcpy(packet.covariance, covariance, sizeof(float)*9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN);
#endif
}

/**
 * @brief Pack a attitude_quaternion_cov message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param q  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
 * @param rollspeed [rad/s] Roll angular speed
 * @param pitchspeed [rad/s] Pitch angular speed
 * @param yawspeed [rad/s] Yaw angular speed
 * @param covariance  Row-major representation of a 3x3 attitude covariance matrix (states: roll, pitch, yaw; first three entries are the first ROW, next three entries are the second row, etc.). If unknown, assign NaN value to first element in the array.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude_quaternion_cov_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,const float *q,float rollspeed,float pitchspeed,float yawspeed,const float *covariance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 24, rollspeed);
    _mav_put_float(buf, 28, pitchspeed);
    _mav_put_float(buf, 32, yawspeed);
    _mav_put_float_array(buf, 8, q, 4);
    _mav_put_float_array(buf, 36, covariance, 9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN);
#else
    mavlink_attitude_quaternion_cov_t packet;
    packet.time_usec = time_usec;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
    mav_array_memcpy(packet.covariance, covariance, sizeof(float)*9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_CRC);
}

/**
 * @brief Encode a attitude_quaternion_cov struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param attitude_quaternion_cov C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_attitude_quaternion_cov_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_attitude_quaternion_cov_t* attitude_quaternion_cov)
{
    return mavlink_msg_attitude_quaternion_cov_pack(system_id, component_id, msg, attitude_quaternion_cov->time_usec, attitude_quaternion_cov->q, attitude_quaternion_cov->rollspeed, attitude_quaternion_cov->pitchspeed, attitude_quaternion_cov->yawspeed, attitude_quaternion_cov->covariance);
}

/**
 * @brief Encode a attitude_quaternion_cov struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param attitude_quaternion_cov C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_attitude_quaternion_cov_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_attitude_quaternion_cov_t* attitude_quaternion_cov)
{
    return mavlink_msg_attitude_quaternion_cov_pack_chan(system_id, component_id, chan, msg, attitude_quaternion_cov->time_usec, attitude_quaternion_cov->q, attitude_quaternion_cov->rollspeed, attitude_quaternion_cov->pitchspeed, attitude_quaternion_cov->yawspeed, attitude_quaternion_cov->covariance);
}

/**
 * @brief Encode a attitude_quaternion_cov struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param attitude_quaternion_cov C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_attitude_quaternion_cov_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_attitude_quaternion_cov_t* attitude_quaternion_cov)
{
    return mavlink_msg_attitude_quaternion_cov_pack_status(system_id, component_id, _status, msg,  attitude_quaternion_cov->time_usec, attitude_quaternion_cov->q, attitude_quaternion_cov->rollspeed, attitude_quaternion_cov->pitchspeed, attitude_quaternion_cov->yawspeed, attitude_quaternion_cov->covariance);
}

/**
 * @brief Send a attitude_quaternion_cov message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param q  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
 * @param rollspeed [rad/s] Roll angular speed
 * @param pitchspeed [rad/s] Pitch angular speed
 * @param yawspeed [rad/s] Yaw angular speed
 * @param covariance  Row-major representation of a 3x3 attitude covariance matrix (states: roll, pitch, yaw; first three entries are the first ROW, next three entries are the second row, etc.). If unknown, assign NaN value to first element in the array.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_attitude_quaternion_cov_send(mavlink_channel_t chan, uint64_t time_usec, const float *q, float rollspeed, float pitchspeed, float yawspeed, const float *covariance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 24, rollspeed);
    _mav_put_float(buf, 28, pitchspeed);
    _mav_put_float(buf, 32, yawspeed);
    _mav_put_float_array(buf, 8, q, 4);
    _mav_put_float_array(buf, 36, covariance, 9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV, buf, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_CRC);
#else
    mavlink_attitude_quaternion_cov_t packet;
    packet.time_usec = time_usec;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
    mav_array_memcpy(packet.covariance, covariance, sizeof(float)*9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV, (const char *)&packet, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_CRC);
#endif
}

/**
 * @brief Send a attitude_quaternion_cov message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_attitude_quaternion_cov_send_struct(mavlink_channel_t chan, const mavlink_attitude_quaternion_cov_t* attitude_quaternion_cov)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_attitude_quaternion_cov_send(chan, attitude_quaternion_cov->time_usec, attitude_quaternion_cov->q, attitude_quaternion_cov->rollspeed, attitude_quaternion_cov->pitchspeed, attitude_quaternion_cov->yawspeed, attitude_quaternion_cov->covariance);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV, (const char *)attitude_quaternion_cov, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_CRC);
#endif
}

#if MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_attitude_quaternion_cov_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, const float *q, float rollspeed, float pitchspeed, float yawspeed, const float *covariance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 24, rollspeed);
    _mav_put_float(buf, 28, pitchspeed);
    _mav_put_float(buf, 32, yawspeed);
    _mav_put_float_array(buf, 8, q, 4);
    _mav_put_float_array(buf, 36, covariance, 9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV, buf, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_CRC);
#else
    mavlink_attitude_quaternion_cov_t *packet = (mavlink_attitude_quaternion_cov_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->rollspeed = rollspeed;
    packet->pitchspeed = pitchspeed;
    packet->yawspeed = yawspeed;
    mav_array_memcpy(packet->q, q, sizeof(float)*4);
    mav_array_memcpy(packet->covariance, covariance, sizeof(float)*9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV, (const char *)packet, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_CRC);
#endif
}
#endif

#endif

// MESSAGE ATTITUDE_QUATERNION_COV UNPACKING


/**
 * @brief Get field time_usec from attitude_quaternion_cov message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
static inline uint64_t mavlink_msg_attitude_quaternion_cov_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field q from attitude_quaternion_cov message
 *
 * @return  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
 */
static inline uint16_t mavlink_msg_attitude_quaternion_cov_get_q(const mavlink_message_t* msg, float *q)
{
    return _MAV_RETURN_float_array(msg, q, 4,  8);
}

/**
 * @brief Get field rollspeed from attitude_quaternion_cov message
 *
 * @return [rad/s] Roll angular speed
 */
static inline float mavlink_msg_attitude_quaternion_cov_get_rollspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field pitchspeed from attitude_quaternion_cov message
 *
 * @return [rad/s] Pitch angular speed
 */
static inline float mavlink_msg_attitude_quaternion_cov_get_pitchspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field yawspeed from attitude_quaternion_cov message
 *
 * @return [rad/s] Yaw angular speed
 */
static inline float mavlink_msg_attitude_quaternion_cov_get_yawspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field covariance from attitude_quaternion_cov message
 *
 * @return  Row-major representation of a 3x3 attitude covariance matrix (states: roll, pitch, yaw; first three entries are the first ROW, next three entries are the second row, etc.). If unknown, assign NaN value to first element in the array.
 */
static inline uint16_t mavlink_msg_attitude_quaternion_cov_get_covariance(const mavlink_message_t* msg, float *covariance)
{
    return _MAV_RETURN_float_array(msg, covariance, 9,  36);
}

/**
 * @brief Decode a attitude_quaternion_cov message into a struct
 *
 * @param msg The message to decode
 * @param attitude_quaternion_cov C-struct to decode the message contents into
 */
static inline void mavlink_msg_attitude_quaternion_cov_decode(const mavlink_message_t* msg, mavlink_attitude_quaternion_cov_t* attitude_quaternion_cov)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    attitude_quaternion_cov->time_usec = mavlink_msg_attitude_quaternion_cov_get_time_usec(msg);
    mavlink_msg_attitude_quaternion_cov_get_q(msg, attitude_quaternion_cov->q);
    attitude_quaternion_cov->rollspeed = mavlink_msg_attitude_quaternion_cov_get_rollspeed(msg);
    attitude_quaternion_cov->pitchspeed = mavlink_msg_attitude_quaternion_cov_get_pitchspeed(msg);
    attitude_quaternion_cov->yawspeed = mavlink_msg_attitude_quaternion_cov_get_yawspeed(msg);
    mavlink_msg_attitude_quaternion_cov_get_covariance(msg, attitude_quaternion_cov->covariance);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN? msg->len : MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN;
        memset(attitude_quaternion_cov, 0, MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN);
    memcpy(attitude_quaternion_cov, _MAV_PAYLOAD(msg), len);
#endif
}
