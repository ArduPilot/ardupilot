#pragma once
// MESSAGE VISION_POSITION_ESTIMATE PACKING

#define MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE 102


typedef struct __mavlink_vision_position_estimate_t {
 uint64_t usec; /*< [us] Timestamp (UNIX time or time since system boot)*/
 float x; /*< [m] Local X position*/
 float y; /*< [m] Local Y position*/
 float z; /*< [m] Local Z position*/
 float roll; /*< [rad] Roll angle*/
 float pitch; /*< [rad] Pitch angle*/
 float yaw; /*< [rad] Yaw angle*/
 float covariance[21]; /*<  Row-major representation of pose 6x6 cross-covariance matrix upper right triangle (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.*/
 uint8_t reset_counter; /*<  Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions (position, velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM system detects a loop-closure and the estimate jumps.*/
} mavlink_vision_position_estimate_t;

#define MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN 117
#define MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_MIN_LEN 32
#define MAVLINK_MSG_ID_102_LEN 117
#define MAVLINK_MSG_ID_102_MIN_LEN 32

#define MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_CRC 158
#define MAVLINK_MSG_ID_102_CRC 158

#define MAVLINK_MSG_VISION_POSITION_ESTIMATE_FIELD_COVARIANCE_LEN 21

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_VISION_POSITION_ESTIMATE { \
    102, \
    "VISION_POSITION_ESTIMATE", \
    9, \
    {  { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_vision_position_estimate_t, usec) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_vision_position_estimate_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_vision_position_estimate_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_vision_position_estimate_t, z) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_vision_position_estimate_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_vision_position_estimate_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_vision_position_estimate_t, yaw) }, \
         { "covariance", NULL, MAVLINK_TYPE_FLOAT, 21, 32, offsetof(mavlink_vision_position_estimate_t, covariance) }, \
         { "reset_counter", NULL, MAVLINK_TYPE_UINT8_T, 0, 116, offsetof(mavlink_vision_position_estimate_t, reset_counter) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_VISION_POSITION_ESTIMATE { \
    "VISION_POSITION_ESTIMATE", \
    9, \
    {  { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_vision_position_estimate_t, usec) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_vision_position_estimate_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_vision_position_estimate_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_vision_position_estimate_t, z) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_vision_position_estimate_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_vision_position_estimate_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_vision_position_estimate_t, yaw) }, \
         { "covariance", NULL, MAVLINK_TYPE_FLOAT, 21, 32, offsetof(mavlink_vision_position_estimate_t, covariance) }, \
         { "reset_counter", NULL, MAVLINK_TYPE_UINT8_T, 0, 116, offsetof(mavlink_vision_position_estimate_t, reset_counter) }, \
         } \
}
#endif

/**
 * @brief Pack a vision_position_estimate message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec [us] Timestamp (UNIX time or time since system boot)
 * @param x [m] Local X position
 * @param y [m] Local Y position
 * @param z [m] Local Z position
 * @param roll [rad] Roll angle
 * @param pitch [rad] Pitch angle
 * @param yaw [rad] Yaw angle
 * @param covariance  Row-major representation of pose 6x6 cross-covariance matrix upper right triangle (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.
 * @param reset_counter  Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions (position, velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM system detects a loop-closure and the estimate jumps.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vision_position_estimate_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t usec, float x, float y, float z, float roll, float pitch, float yaw, const float *covariance, uint8_t reset_counter)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN];
    _mav_put_uint64_t(buf, 0, usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_float(buf, 20, roll);
    _mav_put_float(buf, 24, pitch);
    _mav_put_float(buf, 28, yaw);
    _mav_put_uint8_t(buf, 116, reset_counter);
    _mav_put_float_array(buf, 32, covariance, 21);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN);
#else
    mavlink_vision_position_estimate_t packet;
    packet.usec = usec;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.reset_counter = reset_counter;
    mav_array_memcpy(packet.covariance, covariance, sizeof(float)*21);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_CRC);
}

/**
 * @brief Pack a vision_position_estimate message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec [us] Timestamp (UNIX time or time since system boot)
 * @param x [m] Local X position
 * @param y [m] Local Y position
 * @param z [m] Local Z position
 * @param roll [rad] Roll angle
 * @param pitch [rad] Pitch angle
 * @param yaw [rad] Yaw angle
 * @param covariance  Row-major representation of pose 6x6 cross-covariance matrix upper right triangle (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.
 * @param reset_counter  Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions (position, velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM system detects a loop-closure and the estimate jumps.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vision_position_estimate_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t usec, float x, float y, float z, float roll, float pitch, float yaw, const float *covariance, uint8_t reset_counter)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN];
    _mav_put_uint64_t(buf, 0, usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_float(buf, 20, roll);
    _mav_put_float(buf, 24, pitch);
    _mav_put_float(buf, 28, yaw);
    _mav_put_uint8_t(buf, 116, reset_counter);
    _mav_put_float_array(buf, 32, covariance, 21);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN);
#else
    mavlink_vision_position_estimate_t packet;
    packet.usec = usec;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.reset_counter = reset_counter;
    mav_array_memcpy(packet.covariance, covariance, sizeof(float)*21);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN);
#endif
}

/**
 * @brief Pack a vision_position_estimate message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec [us] Timestamp (UNIX time or time since system boot)
 * @param x [m] Local X position
 * @param y [m] Local Y position
 * @param z [m] Local Z position
 * @param roll [rad] Roll angle
 * @param pitch [rad] Pitch angle
 * @param yaw [rad] Yaw angle
 * @param covariance  Row-major representation of pose 6x6 cross-covariance matrix upper right triangle (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.
 * @param reset_counter  Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions (position, velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM system detects a loop-closure and the estimate jumps.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vision_position_estimate_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t usec,float x,float y,float z,float roll,float pitch,float yaw,const float *covariance,uint8_t reset_counter)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN];
    _mav_put_uint64_t(buf, 0, usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_float(buf, 20, roll);
    _mav_put_float(buf, 24, pitch);
    _mav_put_float(buf, 28, yaw);
    _mav_put_uint8_t(buf, 116, reset_counter);
    _mav_put_float_array(buf, 32, covariance, 21);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN);
#else
    mavlink_vision_position_estimate_t packet;
    packet.usec = usec;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.reset_counter = reset_counter;
    mav_array_memcpy(packet.covariance, covariance, sizeof(float)*21);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_CRC);
}

/**
 * @brief Encode a vision_position_estimate struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vision_position_estimate C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vision_position_estimate_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_vision_position_estimate_t* vision_position_estimate)
{
    return mavlink_msg_vision_position_estimate_pack(system_id, component_id, msg, vision_position_estimate->usec, vision_position_estimate->x, vision_position_estimate->y, vision_position_estimate->z, vision_position_estimate->roll, vision_position_estimate->pitch, vision_position_estimate->yaw, vision_position_estimate->covariance, vision_position_estimate->reset_counter);
}

/**
 * @brief Encode a vision_position_estimate struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vision_position_estimate C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vision_position_estimate_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_vision_position_estimate_t* vision_position_estimate)
{
    return mavlink_msg_vision_position_estimate_pack_chan(system_id, component_id, chan, msg, vision_position_estimate->usec, vision_position_estimate->x, vision_position_estimate->y, vision_position_estimate->z, vision_position_estimate->roll, vision_position_estimate->pitch, vision_position_estimate->yaw, vision_position_estimate->covariance, vision_position_estimate->reset_counter);
}

/**
 * @brief Encode a vision_position_estimate struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param vision_position_estimate C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vision_position_estimate_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_vision_position_estimate_t* vision_position_estimate)
{
    return mavlink_msg_vision_position_estimate_pack_status(system_id, component_id, _status, msg,  vision_position_estimate->usec, vision_position_estimate->x, vision_position_estimate->y, vision_position_estimate->z, vision_position_estimate->roll, vision_position_estimate->pitch, vision_position_estimate->yaw, vision_position_estimate->covariance, vision_position_estimate->reset_counter);
}

/**
 * @brief Send a vision_position_estimate message
 * @param chan MAVLink channel to send the message
 *
 * @param usec [us] Timestamp (UNIX time or time since system boot)
 * @param x [m] Local X position
 * @param y [m] Local Y position
 * @param z [m] Local Z position
 * @param roll [rad] Roll angle
 * @param pitch [rad] Pitch angle
 * @param yaw [rad] Yaw angle
 * @param covariance  Row-major representation of pose 6x6 cross-covariance matrix upper right triangle (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.
 * @param reset_counter  Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions (position, velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM system detects a loop-closure and the estimate jumps.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_vision_position_estimate_send(mavlink_channel_t chan, uint64_t usec, float x, float y, float z, float roll, float pitch, float yaw, const float *covariance, uint8_t reset_counter)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN];
    _mav_put_uint64_t(buf, 0, usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_float(buf, 20, roll);
    _mav_put_float(buf, 24, pitch);
    _mav_put_float(buf, 28, yaw);
    _mav_put_uint8_t(buf, 116, reset_counter);
    _mav_put_float_array(buf, 32, covariance, 21);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE, buf, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_CRC);
#else
    mavlink_vision_position_estimate_t packet;
    packet.usec = usec;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.reset_counter = reset_counter;
    mav_array_memcpy(packet.covariance, covariance, sizeof(float)*21);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE, (const char *)&packet, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_CRC);
#endif
}

/**
 * @brief Send a vision_position_estimate message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_vision_position_estimate_send_struct(mavlink_channel_t chan, const mavlink_vision_position_estimate_t* vision_position_estimate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_vision_position_estimate_send(chan, vision_position_estimate->usec, vision_position_estimate->x, vision_position_estimate->y, vision_position_estimate->z, vision_position_estimate->roll, vision_position_estimate->pitch, vision_position_estimate->yaw, vision_position_estimate->covariance, vision_position_estimate->reset_counter);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE, (const char *)vision_position_estimate, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_CRC);
#endif
}

#if MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_vision_position_estimate_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t usec, float x, float y, float z, float roll, float pitch, float yaw, const float *covariance, uint8_t reset_counter)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_float(buf, 20, roll);
    _mav_put_float(buf, 24, pitch);
    _mav_put_float(buf, 28, yaw);
    _mav_put_uint8_t(buf, 116, reset_counter);
    _mav_put_float_array(buf, 32, covariance, 21);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE, buf, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_CRC);
#else
    mavlink_vision_position_estimate_t *packet = (mavlink_vision_position_estimate_t *)msgbuf;
    packet->usec = usec;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;
    packet->reset_counter = reset_counter;
    mav_array_memcpy(packet->covariance, covariance, sizeof(float)*21);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE, (const char *)packet, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_CRC);
#endif
}
#endif

#endif

// MESSAGE VISION_POSITION_ESTIMATE UNPACKING


/**
 * @brief Get field usec from vision_position_estimate message
 *
 * @return [us] Timestamp (UNIX time or time since system boot)
 */
static inline uint64_t mavlink_msg_vision_position_estimate_get_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field x from vision_position_estimate message
 *
 * @return [m] Local X position
 */
static inline float mavlink_msg_vision_position_estimate_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field y from vision_position_estimate message
 *
 * @return [m] Local Y position
 */
static inline float mavlink_msg_vision_position_estimate_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field z from vision_position_estimate message
 *
 * @return [m] Local Z position
 */
static inline float mavlink_msg_vision_position_estimate_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field roll from vision_position_estimate message
 *
 * @return [rad] Roll angle
 */
static inline float mavlink_msg_vision_position_estimate_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field pitch from vision_position_estimate message
 *
 * @return [rad] Pitch angle
 */
static inline float mavlink_msg_vision_position_estimate_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field yaw from vision_position_estimate message
 *
 * @return [rad] Yaw angle
 */
static inline float mavlink_msg_vision_position_estimate_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field covariance from vision_position_estimate message
 *
 * @return  Row-major representation of pose 6x6 cross-covariance matrix upper right triangle (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.
 */
static inline uint16_t mavlink_msg_vision_position_estimate_get_covariance(const mavlink_message_t* msg, float *covariance)
{
    return _MAV_RETURN_float_array(msg, covariance, 21,  32);
}

/**
 * @brief Get field reset_counter from vision_position_estimate message
 *
 * @return  Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions (position, velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM system detects a loop-closure and the estimate jumps.
 */
static inline uint8_t mavlink_msg_vision_position_estimate_get_reset_counter(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  116);
}

/**
 * @brief Decode a vision_position_estimate message into a struct
 *
 * @param msg The message to decode
 * @param vision_position_estimate C-struct to decode the message contents into
 */
static inline void mavlink_msg_vision_position_estimate_decode(const mavlink_message_t* msg, mavlink_vision_position_estimate_t* vision_position_estimate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    vision_position_estimate->usec = mavlink_msg_vision_position_estimate_get_usec(msg);
    vision_position_estimate->x = mavlink_msg_vision_position_estimate_get_x(msg);
    vision_position_estimate->y = mavlink_msg_vision_position_estimate_get_y(msg);
    vision_position_estimate->z = mavlink_msg_vision_position_estimate_get_z(msg);
    vision_position_estimate->roll = mavlink_msg_vision_position_estimate_get_roll(msg);
    vision_position_estimate->pitch = mavlink_msg_vision_position_estimate_get_pitch(msg);
    vision_position_estimate->yaw = mavlink_msg_vision_position_estimate_get_yaw(msg);
    mavlink_msg_vision_position_estimate_get_covariance(msg, vision_position_estimate->covariance);
    vision_position_estimate->reset_counter = mavlink_msg_vision_position_estimate_get_reset_counter(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN? msg->len : MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN;
        memset(vision_position_estimate, 0, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN);
    memcpy(vision_position_estimate, _MAV_PAYLOAD(msg), len);
#endif
}
