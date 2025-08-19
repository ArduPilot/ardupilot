#pragma once
// MESSAGE VISION_SPEED_ESTIMATE PACKING

#define MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE 103


typedef struct __mavlink_vision_speed_estimate_t {
 uint64_t usec; /*< [us] Timestamp (UNIX time or time since system boot)*/
 float x; /*< [m/s] Global X speed*/
 float y; /*< [m/s] Global Y speed*/
 float z; /*< [m/s] Global Z speed*/
 float covariance[9]; /*<  Row-major representation of 3x3 linear velocity covariance matrix (states: vx, vy, vz; 1st three entries - 1st row, etc.). If unknown, assign NaN value to first element in the array.*/
 uint8_t reset_counter; /*<  Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions (position, velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM system detects a loop-closure and the estimate jumps.*/
} mavlink_vision_speed_estimate_t;

#define MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_LEN 57
#define MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_MIN_LEN 20
#define MAVLINK_MSG_ID_103_LEN 57
#define MAVLINK_MSG_ID_103_MIN_LEN 20

#define MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_CRC 208
#define MAVLINK_MSG_ID_103_CRC 208

#define MAVLINK_MSG_VISION_SPEED_ESTIMATE_FIELD_COVARIANCE_LEN 9

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_VISION_SPEED_ESTIMATE { \
    103, \
    "VISION_SPEED_ESTIMATE", \
    6, \
    {  { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_vision_speed_estimate_t, usec) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_vision_speed_estimate_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_vision_speed_estimate_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_vision_speed_estimate_t, z) }, \
         { "covariance", NULL, MAVLINK_TYPE_FLOAT, 9, 20, offsetof(mavlink_vision_speed_estimate_t, covariance) }, \
         { "reset_counter", NULL, MAVLINK_TYPE_UINT8_T, 0, 56, offsetof(mavlink_vision_speed_estimate_t, reset_counter) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_VISION_SPEED_ESTIMATE { \
    "VISION_SPEED_ESTIMATE", \
    6, \
    {  { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_vision_speed_estimate_t, usec) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_vision_speed_estimate_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_vision_speed_estimate_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_vision_speed_estimate_t, z) }, \
         { "covariance", NULL, MAVLINK_TYPE_FLOAT, 9, 20, offsetof(mavlink_vision_speed_estimate_t, covariance) }, \
         { "reset_counter", NULL, MAVLINK_TYPE_UINT8_T, 0, 56, offsetof(mavlink_vision_speed_estimate_t, reset_counter) }, \
         } \
}
#endif

/**
 * @brief Pack a vision_speed_estimate message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec [us] Timestamp (UNIX time or time since system boot)
 * @param x [m/s] Global X speed
 * @param y [m/s] Global Y speed
 * @param z [m/s] Global Z speed
 * @param covariance  Row-major representation of 3x3 linear velocity covariance matrix (states: vx, vy, vz; 1st three entries - 1st row, etc.). If unknown, assign NaN value to first element in the array.
 * @param reset_counter  Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions (position, velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM system detects a loop-closure and the estimate jumps.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vision_speed_estimate_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t usec, float x, float y, float z, const float *covariance, uint8_t reset_counter)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_LEN];
    _mav_put_uint64_t(buf, 0, usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_uint8_t(buf, 56, reset_counter);
    _mav_put_float_array(buf, 20, covariance, 9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_LEN);
#else
    mavlink_vision_speed_estimate_t packet;
    packet.usec = usec;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.reset_counter = reset_counter;
    mav_array_memcpy(packet.covariance, covariance, sizeof(float)*9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_LEN, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_CRC);
}

/**
 * @brief Pack a vision_speed_estimate message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec [us] Timestamp (UNIX time or time since system boot)
 * @param x [m/s] Global X speed
 * @param y [m/s] Global Y speed
 * @param z [m/s] Global Z speed
 * @param covariance  Row-major representation of 3x3 linear velocity covariance matrix (states: vx, vy, vz; 1st three entries - 1st row, etc.). If unknown, assign NaN value to first element in the array.
 * @param reset_counter  Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions (position, velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM system detects a loop-closure and the estimate jumps.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vision_speed_estimate_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t usec, float x, float y, float z, const float *covariance, uint8_t reset_counter)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_LEN];
    _mav_put_uint64_t(buf, 0, usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_uint8_t(buf, 56, reset_counter);
    _mav_put_float_array(buf, 20, covariance, 9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_LEN);
#else
    mavlink_vision_speed_estimate_t packet;
    packet.usec = usec;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.reset_counter = reset_counter;
    mav_array_memcpy(packet.covariance, covariance, sizeof(float)*9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_LEN, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_LEN);
#endif
}

/**
 * @brief Pack a vision_speed_estimate message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec [us] Timestamp (UNIX time or time since system boot)
 * @param x [m/s] Global X speed
 * @param y [m/s] Global Y speed
 * @param z [m/s] Global Z speed
 * @param covariance  Row-major representation of 3x3 linear velocity covariance matrix (states: vx, vy, vz; 1st three entries - 1st row, etc.). If unknown, assign NaN value to first element in the array.
 * @param reset_counter  Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions (position, velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM system detects a loop-closure and the estimate jumps.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vision_speed_estimate_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t usec,float x,float y,float z,const float *covariance,uint8_t reset_counter)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_LEN];
    _mav_put_uint64_t(buf, 0, usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_uint8_t(buf, 56, reset_counter);
    _mav_put_float_array(buf, 20, covariance, 9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_LEN);
#else
    mavlink_vision_speed_estimate_t packet;
    packet.usec = usec;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.reset_counter = reset_counter;
    mav_array_memcpy(packet.covariance, covariance, sizeof(float)*9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_LEN, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_CRC);
}

/**
 * @brief Encode a vision_speed_estimate struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vision_speed_estimate C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vision_speed_estimate_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_vision_speed_estimate_t* vision_speed_estimate)
{
    return mavlink_msg_vision_speed_estimate_pack(system_id, component_id, msg, vision_speed_estimate->usec, vision_speed_estimate->x, vision_speed_estimate->y, vision_speed_estimate->z, vision_speed_estimate->covariance, vision_speed_estimate->reset_counter);
}

/**
 * @brief Encode a vision_speed_estimate struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vision_speed_estimate C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vision_speed_estimate_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_vision_speed_estimate_t* vision_speed_estimate)
{
    return mavlink_msg_vision_speed_estimate_pack_chan(system_id, component_id, chan, msg, vision_speed_estimate->usec, vision_speed_estimate->x, vision_speed_estimate->y, vision_speed_estimate->z, vision_speed_estimate->covariance, vision_speed_estimate->reset_counter);
}

/**
 * @brief Encode a vision_speed_estimate struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param vision_speed_estimate C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vision_speed_estimate_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_vision_speed_estimate_t* vision_speed_estimate)
{
    return mavlink_msg_vision_speed_estimate_pack_status(system_id, component_id, _status, msg,  vision_speed_estimate->usec, vision_speed_estimate->x, vision_speed_estimate->y, vision_speed_estimate->z, vision_speed_estimate->covariance, vision_speed_estimate->reset_counter);
}

/**
 * @brief Send a vision_speed_estimate message
 * @param chan MAVLink channel to send the message
 *
 * @param usec [us] Timestamp (UNIX time or time since system boot)
 * @param x [m/s] Global X speed
 * @param y [m/s] Global Y speed
 * @param z [m/s] Global Z speed
 * @param covariance  Row-major representation of 3x3 linear velocity covariance matrix (states: vx, vy, vz; 1st three entries - 1st row, etc.). If unknown, assign NaN value to first element in the array.
 * @param reset_counter  Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions (position, velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM system detects a loop-closure and the estimate jumps.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_vision_speed_estimate_send(mavlink_channel_t chan, uint64_t usec, float x, float y, float z, const float *covariance, uint8_t reset_counter)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_LEN];
    _mav_put_uint64_t(buf, 0, usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_uint8_t(buf, 56, reset_counter);
    _mav_put_float_array(buf, 20, covariance, 9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE, buf, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_LEN, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_CRC);
#else
    mavlink_vision_speed_estimate_t packet;
    packet.usec = usec;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.reset_counter = reset_counter;
    mav_array_memcpy(packet.covariance, covariance, sizeof(float)*9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE, (const char *)&packet, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_LEN, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_CRC);
#endif
}

/**
 * @brief Send a vision_speed_estimate message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_vision_speed_estimate_send_struct(mavlink_channel_t chan, const mavlink_vision_speed_estimate_t* vision_speed_estimate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_vision_speed_estimate_send(chan, vision_speed_estimate->usec, vision_speed_estimate->x, vision_speed_estimate->y, vision_speed_estimate->z, vision_speed_estimate->covariance, vision_speed_estimate->reset_counter);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE, (const char *)vision_speed_estimate, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_LEN, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_CRC);
#endif
}

#if MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_vision_speed_estimate_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t usec, float x, float y, float z, const float *covariance, uint8_t reset_counter)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_uint8_t(buf, 56, reset_counter);
    _mav_put_float_array(buf, 20, covariance, 9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE, buf, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_LEN, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_CRC);
#else
    mavlink_vision_speed_estimate_t *packet = (mavlink_vision_speed_estimate_t *)msgbuf;
    packet->usec = usec;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->reset_counter = reset_counter;
    mav_array_memcpy(packet->covariance, covariance, sizeof(float)*9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE, (const char *)packet, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_LEN, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_CRC);
#endif
}
#endif

#endif

// MESSAGE VISION_SPEED_ESTIMATE UNPACKING


/**
 * @brief Get field usec from vision_speed_estimate message
 *
 * @return [us] Timestamp (UNIX time or time since system boot)
 */
static inline uint64_t mavlink_msg_vision_speed_estimate_get_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field x from vision_speed_estimate message
 *
 * @return [m/s] Global X speed
 */
static inline float mavlink_msg_vision_speed_estimate_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field y from vision_speed_estimate message
 *
 * @return [m/s] Global Y speed
 */
static inline float mavlink_msg_vision_speed_estimate_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field z from vision_speed_estimate message
 *
 * @return [m/s] Global Z speed
 */
static inline float mavlink_msg_vision_speed_estimate_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field covariance from vision_speed_estimate message
 *
 * @return  Row-major representation of 3x3 linear velocity covariance matrix (states: vx, vy, vz; 1st three entries - 1st row, etc.). If unknown, assign NaN value to first element in the array.
 */
static inline uint16_t mavlink_msg_vision_speed_estimate_get_covariance(const mavlink_message_t* msg, float *covariance)
{
    return _MAV_RETURN_float_array(msg, covariance, 9,  20);
}

/**
 * @brief Get field reset_counter from vision_speed_estimate message
 *
 * @return  Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions (position, velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM system detects a loop-closure and the estimate jumps.
 */
static inline uint8_t mavlink_msg_vision_speed_estimate_get_reset_counter(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  56);
}

/**
 * @brief Decode a vision_speed_estimate message into a struct
 *
 * @param msg The message to decode
 * @param vision_speed_estimate C-struct to decode the message contents into
 */
static inline void mavlink_msg_vision_speed_estimate_decode(const mavlink_message_t* msg, mavlink_vision_speed_estimate_t* vision_speed_estimate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    vision_speed_estimate->usec = mavlink_msg_vision_speed_estimate_get_usec(msg);
    vision_speed_estimate->x = mavlink_msg_vision_speed_estimate_get_x(msg);
    vision_speed_estimate->y = mavlink_msg_vision_speed_estimate_get_y(msg);
    vision_speed_estimate->z = mavlink_msg_vision_speed_estimate_get_z(msg);
    mavlink_msg_vision_speed_estimate_get_covariance(msg, vision_speed_estimate->covariance);
    vision_speed_estimate->reset_counter = mavlink_msg_vision_speed_estimate_get_reset_counter(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_LEN? msg->len : MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_LEN;
        memset(vision_speed_estimate, 0, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE_LEN);
    memcpy(vision_speed_estimate, _MAV_PAYLOAD(msg), len);
#endif
}
