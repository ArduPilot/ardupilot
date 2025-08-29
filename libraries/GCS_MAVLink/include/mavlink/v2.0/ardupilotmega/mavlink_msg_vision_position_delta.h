#pragma once
// MESSAGE VISION_POSITION_DELTA PACKING

#define MAVLINK_MSG_ID_VISION_POSITION_DELTA 11011


typedef struct __mavlink_vision_position_delta_t {
 uint64_t time_usec; /*< [us] Timestamp (synced to UNIX time or since system boot).*/
 uint64_t time_delta_usec; /*< [us] Time since the last reported camera frame.*/
 float angle_delta[3]; /*< [rad] Defines a rotation vector [roll, pitch, yaw] to the current MAV_FRAME_BODY_FRD from the previous MAV_FRAME_BODY_FRD.*/
 float position_delta[3]; /*< [m] Change in position to the current MAV_FRAME_BODY_FRD from the previous FRAME_BODY_FRD rotated to the current MAV_FRAME_BODY_FRD.*/
 float confidence; /*< [%] Normalised confidence value from 0 to 100.*/
} mavlink_vision_position_delta_t;

#define MAVLINK_MSG_ID_VISION_POSITION_DELTA_LEN 44
#define MAVLINK_MSG_ID_VISION_POSITION_DELTA_MIN_LEN 44
#define MAVLINK_MSG_ID_11011_LEN 44
#define MAVLINK_MSG_ID_11011_MIN_LEN 44

#define MAVLINK_MSG_ID_VISION_POSITION_DELTA_CRC 106
#define MAVLINK_MSG_ID_11011_CRC 106

#define MAVLINK_MSG_VISION_POSITION_DELTA_FIELD_ANGLE_DELTA_LEN 3
#define MAVLINK_MSG_VISION_POSITION_DELTA_FIELD_POSITION_DELTA_LEN 3

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_VISION_POSITION_DELTA { \
    11011, \
    "VISION_POSITION_DELTA", \
    5, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_vision_position_delta_t, time_usec) }, \
         { "time_delta_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_vision_position_delta_t, time_delta_usec) }, \
         { "angle_delta", NULL, MAVLINK_TYPE_FLOAT, 3, 16, offsetof(mavlink_vision_position_delta_t, angle_delta) }, \
         { "position_delta", NULL, MAVLINK_TYPE_FLOAT, 3, 28, offsetof(mavlink_vision_position_delta_t, position_delta) }, \
         { "confidence", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_vision_position_delta_t, confidence) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_VISION_POSITION_DELTA { \
    "VISION_POSITION_DELTA", \
    5, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_vision_position_delta_t, time_usec) }, \
         { "time_delta_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_vision_position_delta_t, time_delta_usec) }, \
         { "angle_delta", NULL, MAVLINK_TYPE_FLOAT, 3, 16, offsetof(mavlink_vision_position_delta_t, angle_delta) }, \
         { "position_delta", NULL, MAVLINK_TYPE_FLOAT, 3, 28, offsetof(mavlink_vision_position_delta_t, position_delta) }, \
         { "confidence", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_vision_position_delta_t, confidence) }, \
         } \
}
#endif

/**
 * @brief Pack a vision_position_delta message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (synced to UNIX time or since system boot).
 * @param time_delta_usec [us] Time since the last reported camera frame.
 * @param angle_delta [rad] Defines a rotation vector [roll, pitch, yaw] to the current MAV_FRAME_BODY_FRD from the previous MAV_FRAME_BODY_FRD.
 * @param position_delta [m] Change in position to the current MAV_FRAME_BODY_FRD from the previous FRAME_BODY_FRD rotated to the current MAV_FRAME_BODY_FRD.
 * @param confidence [%] Normalised confidence value from 0 to 100.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vision_position_delta_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, uint64_t time_delta_usec, const float *angle_delta, const float *position_delta, float confidence)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VISION_POSITION_DELTA_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint64_t(buf, 8, time_delta_usec);
    _mav_put_float(buf, 40, confidence);
    _mav_put_float_array(buf, 16, angle_delta, 3);
    _mav_put_float_array(buf, 28, position_delta, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VISION_POSITION_DELTA_LEN);
#else
    mavlink_vision_position_delta_t packet;
    packet.time_usec = time_usec;
    packet.time_delta_usec = time_delta_usec;
    packet.confidence = confidence;
    mav_array_memcpy(packet.angle_delta, angle_delta, sizeof(float)*3);
    mav_array_memcpy(packet.position_delta, position_delta, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VISION_POSITION_DELTA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VISION_POSITION_DELTA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VISION_POSITION_DELTA_MIN_LEN, MAVLINK_MSG_ID_VISION_POSITION_DELTA_LEN, MAVLINK_MSG_ID_VISION_POSITION_DELTA_CRC);
}

/**
 * @brief Pack a vision_position_delta message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (synced to UNIX time or since system boot).
 * @param time_delta_usec [us] Time since the last reported camera frame.
 * @param angle_delta [rad] Defines a rotation vector [roll, pitch, yaw] to the current MAV_FRAME_BODY_FRD from the previous MAV_FRAME_BODY_FRD.
 * @param position_delta [m] Change in position to the current MAV_FRAME_BODY_FRD from the previous FRAME_BODY_FRD rotated to the current MAV_FRAME_BODY_FRD.
 * @param confidence [%] Normalised confidence value from 0 to 100.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vision_position_delta_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t time_usec, uint64_t time_delta_usec, const float *angle_delta, const float *position_delta, float confidence)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VISION_POSITION_DELTA_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint64_t(buf, 8, time_delta_usec);
    _mav_put_float(buf, 40, confidence);
    _mav_put_float_array(buf, 16, angle_delta, 3);
    _mav_put_float_array(buf, 28, position_delta, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VISION_POSITION_DELTA_LEN);
#else
    mavlink_vision_position_delta_t packet;
    packet.time_usec = time_usec;
    packet.time_delta_usec = time_delta_usec;
    packet.confidence = confidence;
    mav_array_memcpy(packet.angle_delta, angle_delta, sizeof(float)*3);
    mav_array_memcpy(packet.position_delta, position_delta, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VISION_POSITION_DELTA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VISION_POSITION_DELTA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_VISION_POSITION_DELTA_MIN_LEN, MAVLINK_MSG_ID_VISION_POSITION_DELTA_LEN, MAVLINK_MSG_ID_VISION_POSITION_DELTA_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_VISION_POSITION_DELTA_MIN_LEN, MAVLINK_MSG_ID_VISION_POSITION_DELTA_LEN);
#endif
}

/**
 * @brief Pack a vision_position_delta message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (synced to UNIX time or since system boot).
 * @param time_delta_usec [us] Time since the last reported camera frame.
 * @param angle_delta [rad] Defines a rotation vector [roll, pitch, yaw] to the current MAV_FRAME_BODY_FRD from the previous MAV_FRAME_BODY_FRD.
 * @param position_delta [m] Change in position to the current MAV_FRAME_BODY_FRD from the previous FRAME_BODY_FRD rotated to the current MAV_FRAME_BODY_FRD.
 * @param confidence [%] Normalised confidence value from 0 to 100.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vision_position_delta_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,uint64_t time_delta_usec,const float *angle_delta,const float *position_delta,float confidence)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VISION_POSITION_DELTA_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint64_t(buf, 8, time_delta_usec);
    _mav_put_float(buf, 40, confidence);
    _mav_put_float_array(buf, 16, angle_delta, 3);
    _mav_put_float_array(buf, 28, position_delta, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VISION_POSITION_DELTA_LEN);
#else
    mavlink_vision_position_delta_t packet;
    packet.time_usec = time_usec;
    packet.time_delta_usec = time_delta_usec;
    packet.confidence = confidence;
    mav_array_memcpy(packet.angle_delta, angle_delta, sizeof(float)*3);
    mav_array_memcpy(packet.position_delta, position_delta, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VISION_POSITION_DELTA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VISION_POSITION_DELTA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VISION_POSITION_DELTA_MIN_LEN, MAVLINK_MSG_ID_VISION_POSITION_DELTA_LEN, MAVLINK_MSG_ID_VISION_POSITION_DELTA_CRC);
}

/**
 * @brief Encode a vision_position_delta struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vision_position_delta C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vision_position_delta_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_vision_position_delta_t* vision_position_delta)
{
    return mavlink_msg_vision_position_delta_pack(system_id, component_id, msg, vision_position_delta->time_usec, vision_position_delta->time_delta_usec, vision_position_delta->angle_delta, vision_position_delta->position_delta, vision_position_delta->confidence);
}

/**
 * @brief Encode a vision_position_delta struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vision_position_delta C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vision_position_delta_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_vision_position_delta_t* vision_position_delta)
{
    return mavlink_msg_vision_position_delta_pack_chan(system_id, component_id, chan, msg, vision_position_delta->time_usec, vision_position_delta->time_delta_usec, vision_position_delta->angle_delta, vision_position_delta->position_delta, vision_position_delta->confidence);
}

/**
 * @brief Encode a vision_position_delta struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param vision_position_delta C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vision_position_delta_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_vision_position_delta_t* vision_position_delta)
{
    return mavlink_msg_vision_position_delta_pack_status(system_id, component_id, _status, msg,  vision_position_delta->time_usec, vision_position_delta->time_delta_usec, vision_position_delta->angle_delta, vision_position_delta->position_delta, vision_position_delta->confidence);
}

/**
 * @brief Send a vision_position_delta message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (synced to UNIX time or since system boot).
 * @param time_delta_usec [us] Time since the last reported camera frame.
 * @param angle_delta [rad] Defines a rotation vector [roll, pitch, yaw] to the current MAV_FRAME_BODY_FRD from the previous MAV_FRAME_BODY_FRD.
 * @param position_delta [m] Change in position to the current MAV_FRAME_BODY_FRD from the previous FRAME_BODY_FRD rotated to the current MAV_FRAME_BODY_FRD.
 * @param confidence [%] Normalised confidence value from 0 to 100.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_vision_position_delta_send(mavlink_channel_t chan, uint64_t time_usec, uint64_t time_delta_usec, const float *angle_delta, const float *position_delta, float confidence)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VISION_POSITION_DELTA_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint64_t(buf, 8, time_delta_usec);
    _mav_put_float(buf, 40, confidence);
    _mav_put_float_array(buf, 16, angle_delta, 3);
    _mav_put_float_array(buf, 28, position_delta, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_POSITION_DELTA, buf, MAVLINK_MSG_ID_VISION_POSITION_DELTA_MIN_LEN, MAVLINK_MSG_ID_VISION_POSITION_DELTA_LEN, MAVLINK_MSG_ID_VISION_POSITION_DELTA_CRC);
#else
    mavlink_vision_position_delta_t packet;
    packet.time_usec = time_usec;
    packet.time_delta_usec = time_delta_usec;
    packet.confidence = confidence;
    mav_array_memcpy(packet.angle_delta, angle_delta, sizeof(float)*3);
    mav_array_memcpy(packet.position_delta, position_delta, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_POSITION_DELTA, (const char *)&packet, MAVLINK_MSG_ID_VISION_POSITION_DELTA_MIN_LEN, MAVLINK_MSG_ID_VISION_POSITION_DELTA_LEN, MAVLINK_MSG_ID_VISION_POSITION_DELTA_CRC);
#endif
}

/**
 * @brief Send a vision_position_delta message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_vision_position_delta_send_struct(mavlink_channel_t chan, const mavlink_vision_position_delta_t* vision_position_delta)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_vision_position_delta_send(chan, vision_position_delta->time_usec, vision_position_delta->time_delta_usec, vision_position_delta->angle_delta, vision_position_delta->position_delta, vision_position_delta->confidence);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_POSITION_DELTA, (const char *)vision_position_delta, MAVLINK_MSG_ID_VISION_POSITION_DELTA_MIN_LEN, MAVLINK_MSG_ID_VISION_POSITION_DELTA_LEN, MAVLINK_MSG_ID_VISION_POSITION_DELTA_CRC);
#endif
}

#if MAVLINK_MSG_ID_VISION_POSITION_DELTA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_vision_position_delta_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint64_t time_delta_usec, const float *angle_delta, const float *position_delta, float confidence)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint64_t(buf, 8, time_delta_usec);
    _mav_put_float(buf, 40, confidence);
    _mav_put_float_array(buf, 16, angle_delta, 3);
    _mav_put_float_array(buf, 28, position_delta, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_POSITION_DELTA, buf, MAVLINK_MSG_ID_VISION_POSITION_DELTA_MIN_LEN, MAVLINK_MSG_ID_VISION_POSITION_DELTA_LEN, MAVLINK_MSG_ID_VISION_POSITION_DELTA_CRC);
#else
    mavlink_vision_position_delta_t *packet = (mavlink_vision_position_delta_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->time_delta_usec = time_delta_usec;
    packet->confidence = confidence;
    mav_array_memcpy(packet->angle_delta, angle_delta, sizeof(float)*3);
    mav_array_memcpy(packet->position_delta, position_delta, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_POSITION_DELTA, (const char *)packet, MAVLINK_MSG_ID_VISION_POSITION_DELTA_MIN_LEN, MAVLINK_MSG_ID_VISION_POSITION_DELTA_LEN, MAVLINK_MSG_ID_VISION_POSITION_DELTA_CRC);
#endif
}
#endif

#endif

// MESSAGE VISION_POSITION_DELTA UNPACKING


/**
 * @brief Get field time_usec from vision_position_delta message
 *
 * @return [us] Timestamp (synced to UNIX time or since system boot).
 */
static inline uint64_t mavlink_msg_vision_position_delta_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field time_delta_usec from vision_position_delta message
 *
 * @return [us] Time since the last reported camera frame.
 */
static inline uint64_t mavlink_msg_vision_position_delta_get_time_delta_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field angle_delta from vision_position_delta message
 *
 * @return [rad] Defines a rotation vector [roll, pitch, yaw] to the current MAV_FRAME_BODY_FRD from the previous MAV_FRAME_BODY_FRD.
 */
static inline uint16_t mavlink_msg_vision_position_delta_get_angle_delta(const mavlink_message_t* msg, float *angle_delta)
{
    return _MAV_RETURN_float_array(msg, angle_delta, 3,  16);
}

/**
 * @brief Get field position_delta from vision_position_delta message
 *
 * @return [m] Change in position to the current MAV_FRAME_BODY_FRD from the previous FRAME_BODY_FRD rotated to the current MAV_FRAME_BODY_FRD.
 */
static inline uint16_t mavlink_msg_vision_position_delta_get_position_delta(const mavlink_message_t* msg, float *position_delta)
{
    return _MAV_RETURN_float_array(msg, position_delta, 3,  28);
}

/**
 * @brief Get field confidence from vision_position_delta message
 *
 * @return [%] Normalised confidence value from 0 to 100.
 */
static inline float mavlink_msg_vision_position_delta_get_confidence(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Decode a vision_position_delta message into a struct
 *
 * @param msg The message to decode
 * @param vision_position_delta C-struct to decode the message contents into
 */
static inline void mavlink_msg_vision_position_delta_decode(const mavlink_message_t* msg, mavlink_vision_position_delta_t* vision_position_delta)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    vision_position_delta->time_usec = mavlink_msg_vision_position_delta_get_time_usec(msg);
    vision_position_delta->time_delta_usec = mavlink_msg_vision_position_delta_get_time_delta_usec(msg);
    mavlink_msg_vision_position_delta_get_angle_delta(msg, vision_position_delta->angle_delta);
    mavlink_msg_vision_position_delta_get_position_delta(msg, vision_position_delta->position_delta);
    vision_position_delta->confidence = mavlink_msg_vision_position_delta_get_confidence(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_VISION_POSITION_DELTA_LEN? msg->len : MAVLINK_MSG_ID_VISION_POSITION_DELTA_LEN;
        memset(vision_position_delta, 0, MAVLINK_MSG_ID_VISION_POSITION_DELTA_LEN);
    memcpy(vision_position_delta, _MAV_PAYLOAD(msg), len);
#endif
}
