#pragma once
// MESSAGE LANDING_TARGET PACKING

#define MAVLINK_MSG_ID_LANDING_TARGET 149

MAVPACKED(
typedef struct __mavlink_landing_target_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 float angle_x; /*< [rad] X-axis angular offset of the target from the center of the image*/
 float angle_y; /*< [rad] Y-axis angular offset of the target from the center of the image*/
 float distance; /*< [m] Distance to the target from the vehicle*/
 float size_x; /*< [rad] Size of target along x-axis*/
 float size_y; /*< [rad] Size of target along y-axis*/
 uint8_t target_num; /*<  The ID of the target if multiple targets are present*/
 uint8_t frame; /*<  Coordinate frame used for following fields.*/
 float x; /*< [m] X Position of the landing target in MAV_FRAME*/
 float y; /*< [m] Y Position of the landing target in MAV_FRAME*/
 float z; /*< [m] Z Position of the landing target in MAV_FRAME*/
 float q[4]; /*<  Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)*/
 uint8_t type; /*<  Type of landing target*/
 uint8_t position_valid; /*<  Boolean indicating whether the position fields (x, y, z, q, type) contain valid target position information (valid: 1, invalid: 0). Default is 0 (invalid).*/
}) mavlink_landing_target_t;

#define MAVLINK_MSG_ID_LANDING_TARGET_LEN 60
#define MAVLINK_MSG_ID_LANDING_TARGET_MIN_LEN 30
#define MAVLINK_MSG_ID_149_LEN 60
#define MAVLINK_MSG_ID_149_MIN_LEN 30

#define MAVLINK_MSG_ID_LANDING_TARGET_CRC 200
#define MAVLINK_MSG_ID_149_CRC 200

#define MAVLINK_MSG_LANDING_TARGET_FIELD_Q_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LANDING_TARGET { \
    149, \
    "LANDING_TARGET", \
    14, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_landing_target_t, time_usec) }, \
         { "target_num", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_landing_target_t, target_num) }, \
         { "frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_landing_target_t, frame) }, \
         { "angle_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_landing_target_t, angle_x) }, \
         { "angle_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_landing_target_t, angle_y) }, \
         { "distance", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_landing_target_t, distance) }, \
         { "size_x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_landing_target_t, size_x) }, \
         { "size_y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_landing_target_t, size_y) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 30, offsetof(mavlink_landing_target_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 34, offsetof(mavlink_landing_target_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 38, offsetof(mavlink_landing_target_t, z) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 4, 42, offsetof(mavlink_landing_target_t, q) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 58, offsetof(mavlink_landing_target_t, type) }, \
         { "position_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 59, offsetof(mavlink_landing_target_t, position_valid) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LANDING_TARGET { \
    "LANDING_TARGET", \
    14, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_landing_target_t, time_usec) }, \
         { "target_num", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_landing_target_t, target_num) }, \
         { "frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_landing_target_t, frame) }, \
         { "angle_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_landing_target_t, angle_x) }, \
         { "angle_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_landing_target_t, angle_y) }, \
         { "distance", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_landing_target_t, distance) }, \
         { "size_x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_landing_target_t, size_x) }, \
         { "size_y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_landing_target_t, size_y) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 30, offsetof(mavlink_landing_target_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 34, offsetof(mavlink_landing_target_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 38, offsetof(mavlink_landing_target_t, z) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 4, 42, offsetof(mavlink_landing_target_t, q) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 58, offsetof(mavlink_landing_target_t, type) }, \
         { "position_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 59, offsetof(mavlink_landing_target_t, position_valid) }, \
         } \
}
#endif

/**
 * @brief Pack a landing_target message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param target_num  The ID of the target if multiple targets are present
 * @param frame  Coordinate frame used for following fields.
 * @param angle_x [rad] X-axis angular offset of the target from the center of the image
 * @param angle_y [rad] Y-axis angular offset of the target from the center of the image
 * @param distance [m] Distance to the target from the vehicle
 * @param size_x [rad] Size of target along x-axis
 * @param size_y [rad] Size of target along y-axis
 * @param x [m] X Position of the landing target in MAV_FRAME
 * @param y [m] Y Position of the landing target in MAV_FRAME
 * @param z [m] Z Position of the landing target in MAV_FRAME
 * @param q  Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
 * @param type  Type of landing target
 * @param position_valid  Boolean indicating whether the position fields (x, y, z, q, type) contain valid target position information (valid: 1, invalid: 0). Default is 0 (invalid).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_landing_target_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, uint8_t target_num, uint8_t frame, float angle_x, float angle_y, float distance, float size_x, float size_y, float x, float y, float z, const float *q, uint8_t type, uint8_t position_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LANDING_TARGET_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, angle_x);
    _mav_put_float(buf, 12, angle_y);
    _mav_put_float(buf, 16, distance);
    _mav_put_float(buf, 20, size_x);
    _mav_put_float(buf, 24, size_y);
    _mav_put_uint8_t(buf, 28, target_num);
    _mav_put_uint8_t(buf, 29, frame);
    _mav_put_float(buf, 30, x);
    _mav_put_float(buf, 34, y);
    _mav_put_float(buf, 38, z);
    _mav_put_uint8_t(buf, 58, type);
    _mav_put_uint8_t(buf, 59, position_valid);
    _mav_put_float_array(buf, 42, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LANDING_TARGET_LEN);
#else
    mavlink_landing_target_t packet;
    packet.time_usec = time_usec;
    packet.angle_x = angle_x;
    packet.angle_y = angle_y;
    packet.distance = distance;
    packet.size_x = size_x;
    packet.size_y = size_y;
    packet.target_num = target_num;
    packet.frame = frame;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.type = type;
    packet.position_valid = position_valid;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LANDING_TARGET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LANDING_TARGET;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LANDING_TARGET_MIN_LEN, MAVLINK_MSG_ID_LANDING_TARGET_LEN, MAVLINK_MSG_ID_LANDING_TARGET_CRC);
}

/**
 * @brief Pack a landing_target message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param target_num  The ID of the target if multiple targets are present
 * @param frame  Coordinate frame used for following fields.
 * @param angle_x [rad] X-axis angular offset of the target from the center of the image
 * @param angle_y [rad] Y-axis angular offset of the target from the center of the image
 * @param distance [m] Distance to the target from the vehicle
 * @param size_x [rad] Size of target along x-axis
 * @param size_y [rad] Size of target along y-axis
 * @param x [m] X Position of the landing target in MAV_FRAME
 * @param y [m] Y Position of the landing target in MAV_FRAME
 * @param z [m] Z Position of the landing target in MAV_FRAME
 * @param q  Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
 * @param type  Type of landing target
 * @param position_valid  Boolean indicating whether the position fields (x, y, z, q, type) contain valid target position information (valid: 1, invalid: 0). Default is 0 (invalid).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_landing_target_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t time_usec, uint8_t target_num, uint8_t frame, float angle_x, float angle_y, float distance, float size_x, float size_y, float x, float y, float z, const float *q, uint8_t type, uint8_t position_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LANDING_TARGET_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, angle_x);
    _mav_put_float(buf, 12, angle_y);
    _mav_put_float(buf, 16, distance);
    _mav_put_float(buf, 20, size_x);
    _mav_put_float(buf, 24, size_y);
    _mav_put_uint8_t(buf, 28, target_num);
    _mav_put_uint8_t(buf, 29, frame);
    _mav_put_float(buf, 30, x);
    _mav_put_float(buf, 34, y);
    _mav_put_float(buf, 38, z);
    _mav_put_uint8_t(buf, 58, type);
    _mav_put_uint8_t(buf, 59, position_valid);
    _mav_put_float_array(buf, 42, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LANDING_TARGET_LEN);
#else
    mavlink_landing_target_t packet;
    packet.time_usec = time_usec;
    packet.angle_x = angle_x;
    packet.angle_y = angle_y;
    packet.distance = distance;
    packet.size_x = size_x;
    packet.size_y = size_y;
    packet.target_num = target_num;
    packet.frame = frame;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.type = type;
    packet.position_valid = position_valid;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LANDING_TARGET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LANDING_TARGET;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_LANDING_TARGET_MIN_LEN, MAVLINK_MSG_ID_LANDING_TARGET_LEN, MAVLINK_MSG_ID_LANDING_TARGET_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_LANDING_TARGET_MIN_LEN, MAVLINK_MSG_ID_LANDING_TARGET_LEN);
#endif
}

/**
 * @brief Pack a landing_target message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param target_num  The ID of the target if multiple targets are present
 * @param frame  Coordinate frame used for following fields.
 * @param angle_x [rad] X-axis angular offset of the target from the center of the image
 * @param angle_y [rad] Y-axis angular offset of the target from the center of the image
 * @param distance [m] Distance to the target from the vehicle
 * @param size_x [rad] Size of target along x-axis
 * @param size_y [rad] Size of target along y-axis
 * @param x [m] X Position of the landing target in MAV_FRAME
 * @param y [m] Y Position of the landing target in MAV_FRAME
 * @param z [m] Z Position of the landing target in MAV_FRAME
 * @param q  Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
 * @param type  Type of landing target
 * @param position_valid  Boolean indicating whether the position fields (x, y, z, q, type) contain valid target position information (valid: 1, invalid: 0). Default is 0 (invalid).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_landing_target_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,uint8_t target_num,uint8_t frame,float angle_x,float angle_y,float distance,float size_x,float size_y,float x,float y,float z,const float *q,uint8_t type,uint8_t position_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LANDING_TARGET_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, angle_x);
    _mav_put_float(buf, 12, angle_y);
    _mav_put_float(buf, 16, distance);
    _mav_put_float(buf, 20, size_x);
    _mav_put_float(buf, 24, size_y);
    _mav_put_uint8_t(buf, 28, target_num);
    _mav_put_uint8_t(buf, 29, frame);
    _mav_put_float(buf, 30, x);
    _mav_put_float(buf, 34, y);
    _mav_put_float(buf, 38, z);
    _mav_put_uint8_t(buf, 58, type);
    _mav_put_uint8_t(buf, 59, position_valid);
    _mav_put_float_array(buf, 42, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LANDING_TARGET_LEN);
#else
    mavlink_landing_target_t packet;
    packet.time_usec = time_usec;
    packet.angle_x = angle_x;
    packet.angle_y = angle_y;
    packet.distance = distance;
    packet.size_x = size_x;
    packet.size_y = size_y;
    packet.target_num = target_num;
    packet.frame = frame;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.type = type;
    packet.position_valid = position_valid;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LANDING_TARGET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LANDING_TARGET;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LANDING_TARGET_MIN_LEN, MAVLINK_MSG_ID_LANDING_TARGET_LEN, MAVLINK_MSG_ID_LANDING_TARGET_CRC);
}

/**
 * @brief Encode a landing_target struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param landing_target C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_landing_target_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_landing_target_t* landing_target)
{
    return mavlink_msg_landing_target_pack(system_id, component_id, msg, landing_target->time_usec, landing_target->target_num, landing_target->frame, landing_target->angle_x, landing_target->angle_y, landing_target->distance, landing_target->size_x, landing_target->size_y, landing_target->x, landing_target->y, landing_target->z, landing_target->q, landing_target->type, landing_target->position_valid);
}

/**
 * @brief Encode a landing_target struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param landing_target C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_landing_target_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_landing_target_t* landing_target)
{
    return mavlink_msg_landing_target_pack_chan(system_id, component_id, chan, msg, landing_target->time_usec, landing_target->target_num, landing_target->frame, landing_target->angle_x, landing_target->angle_y, landing_target->distance, landing_target->size_x, landing_target->size_y, landing_target->x, landing_target->y, landing_target->z, landing_target->q, landing_target->type, landing_target->position_valid);
}

/**
 * @brief Encode a landing_target struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param landing_target C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_landing_target_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_landing_target_t* landing_target)
{
    return mavlink_msg_landing_target_pack_status(system_id, component_id, _status, msg,  landing_target->time_usec, landing_target->target_num, landing_target->frame, landing_target->angle_x, landing_target->angle_y, landing_target->distance, landing_target->size_x, landing_target->size_y, landing_target->x, landing_target->y, landing_target->z, landing_target->q, landing_target->type, landing_target->position_valid);
}

/**
 * @brief Send a landing_target message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param target_num  The ID of the target if multiple targets are present
 * @param frame  Coordinate frame used for following fields.
 * @param angle_x [rad] X-axis angular offset of the target from the center of the image
 * @param angle_y [rad] Y-axis angular offset of the target from the center of the image
 * @param distance [m] Distance to the target from the vehicle
 * @param size_x [rad] Size of target along x-axis
 * @param size_y [rad] Size of target along y-axis
 * @param x [m] X Position of the landing target in MAV_FRAME
 * @param y [m] Y Position of the landing target in MAV_FRAME
 * @param z [m] Z Position of the landing target in MAV_FRAME
 * @param q  Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
 * @param type  Type of landing target
 * @param position_valid  Boolean indicating whether the position fields (x, y, z, q, type) contain valid target position information (valid: 1, invalid: 0). Default is 0 (invalid).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_landing_target_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t target_num, uint8_t frame, float angle_x, float angle_y, float distance, float size_x, float size_y, float x, float y, float z, const float *q, uint8_t type, uint8_t position_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LANDING_TARGET_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, angle_x);
    _mav_put_float(buf, 12, angle_y);
    _mav_put_float(buf, 16, distance);
    _mav_put_float(buf, 20, size_x);
    _mav_put_float(buf, 24, size_y);
    _mav_put_uint8_t(buf, 28, target_num);
    _mav_put_uint8_t(buf, 29, frame);
    _mav_put_float(buf, 30, x);
    _mav_put_float(buf, 34, y);
    _mav_put_float(buf, 38, z);
    _mav_put_uint8_t(buf, 58, type);
    _mav_put_uint8_t(buf, 59, position_valid);
    _mav_put_float_array(buf, 42, q, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LANDING_TARGET, buf, MAVLINK_MSG_ID_LANDING_TARGET_MIN_LEN, MAVLINK_MSG_ID_LANDING_TARGET_LEN, MAVLINK_MSG_ID_LANDING_TARGET_CRC);
#else
    mavlink_landing_target_t packet;
    packet.time_usec = time_usec;
    packet.angle_x = angle_x;
    packet.angle_y = angle_y;
    packet.distance = distance;
    packet.size_x = size_x;
    packet.size_y = size_y;
    packet.target_num = target_num;
    packet.frame = frame;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.type = type;
    packet.position_valid = position_valid;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LANDING_TARGET, (const char *)&packet, MAVLINK_MSG_ID_LANDING_TARGET_MIN_LEN, MAVLINK_MSG_ID_LANDING_TARGET_LEN, MAVLINK_MSG_ID_LANDING_TARGET_CRC);
#endif
}

/**
 * @brief Send a landing_target message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_landing_target_send_struct(mavlink_channel_t chan, const mavlink_landing_target_t* landing_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_landing_target_send(chan, landing_target->time_usec, landing_target->target_num, landing_target->frame, landing_target->angle_x, landing_target->angle_y, landing_target->distance, landing_target->size_x, landing_target->size_y, landing_target->x, landing_target->y, landing_target->z, landing_target->q, landing_target->type, landing_target->position_valid);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LANDING_TARGET, (const char *)landing_target, MAVLINK_MSG_ID_LANDING_TARGET_MIN_LEN, MAVLINK_MSG_ID_LANDING_TARGET_LEN, MAVLINK_MSG_ID_LANDING_TARGET_CRC);
#endif
}

#if MAVLINK_MSG_ID_LANDING_TARGET_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_landing_target_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint8_t target_num, uint8_t frame, float angle_x, float angle_y, float distance, float size_x, float size_y, float x, float y, float z, const float *q, uint8_t type, uint8_t position_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, angle_x);
    _mav_put_float(buf, 12, angle_y);
    _mav_put_float(buf, 16, distance);
    _mav_put_float(buf, 20, size_x);
    _mav_put_float(buf, 24, size_y);
    _mav_put_uint8_t(buf, 28, target_num);
    _mav_put_uint8_t(buf, 29, frame);
    _mav_put_float(buf, 30, x);
    _mav_put_float(buf, 34, y);
    _mav_put_float(buf, 38, z);
    _mav_put_uint8_t(buf, 58, type);
    _mav_put_uint8_t(buf, 59, position_valid);
    _mav_put_float_array(buf, 42, q, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LANDING_TARGET, buf, MAVLINK_MSG_ID_LANDING_TARGET_MIN_LEN, MAVLINK_MSG_ID_LANDING_TARGET_LEN, MAVLINK_MSG_ID_LANDING_TARGET_CRC);
#else
    mavlink_landing_target_t *packet = (mavlink_landing_target_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->angle_x = angle_x;
    packet->angle_y = angle_y;
    packet->distance = distance;
    packet->size_x = size_x;
    packet->size_y = size_y;
    packet->target_num = target_num;
    packet->frame = frame;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->type = type;
    packet->position_valid = position_valid;
    mav_array_memcpy(packet->q, q, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LANDING_TARGET, (const char *)packet, MAVLINK_MSG_ID_LANDING_TARGET_MIN_LEN, MAVLINK_MSG_ID_LANDING_TARGET_LEN, MAVLINK_MSG_ID_LANDING_TARGET_CRC);
#endif
}
#endif

#endif

// MESSAGE LANDING_TARGET UNPACKING


/**
 * @brief Get field time_usec from landing_target message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
static inline uint64_t mavlink_msg_landing_target_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field target_num from landing_target message
 *
 * @return  The ID of the target if multiple targets are present
 */
static inline uint8_t mavlink_msg_landing_target_get_target_num(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field frame from landing_target message
 *
 * @return  Coordinate frame used for following fields.
 */
static inline uint8_t mavlink_msg_landing_target_get_frame(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  29);
}

/**
 * @brief Get field angle_x from landing_target message
 *
 * @return [rad] X-axis angular offset of the target from the center of the image
 */
static inline float mavlink_msg_landing_target_get_angle_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field angle_y from landing_target message
 *
 * @return [rad] Y-axis angular offset of the target from the center of the image
 */
static inline float mavlink_msg_landing_target_get_angle_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field distance from landing_target message
 *
 * @return [m] Distance to the target from the vehicle
 */
static inline float mavlink_msg_landing_target_get_distance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field size_x from landing_target message
 *
 * @return [rad] Size of target along x-axis
 */
static inline float mavlink_msg_landing_target_get_size_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field size_y from landing_target message
 *
 * @return [rad] Size of target along y-axis
 */
static inline float mavlink_msg_landing_target_get_size_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field x from landing_target message
 *
 * @return [m] X Position of the landing target in MAV_FRAME
 */
static inline float mavlink_msg_landing_target_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  30);
}

/**
 * @brief Get field y from landing_target message
 *
 * @return [m] Y Position of the landing target in MAV_FRAME
 */
static inline float mavlink_msg_landing_target_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  34);
}

/**
 * @brief Get field z from landing_target message
 *
 * @return [m] Z Position of the landing target in MAV_FRAME
 */
static inline float mavlink_msg_landing_target_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  38);
}

/**
 * @brief Get field q from landing_target message
 *
 * @return  Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
 */
static inline uint16_t mavlink_msg_landing_target_get_q(const mavlink_message_t* msg, float *q)
{
    return _MAV_RETURN_float_array(msg, q, 4,  42);
}

/**
 * @brief Get field type from landing_target message
 *
 * @return  Type of landing target
 */
static inline uint8_t mavlink_msg_landing_target_get_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  58);
}

/**
 * @brief Get field position_valid from landing_target message
 *
 * @return  Boolean indicating whether the position fields (x, y, z, q, type) contain valid target position information (valid: 1, invalid: 0). Default is 0 (invalid).
 */
static inline uint8_t mavlink_msg_landing_target_get_position_valid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  59);
}

/**
 * @brief Decode a landing_target message into a struct
 *
 * @param msg The message to decode
 * @param landing_target C-struct to decode the message contents into
 */
static inline void mavlink_msg_landing_target_decode(const mavlink_message_t* msg, mavlink_landing_target_t* landing_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    landing_target->time_usec = mavlink_msg_landing_target_get_time_usec(msg);
    landing_target->angle_x = mavlink_msg_landing_target_get_angle_x(msg);
    landing_target->angle_y = mavlink_msg_landing_target_get_angle_y(msg);
    landing_target->distance = mavlink_msg_landing_target_get_distance(msg);
    landing_target->size_x = mavlink_msg_landing_target_get_size_x(msg);
    landing_target->size_y = mavlink_msg_landing_target_get_size_y(msg);
    landing_target->target_num = mavlink_msg_landing_target_get_target_num(msg);
    landing_target->frame = mavlink_msg_landing_target_get_frame(msg);
    landing_target->x = mavlink_msg_landing_target_get_x(msg);
    landing_target->y = mavlink_msg_landing_target_get_y(msg);
    landing_target->z = mavlink_msg_landing_target_get_z(msg);
    mavlink_msg_landing_target_get_q(msg, landing_target->q);
    landing_target->type = mavlink_msg_landing_target_get_type(msg);
    landing_target->position_valid = mavlink_msg_landing_target_get_position_valid(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LANDING_TARGET_LEN? msg->len : MAVLINK_MSG_ID_LANDING_TARGET_LEN;
        memset(landing_target, 0, MAVLINK_MSG_ID_LANDING_TARGET_LEN);
    memcpy(landing_target, _MAV_PAYLOAD(msg), len);
#endif
}
