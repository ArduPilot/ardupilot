#pragma once
// MESSAGE OBSTACLE_DISTANCE_3D PACKING

#define MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D 11037


typedef struct __mavlink_obstacle_distance_3d_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 float x; /*< [m]  X position of the obstacle.*/
 float y; /*< [m]  Y position of the obstacle.*/
 float z; /*< [m]  Z position of the obstacle.*/
 float min_distance; /*< [m] Minimum distance the sensor can measure.*/
 float max_distance; /*< [m] Maximum distance the sensor can measure.*/
 uint16_t obstacle_id; /*<   Unique ID given to each obstacle so that its movement can be tracked. Use UINT16_MAX if object ID is unknown or cannot be determined.*/
 uint8_t sensor_type; /*<  Class id of the distance sensor type.*/
 uint8_t frame; /*<  Coordinate frame of reference.*/
} mavlink_obstacle_distance_3d_t;

#define MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_LEN 28
#define MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_MIN_LEN 28
#define MAVLINK_MSG_ID_11037_LEN 28
#define MAVLINK_MSG_ID_11037_MIN_LEN 28

#define MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_CRC 130
#define MAVLINK_MSG_ID_11037_CRC 130



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_OBSTACLE_DISTANCE_3D { \
    11037, \
    "OBSTACLE_DISTANCE_3D", \
    9, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_obstacle_distance_3d_t, time_boot_ms) }, \
         { "sensor_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_obstacle_distance_3d_t, sensor_type) }, \
         { "frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_obstacle_distance_3d_t, frame) }, \
         { "obstacle_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_obstacle_distance_3d_t, obstacle_id) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_obstacle_distance_3d_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_obstacle_distance_3d_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_obstacle_distance_3d_t, z) }, \
         { "min_distance", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_obstacle_distance_3d_t, min_distance) }, \
         { "max_distance", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_obstacle_distance_3d_t, max_distance) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_OBSTACLE_DISTANCE_3D { \
    "OBSTACLE_DISTANCE_3D", \
    9, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_obstacle_distance_3d_t, time_boot_ms) }, \
         { "sensor_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_obstacle_distance_3d_t, sensor_type) }, \
         { "frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_obstacle_distance_3d_t, frame) }, \
         { "obstacle_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_obstacle_distance_3d_t, obstacle_id) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_obstacle_distance_3d_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_obstacle_distance_3d_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_obstacle_distance_3d_t, z) }, \
         { "min_distance", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_obstacle_distance_3d_t, min_distance) }, \
         { "max_distance", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_obstacle_distance_3d_t, max_distance) }, \
         } \
}
#endif

/**
 * @brief Pack a obstacle_distance_3d message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param sensor_type  Class id of the distance sensor type.
 * @param frame  Coordinate frame of reference.
 * @param obstacle_id   Unique ID given to each obstacle so that its movement can be tracked. Use UINT16_MAX if object ID is unknown or cannot be determined.
 * @param x [m]  X position of the obstacle.
 * @param y [m]  Y position of the obstacle.
 * @param z [m]  Z position of the obstacle.
 * @param min_distance [m] Minimum distance the sensor can measure.
 * @param max_distance [m] Maximum distance the sensor can measure.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_obstacle_distance_3d_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint8_t sensor_type, uint8_t frame, uint16_t obstacle_id, float x, float y, float z, float min_distance, float max_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, x);
    _mav_put_float(buf, 8, y);
    _mav_put_float(buf, 12, z);
    _mav_put_float(buf, 16, min_distance);
    _mav_put_float(buf, 20, max_distance);
    _mav_put_uint16_t(buf, 24, obstacle_id);
    _mav_put_uint8_t(buf, 26, sensor_type);
    _mav_put_uint8_t(buf, 27, frame);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_LEN);
#else
    mavlink_obstacle_distance_3d_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.min_distance = min_distance;
    packet.max_distance = max_distance;
    packet.obstacle_id = obstacle_id;
    packet.sensor_type = sensor_type;
    packet.frame = frame;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_MIN_LEN, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_LEN, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_CRC);
}

/**
 * @brief Pack a obstacle_distance_3d message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param sensor_type  Class id of the distance sensor type.
 * @param frame  Coordinate frame of reference.
 * @param obstacle_id   Unique ID given to each obstacle so that its movement can be tracked. Use UINT16_MAX if object ID is unknown or cannot be determined.
 * @param x [m]  X position of the obstacle.
 * @param y [m]  Y position of the obstacle.
 * @param z [m]  Z position of the obstacle.
 * @param min_distance [m] Minimum distance the sensor can measure.
 * @param max_distance [m] Maximum distance the sensor can measure.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_obstacle_distance_3d_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint8_t sensor_type, uint8_t frame, uint16_t obstacle_id, float x, float y, float z, float min_distance, float max_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, x);
    _mav_put_float(buf, 8, y);
    _mav_put_float(buf, 12, z);
    _mav_put_float(buf, 16, min_distance);
    _mav_put_float(buf, 20, max_distance);
    _mav_put_uint16_t(buf, 24, obstacle_id);
    _mav_put_uint8_t(buf, 26, sensor_type);
    _mav_put_uint8_t(buf, 27, frame);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_LEN);
#else
    mavlink_obstacle_distance_3d_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.min_distance = min_distance;
    packet.max_distance = max_distance;
    packet.obstacle_id = obstacle_id;
    packet.sensor_type = sensor_type;
    packet.frame = frame;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_MIN_LEN, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_LEN, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_MIN_LEN, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_LEN);
#endif
}

/**
 * @brief Pack a obstacle_distance_3d message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param sensor_type  Class id of the distance sensor type.
 * @param frame  Coordinate frame of reference.
 * @param obstacle_id   Unique ID given to each obstacle so that its movement can be tracked. Use UINT16_MAX if object ID is unknown or cannot be determined.
 * @param x [m]  X position of the obstacle.
 * @param y [m]  Y position of the obstacle.
 * @param z [m]  Z position of the obstacle.
 * @param min_distance [m] Minimum distance the sensor can measure.
 * @param max_distance [m] Maximum distance the sensor can measure.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_obstacle_distance_3d_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,uint8_t sensor_type,uint8_t frame,uint16_t obstacle_id,float x,float y,float z,float min_distance,float max_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, x);
    _mav_put_float(buf, 8, y);
    _mav_put_float(buf, 12, z);
    _mav_put_float(buf, 16, min_distance);
    _mav_put_float(buf, 20, max_distance);
    _mav_put_uint16_t(buf, 24, obstacle_id);
    _mav_put_uint8_t(buf, 26, sensor_type);
    _mav_put_uint8_t(buf, 27, frame);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_LEN);
#else
    mavlink_obstacle_distance_3d_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.min_distance = min_distance;
    packet.max_distance = max_distance;
    packet.obstacle_id = obstacle_id;
    packet.sensor_type = sensor_type;
    packet.frame = frame;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_MIN_LEN, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_LEN, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_CRC);
}

/**
 * @brief Encode a obstacle_distance_3d struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param obstacle_distance_3d C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_obstacle_distance_3d_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_obstacle_distance_3d_t* obstacle_distance_3d)
{
    return mavlink_msg_obstacle_distance_3d_pack(system_id, component_id, msg, obstacle_distance_3d->time_boot_ms, obstacle_distance_3d->sensor_type, obstacle_distance_3d->frame, obstacle_distance_3d->obstacle_id, obstacle_distance_3d->x, obstacle_distance_3d->y, obstacle_distance_3d->z, obstacle_distance_3d->min_distance, obstacle_distance_3d->max_distance);
}

/**
 * @brief Encode a obstacle_distance_3d struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param obstacle_distance_3d C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_obstacle_distance_3d_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_obstacle_distance_3d_t* obstacle_distance_3d)
{
    return mavlink_msg_obstacle_distance_3d_pack_chan(system_id, component_id, chan, msg, obstacle_distance_3d->time_boot_ms, obstacle_distance_3d->sensor_type, obstacle_distance_3d->frame, obstacle_distance_3d->obstacle_id, obstacle_distance_3d->x, obstacle_distance_3d->y, obstacle_distance_3d->z, obstacle_distance_3d->min_distance, obstacle_distance_3d->max_distance);
}

/**
 * @brief Encode a obstacle_distance_3d struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param obstacle_distance_3d C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_obstacle_distance_3d_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_obstacle_distance_3d_t* obstacle_distance_3d)
{
    return mavlink_msg_obstacle_distance_3d_pack_status(system_id, component_id, _status, msg,  obstacle_distance_3d->time_boot_ms, obstacle_distance_3d->sensor_type, obstacle_distance_3d->frame, obstacle_distance_3d->obstacle_id, obstacle_distance_3d->x, obstacle_distance_3d->y, obstacle_distance_3d->z, obstacle_distance_3d->min_distance, obstacle_distance_3d->max_distance);
}

/**
 * @brief Send a obstacle_distance_3d message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param sensor_type  Class id of the distance sensor type.
 * @param frame  Coordinate frame of reference.
 * @param obstacle_id   Unique ID given to each obstacle so that its movement can be tracked. Use UINT16_MAX if object ID is unknown or cannot be determined.
 * @param x [m]  X position of the obstacle.
 * @param y [m]  Y position of the obstacle.
 * @param z [m]  Z position of the obstacle.
 * @param min_distance [m] Minimum distance the sensor can measure.
 * @param max_distance [m] Maximum distance the sensor can measure.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_obstacle_distance_3d_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t sensor_type, uint8_t frame, uint16_t obstacle_id, float x, float y, float z, float min_distance, float max_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, x);
    _mav_put_float(buf, 8, y);
    _mav_put_float(buf, 12, z);
    _mav_put_float(buf, 16, min_distance);
    _mav_put_float(buf, 20, max_distance);
    _mav_put_uint16_t(buf, 24, obstacle_id);
    _mav_put_uint8_t(buf, 26, sensor_type);
    _mav_put_uint8_t(buf, 27, frame);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D, buf, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_MIN_LEN, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_LEN, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_CRC);
#else
    mavlink_obstacle_distance_3d_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.min_distance = min_distance;
    packet.max_distance = max_distance;
    packet.obstacle_id = obstacle_id;
    packet.sensor_type = sensor_type;
    packet.frame = frame;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D, (const char *)&packet, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_MIN_LEN, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_LEN, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_CRC);
#endif
}

/**
 * @brief Send a obstacle_distance_3d message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_obstacle_distance_3d_send_struct(mavlink_channel_t chan, const mavlink_obstacle_distance_3d_t* obstacle_distance_3d)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_obstacle_distance_3d_send(chan, obstacle_distance_3d->time_boot_ms, obstacle_distance_3d->sensor_type, obstacle_distance_3d->frame, obstacle_distance_3d->obstacle_id, obstacle_distance_3d->x, obstacle_distance_3d->y, obstacle_distance_3d->z, obstacle_distance_3d->min_distance, obstacle_distance_3d->max_distance);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D, (const char *)obstacle_distance_3d, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_MIN_LEN, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_LEN, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_CRC);
#endif
}

#if MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_obstacle_distance_3d_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint8_t sensor_type, uint8_t frame, uint16_t obstacle_id, float x, float y, float z, float min_distance, float max_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, x);
    _mav_put_float(buf, 8, y);
    _mav_put_float(buf, 12, z);
    _mav_put_float(buf, 16, min_distance);
    _mav_put_float(buf, 20, max_distance);
    _mav_put_uint16_t(buf, 24, obstacle_id);
    _mav_put_uint8_t(buf, 26, sensor_type);
    _mav_put_uint8_t(buf, 27, frame);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D, buf, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_MIN_LEN, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_LEN, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_CRC);
#else
    mavlink_obstacle_distance_3d_t *packet = (mavlink_obstacle_distance_3d_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->min_distance = min_distance;
    packet->max_distance = max_distance;
    packet->obstacle_id = obstacle_id;
    packet->sensor_type = sensor_type;
    packet->frame = frame;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D, (const char *)packet, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_MIN_LEN, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_LEN, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_CRC);
#endif
}
#endif

#endif

// MESSAGE OBSTACLE_DISTANCE_3D UNPACKING


/**
 * @brief Get field time_boot_ms from obstacle_distance_3d message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_obstacle_distance_3d_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field sensor_type from obstacle_distance_3d message
 *
 * @return  Class id of the distance sensor type.
 */
static inline uint8_t mavlink_msg_obstacle_distance_3d_get_sensor_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field frame from obstacle_distance_3d message
 *
 * @return  Coordinate frame of reference.
 */
static inline uint8_t mavlink_msg_obstacle_distance_3d_get_frame(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  27);
}

/**
 * @brief Get field obstacle_id from obstacle_distance_3d message
 *
 * @return   Unique ID given to each obstacle so that its movement can be tracked. Use UINT16_MAX if object ID is unknown or cannot be determined.
 */
static inline uint16_t mavlink_msg_obstacle_distance_3d_get_obstacle_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Get field x from obstacle_distance_3d message
 *
 * @return [m]  X position of the obstacle.
 */
static inline float mavlink_msg_obstacle_distance_3d_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field y from obstacle_distance_3d message
 *
 * @return [m]  Y position of the obstacle.
 */
static inline float mavlink_msg_obstacle_distance_3d_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field z from obstacle_distance_3d message
 *
 * @return [m]  Z position of the obstacle.
 */
static inline float mavlink_msg_obstacle_distance_3d_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field min_distance from obstacle_distance_3d message
 *
 * @return [m] Minimum distance the sensor can measure.
 */
static inline float mavlink_msg_obstacle_distance_3d_get_min_distance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field max_distance from obstacle_distance_3d message
 *
 * @return [m] Maximum distance the sensor can measure.
 */
static inline float mavlink_msg_obstacle_distance_3d_get_max_distance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a obstacle_distance_3d message into a struct
 *
 * @param msg The message to decode
 * @param obstacle_distance_3d C-struct to decode the message contents into
 */
static inline void mavlink_msg_obstacle_distance_3d_decode(const mavlink_message_t* msg, mavlink_obstacle_distance_3d_t* obstacle_distance_3d)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    obstacle_distance_3d->time_boot_ms = mavlink_msg_obstacle_distance_3d_get_time_boot_ms(msg);
    obstacle_distance_3d->x = mavlink_msg_obstacle_distance_3d_get_x(msg);
    obstacle_distance_3d->y = mavlink_msg_obstacle_distance_3d_get_y(msg);
    obstacle_distance_3d->z = mavlink_msg_obstacle_distance_3d_get_z(msg);
    obstacle_distance_3d->min_distance = mavlink_msg_obstacle_distance_3d_get_min_distance(msg);
    obstacle_distance_3d->max_distance = mavlink_msg_obstacle_distance_3d_get_max_distance(msg);
    obstacle_distance_3d->obstacle_id = mavlink_msg_obstacle_distance_3d_get_obstacle_id(msg);
    obstacle_distance_3d->sensor_type = mavlink_msg_obstacle_distance_3d_get_sensor_type(msg);
    obstacle_distance_3d->frame = mavlink_msg_obstacle_distance_3d_get_frame(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_LEN? msg->len : MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_LEN;
        memset(obstacle_distance_3d, 0, MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_LEN);
    memcpy(obstacle_distance_3d, _MAV_PAYLOAD(msg), len);
#endif
}
