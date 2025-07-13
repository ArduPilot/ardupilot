#pragma once
// MESSAGE LOCAL_POSITION_NED PACKING

#define MAVLINK_MSG_ID_LOCAL_POSITION_NED 32


typedef struct __mavlink_local_position_ned_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 float x; /*< [m] X Position*/
 float y; /*< [m] Y Position*/
 float z; /*< [m] Z Position*/
 float vx; /*< [m/s] X Speed*/
 float vy; /*< [m/s] Y Speed*/
 float vz; /*< [m/s] Z Speed*/
} mavlink_local_position_ned_t;

#define MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN 28
#define MAVLINK_MSG_ID_LOCAL_POSITION_NED_MIN_LEN 28
#define MAVLINK_MSG_ID_32_LEN 28
#define MAVLINK_MSG_ID_32_MIN_LEN 28

#define MAVLINK_MSG_ID_LOCAL_POSITION_NED_CRC 185
#define MAVLINK_MSG_ID_32_CRC 185



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LOCAL_POSITION_NED { \
    32, \
    "LOCAL_POSITION_NED", \
    7, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_local_position_ned_t, time_boot_ms) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_local_position_ned_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_local_position_ned_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_local_position_ned_t, z) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_local_position_ned_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_local_position_ned_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_local_position_ned_t, vz) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LOCAL_POSITION_NED { \
    "LOCAL_POSITION_NED", \
    7, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_local_position_ned_t, time_boot_ms) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_local_position_ned_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_local_position_ned_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_local_position_ned_t, z) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_local_position_ned_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_local_position_ned_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_local_position_ned_t, vz) }, \
         } \
}
#endif

/**
 * @brief Pack a local_position_ned message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param x [m] X Position
 * @param y [m] Y Position
 * @param z [m] Z Position
 * @param vx [m/s] X Speed
 * @param vy [m/s] Y Speed
 * @param vz [m/s] Z Speed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_local_position_ned_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, float x, float y, float z, float vx, float vy, float vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, x);
    _mav_put_float(buf, 8, y);
    _mav_put_float(buf, 12, z);
    _mav_put_float(buf, 16, vx);
    _mav_put_float(buf, 20, vy);
    _mav_put_float(buf, 24, vz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN);
#else
    mavlink_local_position_ned_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LOCAL_POSITION_NED;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LOCAL_POSITION_NED_MIN_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_CRC);
}

/**
 * @brief Pack a local_position_ned message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param x [m] X Position
 * @param y [m] Y Position
 * @param z [m] Z Position
 * @param vx [m/s] X Speed
 * @param vy [m/s] Y Speed
 * @param vz [m/s] Z Speed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_local_position_ned_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t time_boot_ms, float x, float y, float z, float vx, float vy, float vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, x);
    _mav_put_float(buf, 8, y);
    _mav_put_float(buf, 12, z);
    _mav_put_float(buf, 16, vx);
    _mav_put_float(buf, 20, vy);
    _mav_put_float(buf, 24, vz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN);
#else
    mavlink_local_position_ned_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LOCAL_POSITION_NED;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_LOCAL_POSITION_NED_MIN_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_LOCAL_POSITION_NED_MIN_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN);
#endif
}

/**
 * @brief Pack a local_position_ned message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param x [m] X Position
 * @param y [m] Y Position
 * @param z [m] Z Position
 * @param vx [m/s] X Speed
 * @param vy [m/s] Y Speed
 * @param vz [m/s] Z Speed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_local_position_ned_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,float x,float y,float z,float vx,float vy,float vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, x);
    _mav_put_float(buf, 8, y);
    _mav_put_float(buf, 12, z);
    _mav_put_float(buf, 16, vx);
    _mav_put_float(buf, 20, vy);
    _mav_put_float(buf, 24, vz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN);
#else
    mavlink_local_position_ned_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LOCAL_POSITION_NED;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LOCAL_POSITION_NED_MIN_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_CRC);
}

/**
 * @brief Encode a local_position_ned struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param local_position_ned C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_local_position_ned_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_local_position_ned_t* local_position_ned)
{
    return mavlink_msg_local_position_ned_pack(system_id, component_id, msg, local_position_ned->time_boot_ms, local_position_ned->x, local_position_ned->y, local_position_ned->z, local_position_ned->vx, local_position_ned->vy, local_position_ned->vz);
}

/**
 * @brief Encode a local_position_ned struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param local_position_ned C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_local_position_ned_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_local_position_ned_t* local_position_ned)
{
    return mavlink_msg_local_position_ned_pack_chan(system_id, component_id, chan, msg, local_position_ned->time_boot_ms, local_position_ned->x, local_position_ned->y, local_position_ned->z, local_position_ned->vx, local_position_ned->vy, local_position_ned->vz);
}

/**
 * @brief Encode a local_position_ned struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param local_position_ned C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_local_position_ned_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_local_position_ned_t* local_position_ned)
{
    return mavlink_msg_local_position_ned_pack_status(system_id, component_id, _status, msg,  local_position_ned->time_boot_ms, local_position_ned->x, local_position_ned->y, local_position_ned->z, local_position_ned->vx, local_position_ned->vy, local_position_ned->vz);
}

/**
 * @brief Send a local_position_ned message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param x [m] X Position
 * @param y [m] Y Position
 * @param z [m] Z Position
 * @param vx [m/s] X Speed
 * @param vy [m/s] Y Speed
 * @param vz [m/s] Z Speed
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_local_position_ned_send(mavlink_channel_t chan, uint32_t time_boot_ms, float x, float y, float z, float vx, float vy, float vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, x);
    _mav_put_float(buf, 8, y);
    _mav_put_float(buf, 12, z);
    _mav_put_float(buf, 16, vx);
    _mav_put_float(buf, 20, vy);
    _mav_put_float(buf, 24, vz);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOCAL_POSITION_NED, buf, MAVLINK_MSG_ID_LOCAL_POSITION_NED_MIN_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_CRC);
#else
    mavlink_local_position_ned_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOCAL_POSITION_NED, (const char *)&packet, MAVLINK_MSG_ID_LOCAL_POSITION_NED_MIN_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_CRC);
#endif
}

/**
 * @brief Send a local_position_ned message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_local_position_ned_send_struct(mavlink_channel_t chan, const mavlink_local_position_ned_t* local_position_ned)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_local_position_ned_send(chan, local_position_ned->time_boot_ms, local_position_ned->x, local_position_ned->y, local_position_ned->z, local_position_ned->vx, local_position_ned->vy, local_position_ned->vz);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOCAL_POSITION_NED, (const char *)local_position_ned, MAVLINK_MSG_ID_LOCAL_POSITION_NED_MIN_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_CRC);
#endif
}

#if MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_local_position_ned_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, float x, float y, float z, float vx, float vy, float vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, x);
    _mav_put_float(buf, 8, y);
    _mav_put_float(buf, 12, z);
    _mav_put_float(buf, 16, vx);
    _mav_put_float(buf, 20, vy);
    _mav_put_float(buf, 24, vz);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOCAL_POSITION_NED, buf, MAVLINK_MSG_ID_LOCAL_POSITION_NED_MIN_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_CRC);
#else
    mavlink_local_position_ned_t *packet = (mavlink_local_position_ned_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->vx = vx;
    packet->vy = vy;
    packet->vz = vz;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOCAL_POSITION_NED, (const char *)packet, MAVLINK_MSG_ID_LOCAL_POSITION_NED_MIN_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_CRC);
#endif
}
#endif

#endif

// MESSAGE LOCAL_POSITION_NED UNPACKING


/**
 * @brief Get field time_boot_ms from local_position_ned message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_local_position_ned_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field x from local_position_ned message
 *
 * @return [m] X Position
 */
static inline float mavlink_msg_local_position_ned_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field y from local_position_ned message
 *
 * @return [m] Y Position
 */
static inline float mavlink_msg_local_position_ned_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field z from local_position_ned message
 *
 * @return [m] Z Position
 */
static inline float mavlink_msg_local_position_ned_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field vx from local_position_ned message
 *
 * @return [m/s] X Speed
 */
static inline float mavlink_msg_local_position_ned_get_vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field vy from local_position_ned message
 *
 * @return [m/s] Y Speed
 */
static inline float mavlink_msg_local_position_ned_get_vy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field vz from local_position_ned message
 *
 * @return [m/s] Z Speed
 */
static inline float mavlink_msg_local_position_ned_get_vz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a local_position_ned message into a struct
 *
 * @param msg The message to decode
 * @param local_position_ned C-struct to decode the message contents into
 */
static inline void mavlink_msg_local_position_ned_decode(const mavlink_message_t* msg, mavlink_local_position_ned_t* local_position_ned)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    local_position_ned->time_boot_ms = mavlink_msg_local_position_ned_get_time_boot_ms(msg);
    local_position_ned->x = mavlink_msg_local_position_ned_get_x(msg);
    local_position_ned->y = mavlink_msg_local_position_ned_get_y(msg);
    local_position_ned->z = mavlink_msg_local_position_ned_get_z(msg);
    local_position_ned->vx = mavlink_msg_local_position_ned_get_vx(msg);
    local_position_ned->vy = mavlink_msg_local_position_ned_get_vy(msg);
    local_position_ned->vz = mavlink_msg_local_position_ned_get_vz(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN? msg->len : MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN;
        memset(local_position_ned, 0, MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN);
    memcpy(local_position_ned, _MAV_PAYLOAD(msg), len);
#endif
}
