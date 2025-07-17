#pragma once
// MESSAGE DEBUG_VECT PACKING

#define MAVLINK_MSG_ID_DEBUG_VECT 250


typedef struct __mavlink_debug_vect_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 float x; /*<  x*/
 float y; /*<  y*/
 float z; /*<  z*/
 char name[10]; /*<  Name*/
} mavlink_debug_vect_t;

#define MAVLINK_MSG_ID_DEBUG_VECT_LEN 30
#define MAVLINK_MSG_ID_DEBUG_VECT_MIN_LEN 30
#define MAVLINK_MSG_ID_250_LEN 30
#define MAVLINK_MSG_ID_250_MIN_LEN 30

#define MAVLINK_MSG_ID_DEBUG_VECT_CRC 49
#define MAVLINK_MSG_ID_250_CRC 49

#define MAVLINK_MSG_DEBUG_VECT_FIELD_NAME_LEN 10

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DEBUG_VECT { \
    250, \
    "DEBUG_VECT", \
    5, \
    {  { "name", NULL, MAVLINK_TYPE_CHAR, 10, 20, offsetof(mavlink_debug_vect_t, name) }, \
         { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_debug_vect_t, time_usec) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_debug_vect_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_debug_vect_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_debug_vect_t, z) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DEBUG_VECT { \
    "DEBUG_VECT", \
    5, \
    {  { "name", NULL, MAVLINK_TYPE_CHAR, 10, 20, offsetof(mavlink_debug_vect_t, name) }, \
         { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_debug_vect_t, time_usec) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_debug_vect_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_debug_vect_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_debug_vect_t, z) }, \
         } \
}
#endif

/**
 * @brief Pack a debug_vect message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param name  Name
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param x  x
 * @param y  y
 * @param z  z
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_debug_vect_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const char *name, uint64_t time_usec, float x, float y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DEBUG_VECT_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_char_array(buf, 20, name, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DEBUG_VECT_LEN);
#else
    mavlink_debug_vect_t packet;
    packet.time_usec = time_usec;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    mav_array_memcpy(packet.name, name, sizeof(char)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DEBUG_VECT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DEBUG_VECT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DEBUG_VECT_MIN_LEN, MAVLINK_MSG_ID_DEBUG_VECT_LEN, MAVLINK_MSG_ID_DEBUG_VECT_CRC);
}

/**
 * @brief Pack a debug_vect message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param name  Name
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param x  x
 * @param y  y
 * @param z  z
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_debug_vect_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               const char *name, uint64_t time_usec, float x, float y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DEBUG_VECT_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_char_array(buf, 20, name, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DEBUG_VECT_LEN);
#else
    mavlink_debug_vect_t packet;
    packet.time_usec = time_usec;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    mav_array_memcpy(packet.name, name, sizeof(char)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DEBUG_VECT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DEBUG_VECT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DEBUG_VECT_MIN_LEN, MAVLINK_MSG_ID_DEBUG_VECT_LEN, MAVLINK_MSG_ID_DEBUG_VECT_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DEBUG_VECT_MIN_LEN, MAVLINK_MSG_ID_DEBUG_VECT_LEN);
#endif
}

/**
 * @brief Pack a debug_vect message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param name  Name
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param x  x
 * @param y  y
 * @param z  z
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_debug_vect_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const char *name,uint64_t time_usec,float x,float y,float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DEBUG_VECT_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_char_array(buf, 20, name, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DEBUG_VECT_LEN);
#else
    mavlink_debug_vect_t packet;
    packet.time_usec = time_usec;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    mav_array_memcpy(packet.name, name, sizeof(char)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DEBUG_VECT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DEBUG_VECT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DEBUG_VECT_MIN_LEN, MAVLINK_MSG_ID_DEBUG_VECT_LEN, MAVLINK_MSG_ID_DEBUG_VECT_CRC);
}

/**
 * @brief Encode a debug_vect struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param debug_vect C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_debug_vect_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_debug_vect_t* debug_vect)
{
    return mavlink_msg_debug_vect_pack(system_id, component_id, msg, debug_vect->name, debug_vect->time_usec, debug_vect->x, debug_vect->y, debug_vect->z);
}

/**
 * @brief Encode a debug_vect struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param debug_vect C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_debug_vect_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_debug_vect_t* debug_vect)
{
    return mavlink_msg_debug_vect_pack_chan(system_id, component_id, chan, msg, debug_vect->name, debug_vect->time_usec, debug_vect->x, debug_vect->y, debug_vect->z);
}

/**
 * @brief Encode a debug_vect struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param debug_vect C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_debug_vect_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_debug_vect_t* debug_vect)
{
    return mavlink_msg_debug_vect_pack_status(system_id, component_id, _status, msg,  debug_vect->name, debug_vect->time_usec, debug_vect->x, debug_vect->y, debug_vect->z);
}

/**
 * @brief Send a debug_vect message
 * @param chan MAVLink channel to send the message
 *
 * @param name  Name
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param x  x
 * @param y  y
 * @param z  z
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_debug_vect_send(mavlink_channel_t chan, const char *name, uint64_t time_usec, float x, float y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DEBUG_VECT_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_char_array(buf, 20, name, 10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG_VECT, buf, MAVLINK_MSG_ID_DEBUG_VECT_MIN_LEN, MAVLINK_MSG_ID_DEBUG_VECT_LEN, MAVLINK_MSG_ID_DEBUG_VECT_CRC);
#else
    mavlink_debug_vect_t packet;
    packet.time_usec = time_usec;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    mav_array_memcpy(packet.name, name, sizeof(char)*10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG_VECT, (const char *)&packet, MAVLINK_MSG_ID_DEBUG_VECT_MIN_LEN, MAVLINK_MSG_ID_DEBUG_VECT_LEN, MAVLINK_MSG_ID_DEBUG_VECT_CRC);
#endif
}

/**
 * @brief Send a debug_vect message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_debug_vect_send_struct(mavlink_channel_t chan, const mavlink_debug_vect_t* debug_vect)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_debug_vect_send(chan, debug_vect->name, debug_vect->time_usec, debug_vect->x, debug_vect->y, debug_vect->z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG_VECT, (const char *)debug_vect, MAVLINK_MSG_ID_DEBUG_VECT_MIN_LEN, MAVLINK_MSG_ID_DEBUG_VECT_LEN, MAVLINK_MSG_ID_DEBUG_VECT_CRC);
#endif
}

#if MAVLINK_MSG_ID_DEBUG_VECT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_debug_vect_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const char *name, uint64_t time_usec, float x, float y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_char_array(buf, 20, name, 10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG_VECT, buf, MAVLINK_MSG_ID_DEBUG_VECT_MIN_LEN, MAVLINK_MSG_ID_DEBUG_VECT_LEN, MAVLINK_MSG_ID_DEBUG_VECT_CRC);
#else
    mavlink_debug_vect_t *packet = (mavlink_debug_vect_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    mav_array_memcpy(packet->name, name, sizeof(char)*10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG_VECT, (const char *)packet, MAVLINK_MSG_ID_DEBUG_VECT_MIN_LEN, MAVLINK_MSG_ID_DEBUG_VECT_LEN, MAVLINK_MSG_ID_DEBUG_VECT_CRC);
#endif
}
#endif

#endif

// MESSAGE DEBUG_VECT UNPACKING


/**
 * @brief Get field name from debug_vect message
 *
 * @return  Name
 */
static inline uint16_t mavlink_msg_debug_vect_get_name(const mavlink_message_t* msg, char *name)
{
    return _MAV_RETURN_char_array(msg, name, 10,  20);
}

/**
 * @brief Get field time_usec from debug_vect message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
static inline uint64_t mavlink_msg_debug_vect_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field x from debug_vect message
 *
 * @return  x
 */
static inline float mavlink_msg_debug_vect_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field y from debug_vect message
 *
 * @return  y
 */
static inline float mavlink_msg_debug_vect_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field z from debug_vect message
 *
 * @return  z
 */
static inline float mavlink_msg_debug_vect_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Decode a debug_vect message into a struct
 *
 * @param msg The message to decode
 * @param debug_vect C-struct to decode the message contents into
 */
static inline void mavlink_msg_debug_vect_decode(const mavlink_message_t* msg, mavlink_debug_vect_t* debug_vect)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    debug_vect->time_usec = mavlink_msg_debug_vect_get_time_usec(msg);
    debug_vect->x = mavlink_msg_debug_vect_get_x(msg);
    debug_vect->y = mavlink_msg_debug_vect_get_y(msg);
    debug_vect->z = mavlink_msg_debug_vect_get_z(msg);
    mavlink_msg_debug_vect_get_name(msg, debug_vect->name);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DEBUG_VECT_LEN? msg->len : MAVLINK_MSG_ID_DEBUG_VECT_LEN;
        memset(debug_vect, 0, MAVLINK_MSG_ID_DEBUG_VECT_LEN);
    memcpy(debug_vect, _MAV_PAYLOAD(msg), len);
#endif
}
