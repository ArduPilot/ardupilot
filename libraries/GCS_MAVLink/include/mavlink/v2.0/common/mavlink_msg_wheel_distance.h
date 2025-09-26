#pragma once
// MESSAGE WHEEL_DISTANCE PACKING

#define MAVLINK_MSG_ID_WHEEL_DISTANCE 9000


typedef struct __mavlink_wheel_distance_t {
 uint64_t time_usec; /*< [us] Timestamp (synced to UNIX time or since system boot).*/
 double distance[16]; /*< [m] Distance reported by individual wheel encoders. Forward rotations increase values, reverse rotations decrease them. Not all wheels will necessarily have wheel encoders; the mapping of encoders to wheel positions must be agreed/understood by the endpoints.*/
 uint8_t count; /*<  Number of wheels reported.*/
} mavlink_wheel_distance_t;

#define MAVLINK_MSG_ID_WHEEL_DISTANCE_LEN 137
#define MAVLINK_MSG_ID_WHEEL_DISTANCE_MIN_LEN 137
#define MAVLINK_MSG_ID_9000_LEN 137
#define MAVLINK_MSG_ID_9000_MIN_LEN 137

#define MAVLINK_MSG_ID_WHEEL_DISTANCE_CRC 113
#define MAVLINK_MSG_ID_9000_CRC 113

#define MAVLINK_MSG_WHEEL_DISTANCE_FIELD_DISTANCE_LEN 16

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_WHEEL_DISTANCE { \
    9000, \
    "WHEEL_DISTANCE", \
    3, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_wheel_distance_t, time_usec) }, \
         { "count", NULL, MAVLINK_TYPE_UINT8_T, 0, 136, offsetof(mavlink_wheel_distance_t, count) }, \
         { "distance", NULL, MAVLINK_TYPE_DOUBLE, 16, 8, offsetof(mavlink_wheel_distance_t, distance) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_WHEEL_DISTANCE { \
    "WHEEL_DISTANCE", \
    3, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_wheel_distance_t, time_usec) }, \
         { "count", NULL, MAVLINK_TYPE_UINT8_T, 0, 136, offsetof(mavlink_wheel_distance_t, count) }, \
         { "distance", NULL, MAVLINK_TYPE_DOUBLE, 16, 8, offsetof(mavlink_wheel_distance_t, distance) }, \
         } \
}
#endif

/**
 * @brief Pack a wheel_distance message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (synced to UNIX time or since system boot).
 * @param count  Number of wheels reported.
 * @param distance [m] Distance reported by individual wheel encoders. Forward rotations increase values, reverse rotations decrease them. Not all wheels will necessarily have wheel encoders; the mapping of encoders to wheel positions must be agreed/understood by the endpoints.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_wheel_distance_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, uint8_t count, const double *distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WHEEL_DISTANCE_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 136, count);
    _mav_put_double_array(buf, 8, distance, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WHEEL_DISTANCE_LEN);
#else
    mavlink_wheel_distance_t packet;
    packet.time_usec = time_usec;
    packet.count = count;
    mav_array_memcpy(packet.distance, distance, sizeof(double)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WHEEL_DISTANCE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WHEEL_DISTANCE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_WHEEL_DISTANCE_MIN_LEN, MAVLINK_MSG_ID_WHEEL_DISTANCE_LEN, MAVLINK_MSG_ID_WHEEL_DISTANCE_CRC);
}

/**
 * @brief Pack a wheel_distance message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (synced to UNIX time or since system boot).
 * @param count  Number of wheels reported.
 * @param distance [m] Distance reported by individual wheel encoders. Forward rotations increase values, reverse rotations decrease them. Not all wheels will necessarily have wheel encoders; the mapping of encoders to wheel positions must be agreed/understood by the endpoints.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_wheel_distance_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t time_usec, uint8_t count, const double *distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WHEEL_DISTANCE_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 136, count);
    _mav_put_double_array(buf, 8, distance, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WHEEL_DISTANCE_LEN);
#else
    mavlink_wheel_distance_t packet;
    packet.time_usec = time_usec;
    packet.count = count;
    mav_array_memcpy(packet.distance, distance, sizeof(double)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WHEEL_DISTANCE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WHEEL_DISTANCE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_WHEEL_DISTANCE_MIN_LEN, MAVLINK_MSG_ID_WHEEL_DISTANCE_LEN, MAVLINK_MSG_ID_WHEEL_DISTANCE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_WHEEL_DISTANCE_MIN_LEN, MAVLINK_MSG_ID_WHEEL_DISTANCE_LEN);
#endif
}

/**
 * @brief Pack a wheel_distance message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (synced to UNIX time or since system boot).
 * @param count  Number of wheels reported.
 * @param distance [m] Distance reported by individual wheel encoders. Forward rotations increase values, reverse rotations decrease them. Not all wheels will necessarily have wheel encoders; the mapping of encoders to wheel positions must be agreed/understood by the endpoints.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_wheel_distance_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,uint8_t count,const double *distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WHEEL_DISTANCE_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 136, count);
    _mav_put_double_array(buf, 8, distance, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WHEEL_DISTANCE_LEN);
#else
    mavlink_wheel_distance_t packet;
    packet.time_usec = time_usec;
    packet.count = count;
    mav_array_memcpy(packet.distance, distance, sizeof(double)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WHEEL_DISTANCE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WHEEL_DISTANCE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_WHEEL_DISTANCE_MIN_LEN, MAVLINK_MSG_ID_WHEEL_DISTANCE_LEN, MAVLINK_MSG_ID_WHEEL_DISTANCE_CRC);
}

/**
 * @brief Encode a wheel_distance struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param wheel_distance C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_wheel_distance_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_wheel_distance_t* wheel_distance)
{
    return mavlink_msg_wheel_distance_pack(system_id, component_id, msg, wheel_distance->time_usec, wheel_distance->count, wheel_distance->distance);
}

/**
 * @brief Encode a wheel_distance struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param wheel_distance C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_wheel_distance_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_wheel_distance_t* wheel_distance)
{
    return mavlink_msg_wheel_distance_pack_chan(system_id, component_id, chan, msg, wheel_distance->time_usec, wheel_distance->count, wheel_distance->distance);
}

/**
 * @brief Encode a wheel_distance struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param wheel_distance C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_wheel_distance_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_wheel_distance_t* wheel_distance)
{
    return mavlink_msg_wheel_distance_pack_status(system_id, component_id, _status, msg,  wheel_distance->time_usec, wheel_distance->count, wheel_distance->distance);
}

/**
 * @brief Send a wheel_distance message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (synced to UNIX time or since system boot).
 * @param count  Number of wheels reported.
 * @param distance [m] Distance reported by individual wheel encoders. Forward rotations increase values, reverse rotations decrease them. Not all wheels will necessarily have wheel encoders; the mapping of encoders to wheel positions must be agreed/understood by the endpoints.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_wheel_distance_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t count, const double *distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WHEEL_DISTANCE_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 136, count);
    _mav_put_double_array(buf, 8, distance, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WHEEL_DISTANCE, buf, MAVLINK_MSG_ID_WHEEL_DISTANCE_MIN_LEN, MAVLINK_MSG_ID_WHEEL_DISTANCE_LEN, MAVLINK_MSG_ID_WHEEL_DISTANCE_CRC);
#else
    mavlink_wheel_distance_t packet;
    packet.time_usec = time_usec;
    packet.count = count;
    mav_array_memcpy(packet.distance, distance, sizeof(double)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WHEEL_DISTANCE, (const char *)&packet, MAVLINK_MSG_ID_WHEEL_DISTANCE_MIN_LEN, MAVLINK_MSG_ID_WHEEL_DISTANCE_LEN, MAVLINK_MSG_ID_WHEEL_DISTANCE_CRC);
#endif
}

/**
 * @brief Send a wheel_distance message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_wheel_distance_send_struct(mavlink_channel_t chan, const mavlink_wheel_distance_t* wheel_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_wheel_distance_send(chan, wheel_distance->time_usec, wheel_distance->count, wheel_distance->distance);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WHEEL_DISTANCE, (const char *)wheel_distance, MAVLINK_MSG_ID_WHEEL_DISTANCE_MIN_LEN, MAVLINK_MSG_ID_WHEEL_DISTANCE_LEN, MAVLINK_MSG_ID_WHEEL_DISTANCE_CRC);
#endif
}

#if MAVLINK_MSG_ID_WHEEL_DISTANCE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_wheel_distance_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint8_t count, const double *distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 136, count);
    _mav_put_double_array(buf, 8, distance, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WHEEL_DISTANCE, buf, MAVLINK_MSG_ID_WHEEL_DISTANCE_MIN_LEN, MAVLINK_MSG_ID_WHEEL_DISTANCE_LEN, MAVLINK_MSG_ID_WHEEL_DISTANCE_CRC);
#else
    mavlink_wheel_distance_t *packet = (mavlink_wheel_distance_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->count = count;
    mav_array_memcpy(packet->distance, distance, sizeof(double)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WHEEL_DISTANCE, (const char *)packet, MAVLINK_MSG_ID_WHEEL_DISTANCE_MIN_LEN, MAVLINK_MSG_ID_WHEEL_DISTANCE_LEN, MAVLINK_MSG_ID_WHEEL_DISTANCE_CRC);
#endif
}
#endif

#endif

// MESSAGE WHEEL_DISTANCE UNPACKING


/**
 * @brief Get field time_usec from wheel_distance message
 *
 * @return [us] Timestamp (synced to UNIX time or since system boot).
 */
static inline uint64_t mavlink_msg_wheel_distance_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field count from wheel_distance message
 *
 * @return  Number of wheels reported.
 */
static inline uint8_t mavlink_msg_wheel_distance_get_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  136);
}

/**
 * @brief Get field distance from wheel_distance message
 *
 * @return [m] Distance reported by individual wheel encoders. Forward rotations increase values, reverse rotations decrease them. Not all wheels will necessarily have wheel encoders; the mapping of encoders to wheel positions must be agreed/understood by the endpoints.
 */
static inline uint16_t mavlink_msg_wheel_distance_get_distance(const mavlink_message_t* msg, double *distance)
{
    return _MAV_RETURN_double_array(msg, distance, 16,  8);
}

/**
 * @brief Decode a wheel_distance message into a struct
 *
 * @param msg The message to decode
 * @param wheel_distance C-struct to decode the message contents into
 */
static inline void mavlink_msg_wheel_distance_decode(const mavlink_message_t* msg, mavlink_wheel_distance_t* wheel_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    wheel_distance->time_usec = mavlink_msg_wheel_distance_get_time_usec(msg);
    mavlink_msg_wheel_distance_get_distance(msg, wheel_distance->distance);
    wheel_distance->count = mavlink_msg_wheel_distance_get_count(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_WHEEL_DISTANCE_LEN? msg->len : MAVLINK_MSG_ID_WHEEL_DISTANCE_LEN;
        memset(wheel_distance, 0, MAVLINK_MSG_ID_WHEEL_DISTANCE_LEN);
    memcpy(wheel_distance, _MAV_PAYLOAD(msg), len);
#endif
}
