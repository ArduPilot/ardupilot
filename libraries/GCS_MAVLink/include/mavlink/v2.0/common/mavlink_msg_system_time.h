#pragma once
// MESSAGE SYSTEM_TIME PACKING

#define MAVLINK_MSG_ID_SYSTEM_TIME 2


typedef struct __mavlink_system_time_t {
 uint64_t time_unix_usec; /*< [us] Timestamp (UNIX epoch time).*/
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
} mavlink_system_time_t;

#define MAVLINK_MSG_ID_SYSTEM_TIME_LEN 12
#define MAVLINK_MSG_ID_SYSTEM_TIME_MIN_LEN 12
#define MAVLINK_MSG_ID_2_LEN 12
#define MAVLINK_MSG_ID_2_MIN_LEN 12

#define MAVLINK_MSG_ID_SYSTEM_TIME_CRC 137
#define MAVLINK_MSG_ID_2_CRC 137



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SYSTEM_TIME { \
    2, \
    "SYSTEM_TIME", \
    2, \
    {  { "time_unix_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_system_time_t, time_unix_usec) }, \
         { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_system_time_t, time_boot_ms) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SYSTEM_TIME { \
    "SYSTEM_TIME", \
    2, \
    {  { "time_unix_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_system_time_t, time_unix_usec) }, \
         { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_system_time_t, time_boot_ms) }, \
         } \
}
#endif

/**
 * @brief Pack a system_time message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_unix_usec [us] Timestamp (UNIX epoch time).
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_system_time_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_unix_usec, uint32_t time_boot_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SYSTEM_TIME_LEN];
    _mav_put_uint64_t(buf, 0, time_unix_usec);
    _mav_put_uint32_t(buf, 8, time_boot_ms);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SYSTEM_TIME_LEN);
#else
    mavlink_system_time_t packet;
    packet.time_unix_usec = time_unix_usec;
    packet.time_boot_ms = time_boot_ms;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SYSTEM_TIME_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SYSTEM_TIME;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SYSTEM_TIME_MIN_LEN, MAVLINK_MSG_ID_SYSTEM_TIME_LEN, MAVLINK_MSG_ID_SYSTEM_TIME_CRC);
}

/**
 * @brief Pack a system_time message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_unix_usec [us] Timestamp (UNIX epoch time).
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_system_time_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t time_unix_usec, uint32_t time_boot_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SYSTEM_TIME_LEN];
    _mav_put_uint64_t(buf, 0, time_unix_usec);
    _mav_put_uint32_t(buf, 8, time_boot_ms);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SYSTEM_TIME_LEN);
#else
    mavlink_system_time_t packet;
    packet.time_unix_usec = time_unix_usec;
    packet.time_boot_ms = time_boot_ms;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SYSTEM_TIME_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SYSTEM_TIME;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SYSTEM_TIME_MIN_LEN, MAVLINK_MSG_ID_SYSTEM_TIME_LEN, MAVLINK_MSG_ID_SYSTEM_TIME_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SYSTEM_TIME_MIN_LEN, MAVLINK_MSG_ID_SYSTEM_TIME_LEN);
#endif
}

/**
 * @brief Pack a system_time message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_unix_usec [us] Timestamp (UNIX epoch time).
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_system_time_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_unix_usec,uint32_t time_boot_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SYSTEM_TIME_LEN];
    _mav_put_uint64_t(buf, 0, time_unix_usec);
    _mav_put_uint32_t(buf, 8, time_boot_ms);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SYSTEM_TIME_LEN);
#else
    mavlink_system_time_t packet;
    packet.time_unix_usec = time_unix_usec;
    packet.time_boot_ms = time_boot_ms;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SYSTEM_TIME_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SYSTEM_TIME;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SYSTEM_TIME_MIN_LEN, MAVLINK_MSG_ID_SYSTEM_TIME_LEN, MAVLINK_MSG_ID_SYSTEM_TIME_CRC);
}

/**
 * @brief Encode a system_time struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param system_time C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_system_time_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_system_time_t* system_time)
{
    return mavlink_msg_system_time_pack(system_id, component_id, msg, system_time->time_unix_usec, system_time->time_boot_ms);
}

/**
 * @brief Encode a system_time struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param system_time C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_system_time_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_system_time_t* system_time)
{
    return mavlink_msg_system_time_pack_chan(system_id, component_id, chan, msg, system_time->time_unix_usec, system_time->time_boot_ms);
}

/**
 * @brief Encode a system_time struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param system_time C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_system_time_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_system_time_t* system_time)
{
    return mavlink_msg_system_time_pack_status(system_id, component_id, _status, msg,  system_time->time_unix_usec, system_time->time_boot_ms);
}

/**
 * @brief Send a system_time message
 * @param chan MAVLink channel to send the message
 *
 * @param time_unix_usec [us] Timestamp (UNIX epoch time).
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_system_time_send(mavlink_channel_t chan, uint64_t time_unix_usec, uint32_t time_boot_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SYSTEM_TIME_LEN];
    _mav_put_uint64_t(buf, 0, time_unix_usec);
    _mav_put_uint32_t(buf, 8, time_boot_ms);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYSTEM_TIME, buf, MAVLINK_MSG_ID_SYSTEM_TIME_MIN_LEN, MAVLINK_MSG_ID_SYSTEM_TIME_LEN, MAVLINK_MSG_ID_SYSTEM_TIME_CRC);
#else
    mavlink_system_time_t packet;
    packet.time_unix_usec = time_unix_usec;
    packet.time_boot_ms = time_boot_ms;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYSTEM_TIME, (const char *)&packet, MAVLINK_MSG_ID_SYSTEM_TIME_MIN_LEN, MAVLINK_MSG_ID_SYSTEM_TIME_LEN, MAVLINK_MSG_ID_SYSTEM_TIME_CRC);
#endif
}

/**
 * @brief Send a system_time message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_system_time_send_struct(mavlink_channel_t chan, const mavlink_system_time_t* system_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_system_time_send(chan, system_time->time_unix_usec, system_time->time_boot_ms);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYSTEM_TIME, (const char *)system_time, MAVLINK_MSG_ID_SYSTEM_TIME_MIN_LEN, MAVLINK_MSG_ID_SYSTEM_TIME_LEN, MAVLINK_MSG_ID_SYSTEM_TIME_CRC);
#endif
}

#if MAVLINK_MSG_ID_SYSTEM_TIME_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_system_time_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_unix_usec, uint32_t time_boot_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_unix_usec);
    _mav_put_uint32_t(buf, 8, time_boot_ms);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYSTEM_TIME, buf, MAVLINK_MSG_ID_SYSTEM_TIME_MIN_LEN, MAVLINK_MSG_ID_SYSTEM_TIME_LEN, MAVLINK_MSG_ID_SYSTEM_TIME_CRC);
#else
    mavlink_system_time_t *packet = (mavlink_system_time_t *)msgbuf;
    packet->time_unix_usec = time_unix_usec;
    packet->time_boot_ms = time_boot_ms;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYSTEM_TIME, (const char *)packet, MAVLINK_MSG_ID_SYSTEM_TIME_MIN_LEN, MAVLINK_MSG_ID_SYSTEM_TIME_LEN, MAVLINK_MSG_ID_SYSTEM_TIME_CRC);
#endif
}
#endif

#endif

// MESSAGE SYSTEM_TIME UNPACKING


/**
 * @brief Get field time_unix_usec from system_time message
 *
 * @return [us] Timestamp (UNIX epoch time).
 */
static inline uint64_t mavlink_msg_system_time_get_time_unix_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field time_boot_ms from system_time message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_system_time_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Decode a system_time message into a struct
 *
 * @param msg The message to decode
 * @param system_time C-struct to decode the message contents into
 */
static inline void mavlink_msg_system_time_decode(const mavlink_message_t* msg, mavlink_system_time_t* system_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    system_time->time_unix_usec = mavlink_msg_system_time_get_time_unix_usec(msg);
    system_time->time_boot_ms = mavlink_msg_system_time_get_time_boot_ms(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SYSTEM_TIME_LEN? msg->len : MAVLINK_MSG_ID_SYSTEM_TIME_LEN;
        memset(system_time, 0, MAVLINK_MSG_ID_SYSTEM_TIME_LEN);
    memcpy(system_time, _MAV_PAYLOAD(msg), len);
#endif
}
