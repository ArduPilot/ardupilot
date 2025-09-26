#pragma once
// MESSAGE TIMESYNC PACKING

#define MAVLINK_MSG_ID_TIMESYNC 111


typedef struct __mavlink_timesync_t {
 int64_t tc1; /*<  Time sync timestamp 1*/
 int64_t ts1; /*<  Time sync timestamp 2*/
} mavlink_timesync_t;

#define MAVLINK_MSG_ID_TIMESYNC_LEN 16
#define MAVLINK_MSG_ID_TIMESYNC_MIN_LEN 16
#define MAVLINK_MSG_ID_111_LEN 16
#define MAVLINK_MSG_ID_111_MIN_LEN 16

#define MAVLINK_MSG_ID_TIMESYNC_CRC 34
#define MAVLINK_MSG_ID_111_CRC 34



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TIMESYNC { \
    111, \
    "TIMESYNC", \
    2, \
    {  { "tc1", NULL, MAVLINK_TYPE_INT64_T, 0, 0, offsetof(mavlink_timesync_t, tc1) }, \
         { "ts1", NULL, MAVLINK_TYPE_INT64_T, 0, 8, offsetof(mavlink_timesync_t, ts1) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TIMESYNC { \
    "TIMESYNC", \
    2, \
    {  { "tc1", NULL, MAVLINK_TYPE_INT64_T, 0, 0, offsetof(mavlink_timesync_t, tc1) }, \
         { "ts1", NULL, MAVLINK_TYPE_INT64_T, 0, 8, offsetof(mavlink_timesync_t, ts1) }, \
         } \
}
#endif

/**
 * @brief Pack a timesync message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param tc1  Time sync timestamp 1
 * @param ts1  Time sync timestamp 2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_timesync_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int64_t tc1, int64_t ts1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TIMESYNC_LEN];
    _mav_put_int64_t(buf, 0, tc1);
    _mav_put_int64_t(buf, 8, ts1);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TIMESYNC_LEN);
#else
    mavlink_timesync_t packet;
    packet.tc1 = tc1;
    packet.ts1 = ts1;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TIMESYNC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TIMESYNC;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TIMESYNC_MIN_LEN, MAVLINK_MSG_ID_TIMESYNC_LEN, MAVLINK_MSG_ID_TIMESYNC_CRC);
}

/**
 * @brief Pack a timesync message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param tc1  Time sync timestamp 1
 * @param ts1  Time sync timestamp 2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_timesync_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               int64_t tc1, int64_t ts1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TIMESYNC_LEN];
    _mav_put_int64_t(buf, 0, tc1);
    _mav_put_int64_t(buf, 8, ts1);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TIMESYNC_LEN);
#else
    mavlink_timesync_t packet;
    packet.tc1 = tc1;
    packet.ts1 = ts1;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TIMESYNC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TIMESYNC;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TIMESYNC_MIN_LEN, MAVLINK_MSG_ID_TIMESYNC_LEN, MAVLINK_MSG_ID_TIMESYNC_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TIMESYNC_MIN_LEN, MAVLINK_MSG_ID_TIMESYNC_LEN);
#endif
}

/**
 * @brief Pack a timesync message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param tc1  Time sync timestamp 1
 * @param ts1  Time sync timestamp 2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_timesync_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int64_t tc1,int64_t ts1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TIMESYNC_LEN];
    _mav_put_int64_t(buf, 0, tc1);
    _mav_put_int64_t(buf, 8, ts1);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TIMESYNC_LEN);
#else
    mavlink_timesync_t packet;
    packet.tc1 = tc1;
    packet.ts1 = ts1;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TIMESYNC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TIMESYNC;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TIMESYNC_MIN_LEN, MAVLINK_MSG_ID_TIMESYNC_LEN, MAVLINK_MSG_ID_TIMESYNC_CRC);
}

/**
 * @brief Encode a timesync struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param timesync C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_timesync_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_timesync_t* timesync)
{
    return mavlink_msg_timesync_pack(system_id, component_id, msg, timesync->tc1, timesync->ts1);
}

/**
 * @brief Encode a timesync struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timesync C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_timesync_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_timesync_t* timesync)
{
    return mavlink_msg_timesync_pack_chan(system_id, component_id, chan, msg, timesync->tc1, timesync->ts1);
}

/**
 * @brief Encode a timesync struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param timesync C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_timesync_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_timesync_t* timesync)
{
    return mavlink_msg_timesync_pack_status(system_id, component_id, _status, msg,  timesync->tc1, timesync->ts1);
}

/**
 * @brief Send a timesync message
 * @param chan MAVLink channel to send the message
 *
 * @param tc1  Time sync timestamp 1
 * @param ts1  Time sync timestamp 2
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_timesync_send(mavlink_channel_t chan, int64_t tc1, int64_t ts1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TIMESYNC_LEN];
    _mav_put_int64_t(buf, 0, tc1);
    _mav_put_int64_t(buf, 8, ts1);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TIMESYNC, buf, MAVLINK_MSG_ID_TIMESYNC_MIN_LEN, MAVLINK_MSG_ID_TIMESYNC_LEN, MAVLINK_MSG_ID_TIMESYNC_CRC);
#else
    mavlink_timesync_t packet;
    packet.tc1 = tc1;
    packet.ts1 = ts1;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TIMESYNC, (const char *)&packet, MAVLINK_MSG_ID_TIMESYNC_MIN_LEN, MAVLINK_MSG_ID_TIMESYNC_LEN, MAVLINK_MSG_ID_TIMESYNC_CRC);
#endif
}

/**
 * @brief Send a timesync message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_timesync_send_struct(mavlink_channel_t chan, const mavlink_timesync_t* timesync)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_timesync_send(chan, timesync->tc1, timesync->ts1);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TIMESYNC, (const char *)timesync, MAVLINK_MSG_ID_TIMESYNC_MIN_LEN, MAVLINK_MSG_ID_TIMESYNC_LEN, MAVLINK_MSG_ID_TIMESYNC_CRC);
#endif
}

#if MAVLINK_MSG_ID_TIMESYNC_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_timesync_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int64_t tc1, int64_t ts1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int64_t(buf, 0, tc1);
    _mav_put_int64_t(buf, 8, ts1);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TIMESYNC, buf, MAVLINK_MSG_ID_TIMESYNC_MIN_LEN, MAVLINK_MSG_ID_TIMESYNC_LEN, MAVLINK_MSG_ID_TIMESYNC_CRC);
#else
    mavlink_timesync_t *packet = (mavlink_timesync_t *)msgbuf;
    packet->tc1 = tc1;
    packet->ts1 = ts1;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TIMESYNC, (const char *)packet, MAVLINK_MSG_ID_TIMESYNC_MIN_LEN, MAVLINK_MSG_ID_TIMESYNC_LEN, MAVLINK_MSG_ID_TIMESYNC_CRC);
#endif
}
#endif

#endif

// MESSAGE TIMESYNC UNPACKING


/**
 * @brief Get field tc1 from timesync message
 *
 * @return  Time sync timestamp 1
 */
static inline int64_t mavlink_msg_timesync_get_tc1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int64_t(msg,  0);
}

/**
 * @brief Get field ts1 from timesync message
 *
 * @return  Time sync timestamp 2
 */
static inline int64_t mavlink_msg_timesync_get_ts1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int64_t(msg,  8);
}

/**
 * @brief Decode a timesync message into a struct
 *
 * @param msg The message to decode
 * @param timesync C-struct to decode the message contents into
 */
static inline void mavlink_msg_timesync_decode(const mavlink_message_t* msg, mavlink_timesync_t* timesync)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    timesync->tc1 = mavlink_msg_timesync_get_tc1(msg);
    timesync->ts1 = mavlink_msg_timesync_get_ts1(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TIMESYNC_LEN? msg->len : MAVLINK_MSG_ID_TIMESYNC_LEN;
        memset(timesync, 0, MAVLINK_MSG_ID_TIMESYNC_LEN);
    memcpy(timesync, _MAV_PAYLOAD(msg), len);
#endif
}
