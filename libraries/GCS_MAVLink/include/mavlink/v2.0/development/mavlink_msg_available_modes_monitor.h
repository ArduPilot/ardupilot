#pragma once
// MESSAGE AVAILABLE_MODES_MONITOR PACKING

#define MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR 437


typedef struct __mavlink_available_modes_monitor_t {
 uint8_t seq; /*<  Sequence number. The value iterates sequentially whenever AVAILABLE_MODES changes (e.g. support for a new mode is added/removed dynamically).*/
} mavlink_available_modes_monitor_t;

#define MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_LEN 1
#define MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_MIN_LEN 1
#define MAVLINK_MSG_ID_437_LEN 1
#define MAVLINK_MSG_ID_437_MIN_LEN 1

#define MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_CRC 30
#define MAVLINK_MSG_ID_437_CRC 30



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AVAILABLE_MODES_MONITOR { \
    437, \
    "AVAILABLE_MODES_MONITOR", \
    1, \
    {  { "seq", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_available_modes_monitor_t, seq) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AVAILABLE_MODES_MONITOR { \
    "AVAILABLE_MODES_MONITOR", \
    1, \
    {  { "seq", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_available_modes_monitor_t, seq) }, \
         } \
}
#endif

/**
 * @brief Pack a available_modes_monitor message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param seq  Sequence number. The value iterates sequentially whenever AVAILABLE_MODES changes (e.g. support for a new mode is added/removed dynamically).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_available_modes_monitor_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_LEN];
    _mav_put_uint8_t(buf, 0, seq);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_LEN);
#else
    mavlink_available_modes_monitor_t packet;
    packet.seq = seq;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_MIN_LEN, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_LEN, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_CRC);
}

/**
 * @brief Pack a available_modes_monitor message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param seq  Sequence number. The value iterates sequentially whenever AVAILABLE_MODES changes (e.g. support for a new mode is added/removed dynamically).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_available_modes_monitor_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_LEN];
    _mav_put_uint8_t(buf, 0, seq);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_LEN);
#else
    mavlink_available_modes_monitor_t packet;
    packet.seq = seq;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_MIN_LEN, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_LEN, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_MIN_LEN, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_LEN);
#endif
}

/**
 * @brief Pack a available_modes_monitor message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param seq  Sequence number. The value iterates sequentially whenever AVAILABLE_MODES changes (e.g. support for a new mode is added/removed dynamically).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_available_modes_monitor_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_LEN];
    _mav_put_uint8_t(buf, 0, seq);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_LEN);
#else
    mavlink_available_modes_monitor_t packet;
    packet.seq = seq;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_MIN_LEN, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_LEN, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_CRC);
}

/**
 * @brief Encode a available_modes_monitor struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param available_modes_monitor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_available_modes_monitor_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_available_modes_monitor_t* available_modes_monitor)
{
    return mavlink_msg_available_modes_monitor_pack(system_id, component_id, msg, available_modes_monitor->seq);
}

/**
 * @brief Encode a available_modes_monitor struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param available_modes_monitor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_available_modes_monitor_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_available_modes_monitor_t* available_modes_monitor)
{
    return mavlink_msg_available_modes_monitor_pack_chan(system_id, component_id, chan, msg, available_modes_monitor->seq);
}

/**
 * @brief Encode a available_modes_monitor struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param available_modes_monitor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_available_modes_monitor_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_available_modes_monitor_t* available_modes_monitor)
{
    return mavlink_msg_available_modes_monitor_pack_status(system_id, component_id, _status, msg,  available_modes_monitor->seq);
}

/**
 * @brief Send a available_modes_monitor message
 * @param chan MAVLink channel to send the message
 *
 * @param seq  Sequence number. The value iterates sequentially whenever AVAILABLE_MODES changes (e.g. support for a new mode is added/removed dynamically).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_available_modes_monitor_send(mavlink_channel_t chan, uint8_t seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_LEN];
    _mav_put_uint8_t(buf, 0, seq);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR, buf, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_MIN_LEN, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_LEN, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_CRC);
#else
    mavlink_available_modes_monitor_t packet;
    packet.seq = seq;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR, (const char *)&packet, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_MIN_LEN, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_LEN, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_CRC);
#endif
}

/**
 * @brief Send a available_modes_monitor message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_available_modes_monitor_send_struct(mavlink_channel_t chan, const mavlink_available_modes_monitor_t* available_modes_monitor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_available_modes_monitor_send(chan, available_modes_monitor->seq);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR, (const char *)available_modes_monitor, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_MIN_LEN, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_LEN, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_CRC);
#endif
}

#if MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_available_modes_monitor_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, seq);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR, buf, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_MIN_LEN, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_LEN, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_CRC);
#else
    mavlink_available_modes_monitor_t *packet = (mavlink_available_modes_monitor_t *)msgbuf;
    packet->seq = seq;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR, (const char *)packet, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_MIN_LEN, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_LEN, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_CRC);
#endif
}
#endif

#endif

// MESSAGE AVAILABLE_MODES_MONITOR UNPACKING


/**
 * @brief Get field seq from available_modes_monitor message
 *
 * @return  Sequence number. The value iterates sequentially whenever AVAILABLE_MODES changes (e.g. support for a new mode is added/removed dynamically).
 */
static inline uint8_t mavlink_msg_available_modes_monitor_get_seq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a available_modes_monitor message into a struct
 *
 * @param msg The message to decode
 * @param available_modes_monitor C-struct to decode the message contents into
 */
static inline void mavlink_msg_available_modes_monitor_decode(const mavlink_message_t* msg, mavlink_available_modes_monitor_t* available_modes_monitor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    available_modes_monitor->seq = mavlink_msg_available_modes_monitor_get_seq(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_LEN? msg->len : MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_LEN;
        memset(available_modes_monitor, 0, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_LEN);
    memcpy(available_modes_monitor, _MAV_PAYLOAD(msg), len);
#endif
}
