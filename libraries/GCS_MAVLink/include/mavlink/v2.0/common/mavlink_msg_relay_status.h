#pragma once
// MESSAGE RELAY_STATUS PACKING

#define MAVLINK_MSG_ID_RELAY_STATUS 376


typedef struct __mavlink_relay_status_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 uint16_t on; /*<  Relay states.  Relay instance numbers are represented as individual bits in this mask by offset.*/
 uint16_t present; /*<  Relay present.  Relay instance numbers are represented as individual bits in this mask by offset.  Bits will be true if a relay instance is configured.*/
} mavlink_relay_status_t;

#define MAVLINK_MSG_ID_RELAY_STATUS_LEN 8
#define MAVLINK_MSG_ID_RELAY_STATUS_MIN_LEN 8
#define MAVLINK_MSG_ID_376_LEN 8
#define MAVLINK_MSG_ID_376_MIN_LEN 8

#define MAVLINK_MSG_ID_RELAY_STATUS_CRC 199
#define MAVLINK_MSG_ID_376_CRC 199



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RELAY_STATUS { \
    376, \
    "RELAY_STATUS", \
    3, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_relay_status_t, time_boot_ms) }, \
         { "on", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_relay_status_t, on) }, \
         { "present", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_relay_status_t, present) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RELAY_STATUS { \
    "RELAY_STATUS", \
    3, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_relay_status_t, time_boot_ms) }, \
         { "on", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_relay_status_t, on) }, \
         { "present", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_relay_status_t, present) }, \
         } \
}
#endif

/**
 * @brief Pack a relay_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param on  Relay states.  Relay instance numbers are represented as individual bits in this mask by offset.
 * @param present  Relay present.  Relay instance numbers are represented as individual bits in this mask by offset.  Bits will be true if a relay instance is configured.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_relay_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint16_t on, uint16_t present)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RELAY_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint16_t(buf, 4, on);
    _mav_put_uint16_t(buf, 6, present);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RELAY_STATUS_LEN);
#else
    mavlink_relay_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.on = on;
    packet.present = present;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RELAY_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RELAY_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RELAY_STATUS_MIN_LEN, MAVLINK_MSG_ID_RELAY_STATUS_LEN, MAVLINK_MSG_ID_RELAY_STATUS_CRC);
}

/**
 * @brief Pack a relay_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param on  Relay states.  Relay instance numbers are represented as individual bits in this mask by offset.
 * @param present  Relay present.  Relay instance numbers are represented as individual bits in this mask by offset.  Bits will be true if a relay instance is configured.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_relay_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint16_t on, uint16_t present)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RELAY_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint16_t(buf, 4, on);
    _mav_put_uint16_t(buf, 6, present);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RELAY_STATUS_LEN);
#else
    mavlink_relay_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.on = on;
    packet.present = present;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RELAY_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RELAY_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_RELAY_STATUS_MIN_LEN, MAVLINK_MSG_ID_RELAY_STATUS_LEN, MAVLINK_MSG_ID_RELAY_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_RELAY_STATUS_MIN_LEN, MAVLINK_MSG_ID_RELAY_STATUS_LEN);
#endif
}

/**
 * @brief Pack a relay_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param on  Relay states.  Relay instance numbers are represented as individual bits in this mask by offset.
 * @param present  Relay present.  Relay instance numbers are represented as individual bits in this mask by offset.  Bits will be true if a relay instance is configured.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_relay_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,uint16_t on,uint16_t present)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RELAY_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint16_t(buf, 4, on);
    _mav_put_uint16_t(buf, 6, present);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RELAY_STATUS_LEN);
#else
    mavlink_relay_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.on = on;
    packet.present = present;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RELAY_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RELAY_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RELAY_STATUS_MIN_LEN, MAVLINK_MSG_ID_RELAY_STATUS_LEN, MAVLINK_MSG_ID_RELAY_STATUS_CRC);
}

/**
 * @brief Encode a relay_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param relay_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_relay_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_relay_status_t* relay_status)
{
    return mavlink_msg_relay_status_pack(system_id, component_id, msg, relay_status->time_boot_ms, relay_status->on, relay_status->present);
}

/**
 * @brief Encode a relay_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param relay_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_relay_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_relay_status_t* relay_status)
{
    return mavlink_msg_relay_status_pack_chan(system_id, component_id, chan, msg, relay_status->time_boot_ms, relay_status->on, relay_status->present);
}

/**
 * @brief Encode a relay_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param relay_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_relay_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_relay_status_t* relay_status)
{
    return mavlink_msg_relay_status_pack_status(system_id, component_id, _status, msg,  relay_status->time_boot_ms, relay_status->on, relay_status->present);
}

/**
 * @brief Send a relay_status message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param on  Relay states.  Relay instance numbers are represented as individual bits in this mask by offset.
 * @param present  Relay present.  Relay instance numbers are represented as individual bits in this mask by offset.  Bits will be true if a relay instance is configured.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_relay_status_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint16_t on, uint16_t present)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RELAY_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint16_t(buf, 4, on);
    _mav_put_uint16_t(buf, 6, present);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RELAY_STATUS, buf, MAVLINK_MSG_ID_RELAY_STATUS_MIN_LEN, MAVLINK_MSG_ID_RELAY_STATUS_LEN, MAVLINK_MSG_ID_RELAY_STATUS_CRC);
#else
    mavlink_relay_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.on = on;
    packet.present = present;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RELAY_STATUS, (const char *)&packet, MAVLINK_MSG_ID_RELAY_STATUS_MIN_LEN, MAVLINK_MSG_ID_RELAY_STATUS_LEN, MAVLINK_MSG_ID_RELAY_STATUS_CRC);
#endif
}

/**
 * @brief Send a relay_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_relay_status_send_struct(mavlink_channel_t chan, const mavlink_relay_status_t* relay_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_relay_status_send(chan, relay_status->time_boot_ms, relay_status->on, relay_status->present);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RELAY_STATUS, (const char *)relay_status, MAVLINK_MSG_ID_RELAY_STATUS_MIN_LEN, MAVLINK_MSG_ID_RELAY_STATUS_LEN, MAVLINK_MSG_ID_RELAY_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_RELAY_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_relay_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint16_t on, uint16_t present)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint16_t(buf, 4, on);
    _mav_put_uint16_t(buf, 6, present);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RELAY_STATUS, buf, MAVLINK_MSG_ID_RELAY_STATUS_MIN_LEN, MAVLINK_MSG_ID_RELAY_STATUS_LEN, MAVLINK_MSG_ID_RELAY_STATUS_CRC);
#else
    mavlink_relay_status_t *packet = (mavlink_relay_status_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->on = on;
    packet->present = present;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RELAY_STATUS, (const char *)packet, MAVLINK_MSG_ID_RELAY_STATUS_MIN_LEN, MAVLINK_MSG_ID_RELAY_STATUS_LEN, MAVLINK_MSG_ID_RELAY_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE RELAY_STATUS UNPACKING


/**
 * @brief Get field time_boot_ms from relay_status message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_relay_status_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field on from relay_status message
 *
 * @return  Relay states.  Relay instance numbers are represented as individual bits in this mask by offset.
 */
static inline uint16_t mavlink_msg_relay_status_get_on(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field present from relay_status message
 *
 * @return  Relay present.  Relay instance numbers are represented as individual bits in this mask by offset.  Bits will be true if a relay instance is configured.
 */
static inline uint16_t mavlink_msg_relay_status_get_present(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Decode a relay_status message into a struct
 *
 * @param msg The message to decode
 * @param relay_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_relay_status_decode(const mavlink_message_t* msg, mavlink_relay_status_t* relay_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    relay_status->time_boot_ms = mavlink_msg_relay_status_get_time_boot_ms(msg);
    relay_status->on = mavlink_msg_relay_status_get_on(msg);
    relay_status->present = mavlink_msg_relay_status_get_present(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RELAY_STATUS_LEN? msg->len : MAVLINK_MSG_ID_RELAY_STATUS_LEN;
        memset(relay_status, 0, MAVLINK_MSG_ID_RELAY_STATUS_LEN);
    memcpy(relay_status, _MAV_PAYLOAD(msg), len);
#endif
}
