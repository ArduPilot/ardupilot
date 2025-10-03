#pragma once
// MESSAGE COMPONENT_PREARM_STATUS PACKING

#define MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS 60025


typedef struct __mavlink_component_prearm_status_t {
 uint32_t enabled_flags; /*<  Currently enabled prearm checks. 0 means no checks are being performed, UINT32_MAX means not known.*/
 uint32_t fail_flags; /*<  Currently not passed prearm checks. 0 means all checks have been passed.*/
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
} mavlink_component_prearm_status_t;

#define MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_LEN 10
#define MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_MIN_LEN 10
#define MAVLINK_MSG_ID_60025_LEN 10
#define MAVLINK_MSG_ID_60025_MIN_LEN 10

#define MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_CRC 20
#define MAVLINK_MSG_ID_60025_CRC 20



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_COMPONENT_PREARM_STATUS { \
    60025, \
    "COMPONENT_PREARM_STATUS", \
    4, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_component_prearm_status_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_component_prearm_status_t, target_component) }, \
         { "enabled_flags", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_component_prearm_status_t, enabled_flags) }, \
         { "fail_flags", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_component_prearm_status_t, fail_flags) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_COMPONENT_PREARM_STATUS { \
    "COMPONENT_PREARM_STATUS", \
    4, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_component_prearm_status_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_component_prearm_status_t, target_component) }, \
         { "enabled_flags", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_component_prearm_status_t, enabled_flags) }, \
         { "fail_flags", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_component_prearm_status_t, fail_flags) }, \
         } \
}
#endif

/**
 * @brief Pack a component_prearm_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param enabled_flags  Currently enabled prearm checks. 0 means no checks are being performed, UINT32_MAX means not known.
 * @param fail_flags  Currently not passed prearm checks. 0 means all checks have been passed.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_component_prearm_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint32_t enabled_flags, uint32_t fail_flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, enabled_flags);
    _mav_put_uint32_t(buf, 4, fail_flags);
    _mav_put_uint8_t(buf, 8, target_system);
    _mav_put_uint8_t(buf, 9, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_LEN);
#else
    mavlink_component_prearm_status_t packet;
    packet.enabled_flags = enabled_flags;
    packet.fail_flags = fail_flags;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_LEN, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_CRC);
}

/**
 * @brief Pack a component_prearm_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param enabled_flags  Currently enabled prearm checks. 0 means no checks are being performed, UINT32_MAX means not known.
 * @param fail_flags  Currently not passed prearm checks. 0 means all checks have been passed.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_component_prearm_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint32_t enabled_flags, uint32_t fail_flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, enabled_flags);
    _mav_put_uint32_t(buf, 4, fail_flags);
    _mav_put_uint8_t(buf, 8, target_system);
    _mav_put_uint8_t(buf, 9, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_LEN);
#else
    mavlink_component_prearm_status_t packet;
    packet.enabled_flags = enabled_flags;
    packet.fail_flags = fail_flags;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_LEN, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_LEN);
#endif
}

/**
 * @brief Pack a component_prearm_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param enabled_flags  Currently enabled prearm checks. 0 means no checks are being performed, UINT32_MAX means not known.
 * @param fail_flags  Currently not passed prearm checks. 0 means all checks have been passed.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_component_prearm_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint32_t enabled_flags,uint32_t fail_flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, enabled_flags);
    _mav_put_uint32_t(buf, 4, fail_flags);
    _mav_put_uint8_t(buf, 8, target_system);
    _mav_put_uint8_t(buf, 9, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_LEN);
#else
    mavlink_component_prearm_status_t packet;
    packet.enabled_flags = enabled_flags;
    packet.fail_flags = fail_flags;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_LEN, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_CRC);
}

/**
 * @brief Encode a component_prearm_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param component_prearm_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_component_prearm_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_component_prearm_status_t* component_prearm_status)
{
    return mavlink_msg_component_prearm_status_pack(system_id, component_id, msg, component_prearm_status->target_system, component_prearm_status->target_component, component_prearm_status->enabled_flags, component_prearm_status->fail_flags);
}

/**
 * @brief Encode a component_prearm_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param component_prearm_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_component_prearm_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_component_prearm_status_t* component_prearm_status)
{
    return mavlink_msg_component_prearm_status_pack_chan(system_id, component_id, chan, msg, component_prearm_status->target_system, component_prearm_status->target_component, component_prearm_status->enabled_flags, component_prearm_status->fail_flags);
}

/**
 * @brief Encode a component_prearm_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param component_prearm_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_component_prearm_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_component_prearm_status_t* component_prearm_status)
{
    return mavlink_msg_component_prearm_status_pack_status(system_id, component_id, _status, msg,  component_prearm_status->target_system, component_prearm_status->target_component, component_prearm_status->enabled_flags, component_prearm_status->fail_flags);
}

/**
 * @brief Send a component_prearm_status message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param enabled_flags  Currently enabled prearm checks. 0 means no checks are being performed, UINT32_MAX means not known.
 * @param fail_flags  Currently not passed prearm checks. 0 means all checks have been passed.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_component_prearm_status_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint32_t enabled_flags, uint32_t fail_flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, enabled_flags);
    _mav_put_uint32_t(buf, 4, fail_flags);
    _mav_put_uint8_t(buf, 8, target_system);
    _mav_put_uint8_t(buf, 9, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS, buf, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_LEN, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_CRC);
#else
    mavlink_component_prearm_status_t packet;
    packet.enabled_flags = enabled_flags;
    packet.fail_flags = fail_flags;
    packet.target_system = target_system;
    packet.target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS, (const char *)&packet, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_LEN, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_CRC);
#endif
}

/**
 * @brief Send a component_prearm_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_component_prearm_status_send_struct(mavlink_channel_t chan, const mavlink_component_prearm_status_t* component_prearm_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_component_prearm_status_send(chan, component_prearm_status->target_system, component_prearm_status->target_component, component_prearm_status->enabled_flags, component_prearm_status->fail_flags);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS, (const char *)component_prearm_status, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_LEN, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_component_prearm_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint32_t enabled_flags, uint32_t fail_flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, enabled_flags);
    _mav_put_uint32_t(buf, 4, fail_flags);
    _mav_put_uint8_t(buf, 8, target_system);
    _mav_put_uint8_t(buf, 9, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS, buf, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_LEN, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_CRC);
#else
    mavlink_component_prearm_status_t *packet = (mavlink_component_prearm_status_t *)msgbuf;
    packet->enabled_flags = enabled_flags;
    packet->fail_flags = fail_flags;
    packet->target_system = target_system;
    packet->target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS, (const char *)packet, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_MIN_LEN, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_LEN, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE COMPONENT_PREARM_STATUS UNPACKING


/**
 * @brief Get field target_system from component_prearm_status message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_component_prearm_status_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field target_component from component_prearm_status message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_component_prearm_status_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field enabled_flags from component_prearm_status message
 *
 * @return  Currently enabled prearm checks. 0 means no checks are being performed, UINT32_MAX means not known.
 */
static inline uint32_t mavlink_msg_component_prearm_status_get_enabled_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field fail_flags from component_prearm_status message
 *
 * @return  Currently not passed prearm checks. 0 means all checks have been passed.
 */
static inline uint32_t mavlink_msg_component_prearm_status_get_fail_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Decode a component_prearm_status message into a struct
 *
 * @param msg The message to decode
 * @param component_prearm_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_component_prearm_status_decode(const mavlink_message_t* msg, mavlink_component_prearm_status_t* component_prearm_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    component_prearm_status->enabled_flags = mavlink_msg_component_prearm_status_get_enabled_flags(msg);
    component_prearm_status->fail_flags = mavlink_msg_component_prearm_status_get_fail_flags(msg);
    component_prearm_status->target_system = mavlink_msg_component_prearm_status_get_target_system(msg);
    component_prearm_status->target_component = mavlink_msg_component_prearm_status_get_target_component(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_LEN? msg->len : MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_LEN;
        memset(component_prearm_status, 0, MAVLINK_MSG_ID_COMPONENT_PREARM_STATUS_LEN);
    memcpy(component_prearm_status, _MAV_PAYLOAD(msg), len);
#endif
}
