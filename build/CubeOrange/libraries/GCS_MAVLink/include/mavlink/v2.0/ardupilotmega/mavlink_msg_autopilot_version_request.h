#pragma once
// MESSAGE AUTOPILOT_VERSION_REQUEST PACKING

#define MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST 183


typedef struct __mavlink_autopilot_version_request_t {
 uint8_t target_system; /*<  System ID.*/
 uint8_t target_component; /*<  Component ID.*/
} mavlink_autopilot_version_request_t;

#define MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_LEN 2
#define MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_MIN_LEN 2
#define MAVLINK_MSG_ID_183_LEN 2
#define MAVLINK_MSG_ID_183_MIN_LEN 2

#define MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_CRC 85
#define MAVLINK_MSG_ID_183_CRC 85



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AUTOPILOT_VERSION_REQUEST { \
    183, \
    "AUTOPILOT_VERSION_REQUEST", \
    2, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_autopilot_version_request_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_autopilot_version_request_t, target_component) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AUTOPILOT_VERSION_REQUEST { \
    "AUTOPILOT_VERSION_REQUEST", \
    2, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_autopilot_version_request_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_autopilot_version_request_t, target_component) }, \
         } \
}
#endif

/**
 * @brief Pack a autopilot_version_request message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_autopilot_version_request_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_LEN);
#else
    mavlink_autopilot_version_request_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_MIN_LEN, MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_LEN, MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_CRC);
}

/**
 * @brief Pack a autopilot_version_request message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_autopilot_version_request_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_LEN);
#else
    mavlink_autopilot_version_request_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_MIN_LEN, MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_LEN, MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_CRC);
}

/**
 * @brief Encode a autopilot_version_request struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param autopilot_version_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_autopilot_version_request_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_autopilot_version_request_t* autopilot_version_request)
{
    return mavlink_msg_autopilot_version_request_pack(system_id, component_id, msg, autopilot_version_request->target_system, autopilot_version_request->target_component);
}

/**
 * @brief Encode a autopilot_version_request struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param autopilot_version_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_autopilot_version_request_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_autopilot_version_request_t* autopilot_version_request)
{
    return mavlink_msg_autopilot_version_request_pack_chan(system_id, component_id, chan, msg, autopilot_version_request->target_system, autopilot_version_request->target_component);
}

/**
 * @brief Send a autopilot_version_request message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_autopilot_version_request_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST, buf, MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_MIN_LEN, MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_LEN, MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_CRC);
#else
    mavlink_autopilot_version_request_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST, (const char *)&packet, MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_MIN_LEN, MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_LEN, MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_CRC);
#endif
}

/**
 * @brief Send a autopilot_version_request message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_autopilot_version_request_send_struct(mavlink_channel_t chan, const mavlink_autopilot_version_request_t* autopilot_version_request)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_autopilot_version_request_send(chan, autopilot_version_request->target_system, autopilot_version_request->target_component);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST, (const char *)autopilot_version_request, MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_MIN_LEN, MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_LEN, MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_CRC);
#endif
}

#if MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_autopilot_version_request_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST, buf, MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_MIN_LEN, MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_LEN, MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_CRC);
#else
    mavlink_autopilot_version_request_t *packet = (mavlink_autopilot_version_request_t *)msgbuf;
    packet->target_system = target_system;
    packet->target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST, (const char *)packet, MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_MIN_LEN, MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_LEN, MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_CRC);
#endif
}
#endif

#endif

// MESSAGE AUTOPILOT_VERSION_REQUEST UNPACKING


/**
 * @brief Get field target_system from autopilot_version_request message
 *
 * @return  System ID.
 */
static inline uint8_t mavlink_msg_autopilot_version_request_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field target_component from autopilot_version_request message
 *
 * @return  Component ID.
 */
static inline uint8_t mavlink_msg_autopilot_version_request_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a autopilot_version_request message into a struct
 *
 * @param msg The message to decode
 * @param autopilot_version_request C-struct to decode the message contents into
 */
static inline void mavlink_msg_autopilot_version_request_decode(const mavlink_message_t* msg, mavlink_autopilot_version_request_t* autopilot_version_request)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    autopilot_version_request->target_system = mavlink_msg_autopilot_version_request_get_target_system(msg);
    autopilot_version_request->target_component = mavlink_msg_autopilot_version_request_get_target_component(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_LEN? msg->len : MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_LEN;
        memset(autopilot_version_request, 0, MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_LEN);
    memcpy(autopilot_version_request, _MAV_PAYLOAD(msg), len);
#endif
}
