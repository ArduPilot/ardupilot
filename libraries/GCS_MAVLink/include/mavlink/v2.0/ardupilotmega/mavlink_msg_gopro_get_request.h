#pragma once
// MESSAGE GOPRO_GET_REQUEST PACKING

#define MAVLINK_MSG_ID_GOPRO_GET_REQUEST 216


typedef struct __mavlink_gopro_get_request_t {
 uint8_t target_system; /*<  System ID.*/
 uint8_t target_component; /*<  Component ID.*/
 uint8_t cmd_id; /*<  Command ID.*/
} mavlink_gopro_get_request_t;

#define MAVLINK_MSG_ID_GOPRO_GET_REQUEST_LEN 3
#define MAVLINK_MSG_ID_GOPRO_GET_REQUEST_MIN_LEN 3
#define MAVLINK_MSG_ID_216_LEN 3
#define MAVLINK_MSG_ID_216_MIN_LEN 3

#define MAVLINK_MSG_ID_GOPRO_GET_REQUEST_CRC 50
#define MAVLINK_MSG_ID_216_CRC 50



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GOPRO_GET_REQUEST { \
    216, \
    "GOPRO_GET_REQUEST", \
    3, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_gopro_get_request_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_gopro_get_request_t, target_component) }, \
         { "cmd_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_gopro_get_request_t, cmd_id) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GOPRO_GET_REQUEST { \
    "GOPRO_GET_REQUEST", \
    3, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_gopro_get_request_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_gopro_get_request_t, target_component) }, \
         { "cmd_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_gopro_get_request_t, cmd_id) }, \
         } \
}
#endif

/**
 * @brief Pack a gopro_get_request message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param cmd_id  Command ID.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gopro_get_request_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t cmd_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GOPRO_GET_REQUEST_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, cmd_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GOPRO_GET_REQUEST_LEN);
#else
    mavlink_gopro_get_request_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.cmd_id = cmd_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GOPRO_GET_REQUEST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GOPRO_GET_REQUEST;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GOPRO_GET_REQUEST_MIN_LEN, MAVLINK_MSG_ID_GOPRO_GET_REQUEST_LEN, MAVLINK_MSG_ID_GOPRO_GET_REQUEST_CRC);
}

/**
 * @brief Pack a gopro_get_request message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param cmd_id  Command ID.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gopro_get_request_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t cmd_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GOPRO_GET_REQUEST_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, cmd_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GOPRO_GET_REQUEST_LEN);
#else
    mavlink_gopro_get_request_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.cmd_id = cmd_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GOPRO_GET_REQUEST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GOPRO_GET_REQUEST;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_GOPRO_GET_REQUEST_MIN_LEN, MAVLINK_MSG_ID_GOPRO_GET_REQUEST_LEN, MAVLINK_MSG_ID_GOPRO_GET_REQUEST_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_GOPRO_GET_REQUEST_MIN_LEN, MAVLINK_MSG_ID_GOPRO_GET_REQUEST_LEN);
#endif
}

/**
 * @brief Pack a gopro_get_request message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param cmd_id  Command ID.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gopro_get_request_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint8_t cmd_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GOPRO_GET_REQUEST_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, cmd_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GOPRO_GET_REQUEST_LEN);
#else
    mavlink_gopro_get_request_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.cmd_id = cmd_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GOPRO_GET_REQUEST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GOPRO_GET_REQUEST;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GOPRO_GET_REQUEST_MIN_LEN, MAVLINK_MSG_ID_GOPRO_GET_REQUEST_LEN, MAVLINK_MSG_ID_GOPRO_GET_REQUEST_CRC);
}

/**
 * @brief Encode a gopro_get_request struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gopro_get_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gopro_get_request_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gopro_get_request_t* gopro_get_request)
{
    return mavlink_msg_gopro_get_request_pack(system_id, component_id, msg, gopro_get_request->target_system, gopro_get_request->target_component, gopro_get_request->cmd_id);
}

/**
 * @brief Encode a gopro_get_request struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gopro_get_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gopro_get_request_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gopro_get_request_t* gopro_get_request)
{
    return mavlink_msg_gopro_get_request_pack_chan(system_id, component_id, chan, msg, gopro_get_request->target_system, gopro_get_request->target_component, gopro_get_request->cmd_id);
}

/**
 * @brief Encode a gopro_get_request struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param gopro_get_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gopro_get_request_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_gopro_get_request_t* gopro_get_request)
{
    return mavlink_msg_gopro_get_request_pack_status(system_id, component_id, _status, msg,  gopro_get_request->target_system, gopro_get_request->target_component, gopro_get_request->cmd_id);
}

/**
 * @brief Send a gopro_get_request message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param cmd_id  Command ID.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gopro_get_request_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t cmd_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GOPRO_GET_REQUEST_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, cmd_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GOPRO_GET_REQUEST, buf, MAVLINK_MSG_ID_GOPRO_GET_REQUEST_MIN_LEN, MAVLINK_MSG_ID_GOPRO_GET_REQUEST_LEN, MAVLINK_MSG_ID_GOPRO_GET_REQUEST_CRC);
#else
    mavlink_gopro_get_request_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.cmd_id = cmd_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GOPRO_GET_REQUEST, (const char *)&packet, MAVLINK_MSG_ID_GOPRO_GET_REQUEST_MIN_LEN, MAVLINK_MSG_ID_GOPRO_GET_REQUEST_LEN, MAVLINK_MSG_ID_GOPRO_GET_REQUEST_CRC);
#endif
}

/**
 * @brief Send a gopro_get_request message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gopro_get_request_send_struct(mavlink_channel_t chan, const mavlink_gopro_get_request_t* gopro_get_request)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gopro_get_request_send(chan, gopro_get_request->target_system, gopro_get_request->target_component, gopro_get_request->cmd_id);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GOPRO_GET_REQUEST, (const char *)gopro_get_request, MAVLINK_MSG_ID_GOPRO_GET_REQUEST_MIN_LEN, MAVLINK_MSG_ID_GOPRO_GET_REQUEST_LEN, MAVLINK_MSG_ID_GOPRO_GET_REQUEST_CRC);
#endif
}

#if MAVLINK_MSG_ID_GOPRO_GET_REQUEST_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gopro_get_request_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t cmd_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, cmd_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GOPRO_GET_REQUEST, buf, MAVLINK_MSG_ID_GOPRO_GET_REQUEST_MIN_LEN, MAVLINK_MSG_ID_GOPRO_GET_REQUEST_LEN, MAVLINK_MSG_ID_GOPRO_GET_REQUEST_CRC);
#else
    mavlink_gopro_get_request_t *packet = (mavlink_gopro_get_request_t *)msgbuf;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->cmd_id = cmd_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GOPRO_GET_REQUEST, (const char *)packet, MAVLINK_MSG_ID_GOPRO_GET_REQUEST_MIN_LEN, MAVLINK_MSG_ID_GOPRO_GET_REQUEST_LEN, MAVLINK_MSG_ID_GOPRO_GET_REQUEST_CRC);
#endif
}
#endif

#endif

// MESSAGE GOPRO_GET_REQUEST UNPACKING


/**
 * @brief Get field target_system from gopro_get_request message
 *
 * @return  System ID.
 */
static inline uint8_t mavlink_msg_gopro_get_request_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field target_component from gopro_get_request message
 *
 * @return  Component ID.
 */
static inline uint8_t mavlink_msg_gopro_get_request_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field cmd_id from gopro_get_request message
 *
 * @return  Command ID.
 */
static inline uint8_t mavlink_msg_gopro_get_request_get_cmd_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a gopro_get_request message into a struct
 *
 * @param msg The message to decode
 * @param gopro_get_request C-struct to decode the message contents into
 */
static inline void mavlink_msg_gopro_get_request_decode(const mavlink_message_t* msg, mavlink_gopro_get_request_t* gopro_get_request)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gopro_get_request->target_system = mavlink_msg_gopro_get_request_get_target_system(msg);
    gopro_get_request->target_component = mavlink_msg_gopro_get_request_get_target_component(msg);
    gopro_get_request->cmd_id = mavlink_msg_gopro_get_request_get_cmd_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GOPRO_GET_REQUEST_LEN? msg->len : MAVLINK_MSG_ID_GOPRO_GET_REQUEST_LEN;
        memset(gopro_get_request, 0, MAVLINK_MSG_ID_GOPRO_GET_REQUEST_LEN);
    memcpy(gopro_get_request, _MAV_PAYLOAD(msg), len);
#endif
}
