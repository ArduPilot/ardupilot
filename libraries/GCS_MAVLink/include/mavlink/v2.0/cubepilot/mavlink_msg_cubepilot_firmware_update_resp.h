#pragma once
// MESSAGE CUBEPILOT_FIRMWARE_UPDATE_RESP PACKING

#define MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP 50005


typedef struct __mavlink_cubepilot_firmware_update_resp_t {
 uint32_t offset; /*< [bytes] FW Offset.*/
 uint8_t target_system; /*<  System ID.*/
 uint8_t target_component; /*<  Component ID.*/
} mavlink_cubepilot_firmware_update_resp_t;

#define MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_LEN 6
#define MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_MIN_LEN 6
#define MAVLINK_MSG_ID_50005_LEN 6
#define MAVLINK_MSG_ID_50005_MIN_LEN 6

#define MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_CRC 152
#define MAVLINK_MSG_ID_50005_CRC 152



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CUBEPILOT_FIRMWARE_UPDATE_RESP { \
    50005, \
    "CUBEPILOT_FIRMWARE_UPDATE_RESP", \
    3, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_cubepilot_firmware_update_resp_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_cubepilot_firmware_update_resp_t, target_component) }, \
         { "offset", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_cubepilot_firmware_update_resp_t, offset) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CUBEPILOT_FIRMWARE_UPDATE_RESP { \
    "CUBEPILOT_FIRMWARE_UPDATE_RESP", \
    3, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_cubepilot_firmware_update_resp_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_cubepilot_firmware_update_resp_t, target_component) }, \
         { "offset", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_cubepilot_firmware_update_resp_t, offset) }, \
         } \
}
#endif

/**
 * @brief Pack a cubepilot_firmware_update_resp message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param offset [bytes] FW Offset.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cubepilot_firmware_update_resp_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint32_t offset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_LEN];
    _mav_put_uint32_t(buf, 0, offset);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_LEN);
#else
    mavlink_cubepilot_firmware_update_resp_t packet;
    packet.offset = offset;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_MIN_LEN, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_LEN, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_CRC);
}

/**
 * @brief Pack a cubepilot_firmware_update_resp message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param offset [bytes] FW Offset.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cubepilot_firmware_update_resp_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint32_t offset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_LEN];
    _mav_put_uint32_t(buf, 0, offset);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_LEN);
#else
    mavlink_cubepilot_firmware_update_resp_t packet;
    packet.offset = offset;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_MIN_LEN, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_LEN, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_MIN_LEN, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_LEN);
#endif
}

/**
 * @brief Pack a cubepilot_firmware_update_resp message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param offset [bytes] FW Offset.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cubepilot_firmware_update_resp_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint32_t offset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_LEN];
    _mav_put_uint32_t(buf, 0, offset);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_LEN);
#else
    mavlink_cubepilot_firmware_update_resp_t packet;
    packet.offset = offset;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_MIN_LEN, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_LEN, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_CRC);
}

/**
 * @brief Encode a cubepilot_firmware_update_resp struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param cubepilot_firmware_update_resp C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cubepilot_firmware_update_resp_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_cubepilot_firmware_update_resp_t* cubepilot_firmware_update_resp)
{
    return mavlink_msg_cubepilot_firmware_update_resp_pack(system_id, component_id, msg, cubepilot_firmware_update_resp->target_system, cubepilot_firmware_update_resp->target_component, cubepilot_firmware_update_resp->offset);
}

/**
 * @brief Encode a cubepilot_firmware_update_resp struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param cubepilot_firmware_update_resp C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cubepilot_firmware_update_resp_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_cubepilot_firmware_update_resp_t* cubepilot_firmware_update_resp)
{
    return mavlink_msg_cubepilot_firmware_update_resp_pack_chan(system_id, component_id, chan, msg, cubepilot_firmware_update_resp->target_system, cubepilot_firmware_update_resp->target_component, cubepilot_firmware_update_resp->offset);
}

/**
 * @brief Encode a cubepilot_firmware_update_resp struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param cubepilot_firmware_update_resp C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cubepilot_firmware_update_resp_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_cubepilot_firmware_update_resp_t* cubepilot_firmware_update_resp)
{
    return mavlink_msg_cubepilot_firmware_update_resp_pack_status(system_id, component_id, _status, msg,  cubepilot_firmware_update_resp->target_system, cubepilot_firmware_update_resp->target_component, cubepilot_firmware_update_resp->offset);
}

/**
 * @brief Send a cubepilot_firmware_update_resp message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param offset [bytes] FW Offset.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_cubepilot_firmware_update_resp_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint32_t offset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_LEN];
    _mav_put_uint32_t(buf, 0, offset);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP, buf, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_MIN_LEN, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_LEN, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_CRC);
#else
    mavlink_cubepilot_firmware_update_resp_t packet;
    packet.offset = offset;
    packet.target_system = target_system;
    packet.target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP, (const char *)&packet, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_MIN_LEN, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_LEN, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_CRC);
#endif
}

/**
 * @brief Send a cubepilot_firmware_update_resp message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_cubepilot_firmware_update_resp_send_struct(mavlink_channel_t chan, const mavlink_cubepilot_firmware_update_resp_t* cubepilot_firmware_update_resp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_cubepilot_firmware_update_resp_send(chan, cubepilot_firmware_update_resp->target_system, cubepilot_firmware_update_resp->target_component, cubepilot_firmware_update_resp->offset);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP, (const char *)cubepilot_firmware_update_resp, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_MIN_LEN, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_LEN, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_CRC);
#endif
}

#if MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_cubepilot_firmware_update_resp_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint32_t offset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, offset);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP, buf, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_MIN_LEN, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_LEN, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_CRC);
#else
    mavlink_cubepilot_firmware_update_resp_t *packet = (mavlink_cubepilot_firmware_update_resp_t *)msgbuf;
    packet->offset = offset;
    packet->target_system = target_system;
    packet->target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP, (const char *)packet, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_MIN_LEN, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_LEN, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_CRC);
#endif
}
#endif

#endif

// MESSAGE CUBEPILOT_FIRMWARE_UPDATE_RESP UNPACKING


/**
 * @brief Get field target_system from cubepilot_firmware_update_resp message
 *
 * @return  System ID.
 */
static inline uint8_t mavlink_msg_cubepilot_firmware_update_resp_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field target_component from cubepilot_firmware_update_resp message
 *
 * @return  Component ID.
 */
static inline uint8_t mavlink_msg_cubepilot_firmware_update_resp_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field offset from cubepilot_firmware_update_resp message
 *
 * @return [bytes] FW Offset.
 */
static inline uint32_t mavlink_msg_cubepilot_firmware_update_resp_get_offset(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Decode a cubepilot_firmware_update_resp message into a struct
 *
 * @param msg The message to decode
 * @param cubepilot_firmware_update_resp C-struct to decode the message contents into
 */
static inline void mavlink_msg_cubepilot_firmware_update_resp_decode(const mavlink_message_t* msg, mavlink_cubepilot_firmware_update_resp_t* cubepilot_firmware_update_resp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    cubepilot_firmware_update_resp->offset = mavlink_msg_cubepilot_firmware_update_resp_get_offset(msg);
    cubepilot_firmware_update_resp->target_system = mavlink_msg_cubepilot_firmware_update_resp_get_target_system(msg);
    cubepilot_firmware_update_resp->target_component = mavlink_msg_cubepilot_firmware_update_resp_get_target_component(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_LEN? msg->len : MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_LEN;
        memset(cubepilot_firmware_update_resp, 0, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP_LEN);
    memcpy(cubepilot_firmware_update_resp, _MAV_PAYLOAD(msg), len);
#endif
}
