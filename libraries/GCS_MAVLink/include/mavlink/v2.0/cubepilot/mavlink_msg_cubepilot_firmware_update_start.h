#pragma once
// MESSAGE CUBEPILOT_FIRMWARE_UPDATE_START PACKING

#define MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START 50004


typedef struct __mavlink_cubepilot_firmware_update_start_t {
 uint32_t size; /*< [bytes] FW Size.*/
 uint32_t crc; /*<  FW CRC.*/
 uint8_t target_system; /*<  System ID.*/
 uint8_t target_component; /*<  Component ID.*/
} mavlink_cubepilot_firmware_update_start_t;

#define MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_LEN 10
#define MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_MIN_LEN 10
#define MAVLINK_MSG_ID_50004_LEN 10
#define MAVLINK_MSG_ID_50004_MIN_LEN 10

#define MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_CRC 240
#define MAVLINK_MSG_ID_50004_CRC 240



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CUBEPILOT_FIRMWARE_UPDATE_START { \
    50004, \
    "CUBEPILOT_FIRMWARE_UPDATE_START", \
    4, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_cubepilot_firmware_update_start_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_cubepilot_firmware_update_start_t, target_component) }, \
         { "size", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_cubepilot_firmware_update_start_t, size) }, \
         { "crc", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_cubepilot_firmware_update_start_t, crc) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CUBEPILOT_FIRMWARE_UPDATE_START { \
    "CUBEPILOT_FIRMWARE_UPDATE_START", \
    4, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_cubepilot_firmware_update_start_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_cubepilot_firmware_update_start_t, target_component) }, \
         { "size", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_cubepilot_firmware_update_start_t, size) }, \
         { "crc", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_cubepilot_firmware_update_start_t, crc) }, \
         } \
}
#endif

/**
 * @brief Pack a cubepilot_firmware_update_start message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param size [bytes] FW Size.
 * @param crc  FW CRC.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cubepilot_firmware_update_start_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint32_t size, uint32_t crc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_LEN];
    _mav_put_uint32_t(buf, 0, size);
    _mav_put_uint32_t(buf, 4, crc);
    _mav_put_uint8_t(buf, 8, target_system);
    _mav_put_uint8_t(buf, 9, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_LEN);
#else
    mavlink_cubepilot_firmware_update_start_t packet;
    packet.size = size;
    packet.crc = crc;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_MIN_LEN, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_LEN, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_CRC);
}

/**
 * @brief Pack a cubepilot_firmware_update_start message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param size [bytes] FW Size.
 * @param crc  FW CRC.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cubepilot_firmware_update_start_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint32_t size, uint32_t crc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_LEN];
    _mav_put_uint32_t(buf, 0, size);
    _mav_put_uint32_t(buf, 4, crc);
    _mav_put_uint8_t(buf, 8, target_system);
    _mav_put_uint8_t(buf, 9, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_LEN);
#else
    mavlink_cubepilot_firmware_update_start_t packet;
    packet.size = size;
    packet.crc = crc;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_MIN_LEN, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_LEN, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_MIN_LEN, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_LEN);
#endif
}

/**
 * @brief Pack a cubepilot_firmware_update_start message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param size [bytes] FW Size.
 * @param crc  FW CRC.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cubepilot_firmware_update_start_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint32_t size,uint32_t crc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_LEN];
    _mav_put_uint32_t(buf, 0, size);
    _mav_put_uint32_t(buf, 4, crc);
    _mav_put_uint8_t(buf, 8, target_system);
    _mav_put_uint8_t(buf, 9, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_LEN);
#else
    mavlink_cubepilot_firmware_update_start_t packet;
    packet.size = size;
    packet.crc = crc;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_MIN_LEN, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_LEN, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_CRC);
}

/**
 * @brief Encode a cubepilot_firmware_update_start struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param cubepilot_firmware_update_start C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cubepilot_firmware_update_start_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_cubepilot_firmware_update_start_t* cubepilot_firmware_update_start)
{
    return mavlink_msg_cubepilot_firmware_update_start_pack(system_id, component_id, msg, cubepilot_firmware_update_start->target_system, cubepilot_firmware_update_start->target_component, cubepilot_firmware_update_start->size, cubepilot_firmware_update_start->crc);
}

/**
 * @brief Encode a cubepilot_firmware_update_start struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param cubepilot_firmware_update_start C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cubepilot_firmware_update_start_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_cubepilot_firmware_update_start_t* cubepilot_firmware_update_start)
{
    return mavlink_msg_cubepilot_firmware_update_start_pack_chan(system_id, component_id, chan, msg, cubepilot_firmware_update_start->target_system, cubepilot_firmware_update_start->target_component, cubepilot_firmware_update_start->size, cubepilot_firmware_update_start->crc);
}

/**
 * @brief Encode a cubepilot_firmware_update_start struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param cubepilot_firmware_update_start C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cubepilot_firmware_update_start_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_cubepilot_firmware_update_start_t* cubepilot_firmware_update_start)
{
    return mavlink_msg_cubepilot_firmware_update_start_pack_status(system_id, component_id, _status, msg,  cubepilot_firmware_update_start->target_system, cubepilot_firmware_update_start->target_component, cubepilot_firmware_update_start->size, cubepilot_firmware_update_start->crc);
}

/**
 * @brief Send a cubepilot_firmware_update_start message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param size [bytes] FW Size.
 * @param crc  FW CRC.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_cubepilot_firmware_update_start_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint32_t size, uint32_t crc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_LEN];
    _mav_put_uint32_t(buf, 0, size);
    _mav_put_uint32_t(buf, 4, crc);
    _mav_put_uint8_t(buf, 8, target_system);
    _mav_put_uint8_t(buf, 9, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START, buf, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_MIN_LEN, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_LEN, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_CRC);
#else
    mavlink_cubepilot_firmware_update_start_t packet;
    packet.size = size;
    packet.crc = crc;
    packet.target_system = target_system;
    packet.target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START, (const char *)&packet, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_MIN_LEN, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_LEN, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_CRC);
#endif
}

/**
 * @brief Send a cubepilot_firmware_update_start message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_cubepilot_firmware_update_start_send_struct(mavlink_channel_t chan, const mavlink_cubepilot_firmware_update_start_t* cubepilot_firmware_update_start)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_cubepilot_firmware_update_start_send(chan, cubepilot_firmware_update_start->target_system, cubepilot_firmware_update_start->target_component, cubepilot_firmware_update_start->size, cubepilot_firmware_update_start->crc);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START, (const char *)cubepilot_firmware_update_start, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_MIN_LEN, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_LEN, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_CRC);
#endif
}

#if MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_cubepilot_firmware_update_start_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint32_t size, uint32_t crc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, size);
    _mav_put_uint32_t(buf, 4, crc);
    _mav_put_uint8_t(buf, 8, target_system);
    _mav_put_uint8_t(buf, 9, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START, buf, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_MIN_LEN, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_LEN, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_CRC);
#else
    mavlink_cubepilot_firmware_update_start_t *packet = (mavlink_cubepilot_firmware_update_start_t *)msgbuf;
    packet->size = size;
    packet->crc = crc;
    packet->target_system = target_system;
    packet->target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START, (const char *)packet, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_MIN_LEN, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_LEN, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_CRC);
#endif
}
#endif

#endif

// MESSAGE CUBEPILOT_FIRMWARE_UPDATE_START UNPACKING


/**
 * @brief Get field target_system from cubepilot_firmware_update_start message
 *
 * @return  System ID.
 */
static inline uint8_t mavlink_msg_cubepilot_firmware_update_start_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field target_component from cubepilot_firmware_update_start message
 *
 * @return  Component ID.
 */
static inline uint8_t mavlink_msg_cubepilot_firmware_update_start_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field size from cubepilot_firmware_update_start message
 *
 * @return [bytes] FW Size.
 */
static inline uint32_t mavlink_msg_cubepilot_firmware_update_start_get_size(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field crc from cubepilot_firmware_update_start message
 *
 * @return  FW CRC.
 */
static inline uint32_t mavlink_msg_cubepilot_firmware_update_start_get_crc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Decode a cubepilot_firmware_update_start message into a struct
 *
 * @param msg The message to decode
 * @param cubepilot_firmware_update_start C-struct to decode the message contents into
 */
static inline void mavlink_msg_cubepilot_firmware_update_start_decode(const mavlink_message_t* msg, mavlink_cubepilot_firmware_update_start_t* cubepilot_firmware_update_start)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    cubepilot_firmware_update_start->size = mavlink_msg_cubepilot_firmware_update_start_get_size(msg);
    cubepilot_firmware_update_start->crc = mavlink_msg_cubepilot_firmware_update_start_get_crc(msg);
    cubepilot_firmware_update_start->target_system = mavlink_msg_cubepilot_firmware_update_start_get_target_system(msg);
    cubepilot_firmware_update_start->target_component = mavlink_msg_cubepilot_firmware_update_start_get_target_component(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_LEN? msg->len : MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_LEN;
        memset(cubepilot_firmware_update_start, 0, MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_LEN);
    memcpy(cubepilot_firmware_update_start, _MAV_PAYLOAD(msg), len);
#endif
}
