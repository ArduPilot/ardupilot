#pragma once
// MESSAGE PLAY_TUNE PACKING

#define MAVLINK_MSG_ID_PLAY_TUNE 258


typedef struct __mavlink_play_tune_t {
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
 char tune[30]; /*<  tune in board specific format*/
 char tune2[200]; /*<  tune extension (appended to tune)*/
} mavlink_play_tune_t;

#define MAVLINK_MSG_ID_PLAY_TUNE_LEN 232
#define MAVLINK_MSG_ID_PLAY_TUNE_MIN_LEN 32
#define MAVLINK_MSG_ID_258_LEN 232
#define MAVLINK_MSG_ID_258_MIN_LEN 32

#define MAVLINK_MSG_ID_PLAY_TUNE_CRC 187
#define MAVLINK_MSG_ID_258_CRC 187

#define MAVLINK_MSG_PLAY_TUNE_FIELD_TUNE_LEN 30
#define MAVLINK_MSG_PLAY_TUNE_FIELD_TUNE2_LEN 200

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PLAY_TUNE { \
    258, \
    "PLAY_TUNE", \
    4, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_play_tune_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_play_tune_t, target_component) }, \
         { "tune", NULL, MAVLINK_TYPE_CHAR, 30, 2, offsetof(mavlink_play_tune_t, tune) }, \
         { "tune2", NULL, MAVLINK_TYPE_CHAR, 200, 32, offsetof(mavlink_play_tune_t, tune2) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PLAY_TUNE { \
    "PLAY_TUNE", \
    4, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_play_tune_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_play_tune_t, target_component) }, \
         { "tune", NULL, MAVLINK_TYPE_CHAR, 30, 2, offsetof(mavlink_play_tune_t, tune) }, \
         { "tune2", NULL, MAVLINK_TYPE_CHAR, 200, 32, offsetof(mavlink_play_tune_t, tune2) }, \
         } \
}
#endif

/**
 * @brief Pack a play_tune message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param tune  tune in board specific format
 * @param tune2  tune extension (appended to tune)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_play_tune_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, const char *tune, const char *tune2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PLAY_TUNE_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_char_array(buf, 2, tune, 30);
    _mav_put_char_array(buf, 32, tune2, 200);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PLAY_TUNE_LEN);
#else
    mavlink_play_tune_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.tune, tune, sizeof(char)*30);
    mav_array_memcpy(packet.tune2, tune2, sizeof(char)*200);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PLAY_TUNE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PLAY_TUNE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PLAY_TUNE_MIN_LEN, MAVLINK_MSG_ID_PLAY_TUNE_LEN, MAVLINK_MSG_ID_PLAY_TUNE_CRC);
}

/**
 * @brief Pack a play_tune message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param tune  tune in board specific format
 * @param tune2  tune extension (appended to tune)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_play_tune_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, const char *tune, const char *tune2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PLAY_TUNE_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_char_array(buf, 2, tune, 30);
    _mav_put_char_array(buf, 32, tune2, 200);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PLAY_TUNE_LEN);
#else
    mavlink_play_tune_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.tune, tune, sizeof(char)*30);
    mav_array_memcpy(packet.tune2, tune2, sizeof(char)*200);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PLAY_TUNE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PLAY_TUNE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_PLAY_TUNE_MIN_LEN, MAVLINK_MSG_ID_PLAY_TUNE_LEN, MAVLINK_MSG_ID_PLAY_TUNE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_PLAY_TUNE_MIN_LEN, MAVLINK_MSG_ID_PLAY_TUNE_LEN);
#endif
}

/**
 * @brief Pack a play_tune message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param tune  tune in board specific format
 * @param tune2  tune extension (appended to tune)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_play_tune_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,const char *tune,const char *tune2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PLAY_TUNE_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_char_array(buf, 2, tune, 30);
    _mav_put_char_array(buf, 32, tune2, 200);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PLAY_TUNE_LEN);
#else
    mavlink_play_tune_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.tune, tune, sizeof(char)*30);
    mav_array_memcpy(packet.tune2, tune2, sizeof(char)*200);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PLAY_TUNE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PLAY_TUNE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PLAY_TUNE_MIN_LEN, MAVLINK_MSG_ID_PLAY_TUNE_LEN, MAVLINK_MSG_ID_PLAY_TUNE_CRC);
}

/**
 * @brief Encode a play_tune struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param play_tune C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_play_tune_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_play_tune_t* play_tune)
{
    return mavlink_msg_play_tune_pack(system_id, component_id, msg, play_tune->target_system, play_tune->target_component, play_tune->tune, play_tune->tune2);
}

/**
 * @brief Encode a play_tune struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param play_tune C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_play_tune_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_play_tune_t* play_tune)
{
    return mavlink_msg_play_tune_pack_chan(system_id, component_id, chan, msg, play_tune->target_system, play_tune->target_component, play_tune->tune, play_tune->tune2);
}

/**
 * @brief Encode a play_tune struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param play_tune C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_play_tune_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_play_tune_t* play_tune)
{
    return mavlink_msg_play_tune_pack_status(system_id, component_id, _status, msg,  play_tune->target_system, play_tune->target_component, play_tune->tune, play_tune->tune2);
}

/**
 * @brief Send a play_tune message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param tune  tune in board specific format
 * @param tune2  tune extension (appended to tune)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_play_tune_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, const char *tune, const char *tune2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PLAY_TUNE_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_char_array(buf, 2, tune, 30);
    _mav_put_char_array(buf, 32, tune2, 200);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PLAY_TUNE, buf, MAVLINK_MSG_ID_PLAY_TUNE_MIN_LEN, MAVLINK_MSG_ID_PLAY_TUNE_LEN, MAVLINK_MSG_ID_PLAY_TUNE_CRC);
#else
    mavlink_play_tune_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.tune, tune, sizeof(char)*30);
    mav_array_memcpy(packet.tune2, tune2, sizeof(char)*200);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PLAY_TUNE, (const char *)&packet, MAVLINK_MSG_ID_PLAY_TUNE_MIN_LEN, MAVLINK_MSG_ID_PLAY_TUNE_LEN, MAVLINK_MSG_ID_PLAY_TUNE_CRC);
#endif
}

/**
 * @brief Send a play_tune message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_play_tune_send_struct(mavlink_channel_t chan, const mavlink_play_tune_t* play_tune)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_play_tune_send(chan, play_tune->target_system, play_tune->target_component, play_tune->tune, play_tune->tune2);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PLAY_TUNE, (const char *)play_tune, MAVLINK_MSG_ID_PLAY_TUNE_MIN_LEN, MAVLINK_MSG_ID_PLAY_TUNE_LEN, MAVLINK_MSG_ID_PLAY_TUNE_CRC);
#endif
}

#if MAVLINK_MSG_ID_PLAY_TUNE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_play_tune_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, const char *tune, const char *tune2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_char_array(buf, 2, tune, 30);
    _mav_put_char_array(buf, 32, tune2, 200);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PLAY_TUNE, buf, MAVLINK_MSG_ID_PLAY_TUNE_MIN_LEN, MAVLINK_MSG_ID_PLAY_TUNE_LEN, MAVLINK_MSG_ID_PLAY_TUNE_CRC);
#else
    mavlink_play_tune_t *packet = (mavlink_play_tune_t *)msgbuf;
    packet->target_system = target_system;
    packet->target_component = target_component;
    mav_array_memcpy(packet->tune, tune, sizeof(char)*30);
    mav_array_memcpy(packet->tune2, tune2, sizeof(char)*200);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PLAY_TUNE, (const char *)packet, MAVLINK_MSG_ID_PLAY_TUNE_MIN_LEN, MAVLINK_MSG_ID_PLAY_TUNE_LEN, MAVLINK_MSG_ID_PLAY_TUNE_CRC);
#endif
}
#endif

#endif

// MESSAGE PLAY_TUNE UNPACKING


/**
 * @brief Get field target_system from play_tune message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_play_tune_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field target_component from play_tune message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_play_tune_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field tune from play_tune message
 *
 * @return  tune in board specific format
 */
static inline uint16_t mavlink_msg_play_tune_get_tune(const mavlink_message_t* msg, char *tune)
{
    return _MAV_RETURN_char_array(msg, tune, 30,  2);
}

/**
 * @brief Get field tune2 from play_tune message
 *
 * @return  tune extension (appended to tune)
 */
static inline uint16_t mavlink_msg_play_tune_get_tune2(const mavlink_message_t* msg, char *tune2)
{
    return _MAV_RETURN_char_array(msg, tune2, 200,  32);
}

/**
 * @brief Decode a play_tune message into a struct
 *
 * @param msg The message to decode
 * @param play_tune C-struct to decode the message contents into
 */
static inline void mavlink_msg_play_tune_decode(const mavlink_message_t* msg, mavlink_play_tune_t* play_tune)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    play_tune->target_system = mavlink_msg_play_tune_get_target_system(msg);
    play_tune->target_component = mavlink_msg_play_tune_get_target_component(msg);
    mavlink_msg_play_tune_get_tune(msg, play_tune->tune);
    mavlink_msg_play_tune_get_tune2(msg, play_tune->tune2);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PLAY_TUNE_LEN? msg->len : MAVLINK_MSG_ID_PLAY_TUNE_LEN;
        memset(play_tune, 0, MAVLINK_MSG_ID_PLAY_TUNE_LEN);
    memcpy(play_tune, _MAV_PAYLOAD(msg), len);
#endif
}
