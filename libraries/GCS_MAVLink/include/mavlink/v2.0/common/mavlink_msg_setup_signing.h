#pragma once
// MESSAGE SETUP_SIGNING PACKING

#define MAVLINK_MSG_ID_SETUP_SIGNING 256


typedef struct __mavlink_setup_signing_t {
 uint64_t initial_timestamp; /*<  initial timestamp*/
 uint8_t target_system; /*<  system id of the target*/
 uint8_t target_component; /*<  component ID of the target*/
 uint8_t secret_key[32]; /*<  signing key*/
} mavlink_setup_signing_t;

#define MAVLINK_MSG_ID_SETUP_SIGNING_LEN 42
#define MAVLINK_MSG_ID_SETUP_SIGNING_MIN_LEN 42
#define MAVLINK_MSG_ID_256_LEN 42
#define MAVLINK_MSG_ID_256_MIN_LEN 42

#define MAVLINK_MSG_ID_SETUP_SIGNING_CRC 71
#define MAVLINK_MSG_ID_256_CRC 71

#define MAVLINK_MSG_SETUP_SIGNING_FIELD_SECRET_KEY_LEN 32

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SETUP_SIGNING { \
    256, \
    "SETUP_SIGNING", \
    4, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_setup_signing_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_setup_signing_t, target_component) }, \
         { "secret_key", NULL, MAVLINK_TYPE_UINT8_T, 32, 10, offsetof(mavlink_setup_signing_t, secret_key) }, \
         { "initial_timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_setup_signing_t, initial_timestamp) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SETUP_SIGNING { \
    "SETUP_SIGNING", \
    4, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_setup_signing_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_setup_signing_t, target_component) }, \
         { "secret_key", NULL, MAVLINK_TYPE_UINT8_T, 32, 10, offsetof(mavlink_setup_signing_t, secret_key) }, \
         { "initial_timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_setup_signing_t, initial_timestamp) }, \
         } \
}
#endif

/**
 * @brief Pack a setup_signing message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  system id of the target
 * @param target_component  component ID of the target
 * @param secret_key  signing key
 * @param initial_timestamp  initial timestamp
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_setup_signing_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, const uint8_t *secret_key, uint64_t initial_timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SETUP_SIGNING_LEN];
    _mav_put_uint64_t(buf, 0, initial_timestamp);
    _mav_put_uint8_t(buf, 8, target_system);
    _mav_put_uint8_t(buf, 9, target_component);
    _mav_put_uint8_t_array(buf, 10, secret_key, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SETUP_SIGNING_LEN);
#else
    mavlink_setup_signing_t packet;
    packet.initial_timestamp = initial_timestamp;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.secret_key, secret_key, sizeof(uint8_t)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SETUP_SIGNING_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SETUP_SIGNING;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SETUP_SIGNING_MIN_LEN, MAVLINK_MSG_ID_SETUP_SIGNING_LEN, MAVLINK_MSG_ID_SETUP_SIGNING_CRC);
}

/**
 * @brief Pack a setup_signing message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  system id of the target
 * @param target_component  component ID of the target
 * @param secret_key  signing key
 * @param initial_timestamp  initial timestamp
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_setup_signing_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, const uint8_t *secret_key, uint64_t initial_timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SETUP_SIGNING_LEN];
    _mav_put_uint64_t(buf, 0, initial_timestamp);
    _mav_put_uint8_t(buf, 8, target_system);
    _mav_put_uint8_t(buf, 9, target_component);
    _mav_put_uint8_t_array(buf, 10, secret_key, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SETUP_SIGNING_LEN);
#else
    mavlink_setup_signing_t packet;
    packet.initial_timestamp = initial_timestamp;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.secret_key, secret_key, sizeof(uint8_t)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SETUP_SIGNING_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SETUP_SIGNING;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SETUP_SIGNING_MIN_LEN, MAVLINK_MSG_ID_SETUP_SIGNING_LEN, MAVLINK_MSG_ID_SETUP_SIGNING_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SETUP_SIGNING_MIN_LEN, MAVLINK_MSG_ID_SETUP_SIGNING_LEN);
#endif
}

/**
 * @brief Pack a setup_signing message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  system id of the target
 * @param target_component  component ID of the target
 * @param secret_key  signing key
 * @param initial_timestamp  initial timestamp
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_setup_signing_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,const uint8_t *secret_key,uint64_t initial_timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SETUP_SIGNING_LEN];
    _mav_put_uint64_t(buf, 0, initial_timestamp);
    _mav_put_uint8_t(buf, 8, target_system);
    _mav_put_uint8_t(buf, 9, target_component);
    _mav_put_uint8_t_array(buf, 10, secret_key, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SETUP_SIGNING_LEN);
#else
    mavlink_setup_signing_t packet;
    packet.initial_timestamp = initial_timestamp;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.secret_key, secret_key, sizeof(uint8_t)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SETUP_SIGNING_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SETUP_SIGNING;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SETUP_SIGNING_MIN_LEN, MAVLINK_MSG_ID_SETUP_SIGNING_LEN, MAVLINK_MSG_ID_SETUP_SIGNING_CRC);
}

/**
 * @brief Encode a setup_signing struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param setup_signing C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_setup_signing_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_setup_signing_t* setup_signing)
{
    return mavlink_msg_setup_signing_pack(system_id, component_id, msg, setup_signing->target_system, setup_signing->target_component, setup_signing->secret_key, setup_signing->initial_timestamp);
}

/**
 * @brief Encode a setup_signing struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param setup_signing C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_setup_signing_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_setup_signing_t* setup_signing)
{
    return mavlink_msg_setup_signing_pack_chan(system_id, component_id, chan, msg, setup_signing->target_system, setup_signing->target_component, setup_signing->secret_key, setup_signing->initial_timestamp);
}

/**
 * @brief Encode a setup_signing struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param setup_signing C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_setup_signing_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_setup_signing_t* setup_signing)
{
    return mavlink_msg_setup_signing_pack_status(system_id, component_id, _status, msg,  setup_signing->target_system, setup_signing->target_component, setup_signing->secret_key, setup_signing->initial_timestamp);
}

/**
 * @brief Send a setup_signing message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  system id of the target
 * @param target_component  component ID of the target
 * @param secret_key  signing key
 * @param initial_timestamp  initial timestamp
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_setup_signing_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, const uint8_t *secret_key, uint64_t initial_timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SETUP_SIGNING_LEN];
    _mav_put_uint64_t(buf, 0, initial_timestamp);
    _mav_put_uint8_t(buf, 8, target_system);
    _mav_put_uint8_t(buf, 9, target_component);
    _mav_put_uint8_t_array(buf, 10, secret_key, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SETUP_SIGNING, buf, MAVLINK_MSG_ID_SETUP_SIGNING_MIN_LEN, MAVLINK_MSG_ID_SETUP_SIGNING_LEN, MAVLINK_MSG_ID_SETUP_SIGNING_CRC);
#else
    mavlink_setup_signing_t packet;
    packet.initial_timestamp = initial_timestamp;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.secret_key, secret_key, sizeof(uint8_t)*32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SETUP_SIGNING, (const char *)&packet, MAVLINK_MSG_ID_SETUP_SIGNING_MIN_LEN, MAVLINK_MSG_ID_SETUP_SIGNING_LEN, MAVLINK_MSG_ID_SETUP_SIGNING_CRC);
#endif
}

/**
 * @brief Send a setup_signing message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_setup_signing_send_struct(mavlink_channel_t chan, const mavlink_setup_signing_t* setup_signing)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_setup_signing_send(chan, setup_signing->target_system, setup_signing->target_component, setup_signing->secret_key, setup_signing->initial_timestamp);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SETUP_SIGNING, (const char *)setup_signing, MAVLINK_MSG_ID_SETUP_SIGNING_MIN_LEN, MAVLINK_MSG_ID_SETUP_SIGNING_LEN, MAVLINK_MSG_ID_SETUP_SIGNING_CRC);
#endif
}

#if MAVLINK_MSG_ID_SETUP_SIGNING_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_setup_signing_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, const uint8_t *secret_key, uint64_t initial_timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, initial_timestamp);
    _mav_put_uint8_t(buf, 8, target_system);
    _mav_put_uint8_t(buf, 9, target_component);
    _mav_put_uint8_t_array(buf, 10, secret_key, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SETUP_SIGNING, buf, MAVLINK_MSG_ID_SETUP_SIGNING_MIN_LEN, MAVLINK_MSG_ID_SETUP_SIGNING_LEN, MAVLINK_MSG_ID_SETUP_SIGNING_CRC);
#else
    mavlink_setup_signing_t *packet = (mavlink_setup_signing_t *)msgbuf;
    packet->initial_timestamp = initial_timestamp;
    packet->target_system = target_system;
    packet->target_component = target_component;
    mav_array_memcpy(packet->secret_key, secret_key, sizeof(uint8_t)*32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SETUP_SIGNING, (const char *)packet, MAVLINK_MSG_ID_SETUP_SIGNING_MIN_LEN, MAVLINK_MSG_ID_SETUP_SIGNING_LEN, MAVLINK_MSG_ID_SETUP_SIGNING_CRC);
#endif
}
#endif

#endif

// MESSAGE SETUP_SIGNING UNPACKING


/**
 * @brief Get field target_system from setup_signing message
 *
 * @return  system id of the target
 */
static inline uint8_t mavlink_msg_setup_signing_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field target_component from setup_signing message
 *
 * @return  component ID of the target
 */
static inline uint8_t mavlink_msg_setup_signing_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field secret_key from setup_signing message
 *
 * @return  signing key
 */
static inline uint16_t mavlink_msg_setup_signing_get_secret_key(const mavlink_message_t* msg, uint8_t *secret_key)
{
    return _MAV_RETURN_uint8_t_array(msg, secret_key, 32,  10);
}

/**
 * @brief Get field initial_timestamp from setup_signing message
 *
 * @return  initial timestamp
 */
static inline uint64_t mavlink_msg_setup_signing_get_initial_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Decode a setup_signing message into a struct
 *
 * @param msg The message to decode
 * @param setup_signing C-struct to decode the message contents into
 */
static inline void mavlink_msg_setup_signing_decode(const mavlink_message_t* msg, mavlink_setup_signing_t* setup_signing)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    setup_signing->initial_timestamp = mavlink_msg_setup_signing_get_initial_timestamp(msg);
    setup_signing->target_system = mavlink_msg_setup_signing_get_target_system(msg);
    setup_signing->target_component = mavlink_msg_setup_signing_get_target_component(msg);
    mavlink_msg_setup_signing_get_secret_key(msg, setup_signing->secret_key);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SETUP_SIGNING_LEN? msg->len : MAVLINK_MSG_ID_SETUP_SIGNING_LEN;
        memset(setup_signing, 0, MAVLINK_MSG_ID_SETUP_SIGNING_LEN);
    memcpy(setup_signing, _MAV_PAYLOAD(msg), len);
#endif
}
