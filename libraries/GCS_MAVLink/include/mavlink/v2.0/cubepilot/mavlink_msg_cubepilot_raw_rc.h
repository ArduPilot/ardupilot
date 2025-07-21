#pragma once
// MESSAGE CUBEPILOT_RAW_RC PACKING

#define MAVLINK_MSG_ID_CUBEPILOT_RAW_RC 50001


typedef struct __mavlink_cubepilot_raw_rc_t {
 uint8_t rc_raw[32]; /*<  */
} mavlink_cubepilot_raw_rc_t;

#define MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_LEN 32
#define MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_MIN_LEN 32
#define MAVLINK_MSG_ID_50001_LEN 32
#define MAVLINK_MSG_ID_50001_MIN_LEN 32

#define MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_CRC 246
#define MAVLINK_MSG_ID_50001_CRC 246

#define MAVLINK_MSG_CUBEPILOT_RAW_RC_FIELD_RC_RAW_LEN 32

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CUBEPILOT_RAW_RC { \
    50001, \
    "CUBEPILOT_RAW_RC", \
    1, \
    {  { "rc_raw", NULL, MAVLINK_TYPE_UINT8_T, 32, 0, offsetof(mavlink_cubepilot_raw_rc_t, rc_raw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CUBEPILOT_RAW_RC { \
    "CUBEPILOT_RAW_RC", \
    1, \
    {  { "rc_raw", NULL, MAVLINK_TYPE_UINT8_T, 32, 0, offsetof(mavlink_cubepilot_raw_rc_t, rc_raw) }, \
         } \
}
#endif

/**
 * @brief Pack a cubepilot_raw_rc message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param rc_raw  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cubepilot_raw_rc_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const uint8_t *rc_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_LEN];

    _mav_put_uint8_t_array(buf, 0, rc_raw, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_LEN);
#else
    mavlink_cubepilot_raw_rc_t packet;

    mav_array_memcpy(packet.rc_raw, rc_raw, sizeof(uint8_t)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CUBEPILOT_RAW_RC;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_MIN_LEN, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_LEN, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_CRC);
}

/**
 * @brief Pack a cubepilot_raw_rc message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param rc_raw  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cubepilot_raw_rc_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               const uint8_t *rc_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_LEN];

    _mav_put_uint8_t_array(buf, 0, rc_raw, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_LEN);
#else
    mavlink_cubepilot_raw_rc_t packet;

    mav_array_memcpy(packet.rc_raw, rc_raw, sizeof(uint8_t)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CUBEPILOT_RAW_RC;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_MIN_LEN, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_LEN, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_MIN_LEN, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_LEN);
#endif
}

/**
 * @brief Pack a cubepilot_raw_rc message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rc_raw  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cubepilot_raw_rc_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const uint8_t *rc_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_LEN];

    _mav_put_uint8_t_array(buf, 0, rc_raw, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_LEN);
#else
    mavlink_cubepilot_raw_rc_t packet;

    mav_array_memcpy(packet.rc_raw, rc_raw, sizeof(uint8_t)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CUBEPILOT_RAW_RC;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_MIN_LEN, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_LEN, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_CRC);
}

/**
 * @brief Encode a cubepilot_raw_rc struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param cubepilot_raw_rc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cubepilot_raw_rc_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_cubepilot_raw_rc_t* cubepilot_raw_rc)
{
    return mavlink_msg_cubepilot_raw_rc_pack(system_id, component_id, msg, cubepilot_raw_rc->rc_raw);
}

/**
 * @brief Encode a cubepilot_raw_rc struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param cubepilot_raw_rc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cubepilot_raw_rc_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_cubepilot_raw_rc_t* cubepilot_raw_rc)
{
    return mavlink_msg_cubepilot_raw_rc_pack_chan(system_id, component_id, chan, msg, cubepilot_raw_rc->rc_raw);
}

/**
 * @brief Encode a cubepilot_raw_rc struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param cubepilot_raw_rc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cubepilot_raw_rc_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_cubepilot_raw_rc_t* cubepilot_raw_rc)
{
    return mavlink_msg_cubepilot_raw_rc_pack_status(system_id, component_id, _status, msg,  cubepilot_raw_rc->rc_raw);
}

/**
 * @brief Send a cubepilot_raw_rc message
 * @param chan MAVLink channel to send the message
 *
 * @param rc_raw  
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_cubepilot_raw_rc_send(mavlink_channel_t chan, const uint8_t *rc_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_LEN];

    _mav_put_uint8_t_array(buf, 0, rc_raw, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC, buf, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_MIN_LEN, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_LEN, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_CRC);
#else
    mavlink_cubepilot_raw_rc_t packet;

    mav_array_memcpy(packet.rc_raw, rc_raw, sizeof(uint8_t)*32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC, (const char *)&packet, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_MIN_LEN, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_LEN, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_CRC);
#endif
}

/**
 * @brief Send a cubepilot_raw_rc message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_cubepilot_raw_rc_send_struct(mavlink_channel_t chan, const mavlink_cubepilot_raw_rc_t* cubepilot_raw_rc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_cubepilot_raw_rc_send(chan, cubepilot_raw_rc->rc_raw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC, (const char *)cubepilot_raw_rc, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_MIN_LEN, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_LEN, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_CRC);
#endif
}

#if MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_cubepilot_raw_rc_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const uint8_t *rc_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_uint8_t_array(buf, 0, rc_raw, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC, buf, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_MIN_LEN, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_LEN, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_CRC);
#else
    mavlink_cubepilot_raw_rc_t *packet = (mavlink_cubepilot_raw_rc_t *)msgbuf;

    mav_array_memcpy(packet->rc_raw, rc_raw, sizeof(uint8_t)*32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC, (const char *)packet, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_MIN_LEN, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_LEN, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_CRC);
#endif
}
#endif

#endif

// MESSAGE CUBEPILOT_RAW_RC UNPACKING


/**
 * @brief Get field rc_raw from cubepilot_raw_rc message
 *
 * @return  
 */
static inline uint16_t mavlink_msg_cubepilot_raw_rc_get_rc_raw(const mavlink_message_t* msg, uint8_t *rc_raw)
{
    return _MAV_RETURN_uint8_t_array(msg, rc_raw, 32,  0);
}

/**
 * @brief Decode a cubepilot_raw_rc message into a struct
 *
 * @param msg The message to decode
 * @param cubepilot_raw_rc C-struct to decode the message contents into
 */
static inline void mavlink_msg_cubepilot_raw_rc_decode(const mavlink_message_t* msg, mavlink_cubepilot_raw_rc_t* cubepilot_raw_rc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_cubepilot_raw_rc_get_rc_raw(msg, cubepilot_raw_rc->rc_raw);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_LEN? msg->len : MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_LEN;
        memset(cubepilot_raw_rc, 0, MAVLINK_MSG_ID_CUBEPILOT_RAW_RC_LEN);
    memcpy(cubepilot_raw_rc, _MAV_PAYLOAD(msg), len);
#endif
}
