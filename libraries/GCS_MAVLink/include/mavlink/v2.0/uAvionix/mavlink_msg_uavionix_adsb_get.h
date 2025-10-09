#pragma once
// MESSAGE UAVIONIX_ADSB_GET PACKING

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_GET 10006


typedef struct __mavlink_uavionix_adsb_get_t {
 uint32_t ReqMessageId; /*<  Message ID to request. Supports any message in this 10000-10099 range*/
} mavlink_uavionix_adsb_get_t;

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_LEN 4
#define MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_MIN_LEN 4
#define MAVLINK_MSG_ID_10006_LEN 4
#define MAVLINK_MSG_ID_10006_MIN_LEN 4

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_CRC 193
#define MAVLINK_MSG_ID_10006_CRC 193



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_UAVIONIX_ADSB_GET { \
    10006, \
    "UAVIONIX_ADSB_GET", \
    1, \
    {  { "ReqMessageId", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_uavionix_adsb_get_t, ReqMessageId) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_UAVIONIX_ADSB_GET { \
    "UAVIONIX_ADSB_GET", \
    1, \
    {  { "ReqMessageId", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_uavionix_adsb_get_t, ReqMessageId) }, \
         } \
}
#endif

/**
 * @brief Pack a uavionix_adsb_get message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param ReqMessageId  Message ID to request. Supports any message in this 10000-10099 range
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uavionix_adsb_get_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t ReqMessageId)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_LEN];
    _mav_put_uint32_t(buf, 0, ReqMessageId);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_LEN);
#else
    mavlink_uavionix_adsb_get_t packet;
    packet.ReqMessageId = ReqMessageId;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAVIONIX_ADSB_GET;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_CRC);
}

/**
 * @brief Pack a uavionix_adsb_get message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param ReqMessageId  Message ID to request. Supports any message in this 10000-10099 range
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uavionix_adsb_get_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t ReqMessageId)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_LEN];
    _mav_put_uint32_t(buf, 0, ReqMessageId);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_LEN);
#else
    mavlink_uavionix_adsb_get_t packet;
    packet.ReqMessageId = ReqMessageId;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAVIONIX_ADSB_GET;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_LEN);
#endif
}

/**
 * @brief Pack a uavionix_adsb_get message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ReqMessageId  Message ID to request. Supports any message in this 10000-10099 range
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uavionix_adsb_get_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t ReqMessageId)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_LEN];
    _mav_put_uint32_t(buf, 0, ReqMessageId);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_LEN);
#else
    mavlink_uavionix_adsb_get_t packet;
    packet.ReqMessageId = ReqMessageId;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAVIONIX_ADSB_GET;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_CRC);
}

/**
 * @brief Encode a uavionix_adsb_get struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param uavionix_adsb_get C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uavionix_adsb_get_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_uavionix_adsb_get_t* uavionix_adsb_get)
{
    return mavlink_msg_uavionix_adsb_get_pack(system_id, component_id, msg, uavionix_adsb_get->ReqMessageId);
}

/**
 * @brief Encode a uavionix_adsb_get struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param uavionix_adsb_get C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uavionix_adsb_get_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_uavionix_adsb_get_t* uavionix_adsb_get)
{
    return mavlink_msg_uavionix_adsb_get_pack_chan(system_id, component_id, chan, msg, uavionix_adsb_get->ReqMessageId);
}

/**
 * @brief Encode a uavionix_adsb_get struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param uavionix_adsb_get C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uavionix_adsb_get_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_uavionix_adsb_get_t* uavionix_adsb_get)
{
    return mavlink_msg_uavionix_adsb_get_pack_status(system_id, component_id, _status, msg,  uavionix_adsb_get->ReqMessageId);
}

/**
 * @brief Send a uavionix_adsb_get message
 * @param chan MAVLink channel to send the message
 *
 * @param ReqMessageId  Message ID to request. Supports any message in this 10000-10099 range
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_uavionix_adsb_get_send(mavlink_channel_t chan, uint32_t ReqMessageId)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_LEN];
    _mav_put_uint32_t(buf, 0, ReqMessageId);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET, buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_CRC);
#else
    mavlink_uavionix_adsb_get_t packet;
    packet.ReqMessageId = ReqMessageId;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET, (const char *)&packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_CRC);
#endif
}

/**
 * @brief Send a uavionix_adsb_get message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_uavionix_adsb_get_send_struct(mavlink_channel_t chan, const mavlink_uavionix_adsb_get_t* uavionix_adsb_get)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_uavionix_adsb_get_send(chan, uavionix_adsb_get->ReqMessageId);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET, (const char *)uavionix_adsb_get, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_CRC);
#endif
}

#if MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_uavionix_adsb_get_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t ReqMessageId)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, ReqMessageId);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET, buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_CRC);
#else
    mavlink_uavionix_adsb_get_t *packet = (mavlink_uavionix_adsb_get_t *)msgbuf;
    packet->ReqMessageId = ReqMessageId;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET, (const char *)packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_CRC);
#endif
}
#endif

#endif

// MESSAGE UAVIONIX_ADSB_GET UNPACKING


/**
 * @brief Get field ReqMessageId from uavionix_adsb_get message
 *
 * @return  Message ID to request. Supports any message in this 10000-10099 range
 */
static inline uint32_t mavlink_msg_uavionix_adsb_get_get_ReqMessageId(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Decode a uavionix_adsb_get message into a struct
 *
 * @param msg The message to decode
 * @param uavionix_adsb_get C-struct to decode the message contents into
 */
static inline void mavlink_msg_uavionix_adsb_get_decode(const mavlink_message_t* msg, mavlink_uavionix_adsb_get_t* uavionix_adsb_get)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    uavionix_adsb_get->ReqMessageId = mavlink_msg_uavionix_adsb_get_get_ReqMessageId(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_LEN? msg->len : MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_LEN;
        memset(uavionix_adsb_get, 0, MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_LEN);
    memcpy(uavionix_adsb_get, _MAV_PAYLOAD(msg), len);
#endif
}
