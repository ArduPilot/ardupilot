#pragma once
// MESSAGE AIRLINK_AUTH_RESPONSE PACKING

#define MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE 52001


typedef struct __mavlink_airlink_auth_response_t {
 uint8_t resp_type; /*<  Response type*/
} mavlink_airlink_auth_response_t;

#define MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_LEN 1
#define MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_MIN_LEN 1
#define MAVLINK_MSG_ID_52001_LEN 1
#define MAVLINK_MSG_ID_52001_MIN_LEN 1

#define MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_CRC 239
#define MAVLINK_MSG_ID_52001_CRC 239



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AIRLINK_AUTH_RESPONSE { \
    52001, \
    "AIRLINK_AUTH_RESPONSE", \
    1, \
    {  { "resp_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_airlink_auth_response_t, resp_type) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AIRLINK_AUTH_RESPONSE { \
    "AIRLINK_AUTH_RESPONSE", \
    1, \
    {  { "resp_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_airlink_auth_response_t, resp_type) }, \
         } \
}
#endif

/**
 * @brief Pack a airlink_auth_response message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param resp_type  Response type
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_airlink_auth_response_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t resp_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_LEN];
    _mav_put_uint8_t(buf, 0, resp_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_LEN);
#else
    mavlink_airlink_auth_response_t packet;
    packet.resp_type = resp_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_LEN, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_CRC);
}

/**
 * @brief Pack a airlink_auth_response message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param resp_type  Response type
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_airlink_auth_response_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t resp_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_LEN];
    _mav_put_uint8_t(buf, 0, resp_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_LEN);
#else
    mavlink_airlink_auth_response_t packet;
    packet.resp_type = resp_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_LEN, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_LEN);
#endif
}

/**
 * @brief Pack a airlink_auth_response message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param resp_type  Response type
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_airlink_auth_response_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t resp_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_LEN];
    _mav_put_uint8_t(buf, 0, resp_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_LEN);
#else
    mavlink_airlink_auth_response_t packet;
    packet.resp_type = resp_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_LEN, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_CRC);
}

/**
 * @brief Encode a airlink_auth_response struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param airlink_auth_response C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_airlink_auth_response_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_airlink_auth_response_t* airlink_auth_response)
{
    return mavlink_msg_airlink_auth_response_pack(system_id, component_id, msg, airlink_auth_response->resp_type);
}

/**
 * @brief Encode a airlink_auth_response struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param airlink_auth_response C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_airlink_auth_response_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_airlink_auth_response_t* airlink_auth_response)
{
    return mavlink_msg_airlink_auth_response_pack_chan(system_id, component_id, chan, msg, airlink_auth_response->resp_type);
}

/**
 * @brief Encode a airlink_auth_response struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param airlink_auth_response C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_airlink_auth_response_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_airlink_auth_response_t* airlink_auth_response)
{
    return mavlink_msg_airlink_auth_response_pack_status(system_id, component_id, _status, msg,  airlink_auth_response->resp_type);
}

/**
 * @brief Send a airlink_auth_response message
 * @param chan MAVLink channel to send the message
 *
 * @param resp_type  Response type
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_airlink_auth_response_send(mavlink_channel_t chan, uint8_t resp_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_LEN];
    _mav_put_uint8_t(buf, 0, resp_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE, buf, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_LEN, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_CRC);
#else
    mavlink_airlink_auth_response_t packet;
    packet.resp_type = resp_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE, (const char *)&packet, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_LEN, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_CRC);
#endif
}

/**
 * @brief Send a airlink_auth_response message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_airlink_auth_response_send_struct(mavlink_channel_t chan, const mavlink_airlink_auth_response_t* airlink_auth_response)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_airlink_auth_response_send(chan, airlink_auth_response->resp_type);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE, (const char *)airlink_auth_response, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_LEN, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_CRC);
#endif
}

#if MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_airlink_auth_response_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t resp_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, resp_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE, buf, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_LEN, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_CRC);
#else
    mavlink_airlink_auth_response_t *packet = (mavlink_airlink_auth_response_t *)msgbuf;
    packet->resp_type = resp_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE, (const char *)packet, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_LEN, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_CRC);
#endif
}
#endif

#endif

// MESSAGE AIRLINK_AUTH_RESPONSE UNPACKING


/**
 * @brief Get field resp_type from airlink_auth_response message
 *
 * @return  Response type
 */
static inline uint8_t mavlink_msg_airlink_auth_response_get_resp_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a airlink_auth_response message into a struct
 *
 * @param msg The message to decode
 * @param airlink_auth_response C-struct to decode the message contents into
 */
static inline void mavlink_msg_airlink_auth_response_decode(const mavlink_message_t* msg, mavlink_airlink_auth_response_t* airlink_auth_response)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    airlink_auth_response->resp_type = mavlink_msg_airlink_auth_response_get_resp_type(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_LEN? msg->len : MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_LEN;
        memset(airlink_auth_response, 0, MAVLINK_MSG_ID_AIRLINK_AUTH_RESPONSE_LEN);
    memcpy(airlink_auth_response, _MAV_PAYLOAD(msg), len);
#endif
}
