#pragma once
// MESSAGE UAVIONIX_ADSB_OUT_CFG_REGISTRATION PACKING

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION 10004


typedef struct __mavlink_uavionix_adsb_out_cfg_registration_t {
 char registration[9]; /*<  Aircraft Registration (ASCII string A-Z, 0-9 only), e.g. "N8644B ". Trailing spaces (0x20) only. This is null-terminated.*/
} mavlink_uavionix_adsb_out_cfg_registration_t;

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_LEN 9
#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_MIN_LEN 9
#define MAVLINK_MSG_ID_10004_LEN 9
#define MAVLINK_MSG_ID_10004_MIN_LEN 9

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_CRC 133
#define MAVLINK_MSG_ID_10004_CRC 133

#define MAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_FIELD_REGISTRATION_LEN 9

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_UAVIONIX_ADSB_OUT_CFG_REGISTRATION { \
    10004, \
    "UAVIONIX_ADSB_OUT_CFG_REGISTRATION", \
    1, \
    {  { "registration", NULL, MAVLINK_TYPE_CHAR, 9, 0, offsetof(mavlink_uavionix_adsb_out_cfg_registration_t, registration) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_UAVIONIX_ADSB_OUT_CFG_REGISTRATION { \
    "UAVIONIX_ADSB_OUT_CFG_REGISTRATION", \
    1, \
    {  { "registration", NULL, MAVLINK_TYPE_CHAR, 9, 0, offsetof(mavlink_uavionix_adsb_out_cfg_registration_t, registration) }, \
         } \
}
#endif

/**
 * @brief Pack a uavionix_adsb_out_cfg_registration message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param registration  Aircraft Registration (ASCII string A-Z, 0-9 only), e.g. "N8644B ". Trailing spaces (0x20) only. This is null-terminated.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_cfg_registration_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const char *registration)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_LEN];

    _mav_put_char_array(buf, 0, registration, 9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_LEN);
#else
    mavlink_uavionix_adsb_out_cfg_registration_t packet;

    mav_array_memcpy(packet.registration, registration, sizeof(char)*9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_CRC);
}

/**
 * @brief Pack a uavionix_adsb_out_cfg_registration message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param registration  Aircraft Registration (ASCII string A-Z, 0-9 only), e.g. "N8644B ". Trailing spaces (0x20) only. This is null-terminated.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_cfg_registration_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               const char *registration)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_LEN];

    _mav_put_char_array(buf, 0, registration, 9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_LEN);
#else
    mavlink_uavionix_adsb_out_cfg_registration_t packet;

    mav_array_memcpy(packet.registration, registration, sizeof(char)*9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_LEN);
#endif
}

/**
 * @brief Pack a uavionix_adsb_out_cfg_registration message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param registration  Aircraft Registration (ASCII string A-Z, 0-9 only), e.g. "N8644B ". Trailing spaces (0x20) only. This is null-terminated.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_cfg_registration_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const char *registration)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_LEN];

    _mav_put_char_array(buf, 0, registration, 9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_LEN);
#else
    mavlink_uavionix_adsb_out_cfg_registration_t packet;

    mav_array_memcpy(packet.registration, registration, sizeof(char)*9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_CRC);
}

/**
 * @brief Encode a uavionix_adsb_out_cfg_registration struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param uavionix_adsb_out_cfg_registration C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_cfg_registration_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_uavionix_adsb_out_cfg_registration_t* uavionix_adsb_out_cfg_registration)
{
    return mavlink_msg_uavionix_adsb_out_cfg_registration_pack(system_id, component_id, msg, uavionix_adsb_out_cfg_registration->registration);
}

/**
 * @brief Encode a uavionix_adsb_out_cfg_registration struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param uavionix_adsb_out_cfg_registration C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_cfg_registration_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_uavionix_adsb_out_cfg_registration_t* uavionix_adsb_out_cfg_registration)
{
    return mavlink_msg_uavionix_adsb_out_cfg_registration_pack_chan(system_id, component_id, chan, msg, uavionix_adsb_out_cfg_registration->registration);
}

/**
 * @brief Encode a uavionix_adsb_out_cfg_registration struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param uavionix_adsb_out_cfg_registration C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_cfg_registration_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_uavionix_adsb_out_cfg_registration_t* uavionix_adsb_out_cfg_registration)
{
    return mavlink_msg_uavionix_adsb_out_cfg_registration_pack_status(system_id, component_id, _status, msg,  uavionix_adsb_out_cfg_registration->registration);
}

/**
 * @brief Send a uavionix_adsb_out_cfg_registration message
 * @param chan MAVLink channel to send the message
 *
 * @param registration  Aircraft Registration (ASCII string A-Z, 0-9 only), e.g. "N8644B ". Trailing spaces (0x20) only. This is null-terminated.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_uavionix_adsb_out_cfg_registration_send(mavlink_channel_t chan, const char *registration)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_LEN];

    _mav_put_char_array(buf, 0, registration, 9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION, buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_CRC);
#else
    mavlink_uavionix_adsb_out_cfg_registration_t packet;

    mav_array_memcpy(packet.registration, registration, sizeof(char)*9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION, (const char *)&packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_CRC);
#endif
}

/**
 * @brief Send a uavionix_adsb_out_cfg_registration message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_uavionix_adsb_out_cfg_registration_send_struct(mavlink_channel_t chan, const mavlink_uavionix_adsb_out_cfg_registration_t* uavionix_adsb_out_cfg_registration)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_uavionix_adsb_out_cfg_registration_send(chan, uavionix_adsb_out_cfg_registration->registration);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION, (const char *)uavionix_adsb_out_cfg_registration, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_CRC);
#endif
}

#if MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_uavionix_adsb_out_cfg_registration_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const char *registration)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_char_array(buf, 0, registration, 9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION, buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_CRC);
#else
    mavlink_uavionix_adsb_out_cfg_registration_t *packet = (mavlink_uavionix_adsb_out_cfg_registration_t *)msgbuf;

    mav_array_memcpy(packet->registration, registration, sizeof(char)*9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION, (const char *)packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_CRC);
#endif
}
#endif

#endif

// MESSAGE UAVIONIX_ADSB_OUT_CFG_REGISTRATION UNPACKING


/**
 * @brief Get field registration from uavionix_adsb_out_cfg_registration message
 *
 * @return  Aircraft Registration (ASCII string A-Z, 0-9 only), e.g. "N8644B ". Trailing spaces (0x20) only. This is null-terminated.
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_cfg_registration_get_registration(const mavlink_message_t* msg, char *registration)
{
    return _MAV_RETURN_char_array(msg, registration, 9,  0);
}

/**
 * @brief Decode a uavionix_adsb_out_cfg_registration message into a struct
 *
 * @param msg The message to decode
 * @param uavionix_adsb_out_cfg_registration C-struct to decode the message contents into
 */
static inline void mavlink_msg_uavionix_adsb_out_cfg_registration_decode(const mavlink_message_t* msg, mavlink_uavionix_adsb_out_cfg_registration_t* uavionix_adsb_out_cfg_registration)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_uavionix_adsb_out_cfg_registration_get_registration(msg, uavionix_adsb_out_cfg_registration->registration);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_LEN? msg->len : MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_LEN;
        memset(uavionix_adsb_out_cfg_registration, 0, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_LEN);
    memcpy(uavionix_adsb_out_cfg_registration, _MAV_PAYLOAD(msg), len);
#endif
}
