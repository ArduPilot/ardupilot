#pragma once
// MESSAGE WIFI_CONFIG_AP PACKING

#define MAVLINK_MSG_ID_WIFI_CONFIG_AP 299


typedef struct __mavlink_wifi_config_ap_t {
 char ssid[32]; /*<  Name of Wi-Fi network (SSID). Blank to leave it unchanged when setting. Current SSID when sent back as a response.*/
 char password[64]; /*<  Password. Blank for an open AP. MD5 hash when message is sent back as a response.*/
} mavlink_wifi_config_ap_t;

#define MAVLINK_MSG_ID_WIFI_CONFIG_AP_LEN 96
#define MAVLINK_MSG_ID_WIFI_CONFIG_AP_MIN_LEN 96
#define MAVLINK_MSG_ID_299_LEN 96
#define MAVLINK_MSG_ID_299_MIN_LEN 96

#define MAVLINK_MSG_ID_WIFI_CONFIG_AP_CRC 19
#define MAVLINK_MSG_ID_299_CRC 19

#define MAVLINK_MSG_WIFI_CONFIG_AP_FIELD_SSID_LEN 32
#define MAVLINK_MSG_WIFI_CONFIG_AP_FIELD_PASSWORD_LEN 64

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_WIFI_CONFIG_AP { \
    299, \
    "WIFI_CONFIG_AP", \
    2, \
    {  { "ssid", NULL, MAVLINK_TYPE_CHAR, 32, 0, offsetof(mavlink_wifi_config_ap_t, ssid) }, \
         { "password", NULL, MAVLINK_TYPE_CHAR, 64, 32, offsetof(mavlink_wifi_config_ap_t, password) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_WIFI_CONFIG_AP { \
    "WIFI_CONFIG_AP", \
    2, \
    {  { "ssid", NULL, MAVLINK_TYPE_CHAR, 32, 0, offsetof(mavlink_wifi_config_ap_t, ssid) }, \
         { "password", NULL, MAVLINK_TYPE_CHAR, 64, 32, offsetof(mavlink_wifi_config_ap_t, password) }, \
         } \
}
#endif

/**
 * @brief Pack a wifi_config_ap message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param ssid  Name of Wi-Fi network (SSID). Blank to leave it unchanged when setting. Current SSID when sent back as a response.
 * @param password  Password. Blank for an open AP. MD5 hash when message is sent back as a response.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_wifi_config_ap_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const char *ssid, const char *password)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WIFI_CONFIG_AP_LEN];

    _mav_put_char_array(buf, 0, ssid, 32);
    _mav_put_char_array(buf, 32, password, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WIFI_CONFIG_AP_LEN);
#else
    mavlink_wifi_config_ap_t packet;

    mav_array_memcpy(packet.ssid, ssid, sizeof(char)*32);
    mav_array_memcpy(packet.password, password, sizeof(char)*64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WIFI_CONFIG_AP_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WIFI_CONFIG_AP;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_WIFI_CONFIG_AP_MIN_LEN, MAVLINK_MSG_ID_WIFI_CONFIG_AP_LEN, MAVLINK_MSG_ID_WIFI_CONFIG_AP_CRC);
}

/**
 * @brief Pack a wifi_config_ap message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param ssid  Name of Wi-Fi network (SSID). Blank to leave it unchanged when setting. Current SSID when sent back as a response.
 * @param password  Password. Blank for an open AP. MD5 hash when message is sent back as a response.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_wifi_config_ap_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               const char *ssid, const char *password)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WIFI_CONFIG_AP_LEN];

    _mav_put_char_array(buf, 0, ssid, 32);
    _mav_put_char_array(buf, 32, password, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WIFI_CONFIG_AP_LEN);
#else
    mavlink_wifi_config_ap_t packet;

    mav_array_memcpy(packet.ssid, ssid, sizeof(char)*32);
    mav_array_memcpy(packet.password, password, sizeof(char)*64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WIFI_CONFIG_AP_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WIFI_CONFIG_AP;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_WIFI_CONFIG_AP_MIN_LEN, MAVLINK_MSG_ID_WIFI_CONFIG_AP_LEN, MAVLINK_MSG_ID_WIFI_CONFIG_AP_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_WIFI_CONFIG_AP_MIN_LEN, MAVLINK_MSG_ID_WIFI_CONFIG_AP_LEN);
#endif
}

/**
 * @brief Pack a wifi_config_ap message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ssid  Name of Wi-Fi network (SSID). Blank to leave it unchanged when setting. Current SSID when sent back as a response.
 * @param password  Password. Blank for an open AP. MD5 hash when message is sent back as a response.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_wifi_config_ap_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const char *ssid,const char *password)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WIFI_CONFIG_AP_LEN];

    _mav_put_char_array(buf, 0, ssid, 32);
    _mav_put_char_array(buf, 32, password, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WIFI_CONFIG_AP_LEN);
#else
    mavlink_wifi_config_ap_t packet;

    mav_array_memcpy(packet.ssid, ssid, sizeof(char)*32);
    mav_array_memcpy(packet.password, password, sizeof(char)*64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WIFI_CONFIG_AP_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WIFI_CONFIG_AP;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_WIFI_CONFIG_AP_MIN_LEN, MAVLINK_MSG_ID_WIFI_CONFIG_AP_LEN, MAVLINK_MSG_ID_WIFI_CONFIG_AP_CRC);
}

/**
 * @brief Encode a wifi_config_ap struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param wifi_config_ap C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_wifi_config_ap_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_wifi_config_ap_t* wifi_config_ap)
{
    return mavlink_msg_wifi_config_ap_pack(system_id, component_id, msg, wifi_config_ap->ssid, wifi_config_ap->password);
}

/**
 * @brief Encode a wifi_config_ap struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param wifi_config_ap C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_wifi_config_ap_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_wifi_config_ap_t* wifi_config_ap)
{
    return mavlink_msg_wifi_config_ap_pack_chan(system_id, component_id, chan, msg, wifi_config_ap->ssid, wifi_config_ap->password);
}

/**
 * @brief Encode a wifi_config_ap struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param wifi_config_ap C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_wifi_config_ap_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_wifi_config_ap_t* wifi_config_ap)
{
    return mavlink_msg_wifi_config_ap_pack_status(system_id, component_id, _status, msg,  wifi_config_ap->ssid, wifi_config_ap->password);
}

/**
 * @brief Send a wifi_config_ap message
 * @param chan MAVLink channel to send the message
 *
 * @param ssid  Name of Wi-Fi network (SSID). Blank to leave it unchanged when setting. Current SSID when sent back as a response.
 * @param password  Password. Blank for an open AP. MD5 hash when message is sent back as a response.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_wifi_config_ap_send(mavlink_channel_t chan, const char *ssid, const char *password)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WIFI_CONFIG_AP_LEN];

    _mav_put_char_array(buf, 0, ssid, 32);
    _mav_put_char_array(buf, 32, password, 64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WIFI_CONFIG_AP, buf, MAVLINK_MSG_ID_WIFI_CONFIG_AP_MIN_LEN, MAVLINK_MSG_ID_WIFI_CONFIG_AP_LEN, MAVLINK_MSG_ID_WIFI_CONFIG_AP_CRC);
#else
    mavlink_wifi_config_ap_t packet;

    mav_array_memcpy(packet.ssid, ssid, sizeof(char)*32);
    mav_array_memcpy(packet.password, password, sizeof(char)*64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WIFI_CONFIG_AP, (const char *)&packet, MAVLINK_MSG_ID_WIFI_CONFIG_AP_MIN_LEN, MAVLINK_MSG_ID_WIFI_CONFIG_AP_LEN, MAVLINK_MSG_ID_WIFI_CONFIG_AP_CRC);
#endif
}

/**
 * @brief Send a wifi_config_ap message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_wifi_config_ap_send_struct(mavlink_channel_t chan, const mavlink_wifi_config_ap_t* wifi_config_ap)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_wifi_config_ap_send(chan, wifi_config_ap->ssid, wifi_config_ap->password);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WIFI_CONFIG_AP, (const char *)wifi_config_ap, MAVLINK_MSG_ID_WIFI_CONFIG_AP_MIN_LEN, MAVLINK_MSG_ID_WIFI_CONFIG_AP_LEN, MAVLINK_MSG_ID_WIFI_CONFIG_AP_CRC);
#endif
}

#if MAVLINK_MSG_ID_WIFI_CONFIG_AP_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_wifi_config_ap_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const char *ssid, const char *password)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_char_array(buf, 0, ssid, 32);
    _mav_put_char_array(buf, 32, password, 64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WIFI_CONFIG_AP, buf, MAVLINK_MSG_ID_WIFI_CONFIG_AP_MIN_LEN, MAVLINK_MSG_ID_WIFI_CONFIG_AP_LEN, MAVLINK_MSG_ID_WIFI_CONFIG_AP_CRC);
#else
    mavlink_wifi_config_ap_t *packet = (mavlink_wifi_config_ap_t *)msgbuf;

    mav_array_memcpy(packet->ssid, ssid, sizeof(char)*32);
    mav_array_memcpy(packet->password, password, sizeof(char)*64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WIFI_CONFIG_AP, (const char *)packet, MAVLINK_MSG_ID_WIFI_CONFIG_AP_MIN_LEN, MAVLINK_MSG_ID_WIFI_CONFIG_AP_LEN, MAVLINK_MSG_ID_WIFI_CONFIG_AP_CRC);
#endif
}
#endif

#endif

// MESSAGE WIFI_CONFIG_AP UNPACKING


/**
 * @brief Get field ssid from wifi_config_ap message
 *
 * @return  Name of Wi-Fi network (SSID). Blank to leave it unchanged when setting. Current SSID when sent back as a response.
 */
static inline uint16_t mavlink_msg_wifi_config_ap_get_ssid(const mavlink_message_t* msg, char *ssid)
{
    return _MAV_RETURN_char_array(msg, ssid, 32,  0);
}

/**
 * @brief Get field password from wifi_config_ap message
 *
 * @return  Password. Blank for an open AP. MD5 hash when message is sent back as a response.
 */
static inline uint16_t mavlink_msg_wifi_config_ap_get_password(const mavlink_message_t* msg, char *password)
{
    return _MAV_RETURN_char_array(msg, password, 64,  32);
}

/**
 * @brief Decode a wifi_config_ap message into a struct
 *
 * @param msg The message to decode
 * @param wifi_config_ap C-struct to decode the message contents into
 */
static inline void mavlink_msg_wifi_config_ap_decode(const mavlink_message_t* msg, mavlink_wifi_config_ap_t* wifi_config_ap)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_wifi_config_ap_get_ssid(msg, wifi_config_ap->ssid);
    mavlink_msg_wifi_config_ap_get_password(msg, wifi_config_ap->password);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_WIFI_CONFIG_AP_LEN? msg->len : MAVLINK_MSG_ID_WIFI_CONFIG_AP_LEN;
        memset(wifi_config_ap, 0, MAVLINK_MSG_ID_WIFI_CONFIG_AP_LEN);
    memcpy(wifi_config_ap, _MAV_PAYLOAD(msg), len);
#endif
}
