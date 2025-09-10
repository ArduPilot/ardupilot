#pragma once
// MESSAGE UAVIONIX_ADSB_OUT_CFG_FLIGHTID PACKING

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID 10005


typedef struct __mavlink_uavionix_adsb_out_cfg_flightid_t {
 char flight_id[9]; /*<  Flight Identification: 8 ASCII characters, '0' through '9', 'A' through 'Z' or space. Spaces (0x20) used as a trailing pad character, or when call sign is unavailable. Reflects Control message setting. This is null-terminated.*/
} mavlink_uavionix_adsb_out_cfg_flightid_t;

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_LEN 9
#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_MIN_LEN 9
#define MAVLINK_MSG_ID_10005_LEN 9
#define MAVLINK_MSG_ID_10005_MIN_LEN 9

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_CRC 103
#define MAVLINK_MSG_ID_10005_CRC 103

#define MAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_FIELD_FLIGHT_ID_LEN 9

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_UAVIONIX_ADSB_OUT_CFG_FLIGHTID { \
    10005, \
    "UAVIONIX_ADSB_OUT_CFG_FLIGHTID", \
    1, \
    {  { "flight_id", NULL, MAVLINK_TYPE_CHAR, 9, 0, offsetof(mavlink_uavionix_adsb_out_cfg_flightid_t, flight_id) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_UAVIONIX_ADSB_OUT_CFG_FLIGHTID { \
    "UAVIONIX_ADSB_OUT_CFG_FLIGHTID", \
    1, \
    {  { "flight_id", NULL, MAVLINK_TYPE_CHAR, 9, 0, offsetof(mavlink_uavionix_adsb_out_cfg_flightid_t, flight_id) }, \
         } \
}
#endif

/**
 * @brief Pack a uavionix_adsb_out_cfg_flightid message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param flight_id  Flight Identification: 8 ASCII characters, '0' through '9', 'A' through 'Z' or space. Spaces (0x20) used as a trailing pad character, or when call sign is unavailable. Reflects Control message setting. This is null-terminated.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_cfg_flightid_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const char *flight_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_LEN];

    _mav_put_char_array(buf, 0, flight_id, 9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_LEN);
#else
    mavlink_uavionix_adsb_out_cfg_flightid_t packet;

    mav_array_memcpy(packet.flight_id, flight_id, sizeof(char)*9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_CRC);
}

/**
 * @brief Pack a uavionix_adsb_out_cfg_flightid message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param flight_id  Flight Identification: 8 ASCII characters, '0' through '9', 'A' through 'Z' or space. Spaces (0x20) used as a trailing pad character, or when call sign is unavailable. Reflects Control message setting. This is null-terminated.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_cfg_flightid_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               const char *flight_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_LEN];

    _mav_put_char_array(buf, 0, flight_id, 9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_LEN);
#else
    mavlink_uavionix_adsb_out_cfg_flightid_t packet;

    mav_array_memcpy(packet.flight_id, flight_id, sizeof(char)*9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_LEN);
#endif
}

/**
 * @brief Pack a uavionix_adsb_out_cfg_flightid message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param flight_id  Flight Identification: 8 ASCII characters, '0' through '9', 'A' through 'Z' or space. Spaces (0x20) used as a trailing pad character, or when call sign is unavailable. Reflects Control message setting. This is null-terminated.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_cfg_flightid_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const char *flight_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_LEN];

    _mav_put_char_array(buf, 0, flight_id, 9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_LEN);
#else
    mavlink_uavionix_adsb_out_cfg_flightid_t packet;

    mav_array_memcpy(packet.flight_id, flight_id, sizeof(char)*9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_CRC);
}

/**
 * @brief Encode a uavionix_adsb_out_cfg_flightid struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param uavionix_adsb_out_cfg_flightid C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_cfg_flightid_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_uavionix_adsb_out_cfg_flightid_t* uavionix_adsb_out_cfg_flightid)
{
    return mavlink_msg_uavionix_adsb_out_cfg_flightid_pack(system_id, component_id, msg, uavionix_adsb_out_cfg_flightid->flight_id);
}

/**
 * @brief Encode a uavionix_adsb_out_cfg_flightid struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param uavionix_adsb_out_cfg_flightid C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_cfg_flightid_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_uavionix_adsb_out_cfg_flightid_t* uavionix_adsb_out_cfg_flightid)
{
    return mavlink_msg_uavionix_adsb_out_cfg_flightid_pack_chan(system_id, component_id, chan, msg, uavionix_adsb_out_cfg_flightid->flight_id);
}

/**
 * @brief Encode a uavionix_adsb_out_cfg_flightid struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param uavionix_adsb_out_cfg_flightid C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_cfg_flightid_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_uavionix_adsb_out_cfg_flightid_t* uavionix_adsb_out_cfg_flightid)
{
    return mavlink_msg_uavionix_adsb_out_cfg_flightid_pack_status(system_id, component_id, _status, msg,  uavionix_adsb_out_cfg_flightid->flight_id);
}

/**
 * @brief Send a uavionix_adsb_out_cfg_flightid message
 * @param chan MAVLink channel to send the message
 *
 * @param flight_id  Flight Identification: 8 ASCII characters, '0' through '9', 'A' through 'Z' or space. Spaces (0x20) used as a trailing pad character, or when call sign is unavailable. Reflects Control message setting. This is null-terminated.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_uavionix_adsb_out_cfg_flightid_send(mavlink_channel_t chan, const char *flight_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_LEN];

    _mav_put_char_array(buf, 0, flight_id, 9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID, buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_CRC);
#else
    mavlink_uavionix_adsb_out_cfg_flightid_t packet;

    mav_array_memcpy(packet.flight_id, flight_id, sizeof(char)*9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID, (const char *)&packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_CRC);
#endif
}

/**
 * @brief Send a uavionix_adsb_out_cfg_flightid message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_uavionix_adsb_out_cfg_flightid_send_struct(mavlink_channel_t chan, const mavlink_uavionix_adsb_out_cfg_flightid_t* uavionix_adsb_out_cfg_flightid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_uavionix_adsb_out_cfg_flightid_send(chan, uavionix_adsb_out_cfg_flightid->flight_id);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID, (const char *)uavionix_adsb_out_cfg_flightid, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_CRC);
#endif
}

#if MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_uavionix_adsb_out_cfg_flightid_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const char *flight_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_char_array(buf, 0, flight_id, 9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID, buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_CRC);
#else
    mavlink_uavionix_adsb_out_cfg_flightid_t *packet = (mavlink_uavionix_adsb_out_cfg_flightid_t *)msgbuf;

    mav_array_memcpy(packet->flight_id, flight_id, sizeof(char)*9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID, (const char *)packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_CRC);
#endif
}
#endif

#endif

// MESSAGE UAVIONIX_ADSB_OUT_CFG_FLIGHTID UNPACKING


/**
 * @brief Get field flight_id from uavionix_adsb_out_cfg_flightid message
 *
 * @return  Flight Identification: 8 ASCII characters, '0' through '9', 'A' through 'Z' or space. Spaces (0x20) used as a trailing pad character, or when call sign is unavailable. Reflects Control message setting. This is null-terminated.
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_cfg_flightid_get_flight_id(const mavlink_message_t* msg, char *flight_id)
{
    return _MAV_RETURN_char_array(msg, flight_id, 9,  0);
}

/**
 * @brief Decode a uavionix_adsb_out_cfg_flightid message into a struct
 *
 * @param msg The message to decode
 * @param uavionix_adsb_out_cfg_flightid C-struct to decode the message contents into
 */
static inline void mavlink_msg_uavionix_adsb_out_cfg_flightid_decode(const mavlink_message_t* msg, mavlink_uavionix_adsb_out_cfg_flightid_t* uavionix_adsb_out_cfg_flightid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_uavionix_adsb_out_cfg_flightid_get_flight_id(msg, uavionix_adsb_out_cfg_flightid->flight_id);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_LEN? msg->len : MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_LEN;
        memset(uavionix_adsb_out_cfg_flightid, 0, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_FLIGHTID_LEN);
    memcpy(uavionix_adsb_out_cfg_flightid, _MAV_PAYLOAD(msg), len);
#endif
}
