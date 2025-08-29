#pragma once
// MESSAGE GSM_LINK_STATUS PACKING

#define MAVLINK_MSG_ID_GSM_LINK_STATUS 8014


typedef struct __mavlink_gsm_link_status_t {
 uint64_t timestamp; /*< [us] Timestamp (of OBC)*/
 uint8_t gsm_modem_type; /*<  GSM modem used*/
 uint8_t gsm_link_type; /*<  GSM link type*/
 uint8_t rssi; /*<  RSSI as reported by modem (unconverted)*/
 uint8_t rsrp_rscp; /*<  RSRP (LTE) or RSCP (WCDMA) as reported by modem (unconverted)*/
 uint8_t sinr_ecio; /*<  SINR (LTE) or ECIO (WCDMA) as reported by modem (unconverted)*/
 uint8_t rsrq; /*<  RSRQ (LTE only) as reported by modem (unconverted)*/
} mavlink_gsm_link_status_t;

#define MAVLINK_MSG_ID_GSM_LINK_STATUS_LEN 14
#define MAVLINK_MSG_ID_GSM_LINK_STATUS_MIN_LEN 14
#define MAVLINK_MSG_ID_8014_LEN 14
#define MAVLINK_MSG_ID_8014_MIN_LEN 14

#define MAVLINK_MSG_ID_GSM_LINK_STATUS_CRC 200
#define MAVLINK_MSG_ID_8014_CRC 200



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GSM_LINK_STATUS { \
    8014, \
    "GSM_LINK_STATUS", \
    7, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_gsm_link_status_t, timestamp) }, \
         { "gsm_modem_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_gsm_link_status_t, gsm_modem_type) }, \
         { "gsm_link_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_gsm_link_status_t, gsm_link_type) }, \
         { "rssi", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_gsm_link_status_t, rssi) }, \
         { "rsrp_rscp", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_gsm_link_status_t, rsrp_rscp) }, \
         { "sinr_ecio", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_gsm_link_status_t, sinr_ecio) }, \
         { "rsrq", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_gsm_link_status_t, rsrq) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GSM_LINK_STATUS { \
    "GSM_LINK_STATUS", \
    7, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_gsm_link_status_t, timestamp) }, \
         { "gsm_modem_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_gsm_link_status_t, gsm_modem_type) }, \
         { "gsm_link_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_gsm_link_status_t, gsm_link_type) }, \
         { "rssi", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_gsm_link_status_t, rssi) }, \
         { "rsrp_rscp", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_gsm_link_status_t, rsrp_rscp) }, \
         { "sinr_ecio", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_gsm_link_status_t, sinr_ecio) }, \
         { "rsrq", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_gsm_link_status_t, rsrq) }, \
         } \
}
#endif

/**
 * @brief Pack a gsm_link_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp (of OBC)
 * @param gsm_modem_type  GSM modem used
 * @param gsm_link_type  GSM link type
 * @param rssi  RSSI as reported by modem (unconverted)
 * @param rsrp_rscp  RSRP (LTE) or RSCP (WCDMA) as reported by modem (unconverted)
 * @param sinr_ecio  SINR (LTE) or ECIO (WCDMA) as reported by modem (unconverted)
 * @param rsrq  RSRQ (LTE only) as reported by modem (unconverted)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gsm_link_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t gsm_modem_type, uint8_t gsm_link_type, uint8_t rssi, uint8_t rsrp_rscp, uint8_t sinr_ecio, uint8_t rsrq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GSM_LINK_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 8, gsm_modem_type);
    _mav_put_uint8_t(buf, 9, gsm_link_type);
    _mav_put_uint8_t(buf, 10, rssi);
    _mav_put_uint8_t(buf, 11, rsrp_rscp);
    _mav_put_uint8_t(buf, 12, sinr_ecio);
    _mav_put_uint8_t(buf, 13, rsrq);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GSM_LINK_STATUS_LEN);
#else
    mavlink_gsm_link_status_t packet;
    packet.timestamp = timestamp;
    packet.gsm_modem_type = gsm_modem_type;
    packet.gsm_link_type = gsm_link_type;
    packet.rssi = rssi;
    packet.rsrp_rscp = rsrp_rscp;
    packet.sinr_ecio = sinr_ecio;
    packet.rsrq = rsrq;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GSM_LINK_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GSM_LINK_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GSM_LINK_STATUS_MIN_LEN, MAVLINK_MSG_ID_GSM_LINK_STATUS_LEN, MAVLINK_MSG_ID_GSM_LINK_STATUS_CRC);
}

/**
 * @brief Pack a gsm_link_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp (of OBC)
 * @param gsm_modem_type  GSM modem used
 * @param gsm_link_type  GSM link type
 * @param rssi  RSSI as reported by modem (unconverted)
 * @param rsrp_rscp  RSRP (LTE) or RSCP (WCDMA) as reported by modem (unconverted)
 * @param sinr_ecio  SINR (LTE) or ECIO (WCDMA) as reported by modem (unconverted)
 * @param rsrq  RSRQ (LTE only) as reported by modem (unconverted)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gsm_link_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t gsm_modem_type, uint8_t gsm_link_type, uint8_t rssi, uint8_t rsrp_rscp, uint8_t sinr_ecio, uint8_t rsrq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GSM_LINK_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 8, gsm_modem_type);
    _mav_put_uint8_t(buf, 9, gsm_link_type);
    _mav_put_uint8_t(buf, 10, rssi);
    _mav_put_uint8_t(buf, 11, rsrp_rscp);
    _mav_put_uint8_t(buf, 12, sinr_ecio);
    _mav_put_uint8_t(buf, 13, rsrq);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GSM_LINK_STATUS_LEN);
#else
    mavlink_gsm_link_status_t packet;
    packet.timestamp = timestamp;
    packet.gsm_modem_type = gsm_modem_type;
    packet.gsm_link_type = gsm_link_type;
    packet.rssi = rssi;
    packet.rsrp_rscp = rsrp_rscp;
    packet.sinr_ecio = sinr_ecio;
    packet.rsrq = rsrq;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GSM_LINK_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GSM_LINK_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_GSM_LINK_STATUS_MIN_LEN, MAVLINK_MSG_ID_GSM_LINK_STATUS_LEN, MAVLINK_MSG_ID_GSM_LINK_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_GSM_LINK_STATUS_MIN_LEN, MAVLINK_MSG_ID_GSM_LINK_STATUS_LEN);
#endif
}

/**
 * @brief Pack a gsm_link_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp (of OBC)
 * @param gsm_modem_type  GSM modem used
 * @param gsm_link_type  GSM link type
 * @param rssi  RSSI as reported by modem (unconverted)
 * @param rsrp_rscp  RSRP (LTE) or RSCP (WCDMA) as reported by modem (unconverted)
 * @param sinr_ecio  SINR (LTE) or ECIO (WCDMA) as reported by modem (unconverted)
 * @param rsrq  RSRQ (LTE only) as reported by modem (unconverted)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gsm_link_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint8_t gsm_modem_type,uint8_t gsm_link_type,uint8_t rssi,uint8_t rsrp_rscp,uint8_t sinr_ecio,uint8_t rsrq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GSM_LINK_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 8, gsm_modem_type);
    _mav_put_uint8_t(buf, 9, gsm_link_type);
    _mav_put_uint8_t(buf, 10, rssi);
    _mav_put_uint8_t(buf, 11, rsrp_rscp);
    _mav_put_uint8_t(buf, 12, sinr_ecio);
    _mav_put_uint8_t(buf, 13, rsrq);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GSM_LINK_STATUS_LEN);
#else
    mavlink_gsm_link_status_t packet;
    packet.timestamp = timestamp;
    packet.gsm_modem_type = gsm_modem_type;
    packet.gsm_link_type = gsm_link_type;
    packet.rssi = rssi;
    packet.rsrp_rscp = rsrp_rscp;
    packet.sinr_ecio = sinr_ecio;
    packet.rsrq = rsrq;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GSM_LINK_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GSM_LINK_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GSM_LINK_STATUS_MIN_LEN, MAVLINK_MSG_ID_GSM_LINK_STATUS_LEN, MAVLINK_MSG_ID_GSM_LINK_STATUS_CRC);
}

/**
 * @brief Encode a gsm_link_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gsm_link_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gsm_link_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gsm_link_status_t* gsm_link_status)
{
    return mavlink_msg_gsm_link_status_pack(system_id, component_id, msg, gsm_link_status->timestamp, gsm_link_status->gsm_modem_type, gsm_link_status->gsm_link_type, gsm_link_status->rssi, gsm_link_status->rsrp_rscp, gsm_link_status->sinr_ecio, gsm_link_status->rsrq);
}

/**
 * @brief Encode a gsm_link_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gsm_link_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gsm_link_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gsm_link_status_t* gsm_link_status)
{
    return mavlink_msg_gsm_link_status_pack_chan(system_id, component_id, chan, msg, gsm_link_status->timestamp, gsm_link_status->gsm_modem_type, gsm_link_status->gsm_link_type, gsm_link_status->rssi, gsm_link_status->rsrp_rscp, gsm_link_status->sinr_ecio, gsm_link_status->rsrq);
}

/**
 * @brief Encode a gsm_link_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param gsm_link_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gsm_link_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_gsm_link_status_t* gsm_link_status)
{
    return mavlink_msg_gsm_link_status_pack_status(system_id, component_id, _status, msg,  gsm_link_status->timestamp, gsm_link_status->gsm_modem_type, gsm_link_status->gsm_link_type, gsm_link_status->rssi, gsm_link_status->rsrp_rscp, gsm_link_status->sinr_ecio, gsm_link_status->rsrq);
}

/**
 * @brief Send a gsm_link_status message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp (of OBC)
 * @param gsm_modem_type  GSM modem used
 * @param gsm_link_type  GSM link type
 * @param rssi  RSSI as reported by modem (unconverted)
 * @param rsrp_rscp  RSRP (LTE) or RSCP (WCDMA) as reported by modem (unconverted)
 * @param sinr_ecio  SINR (LTE) or ECIO (WCDMA) as reported by modem (unconverted)
 * @param rsrq  RSRQ (LTE only) as reported by modem (unconverted)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gsm_link_status_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t gsm_modem_type, uint8_t gsm_link_type, uint8_t rssi, uint8_t rsrp_rscp, uint8_t sinr_ecio, uint8_t rsrq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GSM_LINK_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 8, gsm_modem_type);
    _mav_put_uint8_t(buf, 9, gsm_link_type);
    _mav_put_uint8_t(buf, 10, rssi);
    _mav_put_uint8_t(buf, 11, rsrp_rscp);
    _mav_put_uint8_t(buf, 12, sinr_ecio);
    _mav_put_uint8_t(buf, 13, rsrq);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GSM_LINK_STATUS, buf, MAVLINK_MSG_ID_GSM_LINK_STATUS_MIN_LEN, MAVLINK_MSG_ID_GSM_LINK_STATUS_LEN, MAVLINK_MSG_ID_GSM_LINK_STATUS_CRC);
#else
    mavlink_gsm_link_status_t packet;
    packet.timestamp = timestamp;
    packet.gsm_modem_type = gsm_modem_type;
    packet.gsm_link_type = gsm_link_type;
    packet.rssi = rssi;
    packet.rsrp_rscp = rsrp_rscp;
    packet.sinr_ecio = sinr_ecio;
    packet.rsrq = rsrq;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GSM_LINK_STATUS, (const char *)&packet, MAVLINK_MSG_ID_GSM_LINK_STATUS_MIN_LEN, MAVLINK_MSG_ID_GSM_LINK_STATUS_LEN, MAVLINK_MSG_ID_GSM_LINK_STATUS_CRC);
#endif
}

/**
 * @brief Send a gsm_link_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gsm_link_status_send_struct(mavlink_channel_t chan, const mavlink_gsm_link_status_t* gsm_link_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gsm_link_status_send(chan, gsm_link_status->timestamp, gsm_link_status->gsm_modem_type, gsm_link_status->gsm_link_type, gsm_link_status->rssi, gsm_link_status->rsrp_rscp, gsm_link_status->sinr_ecio, gsm_link_status->rsrq);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GSM_LINK_STATUS, (const char *)gsm_link_status, MAVLINK_MSG_ID_GSM_LINK_STATUS_MIN_LEN, MAVLINK_MSG_ID_GSM_LINK_STATUS_LEN, MAVLINK_MSG_ID_GSM_LINK_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_GSM_LINK_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gsm_link_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t gsm_modem_type, uint8_t gsm_link_type, uint8_t rssi, uint8_t rsrp_rscp, uint8_t sinr_ecio, uint8_t rsrq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 8, gsm_modem_type);
    _mav_put_uint8_t(buf, 9, gsm_link_type);
    _mav_put_uint8_t(buf, 10, rssi);
    _mav_put_uint8_t(buf, 11, rsrp_rscp);
    _mav_put_uint8_t(buf, 12, sinr_ecio);
    _mav_put_uint8_t(buf, 13, rsrq);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GSM_LINK_STATUS, buf, MAVLINK_MSG_ID_GSM_LINK_STATUS_MIN_LEN, MAVLINK_MSG_ID_GSM_LINK_STATUS_LEN, MAVLINK_MSG_ID_GSM_LINK_STATUS_CRC);
#else
    mavlink_gsm_link_status_t *packet = (mavlink_gsm_link_status_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->gsm_modem_type = gsm_modem_type;
    packet->gsm_link_type = gsm_link_type;
    packet->rssi = rssi;
    packet->rsrp_rscp = rsrp_rscp;
    packet->sinr_ecio = sinr_ecio;
    packet->rsrq = rsrq;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GSM_LINK_STATUS, (const char *)packet, MAVLINK_MSG_ID_GSM_LINK_STATUS_MIN_LEN, MAVLINK_MSG_ID_GSM_LINK_STATUS_LEN, MAVLINK_MSG_ID_GSM_LINK_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE GSM_LINK_STATUS UNPACKING


/**
 * @brief Get field timestamp from gsm_link_status message
 *
 * @return [us] Timestamp (of OBC)
 */
static inline uint64_t mavlink_msg_gsm_link_status_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field gsm_modem_type from gsm_link_status message
 *
 * @return  GSM modem used
 */
static inline uint8_t mavlink_msg_gsm_link_status_get_gsm_modem_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field gsm_link_type from gsm_link_status message
 *
 * @return  GSM link type
 */
static inline uint8_t mavlink_msg_gsm_link_status_get_gsm_link_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field rssi from gsm_link_status message
 *
 * @return  RSSI as reported by modem (unconverted)
 */
static inline uint8_t mavlink_msg_gsm_link_status_get_rssi(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field rsrp_rscp from gsm_link_status message
 *
 * @return  RSRP (LTE) or RSCP (WCDMA) as reported by modem (unconverted)
 */
static inline uint8_t mavlink_msg_gsm_link_status_get_rsrp_rscp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field sinr_ecio from gsm_link_status message
 *
 * @return  SINR (LTE) or ECIO (WCDMA) as reported by modem (unconverted)
 */
static inline uint8_t mavlink_msg_gsm_link_status_get_sinr_ecio(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field rsrq from gsm_link_status message
 *
 * @return  RSRQ (LTE only) as reported by modem (unconverted)
 */
static inline uint8_t mavlink_msg_gsm_link_status_get_rsrq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Decode a gsm_link_status message into a struct
 *
 * @param msg The message to decode
 * @param gsm_link_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_gsm_link_status_decode(const mavlink_message_t* msg, mavlink_gsm_link_status_t* gsm_link_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gsm_link_status->timestamp = mavlink_msg_gsm_link_status_get_timestamp(msg);
    gsm_link_status->gsm_modem_type = mavlink_msg_gsm_link_status_get_gsm_modem_type(msg);
    gsm_link_status->gsm_link_type = mavlink_msg_gsm_link_status_get_gsm_link_type(msg);
    gsm_link_status->rssi = mavlink_msg_gsm_link_status_get_rssi(msg);
    gsm_link_status->rsrp_rscp = mavlink_msg_gsm_link_status_get_rsrp_rscp(msg);
    gsm_link_status->sinr_ecio = mavlink_msg_gsm_link_status_get_sinr_ecio(msg);
    gsm_link_status->rsrq = mavlink_msg_gsm_link_status_get_rsrq(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GSM_LINK_STATUS_LEN? msg->len : MAVLINK_MSG_ID_GSM_LINK_STATUS_LEN;
        memset(gsm_link_status, 0, MAVLINK_MSG_ID_GSM_LINK_STATUS_LEN);
    memcpy(gsm_link_status, _MAV_PAYLOAD(msg), len);
#endif
}
