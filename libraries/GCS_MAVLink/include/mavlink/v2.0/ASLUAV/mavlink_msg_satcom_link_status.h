#pragma once
// MESSAGE SATCOM_LINK_STATUS PACKING

#define MAVLINK_MSG_ID_SATCOM_LINK_STATUS 8015


typedef struct __mavlink_satcom_link_status_t {
 uint64_t timestamp; /*< [us] Timestamp*/
 uint64_t last_heartbeat; /*< [us] Timestamp of the last successful sbd session*/
 uint16_t failed_sessions; /*<  Number of failed sessions*/
 uint16_t successful_sessions; /*<  Number of successful sessions*/
 uint8_t signal_quality; /*<  Signal quality*/
 uint8_t ring_pending; /*<  Ring call pending*/
 uint8_t tx_session_pending; /*<  Transmission session pending*/
 uint8_t rx_session_pending; /*<  Receiving session pending*/
} mavlink_satcom_link_status_t;

#define MAVLINK_MSG_ID_SATCOM_LINK_STATUS_LEN 24
#define MAVLINK_MSG_ID_SATCOM_LINK_STATUS_MIN_LEN 24
#define MAVLINK_MSG_ID_8015_LEN 24
#define MAVLINK_MSG_ID_8015_MIN_LEN 24

#define MAVLINK_MSG_ID_SATCOM_LINK_STATUS_CRC 23
#define MAVLINK_MSG_ID_8015_CRC 23



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SATCOM_LINK_STATUS { \
    8015, \
    "SATCOM_LINK_STATUS", \
    8, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_satcom_link_status_t, timestamp) }, \
         { "last_heartbeat", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_satcom_link_status_t, last_heartbeat) }, \
         { "failed_sessions", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_satcom_link_status_t, failed_sessions) }, \
         { "successful_sessions", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_satcom_link_status_t, successful_sessions) }, \
         { "signal_quality", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_satcom_link_status_t, signal_quality) }, \
         { "ring_pending", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_satcom_link_status_t, ring_pending) }, \
         { "tx_session_pending", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_satcom_link_status_t, tx_session_pending) }, \
         { "rx_session_pending", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_satcom_link_status_t, rx_session_pending) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SATCOM_LINK_STATUS { \
    "SATCOM_LINK_STATUS", \
    8, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_satcom_link_status_t, timestamp) }, \
         { "last_heartbeat", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_satcom_link_status_t, last_heartbeat) }, \
         { "failed_sessions", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_satcom_link_status_t, failed_sessions) }, \
         { "successful_sessions", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_satcom_link_status_t, successful_sessions) }, \
         { "signal_quality", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_satcom_link_status_t, signal_quality) }, \
         { "ring_pending", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_satcom_link_status_t, ring_pending) }, \
         { "tx_session_pending", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_satcom_link_status_t, tx_session_pending) }, \
         { "rx_session_pending", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_satcom_link_status_t, rx_session_pending) }, \
         } \
}
#endif

/**
 * @brief Pack a satcom_link_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp
 * @param last_heartbeat [us] Timestamp of the last successful sbd session
 * @param failed_sessions  Number of failed sessions
 * @param successful_sessions  Number of successful sessions
 * @param signal_quality  Signal quality
 * @param ring_pending  Ring call pending
 * @param tx_session_pending  Transmission session pending
 * @param rx_session_pending  Receiving session pending
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_satcom_link_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint64_t last_heartbeat, uint16_t failed_sessions, uint16_t successful_sessions, uint8_t signal_quality, uint8_t ring_pending, uint8_t tx_session_pending, uint8_t rx_session_pending)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SATCOM_LINK_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, last_heartbeat);
    _mav_put_uint16_t(buf, 16, failed_sessions);
    _mav_put_uint16_t(buf, 18, successful_sessions);
    _mav_put_uint8_t(buf, 20, signal_quality);
    _mav_put_uint8_t(buf, 21, ring_pending);
    _mav_put_uint8_t(buf, 22, tx_session_pending);
    _mav_put_uint8_t(buf, 23, rx_session_pending);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SATCOM_LINK_STATUS_LEN);
#else
    mavlink_satcom_link_status_t packet;
    packet.timestamp = timestamp;
    packet.last_heartbeat = last_heartbeat;
    packet.failed_sessions = failed_sessions;
    packet.successful_sessions = successful_sessions;
    packet.signal_quality = signal_quality;
    packet.ring_pending = ring_pending;
    packet.tx_session_pending = tx_session_pending;
    packet.rx_session_pending = rx_session_pending;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SATCOM_LINK_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SATCOM_LINK_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SATCOM_LINK_STATUS_MIN_LEN, MAVLINK_MSG_ID_SATCOM_LINK_STATUS_LEN, MAVLINK_MSG_ID_SATCOM_LINK_STATUS_CRC);
}

/**
 * @brief Pack a satcom_link_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp
 * @param last_heartbeat [us] Timestamp of the last successful sbd session
 * @param failed_sessions  Number of failed sessions
 * @param successful_sessions  Number of successful sessions
 * @param signal_quality  Signal quality
 * @param ring_pending  Ring call pending
 * @param tx_session_pending  Transmission session pending
 * @param rx_session_pending  Receiving session pending
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_satcom_link_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t timestamp, uint64_t last_heartbeat, uint16_t failed_sessions, uint16_t successful_sessions, uint8_t signal_quality, uint8_t ring_pending, uint8_t tx_session_pending, uint8_t rx_session_pending)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SATCOM_LINK_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, last_heartbeat);
    _mav_put_uint16_t(buf, 16, failed_sessions);
    _mav_put_uint16_t(buf, 18, successful_sessions);
    _mav_put_uint8_t(buf, 20, signal_quality);
    _mav_put_uint8_t(buf, 21, ring_pending);
    _mav_put_uint8_t(buf, 22, tx_session_pending);
    _mav_put_uint8_t(buf, 23, rx_session_pending);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SATCOM_LINK_STATUS_LEN);
#else
    mavlink_satcom_link_status_t packet;
    packet.timestamp = timestamp;
    packet.last_heartbeat = last_heartbeat;
    packet.failed_sessions = failed_sessions;
    packet.successful_sessions = successful_sessions;
    packet.signal_quality = signal_quality;
    packet.ring_pending = ring_pending;
    packet.tx_session_pending = tx_session_pending;
    packet.rx_session_pending = rx_session_pending;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SATCOM_LINK_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SATCOM_LINK_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SATCOM_LINK_STATUS_MIN_LEN, MAVLINK_MSG_ID_SATCOM_LINK_STATUS_LEN, MAVLINK_MSG_ID_SATCOM_LINK_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SATCOM_LINK_STATUS_MIN_LEN, MAVLINK_MSG_ID_SATCOM_LINK_STATUS_LEN);
#endif
}

/**
 * @brief Pack a satcom_link_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp
 * @param last_heartbeat [us] Timestamp of the last successful sbd session
 * @param failed_sessions  Number of failed sessions
 * @param successful_sessions  Number of successful sessions
 * @param signal_quality  Signal quality
 * @param ring_pending  Ring call pending
 * @param tx_session_pending  Transmission session pending
 * @param rx_session_pending  Receiving session pending
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_satcom_link_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint64_t last_heartbeat,uint16_t failed_sessions,uint16_t successful_sessions,uint8_t signal_quality,uint8_t ring_pending,uint8_t tx_session_pending,uint8_t rx_session_pending)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SATCOM_LINK_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, last_heartbeat);
    _mav_put_uint16_t(buf, 16, failed_sessions);
    _mav_put_uint16_t(buf, 18, successful_sessions);
    _mav_put_uint8_t(buf, 20, signal_quality);
    _mav_put_uint8_t(buf, 21, ring_pending);
    _mav_put_uint8_t(buf, 22, tx_session_pending);
    _mav_put_uint8_t(buf, 23, rx_session_pending);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SATCOM_LINK_STATUS_LEN);
#else
    mavlink_satcom_link_status_t packet;
    packet.timestamp = timestamp;
    packet.last_heartbeat = last_heartbeat;
    packet.failed_sessions = failed_sessions;
    packet.successful_sessions = successful_sessions;
    packet.signal_quality = signal_quality;
    packet.ring_pending = ring_pending;
    packet.tx_session_pending = tx_session_pending;
    packet.rx_session_pending = rx_session_pending;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SATCOM_LINK_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SATCOM_LINK_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SATCOM_LINK_STATUS_MIN_LEN, MAVLINK_MSG_ID_SATCOM_LINK_STATUS_LEN, MAVLINK_MSG_ID_SATCOM_LINK_STATUS_CRC);
}

/**
 * @brief Encode a satcom_link_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param satcom_link_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_satcom_link_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_satcom_link_status_t* satcom_link_status)
{
    return mavlink_msg_satcom_link_status_pack(system_id, component_id, msg, satcom_link_status->timestamp, satcom_link_status->last_heartbeat, satcom_link_status->failed_sessions, satcom_link_status->successful_sessions, satcom_link_status->signal_quality, satcom_link_status->ring_pending, satcom_link_status->tx_session_pending, satcom_link_status->rx_session_pending);
}

/**
 * @brief Encode a satcom_link_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param satcom_link_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_satcom_link_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_satcom_link_status_t* satcom_link_status)
{
    return mavlink_msg_satcom_link_status_pack_chan(system_id, component_id, chan, msg, satcom_link_status->timestamp, satcom_link_status->last_heartbeat, satcom_link_status->failed_sessions, satcom_link_status->successful_sessions, satcom_link_status->signal_quality, satcom_link_status->ring_pending, satcom_link_status->tx_session_pending, satcom_link_status->rx_session_pending);
}

/**
 * @brief Encode a satcom_link_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param satcom_link_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_satcom_link_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_satcom_link_status_t* satcom_link_status)
{
    return mavlink_msg_satcom_link_status_pack_status(system_id, component_id, _status, msg,  satcom_link_status->timestamp, satcom_link_status->last_heartbeat, satcom_link_status->failed_sessions, satcom_link_status->successful_sessions, satcom_link_status->signal_quality, satcom_link_status->ring_pending, satcom_link_status->tx_session_pending, satcom_link_status->rx_session_pending);
}

/**
 * @brief Send a satcom_link_status message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp
 * @param last_heartbeat [us] Timestamp of the last successful sbd session
 * @param failed_sessions  Number of failed sessions
 * @param successful_sessions  Number of successful sessions
 * @param signal_quality  Signal quality
 * @param ring_pending  Ring call pending
 * @param tx_session_pending  Transmission session pending
 * @param rx_session_pending  Receiving session pending
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_satcom_link_status_send(mavlink_channel_t chan, uint64_t timestamp, uint64_t last_heartbeat, uint16_t failed_sessions, uint16_t successful_sessions, uint8_t signal_quality, uint8_t ring_pending, uint8_t tx_session_pending, uint8_t rx_session_pending)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SATCOM_LINK_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, last_heartbeat);
    _mav_put_uint16_t(buf, 16, failed_sessions);
    _mav_put_uint16_t(buf, 18, successful_sessions);
    _mav_put_uint8_t(buf, 20, signal_quality);
    _mav_put_uint8_t(buf, 21, ring_pending);
    _mav_put_uint8_t(buf, 22, tx_session_pending);
    _mav_put_uint8_t(buf, 23, rx_session_pending);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SATCOM_LINK_STATUS, buf, MAVLINK_MSG_ID_SATCOM_LINK_STATUS_MIN_LEN, MAVLINK_MSG_ID_SATCOM_LINK_STATUS_LEN, MAVLINK_MSG_ID_SATCOM_LINK_STATUS_CRC);
#else
    mavlink_satcom_link_status_t packet;
    packet.timestamp = timestamp;
    packet.last_heartbeat = last_heartbeat;
    packet.failed_sessions = failed_sessions;
    packet.successful_sessions = successful_sessions;
    packet.signal_quality = signal_quality;
    packet.ring_pending = ring_pending;
    packet.tx_session_pending = tx_session_pending;
    packet.rx_session_pending = rx_session_pending;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SATCOM_LINK_STATUS, (const char *)&packet, MAVLINK_MSG_ID_SATCOM_LINK_STATUS_MIN_LEN, MAVLINK_MSG_ID_SATCOM_LINK_STATUS_LEN, MAVLINK_MSG_ID_SATCOM_LINK_STATUS_CRC);
#endif
}

/**
 * @brief Send a satcom_link_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_satcom_link_status_send_struct(mavlink_channel_t chan, const mavlink_satcom_link_status_t* satcom_link_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_satcom_link_status_send(chan, satcom_link_status->timestamp, satcom_link_status->last_heartbeat, satcom_link_status->failed_sessions, satcom_link_status->successful_sessions, satcom_link_status->signal_quality, satcom_link_status->ring_pending, satcom_link_status->tx_session_pending, satcom_link_status->rx_session_pending);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SATCOM_LINK_STATUS, (const char *)satcom_link_status, MAVLINK_MSG_ID_SATCOM_LINK_STATUS_MIN_LEN, MAVLINK_MSG_ID_SATCOM_LINK_STATUS_LEN, MAVLINK_MSG_ID_SATCOM_LINK_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_SATCOM_LINK_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_satcom_link_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint64_t last_heartbeat, uint16_t failed_sessions, uint16_t successful_sessions, uint8_t signal_quality, uint8_t ring_pending, uint8_t tx_session_pending, uint8_t rx_session_pending)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, last_heartbeat);
    _mav_put_uint16_t(buf, 16, failed_sessions);
    _mav_put_uint16_t(buf, 18, successful_sessions);
    _mav_put_uint8_t(buf, 20, signal_quality);
    _mav_put_uint8_t(buf, 21, ring_pending);
    _mav_put_uint8_t(buf, 22, tx_session_pending);
    _mav_put_uint8_t(buf, 23, rx_session_pending);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SATCOM_LINK_STATUS, buf, MAVLINK_MSG_ID_SATCOM_LINK_STATUS_MIN_LEN, MAVLINK_MSG_ID_SATCOM_LINK_STATUS_LEN, MAVLINK_MSG_ID_SATCOM_LINK_STATUS_CRC);
#else
    mavlink_satcom_link_status_t *packet = (mavlink_satcom_link_status_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->last_heartbeat = last_heartbeat;
    packet->failed_sessions = failed_sessions;
    packet->successful_sessions = successful_sessions;
    packet->signal_quality = signal_quality;
    packet->ring_pending = ring_pending;
    packet->tx_session_pending = tx_session_pending;
    packet->rx_session_pending = rx_session_pending;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SATCOM_LINK_STATUS, (const char *)packet, MAVLINK_MSG_ID_SATCOM_LINK_STATUS_MIN_LEN, MAVLINK_MSG_ID_SATCOM_LINK_STATUS_LEN, MAVLINK_MSG_ID_SATCOM_LINK_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE SATCOM_LINK_STATUS UNPACKING


/**
 * @brief Get field timestamp from satcom_link_status message
 *
 * @return [us] Timestamp
 */
static inline uint64_t mavlink_msg_satcom_link_status_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field last_heartbeat from satcom_link_status message
 *
 * @return [us] Timestamp of the last successful sbd session
 */
static inline uint64_t mavlink_msg_satcom_link_status_get_last_heartbeat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field failed_sessions from satcom_link_status message
 *
 * @return  Number of failed sessions
 */
static inline uint16_t mavlink_msg_satcom_link_status_get_failed_sessions(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field successful_sessions from satcom_link_status message
 *
 * @return  Number of successful sessions
 */
static inline uint16_t mavlink_msg_satcom_link_status_get_successful_sessions(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  18);
}

/**
 * @brief Get field signal_quality from satcom_link_status message
 *
 * @return  Signal quality
 */
static inline uint8_t mavlink_msg_satcom_link_status_get_signal_quality(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field ring_pending from satcom_link_status message
 *
 * @return  Ring call pending
 */
static inline uint8_t mavlink_msg_satcom_link_status_get_ring_pending(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Get field tx_session_pending from satcom_link_status message
 *
 * @return  Transmission session pending
 */
static inline uint8_t mavlink_msg_satcom_link_status_get_tx_session_pending(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Get field rx_session_pending from satcom_link_status message
 *
 * @return  Receiving session pending
 */
static inline uint8_t mavlink_msg_satcom_link_status_get_rx_session_pending(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  23);
}

/**
 * @brief Decode a satcom_link_status message into a struct
 *
 * @param msg The message to decode
 * @param satcom_link_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_satcom_link_status_decode(const mavlink_message_t* msg, mavlink_satcom_link_status_t* satcom_link_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    satcom_link_status->timestamp = mavlink_msg_satcom_link_status_get_timestamp(msg);
    satcom_link_status->last_heartbeat = mavlink_msg_satcom_link_status_get_last_heartbeat(msg);
    satcom_link_status->failed_sessions = mavlink_msg_satcom_link_status_get_failed_sessions(msg);
    satcom_link_status->successful_sessions = mavlink_msg_satcom_link_status_get_successful_sessions(msg);
    satcom_link_status->signal_quality = mavlink_msg_satcom_link_status_get_signal_quality(msg);
    satcom_link_status->ring_pending = mavlink_msg_satcom_link_status_get_ring_pending(msg);
    satcom_link_status->tx_session_pending = mavlink_msg_satcom_link_status_get_tx_session_pending(msg);
    satcom_link_status->rx_session_pending = mavlink_msg_satcom_link_status_get_rx_session_pending(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SATCOM_LINK_STATUS_LEN? msg->len : MAVLINK_MSG_ID_SATCOM_LINK_STATUS_LEN;
        memset(satcom_link_status, 0, MAVLINK_MSG_ID_SATCOM_LINK_STATUS_LEN);
    memcpy(satcom_link_status, _MAV_PAYLOAD(msg), len);
#endif
}
