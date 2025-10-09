#pragma once
// MESSAGE RADIO_STATUS PACKING

#define MAVLINK_MSG_ID_RADIO_STATUS 109


typedef struct __mavlink_radio_status_t {
 uint16_t rxerrors; /*<  Count of radio packet receive errors (since boot).*/
 uint16_t fixed; /*<  Count of error corrected radio packets (since boot).*/
 uint8_t rssi; /*<  Local (message sender) recieved signal strength indication in device-dependent units/scale. Values: [0-254], 255: invalid/unknown.*/
 uint8_t remrssi; /*<  Remote (message receiver) signal strength indication in device-dependent units/scale. Values: [0-254], 255: invalid/unknown.*/
 uint8_t txbuf; /*< [%] Remaining free transmitter buffer space.*/
 uint8_t noise; /*<  Local background noise level. These are device dependent RSSI values (scale as approx 2x dB on SiK radios). Values: [0-254], 255: invalid/unknown.*/
 uint8_t remnoise; /*<  Remote background noise level. These are device dependent RSSI values (scale as approx 2x dB on SiK radios). Values: [0-254], 255: invalid/unknown.*/
} mavlink_radio_status_t;

#define MAVLINK_MSG_ID_RADIO_STATUS_LEN 9
#define MAVLINK_MSG_ID_RADIO_STATUS_MIN_LEN 9
#define MAVLINK_MSG_ID_109_LEN 9
#define MAVLINK_MSG_ID_109_MIN_LEN 9

#define MAVLINK_MSG_ID_RADIO_STATUS_CRC 185
#define MAVLINK_MSG_ID_109_CRC 185



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RADIO_STATUS { \
    109, \
    "RADIO_STATUS", \
    7, \
    {  { "rssi", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_radio_status_t, rssi) }, \
         { "remrssi", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_radio_status_t, remrssi) }, \
         { "txbuf", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_radio_status_t, txbuf) }, \
         { "noise", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_radio_status_t, noise) }, \
         { "remnoise", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_radio_status_t, remnoise) }, \
         { "rxerrors", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_radio_status_t, rxerrors) }, \
         { "fixed", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_radio_status_t, fixed) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RADIO_STATUS { \
    "RADIO_STATUS", \
    7, \
    {  { "rssi", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_radio_status_t, rssi) }, \
         { "remrssi", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_radio_status_t, remrssi) }, \
         { "txbuf", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_radio_status_t, txbuf) }, \
         { "noise", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_radio_status_t, noise) }, \
         { "remnoise", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_radio_status_t, remnoise) }, \
         { "rxerrors", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_radio_status_t, rxerrors) }, \
         { "fixed", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_radio_status_t, fixed) }, \
         } \
}
#endif

/**
 * @brief Pack a radio_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param rssi  Local (message sender) recieved signal strength indication in device-dependent units/scale. Values: [0-254], 255: invalid/unknown.
 * @param remrssi  Remote (message receiver) signal strength indication in device-dependent units/scale. Values: [0-254], 255: invalid/unknown.
 * @param txbuf [%] Remaining free transmitter buffer space.
 * @param noise  Local background noise level. These are device dependent RSSI values (scale as approx 2x dB on SiK radios). Values: [0-254], 255: invalid/unknown.
 * @param remnoise  Remote background noise level. These are device dependent RSSI values (scale as approx 2x dB on SiK radios). Values: [0-254], 255: invalid/unknown.
 * @param rxerrors  Count of radio packet receive errors (since boot).
 * @param fixed  Count of error corrected radio packets (since boot).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radio_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t rssi, uint8_t remrssi, uint8_t txbuf, uint8_t noise, uint8_t remnoise, uint16_t rxerrors, uint16_t fixed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RADIO_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, rxerrors);
    _mav_put_uint16_t(buf, 2, fixed);
    _mav_put_uint8_t(buf, 4, rssi);
    _mav_put_uint8_t(buf, 5, remrssi);
    _mav_put_uint8_t(buf, 6, txbuf);
    _mav_put_uint8_t(buf, 7, noise);
    _mav_put_uint8_t(buf, 8, remnoise);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RADIO_STATUS_LEN);
#else
    mavlink_radio_status_t packet;
    packet.rxerrors = rxerrors;
    packet.fixed = fixed;
    packet.rssi = rssi;
    packet.remrssi = remrssi;
    packet.txbuf = txbuf;
    packet.noise = noise;
    packet.remnoise = remnoise;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RADIO_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RADIO_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RADIO_STATUS_MIN_LEN, MAVLINK_MSG_ID_RADIO_STATUS_LEN, MAVLINK_MSG_ID_RADIO_STATUS_CRC);
}

/**
 * @brief Pack a radio_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param rssi  Local (message sender) recieved signal strength indication in device-dependent units/scale. Values: [0-254], 255: invalid/unknown.
 * @param remrssi  Remote (message receiver) signal strength indication in device-dependent units/scale. Values: [0-254], 255: invalid/unknown.
 * @param txbuf [%] Remaining free transmitter buffer space.
 * @param noise  Local background noise level. These are device dependent RSSI values (scale as approx 2x dB on SiK radios). Values: [0-254], 255: invalid/unknown.
 * @param remnoise  Remote background noise level. These are device dependent RSSI values (scale as approx 2x dB on SiK radios). Values: [0-254], 255: invalid/unknown.
 * @param rxerrors  Count of radio packet receive errors (since boot).
 * @param fixed  Count of error corrected radio packets (since boot).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radio_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t rssi, uint8_t remrssi, uint8_t txbuf, uint8_t noise, uint8_t remnoise, uint16_t rxerrors, uint16_t fixed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RADIO_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, rxerrors);
    _mav_put_uint16_t(buf, 2, fixed);
    _mav_put_uint8_t(buf, 4, rssi);
    _mav_put_uint8_t(buf, 5, remrssi);
    _mav_put_uint8_t(buf, 6, txbuf);
    _mav_put_uint8_t(buf, 7, noise);
    _mav_put_uint8_t(buf, 8, remnoise);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RADIO_STATUS_LEN);
#else
    mavlink_radio_status_t packet;
    packet.rxerrors = rxerrors;
    packet.fixed = fixed;
    packet.rssi = rssi;
    packet.remrssi = remrssi;
    packet.txbuf = txbuf;
    packet.noise = noise;
    packet.remnoise = remnoise;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RADIO_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RADIO_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_RADIO_STATUS_MIN_LEN, MAVLINK_MSG_ID_RADIO_STATUS_LEN, MAVLINK_MSG_ID_RADIO_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_RADIO_STATUS_MIN_LEN, MAVLINK_MSG_ID_RADIO_STATUS_LEN);
#endif
}

/**
 * @brief Pack a radio_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rssi  Local (message sender) recieved signal strength indication in device-dependent units/scale. Values: [0-254], 255: invalid/unknown.
 * @param remrssi  Remote (message receiver) signal strength indication in device-dependent units/scale. Values: [0-254], 255: invalid/unknown.
 * @param txbuf [%] Remaining free transmitter buffer space.
 * @param noise  Local background noise level. These are device dependent RSSI values (scale as approx 2x dB on SiK radios). Values: [0-254], 255: invalid/unknown.
 * @param remnoise  Remote background noise level. These are device dependent RSSI values (scale as approx 2x dB on SiK radios). Values: [0-254], 255: invalid/unknown.
 * @param rxerrors  Count of radio packet receive errors (since boot).
 * @param fixed  Count of error corrected radio packets (since boot).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radio_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t rssi,uint8_t remrssi,uint8_t txbuf,uint8_t noise,uint8_t remnoise,uint16_t rxerrors,uint16_t fixed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RADIO_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, rxerrors);
    _mav_put_uint16_t(buf, 2, fixed);
    _mav_put_uint8_t(buf, 4, rssi);
    _mav_put_uint8_t(buf, 5, remrssi);
    _mav_put_uint8_t(buf, 6, txbuf);
    _mav_put_uint8_t(buf, 7, noise);
    _mav_put_uint8_t(buf, 8, remnoise);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RADIO_STATUS_LEN);
#else
    mavlink_radio_status_t packet;
    packet.rxerrors = rxerrors;
    packet.fixed = fixed;
    packet.rssi = rssi;
    packet.remrssi = remrssi;
    packet.txbuf = txbuf;
    packet.noise = noise;
    packet.remnoise = remnoise;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RADIO_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RADIO_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RADIO_STATUS_MIN_LEN, MAVLINK_MSG_ID_RADIO_STATUS_LEN, MAVLINK_MSG_ID_RADIO_STATUS_CRC);
}

/**
 * @brief Encode a radio_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param radio_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_radio_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_radio_status_t* radio_status)
{
    return mavlink_msg_radio_status_pack(system_id, component_id, msg, radio_status->rssi, radio_status->remrssi, radio_status->txbuf, radio_status->noise, radio_status->remnoise, radio_status->rxerrors, radio_status->fixed);
}

/**
 * @brief Encode a radio_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param radio_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_radio_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_radio_status_t* radio_status)
{
    return mavlink_msg_radio_status_pack_chan(system_id, component_id, chan, msg, radio_status->rssi, radio_status->remrssi, radio_status->txbuf, radio_status->noise, radio_status->remnoise, radio_status->rxerrors, radio_status->fixed);
}

/**
 * @brief Encode a radio_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param radio_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_radio_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_radio_status_t* radio_status)
{
    return mavlink_msg_radio_status_pack_status(system_id, component_id, _status, msg,  radio_status->rssi, radio_status->remrssi, radio_status->txbuf, radio_status->noise, radio_status->remnoise, radio_status->rxerrors, radio_status->fixed);
}

/**
 * @brief Send a radio_status message
 * @param chan MAVLink channel to send the message
 *
 * @param rssi  Local (message sender) recieved signal strength indication in device-dependent units/scale. Values: [0-254], 255: invalid/unknown.
 * @param remrssi  Remote (message receiver) signal strength indication in device-dependent units/scale. Values: [0-254], 255: invalid/unknown.
 * @param txbuf [%] Remaining free transmitter buffer space.
 * @param noise  Local background noise level. These are device dependent RSSI values (scale as approx 2x dB on SiK radios). Values: [0-254], 255: invalid/unknown.
 * @param remnoise  Remote background noise level. These are device dependent RSSI values (scale as approx 2x dB on SiK radios). Values: [0-254], 255: invalid/unknown.
 * @param rxerrors  Count of radio packet receive errors (since boot).
 * @param fixed  Count of error corrected radio packets (since boot).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_radio_status_send(mavlink_channel_t chan, uint8_t rssi, uint8_t remrssi, uint8_t txbuf, uint8_t noise, uint8_t remnoise, uint16_t rxerrors, uint16_t fixed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RADIO_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, rxerrors);
    _mav_put_uint16_t(buf, 2, fixed);
    _mav_put_uint8_t(buf, 4, rssi);
    _mav_put_uint8_t(buf, 5, remrssi);
    _mav_put_uint8_t(buf, 6, txbuf);
    _mav_put_uint8_t(buf, 7, noise);
    _mav_put_uint8_t(buf, 8, remnoise);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADIO_STATUS, buf, MAVLINK_MSG_ID_RADIO_STATUS_MIN_LEN, MAVLINK_MSG_ID_RADIO_STATUS_LEN, MAVLINK_MSG_ID_RADIO_STATUS_CRC);
#else
    mavlink_radio_status_t packet;
    packet.rxerrors = rxerrors;
    packet.fixed = fixed;
    packet.rssi = rssi;
    packet.remrssi = remrssi;
    packet.txbuf = txbuf;
    packet.noise = noise;
    packet.remnoise = remnoise;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADIO_STATUS, (const char *)&packet, MAVLINK_MSG_ID_RADIO_STATUS_MIN_LEN, MAVLINK_MSG_ID_RADIO_STATUS_LEN, MAVLINK_MSG_ID_RADIO_STATUS_CRC);
#endif
}

/**
 * @brief Send a radio_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_radio_status_send_struct(mavlink_channel_t chan, const mavlink_radio_status_t* radio_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_radio_status_send(chan, radio_status->rssi, radio_status->remrssi, radio_status->txbuf, radio_status->noise, radio_status->remnoise, radio_status->rxerrors, radio_status->fixed);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADIO_STATUS, (const char *)radio_status, MAVLINK_MSG_ID_RADIO_STATUS_MIN_LEN, MAVLINK_MSG_ID_RADIO_STATUS_LEN, MAVLINK_MSG_ID_RADIO_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_RADIO_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_radio_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t rssi, uint8_t remrssi, uint8_t txbuf, uint8_t noise, uint8_t remnoise, uint16_t rxerrors, uint16_t fixed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, rxerrors);
    _mav_put_uint16_t(buf, 2, fixed);
    _mav_put_uint8_t(buf, 4, rssi);
    _mav_put_uint8_t(buf, 5, remrssi);
    _mav_put_uint8_t(buf, 6, txbuf);
    _mav_put_uint8_t(buf, 7, noise);
    _mav_put_uint8_t(buf, 8, remnoise);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADIO_STATUS, buf, MAVLINK_MSG_ID_RADIO_STATUS_MIN_LEN, MAVLINK_MSG_ID_RADIO_STATUS_LEN, MAVLINK_MSG_ID_RADIO_STATUS_CRC);
#else
    mavlink_radio_status_t *packet = (mavlink_radio_status_t *)msgbuf;
    packet->rxerrors = rxerrors;
    packet->fixed = fixed;
    packet->rssi = rssi;
    packet->remrssi = remrssi;
    packet->txbuf = txbuf;
    packet->noise = noise;
    packet->remnoise = remnoise;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADIO_STATUS, (const char *)packet, MAVLINK_MSG_ID_RADIO_STATUS_MIN_LEN, MAVLINK_MSG_ID_RADIO_STATUS_LEN, MAVLINK_MSG_ID_RADIO_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE RADIO_STATUS UNPACKING


/**
 * @brief Get field rssi from radio_status message
 *
 * @return  Local (message sender) recieved signal strength indication in device-dependent units/scale. Values: [0-254], 255: invalid/unknown.
 */
static inline uint8_t mavlink_msg_radio_status_get_rssi(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field remrssi from radio_status message
 *
 * @return  Remote (message receiver) signal strength indication in device-dependent units/scale. Values: [0-254], 255: invalid/unknown.
 */
static inline uint8_t mavlink_msg_radio_status_get_remrssi(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field txbuf from radio_status message
 *
 * @return [%] Remaining free transmitter buffer space.
 */
static inline uint8_t mavlink_msg_radio_status_get_txbuf(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field noise from radio_status message
 *
 * @return  Local background noise level. These are device dependent RSSI values (scale as approx 2x dB on SiK radios). Values: [0-254], 255: invalid/unknown.
 */
static inline uint8_t mavlink_msg_radio_status_get_noise(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field remnoise from radio_status message
 *
 * @return  Remote background noise level. These are device dependent RSSI values (scale as approx 2x dB on SiK radios). Values: [0-254], 255: invalid/unknown.
 */
static inline uint8_t mavlink_msg_radio_status_get_remnoise(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field rxerrors from radio_status message
 *
 * @return  Count of radio packet receive errors (since boot).
 */
static inline uint16_t mavlink_msg_radio_status_get_rxerrors(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field fixed from radio_status message
 *
 * @return  Count of error corrected radio packets (since boot).
 */
static inline uint16_t mavlink_msg_radio_status_get_fixed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Decode a radio_status message into a struct
 *
 * @param msg The message to decode
 * @param radio_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_radio_status_decode(const mavlink_message_t* msg, mavlink_radio_status_t* radio_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    radio_status->rxerrors = mavlink_msg_radio_status_get_rxerrors(msg);
    radio_status->fixed = mavlink_msg_radio_status_get_fixed(msg);
    radio_status->rssi = mavlink_msg_radio_status_get_rssi(msg);
    radio_status->remrssi = mavlink_msg_radio_status_get_remrssi(msg);
    radio_status->txbuf = mavlink_msg_radio_status_get_txbuf(msg);
    radio_status->noise = mavlink_msg_radio_status_get_noise(msg);
    radio_status->remnoise = mavlink_msg_radio_status_get_remnoise(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RADIO_STATUS_LEN? msg->len : MAVLINK_MSG_ID_RADIO_STATUS_LEN;
        memset(radio_status, 0, MAVLINK_MSG_ID_RADIO_STATUS_LEN);
    memcpy(radio_status, _MAV_PAYLOAD(msg), len);
#endif
}
