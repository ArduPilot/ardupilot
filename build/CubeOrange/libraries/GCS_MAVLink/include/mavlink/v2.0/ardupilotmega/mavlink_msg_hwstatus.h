#pragma once
// MESSAGE HWSTATUS PACKING

#define MAVLINK_MSG_ID_HWSTATUS 165


typedef struct __mavlink_hwstatus_t {
 uint16_t Vcc; /*< [mV] Board voltage.*/
 uint8_t I2Cerr; /*<  I2C error count.*/
} mavlink_hwstatus_t;

#define MAVLINK_MSG_ID_HWSTATUS_LEN 3
#define MAVLINK_MSG_ID_HWSTATUS_MIN_LEN 3
#define MAVLINK_MSG_ID_165_LEN 3
#define MAVLINK_MSG_ID_165_MIN_LEN 3

#define MAVLINK_MSG_ID_HWSTATUS_CRC 21
#define MAVLINK_MSG_ID_165_CRC 21



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_HWSTATUS { \
    165, \
    "HWSTATUS", \
    2, \
    {  { "Vcc", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_hwstatus_t, Vcc) }, \
         { "I2Cerr", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_hwstatus_t, I2Cerr) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_HWSTATUS { \
    "HWSTATUS", \
    2, \
    {  { "Vcc", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_hwstatus_t, Vcc) }, \
         { "I2Cerr", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_hwstatus_t, I2Cerr) }, \
         } \
}
#endif

/**
 * @brief Pack a hwstatus message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param Vcc [mV] Board voltage.
 * @param I2Cerr  I2C error count.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hwstatus_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t Vcc, uint8_t I2Cerr)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HWSTATUS_LEN];
    _mav_put_uint16_t(buf, 0, Vcc);
    _mav_put_uint8_t(buf, 2, I2Cerr);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HWSTATUS_LEN);
#else
    mavlink_hwstatus_t packet;
    packet.Vcc = Vcc;
    packet.I2Cerr = I2Cerr;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HWSTATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HWSTATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HWSTATUS_MIN_LEN, MAVLINK_MSG_ID_HWSTATUS_LEN, MAVLINK_MSG_ID_HWSTATUS_CRC);
}

/**
 * @brief Pack a hwstatus message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param Vcc [mV] Board voltage.
 * @param I2Cerr  I2C error count.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hwstatus_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t Vcc,uint8_t I2Cerr)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HWSTATUS_LEN];
    _mav_put_uint16_t(buf, 0, Vcc);
    _mav_put_uint8_t(buf, 2, I2Cerr);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HWSTATUS_LEN);
#else
    mavlink_hwstatus_t packet;
    packet.Vcc = Vcc;
    packet.I2Cerr = I2Cerr;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HWSTATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HWSTATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HWSTATUS_MIN_LEN, MAVLINK_MSG_ID_HWSTATUS_LEN, MAVLINK_MSG_ID_HWSTATUS_CRC);
}

/**
 * @brief Encode a hwstatus struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param hwstatus C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hwstatus_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_hwstatus_t* hwstatus)
{
    return mavlink_msg_hwstatus_pack(system_id, component_id, msg, hwstatus->Vcc, hwstatus->I2Cerr);
}

/**
 * @brief Encode a hwstatus struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param hwstatus C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hwstatus_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_hwstatus_t* hwstatus)
{
    return mavlink_msg_hwstatus_pack_chan(system_id, component_id, chan, msg, hwstatus->Vcc, hwstatus->I2Cerr);
}

/**
 * @brief Send a hwstatus message
 * @param chan MAVLink channel to send the message
 *
 * @param Vcc [mV] Board voltage.
 * @param I2Cerr  I2C error count.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_hwstatus_send(mavlink_channel_t chan, uint16_t Vcc, uint8_t I2Cerr)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HWSTATUS_LEN];
    _mav_put_uint16_t(buf, 0, Vcc);
    _mav_put_uint8_t(buf, 2, I2Cerr);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HWSTATUS, buf, MAVLINK_MSG_ID_HWSTATUS_MIN_LEN, MAVLINK_MSG_ID_HWSTATUS_LEN, MAVLINK_MSG_ID_HWSTATUS_CRC);
#else
    mavlink_hwstatus_t packet;
    packet.Vcc = Vcc;
    packet.I2Cerr = I2Cerr;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HWSTATUS, (const char *)&packet, MAVLINK_MSG_ID_HWSTATUS_MIN_LEN, MAVLINK_MSG_ID_HWSTATUS_LEN, MAVLINK_MSG_ID_HWSTATUS_CRC);
#endif
}

/**
 * @brief Send a hwstatus message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_hwstatus_send_struct(mavlink_channel_t chan, const mavlink_hwstatus_t* hwstatus)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_hwstatus_send(chan, hwstatus->Vcc, hwstatus->I2Cerr);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HWSTATUS, (const char *)hwstatus, MAVLINK_MSG_ID_HWSTATUS_MIN_LEN, MAVLINK_MSG_ID_HWSTATUS_LEN, MAVLINK_MSG_ID_HWSTATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_HWSTATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_hwstatus_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t Vcc, uint8_t I2Cerr)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, Vcc);
    _mav_put_uint8_t(buf, 2, I2Cerr);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HWSTATUS, buf, MAVLINK_MSG_ID_HWSTATUS_MIN_LEN, MAVLINK_MSG_ID_HWSTATUS_LEN, MAVLINK_MSG_ID_HWSTATUS_CRC);
#else
    mavlink_hwstatus_t *packet = (mavlink_hwstatus_t *)msgbuf;
    packet->Vcc = Vcc;
    packet->I2Cerr = I2Cerr;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HWSTATUS, (const char *)packet, MAVLINK_MSG_ID_HWSTATUS_MIN_LEN, MAVLINK_MSG_ID_HWSTATUS_LEN, MAVLINK_MSG_ID_HWSTATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE HWSTATUS UNPACKING


/**
 * @brief Get field Vcc from hwstatus message
 *
 * @return [mV] Board voltage.
 */
static inline uint16_t mavlink_msg_hwstatus_get_Vcc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field I2Cerr from hwstatus message
 *
 * @return  I2C error count.
 */
static inline uint8_t mavlink_msg_hwstatus_get_I2Cerr(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a hwstatus message into a struct
 *
 * @param msg The message to decode
 * @param hwstatus C-struct to decode the message contents into
 */
static inline void mavlink_msg_hwstatus_decode(const mavlink_message_t* msg, mavlink_hwstatus_t* hwstatus)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    hwstatus->Vcc = mavlink_msg_hwstatus_get_Vcc(msg);
    hwstatus->I2Cerr = mavlink_msg_hwstatus_get_I2Cerr(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_HWSTATUS_LEN? msg->len : MAVLINK_MSG_ID_HWSTATUS_LEN;
        memset(hwstatus, 0, MAVLINK_MSG_ID_HWSTATUS_LEN);
    memcpy(hwstatus, _MAV_PAYLOAD(msg), len);
#endif
}
