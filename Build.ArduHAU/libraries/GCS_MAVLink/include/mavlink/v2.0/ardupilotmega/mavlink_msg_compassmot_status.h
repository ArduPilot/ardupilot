#pragma once
// MESSAGE COMPASSMOT_STATUS PACKING

#define MAVLINK_MSG_ID_COMPASSMOT_STATUS 177

MAVPACKED(
typedef struct __mavlink_compassmot_status_t {
 float current; /*< current (Ampere)*/
 float CompensationX; /*< Motor Compensation X*/
 float CompensationY; /*< Motor Compensation Y*/
 float CompensationZ; /*< Motor Compensation Z*/
 uint16_t throttle; /*< throttle (percent*10)*/
 uint16_t interference; /*< interference (percent)*/
}) mavlink_compassmot_status_t;

#define MAVLINK_MSG_ID_COMPASSMOT_STATUS_LEN 20
#define MAVLINK_MSG_ID_COMPASSMOT_STATUS_MIN_LEN 20
#define MAVLINK_MSG_ID_177_LEN 20
#define MAVLINK_MSG_ID_177_MIN_LEN 20

#define MAVLINK_MSG_ID_COMPASSMOT_STATUS_CRC 240
#define MAVLINK_MSG_ID_177_CRC 240



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_COMPASSMOT_STATUS { \
    177, \
    "COMPASSMOT_STATUS", \
    6, \
    {  { "throttle", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_compassmot_status_t, throttle) }, \
         { "current", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_compassmot_status_t, current) }, \
         { "interference", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_compassmot_status_t, interference) }, \
         { "CompensationX", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_compassmot_status_t, CompensationX) }, \
         { "CompensationY", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_compassmot_status_t, CompensationY) }, \
         { "CompensationZ", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_compassmot_status_t, CompensationZ) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_COMPASSMOT_STATUS { \
    "COMPASSMOT_STATUS", \
    6, \
    {  { "throttle", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_compassmot_status_t, throttle) }, \
         { "current", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_compassmot_status_t, current) }, \
         { "interference", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_compassmot_status_t, interference) }, \
         { "CompensationX", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_compassmot_status_t, CompensationX) }, \
         { "CompensationY", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_compassmot_status_t, CompensationY) }, \
         { "CompensationZ", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_compassmot_status_t, CompensationZ) }, \
         } \
}
#endif

/**
 * @brief Pack a compassmot_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param throttle throttle (percent*10)
 * @param current current (Ampere)
 * @param interference interference (percent)
 * @param CompensationX Motor Compensation X
 * @param CompensationY Motor Compensation Y
 * @param CompensationZ Motor Compensation Z
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_compassmot_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t throttle, float current, uint16_t interference, float CompensationX, float CompensationY, float CompensationZ)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMPASSMOT_STATUS_LEN];
    _mav_put_float(buf, 0, current);
    _mav_put_float(buf, 4, CompensationX);
    _mav_put_float(buf, 8, CompensationY);
    _mav_put_float(buf, 12, CompensationZ);
    _mav_put_uint16_t(buf, 16, throttle);
    _mav_put_uint16_t(buf, 18, interference);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_COMPASSMOT_STATUS_LEN);
#else
    mavlink_compassmot_status_t packet;
    packet.current = current;
    packet.CompensationX = CompensationX;
    packet.CompensationY = CompensationY;
    packet.CompensationZ = CompensationZ;
    packet.throttle = throttle;
    packet.interference = interference;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMPASSMOT_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_COMPASSMOT_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_COMPASSMOT_STATUS_MIN_LEN, MAVLINK_MSG_ID_COMPASSMOT_STATUS_LEN, MAVLINK_MSG_ID_COMPASSMOT_STATUS_CRC);
}

/**
 * @brief Pack a compassmot_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param throttle throttle (percent*10)
 * @param current current (Ampere)
 * @param interference interference (percent)
 * @param CompensationX Motor Compensation X
 * @param CompensationY Motor Compensation Y
 * @param CompensationZ Motor Compensation Z
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_compassmot_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t throttle,float current,uint16_t interference,float CompensationX,float CompensationY,float CompensationZ)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMPASSMOT_STATUS_LEN];
    _mav_put_float(buf, 0, current);
    _mav_put_float(buf, 4, CompensationX);
    _mav_put_float(buf, 8, CompensationY);
    _mav_put_float(buf, 12, CompensationZ);
    _mav_put_uint16_t(buf, 16, throttle);
    _mav_put_uint16_t(buf, 18, interference);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_COMPASSMOT_STATUS_LEN);
#else
    mavlink_compassmot_status_t packet;
    packet.current = current;
    packet.CompensationX = CompensationX;
    packet.CompensationY = CompensationY;
    packet.CompensationZ = CompensationZ;
    packet.throttle = throttle;
    packet.interference = interference;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMPASSMOT_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_COMPASSMOT_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_COMPASSMOT_STATUS_MIN_LEN, MAVLINK_MSG_ID_COMPASSMOT_STATUS_LEN, MAVLINK_MSG_ID_COMPASSMOT_STATUS_CRC);
}

/**
 * @brief Encode a compassmot_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param compassmot_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_compassmot_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_compassmot_status_t* compassmot_status)
{
    return mavlink_msg_compassmot_status_pack(system_id, component_id, msg, compassmot_status->throttle, compassmot_status->current, compassmot_status->interference, compassmot_status->CompensationX, compassmot_status->CompensationY, compassmot_status->CompensationZ);
}

/**
 * @brief Encode a compassmot_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param compassmot_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_compassmot_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_compassmot_status_t* compassmot_status)
{
    return mavlink_msg_compassmot_status_pack_chan(system_id, component_id, chan, msg, compassmot_status->throttle, compassmot_status->current, compassmot_status->interference, compassmot_status->CompensationX, compassmot_status->CompensationY, compassmot_status->CompensationZ);
}

/**
 * @brief Send a compassmot_status message
 * @param chan MAVLink channel to send the message
 *
 * @param throttle throttle (percent*10)
 * @param current current (Ampere)
 * @param interference interference (percent)
 * @param CompensationX Motor Compensation X
 * @param CompensationY Motor Compensation Y
 * @param CompensationZ Motor Compensation Z
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_compassmot_status_send(mavlink_channel_t chan, uint16_t throttle, float current, uint16_t interference, float CompensationX, float CompensationY, float CompensationZ)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMPASSMOT_STATUS_LEN];
    _mav_put_float(buf, 0, current);
    _mav_put_float(buf, 4, CompensationX);
    _mav_put_float(buf, 8, CompensationY);
    _mav_put_float(buf, 12, CompensationZ);
    _mav_put_uint16_t(buf, 16, throttle);
    _mav_put_uint16_t(buf, 18, interference);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMPASSMOT_STATUS, buf, MAVLINK_MSG_ID_COMPASSMOT_STATUS_MIN_LEN, MAVLINK_MSG_ID_COMPASSMOT_STATUS_LEN, MAVLINK_MSG_ID_COMPASSMOT_STATUS_CRC);
#else
    mavlink_compassmot_status_t packet;
    packet.current = current;
    packet.CompensationX = CompensationX;
    packet.CompensationY = CompensationY;
    packet.CompensationZ = CompensationZ;
    packet.throttle = throttle;
    packet.interference = interference;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMPASSMOT_STATUS, (const char *)&packet, MAVLINK_MSG_ID_COMPASSMOT_STATUS_MIN_LEN, MAVLINK_MSG_ID_COMPASSMOT_STATUS_LEN, MAVLINK_MSG_ID_COMPASSMOT_STATUS_CRC);
#endif
}

/**
 * @brief Send a compassmot_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_compassmot_status_send_struct(mavlink_channel_t chan, const mavlink_compassmot_status_t* compassmot_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_compassmot_status_send(chan, compassmot_status->throttle, compassmot_status->current, compassmot_status->interference, compassmot_status->CompensationX, compassmot_status->CompensationY, compassmot_status->CompensationZ);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMPASSMOT_STATUS, (const char *)compassmot_status, MAVLINK_MSG_ID_COMPASSMOT_STATUS_MIN_LEN, MAVLINK_MSG_ID_COMPASSMOT_STATUS_LEN, MAVLINK_MSG_ID_COMPASSMOT_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_COMPASSMOT_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_compassmot_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t throttle, float current, uint16_t interference, float CompensationX, float CompensationY, float CompensationZ)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, current);
    _mav_put_float(buf, 4, CompensationX);
    _mav_put_float(buf, 8, CompensationY);
    _mav_put_float(buf, 12, CompensationZ);
    _mav_put_uint16_t(buf, 16, throttle);
    _mav_put_uint16_t(buf, 18, interference);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMPASSMOT_STATUS, buf, MAVLINK_MSG_ID_COMPASSMOT_STATUS_MIN_LEN, MAVLINK_MSG_ID_COMPASSMOT_STATUS_LEN, MAVLINK_MSG_ID_COMPASSMOT_STATUS_CRC);
#else
    mavlink_compassmot_status_t *packet = (mavlink_compassmot_status_t *)msgbuf;
    packet->current = current;
    packet->CompensationX = CompensationX;
    packet->CompensationY = CompensationY;
    packet->CompensationZ = CompensationZ;
    packet->throttle = throttle;
    packet->interference = interference;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMPASSMOT_STATUS, (const char *)packet, MAVLINK_MSG_ID_COMPASSMOT_STATUS_MIN_LEN, MAVLINK_MSG_ID_COMPASSMOT_STATUS_LEN, MAVLINK_MSG_ID_COMPASSMOT_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE COMPASSMOT_STATUS UNPACKING


/**
 * @brief Get field throttle from compassmot_status message
 *
 * @return throttle (percent*10)
 */
static inline uint16_t mavlink_msg_compassmot_status_get_throttle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field current from compassmot_status message
 *
 * @return current (Ampere)
 */
static inline float mavlink_msg_compassmot_status_get_current(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field interference from compassmot_status message
 *
 * @return interference (percent)
 */
static inline uint16_t mavlink_msg_compassmot_status_get_interference(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  18);
}

/**
 * @brief Get field CompensationX from compassmot_status message
 *
 * @return Motor Compensation X
 */
static inline float mavlink_msg_compassmot_status_get_CompensationX(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field CompensationY from compassmot_status message
 *
 * @return Motor Compensation Y
 */
static inline float mavlink_msg_compassmot_status_get_CompensationY(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field CompensationZ from compassmot_status message
 *
 * @return Motor Compensation Z
 */
static inline float mavlink_msg_compassmot_status_get_CompensationZ(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a compassmot_status message into a struct
 *
 * @param msg The message to decode
 * @param compassmot_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_compassmot_status_decode(const mavlink_message_t* msg, mavlink_compassmot_status_t* compassmot_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    compassmot_status->current = mavlink_msg_compassmot_status_get_current(msg);
    compassmot_status->CompensationX = mavlink_msg_compassmot_status_get_CompensationX(msg);
    compassmot_status->CompensationY = mavlink_msg_compassmot_status_get_CompensationY(msg);
    compassmot_status->CompensationZ = mavlink_msg_compassmot_status_get_CompensationZ(msg);
    compassmot_status->throttle = mavlink_msg_compassmot_status_get_throttle(msg);
    compassmot_status->interference = mavlink_msg_compassmot_status_get_interference(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_COMPASSMOT_STATUS_LEN? msg->len : MAVLINK_MSG_ID_COMPASSMOT_STATUS_LEN;
        memset(compassmot_status, 0, MAVLINK_MSG_ID_COMPASSMOT_STATUS_LEN);
    memcpy(compassmot_status, _MAV_PAYLOAD(msg), len);
#endif
}
