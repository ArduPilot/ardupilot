#pragma once
// MESSAGE ACTUATOR_OUTPUT_STATUS PACKING

#define MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS 375


typedef struct __mavlink_actuator_output_status_t {
 uint64_t time_usec; /*< [us] Timestamp (since system boot).*/
 uint32_t active; /*<  Active outputs*/
 float actuator[32]; /*<  Servo / motor output array values. Zero values indicate unused channels.*/
} mavlink_actuator_output_status_t;

#define MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_LEN 140
#define MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_MIN_LEN 140
#define MAVLINK_MSG_ID_375_LEN 140
#define MAVLINK_MSG_ID_375_MIN_LEN 140

#define MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_CRC 251
#define MAVLINK_MSG_ID_375_CRC 251

#define MAVLINK_MSG_ACTUATOR_OUTPUT_STATUS_FIELD_ACTUATOR_LEN 32

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ACTUATOR_OUTPUT_STATUS { \
    375, \
    "ACTUATOR_OUTPUT_STATUS", \
    3, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_actuator_output_status_t, time_usec) }, \
         { "active", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_actuator_output_status_t, active) }, \
         { "actuator", NULL, MAVLINK_TYPE_FLOAT, 32, 12, offsetof(mavlink_actuator_output_status_t, actuator) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ACTUATOR_OUTPUT_STATUS { \
    "ACTUATOR_OUTPUT_STATUS", \
    3, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_actuator_output_status_t, time_usec) }, \
         { "active", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_actuator_output_status_t, active) }, \
         { "actuator", NULL, MAVLINK_TYPE_FLOAT, 32, 12, offsetof(mavlink_actuator_output_status_t, actuator) }, \
         } \
}
#endif

/**
 * @brief Pack a actuator_output_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (since system boot).
 * @param active  Active outputs
 * @param actuator  Servo / motor output array values. Zero values indicate unused channels.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_actuator_output_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, uint32_t active, const float *actuator)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, active);
    _mav_put_float_array(buf, 12, actuator, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_LEN);
#else
    mavlink_actuator_output_status_t packet;
    packet.time_usec = time_usec;
    packet.active = active;
    mav_array_memcpy(packet.actuator, actuator, sizeof(float)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_LEN, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_CRC);
}

/**
 * @brief Pack a actuator_output_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (since system boot).
 * @param active  Active outputs
 * @param actuator  Servo / motor output array values. Zero values indicate unused channels.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_actuator_output_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t time_usec, uint32_t active, const float *actuator)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, active);
    _mav_put_float_array(buf, 12, actuator, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_LEN);
#else
    mavlink_actuator_output_status_t packet;
    packet.time_usec = time_usec;
    packet.active = active;
    mav_array_memcpy(packet.actuator, actuator, sizeof(float)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_LEN, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_LEN);
#endif
}

/**
 * @brief Pack a actuator_output_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (since system boot).
 * @param active  Active outputs
 * @param actuator  Servo / motor output array values. Zero values indicate unused channels.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_actuator_output_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,uint32_t active,const float *actuator)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, active);
    _mav_put_float_array(buf, 12, actuator, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_LEN);
#else
    mavlink_actuator_output_status_t packet;
    packet.time_usec = time_usec;
    packet.active = active;
    mav_array_memcpy(packet.actuator, actuator, sizeof(float)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_LEN, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_CRC);
}

/**
 * @brief Encode a actuator_output_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param actuator_output_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_actuator_output_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_actuator_output_status_t* actuator_output_status)
{
    return mavlink_msg_actuator_output_status_pack(system_id, component_id, msg, actuator_output_status->time_usec, actuator_output_status->active, actuator_output_status->actuator);
}

/**
 * @brief Encode a actuator_output_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param actuator_output_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_actuator_output_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_actuator_output_status_t* actuator_output_status)
{
    return mavlink_msg_actuator_output_status_pack_chan(system_id, component_id, chan, msg, actuator_output_status->time_usec, actuator_output_status->active, actuator_output_status->actuator);
}

/**
 * @brief Encode a actuator_output_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param actuator_output_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_actuator_output_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_actuator_output_status_t* actuator_output_status)
{
    return mavlink_msg_actuator_output_status_pack_status(system_id, component_id, _status, msg,  actuator_output_status->time_usec, actuator_output_status->active, actuator_output_status->actuator);
}

/**
 * @brief Send a actuator_output_status message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (since system boot).
 * @param active  Active outputs
 * @param actuator  Servo / motor output array values. Zero values indicate unused channels.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_actuator_output_status_send(mavlink_channel_t chan, uint64_t time_usec, uint32_t active, const float *actuator)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, active);
    _mav_put_float_array(buf, 12, actuator, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS, buf, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_LEN, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_CRC);
#else
    mavlink_actuator_output_status_t packet;
    packet.time_usec = time_usec;
    packet.active = active;
    mav_array_memcpy(packet.actuator, actuator, sizeof(float)*32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS, (const char *)&packet, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_LEN, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_CRC);
#endif
}

/**
 * @brief Send a actuator_output_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_actuator_output_status_send_struct(mavlink_channel_t chan, const mavlink_actuator_output_status_t* actuator_output_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_actuator_output_status_send(chan, actuator_output_status->time_usec, actuator_output_status->active, actuator_output_status->actuator);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS, (const char *)actuator_output_status, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_LEN, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_actuator_output_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint32_t active, const float *actuator)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, active);
    _mav_put_float_array(buf, 12, actuator, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS, buf, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_LEN, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_CRC);
#else
    mavlink_actuator_output_status_t *packet = (mavlink_actuator_output_status_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->active = active;
    mav_array_memcpy(packet->actuator, actuator, sizeof(float)*32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS, (const char *)packet, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_LEN, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE ACTUATOR_OUTPUT_STATUS UNPACKING


/**
 * @brief Get field time_usec from actuator_output_status message
 *
 * @return [us] Timestamp (since system boot).
 */
static inline uint64_t mavlink_msg_actuator_output_status_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field active from actuator_output_status message
 *
 * @return  Active outputs
 */
static inline uint32_t mavlink_msg_actuator_output_status_get_active(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field actuator from actuator_output_status message
 *
 * @return  Servo / motor output array values. Zero values indicate unused channels.
 */
static inline uint16_t mavlink_msg_actuator_output_status_get_actuator(const mavlink_message_t* msg, float *actuator)
{
    return _MAV_RETURN_float_array(msg, actuator, 32,  12);
}

/**
 * @brief Decode a actuator_output_status message into a struct
 *
 * @param msg The message to decode
 * @param actuator_output_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_actuator_output_status_decode(const mavlink_message_t* msg, mavlink_actuator_output_status_t* actuator_output_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    actuator_output_status->time_usec = mavlink_msg_actuator_output_status_get_time_usec(msg);
    actuator_output_status->active = mavlink_msg_actuator_output_status_get_active(msg);
    mavlink_msg_actuator_output_status_get_actuator(msg, actuator_output_status->actuator);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_LEN? msg->len : MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_LEN;
        memset(actuator_output_status, 0, MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_LEN);
    memcpy(actuator_output_status, _MAV_PAYLOAD(msg), len);
#endif
}
