#pragma once
// MESSAGE AVSS_DRONE_OPERATION_MODE PACKING

#define MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE 60053


typedef struct __mavlink_avss_drone_operation_mode_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since FC boot).*/
 uint8_t M300_operation_mode; /*<  DJI M300 operation mode*/
 uint8_t horsefly_operation_mode; /*<  horsefly operation mode*/
} mavlink_avss_drone_operation_mode_t;

#define MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_LEN 6
#define MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_MIN_LEN 6
#define MAVLINK_MSG_ID_60053_LEN 6
#define MAVLINK_MSG_ID_60053_MIN_LEN 6

#define MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_CRC 45
#define MAVLINK_MSG_ID_60053_CRC 45



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AVSS_DRONE_OPERATION_MODE { \
    60053, \
    "AVSS_DRONE_OPERATION_MODE", \
    3, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_avss_drone_operation_mode_t, time_boot_ms) }, \
         { "M300_operation_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_avss_drone_operation_mode_t, M300_operation_mode) }, \
         { "horsefly_operation_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_avss_drone_operation_mode_t, horsefly_operation_mode) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AVSS_DRONE_OPERATION_MODE { \
    "AVSS_DRONE_OPERATION_MODE", \
    3, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_avss_drone_operation_mode_t, time_boot_ms) }, \
         { "M300_operation_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_avss_drone_operation_mode_t, M300_operation_mode) }, \
         { "horsefly_operation_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_avss_drone_operation_mode_t, horsefly_operation_mode) }, \
         } \
}
#endif

/**
 * @brief Pack a avss_drone_operation_mode message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since FC boot).
 * @param M300_operation_mode  DJI M300 operation mode
 * @param horsefly_operation_mode  horsefly operation mode
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_avss_drone_operation_mode_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint8_t M300_operation_mode, uint8_t horsefly_operation_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint8_t(buf, 4, M300_operation_mode);
    _mav_put_uint8_t(buf, 5, horsefly_operation_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_LEN);
#else
    mavlink_avss_drone_operation_mode_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.M300_operation_mode = M300_operation_mode;
    packet.horsefly_operation_mode = horsefly_operation_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_MIN_LEN, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_LEN, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_CRC);
}

/**
 * @brief Pack a avss_drone_operation_mode message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since FC boot).
 * @param M300_operation_mode  DJI M300 operation mode
 * @param horsefly_operation_mode  horsefly operation mode
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_avss_drone_operation_mode_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint8_t M300_operation_mode, uint8_t horsefly_operation_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint8_t(buf, 4, M300_operation_mode);
    _mav_put_uint8_t(buf, 5, horsefly_operation_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_LEN);
#else
    mavlink_avss_drone_operation_mode_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.M300_operation_mode = M300_operation_mode;
    packet.horsefly_operation_mode = horsefly_operation_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_MIN_LEN, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_LEN, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_MIN_LEN, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_LEN);
#endif
}

/**
 * @brief Pack a avss_drone_operation_mode message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since FC boot).
 * @param M300_operation_mode  DJI M300 operation mode
 * @param horsefly_operation_mode  horsefly operation mode
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_avss_drone_operation_mode_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,uint8_t M300_operation_mode,uint8_t horsefly_operation_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint8_t(buf, 4, M300_operation_mode);
    _mav_put_uint8_t(buf, 5, horsefly_operation_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_LEN);
#else
    mavlink_avss_drone_operation_mode_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.M300_operation_mode = M300_operation_mode;
    packet.horsefly_operation_mode = horsefly_operation_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_MIN_LEN, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_LEN, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_CRC);
}

/**
 * @brief Encode a avss_drone_operation_mode struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param avss_drone_operation_mode C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_avss_drone_operation_mode_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_avss_drone_operation_mode_t* avss_drone_operation_mode)
{
    return mavlink_msg_avss_drone_operation_mode_pack(system_id, component_id, msg, avss_drone_operation_mode->time_boot_ms, avss_drone_operation_mode->M300_operation_mode, avss_drone_operation_mode->horsefly_operation_mode);
}

/**
 * @brief Encode a avss_drone_operation_mode struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param avss_drone_operation_mode C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_avss_drone_operation_mode_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_avss_drone_operation_mode_t* avss_drone_operation_mode)
{
    return mavlink_msg_avss_drone_operation_mode_pack_chan(system_id, component_id, chan, msg, avss_drone_operation_mode->time_boot_ms, avss_drone_operation_mode->M300_operation_mode, avss_drone_operation_mode->horsefly_operation_mode);
}

/**
 * @brief Encode a avss_drone_operation_mode struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param avss_drone_operation_mode C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_avss_drone_operation_mode_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_avss_drone_operation_mode_t* avss_drone_operation_mode)
{
    return mavlink_msg_avss_drone_operation_mode_pack_status(system_id, component_id, _status, msg,  avss_drone_operation_mode->time_boot_ms, avss_drone_operation_mode->M300_operation_mode, avss_drone_operation_mode->horsefly_operation_mode);
}

/**
 * @brief Send a avss_drone_operation_mode message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since FC boot).
 * @param M300_operation_mode  DJI M300 operation mode
 * @param horsefly_operation_mode  horsefly operation mode
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_avss_drone_operation_mode_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t M300_operation_mode, uint8_t horsefly_operation_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint8_t(buf, 4, M300_operation_mode);
    _mav_put_uint8_t(buf, 5, horsefly_operation_mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE, buf, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_MIN_LEN, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_LEN, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_CRC);
#else
    mavlink_avss_drone_operation_mode_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.M300_operation_mode = M300_operation_mode;
    packet.horsefly_operation_mode = horsefly_operation_mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE, (const char *)&packet, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_MIN_LEN, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_LEN, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_CRC);
#endif
}

/**
 * @brief Send a avss_drone_operation_mode message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_avss_drone_operation_mode_send_struct(mavlink_channel_t chan, const mavlink_avss_drone_operation_mode_t* avss_drone_operation_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_avss_drone_operation_mode_send(chan, avss_drone_operation_mode->time_boot_ms, avss_drone_operation_mode->M300_operation_mode, avss_drone_operation_mode->horsefly_operation_mode);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE, (const char *)avss_drone_operation_mode, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_MIN_LEN, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_LEN, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_CRC);
#endif
}

#if MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_avss_drone_operation_mode_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint8_t M300_operation_mode, uint8_t horsefly_operation_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint8_t(buf, 4, M300_operation_mode);
    _mav_put_uint8_t(buf, 5, horsefly_operation_mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE, buf, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_MIN_LEN, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_LEN, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_CRC);
#else
    mavlink_avss_drone_operation_mode_t *packet = (mavlink_avss_drone_operation_mode_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->M300_operation_mode = M300_operation_mode;
    packet->horsefly_operation_mode = horsefly_operation_mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE, (const char *)packet, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_MIN_LEN, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_LEN, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_CRC);
#endif
}
#endif

#endif

// MESSAGE AVSS_DRONE_OPERATION_MODE UNPACKING


/**
 * @brief Get field time_boot_ms from avss_drone_operation_mode message
 *
 * @return [ms] Timestamp (time since FC boot).
 */
static inline uint32_t mavlink_msg_avss_drone_operation_mode_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field M300_operation_mode from avss_drone_operation_mode message
 *
 * @return  DJI M300 operation mode
 */
static inline uint8_t mavlink_msg_avss_drone_operation_mode_get_M300_operation_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field horsefly_operation_mode from avss_drone_operation_mode message
 *
 * @return  horsefly operation mode
 */
static inline uint8_t mavlink_msg_avss_drone_operation_mode_get_horsefly_operation_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Decode a avss_drone_operation_mode message into a struct
 *
 * @param msg The message to decode
 * @param avss_drone_operation_mode C-struct to decode the message contents into
 */
static inline void mavlink_msg_avss_drone_operation_mode_decode(const mavlink_message_t* msg, mavlink_avss_drone_operation_mode_t* avss_drone_operation_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    avss_drone_operation_mode->time_boot_ms = mavlink_msg_avss_drone_operation_mode_get_time_boot_ms(msg);
    avss_drone_operation_mode->M300_operation_mode = mavlink_msg_avss_drone_operation_mode_get_M300_operation_mode(msg);
    avss_drone_operation_mode->horsefly_operation_mode = mavlink_msg_avss_drone_operation_mode_get_horsefly_operation_mode(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_LEN? msg->len : MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_LEN;
        memset(avss_drone_operation_mode, 0, MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_LEN);
    memcpy(avss_drone_operation_mode, _MAV_PAYLOAD(msg), len);
#endif
}
