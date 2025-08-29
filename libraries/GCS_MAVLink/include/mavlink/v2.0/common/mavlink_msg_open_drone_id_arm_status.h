#pragma once
// MESSAGE OPEN_DRONE_ID_ARM_STATUS PACKING

#define MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS 12918


typedef struct __mavlink_open_drone_id_arm_status_t {
 uint8_t status; /*<  Status level indicating if arming is allowed.*/
 char error[50]; /*<  Text error message, should be empty if status is good to arm. Fill with nulls in unused portion.*/
} mavlink_open_drone_id_arm_status_t;

#define MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_LEN 51
#define MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_MIN_LEN 51
#define MAVLINK_MSG_ID_12918_LEN 51
#define MAVLINK_MSG_ID_12918_MIN_LEN 51

#define MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_CRC 139
#define MAVLINK_MSG_ID_12918_CRC 139

#define MAVLINK_MSG_OPEN_DRONE_ID_ARM_STATUS_FIELD_ERROR_LEN 50

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_OPEN_DRONE_ID_ARM_STATUS { \
    12918, \
    "OPEN_DRONE_ID_ARM_STATUS", \
    2, \
    {  { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_open_drone_id_arm_status_t, status) }, \
         { "error", NULL, MAVLINK_TYPE_CHAR, 50, 1, offsetof(mavlink_open_drone_id_arm_status_t, error) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_OPEN_DRONE_ID_ARM_STATUS { \
    "OPEN_DRONE_ID_ARM_STATUS", \
    2, \
    {  { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_open_drone_id_arm_status_t, status) }, \
         { "error", NULL, MAVLINK_TYPE_CHAR, 50, 1, offsetof(mavlink_open_drone_id_arm_status_t, error) }, \
         } \
}
#endif

/**
 * @brief Pack a open_drone_id_arm_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param status  Status level indicating if arming is allowed.
 * @param error  Text error message, should be empty if status is good to arm. Fill with nulls in unused portion.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_open_drone_id_arm_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t status, const char *error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_LEN];
    _mav_put_uint8_t(buf, 0, status);
    _mav_put_char_array(buf, 1, error, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_LEN);
#else
    mavlink_open_drone_id_arm_status_t packet;
    packet.status = status;
    mav_array_memcpy(packet.error, error, sizeof(char)*50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_CRC);
}

/**
 * @brief Pack a open_drone_id_arm_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param status  Status level indicating if arming is allowed.
 * @param error  Text error message, should be empty if status is good to arm. Fill with nulls in unused portion.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_open_drone_id_arm_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t status, const char *error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_LEN];
    _mav_put_uint8_t(buf, 0, status);
    _mav_put_char_array(buf, 1, error, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_LEN);
#else
    mavlink_open_drone_id_arm_status_t packet;
    packet.status = status;
    mav_array_memcpy(packet.error, error, sizeof(char)*50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_LEN);
#endif
}

/**
 * @brief Pack a open_drone_id_arm_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param status  Status level indicating if arming is allowed.
 * @param error  Text error message, should be empty if status is good to arm. Fill with nulls in unused portion.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_open_drone_id_arm_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t status,const char *error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_LEN];
    _mav_put_uint8_t(buf, 0, status);
    _mav_put_char_array(buf, 1, error, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_LEN);
#else
    mavlink_open_drone_id_arm_status_t packet;
    packet.status = status;
    mav_array_memcpy(packet.error, error, sizeof(char)*50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_CRC);
}

/**
 * @brief Encode a open_drone_id_arm_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param open_drone_id_arm_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_open_drone_id_arm_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_open_drone_id_arm_status_t* open_drone_id_arm_status)
{
    return mavlink_msg_open_drone_id_arm_status_pack(system_id, component_id, msg, open_drone_id_arm_status->status, open_drone_id_arm_status->error);
}

/**
 * @brief Encode a open_drone_id_arm_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param open_drone_id_arm_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_open_drone_id_arm_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_open_drone_id_arm_status_t* open_drone_id_arm_status)
{
    return mavlink_msg_open_drone_id_arm_status_pack_chan(system_id, component_id, chan, msg, open_drone_id_arm_status->status, open_drone_id_arm_status->error);
}

/**
 * @brief Encode a open_drone_id_arm_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param open_drone_id_arm_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_open_drone_id_arm_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_open_drone_id_arm_status_t* open_drone_id_arm_status)
{
    return mavlink_msg_open_drone_id_arm_status_pack_status(system_id, component_id, _status, msg,  open_drone_id_arm_status->status, open_drone_id_arm_status->error);
}

/**
 * @brief Send a open_drone_id_arm_status message
 * @param chan MAVLink channel to send the message
 *
 * @param status  Status level indicating if arming is allowed.
 * @param error  Text error message, should be empty if status is good to arm. Fill with nulls in unused portion.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_open_drone_id_arm_status_send(mavlink_channel_t chan, uint8_t status, const char *error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_LEN];
    _mav_put_uint8_t(buf, 0, status);
    _mav_put_char_array(buf, 1, error, 50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS, buf, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_CRC);
#else
    mavlink_open_drone_id_arm_status_t packet;
    packet.status = status;
    mav_array_memcpy(packet.error, error, sizeof(char)*50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS, (const char *)&packet, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_CRC);
#endif
}

/**
 * @brief Send a open_drone_id_arm_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_open_drone_id_arm_status_send_struct(mavlink_channel_t chan, const mavlink_open_drone_id_arm_status_t* open_drone_id_arm_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_open_drone_id_arm_status_send(chan, open_drone_id_arm_status->status, open_drone_id_arm_status->error);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS, (const char *)open_drone_id_arm_status, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_open_drone_id_arm_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t status, const char *error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, status);
    _mav_put_char_array(buf, 1, error, 50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS, buf, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_CRC);
#else
    mavlink_open_drone_id_arm_status_t *packet = (mavlink_open_drone_id_arm_status_t *)msgbuf;
    packet->status = status;
    mav_array_memcpy(packet->error, error, sizeof(char)*50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS, (const char *)packet, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE OPEN_DRONE_ID_ARM_STATUS UNPACKING


/**
 * @brief Get field status from open_drone_id_arm_status message
 *
 * @return  Status level indicating if arming is allowed.
 */
static inline uint8_t mavlink_msg_open_drone_id_arm_status_get_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field error from open_drone_id_arm_status message
 *
 * @return  Text error message, should be empty if status is good to arm. Fill with nulls in unused portion.
 */
static inline uint16_t mavlink_msg_open_drone_id_arm_status_get_error(const mavlink_message_t* msg, char *error)
{
    return _MAV_RETURN_char_array(msg, error, 50,  1);
}

/**
 * @brief Decode a open_drone_id_arm_status message into a struct
 *
 * @param msg The message to decode
 * @param open_drone_id_arm_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_open_drone_id_arm_status_decode(const mavlink_message_t* msg, mavlink_open_drone_id_arm_status_t* open_drone_id_arm_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    open_drone_id_arm_status->status = mavlink_msg_open_drone_id_arm_status_get_status(msg);
    mavlink_msg_open_drone_id_arm_status_get_error(msg, open_drone_id_arm_status->error);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_LEN? msg->len : MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_LEN;
        memset(open_drone_id_arm_status, 0, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_LEN);
    memcpy(open_drone_id_arm_status, _MAV_PAYLOAD(msg), len);
#endif
}
