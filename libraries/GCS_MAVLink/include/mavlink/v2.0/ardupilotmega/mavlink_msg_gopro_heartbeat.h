#pragma once
// MESSAGE GOPRO_HEARTBEAT PACKING

#define MAVLINK_MSG_ID_GOPRO_HEARTBEAT 215


typedef struct __mavlink_gopro_heartbeat_t {
 uint8_t status; /*<  Status.*/
 uint8_t capture_mode; /*<  Current capture mode.*/
 uint8_t flags; /*<  Additional status bits.*/
} mavlink_gopro_heartbeat_t;

#define MAVLINK_MSG_ID_GOPRO_HEARTBEAT_LEN 3
#define MAVLINK_MSG_ID_GOPRO_HEARTBEAT_MIN_LEN 3
#define MAVLINK_MSG_ID_215_LEN 3
#define MAVLINK_MSG_ID_215_MIN_LEN 3

#define MAVLINK_MSG_ID_GOPRO_HEARTBEAT_CRC 101
#define MAVLINK_MSG_ID_215_CRC 101



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GOPRO_HEARTBEAT { \
    215, \
    "GOPRO_HEARTBEAT", \
    3, \
    {  { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_gopro_heartbeat_t, status) }, \
         { "capture_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_gopro_heartbeat_t, capture_mode) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_gopro_heartbeat_t, flags) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GOPRO_HEARTBEAT { \
    "GOPRO_HEARTBEAT", \
    3, \
    {  { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_gopro_heartbeat_t, status) }, \
         { "capture_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_gopro_heartbeat_t, capture_mode) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_gopro_heartbeat_t, flags) }, \
         } \
}
#endif

/**
 * @brief Pack a gopro_heartbeat message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param status  Status.
 * @param capture_mode  Current capture mode.
 * @param flags  Additional status bits.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gopro_heartbeat_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t status, uint8_t capture_mode, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GOPRO_HEARTBEAT_LEN];
    _mav_put_uint8_t(buf, 0, status);
    _mav_put_uint8_t(buf, 1, capture_mode);
    _mav_put_uint8_t(buf, 2, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GOPRO_HEARTBEAT_LEN);
#else
    mavlink_gopro_heartbeat_t packet;
    packet.status = status;
    packet.capture_mode = capture_mode;
    packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GOPRO_HEARTBEAT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GOPRO_HEARTBEAT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GOPRO_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_GOPRO_HEARTBEAT_LEN, MAVLINK_MSG_ID_GOPRO_HEARTBEAT_CRC);
}

/**
 * @brief Pack a gopro_heartbeat message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param status  Status.
 * @param capture_mode  Current capture mode.
 * @param flags  Additional status bits.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gopro_heartbeat_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t status, uint8_t capture_mode, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GOPRO_HEARTBEAT_LEN];
    _mav_put_uint8_t(buf, 0, status);
    _mav_put_uint8_t(buf, 1, capture_mode);
    _mav_put_uint8_t(buf, 2, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GOPRO_HEARTBEAT_LEN);
#else
    mavlink_gopro_heartbeat_t packet;
    packet.status = status;
    packet.capture_mode = capture_mode;
    packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GOPRO_HEARTBEAT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GOPRO_HEARTBEAT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_GOPRO_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_GOPRO_HEARTBEAT_LEN, MAVLINK_MSG_ID_GOPRO_HEARTBEAT_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_GOPRO_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_GOPRO_HEARTBEAT_LEN);
#endif
}

/**
 * @brief Pack a gopro_heartbeat message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param status  Status.
 * @param capture_mode  Current capture mode.
 * @param flags  Additional status bits.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gopro_heartbeat_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t status,uint8_t capture_mode,uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GOPRO_HEARTBEAT_LEN];
    _mav_put_uint8_t(buf, 0, status);
    _mav_put_uint8_t(buf, 1, capture_mode);
    _mav_put_uint8_t(buf, 2, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GOPRO_HEARTBEAT_LEN);
#else
    mavlink_gopro_heartbeat_t packet;
    packet.status = status;
    packet.capture_mode = capture_mode;
    packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GOPRO_HEARTBEAT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GOPRO_HEARTBEAT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GOPRO_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_GOPRO_HEARTBEAT_LEN, MAVLINK_MSG_ID_GOPRO_HEARTBEAT_CRC);
}

/**
 * @brief Encode a gopro_heartbeat struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gopro_heartbeat C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gopro_heartbeat_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gopro_heartbeat_t* gopro_heartbeat)
{
    return mavlink_msg_gopro_heartbeat_pack(system_id, component_id, msg, gopro_heartbeat->status, gopro_heartbeat->capture_mode, gopro_heartbeat->flags);
}

/**
 * @brief Encode a gopro_heartbeat struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gopro_heartbeat C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gopro_heartbeat_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gopro_heartbeat_t* gopro_heartbeat)
{
    return mavlink_msg_gopro_heartbeat_pack_chan(system_id, component_id, chan, msg, gopro_heartbeat->status, gopro_heartbeat->capture_mode, gopro_heartbeat->flags);
}

/**
 * @brief Encode a gopro_heartbeat struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param gopro_heartbeat C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gopro_heartbeat_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_gopro_heartbeat_t* gopro_heartbeat)
{
    return mavlink_msg_gopro_heartbeat_pack_status(system_id, component_id, _status, msg,  gopro_heartbeat->status, gopro_heartbeat->capture_mode, gopro_heartbeat->flags);
}

/**
 * @brief Send a gopro_heartbeat message
 * @param chan MAVLink channel to send the message
 *
 * @param status  Status.
 * @param capture_mode  Current capture mode.
 * @param flags  Additional status bits.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gopro_heartbeat_send(mavlink_channel_t chan, uint8_t status, uint8_t capture_mode, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GOPRO_HEARTBEAT_LEN];
    _mav_put_uint8_t(buf, 0, status);
    _mav_put_uint8_t(buf, 1, capture_mode);
    _mav_put_uint8_t(buf, 2, flags);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GOPRO_HEARTBEAT, buf, MAVLINK_MSG_ID_GOPRO_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_GOPRO_HEARTBEAT_LEN, MAVLINK_MSG_ID_GOPRO_HEARTBEAT_CRC);
#else
    mavlink_gopro_heartbeat_t packet;
    packet.status = status;
    packet.capture_mode = capture_mode;
    packet.flags = flags;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GOPRO_HEARTBEAT, (const char *)&packet, MAVLINK_MSG_ID_GOPRO_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_GOPRO_HEARTBEAT_LEN, MAVLINK_MSG_ID_GOPRO_HEARTBEAT_CRC);
#endif
}

/**
 * @brief Send a gopro_heartbeat message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gopro_heartbeat_send_struct(mavlink_channel_t chan, const mavlink_gopro_heartbeat_t* gopro_heartbeat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gopro_heartbeat_send(chan, gopro_heartbeat->status, gopro_heartbeat->capture_mode, gopro_heartbeat->flags);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GOPRO_HEARTBEAT, (const char *)gopro_heartbeat, MAVLINK_MSG_ID_GOPRO_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_GOPRO_HEARTBEAT_LEN, MAVLINK_MSG_ID_GOPRO_HEARTBEAT_CRC);
#endif
}

#if MAVLINK_MSG_ID_GOPRO_HEARTBEAT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gopro_heartbeat_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t status, uint8_t capture_mode, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, status);
    _mav_put_uint8_t(buf, 1, capture_mode);
    _mav_put_uint8_t(buf, 2, flags);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GOPRO_HEARTBEAT, buf, MAVLINK_MSG_ID_GOPRO_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_GOPRO_HEARTBEAT_LEN, MAVLINK_MSG_ID_GOPRO_HEARTBEAT_CRC);
#else
    mavlink_gopro_heartbeat_t *packet = (mavlink_gopro_heartbeat_t *)msgbuf;
    packet->status = status;
    packet->capture_mode = capture_mode;
    packet->flags = flags;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GOPRO_HEARTBEAT, (const char *)packet, MAVLINK_MSG_ID_GOPRO_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_GOPRO_HEARTBEAT_LEN, MAVLINK_MSG_ID_GOPRO_HEARTBEAT_CRC);
#endif
}
#endif

#endif

// MESSAGE GOPRO_HEARTBEAT UNPACKING


/**
 * @brief Get field status from gopro_heartbeat message
 *
 * @return  Status.
 */
static inline uint8_t mavlink_msg_gopro_heartbeat_get_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field capture_mode from gopro_heartbeat message
 *
 * @return  Current capture mode.
 */
static inline uint8_t mavlink_msg_gopro_heartbeat_get_capture_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field flags from gopro_heartbeat message
 *
 * @return  Additional status bits.
 */
static inline uint8_t mavlink_msg_gopro_heartbeat_get_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a gopro_heartbeat message into a struct
 *
 * @param msg The message to decode
 * @param gopro_heartbeat C-struct to decode the message contents into
 */
static inline void mavlink_msg_gopro_heartbeat_decode(const mavlink_message_t* msg, mavlink_gopro_heartbeat_t* gopro_heartbeat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gopro_heartbeat->status = mavlink_msg_gopro_heartbeat_get_status(msg);
    gopro_heartbeat->capture_mode = mavlink_msg_gopro_heartbeat_get_capture_mode(msg);
    gopro_heartbeat->flags = mavlink_msg_gopro_heartbeat_get_flags(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GOPRO_HEARTBEAT_LEN? msg->len : MAVLINK_MSG_ID_GOPRO_HEARTBEAT_LEN;
        memset(gopro_heartbeat, 0, MAVLINK_MSG_ID_GOPRO_HEARTBEAT_LEN);
    memcpy(gopro_heartbeat, _MAV_PAYLOAD(msg), len);
#endif
}
