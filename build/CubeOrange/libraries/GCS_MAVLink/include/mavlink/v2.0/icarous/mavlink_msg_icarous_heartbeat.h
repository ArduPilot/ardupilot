#pragma once
// MESSAGE ICAROUS_HEARTBEAT PACKING

#define MAVLINK_MSG_ID_ICAROUS_HEARTBEAT 42000


typedef struct __mavlink_icarous_heartbeat_t {
 uint8_t status; /*<  See the FMS_STATE enum.*/
} mavlink_icarous_heartbeat_t;

#define MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_LEN 1
#define MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_MIN_LEN 1
#define MAVLINK_MSG_ID_42000_LEN 1
#define MAVLINK_MSG_ID_42000_MIN_LEN 1

#define MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_CRC 227
#define MAVLINK_MSG_ID_42000_CRC 227



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ICAROUS_HEARTBEAT { \
    42000, \
    "ICAROUS_HEARTBEAT", \
    1, \
    {  { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_icarous_heartbeat_t, status) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ICAROUS_HEARTBEAT { \
    "ICAROUS_HEARTBEAT", \
    1, \
    {  { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_icarous_heartbeat_t, status) }, \
         } \
}
#endif

/**
 * @brief Pack a icarous_heartbeat message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param status  See the FMS_STATE enum.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_icarous_heartbeat_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_LEN];
    _mav_put_uint8_t(buf, 0, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_LEN);
#else
    mavlink_icarous_heartbeat_t packet;
    packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ICAROUS_HEARTBEAT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_LEN, MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_CRC);
}

/**
 * @brief Pack a icarous_heartbeat message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param status  See the FMS_STATE enum.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_icarous_heartbeat_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_LEN];
    _mav_put_uint8_t(buf, 0, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_LEN);
#else
    mavlink_icarous_heartbeat_t packet;
    packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ICAROUS_HEARTBEAT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_LEN, MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_CRC);
}

/**
 * @brief Encode a icarous_heartbeat struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param icarous_heartbeat C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_icarous_heartbeat_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_icarous_heartbeat_t* icarous_heartbeat)
{
    return mavlink_msg_icarous_heartbeat_pack(system_id, component_id, msg, icarous_heartbeat->status);
}

/**
 * @brief Encode a icarous_heartbeat struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param icarous_heartbeat C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_icarous_heartbeat_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_icarous_heartbeat_t* icarous_heartbeat)
{
    return mavlink_msg_icarous_heartbeat_pack_chan(system_id, component_id, chan, msg, icarous_heartbeat->status);
}

/**
 * @brief Send a icarous_heartbeat message
 * @param chan MAVLink channel to send the message
 *
 * @param status  See the FMS_STATE enum.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_icarous_heartbeat_send(mavlink_channel_t chan, uint8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_LEN];
    _mav_put_uint8_t(buf, 0, status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ICAROUS_HEARTBEAT, buf, MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_LEN, MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_CRC);
#else
    mavlink_icarous_heartbeat_t packet;
    packet.status = status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ICAROUS_HEARTBEAT, (const char *)&packet, MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_LEN, MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_CRC);
#endif
}

/**
 * @brief Send a icarous_heartbeat message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_icarous_heartbeat_send_struct(mavlink_channel_t chan, const mavlink_icarous_heartbeat_t* icarous_heartbeat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_icarous_heartbeat_send(chan, icarous_heartbeat->status);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ICAROUS_HEARTBEAT, (const char *)icarous_heartbeat, MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_LEN, MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_CRC);
#endif
}

#if MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_icarous_heartbeat_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ICAROUS_HEARTBEAT, buf, MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_LEN, MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_CRC);
#else
    mavlink_icarous_heartbeat_t *packet = (mavlink_icarous_heartbeat_t *)msgbuf;
    packet->status = status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ICAROUS_HEARTBEAT, (const char *)packet, MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_LEN, MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_CRC);
#endif
}
#endif

#endif

// MESSAGE ICAROUS_HEARTBEAT UNPACKING


/**
 * @brief Get field status from icarous_heartbeat message
 *
 * @return  See the FMS_STATE enum.
 */
static inline uint8_t mavlink_msg_icarous_heartbeat_get_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a icarous_heartbeat message into a struct
 *
 * @param msg The message to decode
 * @param icarous_heartbeat C-struct to decode the message contents into
 */
static inline void mavlink_msg_icarous_heartbeat_decode(const mavlink_message_t* msg, mavlink_icarous_heartbeat_t* icarous_heartbeat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    icarous_heartbeat->status = mavlink_msg_icarous_heartbeat_get_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_LEN? msg->len : MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_LEN;
        memset(icarous_heartbeat, 0, MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_LEN);
    memcpy(icarous_heartbeat, _MAV_PAYLOAD(msg), len);
#endif
}
