#pragma once
// MESSAGE QSHOT_STATUS PACKING

#define MAVLINK_MSG_ID_QSHOT_STATUS 60020


typedef struct __mavlink_qshot_status_t {
 uint16_t mode; /*<  Current shot mode.*/
 uint16_t shot_state; /*<  Current state in the shot. States are specific to the selected shot mode.*/
} mavlink_qshot_status_t;

#define MAVLINK_MSG_ID_QSHOT_STATUS_LEN 4
#define MAVLINK_MSG_ID_QSHOT_STATUS_MIN_LEN 4
#define MAVLINK_MSG_ID_60020_LEN 4
#define MAVLINK_MSG_ID_60020_MIN_LEN 4

#define MAVLINK_MSG_ID_QSHOT_STATUS_CRC 202
#define MAVLINK_MSG_ID_60020_CRC 202



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_QSHOT_STATUS { \
    60020, \
    "QSHOT_STATUS", \
    2, \
    {  { "mode", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_qshot_status_t, mode) }, \
         { "shot_state", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_qshot_status_t, shot_state) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_QSHOT_STATUS { \
    "QSHOT_STATUS", \
    2, \
    {  { "mode", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_qshot_status_t, mode) }, \
         { "shot_state", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_qshot_status_t, shot_state) }, \
         } \
}
#endif

/**
 * @brief Pack a qshot_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param mode  Current shot mode.
 * @param shot_state  Current state in the shot. States are specific to the selected shot mode.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_qshot_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t mode, uint16_t shot_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_QSHOT_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, mode);
    _mav_put_uint16_t(buf, 2, shot_state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_QSHOT_STATUS_LEN);
#else
    mavlink_qshot_status_t packet;
    packet.mode = mode;
    packet.shot_state = shot_state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_QSHOT_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_QSHOT_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_QSHOT_STATUS_MIN_LEN, MAVLINK_MSG_ID_QSHOT_STATUS_LEN, MAVLINK_MSG_ID_QSHOT_STATUS_CRC);
}

/**
 * @brief Pack a qshot_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param mode  Current shot mode.
 * @param shot_state  Current state in the shot. States are specific to the selected shot mode.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_qshot_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint16_t mode, uint16_t shot_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_QSHOT_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, mode);
    _mav_put_uint16_t(buf, 2, shot_state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_QSHOT_STATUS_LEN);
#else
    mavlink_qshot_status_t packet;
    packet.mode = mode;
    packet.shot_state = shot_state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_QSHOT_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_QSHOT_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_QSHOT_STATUS_MIN_LEN, MAVLINK_MSG_ID_QSHOT_STATUS_LEN, MAVLINK_MSG_ID_QSHOT_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_QSHOT_STATUS_MIN_LEN, MAVLINK_MSG_ID_QSHOT_STATUS_LEN);
#endif
}

/**
 * @brief Pack a qshot_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mode  Current shot mode.
 * @param shot_state  Current state in the shot. States are specific to the selected shot mode.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_qshot_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t mode,uint16_t shot_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_QSHOT_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, mode);
    _mav_put_uint16_t(buf, 2, shot_state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_QSHOT_STATUS_LEN);
#else
    mavlink_qshot_status_t packet;
    packet.mode = mode;
    packet.shot_state = shot_state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_QSHOT_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_QSHOT_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_QSHOT_STATUS_MIN_LEN, MAVLINK_MSG_ID_QSHOT_STATUS_LEN, MAVLINK_MSG_ID_QSHOT_STATUS_CRC);
}

/**
 * @brief Encode a qshot_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param qshot_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_qshot_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_qshot_status_t* qshot_status)
{
    return mavlink_msg_qshot_status_pack(system_id, component_id, msg, qshot_status->mode, qshot_status->shot_state);
}

/**
 * @brief Encode a qshot_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param qshot_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_qshot_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_qshot_status_t* qshot_status)
{
    return mavlink_msg_qshot_status_pack_chan(system_id, component_id, chan, msg, qshot_status->mode, qshot_status->shot_state);
}

/**
 * @brief Encode a qshot_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param qshot_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_qshot_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_qshot_status_t* qshot_status)
{
    return mavlink_msg_qshot_status_pack_status(system_id, component_id, _status, msg,  qshot_status->mode, qshot_status->shot_state);
}

/**
 * @brief Send a qshot_status message
 * @param chan MAVLink channel to send the message
 *
 * @param mode  Current shot mode.
 * @param shot_state  Current state in the shot. States are specific to the selected shot mode.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_qshot_status_send(mavlink_channel_t chan, uint16_t mode, uint16_t shot_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_QSHOT_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, mode);
    _mav_put_uint16_t(buf, 2, shot_state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QSHOT_STATUS, buf, MAVLINK_MSG_ID_QSHOT_STATUS_MIN_LEN, MAVLINK_MSG_ID_QSHOT_STATUS_LEN, MAVLINK_MSG_ID_QSHOT_STATUS_CRC);
#else
    mavlink_qshot_status_t packet;
    packet.mode = mode;
    packet.shot_state = shot_state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QSHOT_STATUS, (const char *)&packet, MAVLINK_MSG_ID_QSHOT_STATUS_MIN_LEN, MAVLINK_MSG_ID_QSHOT_STATUS_LEN, MAVLINK_MSG_ID_QSHOT_STATUS_CRC);
#endif
}

/**
 * @brief Send a qshot_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_qshot_status_send_struct(mavlink_channel_t chan, const mavlink_qshot_status_t* qshot_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_qshot_status_send(chan, qshot_status->mode, qshot_status->shot_state);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QSHOT_STATUS, (const char *)qshot_status, MAVLINK_MSG_ID_QSHOT_STATUS_MIN_LEN, MAVLINK_MSG_ID_QSHOT_STATUS_LEN, MAVLINK_MSG_ID_QSHOT_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_QSHOT_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_qshot_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t mode, uint16_t shot_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, mode);
    _mav_put_uint16_t(buf, 2, shot_state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QSHOT_STATUS, buf, MAVLINK_MSG_ID_QSHOT_STATUS_MIN_LEN, MAVLINK_MSG_ID_QSHOT_STATUS_LEN, MAVLINK_MSG_ID_QSHOT_STATUS_CRC);
#else
    mavlink_qshot_status_t *packet = (mavlink_qshot_status_t *)msgbuf;
    packet->mode = mode;
    packet->shot_state = shot_state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QSHOT_STATUS, (const char *)packet, MAVLINK_MSG_ID_QSHOT_STATUS_MIN_LEN, MAVLINK_MSG_ID_QSHOT_STATUS_LEN, MAVLINK_MSG_ID_QSHOT_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE QSHOT_STATUS UNPACKING


/**
 * @brief Get field mode from qshot_status message
 *
 * @return  Current shot mode.
 */
static inline uint16_t mavlink_msg_qshot_status_get_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field shot_state from qshot_status message
 *
 * @return  Current state in the shot. States are specific to the selected shot mode.
 */
static inline uint16_t mavlink_msg_qshot_status_get_shot_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Decode a qshot_status message into a struct
 *
 * @param msg The message to decode
 * @param qshot_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_qshot_status_decode(const mavlink_message_t* msg, mavlink_qshot_status_t* qshot_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    qshot_status->mode = mavlink_msg_qshot_status_get_mode(msg);
    qshot_status->shot_state = mavlink_msg_qshot_status_get_shot_state(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_QSHOT_STATUS_LEN? msg->len : MAVLINK_MSG_ID_QSHOT_STATUS_LEN;
        memset(qshot_status, 0, MAVLINK_MSG_ID_QSHOT_STATUS_LEN);
    memcpy(qshot_status, _MAV_PAYLOAD(msg), len);
#endif
}
