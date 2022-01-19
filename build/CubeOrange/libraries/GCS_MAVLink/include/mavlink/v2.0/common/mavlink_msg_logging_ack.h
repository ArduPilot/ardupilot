#pragma once
// MESSAGE LOGGING_ACK PACKING

#define MAVLINK_MSG_ID_LOGGING_ACK 268


typedef struct __mavlink_logging_ack_t {
 uint16_t sequence; /*<  sequence number (must match the one in LOGGING_DATA_ACKED)*/
 uint8_t target_system; /*<  system ID of the target*/
 uint8_t target_component; /*<  component ID of the target*/
} mavlink_logging_ack_t;

#define MAVLINK_MSG_ID_LOGGING_ACK_LEN 4
#define MAVLINK_MSG_ID_LOGGING_ACK_MIN_LEN 4
#define MAVLINK_MSG_ID_268_LEN 4
#define MAVLINK_MSG_ID_268_MIN_LEN 4

#define MAVLINK_MSG_ID_LOGGING_ACK_CRC 14
#define MAVLINK_MSG_ID_268_CRC 14



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LOGGING_ACK { \
    268, \
    "LOGGING_ACK", \
    3, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_logging_ack_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_logging_ack_t, target_component) }, \
         { "sequence", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_logging_ack_t, sequence) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LOGGING_ACK { \
    "LOGGING_ACK", \
    3, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_logging_ack_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_logging_ack_t, target_component) }, \
         { "sequence", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_logging_ack_t, sequence) }, \
         } \
}
#endif

/**
 * @brief Pack a logging_ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  system ID of the target
 * @param target_component  component ID of the target
 * @param sequence  sequence number (must match the one in LOGGING_DATA_ACKED)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_logging_ack_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint16_t sequence)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LOGGING_ACK_LEN];
    _mav_put_uint16_t(buf, 0, sequence);
    _mav_put_uint8_t(buf, 2, target_system);
    _mav_put_uint8_t(buf, 3, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LOGGING_ACK_LEN);
#else
    mavlink_logging_ack_t packet;
    packet.sequence = sequence;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LOGGING_ACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LOGGING_ACK;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LOGGING_ACK_MIN_LEN, MAVLINK_MSG_ID_LOGGING_ACK_LEN, MAVLINK_MSG_ID_LOGGING_ACK_CRC);
}

/**
 * @brief Pack a logging_ack message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  system ID of the target
 * @param target_component  component ID of the target
 * @param sequence  sequence number (must match the one in LOGGING_DATA_ACKED)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_logging_ack_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint16_t sequence)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LOGGING_ACK_LEN];
    _mav_put_uint16_t(buf, 0, sequence);
    _mav_put_uint8_t(buf, 2, target_system);
    _mav_put_uint8_t(buf, 3, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LOGGING_ACK_LEN);
#else
    mavlink_logging_ack_t packet;
    packet.sequence = sequence;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LOGGING_ACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LOGGING_ACK;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LOGGING_ACK_MIN_LEN, MAVLINK_MSG_ID_LOGGING_ACK_LEN, MAVLINK_MSG_ID_LOGGING_ACK_CRC);
}

/**
 * @brief Encode a logging_ack struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param logging_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_logging_ack_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_logging_ack_t* logging_ack)
{
    return mavlink_msg_logging_ack_pack(system_id, component_id, msg, logging_ack->target_system, logging_ack->target_component, logging_ack->sequence);
}

/**
 * @brief Encode a logging_ack struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param logging_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_logging_ack_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_logging_ack_t* logging_ack)
{
    return mavlink_msg_logging_ack_pack_chan(system_id, component_id, chan, msg, logging_ack->target_system, logging_ack->target_component, logging_ack->sequence);
}

/**
 * @brief Send a logging_ack message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  system ID of the target
 * @param target_component  component ID of the target
 * @param sequence  sequence number (must match the one in LOGGING_DATA_ACKED)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_logging_ack_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint16_t sequence)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LOGGING_ACK_LEN];
    _mav_put_uint16_t(buf, 0, sequence);
    _mav_put_uint8_t(buf, 2, target_system);
    _mav_put_uint8_t(buf, 3, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOGGING_ACK, buf, MAVLINK_MSG_ID_LOGGING_ACK_MIN_LEN, MAVLINK_MSG_ID_LOGGING_ACK_LEN, MAVLINK_MSG_ID_LOGGING_ACK_CRC);
#else
    mavlink_logging_ack_t packet;
    packet.sequence = sequence;
    packet.target_system = target_system;
    packet.target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOGGING_ACK, (const char *)&packet, MAVLINK_MSG_ID_LOGGING_ACK_MIN_LEN, MAVLINK_MSG_ID_LOGGING_ACK_LEN, MAVLINK_MSG_ID_LOGGING_ACK_CRC);
#endif
}

/**
 * @brief Send a logging_ack message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_logging_ack_send_struct(mavlink_channel_t chan, const mavlink_logging_ack_t* logging_ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_logging_ack_send(chan, logging_ack->target_system, logging_ack->target_component, logging_ack->sequence);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOGGING_ACK, (const char *)logging_ack, MAVLINK_MSG_ID_LOGGING_ACK_MIN_LEN, MAVLINK_MSG_ID_LOGGING_ACK_LEN, MAVLINK_MSG_ID_LOGGING_ACK_CRC);
#endif
}

#if MAVLINK_MSG_ID_LOGGING_ACK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_logging_ack_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint16_t sequence)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, sequence);
    _mav_put_uint8_t(buf, 2, target_system);
    _mav_put_uint8_t(buf, 3, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOGGING_ACK, buf, MAVLINK_MSG_ID_LOGGING_ACK_MIN_LEN, MAVLINK_MSG_ID_LOGGING_ACK_LEN, MAVLINK_MSG_ID_LOGGING_ACK_CRC);
#else
    mavlink_logging_ack_t *packet = (mavlink_logging_ack_t *)msgbuf;
    packet->sequence = sequence;
    packet->target_system = target_system;
    packet->target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOGGING_ACK, (const char *)packet, MAVLINK_MSG_ID_LOGGING_ACK_MIN_LEN, MAVLINK_MSG_ID_LOGGING_ACK_LEN, MAVLINK_MSG_ID_LOGGING_ACK_CRC);
#endif
}
#endif

#endif

// MESSAGE LOGGING_ACK UNPACKING


/**
 * @brief Get field target_system from logging_ack message
 *
 * @return  system ID of the target
 */
static inline uint8_t mavlink_msg_logging_ack_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field target_component from logging_ack message
 *
 * @return  component ID of the target
 */
static inline uint8_t mavlink_msg_logging_ack_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field sequence from logging_ack message
 *
 * @return  sequence number (must match the one in LOGGING_DATA_ACKED)
 */
static inline uint16_t mavlink_msg_logging_ack_get_sequence(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a logging_ack message into a struct
 *
 * @param msg The message to decode
 * @param logging_ack C-struct to decode the message contents into
 */
static inline void mavlink_msg_logging_ack_decode(const mavlink_message_t* msg, mavlink_logging_ack_t* logging_ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    logging_ack->sequence = mavlink_msg_logging_ack_get_sequence(msg);
    logging_ack->target_system = mavlink_msg_logging_ack_get_target_system(msg);
    logging_ack->target_component = mavlink_msg_logging_ack_get_target_component(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LOGGING_ACK_LEN? msg->len : MAVLINK_MSG_ID_LOGGING_ACK_LEN;
        memset(logging_ack, 0, MAVLINK_MSG_ID_LOGGING_ACK_LEN);
    memcpy(logging_ack, _MAV_PAYLOAD(msg), len);
#endif
}
