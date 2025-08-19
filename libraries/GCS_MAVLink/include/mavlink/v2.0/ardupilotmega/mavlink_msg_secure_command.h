#pragma once
// MESSAGE SECURE_COMMAND PACKING

#define MAVLINK_MSG_ID_SECURE_COMMAND 11004


typedef struct __mavlink_secure_command_t {
 uint32_t sequence; /*<  Sequence ID for tagging reply.*/
 uint32_t operation; /*<  Operation being requested.*/
 uint8_t target_system; /*<  System ID.*/
 uint8_t target_component; /*<  Component ID.*/
 uint8_t data_length; /*<  Data length.*/
 uint8_t sig_length; /*<  Signature length.*/
 uint8_t data[220]; /*<  Signed data.*/
} mavlink_secure_command_t;

#define MAVLINK_MSG_ID_SECURE_COMMAND_LEN 232
#define MAVLINK_MSG_ID_SECURE_COMMAND_MIN_LEN 232
#define MAVLINK_MSG_ID_11004_LEN 232
#define MAVLINK_MSG_ID_11004_MIN_LEN 232

#define MAVLINK_MSG_ID_SECURE_COMMAND_CRC 11
#define MAVLINK_MSG_ID_11004_CRC 11

#define MAVLINK_MSG_SECURE_COMMAND_FIELD_DATA_LEN 220

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SECURE_COMMAND { \
    11004, \
    "SECURE_COMMAND", \
    7, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_secure_command_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_secure_command_t, target_component) }, \
         { "sequence", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_secure_command_t, sequence) }, \
         { "operation", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_secure_command_t, operation) }, \
         { "data_length", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_secure_command_t, data_length) }, \
         { "sig_length", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_secure_command_t, sig_length) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 220, 12, offsetof(mavlink_secure_command_t, data) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SECURE_COMMAND { \
    "SECURE_COMMAND", \
    7, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_secure_command_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_secure_command_t, target_component) }, \
         { "sequence", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_secure_command_t, sequence) }, \
         { "operation", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_secure_command_t, operation) }, \
         { "data_length", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_secure_command_t, data_length) }, \
         { "sig_length", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_secure_command_t, sig_length) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 220, 12, offsetof(mavlink_secure_command_t, data) }, \
         } \
}
#endif

/**
 * @brief Pack a secure_command message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param sequence  Sequence ID for tagging reply.
 * @param operation  Operation being requested.
 * @param data_length  Data length.
 * @param sig_length  Signature length.
 * @param data  Signed data.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_secure_command_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint32_t sequence, uint32_t operation, uint8_t data_length, uint8_t sig_length, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SECURE_COMMAND_LEN];
    _mav_put_uint32_t(buf, 0, sequence);
    _mav_put_uint32_t(buf, 4, operation);
    _mav_put_uint8_t(buf, 8, target_system);
    _mav_put_uint8_t(buf, 9, target_component);
    _mav_put_uint8_t(buf, 10, data_length);
    _mav_put_uint8_t(buf, 11, sig_length);
    _mav_put_uint8_t_array(buf, 12, data, 220);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SECURE_COMMAND_LEN);
#else
    mavlink_secure_command_t packet;
    packet.sequence = sequence;
    packet.operation = operation;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.data_length = data_length;
    packet.sig_length = sig_length;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*220);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SECURE_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SECURE_COMMAND;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SECURE_COMMAND_MIN_LEN, MAVLINK_MSG_ID_SECURE_COMMAND_LEN, MAVLINK_MSG_ID_SECURE_COMMAND_CRC);
}

/**
 * @brief Pack a secure_command message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param sequence  Sequence ID for tagging reply.
 * @param operation  Operation being requested.
 * @param data_length  Data length.
 * @param sig_length  Signature length.
 * @param data  Signed data.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_secure_command_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint32_t sequence, uint32_t operation, uint8_t data_length, uint8_t sig_length, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SECURE_COMMAND_LEN];
    _mav_put_uint32_t(buf, 0, sequence);
    _mav_put_uint32_t(buf, 4, operation);
    _mav_put_uint8_t(buf, 8, target_system);
    _mav_put_uint8_t(buf, 9, target_component);
    _mav_put_uint8_t(buf, 10, data_length);
    _mav_put_uint8_t(buf, 11, sig_length);
    _mav_put_uint8_t_array(buf, 12, data, 220);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SECURE_COMMAND_LEN);
#else
    mavlink_secure_command_t packet;
    packet.sequence = sequence;
    packet.operation = operation;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.data_length = data_length;
    packet.sig_length = sig_length;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*220);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SECURE_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SECURE_COMMAND;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SECURE_COMMAND_MIN_LEN, MAVLINK_MSG_ID_SECURE_COMMAND_LEN, MAVLINK_MSG_ID_SECURE_COMMAND_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SECURE_COMMAND_MIN_LEN, MAVLINK_MSG_ID_SECURE_COMMAND_LEN);
#endif
}

/**
 * @brief Pack a secure_command message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param sequence  Sequence ID for tagging reply.
 * @param operation  Operation being requested.
 * @param data_length  Data length.
 * @param sig_length  Signature length.
 * @param data  Signed data.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_secure_command_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint32_t sequence,uint32_t operation,uint8_t data_length,uint8_t sig_length,const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SECURE_COMMAND_LEN];
    _mav_put_uint32_t(buf, 0, sequence);
    _mav_put_uint32_t(buf, 4, operation);
    _mav_put_uint8_t(buf, 8, target_system);
    _mav_put_uint8_t(buf, 9, target_component);
    _mav_put_uint8_t(buf, 10, data_length);
    _mav_put_uint8_t(buf, 11, sig_length);
    _mav_put_uint8_t_array(buf, 12, data, 220);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SECURE_COMMAND_LEN);
#else
    mavlink_secure_command_t packet;
    packet.sequence = sequence;
    packet.operation = operation;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.data_length = data_length;
    packet.sig_length = sig_length;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*220);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SECURE_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SECURE_COMMAND;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SECURE_COMMAND_MIN_LEN, MAVLINK_MSG_ID_SECURE_COMMAND_LEN, MAVLINK_MSG_ID_SECURE_COMMAND_CRC);
}

/**
 * @brief Encode a secure_command struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param secure_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_secure_command_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_secure_command_t* secure_command)
{
    return mavlink_msg_secure_command_pack(system_id, component_id, msg, secure_command->target_system, secure_command->target_component, secure_command->sequence, secure_command->operation, secure_command->data_length, secure_command->sig_length, secure_command->data);
}

/**
 * @brief Encode a secure_command struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param secure_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_secure_command_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_secure_command_t* secure_command)
{
    return mavlink_msg_secure_command_pack_chan(system_id, component_id, chan, msg, secure_command->target_system, secure_command->target_component, secure_command->sequence, secure_command->operation, secure_command->data_length, secure_command->sig_length, secure_command->data);
}

/**
 * @brief Encode a secure_command struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param secure_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_secure_command_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_secure_command_t* secure_command)
{
    return mavlink_msg_secure_command_pack_status(system_id, component_id, _status, msg,  secure_command->target_system, secure_command->target_component, secure_command->sequence, secure_command->operation, secure_command->data_length, secure_command->sig_length, secure_command->data);
}

/**
 * @brief Send a secure_command message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param sequence  Sequence ID for tagging reply.
 * @param operation  Operation being requested.
 * @param data_length  Data length.
 * @param sig_length  Signature length.
 * @param data  Signed data.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_secure_command_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint32_t sequence, uint32_t operation, uint8_t data_length, uint8_t sig_length, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SECURE_COMMAND_LEN];
    _mav_put_uint32_t(buf, 0, sequence);
    _mav_put_uint32_t(buf, 4, operation);
    _mav_put_uint8_t(buf, 8, target_system);
    _mav_put_uint8_t(buf, 9, target_component);
    _mav_put_uint8_t(buf, 10, data_length);
    _mav_put_uint8_t(buf, 11, sig_length);
    _mav_put_uint8_t_array(buf, 12, data, 220);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SECURE_COMMAND, buf, MAVLINK_MSG_ID_SECURE_COMMAND_MIN_LEN, MAVLINK_MSG_ID_SECURE_COMMAND_LEN, MAVLINK_MSG_ID_SECURE_COMMAND_CRC);
#else
    mavlink_secure_command_t packet;
    packet.sequence = sequence;
    packet.operation = operation;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.data_length = data_length;
    packet.sig_length = sig_length;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*220);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SECURE_COMMAND, (const char *)&packet, MAVLINK_MSG_ID_SECURE_COMMAND_MIN_LEN, MAVLINK_MSG_ID_SECURE_COMMAND_LEN, MAVLINK_MSG_ID_SECURE_COMMAND_CRC);
#endif
}

/**
 * @brief Send a secure_command message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_secure_command_send_struct(mavlink_channel_t chan, const mavlink_secure_command_t* secure_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_secure_command_send(chan, secure_command->target_system, secure_command->target_component, secure_command->sequence, secure_command->operation, secure_command->data_length, secure_command->sig_length, secure_command->data);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SECURE_COMMAND, (const char *)secure_command, MAVLINK_MSG_ID_SECURE_COMMAND_MIN_LEN, MAVLINK_MSG_ID_SECURE_COMMAND_LEN, MAVLINK_MSG_ID_SECURE_COMMAND_CRC);
#endif
}

#if MAVLINK_MSG_ID_SECURE_COMMAND_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_secure_command_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint32_t sequence, uint32_t operation, uint8_t data_length, uint8_t sig_length, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, sequence);
    _mav_put_uint32_t(buf, 4, operation);
    _mav_put_uint8_t(buf, 8, target_system);
    _mav_put_uint8_t(buf, 9, target_component);
    _mav_put_uint8_t(buf, 10, data_length);
    _mav_put_uint8_t(buf, 11, sig_length);
    _mav_put_uint8_t_array(buf, 12, data, 220);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SECURE_COMMAND, buf, MAVLINK_MSG_ID_SECURE_COMMAND_MIN_LEN, MAVLINK_MSG_ID_SECURE_COMMAND_LEN, MAVLINK_MSG_ID_SECURE_COMMAND_CRC);
#else
    mavlink_secure_command_t *packet = (mavlink_secure_command_t *)msgbuf;
    packet->sequence = sequence;
    packet->operation = operation;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->data_length = data_length;
    packet->sig_length = sig_length;
    mav_array_memcpy(packet->data, data, sizeof(uint8_t)*220);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SECURE_COMMAND, (const char *)packet, MAVLINK_MSG_ID_SECURE_COMMAND_MIN_LEN, MAVLINK_MSG_ID_SECURE_COMMAND_LEN, MAVLINK_MSG_ID_SECURE_COMMAND_CRC);
#endif
}
#endif

#endif

// MESSAGE SECURE_COMMAND UNPACKING


/**
 * @brief Get field target_system from secure_command message
 *
 * @return  System ID.
 */
static inline uint8_t mavlink_msg_secure_command_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field target_component from secure_command message
 *
 * @return  Component ID.
 */
static inline uint8_t mavlink_msg_secure_command_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field sequence from secure_command message
 *
 * @return  Sequence ID for tagging reply.
 */
static inline uint32_t mavlink_msg_secure_command_get_sequence(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field operation from secure_command message
 *
 * @return  Operation being requested.
 */
static inline uint32_t mavlink_msg_secure_command_get_operation(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field data_length from secure_command message
 *
 * @return  Data length.
 */
static inline uint8_t mavlink_msg_secure_command_get_data_length(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field sig_length from secure_command message
 *
 * @return  Signature length.
 */
static inline uint8_t mavlink_msg_secure_command_get_sig_length(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field data from secure_command message
 *
 * @return  Signed data.
 */
static inline uint16_t mavlink_msg_secure_command_get_data(const mavlink_message_t* msg, uint8_t *data)
{
    return _MAV_RETURN_uint8_t_array(msg, data, 220,  12);
}

/**
 * @brief Decode a secure_command message into a struct
 *
 * @param msg The message to decode
 * @param secure_command C-struct to decode the message contents into
 */
static inline void mavlink_msg_secure_command_decode(const mavlink_message_t* msg, mavlink_secure_command_t* secure_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    secure_command->sequence = mavlink_msg_secure_command_get_sequence(msg);
    secure_command->operation = mavlink_msg_secure_command_get_operation(msg);
    secure_command->target_system = mavlink_msg_secure_command_get_target_system(msg);
    secure_command->target_component = mavlink_msg_secure_command_get_target_component(msg);
    secure_command->data_length = mavlink_msg_secure_command_get_data_length(msg);
    secure_command->sig_length = mavlink_msg_secure_command_get_sig_length(msg);
    mavlink_msg_secure_command_get_data(msg, secure_command->data);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SECURE_COMMAND_LEN? msg->len : MAVLINK_MSG_ID_SECURE_COMMAND_LEN;
        memset(secure_command, 0, MAVLINK_MSG_ID_SECURE_COMMAND_LEN);
    memcpy(secure_command, _MAV_PAYLOAD(msg), len);
#endif
}
