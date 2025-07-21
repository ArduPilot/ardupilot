#pragma once
// MESSAGE SECURE_COMMAND_REPLY PACKING

#define MAVLINK_MSG_ID_SECURE_COMMAND_REPLY 11005


typedef struct __mavlink_secure_command_reply_t {
 uint32_t sequence; /*<  Sequence ID from request.*/
 uint32_t operation; /*<  Operation that was requested.*/
 uint8_t result; /*<  Result of command.*/
 uint8_t data_length; /*<  Data length.*/
 uint8_t data[220]; /*<  Reply data.*/
} mavlink_secure_command_reply_t;

#define MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_LEN 230
#define MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_MIN_LEN 230
#define MAVLINK_MSG_ID_11005_LEN 230
#define MAVLINK_MSG_ID_11005_MIN_LEN 230

#define MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_CRC 93
#define MAVLINK_MSG_ID_11005_CRC 93

#define MAVLINK_MSG_SECURE_COMMAND_REPLY_FIELD_DATA_LEN 220

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SECURE_COMMAND_REPLY { \
    11005, \
    "SECURE_COMMAND_REPLY", \
    5, \
    {  { "sequence", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_secure_command_reply_t, sequence) }, \
         { "operation", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_secure_command_reply_t, operation) }, \
         { "result", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_secure_command_reply_t, result) }, \
         { "data_length", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_secure_command_reply_t, data_length) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 220, 10, offsetof(mavlink_secure_command_reply_t, data) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SECURE_COMMAND_REPLY { \
    "SECURE_COMMAND_REPLY", \
    5, \
    {  { "sequence", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_secure_command_reply_t, sequence) }, \
         { "operation", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_secure_command_reply_t, operation) }, \
         { "result", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_secure_command_reply_t, result) }, \
         { "data_length", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_secure_command_reply_t, data_length) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 220, 10, offsetof(mavlink_secure_command_reply_t, data) }, \
         } \
}
#endif

/**
 * @brief Pack a secure_command_reply message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param sequence  Sequence ID from request.
 * @param operation  Operation that was requested.
 * @param result  Result of command.
 * @param data_length  Data length.
 * @param data  Reply data.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_secure_command_reply_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t sequence, uint32_t operation, uint8_t result, uint8_t data_length, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_LEN];
    _mav_put_uint32_t(buf, 0, sequence);
    _mav_put_uint32_t(buf, 4, operation);
    _mav_put_uint8_t(buf, 8, result);
    _mav_put_uint8_t(buf, 9, data_length);
    _mav_put_uint8_t_array(buf, 10, data, 220);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_LEN);
#else
    mavlink_secure_command_reply_t packet;
    packet.sequence = sequence;
    packet.operation = operation;
    packet.result = result;
    packet.data_length = data_length;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*220);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SECURE_COMMAND_REPLY;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_MIN_LEN, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_LEN, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_CRC);
}

/**
 * @brief Pack a secure_command_reply message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param sequence  Sequence ID from request.
 * @param operation  Operation that was requested.
 * @param result  Result of command.
 * @param data_length  Data length.
 * @param data  Reply data.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_secure_command_reply_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t sequence, uint32_t operation, uint8_t result, uint8_t data_length, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_LEN];
    _mav_put_uint32_t(buf, 0, sequence);
    _mav_put_uint32_t(buf, 4, operation);
    _mav_put_uint8_t(buf, 8, result);
    _mav_put_uint8_t(buf, 9, data_length);
    _mav_put_uint8_t_array(buf, 10, data, 220);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_LEN);
#else
    mavlink_secure_command_reply_t packet;
    packet.sequence = sequence;
    packet.operation = operation;
    packet.result = result;
    packet.data_length = data_length;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*220);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SECURE_COMMAND_REPLY;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_MIN_LEN, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_LEN, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_MIN_LEN, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_LEN);
#endif
}

/**
 * @brief Pack a secure_command_reply message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sequence  Sequence ID from request.
 * @param operation  Operation that was requested.
 * @param result  Result of command.
 * @param data_length  Data length.
 * @param data  Reply data.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_secure_command_reply_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t sequence,uint32_t operation,uint8_t result,uint8_t data_length,const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_LEN];
    _mav_put_uint32_t(buf, 0, sequence);
    _mav_put_uint32_t(buf, 4, operation);
    _mav_put_uint8_t(buf, 8, result);
    _mav_put_uint8_t(buf, 9, data_length);
    _mav_put_uint8_t_array(buf, 10, data, 220);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_LEN);
#else
    mavlink_secure_command_reply_t packet;
    packet.sequence = sequence;
    packet.operation = operation;
    packet.result = result;
    packet.data_length = data_length;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*220);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SECURE_COMMAND_REPLY;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_MIN_LEN, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_LEN, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_CRC);
}

/**
 * @brief Encode a secure_command_reply struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param secure_command_reply C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_secure_command_reply_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_secure_command_reply_t* secure_command_reply)
{
    return mavlink_msg_secure_command_reply_pack(system_id, component_id, msg, secure_command_reply->sequence, secure_command_reply->operation, secure_command_reply->result, secure_command_reply->data_length, secure_command_reply->data);
}

/**
 * @brief Encode a secure_command_reply struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param secure_command_reply C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_secure_command_reply_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_secure_command_reply_t* secure_command_reply)
{
    return mavlink_msg_secure_command_reply_pack_chan(system_id, component_id, chan, msg, secure_command_reply->sequence, secure_command_reply->operation, secure_command_reply->result, secure_command_reply->data_length, secure_command_reply->data);
}

/**
 * @brief Encode a secure_command_reply struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param secure_command_reply C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_secure_command_reply_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_secure_command_reply_t* secure_command_reply)
{
    return mavlink_msg_secure_command_reply_pack_status(system_id, component_id, _status, msg,  secure_command_reply->sequence, secure_command_reply->operation, secure_command_reply->result, secure_command_reply->data_length, secure_command_reply->data);
}

/**
 * @brief Send a secure_command_reply message
 * @param chan MAVLink channel to send the message
 *
 * @param sequence  Sequence ID from request.
 * @param operation  Operation that was requested.
 * @param result  Result of command.
 * @param data_length  Data length.
 * @param data  Reply data.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_secure_command_reply_send(mavlink_channel_t chan, uint32_t sequence, uint32_t operation, uint8_t result, uint8_t data_length, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_LEN];
    _mav_put_uint32_t(buf, 0, sequence);
    _mav_put_uint32_t(buf, 4, operation);
    _mav_put_uint8_t(buf, 8, result);
    _mav_put_uint8_t(buf, 9, data_length);
    _mav_put_uint8_t_array(buf, 10, data, 220);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY, buf, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_MIN_LEN, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_LEN, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_CRC);
#else
    mavlink_secure_command_reply_t packet;
    packet.sequence = sequence;
    packet.operation = operation;
    packet.result = result;
    packet.data_length = data_length;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*220);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY, (const char *)&packet, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_MIN_LEN, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_LEN, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_CRC);
#endif
}

/**
 * @brief Send a secure_command_reply message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_secure_command_reply_send_struct(mavlink_channel_t chan, const mavlink_secure_command_reply_t* secure_command_reply)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_secure_command_reply_send(chan, secure_command_reply->sequence, secure_command_reply->operation, secure_command_reply->result, secure_command_reply->data_length, secure_command_reply->data);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY, (const char *)secure_command_reply, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_MIN_LEN, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_LEN, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_CRC);
#endif
}

#if MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_secure_command_reply_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t sequence, uint32_t operation, uint8_t result, uint8_t data_length, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, sequence);
    _mav_put_uint32_t(buf, 4, operation);
    _mav_put_uint8_t(buf, 8, result);
    _mav_put_uint8_t(buf, 9, data_length);
    _mav_put_uint8_t_array(buf, 10, data, 220);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY, buf, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_MIN_LEN, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_LEN, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_CRC);
#else
    mavlink_secure_command_reply_t *packet = (mavlink_secure_command_reply_t *)msgbuf;
    packet->sequence = sequence;
    packet->operation = operation;
    packet->result = result;
    packet->data_length = data_length;
    mav_array_memcpy(packet->data, data, sizeof(uint8_t)*220);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY, (const char *)packet, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_MIN_LEN, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_LEN, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_CRC);
#endif
}
#endif

#endif

// MESSAGE SECURE_COMMAND_REPLY UNPACKING


/**
 * @brief Get field sequence from secure_command_reply message
 *
 * @return  Sequence ID from request.
 */
static inline uint32_t mavlink_msg_secure_command_reply_get_sequence(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field operation from secure_command_reply message
 *
 * @return  Operation that was requested.
 */
static inline uint32_t mavlink_msg_secure_command_reply_get_operation(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field result from secure_command_reply message
 *
 * @return  Result of command.
 */
static inline uint8_t mavlink_msg_secure_command_reply_get_result(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field data_length from secure_command_reply message
 *
 * @return  Data length.
 */
static inline uint8_t mavlink_msg_secure_command_reply_get_data_length(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field data from secure_command_reply message
 *
 * @return  Reply data.
 */
static inline uint16_t mavlink_msg_secure_command_reply_get_data(const mavlink_message_t* msg, uint8_t *data)
{
    return _MAV_RETURN_uint8_t_array(msg, data, 220,  10);
}

/**
 * @brief Decode a secure_command_reply message into a struct
 *
 * @param msg The message to decode
 * @param secure_command_reply C-struct to decode the message contents into
 */
static inline void mavlink_msg_secure_command_reply_decode(const mavlink_message_t* msg, mavlink_secure_command_reply_t* secure_command_reply)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    secure_command_reply->sequence = mavlink_msg_secure_command_reply_get_sequence(msg);
    secure_command_reply->operation = mavlink_msg_secure_command_reply_get_operation(msg);
    secure_command_reply->result = mavlink_msg_secure_command_reply_get_result(msg);
    secure_command_reply->data_length = mavlink_msg_secure_command_reply_get_data_length(msg);
    mavlink_msg_secure_command_reply_get_data(msg, secure_command_reply->data);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_LEN? msg->len : MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_LEN;
        memset(secure_command_reply, 0, MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_LEN);
    memcpy(secure_command_reply, _MAV_PAYLOAD(msg), len);
#endif
}
