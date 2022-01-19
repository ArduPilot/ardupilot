#pragma once
// MESSAGE DEVICE_OP_READ_REPLY PACKING

#define MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY 11001


typedef struct __mavlink_device_op_read_reply_t {
 uint32_t request_id; /*<  Request ID - copied from request.*/
 uint8_t result; /*<  0 for success, anything else is failure code.*/
 uint8_t regstart; /*<  Starting register.*/
 uint8_t count; /*<  Count of bytes read.*/
 uint8_t data[128]; /*<  Reply data.*/
 uint8_t bank; /*<  Bank number.*/
} mavlink_device_op_read_reply_t;

#define MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_LEN 136
#define MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_MIN_LEN 135
#define MAVLINK_MSG_ID_11001_LEN 136
#define MAVLINK_MSG_ID_11001_MIN_LEN 135

#define MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_CRC 15
#define MAVLINK_MSG_ID_11001_CRC 15

#define MAVLINK_MSG_DEVICE_OP_READ_REPLY_FIELD_DATA_LEN 128

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DEVICE_OP_READ_REPLY { \
    11001, \
    "DEVICE_OP_READ_REPLY", \
    6, \
    {  { "request_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_device_op_read_reply_t, request_id) }, \
         { "result", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_device_op_read_reply_t, result) }, \
         { "regstart", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_device_op_read_reply_t, regstart) }, \
         { "count", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_device_op_read_reply_t, count) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 128, 7, offsetof(mavlink_device_op_read_reply_t, data) }, \
         { "bank", NULL, MAVLINK_TYPE_UINT8_T, 0, 135, offsetof(mavlink_device_op_read_reply_t, bank) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DEVICE_OP_READ_REPLY { \
    "DEVICE_OP_READ_REPLY", \
    6, \
    {  { "request_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_device_op_read_reply_t, request_id) }, \
         { "result", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_device_op_read_reply_t, result) }, \
         { "regstart", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_device_op_read_reply_t, regstart) }, \
         { "count", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_device_op_read_reply_t, count) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 128, 7, offsetof(mavlink_device_op_read_reply_t, data) }, \
         { "bank", NULL, MAVLINK_TYPE_UINT8_T, 0, 135, offsetof(mavlink_device_op_read_reply_t, bank) }, \
         } \
}
#endif

/**
 * @brief Pack a device_op_read_reply message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param request_id  Request ID - copied from request.
 * @param result  0 for success, anything else is failure code.
 * @param regstart  Starting register.
 * @param count  Count of bytes read.
 * @param data  Reply data.
 * @param bank  Bank number.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_device_op_read_reply_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t request_id, uint8_t result, uint8_t regstart, uint8_t count, const uint8_t *data, uint8_t bank)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_LEN];
    _mav_put_uint32_t(buf, 0, request_id);
    _mav_put_uint8_t(buf, 4, result);
    _mav_put_uint8_t(buf, 5, regstart);
    _mav_put_uint8_t(buf, 6, count);
    _mav_put_uint8_t(buf, 135, bank);
    _mav_put_uint8_t_array(buf, 7, data, 128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_LEN);
#else
    mavlink_device_op_read_reply_t packet;
    packet.request_id = request_id;
    packet.result = result;
    packet.regstart = regstart;
    packet.count = count;
    packet.bank = bank;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_MIN_LEN, MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_LEN, MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_CRC);
}

/**
 * @brief Pack a device_op_read_reply message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param request_id  Request ID - copied from request.
 * @param result  0 for success, anything else is failure code.
 * @param regstart  Starting register.
 * @param count  Count of bytes read.
 * @param data  Reply data.
 * @param bank  Bank number.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_device_op_read_reply_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t request_id,uint8_t result,uint8_t regstart,uint8_t count,const uint8_t *data,uint8_t bank)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_LEN];
    _mav_put_uint32_t(buf, 0, request_id);
    _mav_put_uint8_t(buf, 4, result);
    _mav_put_uint8_t(buf, 5, regstart);
    _mav_put_uint8_t(buf, 6, count);
    _mav_put_uint8_t(buf, 135, bank);
    _mav_put_uint8_t_array(buf, 7, data, 128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_LEN);
#else
    mavlink_device_op_read_reply_t packet;
    packet.request_id = request_id;
    packet.result = result;
    packet.regstart = regstart;
    packet.count = count;
    packet.bank = bank;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_MIN_LEN, MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_LEN, MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_CRC);
}

/**
 * @brief Encode a device_op_read_reply struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param device_op_read_reply C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_device_op_read_reply_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_device_op_read_reply_t* device_op_read_reply)
{
    return mavlink_msg_device_op_read_reply_pack(system_id, component_id, msg, device_op_read_reply->request_id, device_op_read_reply->result, device_op_read_reply->regstart, device_op_read_reply->count, device_op_read_reply->data, device_op_read_reply->bank);
}

/**
 * @brief Encode a device_op_read_reply struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param device_op_read_reply C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_device_op_read_reply_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_device_op_read_reply_t* device_op_read_reply)
{
    return mavlink_msg_device_op_read_reply_pack_chan(system_id, component_id, chan, msg, device_op_read_reply->request_id, device_op_read_reply->result, device_op_read_reply->regstart, device_op_read_reply->count, device_op_read_reply->data, device_op_read_reply->bank);
}

/**
 * @brief Send a device_op_read_reply message
 * @param chan MAVLink channel to send the message
 *
 * @param request_id  Request ID - copied from request.
 * @param result  0 for success, anything else is failure code.
 * @param regstart  Starting register.
 * @param count  Count of bytes read.
 * @param data  Reply data.
 * @param bank  Bank number.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_device_op_read_reply_send(mavlink_channel_t chan, uint32_t request_id, uint8_t result, uint8_t regstart, uint8_t count, const uint8_t *data, uint8_t bank)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_LEN];
    _mav_put_uint32_t(buf, 0, request_id);
    _mav_put_uint8_t(buf, 4, result);
    _mav_put_uint8_t(buf, 5, regstart);
    _mav_put_uint8_t(buf, 6, count);
    _mav_put_uint8_t(buf, 135, bank);
    _mav_put_uint8_t_array(buf, 7, data, 128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY, buf, MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_MIN_LEN, MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_LEN, MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_CRC);
#else
    mavlink_device_op_read_reply_t packet;
    packet.request_id = request_id;
    packet.result = result;
    packet.regstart = regstart;
    packet.count = count;
    packet.bank = bank;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY, (const char *)&packet, MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_MIN_LEN, MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_LEN, MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_CRC);
#endif
}

/**
 * @brief Send a device_op_read_reply message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_device_op_read_reply_send_struct(mavlink_channel_t chan, const mavlink_device_op_read_reply_t* device_op_read_reply)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_device_op_read_reply_send(chan, device_op_read_reply->request_id, device_op_read_reply->result, device_op_read_reply->regstart, device_op_read_reply->count, device_op_read_reply->data, device_op_read_reply->bank);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY, (const char *)device_op_read_reply, MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_MIN_LEN, MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_LEN, MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_CRC);
#endif
}

#if MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_device_op_read_reply_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t request_id, uint8_t result, uint8_t regstart, uint8_t count, const uint8_t *data, uint8_t bank)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, request_id);
    _mav_put_uint8_t(buf, 4, result);
    _mav_put_uint8_t(buf, 5, regstart);
    _mav_put_uint8_t(buf, 6, count);
    _mav_put_uint8_t(buf, 135, bank);
    _mav_put_uint8_t_array(buf, 7, data, 128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY, buf, MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_MIN_LEN, MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_LEN, MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_CRC);
#else
    mavlink_device_op_read_reply_t *packet = (mavlink_device_op_read_reply_t *)msgbuf;
    packet->request_id = request_id;
    packet->result = result;
    packet->regstart = regstart;
    packet->count = count;
    packet->bank = bank;
    mav_array_memcpy(packet->data, data, sizeof(uint8_t)*128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY, (const char *)packet, MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_MIN_LEN, MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_LEN, MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_CRC);
#endif
}
#endif

#endif

// MESSAGE DEVICE_OP_READ_REPLY UNPACKING


/**
 * @brief Get field request_id from device_op_read_reply message
 *
 * @return  Request ID - copied from request.
 */
static inline uint32_t mavlink_msg_device_op_read_reply_get_request_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field result from device_op_read_reply message
 *
 * @return  0 for success, anything else is failure code.
 */
static inline uint8_t mavlink_msg_device_op_read_reply_get_result(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field regstart from device_op_read_reply message
 *
 * @return  Starting register.
 */
static inline uint8_t mavlink_msg_device_op_read_reply_get_regstart(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field count from device_op_read_reply message
 *
 * @return  Count of bytes read.
 */
static inline uint8_t mavlink_msg_device_op_read_reply_get_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field data from device_op_read_reply message
 *
 * @return  Reply data.
 */
static inline uint16_t mavlink_msg_device_op_read_reply_get_data(const mavlink_message_t* msg, uint8_t *data)
{
    return _MAV_RETURN_uint8_t_array(msg, data, 128,  7);
}

/**
 * @brief Get field bank from device_op_read_reply message
 *
 * @return  Bank number.
 */
static inline uint8_t mavlink_msg_device_op_read_reply_get_bank(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  135);
}

/**
 * @brief Decode a device_op_read_reply message into a struct
 *
 * @param msg The message to decode
 * @param device_op_read_reply C-struct to decode the message contents into
 */
static inline void mavlink_msg_device_op_read_reply_decode(const mavlink_message_t* msg, mavlink_device_op_read_reply_t* device_op_read_reply)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    device_op_read_reply->request_id = mavlink_msg_device_op_read_reply_get_request_id(msg);
    device_op_read_reply->result = mavlink_msg_device_op_read_reply_get_result(msg);
    device_op_read_reply->regstart = mavlink_msg_device_op_read_reply_get_regstart(msg);
    device_op_read_reply->count = mavlink_msg_device_op_read_reply_get_count(msg);
    mavlink_msg_device_op_read_reply_get_data(msg, device_op_read_reply->data);
    device_op_read_reply->bank = mavlink_msg_device_op_read_reply_get_bank(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_LEN? msg->len : MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_LEN;
        memset(device_op_read_reply, 0, MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_LEN);
    memcpy(device_op_read_reply, _MAV_PAYLOAD(msg), len);
#endif
}
