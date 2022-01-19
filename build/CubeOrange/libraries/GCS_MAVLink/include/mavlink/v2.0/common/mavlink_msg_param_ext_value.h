#pragma once
// MESSAGE PARAM_EXT_VALUE PACKING

#define MAVLINK_MSG_ID_PARAM_EXT_VALUE 322


typedef struct __mavlink_param_ext_value_t {
 uint16_t param_count; /*<  Total number of parameters*/
 uint16_t param_index; /*<  Index of this parameter*/
 char param_id[16]; /*<  Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string*/
 char param_value[128]; /*<  Parameter value*/
 uint8_t param_type; /*<  Parameter type.*/
} mavlink_param_ext_value_t;

#define MAVLINK_MSG_ID_PARAM_EXT_VALUE_LEN 149
#define MAVLINK_MSG_ID_PARAM_EXT_VALUE_MIN_LEN 149
#define MAVLINK_MSG_ID_322_LEN 149
#define MAVLINK_MSG_ID_322_MIN_LEN 149

#define MAVLINK_MSG_ID_PARAM_EXT_VALUE_CRC 243
#define MAVLINK_MSG_ID_322_CRC 243

#define MAVLINK_MSG_PARAM_EXT_VALUE_FIELD_PARAM_ID_LEN 16
#define MAVLINK_MSG_PARAM_EXT_VALUE_FIELD_PARAM_VALUE_LEN 128

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PARAM_EXT_VALUE { \
    322, \
    "PARAM_EXT_VALUE", \
    5, \
    {  { "param_id", NULL, MAVLINK_TYPE_CHAR, 16, 4, offsetof(mavlink_param_ext_value_t, param_id) }, \
         { "param_value", NULL, MAVLINK_TYPE_CHAR, 128, 20, offsetof(mavlink_param_ext_value_t, param_value) }, \
         { "param_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 148, offsetof(mavlink_param_ext_value_t, param_type) }, \
         { "param_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_param_ext_value_t, param_count) }, \
         { "param_index", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_param_ext_value_t, param_index) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PARAM_EXT_VALUE { \
    "PARAM_EXT_VALUE", \
    5, \
    {  { "param_id", NULL, MAVLINK_TYPE_CHAR, 16, 4, offsetof(mavlink_param_ext_value_t, param_id) }, \
         { "param_value", NULL, MAVLINK_TYPE_CHAR, 128, 20, offsetof(mavlink_param_ext_value_t, param_value) }, \
         { "param_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 148, offsetof(mavlink_param_ext_value_t, param_type) }, \
         { "param_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_param_ext_value_t, param_count) }, \
         { "param_index", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_param_ext_value_t, param_index) }, \
         } \
}
#endif

/**
 * @brief Pack a param_ext_value message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param param_id  Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 * @param param_value  Parameter value
 * @param param_type  Parameter type.
 * @param param_count  Total number of parameters
 * @param param_index  Index of this parameter
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_param_ext_value_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const char *param_id, const char *param_value, uint8_t param_type, uint16_t param_count, uint16_t param_index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PARAM_EXT_VALUE_LEN];
    _mav_put_uint16_t(buf, 0, param_count);
    _mav_put_uint16_t(buf, 2, param_index);
    _mav_put_uint8_t(buf, 148, param_type);
    _mav_put_char_array(buf, 4, param_id, 16);
    _mav_put_char_array(buf, 20, param_value, 128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PARAM_EXT_VALUE_LEN);
#else
    mavlink_param_ext_value_t packet;
    packet.param_count = param_count;
    packet.param_index = param_index;
    packet.param_type = param_type;
    mav_array_memcpy(packet.param_id, param_id, sizeof(char)*16);
    mav_array_memcpy(packet.param_value, param_value, sizeof(char)*128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PARAM_EXT_VALUE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PARAM_EXT_VALUE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PARAM_EXT_VALUE_MIN_LEN, MAVLINK_MSG_ID_PARAM_EXT_VALUE_LEN, MAVLINK_MSG_ID_PARAM_EXT_VALUE_CRC);
}

/**
 * @brief Pack a param_ext_value message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param param_id  Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 * @param param_value  Parameter value
 * @param param_type  Parameter type.
 * @param param_count  Total number of parameters
 * @param param_index  Index of this parameter
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_param_ext_value_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const char *param_id,const char *param_value,uint8_t param_type,uint16_t param_count,uint16_t param_index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PARAM_EXT_VALUE_LEN];
    _mav_put_uint16_t(buf, 0, param_count);
    _mav_put_uint16_t(buf, 2, param_index);
    _mav_put_uint8_t(buf, 148, param_type);
    _mav_put_char_array(buf, 4, param_id, 16);
    _mav_put_char_array(buf, 20, param_value, 128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PARAM_EXT_VALUE_LEN);
#else
    mavlink_param_ext_value_t packet;
    packet.param_count = param_count;
    packet.param_index = param_index;
    packet.param_type = param_type;
    mav_array_memcpy(packet.param_id, param_id, sizeof(char)*16);
    mav_array_memcpy(packet.param_value, param_value, sizeof(char)*128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PARAM_EXT_VALUE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PARAM_EXT_VALUE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PARAM_EXT_VALUE_MIN_LEN, MAVLINK_MSG_ID_PARAM_EXT_VALUE_LEN, MAVLINK_MSG_ID_PARAM_EXT_VALUE_CRC);
}

/**
 * @brief Encode a param_ext_value struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param param_ext_value C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_param_ext_value_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_param_ext_value_t* param_ext_value)
{
    return mavlink_msg_param_ext_value_pack(system_id, component_id, msg, param_ext_value->param_id, param_ext_value->param_value, param_ext_value->param_type, param_ext_value->param_count, param_ext_value->param_index);
}

/**
 * @brief Encode a param_ext_value struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param param_ext_value C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_param_ext_value_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_param_ext_value_t* param_ext_value)
{
    return mavlink_msg_param_ext_value_pack_chan(system_id, component_id, chan, msg, param_ext_value->param_id, param_ext_value->param_value, param_ext_value->param_type, param_ext_value->param_count, param_ext_value->param_index);
}

/**
 * @brief Send a param_ext_value message
 * @param chan MAVLink channel to send the message
 *
 * @param param_id  Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 * @param param_value  Parameter value
 * @param param_type  Parameter type.
 * @param param_count  Total number of parameters
 * @param param_index  Index of this parameter
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_param_ext_value_send(mavlink_channel_t chan, const char *param_id, const char *param_value, uint8_t param_type, uint16_t param_count, uint16_t param_index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PARAM_EXT_VALUE_LEN];
    _mav_put_uint16_t(buf, 0, param_count);
    _mav_put_uint16_t(buf, 2, param_index);
    _mav_put_uint8_t(buf, 148, param_type);
    _mav_put_char_array(buf, 4, param_id, 16);
    _mav_put_char_array(buf, 20, param_value, 128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_EXT_VALUE, buf, MAVLINK_MSG_ID_PARAM_EXT_VALUE_MIN_LEN, MAVLINK_MSG_ID_PARAM_EXT_VALUE_LEN, MAVLINK_MSG_ID_PARAM_EXT_VALUE_CRC);
#else
    mavlink_param_ext_value_t packet;
    packet.param_count = param_count;
    packet.param_index = param_index;
    packet.param_type = param_type;
    mav_array_memcpy(packet.param_id, param_id, sizeof(char)*16);
    mav_array_memcpy(packet.param_value, param_value, sizeof(char)*128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_EXT_VALUE, (const char *)&packet, MAVLINK_MSG_ID_PARAM_EXT_VALUE_MIN_LEN, MAVLINK_MSG_ID_PARAM_EXT_VALUE_LEN, MAVLINK_MSG_ID_PARAM_EXT_VALUE_CRC);
#endif
}

/**
 * @brief Send a param_ext_value message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_param_ext_value_send_struct(mavlink_channel_t chan, const mavlink_param_ext_value_t* param_ext_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_param_ext_value_send(chan, param_ext_value->param_id, param_ext_value->param_value, param_ext_value->param_type, param_ext_value->param_count, param_ext_value->param_index);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_EXT_VALUE, (const char *)param_ext_value, MAVLINK_MSG_ID_PARAM_EXT_VALUE_MIN_LEN, MAVLINK_MSG_ID_PARAM_EXT_VALUE_LEN, MAVLINK_MSG_ID_PARAM_EXT_VALUE_CRC);
#endif
}

#if MAVLINK_MSG_ID_PARAM_EXT_VALUE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_param_ext_value_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const char *param_id, const char *param_value, uint8_t param_type, uint16_t param_count, uint16_t param_index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, param_count);
    _mav_put_uint16_t(buf, 2, param_index);
    _mav_put_uint8_t(buf, 148, param_type);
    _mav_put_char_array(buf, 4, param_id, 16);
    _mav_put_char_array(buf, 20, param_value, 128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_EXT_VALUE, buf, MAVLINK_MSG_ID_PARAM_EXT_VALUE_MIN_LEN, MAVLINK_MSG_ID_PARAM_EXT_VALUE_LEN, MAVLINK_MSG_ID_PARAM_EXT_VALUE_CRC);
#else
    mavlink_param_ext_value_t *packet = (mavlink_param_ext_value_t *)msgbuf;
    packet->param_count = param_count;
    packet->param_index = param_index;
    packet->param_type = param_type;
    mav_array_memcpy(packet->param_id, param_id, sizeof(char)*16);
    mav_array_memcpy(packet->param_value, param_value, sizeof(char)*128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_EXT_VALUE, (const char *)packet, MAVLINK_MSG_ID_PARAM_EXT_VALUE_MIN_LEN, MAVLINK_MSG_ID_PARAM_EXT_VALUE_LEN, MAVLINK_MSG_ID_PARAM_EXT_VALUE_CRC);
#endif
}
#endif

#endif

// MESSAGE PARAM_EXT_VALUE UNPACKING


/**
 * @brief Get field param_id from param_ext_value message
 *
 * @return  Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 */
static inline uint16_t mavlink_msg_param_ext_value_get_param_id(const mavlink_message_t* msg, char *param_id)
{
    return _MAV_RETURN_char_array(msg, param_id, 16,  4);
}

/**
 * @brief Get field param_value from param_ext_value message
 *
 * @return  Parameter value
 */
static inline uint16_t mavlink_msg_param_ext_value_get_param_value(const mavlink_message_t* msg, char *param_value)
{
    return _MAV_RETURN_char_array(msg, param_value, 128,  20);
}

/**
 * @brief Get field param_type from param_ext_value message
 *
 * @return  Parameter type.
 */
static inline uint8_t mavlink_msg_param_ext_value_get_param_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  148);
}

/**
 * @brief Get field param_count from param_ext_value message
 *
 * @return  Total number of parameters
 */
static inline uint16_t mavlink_msg_param_ext_value_get_param_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field param_index from param_ext_value message
 *
 * @return  Index of this parameter
 */
static inline uint16_t mavlink_msg_param_ext_value_get_param_index(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Decode a param_ext_value message into a struct
 *
 * @param msg The message to decode
 * @param param_ext_value C-struct to decode the message contents into
 */
static inline void mavlink_msg_param_ext_value_decode(const mavlink_message_t* msg, mavlink_param_ext_value_t* param_ext_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    param_ext_value->param_count = mavlink_msg_param_ext_value_get_param_count(msg);
    param_ext_value->param_index = mavlink_msg_param_ext_value_get_param_index(msg);
    mavlink_msg_param_ext_value_get_param_id(msg, param_ext_value->param_id);
    mavlink_msg_param_ext_value_get_param_value(msg, param_ext_value->param_value);
    param_ext_value->param_type = mavlink_msg_param_ext_value_get_param_type(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PARAM_EXT_VALUE_LEN? msg->len : MAVLINK_MSG_ID_PARAM_EXT_VALUE_LEN;
        memset(param_ext_value, 0, MAVLINK_MSG_ID_PARAM_EXT_VALUE_LEN);
    memcpy(param_ext_value, _MAV_PAYLOAD(msg), len);
#endif
}
