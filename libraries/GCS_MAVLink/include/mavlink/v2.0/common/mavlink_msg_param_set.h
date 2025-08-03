#pragma once
// MESSAGE PARAM_SET PACKING

#define MAVLINK_MSG_ID_PARAM_SET 23


typedef struct __mavlink_param_set_t {
 float param_value; /*<  Onboard parameter value*/
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
 char param_id[16]; /*<  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string*/
 uint8_t param_type; /*<  Onboard parameter type.*/
} mavlink_param_set_t;

#define MAVLINK_MSG_ID_PARAM_SET_LEN 23
#define MAVLINK_MSG_ID_PARAM_SET_MIN_LEN 23
#define MAVLINK_MSG_ID_23_LEN 23
#define MAVLINK_MSG_ID_23_MIN_LEN 23

#define MAVLINK_MSG_ID_PARAM_SET_CRC 168
#define MAVLINK_MSG_ID_23_CRC 168

#define MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN 16

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PARAM_SET { \
    23, \
    "PARAM_SET", \
    5, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_param_set_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_param_set_t, target_component) }, \
         { "param_id", NULL, MAVLINK_TYPE_CHAR, 16, 6, offsetof(mavlink_param_set_t, param_id) }, \
         { "param_value", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_param_set_t, param_value) }, \
         { "param_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_param_set_t, param_type) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PARAM_SET { \
    "PARAM_SET", \
    5, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_param_set_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_param_set_t, target_component) }, \
         { "param_id", NULL, MAVLINK_TYPE_CHAR, 16, 6, offsetof(mavlink_param_set_t, param_id) }, \
         { "param_value", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_param_set_t, param_value) }, \
         { "param_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_param_set_t, param_type) }, \
         } \
}
#endif

/**
 * @brief Pack a param_set message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param param_id  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 * @param param_value  Onboard parameter value
 * @param param_type  Onboard parameter type.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_param_set_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, const char *param_id, float param_value, uint8_t param_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PARAM_SET_LEN];
    _mav_put_float(buf, 0, param_value);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 22, param_type);
    _mav_put_char_array(buf, 6, param_id, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PARAM_SET_LEN);
#else
    mavlink_param_set_t packet;
    packet.param_value = param_value;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.param_type = param_type;
    mav_array_memcpy(packet.param_id, param_id, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PARAM_SET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PARAM_SET;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PARAM_SET_MIN_LEN, MAVLINK_MSG_ID_PARAM_SET_LEN, MAVLINK_MSG_ID_PARAM_SET_CRC);
}

/**
 * @brief Pack a param_set message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param param_id  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 * @param param_value  Onboard parameter value
 * @param param_type  Onboard parameter type.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_param_set_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, const char *param_id, float param_value, uint8_t param_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PARAM_SET_LEN];
    _mav_put_float(buf, 0, param_value);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 22, param_type);
    _mav_put_char_array(buf, 6, param_id, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PARAM_SET_LEN);
#else
    mavlink_param_set_t packet;
    packet.param_value = param_value;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.param_type = param_type;
    mav_array_memcpy(packet.param_id, param_id, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PARAM_SET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PARAM_SET;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_PARAM_SET_MIN_LEN, MAVLINK_MSG_ID_PARAM_SET_LEN, MAVLINK_MSG_ID_PARAM_SET_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_PARAM_SET_MIN_LEN, MAVLINK_MSG_ID_PARAM_SET_LEN);
#endif
}

/**
 * @brief Pack a param_set message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param param_id  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 * @param param_value  Onboard parameter value
 * @param param_type  Onboard parameter type.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_param_set_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,const char *param_id,float param_value,uint8_t param_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PARAM_SET_LEN];
    _mav_put_float(buf, 0, param_value);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 22, param_type);
    _mav_put_char_array(buf, 6, param_id, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PARAM_SET_LEN);
#else
    mavlink_param_set_t packet;
    packet.param_value = param_value;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.param_type = param_type;
    mav_array_memcpy(packet.param_id, param_id, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PARAM_SET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PARAM_SET;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PARAM_SET_MIN_LEN, MAVLINK_MSG_ID_PARAM_SET_LEN, MAVLINK_MSG_ID_PARAM_SET_CRC);
}

/**
 * @brief Encode a param_set struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param param_set C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_param_set_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_param_set_t* param_set)
{
    return mavlink_msg_param_set_pack(system_id, component_id, msg, param_set->target_system, param_set->target_component, param_set->param_id, param_set->param_value, param_set->param_type);
}

/**
 * @brief Encode a param_set struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param param_set C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_param_set_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_param_set_t* param_set)
{
    return mavlink_msg_param_set_pack_chan(system_id, component_id, chan, msg, param_set->target_system, param_set->target_component, param_set->param_id, param_set->param_value, param_set->param_type);
}

/**
 * @brief Encode a param_set struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param param_set C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_param_set_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_param_set_t* param_set)
{
    return mavlink_msg_param_set_pack_status(system_id, component_id, _status, msg,  param_set->target_system, param_set->target_component, param_set->param_id, param_set->param_value, param_set->param_type);
}

/**
 * @brief Send a param_set message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param param_id  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 * @param param_value  Onboard parameter value
 * @param param_type  Onboard parameter type.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_param_set_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, const char *param_id, float param_value, uint8_t param_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PARAM_SET_LEN];
    _mav_put_float(buf, 0, param_value);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 22, param_type);
    _mav_put_char_array(buf, 6, param_id, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_SET, buf, MAVLINK_MSG_ID_PARAM_SET_MIN_LEN, MAVLINK_MSG_ID_PARAM_SET_LEN, MAVLINK_MSG_ID_PARAM_SET_CRC);
#else
    mavlink_param_set_t packet;
    packet.param_value = param_value;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.param_type = param_type;
    mav_array_memcpy(packet.param_id, param_id, sizeof(char)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_SET, (const char *)&packet, MAVLINK_MSG_ID_PARAM_SET_MIN_LEN, MAVLINK_MSG_ID_PARAM_SET_LEN, MAVLINK_MSG_ID_PARAM_SET_CRC);
#endif
}

/**
 * @brief Send a param_set message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_param_set_send_struct(mavlink_channel_t chan, const mavlink_param_set_t* param_set)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_param_set_send(chan, param_set->target_system, param_set->target_component, param_set->param_id, param_set->param_value, param_set->param_type);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_SET, (const char *)param_set, MAVLINK_MSG_ID_PARAM_SET_MIN_LEN, MAVLINK_MSG_ID_PARAM_SET_LEN, MAVLINK_MSG_ID_PARAM_SET_CRC);
#endif
}

#if MAVLINK_MSG_ID_PARAM_SET_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_param_set_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, const char *param_id, float param_value, uint8_t param_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, param_value);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 22, param_type);
    _mav_put_char_array(buf, 6, param_id, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_SET, buf, MAVLINK_MSG_ID_PARAM_SET_MIN_LEN, MAVLINK_MSG_ID_PARAM_SET_LEN, MAVLINK_MSG_ID_PARAM_SET_CRC);
#else
    mavlink_param_set_t *packet = (mavlink_param_set_t *)msgbuf;
    packet->param_value = param_value;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->param_type = param_type;
    mav_array_memcpy(packet->param_id, param_id, sizeof(char)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_SET, (const char *)packet, MAVLINK_MSG_ID_PARAM_SET_MIN_LEN, MAVLINK_MSG_ID_PARAM_SET_LEN, MAVLINK_MSG_ID_PARAM_SET_CRC);
#endif
}
#endif

#endif

// MESSAGE PARAM_SET UNPACKING


/**
 * @brief Get field target_system from param_set message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_param_set_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field target_component from param_set message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_param_set_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field param_id from param_set message
 *
 * @return  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 */
static inline uint16_t mavlink_msg_param_set_get_param_id(const mavlink_message_t* msg, char *param_id)
{
    return _MAV_RETURN_char_array(msg, param_id, 16,  6);
}

/**
 * @brief Get field param_value from param_set message
 *
 * @return  Onboard parameter value
 */
static inline float mavlink_msg_param_set_get_param_value(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field param_type from param_set message
 *
 * @return  Onboard parameter type.
 */
static inline uint8_t mavlink_msg_param_set_get_param_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Decode a param_set message into a struct
 *
 * @param msg The message to decode
 * @param param_set C-struct to decode the message contents into
 */
static inline void mavlink_msg_param_set_decode(const mavlink_message_t* msg, mavlink_param_set_t* param_set)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    param_set->param_value = mavlink_msg_param_set_get_param_value(msg);
    param_set->target_system = mavlink_msg_param_set_get_target_system(msg);
    param_set->target_component = mavlink_msg_param_set_get_target_component(msg);
    mavlink_msg_param_set_get_param_id(msg, param_set->param_id);
    param_set->param_type = mavlink_msg_param_set_get_param_type(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PARAM_SET_LEN? msg->len : MAVLINK_MSG_ID_PARAM_SET_LEN;
        memset(param_set, 0, MAVLINK_MSG_ID_PARAM_SET_LEN);
    memcpy(param_set, _MAV_PAYLOAD(msg), len);
#endif
}
