#pragma once
// MESSAGE OSD_PARAM_SHOW_CONFIG_REPLY PACKING

#define MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY 11036


typedef struct __mavlink_osd_param_show_config_reply_t {
 uint32_t request_id; /*<  Request ID - copied from request.*/
 float min_value; /*<  OSD parameter minimum value.*/
 float max_value; /*<  OSD parameter maximum value.*/
 float increment; /*<  OSD parameter increment.*/
 uint8_t result; /*<  Config error type.*/
 char param_id[16]; /*<  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string*/
 uint8_t config_type; /*<  Config type.*/
} mavlink_osd_param_show_config_reply_t;

#define MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_LEN 34
#define MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_MIN_LEN 34
#define MAVLINK_MSG_ID_11036_LEN 34
#define MAVLINK_MSG_ID_11036_MIN_LEN 34

#define MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_CRC 177
#define MAVLINK_MSG_ID_11036_CRC 177

#define MAVLINK_MSG_OSD_PARAM_SHOW_CONFIG_REPLY_FIELD_PARAM_ID_LEN 16

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_OSD_PARAM_SHOW_CONFIG_REPLY { \
    11036, \
    "OSD_PARAM_SHOW_CONFIG_REPLY", \
    7, \
    {  { "request_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_osd_param_show_config_reply_t, request_id) }, \
         { "result", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_osd_param_show_config_reply_t, result) }, \
         { "param_id", NULL, MAVLINK_TYPE_CHAR, 16, 17, offsetof(mavlink_osd_param_show_config_reply_t, param_id) }, \
         { "config_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_osd_param_show_config_reply_t, config_type) }, \
         { "min_value", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_osd_param_show_config_reply_t, min_value) }, \
         { "max_value", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_osd_param_show_config_reply_t, max_value) }, \
         { "increment", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_osd_param_show_config_reply_t, increment) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_OSD_PARAM_SHOW_CONFIG_REPLY { \
    "OSD_PARAM_SHOW_CONFIG_REPLY", \
    7, \
    {  { "request_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_osd_param_show_config_reply_t, request_id) }, \
         { "result", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_osd_param_show_config_reply_t, result) }, \
         { "param_id", NULL, MAVLINK_TYPE_CHAR, 16, 17, offsetof(mavlink_osd_param_show_config_reply_t, param_id) }, \
         { "config_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_osd_param_show_config_reply_t, config_type) }, \
         { "min_value", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_osd_param_show_config_reply_t, min_value) }, \
         { "max_value", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_osd_param_show_config_reply_t, max_value) }, \
         { "increment", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_osd_param_show_config_reply_t, increment) }, \
         } \
}
#endif

/**
 * @brief Pack a osd_param_show_config_reply message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param request_id  Request ID - copied from request.
 * @param result  Config error type.
 * @param param_id  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 * @param config_type  Config type.
 * @param min_value  OSD parameter minimum value.
 * @param max_value  OSD parameter maximum value.
 * @param increment  OSD parameter increment.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_osd_param_show_config_reply_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t request_id, uint8_t result, const char *param_id, uint8_t config_type, float min_value, float max_value, float increment)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_LEN];
    _mav_put_uint32_t(buf, 0, request_id);
    _mav_put_float(buf, 4, min_value);
    _mav_put_float(buf, 8, max_value);
    _mav_put_float(buf, 12, increment);
    _mav_put_uint8_t(buf, 16, result);
    _mav_put_uint8_t(buf, 33, config_type);
    _mav_put_char_array(buf, 17, param_id, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_LEN);
#else
    mavlink_osd_param_show_config_reply_t packet;
    packet.request_id = request_id;
    packet.min_value = min_value;
    packet.max_value = max_value;
    packet.increment = increment;
    packet.result = result;
    packet.config_type = config_type;
    mav_array_memcpy(packet.param_id, param_id, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_MIN_LEN, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_LEN, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_CRC);
}

/**
 * @brief Pack a osd_param_show_config_reply message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param request_id  Request ID - copied from request.
 * @param result  Config error type.
 * @param param_id  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 * @param config_type  Config type.
 * @param min_value  OSD parameter minimum value.
 * @param max_value  OSD parameter maximum value.
 * @param increment  OSD parameter increment.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_osd_param_show_config_reply_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t request_id, uint8_t result, const char *param_id, uint8_t config_type, float min_value, float max_value, float increment)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_LEN];
    _mav_put_uint32_t(buf, 0, request_id);
    _mav_put_float(buf, 4, min_value);
    _mav_put_float(buf, 8, max_value);
    _mav_put_float(buf, 12, increment);
    _mav_put_uint8_t(buf, 16, result);
    _mav_put_uint8_t(buf, 33, config_type);
    _mav_put_char_array(buf, 17, param_id, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_LEN);
#else
    mavlink_osd_param_show_config_reply_t packet;
    packet.request_id = request_id;
    packet.min_value = min_value;
    packet.max_value = max_value;
    packet.increment = increment;
    packet.result = result;
    packet.config_type = config_type;
    mav_array_memcpy(packet.param_id, param_id, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_MIN_LEN, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_LEN, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_MIN_LEN, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_LEN);
#endif
}

/**
 * @brief Pack a osd_param_show_config_reply message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param request_id  Request ID - copied from request.
 * @param result  Config error type.
 * @param param_id  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 * @param config_type  Config type.
 * @param min_value  OSD parameter minimum value.
 * @param max_value  OSD parameter maximum value.
 * @param increment  OSD parameter increment.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_osd_param_show_config_reply_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t request_id,uint8_t result,const char *param_id,uint8_t config_type,float min_value,float max_value,float increment)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_LEN];
    _mav_put_uint32_t(buf, 0, request_id);
    _mav_put_float(buf, 4, min_value);
    _mav_put_float(buf, 8, max_value);
    _mav_put_float(buf, 12, increment);
    _mav_put_uint8_t(buf, 16, result);
    _mav_put_uint8_t(buf, 33, config_type);
    _mav_put_char_array(buf, 17, param_id, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_LEN);
#else
    mavlink_osd_param_show_config_reply_t packet;
    packet.request_id = request_id;
    packet.min_value = min_value;
    packet.max_value = max_value;
    packet.increment = increment;
    packet.result = result;
    packet.config_type = config_type;
    mav_array_memcpy(packet.param_id, param_id, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_MIN_LEN, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_LEN, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_CRC);
}

/**
 * @brief Encode a osd_param_show_config_reply struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param osd_param_show_config_reply C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_osd_param_show_config_reply_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_osd_param_show_config_reply_t* osd_param_show_config_reply)
{
    return mavlink_msg_osd_param_show_config_reply_pack(system_id, component_id, msg, osd_param_show_config_reply->request_id, osd_param_show_config_reply->result, osd_param_show_config_reply->param_id, osd_param_show_config_reply->config_type, osd_param_show_config_reply->min_value, osd_param_show_config_reply->max_value, osd_param_show_config_reply->increment);
}

/**
 * @brief Encode a osd_param_show_config_reply struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param osd_param_show_config_reply C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_osd_param_show_config_reply_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_osd_param_show_config_reply_t* osd_param_show_config_reply)
{
    return mavlink_msg_osd_param_show_config_reply_pack_chan(system_id, component_id, chan, msg, osd_param_show_config_reply->request_id, osd_param_show_config_reply->result, osd_param_show_config_reply->param_id, osd_param_show_config_reply->config_type, osd_param_show_config_reply->min_value, osd_param_show_config_reply->max_value, osd_param_show_config_reply->increment);
}

/**
 * @brief Encode a osd_param_show_config_reply struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param osd_param_show_config_reply C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_osd_param_show_config_reply_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_osd_param_show_config_reply_t* osd_param_show_config_reply)
{
    return mavlink_msg_osd_param_show_config_reply_pack_status(system_id, component_id, _status, msg,  osd_param_show_config_reply->request_id, osd_param_show_config_reply->result, osd_param_show_config_reply->param_id, osd_param_show_config_reply->config_type, osd_param_show_config_reply->min_value, osd_param_show_config_reply->max_value, osd_param_show_config_reply->increment);
}

/**
 * @brief Send a osd_param_show_config_reply message
 * @param chan MAVLink channel to send the message
 *
 * @param request_id  Request ID - copied from request.
 * @param result  Config error type.
 * @param param_id  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 * @param config_type  Config type.
 * @param min_value  OSD parameter minimum value.
 * @param max_value  OSD parameter maximum value.
 * @param increment  OSD parameter increment.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_osd_param_show_config_reply_send(mavlink_channel_t chan, uint32_t request_id, uint8_t result, const char *param_id, uint8_t config_type, float min_value, float max_value, float increment)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_LEN];
    _mav_put_uint32_t(buf, 0, request_id);
    _mav_put_float(buf, 4, min_value);
    _mav_put_float(buf, 8, max_value);
    _mav_put_float(buf, 12, increment);
    _mav_put_uint8_t(buf, 16, result);
    _mav_put_uint8_t(buf, 33, config_type);
    _mav_put_char_array(buf, 17, param_id, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY, buf, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_MIN_LEN, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_LEN, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_CRC);
#else
    mavlink_osd_param_show_config_reply_t packet;
    packet.request_id = request_id;
    packet.min_value = min_value;
    packet.max_value = max_value;
    packet.increment = increment;
    packet.result = result;
    packet.config_type = config_type;
    mav_array_memcpy(packet.param_id, param_id, sizeof(char)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY, (const char *)&packet, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_MIN_LEN, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_LEN, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_CRC);
#endif
}

/**
 * @brief Send a osd_param_show_config_reply message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_osd_param_show_config_reply_send_struct(mavlink_channel_t chan, const mavlink_osd_param_show_config_reply_t* osd_param_show_config_reply)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_osd_param_show_config_reply_send(chan, osd_param_show_config_reply->request_id, osd_param_show_config_reply->result, osd_param_show_config_reply->param_id, osd_param_show_config_reply->config_type, osd_param_show_config_reply->min_value, osd_param_show_config_reply->max_value, osd_param_show_config_reply->increment);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY, (const char *)osd_param_show_config_reply, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_MIN_LEN, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_LEN, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_CRC);
#endif
}

#if MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_osd_param_show_config_reply_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t request_id, uint8_t result, const char *param_id, uint8_t config_type, float min_value, float max_value, float increment)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, request_id);
    _mav_put_float(buf, 4, min_value);
    _mav_put_float(buf, 8, max_value);
    _mav_put_float(buf, 12, increment);
    _mav_put_uint8_t(buf, 16, result);
    _mav_put_uint8_t(buf, 33, config_type);
    _mav_put_char_array(buf, 17, param_id, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY, buf, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_MIN_LEN, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_LEN, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_CRC);
#else
    mavlink_osd_param_show_config_reply_t *packet = (mavlink_osd_param_show_config_reply_t *)msgbuf;
    packet->request_id = request_id;
    packet->min_value = min_value;
    packet->max_value = max_value;
    packet->increment = increment;
    packet->result = result;
    packet->config_type = config_type;
    mav_array_memcpy(packet->param_id, param_id, sizeof(char)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY, (const char *)packet, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_MIN_LEN, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_LEN, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_CRC);
#endif
}
#endif

#endif

// MESSAGE OSD_PARAM_SHOW_CONFIG_REPLY UNPACKING


/**
 * @brief Get field request_id from osd_param_show_config_reply message
 *
 * @return  Request ID - copied from request.
 */
static inline uint32_t mavlink_msg_osd_param_show_config_reply_get_request_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field result from osd_param_show_config_reply message
 *
 * @return  Config error type.
 */
static inline uint8_t mavlink_msg_osd_param_show_config_reply_get_result(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field param_id from osd_param_show_config_reply message
 *
 * @return  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 */
static inline uint16_t mavlink_msg_osd_param_show_config_reply_get_param_id(const mavlink_message_t* msg, char *param_id)
{
    return _MAV_RETURN_char_array(msg, param_id, 16,  17);
}

/**
 * @brief Get field config_type from osd_param_show_config_reply message
 *
 * @return  Config type.
 */
static inline uint8_t mavlink_msg_osd_param_show_config_reply_get_config_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  33);
}

/**
 * @brief Get field min_value from osd_param_show_config_reply message
 *
 * @return  OSD parameter minimum value.
 */
static inline float mavlink_msg_osd_param_show_config_reply_get_min_value(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field max_value from osd_param_show_config_reply message
 *
 * @return  OSD parameter maximum value.
 */
static inline float mavlink_msg_osd_param_show_config_reply_get_max_value(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field increment from osd_param_show_config_reply message
 *
 * @return  OSD parameter increment.
 */
static inline float mavlink_msg_osd_param_show_config_reply_get_increment(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a osd_param_show_config_reply message into a struct
 *
 * @param msg The message to decode
 * @param osd_param_show_config_reply C-struct to decode the message contents into
 */
static inline void mavlink_msg_osd_param_show_config_reply_decode(const mavlink_message_t* msg, mavlink_osd_param_show_config_reply_t* osd_param_show_config_reply)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    osd_param_show_config_reply->request_id = mavlink_msg_osd_param_show_config_reply_get_request_id(msg);
    osd_param_show_config_reply->min_value = mavlink_msg_osd_param_show_config_reply_get_min_value(msg);
    osd_param_show_config_reply->max_value = mavlink_msg_osd_param_show_config_reply_get_max_value(msg);
    osd_param_show_config_reply->increment = mavlink_msg_osd_param_show_config_reply_get_increment(msg);
    osd_param_show_config_reply->result = mavlink_msg_osd_param_show_config_reply_get_result(msg);
    mavlink_msg_osd_param_show_config_reply_get_param_id(msg, osd_param_show_config_reply->param_id);
    osd_param_show_config_reply->config_type = mavlink_msg_osd_param_show_config_reply_get_config_type(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_LEN? msg->len : MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_LEN;
        memset(osd_param_show_config_reply, 0, MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_LEN);
    memcpy(osd_param_show_config_reply, _MAV_PAYLOAD(msg), len);
#endif
}
