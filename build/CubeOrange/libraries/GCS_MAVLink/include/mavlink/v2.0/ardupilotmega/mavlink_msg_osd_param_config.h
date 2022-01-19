#pragma once
// MESSAGE OSD_PARAM_CONFIG PACKING

#define MAVLINK_MSG_ID_OSD_PARAM_CONFIG 11033


typedef struct __mavlink_osd_param_config_t {
 uint32_t request_id; /*<  Request ID - copied to reply.*/
 float min_value; /*<  OSD parameter minimum value.*/
 float max_value; /*<  OSD parameter maximum value.*/
 float increment; /*<  OSD parameter increment.*/
 uint8_t target_system; /*<  System ID.*/
 uint8_t target_component; /*<  Component ID.*/
 uint8_t osd_screen; /*<  OSD parameter screen index.*/
 uint8_t osd_index; /*<  OSD parameter display index.*/
 char param_id[16]; /*<  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string*/
 uint8_t config_type; /*<  Config type.*/
} mavlink_osd_param_config_t;

#define MAVLINK_MSG_ID_OSD_PARAM_CONFIG_LEN 37
#define MAVLINK_MSG_ID_OSD_PARAM_CONFIG_MIN_LEN 37
#define MAVLINK_MSG_ID_11033_LEN 37
#define MAVLINK_MSG_ID_11033_MIN_LEN 37

#define MAVLINK_MSG_ID_OSD_PARAM_CONFIG_CRC 195
#define MAVLINK_MSG_ID_11033_CRC 195

#define MAVLINK_MSG_OSD_PARAM_CONFIG_FIELD_PARAM_ID_LEN 16

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_OSD_PARAM_CONFIG { \
    11033, \
    "OSD_PARAM_CONFIG", \
    10, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_osd_param_config_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_osd_param_config_t, target_component) }, \
         { "request_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_osd_param_config_t, request_id) }, \
         { "osd_screen", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_osd_param_config_t, osd_screen) }, \
         { "osd_index", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_osd_param_config_t, osd_index) }, \
         { "param_id", NULL, MAVLINK_TYPE_CHAR, 16, 20, offsetof(mavlink_osd_param_config_t, param_id) }, \
         { "config_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_osd_param_config_t, config_type) }, \
         { "min_value", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_osd_param_config_t, min_value) }, \
         { "max_value", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_osd_param_config_t, max_value) }, \
         { "increment", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_osd_param_config_t, increment) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_OSD_PARAM_CONFIG { \
    "OSD_PARAM_CONFIG", \
    10, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_osd_param_config_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_osd_param_config_t, target_component) }, \
         { "request_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_osd_param_config_t, request_id) }, \
         { "osd_screen", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_osd_param_config_t, osd_screen) }, \
         { "osd_index", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_osd_param_config_t, osd_index) }, \
         { "param_id", NULL, MAVLINK_TYPE_CHAR, 16, 20, offsetof(mavlink_osd_param_config_t, param_id) }, \
         { "config_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_osd_param_config_t, config_type) }, \
         { "min_value", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_osd_param_config_t, min_value) }, \
         { "max_value", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_osd_param_config_t, max_value) }, \
         { "increment", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_osd_param_config_t, increment) }, \
         } \
}
#endif

/**
 * @brief Pack a osd_param_config message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param request_id  Request ID - copied to reply.
 * @param osd_screen  OSD parameter screen index.
 * @param osd_index  OSD parameter display index.
 * @param param_id  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 * @param config_type  Config type.
 * @param min_value  OSD parameter minimum value.
 * @param max_value  OSD parameter maximum value.
 * @param increment  OSD parameter increment.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_osd_param_config_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint32_t request_id, uint8_t osd_screen, uint8_t osd_index, const char *param_id, uint8_t config_type, float min_value, float max_value, float increment)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OSD_PARAM_CONFIG_LEN];
    _mav_put_uint32_t(buf, 0, request_id);
    _mav_put_float(buf, 4, min_value);
    _mav_put_float(buf, 8, max_value);
    _mav_put_float(buf, 12, increment);
    _mav_put_uint8_t(buf, 16, target_system);
    _mav_put_uint8_t(buf, 17, target_component);
    _mav_put_uint8_t(buf, 18, osd_screen);
    _mav_put_uint8_t(buf, 19, osd_index);
    _mav_put_uint8_t(buf, 36, config_type);
    _mav_put_char_array(buf, 20, param_id, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OSD_PARAM_CONFIG_LEN);
#else
    mavlink_osd_param_config_t packet;
    packet.request_id = request_id;
    packet.min_value = min_value;
    packet.max_value = max_value;
    packet.increment = increment;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.osd_screen = osd_screen;
    packet.osd_index = osd_index;
    packet.config_type = config_type;
    mav_array_memcpy(packet.param_id, param_id, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OSD_PARAM_CONFIG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OSD_PARAM_CONFIG;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_OSD_PARAM_CONFIG_MIN_LEN, MAVLINK_MSG_ID_OSD_PARAM_CONFIG_LEN, MAVLINK_MSG_ID_OSD_PARAM_CONFIG_CRC);
}

/**
 * @brief Pack a osd_param_config message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param request_id  Request ID - copied to reply.
 * @param osd_screen  OSD parameter screen index.
 * @param osd_index  OSD parameter display index.
 * @param param_id  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 * @param config_type  Config type.
 * @param min_value  OSD parameter minimum value.
 * @param max_value  OSD parameter maximum value.
 * @param increment  OSD parameter increment.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_osd_param_config_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint32_t request_id,uint8_t osd_screen,uint8_t osd_index,const char *param_id,uint8_t config_type,float min_value,float max_value,float increment)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OSD_PARAM_CONFIG_LEN];
    _mav_put_uint32_t(buf, 0, request_id);
    _mav_put_float(buf, 4, min_value);
    _mav_put_float(buf, 8, max_value);
    _mav_put_float(buf, 12, increment);
    _mav_put_uint8_t(buf, 16, target_system);
    _mav_put_uint8_t(buf, 17, target_component);
    _mav_put_uint8_t(buf, 18, osd_screen);
    _mav_put_uint8_t(buf, 19, osd_index);
    _mav_put_uint8_t(buf, 36, config_type);
    _mav_put_char_array(buf, 20, param_id, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OSD_PARAM_CONFIG_LEN);
#else
    mavlink_osd_param_config_t packet;
    packet.request_id = request_id;
    packet.min_value = min_value;
    packet.max_value = max_value;
    packet.increment = increment;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.osd_screen = osd_screen;
    packet.osd_index = osd_index;
    packet.config_type = config_type;
    mav_array_memcpy(packet.param_id, param_id, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OSD_PARAM_CONFIG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OSD_PARAM_CONFIG;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_OSD_PARAM_CONFIG_MIN_LEN, MAVLINK_MSG_ID_OSD_PARAM_CONFIG_LEN, MAVLINK_MSG_ID_OSD_PARAM_CONFIG_CRC);
}

/**
 * @brief Encode a osd_param_config struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param osd_param_config C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_osd_param_config_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_osd_param_config_t* osd_param_config)
{
    return mavlink_msg_osd_param_config_pack(system_id, component_id, msg, osd_param_config->target_system, osd_param_config->target_component, osd_param_config->request_id, osd_param_config->osd_screen, osd_param_config->osd_index, osd_param_config->param_id, osd_param_config->config_type, osd_param_config->min_value, osd_param_config->max_value, osd_param_config->increment);
}

/**
 * @brief Encode a osd_param_config struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param osd_param_config C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_osd_param_config_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_osd_param_config_t* osd_param_config)
{
    return mavlink_msg_osd_param_config_pack_chan(system_id, component_id, chan, msg, osd_param_config->target_system, osd_param_config->target_component, osd_param_config->request_id, osd_param_config->osd_screen, osd_param_config->osd_index, osd_param_config->param_id, osd_param_config->config_type, osd_param_config->min_value, osd_param_config->max_value, osd_param_config->increment);
}

/**
 * @brief Send a osd_param_config message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param request_id  Request ID - copied to reply.
 * @param osd_screen  OSD parameter screen index.
 * @param osd_index  OSD parameter display index.
 * @param param_id  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 * @param config_type  Config type.
 * @param min_value  OSD parameter minimum value.
 * @param max_value  OSD parameter maximum value.
 * @param increment  OSD parameter increment.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_osd_param_config_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint32_t request_id, uint8_t osd_screen, uint8_t osd_index, const char *param_id, uint8_t config_type, float min_value, float max_value, float increment)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OSD_PARAM_CONFIG_LEN];
    _mav_put_uint32_t(buf, 0, request_id);
    _mav_put_float(buf, 4, min_value);
    _mav_put_float(buf, 8, max_value);
    _mav_put_float(buf, 12, increment);
    _mav_put_uint8_t(buf, 16, target_system);
    _mav_put_uint8_t(buf, 17, target_component);
    _mav_put_uint8_t(buf, 18, osd_screen);
    _mav_put_uint8_t(buf, 19, osd_index);
    _mav_put_uint8_t(buf, 36, config_type);
    _mav_put_char_array(buf, 20, param_id, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OSD_PARAM_CONFIG, buf, MAVLINK_MSG_ID_OSD_PARAM_CONFIG_MIN_LEN, MAVLINK_MSG_ID_OSD_PARAM_CONFIG_LEN, MAVLINK_MSG_ID_OSD_PARAM_CONFIG_CRC);
#else
    mavlink_osd_param_config_t packet;
    packet.request_id = request_id;
    packet.min_value = min_value;
    packet.max_value = max_value;
    packet.increment = increment;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.osd_screen = osd_screen;
    packet.osd_index = osd_index;
    packet.config_type = config_type;
    mav_array_memcpy(packet.param_id, param_id, sizeof(char)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OSD_PARAM_CONFIG, (const char *)&packet, MAVLINK_MSG_ID_OSD_PARAM_CONFIG_MIN_LEN, MAVLINK_MSG_ID_OSD_PARAM_CONFIG_LEN, MAVLINK_MSG_ID_OSD_PARAM_CONFIG_CRC);
#endif
}

/**
 * @brief Send a osd_param_config message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_osd_param_config_send_struct(mavlink_channel_t chan, const mavlink_osd_param_config_t* osd_param_config)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_osd_param_config_send(chan, osd_param_config->target_system, osd_param_config->target_component, osd_param_config->request_id, osd_param_config->osd_screen, osd_param_config->osd_index, osd_param_config->param_id, osd_param_config->config_type, osd_param_config->min_value, osd_param_config->max_value, osd_param_config->increment);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OSD_PARAM_CONFIG, (const char *)osd_param_config, MAVLINK_MSG_ID_OSD_PARAM_CONFIG_MIN_LEN, MAVLINK_MSG_ID_OSD_PARAM_CONFIG_LEN, MAVLINK_MSG_ID_OSD_PARAM_CONFIG_CRC);
#endif
}

#if MAVLINK_MSG_ID_OSD_PARAM_CONFIG_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_osd_param_config_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint32_t request_id, uint8_t osd_screen, uint8_t osd_index, const char *param_id, uint8_t config_type, float min_value, float max_value, float increment)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, request_id);
    _mav_put_float(buf, 4, min_value);
    _mav_put_float(buf, 8, max_value);
    _mav_put_float(buf, 12, increment);
    _mav_put_uint8_t(buf, 16, target_system);
    _mav_put_uint8_t(buf, 17, target_component);
    _mav_put_uint8_t(buf, 18, osd_screen);
    _mav_put_uint8_t(buf, 19, osd_index);
    _mav_put_uint8_t(buf, 36, config_type);
    _mav_put_char_array(buf, 20, param_id, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OSD_PARAM_CONFIG, buf, MAVLINK_MSG_ID_OSD_PARAM_CONFIG_MIN_LEN, MAVLINK_MSG_ID_OSD_PARAM_CONFIG_LEN, MAVLINK_MSG_ID_OSD_PARAM_CONFIG_CRC);
#else
    mavlink_osd_param_config_t *packet = (mavlink_osd_param_config_t *)msgbuf;
    packet->request_id = request_id;
    packet->min_value = min_value;
    packet->max_value = max_value;
    packet->increment = increment;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->osd_screen = osd_screen;
    packet->osd_index = osd_index;
    packet->config_type = config_type;
    mav_array_memcpy(packet->param_id, param_id, sizeof(char)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OSD_PARAM_CONFIG, (const char *)packet, MAVLINK_MSG_ID_OSD_PARAM_CONFIG_MIN_LEN, MAVLINK_MSG_ID_OSD_PARAM_CONFIG_LEN, MAVLINK_MSG_ID_OSD_PARAM_CONFIG_CRC);
#endif
}
#endif

#endif

// MESSAGE OSD_PARAM_CONFIG UNPACKING


/**
 * @brief Get field target_system from osd_param_config message
 *
 * @return  System ID.
 */
static inline uint8_t mavlink_msg_osd_param_config_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field target_component from osd_param_config message
 *
 * @return  Component ID.
 */
static inline uint8_t mavlink_msg_osd_param_config_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field request_id from osd_param_config message
 *
 * @return  Request ID - copied to reply.
 */
static inline uint32_t mavlink_msg_osd_param_config_get_request_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field osd_screen from osd_param_config message
 *
 * @return  OSD parameter screen index.
 */
static inline uint8_t mavlink_msg_osd_param_config_get_osd_screen(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field osd_index from osd_param_config message
 *
 * @return  OSD parameter display index.
 */
static inline uint8_t mavlink_msg_osd_param_config_get_osd_index(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  19);
}

/**
 * @brief Get field param_id from osd_param_config message
 *
 * @return  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 */
static inline uint16_t mavlink_msg_osd_param_config_get_param_id(const mavlink_message_t* msg, char *param_id)
{
    return _MAV_RETURN_char_array(msg, param_id, 16,  20);
}

/**
 * @brief Get field config_type from osd_param_config message
 *
 * @return  Config type.
 */
static inline uint8_t mavlink_msg_osd_param_config_get_config_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Get field min_value from osd_param_config message
 *
 * @return  OSD parameter minimum value.
 */
static inline float mavlink_msg_osd_param_config_get_min_value(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field max_value from osd_param_config message
 *
 * @return  OSD parameter maximum value.
 */
static inline float mavlink_msg_osd_param_config_get_max_value(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field increment from osd_param_config message
 *
 * @return  OSD parameter increment.
 */
static inline float mavlink_msg_osd_param_config_get_increment(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a osd_param_config message into a struct
 *
 * @param msg The message to decode
 * @param osd_param_config C-struct to decode the message contents into
 */
static inline void mavlink_msg_osd_param_config_decode(const mavlink_message_t* msg, mavlink_osd_param_config_t* osd_param_config)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    osd_param_config->request_id = mavlink_msg_osd_param_config_get_request_id(msg);
    osd_param_config->min_value = mavlink_msg_osd_param_config_get_min_value(msg);
    osd_param_config->max_value = mavlink_msg_osd_param_config_get_max_value(msg);
    osd_param_config->increment = mavlink_msg_osd_param_config_get_increment(msg);
    osd_param_config->target_system = mavlink_msg_osd_param_config_get_target_system(msg);
    osd_param_config->target_component = mavlink_msg_osd_param_config_get_target_component(msg);
    osd_param_config->osd_screen = mavlink_msg_osd_param_config_get_osd_screen(msg);
    osd_param_config->osd_index = mavlink_msg_osd_param_config_get_osd_index(msg);
    mavlink_msg_osd_param_config_get_param_id(msg, osd_param_config->param_id);
    osd_param_config->config_type = mavlink_msg_osd_param_config_get_config_type(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_OSD_PARAM_CONFIG_LEN? msg->len : MAVLINK_MSG_ID_OSD_PARAM_CONFIG_LEN;
        memset(osd_param_config, 0, MAVLINK_MSG_ID_OSD_PARAM_CONFIG_LEN);
    memcpy(osd_param_config, _MAV_PAYLOAD(msg), len);
#endif
}
