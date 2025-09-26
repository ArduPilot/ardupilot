#pragma once
// MESSAGE AVAILABLE_MODES PACKING

#define MAVLINK_MSG_ID_AVAILABLE_MODES 435


typedef struct __mavlink_available_modes_t {
 uint32_t custom_mode; /*<  A bitfield for use for autopilot-specific flags*/
 uint32_t properties; /*<  Mode properties.*/
 uint8_t number_modes; /*<  The total number of available modes for the current vehicle type.*/
 uint8_t mode_index; /*<  The current mode index within number_modes, indexed from 1.*/
 uint8_t standard_mode; /*<  Standard mode.*/
 char mode_name[35]; /*<  Name of custom mode, with null termination character. Should be omitted for standard modes.*/
} mavlink_available_modes_t;

#define MAVLINK_MSG_ID_AVAILABLE_MODES_LEN 46
#define MAVLINK_MSG_ID_AVAILABLE_MODES_MIN_LEN 46
#define MAVLINK_MSG_ID_435_LEN 46
#define MAVLINK_MSG_ID_435_MIN_LEN 46

#define MAVLINK_MSG_ID_AVAILABLE_MODES_CRC 134
#define MAVLINK_MSG_ID_435_CRC 134

#define MAVLINK_MSG_AVAILABLE_MODES_FIELD_MODE_NAME_LEN 35

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AVAILABLE_MODES { \
    435, \
    "AVAILABLE_MODES", \
    6, \
    {  { "number_modes", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_available_modes_t, number_modes) }, \
         { "mode_index", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_available_modes_t, mode_index) }, \
         { "standard_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_available_modes_t, standard_mode) }, \
         { "custom_mode", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_available_modes_t, custom_mode) }, \
         { "properties", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_available_modes_t, properties) }, \
         { "mode_name", NULL, MAVLINK_TYPE_CHAR, 35, 11, offsetof(mavlink_available_modes_t, mode_name) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AVAILABLE_MODES { \
    "AVAILABLE_MODES", \
    6, \
    {  { "number_modes", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_available_modes_t, number_modes) }, \
         { "mode_index", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_available_modes_t, mode_index) }, \
         { "standard_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_available_modes_t, standard_mode) }, \
         { "custom_mode", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_available_modes_t, custom_mode) }, \
         { "properties", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_available_modes_t, properties) }, \
         { "mode_name", NULL, MAVLINK_TYPE_CHAR, 35, 11, offsetof(mavlink_available_modes_t, mode_name) }, \
         } \
}
#endif

/**
 * @brief Pack a available_modes message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param number_modes  The total number of available modes for the current vehicle type.
 * @param mode_index  The current mode index within number_modes, indexed from 1.
 * @param standard_mode  Standard mode.
 * @param custom_mode  A bitfield for use for autopilot-specific flags
 * @param properties  Mode properties.
 * @param mode_name  Name of custom mode, with null termination character. Should be omitted for standard modes.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_available_modes_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t number_modes, uint8_t mode_index, uint8_t standard_mode, uint32_t custom_mode, uint32_t properties, const char *mode_name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AVAILABLE_MODES_LEN];
    _mav_put_uint32_t(buf, 0, custom_mode);
    _mav_put_uint32_t(buf, 4, properties);
    _mav_put_uint8_t(buf, 8, number_modes);
    _mav_put_uint8_t(buf, 9, mode_index);
    _mav_put_uint8_t(buf, 10, standard_mode);
    _mav_put_char_array(buf, 11, mode_name, 35);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AVAILABLE_MODES_LEN);
#else
    mavlink_available_modes_t packet;
    packet.custom_mode = custom_mode;
    packet.properties = properties;
    packet.number_modes = number_modes;
    packet.mode_index = mode_index;
    packet.standard_mode = standard_mode;
    mav_array_memcpy(packet.mode_name, mode_name, sizeof(char)*35);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AVAILABLE_MODES_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AVAILABLE_MODES;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AVAILABLE_MODES_MIN_LEN, MAVLINK_MSG_ID_AVAILABLE_MODES_LEN, MAVLINK_MSG_ID_AVAILABLE_MODES_CRC);
}

/**
 * @brief Pack a available_modes message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param number_modes  The total number of available modes for the current vehicle type.
 * @param mode_index  The current mode index within number_modes, indexed from 1.
 * @param standard_mode  Standard mode.
 * @param custom_mode  A bitfield for use for autopilot-specific flags
 * @param properties  Mode properties.
 * @param mode_name  Name of custom mode, with null termination character. Should be omitted for standard modes.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_available_modes_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t number_modes, uint8_t mode_index, uint8_t standard_mode, uint32_t custom_mode, uint32_t properties, const char *mode_name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AVAILABLE_MODES_LEN];
    _mav_put_uint32_t(buf, 0, custom_mode);
    _mav_put_uint32_t(buf, 4, properties);
    _mav_put_uint8_t(buf, 8, number_modes);
    _mav_put_uint8_t(buf, 9, mode_index);
    _mav_put_uint8_t(buf, 10, standard_mode);
    _mav_put_char_array(buf, 11, mode_name, 35);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AVAILABLE_MODES_LEN);
#else
    mavlink_available_modes_t packet;
    packet.custom_mode = custom_mode;
    packet.properties = properties;
    packet.number_modes = number_modes;
    packet.mode_index = mode_index;
    packet.standard_mode = standard_mode;
    mav_array_memcpy(packet.mode_name, mode_name, sizeof(char)*35);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AVAILABLE_MODES_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AVAILABLE_MODES;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AVAILABLE_MODES_MIN_LEN, MAVLINK_MSG_ID_AVAILABLE_MODES_LEN, MAVLINK_MSG_ID_AVAILABLE_MODES_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AVAILABLE_MODES_MIN_LEN, MAVLINK_MSG_ID_AVAILABLE_MODES_LEN);
#endif
}

/**
 * @brief Pack a available_modes message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param number_modes  The total number of available modes for the current vehicle type.
 * @param mode_index  The current mode index within number_modes, indexed from 1.
 * @param standard_mode  Standard mode.
 * @param custom_mode  A bitfield for use for autopilot-specific flags
 * @param properties  Mode properties.
 * @param mode_name  Name of custom mode, with null termination character. Should be omitted for standard modes.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_available_modes_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t number_modes,uint8_t mode_index,uint8_t standard_mode,uint32_t custom_mode,uint32_t properties,const char *mode_name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AVAILABLE_MODES_LEN];
    _mav_put_uint32_t(buf, 0, custom_mode);
    _mav_put_uint32_t(buf, 4, properties);
    _mav_put_uint8_t(buf, 8, number_modes);
    _mav_put_uint8_t(buf, 9, mode_index);
    _mav_put_uint8_t(buf, 10, standard_mode);
    _mav_put_char_array(buf, 11, mode_name, 35);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AVAILABLE_MODES_LEN);
#else
    mavlink_available_modes_t packet;
    packet.custom_mode = custom_mode;
    packet.properties = properties;
    packet.number_modes = number_modes;
    packet.mode_index = mode_index;
    packet.standard_mode = standard_mode;
    mav_array_memcpy(packet.mode_name, mode_name, sizeof(char)*35);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AVAILABLE_MODES_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AVAILABLE_MODES;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AVAILABLE_MODES_MIN_LEN, MAVLINK_MSG_ID_AVAILABLE_MODES_LEN, MAVLINK_MSG_ID_AVAILABLE_MODES_CRC);
}

/**
 * @brief Encode a available_modes struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param available_modes C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_available_modes_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_available_modes_t* available_modes)
{
    return mavlink_msg_available_modes_pack(system_id, component_id, msg, available_modes->number_modes, available_modes->mode_index, available_modes->standard_mode, available_modes->custom_mode, available_modes->properties, available_modes->mode_name);
}

/**
 * @brief Encode a available_modes struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param available_modes C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_available_modes_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_available_modes_t* available_modes)
{
    return mavlink_msg_available_modes_pack_chan(system_id, component_id, chan, msg, available_modes->number_modes, available_modes->mode_index, available_modes->standard_mode, available_modes->custom_mode, available_modes->properties, available_modes->mode_name);
}

/**
 * @brief Encode a available_modes struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param available_modes C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_available_modes_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_available_modes_t* available_modes)
{
    return mavlink_msg_available_modes_pack_status(system_id, component_id, _status, msg,  available_modes->number_modes, available_modes->mode_index, available_modes->standard_mode, available_modes->custom_mode, available_modes->properties, available_modes->mode_name);
}

/**
 * @brief Send a available_modes message
 * @param chan MAVLink channel to send the message
 *
 * @param number_modes  The total number of available modes for the current vehicle type.
 * @param mode_index  The current mode index within number_modes, indexed from 1.
 * @param standard_mode  Standard mode.
 * @param custom_mode  A bitfield for use for autopilot-specific flags
 * @param properties  Mode properties.
 * @param mode_name  Name of custom mode, with null termination character. Should be omitted for standard modes.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_available_modes_send(mavlink_channel_t chan, uint8_t number_modes, uint8_t mode_index, uint8_t standard_mode, uint32_t custom_mode, uint32_t properties, const char *mode_name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AVAILABLE_MODES_LEN];
    _mav_put_uint32_t(buf, 0, custom_mode);
    _mav_put_uint32_t(buf, 4, properties);
    _mav_put_uint8_t(buf, 8, number_modes);
    _mav_put_uint8_t(buf, 9, mode_index);
    _mav_put_uint8_t(buf, 10, standard_mode);
    _mav_put_char_array(buf, 11, mode_name, 35);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AVAILABLE_MODES, buf, MAVLINK_MSG_ID_AVAILABLE_MODES_MIN_LEN, MAVLINK_MSG_ID_AVAILABLE_MODES_LEN, MAVLINK_MSG_ID_AVAILABLE_MODES_CRC);
#else
    mavlink_available_modes_t packet;
    packet.custom_mode = custom_mode;
    packet.properties = properties;
    packet.number_modes = number_modes;
    packet.mode_index = mode_index;
    packet.standard_mode = standard_mode;
    mav_array_memcpy(packet.mode_name, mode_name, sizeof(char)*35);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AVAILABLE_MODES, (const char *)&packet, MAVLINK_MSG_ID_AVAILABLE_MODES_MIN_LEN, MAVLINK_MSG_ID_AVAILABLE_MODES_LEN, MAVLINK_MSG_ID_AVAILABLE_MODES_CRC);
#endif
}

/**
 * @brief Send a available_modes message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_available_modes_send_struct(mavlink_channel_t chan, const mavlink_available_modes_t* available_modes)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_available_modes_send(chan, available_modes->number_modes, available_modes->mode_index, available_modes->standard_mode, available_modes->custom_mode, available_modes->properties, available_modes->mode_name);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AVAILABLE_MODES, (const char *)available_modes, MAVLINK_MSG_ID_AVAILABLE_MODES_MIN_LEN, MAVLINK_MSG_ID_AVAILABLE_MODES_LEN, MAVLINK_MSG_ID_AVAILABLE_MODES_CRC);
#endif
}

#if MAVLINK_MSG_ID_AVAILABLE_MODES_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_available_modes_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t number_modes, uint8_t mode_index, uint8_t standard_mode, uint32_t custom_mode, uint32_t properties, const char *mode_name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, custom_mode);
    _mav_put_uint32_t(buf, 4, properties);
    _mav_put_uint8_t(buf, 8, number_modes);
    _mav_put_uint8_t(buf, 9, mode_index);
    _mav_put_uint8_t(buf, 10, standard_mode);
    _mav_put_char_array(buf, 11, mode_name, 35);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AVAILABLE_MODES, buf, MAVLINK_MSG_ID_AVAILABLE_MODES_MIN_LEN, MAVLINK_MSG_ID_AVAILABLE_MODES_LEN, MAVLINK_MSG_ID_AVAILABLE_MODES_CRC);
#else
    mavlink_available_modes_t *packet = (mavlink_available_modes_t *)msgbuf;
    packet->custom_mode = custom_mode;
    packet->properties = properties;
    packet->number_modes = number_modes;
    packet->mode_index = mode_index;
    packet->standard_mode = standard_mode;
    mav_array_memcpy(packet->mode_name, mode_name, sizeof(char)*35);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AVAILABLE_MODES, (const char *)packet, MAVLINK_MSG_ID_AVAILABLE_MODES_MIN_LEN, MAVLINK_MSG_ID_AVAILABLE_MODES_LEN, MAVLINK_MSG_ID_AVAILABLE_MODES_CRC);
#endif
}
#endif

#endif

// MESSAGE AVAILABLE_MODES UNPACKING


/**
 * @brief Get field number_modes from available_modes message
 *
 * @return  The total number of available modes for the current vehicle type.
 */
static inline uint8_t mavlink_msg_available_modes_get_number_modes(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field mode_index from available_modes message
 *
 * @return  The current mode index within number_modes, indexed from 1.
 */
static inline uint8_t mavlink_msg_available_modes_get_mode_index(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field standard_mode from available_modes message
 *
 * @return  Standard mode.
 */
static inline uint8_t mavlink_msg_available_modes_get_standard_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field custom_mode from available_modes message
 *
 * @return  A bitfield for use for autopilot-specific flags
 */
static inline uint32_t mavlink_msg_available_modes_get_custom_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field properties from available_modes message
 *
 * @return  Mode properties.
 */
static inline uint32_t mavlink_msg_available_modes_get_properties(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field mode_name from available_modes message
 *
 * @return  Name of custom mode, with null termination character. Should be omitted for standard modes.
 */
static inline uint16_t mavlink_msg_available_modes_get_mode_name(const mavlink_message_t* msg, char *mode_name)
{
    return _MAV_RETURN_char_array(msg, mode_name, 35,  11);
}

/**
 * @brief Decode a available_modes message into a struct
 *
 * @param msg The message to decode
 * @param available_modes C-struct to decode the message contents into
 */
static inline void mavlink_msg_available_modes_decode(const mavlink_message_t* msg, mavlink_available_modes_t* available_modes)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    available_modes->custom_mode = mavlink_msg_available_modes_get_custom_mode(msg);
    available_modes->properties = mavlink_msg_available_modes_get_properties(msg);
    available_modes->number_modes = mavlink_msg_available_modes_get_number_modes(msg);
    available_modes->mode_index = mavlink_msg_available_modes_get_mode_index(msg);
    available_modes->standard_mode = mavlink_msg_available_modes_get_standard_mode(msg);
    mavlink_msg_available_modes_get_mode_name(msg, available_modes->mode_name);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AVAILABLE_MODES_LEN? msg->len : MAVLINK_MSG_ID_AVAILABLE_MODES_LEN;
        memset(available_modes, 0, MAVLINK_MSG_ID_AVAILABLE_MODES_LEN);
    memcpy(available_modes, _MAV_PAYLOAD(msg), len);
#endif
}
