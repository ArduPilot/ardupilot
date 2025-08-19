#pragma once
// MESSAGE OPEN_DRONE_ID_OPERATOR_ID PACKING

#define MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID 12905


typedef struct __mavlink_open_drone_id_operator_id_t {
 uint8_t target_system; /*<  System ID (0 for broadcast).*/
 uint8_t target_component; /*<  Component ID (0 for broadcast).*/
 uint8_t id_or_mac[20]; /*<  Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html. */
 uint8_t operator_id_type; /*<  Indicates the type of the operator_id field.*/
 char operator_id[20]; /*<  Text description or numeric value expressed as ASCII characters. Shall be filled with nulls in the unused portion of the field.*/
} mavlink_open_drone_id_operator_id_t;

#define MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_LEN 43
#define MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_MIN_LEN 43
#define MAVLINK_MSG_ID_12905_LEN 43
#define MAVLINK_MSG_ID_12905_MIN_LEN 43

#define MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_CRC 49
#define MAVLINK_MSG_ID_12905_CRC 49

#define MAVLINK_MSG_OPEN_DRONE_ID_OPERATOR_ID_FIELD_ID_OR_MAC_LEN 20
#define MAVLINK_MSG_OPEN_DRONE_ID_OPERATOR_ID_FIELD_OPERATOR_ID_LEN 20

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_OPEN_DRONE_ID_OPERATOR_ID { \
    12905, \
    "OPEN_DRONE_ID_OPERATOR_ID", \
    5, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_open_drone_id_operator_id_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_open_drone_id_operator_id_t, target_component) }, \
         { "id_or_mac", NULL, MAVLINK_TYPE_UINT8_T, 20, 2, offsetof(mavlink_open_drone_id_operator_id_t, id_or_mac) }, \
         { "operator_id_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_open_drone_id_operator_id_t, operator_id_type) }, \
         { "operator_id", NULL, MAVLINK_TYPE_CHAR, 20, 23, offsetof(mavlink_open_drone_id_operator_id_t, operator_id) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_OPEN_DRONE_ID_OPERATOR_ID { \
    "OPEN_DRONE_ID_OPERATOR_ID", \
    5, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_open_drone_id_operator_id_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_open_drone_id_operator_id_t, target_component) }, \
         { "id_or_mac", NULL, MAVLINK_TYPE_UINT8_T, 20, 2, offsetof(mavlink_open_drone_id_operator_id_t, id_or_mac) }, \
         { "operator_id_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_open_drone_id_operator_id_t, operator_id_type) }, \
         { "operator_id", NULL, MAVLINK_TYPE_CHAR, 20, 23, offsetof(mavlink_open_drone_id_operator_id_t, operator_id) }, \
         } \
}
#endif

/**
 * @brief Pack a open_drone_id_operator_id message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID (0 for broadcast).
 * @param target_component  Component ID (0 for broadcast).
 * @param id_or_mac  Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html. 
 * @param operator_id_type  Indicates the type of the operator_id field.
 * @param operator_id  Text description or numeric value expressed as ASCII characters. Shall be filled with nulls in the unused portion of the field.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_open_drone_id_operator_id_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, const uint8_t *id_or_mac, uint8_t operator_id_type, const char *operator_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 22, operator_id_type);
    _mav_put_uint8_t_array(buf, 2, id_or_mac, 20);
    _mav_put_char_array(buf, 23, operator_id, 20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_LEN);
#else
    mavlink_open_drone_id_operator_id_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.operator_id_type = operator_id_type;
    mav_array_memcpy(packet.id_or_mac, id_or_mac, sizeof(uint8_t)*20);
    mav_array_memcpy(packet.operator_id, operator_id, sizeof(char)*20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_CRC);
}

/**
 * @brief Pack a open_drone_id_operator_id message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID (0 for broadcast).
 * @param target_component  Component ID (0 for broadcast).
 * @param id_or_mac  Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html. 
 * @param operator_id_type  Indicates the type of the operator_id field.
 * @param operator_id  Text description or numeric value expressed as ASCII characters. Shall be filled with nulls in the unused portion of the field.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_open_drone_id_operator_id_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, const uint8_t *id_or_mac, uint8_t operator_id_type, const char *operator_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 22, operator_id_type);
    _mav_put_uint8_t_array(buf, 2, id_or_mac, 20);
    _mav_put_char_array(buf, 23, operator_id, 20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_LEN);
#else
    mavlink_open_drone_id_operator_id_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.operator_id_type = operator_id_type;
    mav_array_memcpy(packet.id_or_mac, id_or_mac, sizeof(uint8_t)*20);
    mav_array_memcpy(packet.operator_id, operator_id, sizeof(char)*20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_LEN);
#endif
}

/**
 * @brief Pack a open_drone_id_operator_id message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID (0 for broadcast).
 * @param target_component  Component ID (0 for broadcast).
 * @param id_or_mac  Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html. 
 * @param operator_id_type  Indicates the type of the operator_id field.
 * @param operator_id  Text description or numeric value expressed as ASCII characters. Shall be filled with nulls in the unused portion of the field.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_open_drone_id_operator_id_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,const uint8_t *id_or_mac,uint8_t operator_id_type,const char *operator_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 22, operator_id_type);
    _mav_put_uint8_t_array(buf, 2, id_or_mac, 20);
    _mav_put_char_array(buf, 23, operator_id, 20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_LEN);
#else
    mavlink_open_drone_id_operator_id_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.operator_id_type = operator_id_type;
    mav_array_memcpy(packet.id_or_mac, id_or_mac, sizeof(uint8_t)*20);
    mav_array_memcpy(packet.operator_id, operator_id, sizeof(char)*20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_CRC);
}

/**
 * @brief Encode a open_drone_id_operator_id struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param open_drone_id_operator_id C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_open_drone_id_operator_id_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_open_drone_id_operator_id_t* open_drone_id_operator_id)
{
    return mavlink_msg_open_drone_id_operator_id_pack(system_id, component_id, msg, open_drone_id_operator_id->target_system, open_drone_id_operator_id->target_component, open_drone_id_operator_id->id_or_mac, open_drone_id_operator_id->operator_id_type, open_drone_id_operator_id->operator_id);
}

/**
 * @brief Encode a open_drone_id_operator_id struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param open_drone_id_operator_id C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_open_drone_id_operator_id_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_open_drone_id_operator_id_t* open_drone_id_operator_id)
{
    return mavlink_msg_open_drone_id_operator_id_pack_chan(system_id, component_id, chan, msg, open_drone_id_operator_id->target_system, open_drone_id_operator_id->target_component, open_drone_id_operator_id->id_or_mac, open_drone_id_operator_id->operator_id_type, open_drone_id_operator_id->operator_id);
}

/**
 * @brief Encode a open_drone_id_operator_id struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param open_drone_id_operator_id C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_open_drone_id_operator_id_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_open_drone_id_operator_id_t* open_drone_id_operator_id)
{
    return mavlink_msg_open_drone_id_operator_id_pack_status(system_id, component_id, _status, msg,  open_drone_id_operator_id->target_system, open_drone_id_operator_id->target_component, open_drone_id_operator_id->id_or_mac, open_drone_id_operator_id->operator_id_type, open_drone_id_operator_id->operator_id);
}

/**
 * @brief Send a open_drone_id_operator_id message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID (0 for broadcast).
 * @param target_component  Component ID (0 for broadcast).
 * @param id_or_mac  Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html. 
 * @param operator_id_type  Indicates the type of the operator_id field.
 * @param operator_id  Text description or numeric value expressed as ASCII characters. Shall be filled with nulls in the unused portion of the field.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_open_drone_id_operator_id_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, const uint8_t *id_or_mac, uint8_t operator_id_type, const char *operator_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 22, operator_id_type);
    _mav_put_uint8_t_array(buf, 2, id_or_mac, 20);
    _mav_put_char_array(buf, 23, operator_id, 20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID, buf, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_CRC);
#else
    mavlink_open_drone_id_operator_id_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.operator_id_type = operator_id_type;
    mav_array_memcpy(packet.id_or_mac, id_or_mac, sizeof(uint8_t)*20);
    mav_array_memcpy(packet.operator_id, operator_id, sizeof(char)*20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID, (const char *)&packet, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_CRC);
#endif
}

/**
 * @brief Send a open_drone_id_operator_id message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_open_drone_id_operator_id_send_struct(mavlink_channel_t chan, const mavlink_open_drone_id_operator_id_t* open_drone_id_operator_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_open_drone_id_operator_id_send(chan, open_drone_id_operator_id->target_system, open_drone_id_operator_id->target_component, open_drone_id_operator_id->id_or_mac, open_drone_id_operator_id->operator_id_type, open_drone_id_operator_id->operator_id);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID, (const char *)open_drone_id_operator_id, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_CRC);
#endif
}

#if MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_open_drone_id_operator_id_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, const uint8_t *id_or_mac, uint8_t operator_id_type, const char *operator_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 22, operator_id_type);
    _mav_put_uint8_t_array(buf, 2, id_or_mac, 20);
    _mav_put_char_array(buf, 23, operator_id, 20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID, buf, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_CRC);
#else
    mavlink_open_drone_id_operator_id_t *packet = (mavlink_open_drone_id_operator_id_t *)msgbuf;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->operator_id_type = operator_id_type;
    mav_array_memcpy(packet->id_or_mac, id_or_mac, sizeof(uint8_t)*20);
    mav_array_memcpy(packet->operator_id, operator_id, sizeof(char)*20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID, (const char *)packet, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_CRC);
#endif
}
#endif

#endif

// MESSAGE OPEN_DRONE_ID_OPERATOR_ID UNPACKING


/**
 * @brief Get field target_system from open_drone_id_operator_id message
 *
 * @return  System ID (0 for broadcast).
 */
static inline uint8_t mavlink_msg_open_drone_id_operator_id_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field target_component from open_drone_id_operator_id message
 *
 * @return  Component ID (0 for broadcast).
 */
static inline uint8_t mavlink_msg_open_drone_id_operator_id_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field id_or_mac from open_drone_id_operator_id message
 *
 * @return  Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html. 
 */
static inline uint16_t mavlink_msg_open_drone_id_operator_id_get_id_or_mac(const mavlink_message_t* msg, uint8_t *id_or_mac)
{
    return _MAV_RETURN_uint8_t_array(msg, id_or_mac, 20,  2);
}

/**
 * @brief Get field operator_id_type from open_drone_id_operator_id message
 *
 * @return  Indicates the type of the operator_id field.
 */
static inline uint8_t mavlink_msg_open_drone_id_operator_id_get_operator_id_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Get field operator_id from open_drone_id_operator_id message
 *
 * @return  Text description or numeric value expressed as ASCII characters. Shall be filled with nulls in the unused portion of the field.
 */
static inline uint16_t mavlink_msg_open_drone_id_operator_id_get_operator_id(const mavlink_message_t* msg, char *operator_id)
{
    return _MAV_RETURN_char_array(msg, operator_id, 20,  23);
}

/**
 * @brief Decode a open_drone_id_operator_id message into a struct
 *
 * @param msg The message to decode
 * @param open_drone_id_operator_id C-struct to decode the message contents into
 */
static inline void mavlink_msg_open_drone_id_operator_id_decode(const mavlink_message_t* msg, mavlink_open_drone_id_operator_id_t* open_drone_id_operator_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    open_drone_id_operator_id->target_system = mavlink_msg_open_drone_id_operator_id_get_target_system(msg);
    open_drone_id_operator_id->target_component = mavlink_msg_open_drone_id_operator_id_get_target_component(msg);
    mavlink_msg_open_drone_id_operator_id_get_id_or_mac(msg, open_drone_id_operator_id->id_or_mac);
    open_drone_id_operator_id->operator_id_type = mavlink_msg_open_drone_id_operator_id_get_operator_id_type(msg);
    mavlink_msg_open_drone_id_operator_id_get_operator_id(msg, open_drone_id_operator_id->operator_id);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_LEN? msg->len : MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_LEN;
        memset(open_drone_id_operator_id, 0, MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID_LEN);
    memcpy(open_drone_id_operator_id, _MAV_PAYLOAD(msg), len);
#endif
}
