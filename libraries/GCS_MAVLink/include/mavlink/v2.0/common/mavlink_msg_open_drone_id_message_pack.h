#pragma once
// MESSAGE OPEN_DRONE_ID_MESSAGE_PACK PACKING

#define MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK 12915


typedef struct __mavlink_open_drone_id_message_pack_t {
 uint8_t target_system; /*<  System ID (0 for broadcast).*/
 uint8_t target_component; /*<  Component ID (0 for broadcast).*/
 uint8_t id_or_mac[20]; /*<  Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html. */
 uint8_t single_message_size; /*< [bytes] This field must currently always be equal to 25 (bytes), since all encoded OpenDroneID messages are specified to have this length.*/
 uint8_t msg_pack_size; /*<  Number of encoded messages in the pack (not the number of bytes). Allowed range is 1 - 9.*/
 uint8_t messages[225]; /*<  Concatenation of encoded OpenDroneID messages. Shall be filled with nulls in the unused portion of the field.*/
} mavlink_open_drone_id_message_pack_t;

#define MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_LEN 249
#define MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_MIN_LEN 249
#define MAVLINK_MSG_ID_12915_LEN 249
#define MAVLINK_MSG_ID_12915_MIN_LEN 249

#define MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_CRC 94
#define MAVLINK_MSG_ID_12915_CRC 94

#define MAVLINK_MSG_OPEN_DRONE_ID_MESSAGE_PACK_FIELD_ID_OR_MAC_LEN 20
#define MAVLINK_MSG_OPEN_DRONE_ID_MESSAGE_PACK_FIELD_MESSAGES_LEN 225

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_OPEN_DRONE_ID_MESSAGE_PACK { \
    12915, \
    "OPEN_DRONE_ID_MESSAGE_PACK", \
    6, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_open_drone_id_message_pack_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_open_drone_id_message_pack_t, target_component) }, \
         { "id_or_mac", NULL, MAVLINK_TYPE_UINT8_T, 20, 2, offsetof(mavlink_open_drone_id_message_pack_t, id_or_mac) }, \
         { "single_message_size", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_open_drone_id_message_pack_t, single_message_size) }, \
         { "msg_pack_size", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_open_drone_id_message_pack_t, msg_pack_size) }, \
         { "messages", NULL, MAVLINK_TYPE_UINT8_T, 225, 24, offsetof(mavlink_open_drone_id_message_pack_t, messages) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_OPEN_DRONE_ID_MESSAGE_PACK { \
    "OPEN_DRONE_ID_MESSAGE_PACK", \
    6, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_open_drone_id_message_pack_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_open_drone_id_message_pack_t, target_component) }, \
         { "id_or_mac", NULL, MAVLINK_TYPE_UINT8_T, 20, 2, offsetof(mavlink_open_drone_id_message_pack_t, id_or_mac) }, \
         { "single_message_size", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_open_drone_id_message_pack_t, single_message_size) }, \
         { "msg_pack_size", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_open_drone_id_message_pack_t, msg_pack_size) }, \
         { "messages", NULL, MAVLINK_TYPE_UINT8_T, 225, 24, offsetof(mavlink_open_drone_id_message_pack_t, messages) }, \
         } \
}
#endif

/**
 * @brief Pack a open_drone_id_message_pack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID (0 for broadcast).
 * @param target_component  Component ID (0 for broadcast).
 * @param id_or_mac  Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html. 
 * @param single_message_size [bytes] This field must currently always be equal to 25 (bytes), since all encoded OpenDroneID messages are specified to have this length.
 * @param msg_pack_size  Number of encoded messages in the pack (not the number of bytes). Allowed range is 1 - 9.
 * @param messages  Concatenation of encoded OpenDroneID messages. Shall be filled with nulls in the unused portion of the field.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_open_drone_id_message_pack_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, const uint8_t *id_or_mac, uint8_t single_message_size, uint8_t msg_pack_size, const uint8_t *messages)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 22, single_message_size);
    _mav_put_uint8_t(buf, 23, msg_pack_size);
    _mav_put_uint8_t_array(buf, 2, id_or_mac, 20);
    _mav_put_uint8_t_array(buf, 24, messages, 225);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_LEN);
#else
    mavlink_open_drone_id_message_pack_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.single_message_size = single_message_size;
    packet.msg_pack_size = msg_pack_size;
    mav_array_memcpy(packet.id_or_mac, id_or_mac, sizeof(uint8_t)*20);
    mav_array_memcpy(packet.messages, messages, sizeof(uint8_t)*225);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_CRC);
}

/**
 * @brief Pack a open_drone_id_message_pack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID (0 for broadcast).
 * @param target_component  Component ID (0 for broadcast).
 * @param id_or_mac  Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html. 
 * @param single_message_size [bytes] This field must currently always be equal to 25 (bytes), since all encoded OpenDroneID messages are specified to have this length.
 * @param msg_pack_size  Number of encoded messages in the pack (not the number of bytes). Allowed range is 1 - 9.
 * @param messages  Concatenation of encoded OpenDroneID messages. Shall be filled with nulls in the unused portion of the field.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_open_drone_id_message_pack_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, const uint8_t *id_or_mac, uint8_t single_message_size, uint8_t msg_pack_size, const uint8_t *messages)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 22, single_message_size);
    _mav_put_uint8_t(buf, 23, msg_pack_size);
    _mav_put_uint8_t_array(buf, 2, id_or_mac, 20);
    _mav_put_uint8_t_array(buf, 24, messages, 225);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_LEN);
#else
    mavlink_open_drone_id_message_pack_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.single_message_size = single_message_size;
    packet.msg_pack_size = msg_pack_size;
    mav_array_memcpy(packet.id_or_mac, id_or_mac, sizeof(uint8_t)*20);
    mav_array_memcpy(packet.messages, messages, sizeof(uint8_t)*225);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_LEN);
#endif
}

/**
 * @brief Pack a open_drone_id_message_pack message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID (0 for broadcast).
 * @param target_component  Component ID (0 for broadcast).
 * @param id_or_mac  Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html. 
 * @param single_message_size [bytes] This field must currently always be equal to 25 (bytes), since all encoded OpenDroneID messages are specified to have this length.
 * @param msg_pack_size  Number of encoded messages in the pack (not the number of bytes). Allowed range is 1 - 9.
 * @param messages  Concatenation of encoded OpenDroneID messages. Shall be filled with nulls in the unused portion of the field.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_open_drone_id_message_pack_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,const uint8_t *id_or_mac,uint8_t single_message_size,uint8_t msg_pack_size,const uint8_t *messages)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 22, single_message_size);
    _mav_put_uint8_t(buf, 23, msg_pack_size);
    _mav_put_uint8_t_array(buf, 2, id_or_mac, 20);
    _mav_put_uint8_t_array(buf, 24, messages, 225);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_LEN);
#else
    mavlink_open_drone_id_message_pack_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.single_message_size = single_message_size;
    packet.msg_pack_size = msg_pack_size;
    mav_array_memcpy(packet.id_or_mac, id_or_mac, sizeof(uint8_t)*20);
    mav_array_memcpy(packet.messages, messages, sizeof(uint8_t)*225);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_CRC);
}

/**
 * @brief Encode a open_drone_id_message_pack struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param open_drone_id_message_pack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_open_drone_id_message_pack_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_open_drone_id_message_pack_t* open_drone_id_message_pack)
{
    return mavlink_msg_open_drone_id_message_pack_pack(system_id, component_id, msg, open_drone_id_message_pack->target_system, open_drone_id_message_pack->target_component, open_drone_id_message_pack->id_or_mac, open_drone_id_message_pack->single_message_size, open_drone_id_message_pack->msg_pack_size, open_drone_id_message_pack->messages);
}

/**
 * @brief Encode a open_drone_id_message_pack struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param open_drone_id_message_pack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_open_drone_id_message_pack_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_open_drone_id_message_pack_t* open_drone_id_message_pack)
{
    return mavlink_msg_open_drone_id_message_pack_pack_chan(system_id, component_id, chan, msg, open_drone_id_message_pack->target_system, open_drone_id_message_pack->target_component, open_drone_id_message_pack->id_or_mac, open_drone_id_message_pack->single_message_size, open_drone_id_message_pack->msg_pack_size, open_drone_id_message_pack->messages);
}

/**
 * @brief Encode a open_drone_id_message_pack struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param open_drone_id_message_pack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_open_drone_id_message_pack_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_open_drone_id_message_pack_t* open_drone_id_message_pack)
{
    return mavlink_msg_open_drone_id_message_pack_pack_status(system_id, component_id, _status, msg,  open_drone_id_message_pack->target_system, open_drone_id_message_pack->target_component, open_drone_id_message_pack->id_or_mac, open_drone_id_message_pack->single_message_size, open_drone_id_message_pack->msg_pack_size, open_drone_id_message_pack->messages);
}

/**
 * @brief Send a open_drone_id_message_pack message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID (0 for broadcast).
 * @param target_component  Component ID (0 for broadcast).
 * @param id_or_mac  Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html. 
 * @param single_message_size [bytes] This field must currently always be equal to 25 (bytes), since all encoded OpenDroneID messages are specified to have this length.
 * @param msg_pack_size  Number of encoded messages in the pack (not the number of bytes). Allowed range is 1 - 9.
 * @param messages  Concatenation of encoded OpenDroneID messages. Shall be filled with nulls in the unused portion of the field.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_open_drone_id_message_pack_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, const uint8_t *id_or_mac, uint8_t single_message_size, uint8_t msg_pack_size, const uint8_t *messages)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 22, single_message_size);
    _mav_put_uint8_t(buf, 23, msg_pack_size);
    _mav_put_uint8_t_array(buf, 2, id_or_mac, 20);
    _mav_put_uint8_t_array(buf, 24, messages, 225);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK, buf, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_CRC);
#else
    mavlink_open_drone_id_message_pack_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.single_message_size = single_message_size;
    packet.msg_pack_size = msg_pack_size;
    mav_array_memcpy(packet.id_or_mac, id_or_mac, sizeof(uint8_t)*20);
    mav_array_memcpy(packet.messages, messages, sizeof(uint8_t)*225);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK, (const char *)&packet, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_CRC);
#endif
}

/**
 * @brief Send a open_drone_id_message_pack message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_open_drone_id_message_pack_send_struct(mavlink_channel_t chan, const mavlink_open_drone_id_message_pack_t* open_drone_id_message_pack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_open_drone_id_message_pack_send(chan, open_drone_id_message_pack->target_system, open_drone_id_message_pack->target_component, open_drone_id_message_pack->id_or_mac, open_drone_id_message_pack->single_message_size, open_drone_id_message_pack->msg_pack_size, open_drone_id_message_pack->messages);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK, (const char *)open_drone_id_message_pack, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_CRC);
#endif
}

#if MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_open_drone_id_message_pack_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, const uint8_t *id_or_mac, uint8_t single_message_size, uint8_t msg_pack_size, const uint8_t *messages)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 22, single_message_size);
    _mav_put_uint8_t(buf, 23, msg_pack_size);
    _mav_put_uint8_t_array(buf, 2, id_or_mac, 20);
    _mav_put_uint8_t_array(buf, 24, messages, 225);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK, buf, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_CRC);
#else
    mavlink_open_drone_id_message_pack_t *packet = (mavlink_open_drone_id_message_pack_t *)msgbuf;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->single_message_size = single_message_size;
    packet->msg_pack_size = msg_pack_size;
    mav_array_memcpy(packet->id_or_mac, id_or_mac, sizeof(uint8_t)*20);
    mav_array_memcpy(packet->messages, messages, sizeof(uint8_t)*225);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK, (const char *)packet, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_MIN_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_LEN, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_CRC);
#endif
}
#endif

#endif

// MESSAGE OPEN_DRONE_ID_MESSAGE_PACK UNPACKING


/**
 * @brief Get field target_system from open_drone_id_message_pack message
 *
 * @return  System ID (0 for broadcast).
 */
static inline uint8_t mavlink_msg_open_drone_id_message_pack_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field target_component from open_drone_id_message_pack message
 *
 * @return  Component ID (0 for broadcast).
 */
static inline uint8_t mavlink_msg_open_drone_id_message_pack_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field id_or_mac from open_drone_id_message_pack message
 *
 * @return  Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html. 
 */
static inline uint16_t mavlink_msg_open_drone_id_message_pack_get_id_or_mac(const mavlink_message_t* msg, uint8_t *id_or_mac)
{
    return _MAV_RETURN_uint8_t_array(msg, id_or_mac, 20,  2);
}

/**
 * @brief Get field single_message_size from open_drone_id_message_pack message
 *
 * @return [bytes] This field must currently always be equal to 25 (bytes), since all encoded OpenDroneID messages are specified to have this length.
 */
static inline uint8_t mavlink_msg_open_drone_id_message_pack_get_single_message_size(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Get field msg_pack_size from open_drone_id_message_pack message
 *
 * @return  Number of encoded messages in the pack (not the number of bytes). Allowed range is 1 - 9.
 */
static inline uint8_t mavlink_msg_open_drone_id_message_pack_get_msg_pack_size(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  23);
}

/**
 * @brief Get field messages from open_drone_id_message_pack message
 *
 * @return  Concatenation of encoded OpenDroneID messages. Shall be filled with nulls in the unused portion of the field.
 */
static inline uint16_t mavlink_msg_open_drone_id_message_pack_get_messages(const mavlink_message_t* msg, uint8_t *messages)
{
    return _MAV_RETURN_uint8_t_array(msg, messages, 225,  24);
}

/**
 * @brief Decode a open_drone_id_message_pack message into a struct
 *
 * @param msg The message to decode
 * @param open_drone_id_message_pack C-struct to decode the message contents into
 */
static inline void mavlink_msg_open_drone_id_message_pack_decode(const mavlink_message_t* msg, mavlink_open_drone_id_message_pack_t* open_drone_id_message_pack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    open_drone_id_message_pack->target_system = mavlink_msg_open_drone_id_message_pack_get_target_system(msg);
    open_drone_id_message_pack->target_component = mavlink_msg_open_drone_id_message_pack_get_target_component(msg);
    mavlink_msg_open_drone_id_message_pack_get_id_or_mac(msg, open_drone_id_message_pack->id_or_mac);
    open_drone_id_message_pack->single_message_size = mavlink_msg_open_drone_id_message_pack_get_single_message_size(msg);
    open_drone_id_message_pack->msg_pack_size = mavlink_msg_open_drone_id_message_pack_get_msg_pack_size(msg);
    mavlink_msg_open_drone_id_message_pack_get_messages(msg, open_drone_id_message_pack->messages);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_LEN? msg->len : MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_LEN;
        memset(open_drone_id_message_pack, 0, MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK_LEN);
    memcpy(open_drone_id_message_pack, _MAV_PAYLOAD(msg), len);
#endif
}
