#pragma once
// MESSAGE TUNNEL PACKING

#define MAVLINK_MSG_ID_TUNNEL 385


typedef struct __mavlink_tunnel_t {
 uint16_t payload_type; /*<  A code that identifies the content of the payload (0 for unknown, which is the default). If this code is less than 32768, it is a 'registered' payload type and the corresponding code should be added to the MAV_TUNNEL_PAYLOAD_TYPE enum. Software creators can register blocks of types as needed. Codes greater than 32767 are considered local experiments and should not be checked in to any widely distributed codebase.*/
 uint8_t target_system; /*<  System ID (can be 0 for broadcast, but this is discouraged)*/
 uint8_t target_component; /*<  Component ID (can be 0 for broadcast, but this is discouraged)*/
 uint8_t payload_length; /*<  Length of the data transported in payload*/
 uint8_t payload[128]; /*<  Variable length payload. The payload length is defined by payload_length. The entire content of this block is opaque unless you understand the encoding specified by payload_type.*/
} mavlink_tunnel_t;

#define MAVLINK_MSG_ID_TUNNEL_LEN 133
#define MAVLINK_MSG_ID_TUNNEL_MIN_LEN 133
#define MAVLINK_MSG_ID_385_LEN 133
#define MAVLINK_MSG_ID_385_MIN_LEN 133

#define MAVLINK_MSG_ID_TUNNEL_CRC 147
#define MAVLINK_MSG_ID_385_CRC 147

#define MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN 128

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TUNNEL { \
    385, \
    "TUNNEL", \
    5, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_tunnel_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_tunnel_t, target_component) }, \
         { "payload_type", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_tunnel_t, payload_type) }, \
         { "payload_length", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_tunnel_t, payload_length) }, \
         { "payload", NULL, MAVLINK_TYPE_UINT8_T, 128, 5, offsetof(mavlink_tunnel_t, payload) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TUNNEL { \
    "TUNNEL", \
    5, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_tunnel_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_tunnel_t, target_component) }, \
         { "payload_type", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_tunnel_t, payload_type) }, \
         { "payload_length", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_tunnel_t, payload_length) }, \
         { "payload", NULL, MAVLINK_TYPE_UINT8_T, 128, 5, offsetof(mavlink_tunnel_t, payload) }, \
         } \
}
#endif

/**
 * @brief Pack a tunnel message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID (can be 0 for broadcast, but this is discouraged)
 * @param target_component  Component ID (can be 0 for broadcast, but this is discouraged)
 * @param payload_type  A code that identifies the content of the payload (0 for unknown, which is the default). If this code is less than 32768, it is a 'registered' payload type and the corresponding code should be added to the MAV_TUNNEL_PAYLOAD_TYPE enum. Software creators can register blocks of types as needed. Codes greater than 32767 are considered local experiments and should not be checked in to any widely distributed codebase.
 * @param payload_length  Length of the data transported in payload
 * @param payload  Variable length payload. The payload length is defined by payload_length. The entire content of this block is opaque unless you understand the encoding specified by payload_type.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tunnel_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint16_t payload_type, uint8_t payload_length, const uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TUNNEL_LEN];
    _mav_put_uint16_t(buf, 0, payload_type);
    _mav_put_uint8_t(buf, 2, target_system);
    _mav_put_uint8_t(buf, 3, target_component);
    _mav_put_uint8_t(buf, 4, payload_length);
    _mav_put_uint8_t_array(buf, 5, payload, 128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TUNNEL_LEN);
#else
    mavlink_tunnel_t packet;
    packet.payload_type = payload_type;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.payload_length = payload_length;
    mav_array_memcpy(packet.payload, payload, sizeof(uint8_t)*128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TUNNEL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TUNNEL;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TUNNEL_MIN_LEN, MAVLINK_MSG_ID_TUNNEL_LEN, MAVLINK_MSG_ID_TUNNEL_CRC);
}

/**
 * @brief Pack a tunnel message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID (can be 0 for broadcast, but this is discouraged)
 * @param target_component  Component ID (can be 0 for broadcast, but this is discouraged)
 * @param payload_type  A code that identifies the content of the payload (0 for unknown, which is the default). If this code is less than 32768, it is a 'registered' payload type and the corresponding code should be added to the MAV_TUNNEL_PAYLOAD_TYPE enum. Software creators can register blocks of types as needed. Codes greater than 32767 are considered local experiments and should not be checked in to any widely distributed codebase.
 * @param payload_length  Length of the data transported in payload
 * @param payload  Variable length payload. The payload length is defined by payload_length. The entire content of this block is opaque unless you understand the encoding specified by payload_type.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tunnel_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint16_t payload_type, uint8_t payload_length, const uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TUNNEL_LEN];
    _mav_put_uint16_t(buf, 0, payload_type);
    _mav_put_uint8_t(buf, 2, target_system);
    _mav_put_uint8_t(buf, 3, target_component);
    _mav_put_uint8_t(buf, 4, payload_length);
    _mav_put_uint8_t_array(buf, 5, payload, 128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TUNNEL_LEN);
#else
    mavlink_tunnel_t packet;
    packet.payload_type = payload_type;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.payload_length = payload_length;
    mav_array_memcpy(packet.payload, payload, sizeof(uint8_t)*128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TUNNEL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TUNNEL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TUNNEL_MIN_LEN, MAVLINK_MSG_ID_TUNNEL_LEN, MAVLINK_MSG_ID_TUNNEL_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TUNNEL_MIN_LEN, MAVLINK_MSG_ID_TUNNEL_LEN);
#endif
}

/**
 * @brief Pack a tunnel message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID (can be 0 for broadcast, but this is discouraged)
 * @param target_component  Component ID (can be 0 for broadcast, but this is discouraged)
 * @param payload_type  A code that identifies the content of the payload (0 for unknown, which is the default). If this code is less than 32768, it is a 'registered' payload type and the corresponding code should be added to the MAV_TUNNEL_PAYLOAD_TYPE enum. Software creators can register blocks of types as needed. Codes greater than 32767 are considered local experiments and should not be checked in to any widely distributed codebase.
 * @param payload_length  Length of the data transported in payload
 * @param payload  Variable length payload. The payload length is defined by payload_length. The entire content of this block is opaque unless you understand the encoding specified by payload_type.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tunnel_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint16_t payload_type,uint8_t payload_length,const uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TUNNEL_LEN];
    _mav_put_uint16_t(buf, 0, payload_type);
    _mav_put_uint8_t(buf, 2, target_system);
    _mav_put_uint8_t(buf, 3, target_component);
    _mav_put_uint8_t(buf, 4, payload_length);
    _mav_put_uint8_t_array(buf, 5, payload, 128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TUNNEL_LEN);
#else
    mavlink_tunnel_t packet;
    packet.payload_type = payload_type;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.payload_length = payload_length;
    mav_array_memcpy(packet.payload, payload, sizeof(uint8_t)*128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TUNNEL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TUNNEL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TUNNEL_MIN_LEN, MAVLINK_MSG_ID_TUNNEL_LEN, MAVLINK_MSG_ID_TUNNEL_CRC);
}

/**
 * @brief Encode a tunnel struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param tunnel C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tunnel_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_tunnel_t* tunnel)
{
    return mavlink_msg_tunnel_pack(system_id, component_id, msg, tunnel->target_system, tunnel->target_component, tunnel->payload_type, tunnel->payload_length, tunnel->payload);
}

/**
 * @brief Encode a tunnel struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param tunnel C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tunnel_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_tunnel_t* tunnel)
{
    return mavlink_msg_tunnel_pack_chan(system_id, component_id, chan, msg, tunnel->target_system, tunnel->target_component, tunnel->payload_type, tunnel->payload_length, tunnel->payload);
}

/**
 * @brief Encode a tunnel struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param tunnel C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tunnel_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_tunnel_t* tunnel)
{
    return mavlink_msg_tunnel_pack_status(system_id, component_id, _status, msg,  tunnel->target_system, tunnel->target_component, tunnel->payload_type, tunnel->payload_length, tunnel->payload);
}

/**
 * @brief Send a tunnel message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID (can be 0 for broadcast, but this is discouraged)
 * @param target_component  Component ID (can be 0 for broadcast, but this is discouraged)
 * @param payload_type  A code that identifies the content of the payload (0 for unknown, which is the default). If this code is less than 32768, it is a 'registered' payload type and the corresponding code should be added to the MAV_TUNNEL_PAYLOAD_TYPE enum. Software creators can register blocks of types as needed. Codes greater than 32767 are considered local experiments and should not be checked in to any widely distributed codebase.
 * @param payload_length  Length of the data transported in payload
 * @param payload  Variable length payload. The payload length is defined by payload_length. The entire content of this block is opaque unless you understand the encoding specified by payload_type.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_tunnel_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint16_t payload_type, uint8_t payload_length, const uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TUNNEL_LEN];
    _mav_put_uint16_t(buf, 0, payload_type);
    _mav_put_uint8_t(buf, 2, target_system);
    _mav_put_uint8_t(buf, 3, target_component);
    _mav_put_uint8_t(buf, 4, payload_length);
    _mav_put_uint8_t_array(buf, 5, payload, 128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TUNNEL, buf, MAVLINK_MSG_ID_TUNNEL_MIN_LEN, MAVLINK_MSG_ID_TUNNEL_LEN, MAVLINK_MSG_ID_TUNNEL_CRC);
#else
    mavlink_tunnel_t packet;
    packet.payload_type = payload_type;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.payload_length = payload_length;
    mav_array_memcpy(packet.payload, payload, sizeof(uint8_t)*128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TUNNEL, (const char *)&packet, MAVLINK_MSG_ID_TUNNEL_MIN_LEN, MAVLINK_MSG_ID_TUNNEL_LEN, MAVLINK_MSG_ID_TUNNEL_CRC);
#endif
}

/**
 * @brief Send a tunnel message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_tunnel_send_struct(mavlink_channel_t chan, const mavlink_tunnel_t* tunnel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_tunnel_send(chan, tunnel->target_system, tunnel->target_component, tunnel->payload_type, tunnel->payload_length, tunnel->payload);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TUNNEL, (const char *)tunnel, MAVLINK_MSG_ID_TUNNEL_MIN_LEN, MAVLINK_MSG_ID_TUNNEL_LEN, MAVLINK_MSG_ID_TUNNEL_CRC);
#endif
}

#if MAVLINK_MSG_ID_TUNNEL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_tunnel_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint16_t payload_type, uint8_t payload_length, const uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, payload_type);
    _mav_put_uint8_t(buf, 2, target_system);
    _mav_put_uint8_t(buf, 3, target_component);
    _mav_put_uint8_t(buf, 4, payload_length);
    _mav_put_uint8_t_array(buf, 5, payload, 128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TUNNEL, buf, MAVLINK_MSG_ID_TUNNEL_MIN_LEN, MAVLINK_MSG_ID_TUNNEL_LEN, MAVLINK_MSG_ID_TUNNEL_CRC);
#else
    mavlink_tunnel_t *packet = (mavlink_tunnel_t *)msgbuf;
    packet->payload_type = payload_type;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->payload_length = payload_length;
    mav_array_memcpy(packet->payload, payload, sizeof(uint8_t)*128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TUNNEL, (const char *)packet, MAVLINK_MSG_ID_TUNNEL_MIN_LEN, MAVLINK_MSG_ID_TUNNEL_LEN, MAVLINK_MSG_ID_TUNNEL_CRC);
#endif
}
#endif

#endif

// MESSAGE TUNNEL UNPACKING


/**
 * @brief Get field target_system from tunnel message
 *
 * @return  System ID (can be 0 for broadcast, but this is discouraged)
 */
static inline uint8_t mavlink_msg_tunnel_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field target_component from tunnel message
 *
 * @return  Component ID (can be 0 for broadcast, but this is discouraged)
 */
static inline uint8_t mavlink_msg_tunnel_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field payload_type from tunnel message
 *
 * @return  A code that identifies the content of the payload (0 for unknown, which is the default). If this code is less than 32768, it is a 'registered' payload type and the corresponding code should be added to the MAV_TUNNEL_PAYLOAD_TYPE enum. Software creators can register blocks of types as needed. Codes greater than 32767 are considered local experiments and should not be checked in to any widely distributed codebase.
 */
static inline uint16_t mavlink_msg_tunnel_get_payload_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field payload_length from tunnel message
 *
 * @return  Length of the data transported in payload
 */
static inline uint8_t mavlink_msg_tunnel_get_payload_length(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field payload from tunnel message
 *
 * @return  Variable length payload. The payload length is defined by payload_length. The entire content of this block is opaque unless you understand the encoding specified by payload_type.
 */
static inline uint16_t mavlink_msg_tunnel_get_payload(const mavlink_message_t* msg, uint8_t *payload)
{
    return _MAV_RETURN_uint8_t_array(msg, payload, 128,  5);
}

/**
 * @brief Decode a tunnel message into a struct
 *
 * @param msg The message to decode
 * @param tunnel C-struct to decode the message contents into
 */
static inline void mavlink_msg_tunnel_decode(const mavlink_message_t* msg, mavlink_tunnel_t* tunnel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    tunnel->payload_type = mavlink_msg_tunnel_get_payload_type(msg);
    tunnel->target_system = mavlink_msg_tunnel_get_target_system(msg);
    tunnel->target_component = mavlink_msg_tunnel_get_target_component(msg);
    tunnel->payload_length = mavlink_msg_tunnel_get_payload_length(msg);
    mavlink_msg_tunnel_get_payload(msg, tunnel->payload);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TUNNEL_LEN? msg->len : MAVLINK_MSG_ID_TUNNEL_LEN;
        memset(tunnel, 0, MAVLINK_MSG_ID_TUNNEL_LEN);
    memcpy(tunnel, _MAV_PAYLOAD(msg), len);
#endif
}
