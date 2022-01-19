#pragma once
// MESSAGE FILE_TRANSFER_PROTOCOL PACKING

#define MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL 110


typedef struct __mavlink_file_transfer_protocol_t {
 uint8_t target_network; /*<  Network ID (0 for broadcast)*/
 uint8_t target_system; /*<  System ID (0 for broadcast)*/
 uint8_t target_component; /*<  Component ID (0 for broadcast)*/
 uint8_t payload[251]; /*<  Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields.  The entire content of this block is opaque unless you understand any the encoding message_type.  The particular encoding used can be extension specific and might not always be documented as part of the mavlink specification.*/
} mavlink_file_transfer_protocol_t;

#define MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_LEN 254
#define MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_MIN_LEN 254
#define MAVLINK_MSG_ID_110_LEN 254
#define MAVLINK_MSG_ID_110_MIN_LEN 254

#define MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_CRC 84
#define MAVLINK_MSG_ID_110_CRC 84

#define MAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FIELD_PAYLOAD_LEN 251

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FILE_TRANSFER_PROTOCOL { \
    110, \
    "FILE_TRANSFER_PROTOCOL", \
    4, \
    {  { "target_network", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_file_transfer_protocol_t, target_network) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_file_transfer_protocol_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_file_transfer_protocol_t, target_component) }, \
         { "payload", NULL, MAVLINK_TYPE_UINT8_T, 251, 3, offsetof(mavlink_file_transfer_protocol_t, payload) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FILE_TRANSFER_PROTOCOL { \
    "FILE_TRANSFER_PROTOCOL", \
    4, \
    {  { "target_network", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_file_transfer_protocol_t, target_network) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_file_transfer_protocol_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_file_transfer_protocol_t, target_component) }, \
         { "payload", NULL, MAVLINK_TYPE_UINT8_T, 251, 3, offsetof(mavlink_file_transfer_protocol_t, payload) }, \
         } \
}
#endif

/**
 * @brief Pack a file_transfer_protocol message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_network  Network ID (0 for broadcast)
 * @param target_system  System ID (0 for broadcast)
 * @param target_component  Component ID (0 for broadcast)
 * @param payload  Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields.  The entire content of this block is opaque unless you understand any the encoding message_type.  The particular encoding used can be extension specific and might not always be documented as part of the mavlink specification.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_file_transfer_protocol_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_network, uint8_t target_system, uint8_t target_component, const uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_LEN];
    _mav_put_uint8_t(buf, 0, target_network);
    _mav_put_uint8_t(buf, 1, target_system);
    _mav_put_uint8_t(buf, 2, target_component);
    _mav_put_uint8_t_array(buf, 3, payload, 251);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_LEN);
#else
    mavlink_file_transfer_protocol_t packet;
    packet.target_network = target_network;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.payload, payload, sizeof(uint8_t)*251);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_MIN_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_CRC);
}

/**
 * @brief Pack a file_transfer_protocol message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_network  Network ID (0 for broadcast)
 * @param target_system  System ID (0 for broadcast)
 * @param target_component  Component ID (0 for broadcast)
 * @param payload  Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields.  The entire content of this block is opaque unless you understand any the encoding message_type.  The particular encoding used can be extension specific and might not always be documented as part of the mavlink specification.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_file_transfer_protocol_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_network,uint8_t target_system,uint8_t target_component,const uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_LEN];
    _mav_put_uint8_t(buf, 0, target_network);
    _mav_put_uint8_t(buf, 1, target_system);
    _mav_put_uint8_t(buf, 2, target_component);
    _mav_put_uint8_t_array(buf, 3, payload, 251);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_LEN);
#else
    mavlink_file_transfer_protocol_t packet;
    packet.target_network = target_network;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.payload, payload, sizeof(uint8_t)*251);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_MIN_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_CRC);
}

/**
 * @brief Encode a file_transfer_protocol struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param file_transfer_protocol C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_file_transfer_protocol_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_file_transfer_protocol_t* file_transfer_protocol)
{
    return mavlink_msg_file_transfer_protocol_pack(system_id, component_id, msg, file_transfer_protocol->target_network, file_transfer_protocol->target_system, file_transfer_protocol->target_component, file_transfer_protocol->payload);
}

/**
 * @brief Encode a file_transfer_protocol struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param file_transfer_protocol C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_file_transfer_protocol_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_file_transfer_protocol_t* file_transfer_protocol)
{
    return mavlink_msg_file_transfer_protocol_pack_chan(system_id, component_id, chan, msg, file_transfer_protocol->target_network, file_transfer_protocol->target_system, file_transfer_protocol->target_component, file_transfer_protocol->payload);
}

/**
 * @brief Send a file_transfer_protocol message
 * @param chan MAVLink channel to send the message
 *
 * @param target_network  Network ID (0 for broadcast)
 * @param target_system  System ID (0 for broadcast)
 * @param target_component  Component ID (0 for broadcast)
 * @param payload  Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields.  The entire content of this block is opaque unless you understand any the encoding message_type.  The particular encoding used can be extension specific and might not always be documented as part of the mavlink specification.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_file_transfer_protocol_send(mavlink_channel_t chan, uint8_t target_network, uint8_t target_system, uint8_t target_component, const uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_LEN];
    _mav_put_uint8_t(buf, 0, target_network);
    _mav_put_uint8_t(buf, 1, target_system);
    _mav_put_uint8_t(buf, 2, target_component);
    _mav_put_uint8_t_array(buf, 3, payload, 251);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL, buf, MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_MIN_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_CRC);
#else
    mavlink_file_transfer_protocol_t packet;
    packet.target_network = target_network;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.payload, payload, sizeof(uint8_t)*251);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL, (const char *)&packet, MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_MIN_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_CRC);
#endif
}

/**
 * @brief Send a file_transfer_protocol message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_file_transfer_protocol_send_struct(mavlink_channel_t chan, const mavlink_file_transfer_protocol_t* file_transfer_protocol)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_file_transfer_protocol_send(chan, file_transfer_protocol->target_network, file_transfer_protocol->target_system, file_transfer_protocol->target_component, file_transfer_protocol->payload);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL, (const char *)file_transfer_protocol, MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_MIN_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_CRC);
#endif
}

#if MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_file_transfer_protocol_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_network, uint8_t target_system, uint8_t target_component, const uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, target_network);
    _mav_put_uint8_t(buf, 1, target_system);
    _mav_put_uint8_t(buf, 2, target_component);
    _mav_put_uint8_t_array(buf, 3, payload, 251);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL, buf, MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_MIN_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_CRC);
#else
    mavlink_file_transfer_protocol_t *packet = (mavlink_file_transfer_protocol_t *)msgbuf;
    packet->target_network = target_network;
    packet->target_system = target_system;
    packet->target_component = target_component;
    mav_array_memcpy(packet->payload, payload, sizeof(uint8_t)*251);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL, (const char *)packet, MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_MIN_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_CRC);
#endif
}
#endif

#endif

// MESSAGE FILE_TRANSFER_PROTOCOL UNPACKING


/**
 * @brief Get field target_network from file_transfer_protocol message
 *
 * @return  Network ID (0 for broadcast)
 */
static inline uint8_t mavlink_msg_file_transfer_protocol_get_target_network(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field target_system from file_transfer_protocol message
 *
 * @return  System ID (0 for broadcast)
 */
static inline uint8_t mavlink_msg_file_transfer_protocol_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field target_component from file_transfer_protocol message
 *
 * @return  Component ID (0 for broadcast)
 */
static inline uint8_t mavlink_msg_file_transfer_protocol_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field payload from file_transfer_protocol message
 *
 * @return  Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields.  The entire content of this block is opaque unless you understand any the encoding message_type.  The particular encoding used can be extension specific and might not always be documented as part of the mavlink specification.
 */
static inline uint16_t mavlink_msg_file_transfer_protocol_get_payload(const mavlink_message_t* msg, uint8_t *payload)
{
    return _MAV_RETURN_uint8_t_array(msg, payload, 251,  3);
}

/**
 * @brief Decode a file_transfer_protocol message into a struct
 *
 * @param msg The message to decode
 * @param file_transfer_protocol C-struct to decode the message contents into
 */
static inline void mavlink_msg_file_transfer_protocol_decode(const mavlink_message_t* msg, mavlink_file_transfer_protocol_t* file_transfer_protocol)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    file_transfer_protocol->target_network = mavlink_msg_file_transfer_protocol_get_target_network(msg);
    file_transfer_protocol->target_system = mavlink_msg_file_transfer_protocol_get_target_system(msg);
    file_transfer_protocol->target_component = mavlink_msg_file_transfer_protocol_get_target_component(msg);
    mavlink_msg_file_transfer_protocol_get_payload(msg, file_transfer_protocol->payload);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_LEN? msg->len : MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_LEN;
        memset(file_transfer_protocol, 0, MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_LEN);
    memcpy(file_transfer_protocol, _MAV_PAYLOAD(msg), len);
#endif
}
