#pragma once
// MESSAGE CAN_FRAME PACKING

#define MAVLINK_MSG_ID_CAN_FRAME 386


typedef struct __mavlink_can_frame_t {
 uint32_t id; /*<  Frame ID*/
 uint8_t target_system; /*<  System ID.*/
 uint8_t target_component; /*<  Component ID.*/
 uint8_t bus; /*<  Bus number*/
 uint8_t len; /*<  Frame length*/
 uint8_t data[8]; /*<  Frame data*/
} mavlink_can_frame_t;

#define MAVLINK_MSG_ID_CAN_FRAME_LEN 16
#define MAVLINK_MSG_ID_CAN_FRAME_MIN_LEN 16
#define MAVLINK_MSG_ID_386_LEN 16
#define MAVLINK_MSG_ID_386_MIN_LEN 16

#define MAVLINK_MSG_ID_CAN_FRAME_CRC 132
#define MAVLINK_MSG_ID_386_CRC 132

#define MAVLINK_MSG_CAN_FRAME_FIELD_DATA_LEN 8

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CAN_FRAME { \
    386, \
    "CAN_FRAME", \
    6, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_can_frame_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_can_frame_t, target_component) }, \
         { "bus", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_can_frame_t, bus) }, \
         { "len", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_can_frame_t, len) }, \
         { "id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_can_frame_t, id) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 8, 8, offsetof(mavlink_can_frame_t, data) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CAN_FRAME { \
    "CAN_FRAME", \
    6, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_can_frame_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_can_frame_t, target_component) }, \
         { "bus", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_can_frame_t, bus) }, \
         { "len", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_can_frame_t, len) }, \
         { "id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_can_frame_t, id) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 8, 8, offsetof(mavlink_can_frame_t, data) }, \
         } \
}
#endif

/**
 * @brief Pack a can_frame message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param bus  Bus number
 * @param len  Frame length
 * @param id  Frame ID
 * @param data  Frame data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_can_frame_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t bus, uint8_t len, uint32_t id, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAN_FRAME_LEN];
    _mav_put_uint32_t(buf, 0, id);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 6, bus);
    _mav_put_uint8_t(buf, 7, len);
    _mav_put_uint8_t_array(buf, 8, data, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAN_FRAME_LEN);
#else
    mavlink_can_frame_t packet;
    packet.id = id;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.bus = bus;
    packet.len = len;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAN_FRAME_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAN_FRAME;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CAN_FRAME_MIN_LEN, MAVLINK_MSG_ID_CAN_FRAME_LEN, MAVLINK_MSG_ID_CAN_FRAME_CRC);
}

/**
 * @brief Pack a can_frame message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param bus  Bus number
 * @param len  Frame length
 * @param id  Frame ID
 * @param data  Frame data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_can_frame_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t bus, uint8_t len, uint32_t id, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAN_FRAME_LEN];
    _mav_put_uint32_t(buf, 0, id);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 6, bus);
    _mav_put_uint8_t(buf, 7, len);
    _mav_put_uint8_t_array(buf, 8, data, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAN_FRAME_LEN);
#else
    mavlink_can_frame_t packet;
    packet.id = id;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.bus = bus;
    packet.len = len;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAN_FRAME_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAN_FRAME;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CAN_FRAME_MIN_LEN, MAVLINK_MSG_ID_CAN_FRAME_LEN, MAVLINK_MSG_ID_CAN_FRAME_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CAN_FRAME_MIN_LEN, MAVLINK_MSG_ID_CAN_FRAME_LEN);
#endif
}

/**
 * @brief Pack a can_frame message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param bus  Bus number
 * @param len  Frame length
 * @param id  Frame ID
 * @param data  Frame data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_can_frame_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint8_t bus,uint8_t len,uint32_t id,const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAN_FRAME_LEN];
    _mav_put_uint32_t(buf, 0, id);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 6, bus);
    _mav_put_uint8_t(buf, 7, len);
    _mav_put_uint8_t_array(buf, 8, data, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAN_FRAME_LEN);
#else
    mavlink_can_frame_t packet;
    packet.id = id;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.bus = bus;
    packet.len = len;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAN_FRAME_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAN_FRAME;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CAN_FRAME_MIN_LEN, MAVLINK_MSG_ID_CAN_FRAME_LEN, MAVLINK_MSG_ID_CAN_FRAME_CRC);
}

/**
 * @brief Encode a can_frame struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param can_frame C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_can_frame_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_can_frame_t* can_frame)
{
    return mavlink_msg_can_frame_pack(system_id, component_id, msg, can_frame->target_system, can_frame->target_component, can_frame->bus, can_frame->len, can_frame->id, can_frame->data);
}

/**
 * @brief Encode a can_frame struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param can_frame C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_can_frame_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_can_frame_t* can_frame)
{
    return mavlink_msg_can_frame_pack_chan(system_id, component_id, chan, msg, can_frame->target_system, can_frame->target_component, can_frame->bus, can_frame->len, can_frame->id, can_frame->data);
}

/**
 * @brief Encode a can_frame struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param can_frame C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_can_frame_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_can_frame_t* can_frame)
{
    return mavlink_msg_can_frame_pack_status(system_id, component_id, _status, msg,  can_frame->target_system, can_frame->target_component, can_frame->bus, can_frame->len, can_frame->id, can_frame->data);
}

/**
 * @brief Send a can_frame message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param bus  Bus number
 * @param len  Frame length
 * @param id  Frame ID
 * @param data  Frame data
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_can_frame_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t bus, uint8_t len, uint32_t id, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAN_FRAME_LEN];
    _mav_put_uint32_t(buf, 0, id);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 6, bus);
    _mav_put_uint8_t(buf, 7, len);
    _mav_put_uint8_t_array(buf, 8, data, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAN_FRAME, buf, MAVLINK_MSG_ID_CAN_FRAME_MIN_LEN, MAVLINK_MSG_ID_CAN_FRAME_LEN, MAVLINK_MSG_ID_CAN_FRAME_CRC);
#else
    mavlink_can_frame_t packet;
    packet.id = id;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.bus = bus;
    packet.len = len;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAN_FRAME, (const char *)&packet, MAVLINK_MSG_ID_CAN_FRAME_MIN_LEN, MAVLINK_MSG_ID_CAN_FRAME_LEN, MAVLINK_MSG_ID_CAN_FRAME_CRC);
#endif
}

/**
 * @brief Send a can_frame message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_can_frame_send_struct(mavlink_channel_t chan, const mavlink_can_frame_t* can_frame)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_can_frame_send(chan, can_frame->target_system, can_frame->target_component, can_frame->bus, can_frame->len, can_frame->id, can_frame->data);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAN_FRAME, (const char *)can_frame, MAVLINK_MSG_ID_CAN_FRAME_MIN_LEN, MAVLINK_MSG_ID_CAN_FRAME_LEN, MAVLINK_MSG_ID_CAN_FRAME_CRC);
#endif
}

#if MAVLINK_MSG_ID_CAN_FRAME_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_can_frame_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t bus, uint8_t len, uint32_t id, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, id);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 6, bus);
    _mav_put_uint8_t(buf, 7, len);
    _mav_put_uint8_t_array(buf, 8, data, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAN_FRAME, buf, MAVLINK_MSG_ID_CAN_FRAME_MIN_LEN, MAVLINK_MSG_ID_CAN_FRAME_LEN, MAVLINK_MSG_ID_CAN_FRAME_CRC);
#else
    mavlink_can_frame_t *packet = (mavlink_can_frame_t *)msgbuf;
    packet->id = id;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->bus = bus;
    packet->len = len;
    mav_array_memcpy(packet->data, data, sizeof(uint8_t)*8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAN_FRAME, (const char *)packet, MAVLINK_MSG_ID_CAN_FRAME_MIN_LEN, MAVLINK_MSG_ID_CAN_FRAME_LEN, MAVLINK_MSG_ID_CAN_FRAME_CRC);
#endif
}
#endif

#endif

// MESSAGE CAN_FRAME UNPACKING


/**
 * @brief Get field target_system from can_frame message
 *
 * @return  System ID.
 */
static inline uint8_t mavlink_msg_can_frame_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field target_component from can_frame message
 *
 * @return  Component ID.
 */
static inline uint8_t mavlink_msg_can_frame_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field bus from can_frame message
 *
 * @return  Bus number
 */
static inline uint8_t mavlink_msg_can_frame_get_bus(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field len from can_frame message
 *
 * @return  Frame length
 */
static inline uint8_t mavlink_msg_can_frame_get_len(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field id from can_frame message
 *
 * @return  Frame ID
 */
static inline uint32_t mavlink_msg_can_frame_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field data from can_frame message
 *
 * @return  Frame data
 */
static inline uint16_t mavlink_msg_can_frame_get_data(const mavlink_message_t* msg, uint8_t *data)
{
    return _MAV_RETURN_uint8_t_array(msg, data, 8,  8);
}

/**
 * @brief Decode a can_frame message into a struct
 *
 * @param msg The message to decode
 * @param can_frame C-struct to decode the message contents into
 */
static inline void mavlink_msg_can_frame_decode(const mavlink_message_t* msg, mavlink_can_frame_t* can_frame)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    can_frame->id = mavlink_msg_can_frame_get_id(msg);
    can_frame->target_system = mavlink_msg_can_frame_get_target_system(msg);
    can_frame->target_component = mavlink_msg_can_frame_get_target_component(msg);
    can_frame->bus = mavlink_msg_can_frame_get_bus(msg);
    can_frame->len = mavlink_msg_can_frame_get_len(msg);
    mavlink_msg_can_frame_get_data(msg, can_frame->data);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CAN_FRAME_LEN? msg->len : MAVLINK_MSG_ID_CAN_FRAME_LEN;
        memset(can_frame, 0, MAVLINK_MSG_ID_CAN_FRAME_LEN);
    memcpy(can_frame, _MAV_PAYLOAD(msg), len);
#endif
}
