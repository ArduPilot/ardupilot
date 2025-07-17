#pragma once
// MESSAGE CANFD_FRAME PACKING

#define MAVLINK_MSG_ID_CANFD_FRAME 387


typedef struct __mavlink_canfd_frame_t {
 uint32_t id; /*<  Frame ID*/
 uint8_t target_system; /*<  System ID.*/
 uint8_t target_component; /*<  Component ID.*/
 uint8_t bus; /*<  bus number*/
 uint8_t len; /*<  Frame length*/
 uint8_t data[64]; /*<  Frame data*/
} mavlink_canfd_frame_t;

#define MAVLINK_MSG_ID_CANFD_FRAME_LEN 72
#define MAVLINK_MSG_ID_CANFD_FRAME_MIN_LEN 72
#define MAVLINK_MSG_ID_387_LEN 72
#define MAVLINK_MSG_ID_387_MIN_LEN 72

#define MAVLINK_MSG_ID_CANFD_FRAME_CRC 4
#define MAVLINK_MSG_ID_387_CRC 4

#define MAVLINK_MSG_CANFD_FRAME_FIELD_DATA_LEN 64

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CANFD_FRAME { \
    387, \
    "CANFD_FRAME", \
    6, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_canfd_frame_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_canfd_frame_t, target_component) }, \
         { "bus", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_canfd_frame_t, bus) }, \
         { "len", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_canfd_frame_t, len) }, \
         { "id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_canfd_frame_t, id) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 64, 8, offsetof(mavlink_canfd_frame_t, data) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CANFD_FRAME { \
    "CANFD_FRAME", \
    6, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_canfd_frame_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_canfd_frame_t, target_component) }, \
         { "bus", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_canfd_frame_t, bus) }, \
         { "len", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_canfd_frame_t, len) }, \
         { "id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_canfd_frame_t, id) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 64, 8, offsetof(mavlink_canfd_frame_t, data) }, \
         } \
}
#endif

/**
 * @brief Pack a canfd_frame message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param bus  bus number
 * @param len  Frame length
 * @param id  Frame ID
 * @param data  Frame data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_canfd_frame_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t bus, uint8_t len, uint32_t id, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CANFD_FRAME_LEN];
    _mav_put_uint32_t(buf, 0, id);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 6, bus);
    _mav_put_uint8_t(buf, 7, len);
    _mav_put_uint8_t_array(buf, 8, data, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CANFD_FRAME_LEN);
#else
    mavlink_canfd_frame_t packet;
    packet.id = id;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.bus = bus;
    packet.len = len;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CANFD_FRAME_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CANFD_FRAME;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CANFD_FRAME_MIN_LEN, MAVLINK_MSG_ID_CANFD_FRAME_LEN, MAVLINK_MSG_ID_CANFD_FRAME_CRC);
}

/**
 * @brief Pack a canfd_frame message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param bus  bus number
 * @param len  Frame length
 * @param id  Frame ID
 * @param data  Frame data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_canfd_frame_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t bus, uint8_t len, uint32_t id, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CANFD_FRAME_LEN];
    _mav_put_uint32_t(buf, 0, id);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 6, bus);
    _mav_put_uint8_t(buf, 7, len);
    _mav_put_uint8_t_array(buf, 8, data, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CANFD_FRAME_LEN);
#else
    mavlink_canfd_frame_t packet;
    packet.id = id;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.bus = bus;
    packet.len = len;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CANFD_FRAME_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CANFD_FRAME;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CANFD_FRAME_MIN_LEN, MAVLINK_MSG_ID_CANFD_FRAME_LEN, MAVLINK_MSG_ID_CANFD_FRAME_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CANFD_FRAME_MIN_LEN, MAVLINK_MSG_ID_CANFD_FRAME_LEN);
#endif
}

/**
 * @brief Pack a canfd_frame message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param bus  bus number
 * @param len  Frame length
 * @param id  Frame ID
 * @param data  Frame data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_canfd_frame_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint8_t bus,uint8_t len,uint32_t id,const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CANFD_FRAME_LEN];
    _mav_put_uint32_t(buf, 0, id);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 6, bus);
    _mav_put_uint8_t(buf, 7, len);
    _mav_put_uint8_t_array(buf, 8, data, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CANFD_FRAME_LEN);
#else
    mavlink_canfd_frame_t packet;
    packet.id = id;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.bus = bus;
    packet.len = len;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CANFD_FRAME_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CANFD_FRAME;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CANFD_FRAME_MIN_LEN, MAVLINK_MSG_ID_CANFD_FRAME_LEN, MAVLINK_MSG_ID_CANFD_FRAME_CRC);
}

/**
 * @brief Encode a canfd_frame struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param canfd_frame C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_canfd_frame_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_canfd_frame_t* canfd_frame)
{
    return mavlink_msg_canfd_frame_pack(system_id, component_id, msg, canfd_frame->target_system, canfd_frame->target_component, canfd_frame->bus, canfd_frame->len, canfd_frame->id, canfd_frame->data);
}

/**
 * @brief Encode a canfd_frame struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param canfd_frame C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_canfd_frame_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_canfd_frame_t* canfd_frame)
{
    return mavlink_msg_canfd_frame_pack_chan(system_id, component_id, chan, msg, canfd_frame->target_system, canfd_frame->target_component, canfd_frame->bus, canfd_frame->len, canfd_frame->id, canfd_frame->data);
}

/**
 * @brief Encode a canfd_frame struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param canfd_frame C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_canfd_frame_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_canfd_frame_t* canfd_frame)
{
    return mavlink_msg_canfd_frame_pack_status(system_id, component_id, _status, msg,  canfd_frame->target_system, canfd_frame->target_component, canfd_frame->bus, canfd_frame->len, canfd_frame->id, canfd_frame->data);
}

/**
 * @brief Send a canfd_frame message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param bus  bus number
 * @param len  Frame length
 * @param id  Frame ID
 * @param data  Frame data
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_canfd_frame_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t bus, uint8_t len, uint32_t id, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CANFD_FRAME_LEN];
    _mav_put_uint32_t(buf, 0, id);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 6, bus);
    _mav_put_uint8_t(buf, 7, len);
    _mav_put_uint8_t_array(buf, 8, data, 64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CANFD_FRAME, buf, MAVLINK_MSG_ID_CANFD_FRAME_MIN_LEN, MAVLINK_MSG_ID_CANFD_FRAME_LEN, MAVLINK_MSG_ID_CANFD_FRAME_CRC);
#else
    mavlink_canfd_frame_t packet;
    packet.id = id;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.bus = bus;
    packet.len = len;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CANFD_FRAME, (const char *)&packet, MAVLINK_MSG_ID_CANFD_FRAME_MIN_LEN, MAVLINK_MSG_ID_CANFD_FRAME_LEN, MAVLINK_MSG_ID_CANFD_FRAME_CRC);
#endif
}

/**
 * @brief Send a canfd_frame message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_canfd_frame_send_struct(mavlink_channel_t chan, const mavlink_canfd_frame_t* canfd_frame)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_canfd_frame_send(chan, canfd_frame->target_system, canfd_frame->target_component, canfd_frame->bus, canfd_frame->len, canfd_frame->id, canfd_frame->data);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CANFD_FRAME, (const char *)canfd_frame, MAVLINK_MSG_ID_CANFD_FRAME_MIN_LEN, MAVLINK_MSG_ID_CANFD_FRAME_LEN, MAVLINK_MSG_ID_CANFD_FRAME_CRC);
#endif
}

#if MAVLINK_MSG_ID_CANFD_FRAME_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_canfd_frame_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t bus, uint8_t len, uint32_t id, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, id);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 6, bus);
    _mav_put_uint8_t(buf, 7, len);
    _mav_put_uint8_t_array(buf, 8, data, 64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CANFD_FRAME, buf, MAVLINK_MSG_ID_CANFD_FRAME_MIN_LEN, MAVLINK_MSG_ID_CANFD_FRAME_LEN, MAVLINK_MSG_ID_CANFD_FRAME_CRC);
#else
    mavlink_canfd_frame_t *packet = (mavlink_canfd_frame_t *)msgbuf;
    packet->id = id;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->bus = bus;
    packet->len = len;
    mav_array_memcpy(packet->data, data, sizeof(uint8_t)*64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CANFD_FRAME, (const char *)packet, MAVLINK_MSG_ID_CANFD_FRAME_MIN_LEN, MAVLINK_MSG_ID_CANFD_FRAME_LEN, MAVLINK_MSG_ID_CANFD_FRAME_CRC);
#endif
}
#endif

#endif

// MESSAGE CANFD_FRAME UNPACKING


/**
 * @brief Get field target_system from canfd_frame message
 *
 * @return  System ID.
 */
static inline uint8_t mavlink_msg_canfd_frame_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field target_component from canfd_frame message
 *
 * @return  Component ID.
 */
static inline uint8_t mavlink_msg_canfd_frame_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field bus from canfd_frame message
 *
 * @return  bus number
 */
static inline uint8_t mavlink_msg_canfd_frame_get_bus(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field len from canfd_frame message
 *
 * @return  Frame length
 */
static inline uint8_t mavlink_msg_canfd_frame_get_len(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field id from canfd_frame message
 *
 * @return  Frame ID
 */
static inline uint32_t mavlink_msg_canfd_frame_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field data from canfd_frame message
 *
 * @return  Frame data
 */
static inline uint16_t mavlink_msg_canfd_frame_get_data(const mavlink_message_t* msg, uint8_t *data)
{
    return _MAV_RETURN_uint8_t_array(msg, data, 64,  8);
}

/**
 * @brief Decode a canfd_frame message into a struct
 *
 * @param msg The message to decode
 * @param canfd_frame C-struct to decode the message contents into
 */
static inline void mavlink_msg_canfd_frame_decode(const mavlink_message_t* msg, mavlink_canfd_frame_t* canfd_frame)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    canfd_frame->id = mavlink_msg_canfd_frame_get_id(msg);
    canfd_frame->target_system = mavlink_msg_canfd_frame_get_target_system(msg);
    canfd_frame->target_component = mavlink_msg_canfd_frame_get_target_component(msg);
    canfd_frame->bus = mavlink_msg_canfd_frame_get_bus(msg);
    canfd_frame->len = mavlink_msg_canfd_frame_get_len(msg);
    mavlink_msg_canfd_frame_get_data(msg, canfd_frame->data);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CANFD_FRAME_LEN? msg->len : MAVLINK_MSG_ID_CANFD_FRAME_LEN;
        memset(canfd_frame, 0, MAVLINK_MSG_ID_CANFD_FRAME_LEN);
    memcpy(canfd_frame, _MAV_PAYLOAD(msg), len);
#endif
}
