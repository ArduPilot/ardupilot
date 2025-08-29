#pragma once
// MESSAGE CAN_FILTER_MODIFY PACKING

#define MAVLINK_MSG_ID_CAN_FILTER_MODIFY 388


typedef struct __mavlink_can_filter_modify_t {
 uint16_t ids[16]; /*<  filter IDs, length num_ids*/
 uint8_t target_system; /*<  System ID.*/
 uint8_t target_component; /*<  Component ID.*/
 uint8_t bus; /*<  bus number*/
 uint8_t operation; /*<  what operation to perform on the filter list. See CAN_FILTER_OP enum.*/
 uint8_t num_ids; /*<  number of IDs in filter list*/
} mavlink_can_filter_modify_t;

#define MAVLINK_MSG_ID_CAN_FILTER_MODIFY_LEN 37
#define MAVLINK_MSG_ID_CAN_FILTER_MODIFY_MIN_LEN 37
#define MAVLINK_MSG_ID_388_LEN 37
#define MAVLINK_MSG_ID_388_MIN_LEN 37

#define MAVLINK_MSG_ID_CAN_FILTER_MODIFY_CRC 8
#define MAVLINK_MSG_ID_388_CRC 8

#define MAVLINK_MSG_CAN_FILTER_MODIFY_FIELD_IDS_LEN 16

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CAN_FILTER_MODIFY { \
    388, \
    "CAN_FILTER_MODIFY", \
    6, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_can_filter_modify_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_can_filter_modify_t, target_component) }, \
         { "bus", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_can_filter_modify_t, bus) }, \
         { "operation", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_can_filter_modify_t, operation) }, \
         { "num_ids", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_can_filter_modify_t, num_ids) }, \
         { "ids", NULL, MAVLINK_TYPE_UINT16_T, 16, 0, offsetof(mavlink_can_filter_modify_t, ids) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CAN_FILTER_MODIFY { \
    "CAN_FILTER_MODIFY", \
    6, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_can_filter_modify_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_can_filter_modify_t, target_component) }, \
         { "bus", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_can_filter_modify_t, bus) }, \
         { "operation", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_can_filter_modify_t, operation) }, \
         { "num_ids", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_can_filter_modify_t, num_ids) }, \
         { "ids", NULL, MAVLINK_TYPE_UINT16_T, 16, 0, offsetof(mavlink_can_filter_modify_t, ids) }, \
         } \
}
#endif

/**
 * @brief Pack a can_filter_modify message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param bus  bus number
 * @param operation  what operation to perform on the filter list. See CAN_FILTER_OP enum.
 * @param num_ids  number of IDs in filter list
 * @param ids  filter IDs, length num_ids
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_can_filter_modify_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t bus, uint8_t operation, uint8_t num_ids, const uint16_t *ids)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAN_FILTER_MODIFY_LEN];
    _mav_put_uint8_t(buf, 32, target_system);
    _mav_put_uint8_t(buf, 33, target_component);
    _mav_put_uint8_t(buf, 34, bus);
    _mav_put_uint8_t(buf, 35, operation);
    _mav_put_uint8_t(buf, 36, num_ids);
    _mav_put_uint16_t_array(buf, 0, ids, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAN_FILTER_MODIFY_LEN);
#else
    mavlink_can_filter_modify_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.bus = bus;
    packet.operation = operation;
    packet.num_ids = num_ids;
    mav_array_memcpy(packet.ids, ids, sizeof(uint16_t)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAN_FILTER_MODIFY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAN_FILTER_MODIFY;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CAN_FILTER_MODIFY_MIN_LEN, MAVLINK_MSG_ID_CAN_FILTER_MODIFY_LEN, MAVLINK_MSG_ID_CAN_FILTER_MODIFY_CRC);
}

/**
 * @brief Pack a can_filter_modify message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param bus  bus number
 * @param operation  what operation to perform on the filter list. See CAN_FILTER_OP enum.
 * @param num_ids  number of IDs in filter list
 * @param ids  filter IDs, length num_ids
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_can_filter_modify_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t bus, uint8_t operation, uint8_t num_ids, const uint16_t *ids)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAN_FILTER_MODIFY_LEN];
    _mav_put_uint8_t(buf, 32, target_system);
    _mav_put_uint8_t(buf, 33, target_component);
    _mav_put_uint8_t(buf, 34, bus);
    _mav_put_uint8_t(buf, 35, operation);
    _mav_put_uint8_t(buf, 36, num_ids);
    _mav_put_uint16_t_array(buf, 0, ids, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAN_FILTER_MODIFY_LEN);
#else
    mavlink_can_filter_modify_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.bus = bus;
    packet.operation = operation;
    packet.num_ids = num_ids;
    mav_array_memcpy(packet.ids, ids, sizeof(uint16_t)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAN_FILTER_MODIFY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAN_FILTER_MODIFY;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CAN_FILTER_MODIFY_MIN_LEN, MAVLINK_MSG_ID_CAN_FILTER_MODIFY_LEN, MAVLINK_MSG_ID_CAN_FILTER_MODIFY_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CAN_FILTER_MODIFY_MIN_LEN, MAVLINK_MSG_ID_CAN_FILTER_MODIFY_LEN);
#endif
}

/**
 * @brief Pack a can_filter_modify message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param bus  bus number
 * @param operation  what operation to perform on the filter list. See CAN_FILTER_OP enum.
 * @param num_ids  number of IDs in filter list
 * @param ids  filter IDs, length num_ids
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_can_filter_modify_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint8_t bus,uint8_t operation,uint8_t num_ids,const uint16_t *ids)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAN_FILTER_MODIFY_LEN];
    _mav_put_uint8_t(buf, 32, target_system);
    _mav_put_uint8_t(buf, 33, target_component);
    _mav_put_uint8_t(buf, 34, bus);
    _mav_put_uint8_t(buf, 35, operation);
    _mav_put_uint8_t(buf, 36, num_ids);
    _mav_put_uint16_t_array(buf, 0, ids, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAN_FILTER_MODIFY_LEN);
#else
    mavlink_can_filter_modify_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.bus = bus;
    packet.operation = operation;
    packet.num_ids = num_ids;
    mav_array_memcpy(packet.ids, ids, sizeof(uint16_t)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAN_FILTER_MODIFY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAN_FILTER_MODIFY;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CAN_FILTER_MODIFY_MIN_LEN, MAVLINK_MSG_ID_CAN_FILTER_MODIFY_LEN, MAVLINK_MSG_ID_CAN_FILTER_MODIFY_CRC);
}

/**
 * @brief Encode a can_filter_modify struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param can_filter_modify C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_can_filter_modify_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_can_filter_modify_t* can_filter_modify)
{
    return mavlink_msg_can_filter_modify_pack(system_id, component_id, msg, can_filter_modify->target_system, can_filter_modify->target_component, can_filter_modify->bus, can_filter_modify->operation, can_filter_modify->num_ids, can_filter_modify->ids);
}

/**
 * @brief Encode a can_filter_modify struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param can_filter_modify C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_can_filter_modify_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_can_filter_modify_t* can_filter_modify)
{
    return mavlink_msg_can_filter_modify_pack_chan(system_id, component_id, chan, msg, can_filter_modify->target_system, can_filter_modify->target_component, can_filter_modify->bus, can_filter_modify->operation, can_filter_modify->num_ids, can_filter_modify->ids);
}

/**
 * @brief Encode a can_filter_modify struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param can_filter_modify C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_can_filter_modify_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_can_filter_modify_t* can_filter_modify)
{
    return mavlink_msg_can_filter_modify_pack_status(system_id, component_id, _status, msg,  can_filter_modify->target_system, can_filter_modify->target_component, can_filter_modify->bus, can_filter_modify->operation, can_filter_modify->num_ids, can_filter_modify->ids);
}

/**
 * @brief Send a can_filter_modify message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param bus  bus number
 * @param operation  what operation to perform on the filter list. See CAN_FILTER_OP enum.
 * @param num_ids  number of IDs in filter list
 * @param ids  filter IDs, length num_ids
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_can_filter_modify_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t bus, uint8_t operation, uint8_t num_ids, const uint16_t *ids)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAN_FILTER_MODIFY_LEN];
    _mav_put_uint8_t(buf, 32, target_system);
    _mav_put_uint8_t(buf, 33, target_component);
    _mav_put_uint8_t(buf, 34, bus);
    _mav_put_uint8_t(buf, 35, operation);
    _mav_put_uint8_t(buf, 36, num_ids);
    _mav_put_uint16_t_array(buf, 0, ids, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAN_FILTER_MODIFY, buf, MAVLINK_MSG_ID_CAN_FILTER_MODIFY_MIN_LEN, MAVLINK_MSG_ID_CAN_FILTER_MODIFY_LEN, MAVLINK_MSG_ID_CAN_FILTER_MODIFY_CRC);
#else
    mavlink_can_filter_modify_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.bus = bus;
    packet.operation = operation;
    packet.num_ids = num_ids;
    mav_array_memcpy(packet.ids, ids, sizeof(uint16_t)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAN_FILTER_MODIFY, (const char *)&packet, MAVLINK_MSG_ID_CAN_FILTER_MODIFY_MIN_LEN, MAVLINK_MSG_ID_CAN_FILTER_MODIFY_LEN, MAVLINK_MSG_ID_CAN_FILTER_MODIFY_CRC);
#endif
}

/**
 * @brief Send a can_filter_modify message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_can_filter_modify_send_struct(mavlink_channel_t chan, const mavlink_can_filter_modify_t* can_filter_modify)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_can_filter_modify_send(chan, can_filter_modify->target_system, can_filter_modify->target_component, can_filter_modify->bus, can_filter_modify->operation, can_filter_modify->num_ids, can_filter_modify->ids);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAN_FILTER_MODIFY, (const char *)can_filter_modify, MAVLINK_MSG_ID_CAN_FILTER_MODIFY_MIN_LEN, MAVLINK_MSG_ID_CAN_FILTER_MODIFY_LEN, MAVLINK_MSG_ID_CAN_FILTER_MODIFY_CRC);
#endif
}

#if MAVLINK_MSG_ID_CAN_FILTER_MODIFY_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_can_filter_modify_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t bus, uint8_t operation, uint8_t num_ids, const uint16_t *ids)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 32, target_system);
    _mav_put_uint8_t(buf, 33, target_component);
    _mav_put_uint8_t(buf, 34, bus);
    _mav_put_uint8_t(buf, 35, operation);
    _mav_put_uint8_t(buf, 36, num_ids);
    _mav_put_uint16_t_array(buf, 0, ids, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAN_FILTER_MODIFY, buf, MAVLINK_MSG_ID_CAN_FILTER_MODIFY_MIN_LEN, MAVLINK_MSG_ID_CAN_FILTER_MODIFY_LEN, MAVLINK_MSG_ID_CAN_FILTER_MODIFY_CRC);
#else
    mavlink_can_filter_modify_t *packet = (mavlink_can_filter_modify_t *)msgbuf;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->bus = bus;
    packet->operation = operation;
    packet->num_ids = num_ids;
    mav_array_memcpy(packet->ids, ids, sizeof(uint16_t)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAN_FILTER_MODIFY, (const char *)packet, MAVLINK_MSG_ID_CAN_FILTER_MODIFY_MIN_LEN, MAVLINK_MSG_ID_CAN_FILTER_MODIFY_LEN, MAVLINK_MSG_ID_CAN_FILTER_MODIFY_CRC);
#endif
}
#endif

#endif

// MESSAGE CAN_FILTER_MODIFY UNPACKING


/**
 * @brief Get field target_system from can_filter_modify message
 *
 * @return  System ID.
 */
static inline uint8_t mavlink_msg_can_filter_modify_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field target_component from can_filter_modify message
 *
 * @return  Component ID.
 */
static inline uint8_t mavlink_msg_can_filter_modify_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  33);
}

/**
 * @brief Get field bus from can_filter_modify message
 *
 * @return  bus number
 */
static inline uint8_t mavlink_msg_can_filter_modify_get_bus(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field operation from can_filter_modify message
 *
 * @return  what operation to perform on the filter list. See CAN_FILTER_OP enum.
 */
static inline uint8_t mavlink_msg_can_filter_modify_get_operation(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  35);
}

/**
 * @brief Get field num_ids from can_filter_modify message
 *
 * @return  number of IDs in filter list
 */
static inline uint8_t mavlink_msg_can_filter_modify_get_num_ids(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Get field ids from can_filter_modify message
 *
 * @return  filter IDs, length num_ids
 */
static inline uint16_t mavlink_msg_can_filter_modify_get_ids(const mavlink_message_t* msg, uint16_t *ids)
{
    return _MAV_RETURN_uint16_t_array(msg, ids, 16,  0);
}

/**
 * @brief Decode a can_filter_modify message into a struct
 *
 * @param msg The message to decode
 * @param can_filter_modify C-struct to decode the message contents into
 */
static inline void mavlink_msg_can_filter_modify_decode(const mavlink_message_t* msg, mavlink_can_filter_modify_t* can_filter_modify)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_can_filter_modify_get_ids(msg, can_filter_modify->ids);
    can_filter_modify->target_system = mavlink_msg_can_filter_modify_get_target_system(msg);
    can_filter_modify->target_component = mavlink_msg_can_filter_modify_get_target_component(msg);
    can_filter_modify->bus = mavlink_msg_can_filter_modify_get_bus(msg);
    can_filter_modify->operation = mavlink_msg_can_filter_modify_get_operation(msg);
    can_filter_modify->num_ids = mavlink_msg_can_filter_modify_get_num_ids(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CAN_FILTER_MODIFY_LEN? msg->len : MAVLINK_MSG_ID_CAN_FILTER_MODIFY_LEN;
        memset(can_filter_modify, 0, MAVLINK_MSG_ID_CAN_FILTER_MODIFY_LEN);
    memcpy(can_filter_modify, _MAV_PAYLOAD(msg), len);
#endif
}
