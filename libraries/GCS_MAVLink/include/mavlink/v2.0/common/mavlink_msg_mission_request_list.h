#pragma once
// MESSAGE MISSION_REQUEST_LIST PACKING

#define MAVLINK_MSG_ID_MISSION_REQUEST_LIST 43


typedef struct __mavlink_mission_request_list_t {
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
 uint8_t mission_type; /*<  Mission type.*/
} mavlink_mission_request_list_t;

#define MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN 3
#define MAVLINK_MSG_ID_MISSION_REQUEST_LIST_MIN_LEN 2
#define MAVLINK_MSG_ID_43_LEN 3
#define MAVLINK_MSG_ID_43_MIN_LEN 2

#define MAVLINK_MSG_ID_MISSION_REQUEST_LIST_CRC 132
#define MAVLINK_MSG_ID_43_CRC 132



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MISSION_REQUEST_LIST { \
    43, \
    "MISSION_REQUEST_LIST", \
    3, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mission_request_list_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_mission_request_list_t, target_component) }, \
         { "mission_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_mission_request_list_t, mission_type) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MISSION_REQUEST_LIST { \
    "MISSION_REQUEST_LIST", \
    3, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mission_request_list_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_mission_request_list_t, target_component) }, \
         { "mission_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_mission_request_list_t, mission_type) }, \
         } \
}
#endif

/**
 * @brief Pack a mission_request_list message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param mission_type  Mission type.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_request_list_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t mission_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, mission_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN);
#else
    mavlink_mission_request_list_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.mission_type = mission_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MISSION_REQUEST_LIST;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MISSION_REQUEST_LIST_MIN_LEN, MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN, MAVLINK_MSG_ID_MISSION_REQUEST_LIST_CRC);
}

/**
 * @brief Pack a mission_request_list message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param mission_type  Mission type.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_request_list_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t mission_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, mission_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN);
#else
    mavlink_mission_request_list_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.mission_type = mission_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MISSION_REQUEST_LIST;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MISSION_REQUEST_LIST_MIN_LEN, MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN, MAVLINK_MSG_ID_MISSION_REQUEST_LIST_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MISSION_REQUEST_LIST_MIN_LEN, MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN);
#endif
}

/**
 * @brief Pack a mission_request_list message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param mission_type  Mission type.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_request_list_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint8_t mission_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, mission_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN);
#else
    mavlink_mission_request_list_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.mission_type = mission_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MISSION_REQUEST_LIST;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MISSION_REQUEST_LIST_MIN_LEN, MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN, MAVLINK_MSG_ID_MISSION_REQUEST_LIST_CRC);
}

/**
 * @brief Encode a mission_request_list struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_request_list C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_request_list_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mission_request_list_t* mission_request_list)
{
    return mavlink_msg_mission_request_list_pack(system_id, component_id, msg, mission_request_list->target_system, mission_request_list->target_component, mission_request_list->mission_type);
}

/**
 * @brief Encode a mission_request_list struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_request_list C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_request_list_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mission_request_list_t* mission_request_list)
{
    return mavlink_msg_mission_request_list_pack_chan(system_id, component_id, chan, msg, mission_request_list->target_system, mission_request_list->target_component, mission_request_list->mission_type);
}

/**
 * @brief Encode a mission_request_list struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param mission_request_list C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_request_list_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_mission_request_list_t* mission_request_list)
{
    return mavlink_msg_mission_request_list_pack_status(system_id, component_id, _status, msg,  mission_request_list->target_system, mission_request_list->target_component, mission_request_list->mission_type);
}

/**
 * @brief Send a mission_request_list message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param mission_type  Mission type.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mission_request_list_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t mission_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, mission_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_REQUEST_LIST, buf, MAVLINK_MSG_ID_MISSION_REQUEST_LIST_MIN_LEN, MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN, MAVLINK_MSG_ID_MISSION_REQUEST_LIST_CRC);
#else
    mavlink_mission_request_list_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.mission_type = mission_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_REQUEST_LIST, (const char *)&packet, MAVLINK_MSG_ID_MISSION_REQUEST_LIST_MIN_LEN, MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN, MAVLINK_MSG_ID_MISSION_REQUEST_LIST_CRC);
#endif
}

/**
 * @brief Send a mission_request_list message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mission_request_list_send_struct(mavlink_channel_t chan, const mavlink_mission_request_list_t* mission_request_list)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mission_request_list_send(chan, mission_request_list->target_system, mission_request_list->target_component, mission_request_list->mission_type);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_REQUEST_LIST, (const char *)mission_request_list, MAVLINK_MSG_ID_MISSION_REQUEST_LIST_MIN_LEN, MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN, MAVLINK_MSG_ID_MISSION_REQUEST_LIST_CRC);
#endif
}

#if MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mission_request_list_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t mission_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, mission_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_REQUEST_LIST, buf, MAVLINK_MSG_ID_MISSION_REQUEST_LIST_MIN_LEN, MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN, MAVLINK_MSG_ID_MISSION_REQUEST_LIST_CRC);
#else
    mavlink_mission_request_list_t *packet = (mavlink_mission_request_list_t *)msgbuf;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->mission_type = mission_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_REQUEST_LIST, (const char *)packet, MAVLINK_MSG_ID_MISSION_REQUEST_LIST_MIN_LEN, MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN, MAVLINK_MSG_ID_MISSION_REQUEST_LIST_CRC);
#endif
}
#endif

#endif

// MESSAGE MISSION_REQUEST_LIST UNPACKING


/**
 * @brief Get field target_system from mission_request_list message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_mission_request_list_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field target_component from mission_request_list message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_mission_request_list_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field mission_type from mission_request_list message
 *
 * @return  Mission type.
 */
static inline uint8_t mavlink_msg_mission_request_list_get_mission_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a mission_request_list message into a struct
 *
 * @param msg The message to decode
 * @param mission_request_list C-struct to decode the message contents into
 */
static inline void mavlink_msg_mission_request_list_decode(const mavlink_message_t* msg, mavlink_mission_request_list_t* mission_request_list)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mission_request_list->target_system = mavlink_msg_mission_request_list_get_target_system(msg);
    mission_request_list->target_component = mavlink_msg_mission_request_list_get_target_component(msg);
    mission_request_list->mission_type = mavlink_msg_mission_request_list_get_mission_type(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN? msg->len : MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN;
        memset(mission_request_list, 0, MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN);
    memcpy(mission_request_list, _MAV_PAYLOAD(msg), len);
#endif
}
