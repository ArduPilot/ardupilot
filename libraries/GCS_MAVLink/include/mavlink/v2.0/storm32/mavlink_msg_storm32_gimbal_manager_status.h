#pragma once
// MESSAGE STORM32_GIMBAL_MANAGER_STATUS PACKING

#define MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS 60011


typedef struct __mavlink_storm32_gimbal_manager_status_t {
 uint16_t device_flags; /*<  Gimbal device flags currently applied.*/
 uint16_t manager_flags; /*<  Gimbal manager flags currently applied.*/
 uint8_t gimbal_id; /*<  Gimbal ID (component ID or 1-6 for non-MAVLink gimbal) that this gimbal manager is responsible for.*/
 uint8_t supervisor; /*<  Client who is currently supervisor (0 = none).*/
 uint8_t profile; /*<  Profile currently applied (0 = default).*/
} mavlink_storm32_gimbal_manager_status_t;

#define MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_LEN 7
#define MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_MIN_LEN 7
#define MAVLINK_MSG_ID_60011_LEN 7
#define MAVLINK_MSG_ID_60011_MIN_LEN 7

#define MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_CRC 183
#define MAVLINK_MSG_ID_60011_CRC 183



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_STORM32_GIMBAL_MANAGER_STATUS { \
    60011, \
    "STORM32_GIMBAL_MANAGER_STATUS", \
    5, \
    {  { "gimbal_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_storm32_gimbal_manager_status_t, gimbal_id) }, \
         { "supervisor", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_storm32_gimbal_manager_status_t, supervisor) }, \
         { "device_flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_storm32_gimbal_manager_status_t, device_flags) }, \
         { "manager_flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_storm32_gimbal_manager_status_t, manager_flags) }, \
         { "profile", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_storm32_gimbal_manager_status_t, profile) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_STORM32_GIMBAL_MANAGER_STATUS { \
    "STORM32_GIMBAL_MANAGER_STATUS", \
    5, \
    {  { "gimbal_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_storm32_gimbal_manager_status_t, gimbal_id) }, \
         { "supervisor", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_storm32_gimbal_manager_status_t, supervisor) }, \
         { "device_flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_storm32_gimbal_manager_status_t, device_flags) }, \
         { "manager_flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_storm32_gimbal_manager_status_t, manager_flags) }, \
         { "profile", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_storm32_gimbal_manager_status_t, profile) }, \
         } \
}
#endif

/**
 * @brief Pack a storm32_gimbal_manager_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param gimbal_id  Gimbal ID (component ID or 1-6 for non-MAVLink gimbal) that this gimbal manager is responsible for.
 * @param supervisor  Client who is currently supervisor (0 = none).
 * @param device_flags  Gimbal device flags currently applied.
 * @param manager_flags  Gimbal manager flags currently applied.
 * @param profile  Profile currently applied (0 = default).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t gimbal_id, uint8_t supervisor, uint16_t device_flags, uint16_t manager_flags, uint8_t profile)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, device_flags);
    _mav_put_uint16_t(buf, 2, manager_flags);
    _mav_put_uint8_t(buf, 4, gimbal_id);
    _mav_put_uint8_t(buf, 5, supervisor);
    _mav_put_uint8_t(buf, 6, profile);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_LEN);
#else
    mavlink_storm32_gimbal_manager_status_t packet;
    packet.device_flags = device_flags;
    packet.manager_flags = manager_flags;
    packet.gimbal_id = gimbal_id;
    packet.supervisor = supervisor;
    packet.profile = profile;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_CRC);
}

/**
 * @brief Pack a storm32_gimbal_manager_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param gimbal_id  Gimbal ID (component ID or 1-6 for non-MAVLink gimbal) that this gimbal manager is responsible for.
 * @param supervisor  Client who is currently supervisor (0 = none).
 * @param device_flags  Gimbal device flags currently applied.
 * @param manager_flags  Gimbal manager flags currently applied.
 * @param profile  Profile currently applied (0 = default).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t gimbal_id, uint8_t supervisor, uint16_t device_flags, uint16_t manager_flags, uint8_t profile)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, device_flags);
    _mav_put_uint16_t(buf, 2, manager_flags);
    _mav_put_uint8_t(buf, 4, gimbal_id);
    _mav_put_uint8_t(buf, 5, supervisor);
    _mav_put_uint8_t(buf, 6, profile);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_LEN);
#else
    mavlink_storm32_gimbal_manager_status_t packet;
    packet.device_flags = device_flags;
    packet.manager_flags = manager_flags;
    packet.gimbal_id = gimbal_id;
    packet.supervisor = supervisor;
    packet.profile = profile;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_LEN);
#endif
}

/**
 * @brief Pack a storm32_gimbal_manager_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_id  Gimbal ID (component ID or 1-6 for non-MAVLink gimbal) that this gimbal manager is responsible for.
 * @param supervisor  Client who is currently supervisor (0 = none).
 * @param device_flags  Gimbal device flags currently applied.
 * @param manager_flags  Gimbal manager flags currently applied.
 * @param profile  Profile currently applied (0 = default).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t gimbal_id,uint8_t supervisor,uint16_t device_flags,uint16_t manager_flags,uint8_t profile)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, device_flags);
    _mav_put_uint16_t(buf, 2, manager_flags);
    _mav_put_uint8_t(buf, 4, gimbal_id);
    _mav_put_uint8_t(buf, 5, supervisor);
    _mav_put_uint8_t(buf, 6, profile);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_LEN);
#else
    mavlink_storm32_gimbal_manager_status_t packet;
    packet.device_flags = device_flags;
    packet.manager_flags = manager_flags;
    packet.gimbal_id = gimbal_id;
    packet.supervisor = supervisor;
    packet.profile = profile;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_CRC);
}

/**
 * @brief Encode a storm32_gimbal_manager_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param storm32_gimbal_manager_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_storm32_gimbal_manager_status_t* storm32_gimbal_manager_status)
{
    return mavlink_msg_storm32_gimbal_manager_status_pack(system_id, component_id, msg, storm32_gimbal_manager_status->gimbal_id, storm32_gimbal_manager_status->supervisor, storm32_gimbal_manager_status->device_flags, storm32_gimbal_manager_status->manager_flags, storm32_gimbal_manager_status->profile);
}

/**
 * @brief Encode a storm32_gimbal_manager_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param storm32_gimbal_manager_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_storm32_gimbal_manager_status_t* storm32_gimbal_manager_status)
{
    return mavlink_msg_storm32_gimbal_manager_status_pack_chan(system_id, component_id, chan, msg, storm32_gimbal_manager_status->gimbal_id, storm32_gimbal_manager_status->supervisor, storm32_gimbal_manager_status->device_flags, storm32_gimbal_manager_status->manager_flags, storm32_gimbal_manager_status->profile);
}

/**
 * @brief Encode a storm32_gimbal_manager_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param storm32_gimbal_manager_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_storm32_gimbal_manager_status_t* storm32_gimbal_manager_status)
{
    return mavlink_msg_storm32_gimbal_manager_status_pack_status(system_id, component_id, _status, msg,  storm32_gimbal_manager_status->gimbal_id, storm32_gimbal_manager_status->supervisor, storm32_gimbal_manager_status->device_flags, storm32_gimbal_manager_status->manager_flags, storm32_gimbal_manager_status->profile);
}

/**
 * @brief Send a storm32_gimbal_manager_status message
 * @param chan MAVLink channel to send the message
 *
 * @param gimbal_id  Gimbal ID (component ID or 1-6 for non-MAVLink gimbal) that this gimbal manager is responsible for.
 * @param supervisor  Client who is currently supervisor (0 = none).
 * @param device_flags  Gimbal device flags currently applied.
 * @param manager_flags  Gimbal manager flags currently applied.
 * @param profile  Profile currently applied (0 = default).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_storm32_gimbal_manager_status_send(mavlink_channel_t chan, uint8_t gimbal_id, uint8_t supervisor, uint16_t device_flags, uint16_t manager_flags, uint8_t profile)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, device_flags);
    _mav_put_uint16_t(buf, 2, manager_flags);
    _mav_put_uint8_t(buf, 4, gimbal_id);
    _mav_put_uint8_t(buf, 5, supervisor);
    _mav_put_uint8_t(buf, 6, profile);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS, buf, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_CRC);
#else
    mavlink_storm32_gimbal_manager_status_t packet;
    packet.device_flags = device_flags;
    packet.manager_flags = manager_flags;
    packet.gimbal_id = gimbal_id;
    packet.supervisor = supervisor;
    packet.profile = profile;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS, (const char *)&packet, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_CRC);
#endif
}

/**
 * @brief Send a storm32_gimbal_manager_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_storm32_gimbal_manager_status_send_struct(mavlink_channel_t chan, const mavlink_storm32_gimbal_manager_status_t* storm32_gimbal_manager_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_storm32_gimbal_manager_status_send(chan, storm32_gimbal_manager_status->gimbal_id, storm32_gimbal_manager_status->supervisor, storm32_gimbal_manager_status->device_flags, storm32_gimbal_manager_status->manager_flags, storm32_gimbal_manager_status->profile);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS, (const char *)storm32_gimbal_manager_status, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_storm32_gimbal_manager_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t gimbal_id, uint8_t supervisor, uint16_t device_flags, uint16_t manager_flags, uint8_t profile)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, device_flags);
    _mav_put_uint16_t(buf, 2, manager_flags);
    _mav_put_uint8_t(buf, 4, gimbal_id);
    _mav_put_uint8_t(buf, 5, supervisor);
    _mav_put_uint8_t(buf, 6, profile);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS, buf, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_CRC);
#else
    mavlink_storm32_gimbal_manager_status_t *packet = (mavlink_storm32_gimbal_manager_status_t *)msgbuf;
    packet->device_flags = device_flags;
    packet->manager_flags = manager_flags;
    packet->gimbal_id = gimbal_id;
    packet->supervisor = supervisor;
    packet->profile = profile;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS, (const char *)packet, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE STORM32_GIMBAL_MANAGER_STATUS UNPACKING


/**
 * @brief Get field gimbal_id from storm32_gimbal_manager_status message
 *
 * @return  Gimbal ID (component ID or 1-6 for non-MAVLink gimbal) that this gimbal manager is responsible for.
 */
static inline uint8_t mavlink_msg_storm32_gimbal_manager_status_get_gimbal_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field supervisor from storm32_gimbal_manager_status message
 *
 * @return  Client who is currently supervisor (0 = none).
 */
static inline uint8_t mavlink_msg_storm32_gimbal_manager_status_get_supervisor(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field device_flags from storm32_gimbal_manager_status message
 *
 * @return  Gimbal device flags currently applied.
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_status_get_device_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field manager_flags from storm32_gimbal_manager_status message
 *
 * @return  Gimbal manager flags currently applied.
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_status_get_manager_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field profile from storm32_gimbal_manager_status message
 *
 * @return  Profile currently applied (0 = default).
 */
static inline uint8_t mavlink_msg_storm32_gimbal_manager_status_get_profile(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Decode a storm32_gimbal_manager_status message into a struct
 *
 * @param msg The message to decode
 * @param storm32_gimbal_manager_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_storm32_gimbal_manager_status_decode(const mavlink_message_t* msg, mavlink_storm32_gimbal_manager_status_t* storm32_gimbal_manager_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    storm32_gimbal_manager_status->device_flags = mavlink_msg_storm32_gimbal_manager_status_get_device_flags(msg);
    storm32_gimbal_manager_status->manager_flags = mavlink_msg_storm32_gimbal_manager_status_get_manager_flags(msg);
    storm32_gimbal_manager_status->gimbal_id = mavlink_msg_storm32_gimbal_manager_status_get_gimbal_id(msg);
    storm32_gimbal_manager_status->supervisor = mavlink_msg_storm32_gimbal_manager_status_get_supervisor(msg);
    storm32_gimbal_manager_status->profile = mavlink_msg_storm32_gimbal_manager_status_get_profile(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_LEN? msg->len : MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_LEN;
        memset(storm32_gimbal_manager_status, 0, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS_LEN);
    memcpy(storm32_gimbal_manager_status, _MAV_PAYLOAD(msg), len);
#endif
}
