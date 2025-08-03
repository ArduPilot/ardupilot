#pragma once
// MESSAGE GIMBAL_MANAGER_STATUS PACKING

#define MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS 281


typedef struct __mavlink_gimbal_manager_status_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 uint32_t flags; /*<  High level gimbal manager flags currently applied.*/
 uint8_t gimbal_device_id; /*<  Gimbal device ID that this gimbal manager is responsible for. Component ID of gimbal device (or 1-6 for non-MAVLink gimbal).*/
 uint8_t primary_control_sysid; /*<  System ID of MAVLink component with primary control, 0 for none.*/
 uint8_t primary_control_compid; /*<  Component ID of MAVLink component with primary control, 0 for none.*/
 uint8_t secondary_control_sysid; /*<  System ID of MAVLink component with secondary control, 0 for none.*/
 uint8_t secondary_control_compid; /*<  Component ID of MAVLink component with secondary control, 0 for none.*/
} mavlink_gimbal_manager_status_t;

#define MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN 13
#define MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_MIN_LEN 13
#define MAVLINK_MSG_ID_281_LEN 13
#define MAVLINK_MSG_ID_281_MIN_LEN 13

#define MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_CRC 48
#define MAVLINK_MSG_ID_281_CRC 48



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GIMBAL_MANAGER_STATUS { \
    281, \
    "GIMBAL_MANAGER_STATUS", \
    7, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_gimbal_manager_status_t, time_boot_ms) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_gimbal_manager_status_t, flags) }, \
         { "gimbal_device_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_gimbal_manager_status_t, gimbal_device_id) }, \
         { "primary_control_sysid", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_gimbal_manager_status_t, primary_control_sysid) }, \
         { "primary_control_compid", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_gimbal_manager_status_t, primary_control_compid) }, \
         { "secondary_control_sysid", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_gimbal_manager_status_t, secondary_control_sysid) }, \
         { "secondary_control_compid", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_gimbal_manager_status_t, secondary_control_compid) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GIMBAL_MANAGER_STATUS { \
    "GIMBAL_MANAGER_STATUS", \
    7, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_gimbal_manager_status_t, time_boot_ms) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_gimbal_manager_status_t, flags) }, \
         { "gimbal_device_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_gimbal_manager_status_t, gimbal_device_id) }, \
         { "primary_control_sysid", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_gimbal_manager_status_t, primary_control_sysid) }, \
         { "primary_control_compid", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_gimbal_manager_status_t, primary_control_compid) }, \
         { "secondary_control_sysid", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_gimbal_manager_status_t, secondary_control_sysid) }, \
         { "secondary_control_compid", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_gimbal_manager_status_t, secondary_control_compid) }, \
         } \
}
#endif

/**
 * @brief Pack a gimbal_manager_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param flags  High level gimbal manager flags currently applied.
 * @param gimbal_device_id  Gimbal device ID that this gimbal manager is responsible for. Component ID of gimbal device (or 1-6 for non-MAVLink gimbal).
 * @param primary_control_sysid  System ID of MAVLink component with primary control, 0 for none.
 * @param primary_control_compid  Component ID of MAVLink component with primary control, 0 for none.
 * @param secondary_control_sysid  System ID of MAVLink component with secondary control, 0 for none.
 * @param secondary_control_compid  Component ID of MAVLink component with secondary control, 0 for none.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_manager_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint32_t flags, uint8_t gimbal_device_id, uint8_t primary_control_sysid, uint8_t primary_control_compid, uint8_t secondary_control_sysid, uint8_t secondary_control_compid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, flags);
    _mav_put_uint8_t(buf, 8, gimbal_device_id);
    _mav_put_uint8_t(buf, 9, primary_control_sysid);
    _mav_put_uint8_t(buf, 10, primary_control_compid);
    _mav_put_uint8_t(buf, 11, secondary_control_sysid);
    _mav_put_uint8_t(buf, 12, secondary_control_compid);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN);
#else
    mavlink_gimbal_manager_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.flags = flags;
    packet.gimbal_device_id = gimbal_device_id;
    packet.primary_control_sysid = primary_control_sysid;
    packet.primary_control_compid = primary_control_compid;
    packet.secondary_control_sysid = secondary_control_sysid;
    packet.secondary_control_compid = secondary_control_compid;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_CRC);
}

/**
 * @brief Pack a gimbal_manager_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param flags  High level gimbal manager flags currently applied.
 * @param gimbal_device_id  Gimbal device ID that this gimbal manager is responsible for. Component ID of gimbal device (or 1-6 for non-MAVLink gimbal).
 * @param primary_control_sysid  System ID of MAVLink component with primary control, 0 for none.
 * @param primary_control_compid  Component ID of MAVLink component with primary control, 0 for none.
 * @param secondary_control_sysid  System ID of MAVLink component with secondary control, 0 for none.
 * @param secondary_control_compid  Component ID of MAVLink component with secondary control, 0 for none.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_manager_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint32_t flags, uint8_t gimbal_device_id, uint8_t primary_control_sysid, uint8_t primary_control_compid, uint8_t secondary_control_sysid, uint8_t secondary_control_compid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, flags);
    _mav_put_uint8_t(buf, 8, gimbal_device_id);
    _mav_put_uint8_t(buf, 9, primary_control_sysid);
    _mav_put_uint8_t(buf, 10, primary_control_compid);
    _mav_put_uint8_t(buf, 11, secondary_control_sysid);
    _mav_put_uint8_t(buf, 12, secondary_control_compid);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN);
#else
    mavlink_gimbal_manager_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.flags = flags;
    packet.gimbal_device_id = gimbal_device_id;
    packet.primary_control_sysid = primary_control_sysid;
    packet.primary_control_compid = primary_control_compid;
    packet.secondary_control_sysid = secondary_control_sysid;
    packet.secondary_control_compid = secondary_control_compid;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN);
#endif
}

/**
 * @brief Pack a gimbal_manager_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param flags  High level gimbal manager flags currently applied.
 * @param gimbal_device_id  Gimbal device ID that this gimbal manager is responsible for. Component ID of gimbal device (or 1-6 for non-MAVLink gimbal).
 * @param primary_control_sysid  System ID of MAVLink component with primary control, 0 for none.
 * @param primary_control_compid  Component ID of MAVLink component with primary control, 0 for none.
 * @param secondary_control_sysid  System ID of MAVLink component with secondary control, 0 for none.
 * @param secondary_control_compid  Component ID of MAVLink component with secondary control, 0 for none.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_manager_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,uint32_t flags,uint8_t gimbal_device_id,uint8_t primary_control_sysid,uint8_t primary_control_compid,uint8_t secondary_control_sysid,uint8_t secondary_control_compid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, flags);
    _mav_put_uint8_t(buf, 8, gimbal_device_id);
    _mav_put_uint8_t(buf, 9, primary_control_sysid);
    _mav_put_uint8_t(buf, 10, primary_control_compid);
    _mav_put_uint8_t(buf, 11, secondary_control_sysid);
    _mav_put_uint8_t(buf, 12, secondary_control_compid);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN);
#else
    mavlink_gimbal_manager_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.flags = flags;
    packet.gimbal_device_id = gimbal_device_id;
    packet.primary_control_sysid = primary_control_sysid;
    packet.primary_control_compid = primary_control_compid;
    packet.secondary_control_sysid = secondary_control_sysid;
    packet.secondary_control_compid = secondary_control_compid;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_CRC);
}

/**
 * @brief Encode a gimbal_manager_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_manager_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_manager_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gimbal_manager_status_t* gimbal_manager_status)
{
    return mavlink_msg_gimbal_manager_status_pack(system_id, component_id, msg, gimbal_manager_status->time_boot_ms, gimbal_manager_status->flags, gimbal_manager_status->gimbal_device_id, gimbal_manager_status->primary_control_sysid, gimbal_manager_status->primary_control_compid, gimbal_manager_status->secondary_control_sysid, gimbal_manager_status->secondary_control_compid);
}

/**
 * @brief Encode a gimbal_manager_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_manager_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_manager_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gimbal_manager_status_t* gimbal_manager_status)
{
    return mavlink_msg_gimbal_manager_status_pack_chan(system_id, component_id, chan, msg, gimbal_manager_status->time_boot_ms, gimbal_manager_status->flags, gimbal_manager_status->gimbal_device_id, gimbal_manager_status->primary_control_sysid, gimbal_manager_status->primary_control_compid, gimbal_manager_status->secondary_control_sysid, gimbal_manager_status->secondary_control_compid);
}

/**
 * @brief Encode a gimbal_manager_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_manager_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_manager_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_gimbal_manager_status_t* gimbal_manager_status)
{
    return mavlink_msg_gimbal_manager_status_pack_status(system_id, component_id, _status, msg,  gimbal_manager_status->time_boot_ms, gimbal_manager_status->flags, gimbal_manager_status->gimbal_device_id, gimbal_manager_status->primary_control_sysid, gimbal_manager_status->primary_control_compid, gimbal_manager_status->secondary_control_sysid, gimbal_manager_status->secondary_control_compid);
}

/**
 * @brief Send a gimbal_manager_status message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param flags  High level gimbal manager flags currently applied.
 * @param gimbal_device_id  Gimbal device ID that this gimbal manager is responsible for. Component ID of gimbal device (or 1-6 for non-MAVLink gimbal).
 * @param primary_control_sysid  System ID of MAVLink component with primary control, 0 for none.
 * @param primary_control_compid  Component ID of MAVLink component with primary control, 0 for none.
 * @param secondary_control_sysid  System ID of MAVLink component with secondary control, 0 for none.
 * @param secondary_control_compid  Component ID of MAVLink component with secondary control, 0 for none.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gimbal_manager_status_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint32_t flags, uint8_t gimbal_device_id, uint8_t primary_control_sysid, uint8_t primary_control_compid, uint8_t secondary_control_sysid, uint8_t secondary_control_compid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, flags);
    _mav_put_uint8_t(buf, 8, gimbal_device_id);
    _mav_put_uint8_t(buf, 9, primary_control_sysid);
    _mav_put_uint8_t(buf, 10, primary_control_compid);
    _mav_put_uint8_t(buf, 11, secondary_control_sysid);
    _mav_put_uint8_t(buf, 12, secondary_control_compid);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS, buf, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_CRC);
#else
    mavlink_gimbal_manager_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.flags = flags;
    packet.gimbal_device_id = gimbal_device_id;
    packet.primary_control_sysid = primary_control_sysid;
    packet.primary_control_compid = primary_control_compid;
    packet.secondary_control_sysid = secondary_control_sysid;
    packet.secondary_control_compid = secondary_control_compid;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS, (const char *)&packet, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_CRC);
#endif
}

/**
 * @brief Send a gimbal_manager_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gimbal_manager_status_send_struct(mavlink_channel_t chan, const mavlink_gimbal_manager_status_t* gimbal_manager_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gimbal_manager_status_send(chan, gimbal_manager_status->time_boot_ms, gimbal_manager_status->flags, gimbal_manager_status->gimbal_device_id, gimbal_manager_status->primary_control_sysid, gimbal_manager_status->primary_control_compid, gimbal_manager_status->secondary_control_sysid, gimbal_manager_status->secondary_control_compid);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS, (const char *)gimbal_manager_status, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gimbal_manager_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint32_t flags, uint8_t gimbal_device_id, uint8_t primary_control_sysid, uint8_t primary_control_compid, uint8_t secondary_control_sysid, uint8_t secondary_control_compid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, flags);
    _mav_put_uint8_t(buf, 8, gimbal_device_id);
    _mav_put_uint8_t(buf, 9, primary_control_sysid);
    _mav_put_uint8_t(buf, 10, primary_control_compid);
    _mav_put_uint8_t(buf, 11, secondary_control_sysid);
    _mav_put_uint8_t(buf, 12, secondary_control_compid);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS, buf, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_CRC);
#else
    mavlink_gimbal_manager_status_t *packet = (mavlink_gimbal_manager_status_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->flags = flags;
    packet->gimbal_device_id = gimbal_device_id;
    packet->primary_control_sysid = primary_control_sysid;
    packet->primary_control_compid = primary_control_compid;
    packet->secondary_control_sysid = secondary_control_sysid;
    packet->secondary_control_compid = secondary_control_compid;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS, (const char *)packet, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE GIMBAL_MANAGER_STATUS UNPACKING


/**
 * @brief Get field time_boot_ms from gimbal_manager_status message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_gimbal_manager_status_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field flags from gimbal_manager_status message
 *
 * @return  High level gimbal manager flags currently applied.
 */
static inline uint32_t mavlink_msg_gimbal_manager_status_get_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field gimbal_device_id from gimbal_manager_status message
 *
 * @return  Gimbal device ID that this gimbal manager is responsible for. Component ID of gimbal device (or 1-6 for non-MAVLink gimbal).
 */
static inline uint8_t mavlink_msg_gimbal_manager_status_get_gimbal_device_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field primary_control_sysid from gimbal_manager_status message
 *
 * @return  System ID of MAVLink component with primary control, 0 for none.
 */
static inline uint8_t mavlink_msg_gimbal_manager_status_get_primary_control_sysid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field primary_control_compid from gimbal_manager_status message
 *
 * @return  Component ID of MAVLink component with primary control, 0 for none.
 */
static inline uint8_t mavlink_msg_gimbal_manager_status_get_primary_control_compid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field secondary_control_sysid from gimbal_manager_status message
 *
 * @return  System ID of MAVLink component with secondary control, 0 for none.
 */
static inline uint8_t mavlink_msg_gimbal_manager_status_get_secondary_control_sysid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field secondary_control_compid from gimbal_manager_status message
 *
 * @return  Component ID of MAVLink component with secondary control, 0 for none.
 */
static inline uint8_t mavlink_msg_gimbal_manager_status_get_secondary_control_compid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Decode a gimbal_manager_status message into a struct
 *
 * @param msg The message to decode
 * @param gimbal_manager_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_gimbal_manager_status_decode(const mavlink_message_t* msg, mavlink_gimbal_manager_status_t* gimbal_manager_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gimbal_manager_status->time_boot_ms = mavlink_msg_gimbal_manager_status_get_time_boot_ms(msg);
    gimbal_manager_status->flags = mavlink_msg_gimbal_manager_status_get_flags(msg);
    gimbal_manager_status->gimbal_device_id = mavlink_msg_gimbal_manager_status_get_gimbal_device_id(msg);
    gimbal_manager_status->primary_control_sysid = mavlink_msg_gimbal_manager_status_get_primary_control_sysid(msg);
    gimbal_manager_status->primary_control_compid = mavlink_msg_gimbal_manager_status_get_primary_control_compid(msg);
    gimbal_manager_status->secondary_control_sysid = mavlink_msg_gimbal_manager_status_get_secondary_control_sysid(msg);
    gimbal_manager_status->secondary_control_compid = mavlink_msg_gimbal_manager_status_get_secondary_control_compid(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN? msg->len : MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN;
        memset(gimbal_manager_status, 0, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN);
    memcpy(gimbal_manager_status, _MAV_PAYLOAD(msg), len);
#endif
}
