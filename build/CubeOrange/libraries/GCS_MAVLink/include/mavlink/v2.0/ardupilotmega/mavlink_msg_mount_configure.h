#pragma once
// MESSAGE MOUNT_CONFIGURE PACKING

#define MAVLINK_MSG_ID_MOUNT_CONFIGURE 156


typedef struct __mavlink_mount_configure_t {
 uint8_t target_system; /*<  System ID.*/
 uint8_t target_component; /*<  Component ID.*/
 uint8_t mount_mode; /*<  Mount operating mode.*/
 uint8_t stab_roll; /*<  (1 = yes, 0 = no).*/
 uint8_t stab_pitch; /*<  (1 = yes, 0 = no).*/
 uint8_t stab_yaw; /*<  (1 = yes, 0 = no).*/
} mavlink_mount_configure_t;

#define MAVLINK_MSG_ID_MOUNT_CONFIGURE_LEN 6
#define MAVLINK_MSG_ID_MOUNT_CONFIGURE_MIN_LEN 6
#define MAVLINK_MSG_ID_156_LEN 6
#define MAVLINK_MSG_ID_156_MIN_LEN 6

#define MAVLINK_MSG_ID_MOUNT_CONFIGURE_CRC 19
#define MAVLINK_MSG_ID_156_CRC 19



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MOUNT_CONFIGURE { \
    156, \
    "MOUNT_CONFIGURE", \
    6, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mount_configure_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_mount_configure_t, target_component) }, \
         { "mount_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_mount_configure_t, mount_mode) }, \
         { "stab_roll", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_mount_configure_t, stab_roll) }, \
         { "stab_pitch", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_mount_configure_t, stab_pitch) }, \
         { "stab_yaw", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_mount_configure_t, stab_yaw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MOUNT_CONFIGURE { \
    "MOUNT_CONFIGURE", \
    6, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mount_configure_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_mount_configure_t, target_component) }, \
         { "mount_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_mount_configure_t, mount_mode) }, \
         { "stab_roll", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_mount_configure_t, stab_roll) }, \
         { "stab_pitch", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_mount_configure_t, stab_pitch) }, \
         { "stab_yaw", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_mount_configure_t, stab_yaw) }, \
         } \
}
#endif

/**
 * @brief Pack a mount_configure message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param mount_mode  Mount operating mode.
 * @param stab_roll  (1 = yes, 0 = no).
 * @param stab_pitch  (1 = yes, 0 = no).
 * @param stab_yaw  (1 = yes, 0 = no).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mount_configure_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t mount_mode, uint8_t stab_roll, uint8_t stab_pitch, uint8_t stab_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOUNT_CONFIGURE_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, mount_mode);
    _mav_put_uint8_t(buf, 3, stab_roll);
    _mav_put_uint8_t(buf, 4, stab_pitch);
    _mav_put_uint8_t(buf, 5, stab_yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOUNT_CONFIGURE_LEN);
#else
    mavlink_mount_configure_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.mount_mode = mount_mode;
    packet.stab_roll = stab_roll;
    packet.stab_pitch = stab_pitch;
    packet.stab_yaw = stab_yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOUNT_CONFIGURE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOUNT_CONFIGURE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MOUNT_CONFIGURE_MIN_LEN, MAVLINK_MSG_ID_MOUNT_CONFIGURE_LEN, MAVLINK_MSG_ID_MOUNT_CONFIGURE_CRC);
}

/**
 * @brief Pack a mount_configure message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param mount_mode  Mount operating mode.
 * @param stab_roll  (1 = yes, 0 = no).
 * @param stab_pitch  (1 = yes, 0 = no).
 * @param stab_yaw  (1 = yes, 0 = no).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mount_configure_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint8_t mount_mode,uint8_t stab_roll,uint8_t stab_pitch,uint8_t stab_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOUNT_CONFIGURE_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, mount_mode);
    _mav_put_uint8_t(buf, 3, stab_roll);
    _mav_put_uint8_t(buf, 4, stab_pitch);
    _mav_put_uint8_t(buf, 5, stab_yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOUNT_CONFIGURE_LEN);
#else
    mavlink_mount_configure_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.mount_mode = mount_mode;
    packet.stab_roll = stab_roll;
    packet.stab_pitch = stab_pitch;
    packet.stab_yaw = stab_yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOUNT_CONFIGURE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOUNT_CONFIGURE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MOUNT_CONFIGURE_MIN_LEN, MAVLINK_MSG_ID_MOUNT_CONFIGURE_LEN, MAVLINK_MSG_ID_MOUNT_CONFIGURE_CRC);
}

/**
 * @brief Encode a mount_configure struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mount_configure C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mount_configure_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mount_configure_t* mount_configure)
{
    return mavlink_msg_mount_configure_pack(system_id, component_id, msg, mount_configure->target_system, mount_configure->target_component, mount_configure->mount_mode, mount_configure->stab_roll, mount_configure->stab_pitch, mount_configure->stab_yaw);
}

/**
 * @brief Encode a mount_configure struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mount_configure C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mount_configure_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mount_configure_t* mount_configure)
{
    return mavlink_msg_mount_configure_pack_chan(system_id, component_id, chan, msg, mount_configure->target_system, mount_configure->target_component, mount_configure->mount_mode, mount_configure->stab_roll, mount_configure->stab_pitch, mount_configure->stab_yaw);
}

/**
 * @brief Send a mount_configure message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param mount_mode  Mount operating mode.
 * @param stab_roll  (1 = yes, 0 = no).
 * @param stab_pitch  (1 = yes, 0 = no).
 * @param stab_yaw  (1 = yes, 0 = no).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mount_configure_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t mount_mode, uint8_t stab_roll, uint8_t stab_pitch, uint8_t stab_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOUNT_CONFIGURE_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, mount_mode);
    _mav_put_uint8_t(buf, 3, stab_roll);
    _mav_put_uint8_t(buf, 4, stab_pitch);
    _mav_put_uint8_t(buf, 5, stab_yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOUNT_CONFIGURE, buf, MAVLINK_MSG_ID_MOUNT_CONFIGURE_MIN_LEN, MAVLINK_MSG_ID_MOUNT_CONFIGURE_LEN, MAVLINK_MSG_ID_MOUNT_CONFIGURE_CRC);
#else
    mavlink_mount_configure_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.mount_mode = mount_mode;
    packet.stab_roll = stab_roll;
    packet.stab_pitch = stab_pitch;
    packet.stab_yaw = stab_yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOUNT_CONFIGURE, (const char *)&packet, MAVLINK_MSG_ID_MOUNT_CONFIGURE_MIN_LEN, MAVLINK_MSG_ID_MOUNT_CONFIGURE_LEN, MAVLINK_MSG_ID_MOUNT_CONFIGURE_CRC);
#endif
}

/**
 * @brief Send a mount_configure message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mount_configure_send_struct(mavlink_channel_t chan, const mavlink_mount_configure_t* mount_configure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mount_configure_send(chan, mount_configure->target_system, mount_configure->target_component, mount_configure->mount_mode, mount_configure->stab_roll, mount_configure->stab_pitch, mount_configure->stab_yaw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOUNT_CONFIGURE, (const char *)mount_configure, MAVLINK_MSG_ID_MOUNT_CONFIGURE_MIN_LEN, MAVLINK_MSG_ID_MOUNT_CONFIGURE_LEN, MAVLINK_MSG_ID_MOUNT_CONFIGURE_CRC);
#endif
}

#if MAVLINK_MSG_ID_MOUNT_CONFIGURE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mount_configure_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t mount_mode, uint8_t stab_roll, uint8_t stab_pitch, uint8_t stab_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, mount_mode);
    _mav_put_uint8_t(buf, 3, stab_roll);
    _mav_put_uint8_t(buf, 4, stab_pitch);
    _mav_put_uint8_t(buf, 5, stab_yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOUNT_CONFIGURE, buf, MAVLINK_MSG_ID_MOUNT_CONFIGURE_MIN_LEN, MAVLINK_MSG_ID_MOUNT_CONFIGURE_LEN, MAVLINK_MSG_ID_MOUNT_CONFIGURE_CRC);
#else
    mavlink_mount_configure_t *packet = (mavlink_mount_configure_t *)msgbuf;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->mount_mode = mount_mode;
    packet->stab_roll = stab_roll;
    packet->stab_pitch = stab_pitch;
    packet->stab_yaw = stab_yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOUNT_CONFIGURE, (const char *)packet, MAVLINK_MSG_ID_MOUNT_CONFIGURE_MIN_LEN, MAVLINK_MSG_ID_MOUNT_CONFIGURE_LEN, MAVLINK_MSG_ID_MOUNT_CONFIGURE_CRC);
#endif
}
#endif

#endif

// MESSAGE MOUNT_CONFIGURE UNPACKING


/**
 * @brief Get field target_system from mount_configure message
 *
 * @return  System ID.
 */
static inline uint8_t mavlink_msg_mount_configure_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field target_component from mount_configure message
 *
 * @return  Component ID.
 */
static inline uint8_t mavlink_msg_mount_configure_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field mount_mode from mount_configure message
 *
 * @return  Mount operating mode.
 */
static inline uint8_t mavlink_msg_mount_configure_get_mount_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field stab_roll from mount_configure message
 *
 * @return  (1 = yes, 0 = no).
 */
static inline uint8_t mavlink_msg_mount_configure_get_stab_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field stab_pitch from mount_configure message
 *
 * @return  (1 = yes, 0 = no).
 */
static inline uint8_t mavlink_msg_mount_configure_get_stab_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field stab_yaw from mount_configure message
 *
 * @return  (1 = yes, 0 = no).
 */
static inline uint8_t mavlink_msg_mount_configure_get_stab_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Decode a mount_configure message into a struct
 *
 * @param msg The message to decode
 * @param mount_configure C-struct to decode the message contents into
 */
static inline void mavlink_msg_mount_configure_decode(const mavlink_message_t* msg, mavlink_mount_configure_t* mount_configure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mount_configure->target_system = mavlink_msg_mount_configure_get_target_system(msg);
    mount_configure->target_component = mavlink_msg_mount_configure_get_target_component(msg);
    mount_configure->mount_mode = mavlink_msg_mount_configure_get_mount_mode(msg);
    mount_configure->stab_roll = mavlink_msg_mount_configure_get_stab_roll(msg);
    mount_configure->stab_pitch = mavlink_msg_mount_configure_get_stab_pitch(msg);
    mount_configure->stab_yaw = mavlink_msg_mount_configure_get_stab_yaw(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MOUNT_CONFIGURE_LEN? msg->len : MAVLINK_MSG_ID_MOUNT_CONFIGURE_LEN;
        memset(mount_configure, 0, MAVLINK_MSG_ID_MOUNT_CONFIGURE_LEN);
    memcpy(mount_configure, _MAV_PAYLOAD(msg), len);
#endif
}
