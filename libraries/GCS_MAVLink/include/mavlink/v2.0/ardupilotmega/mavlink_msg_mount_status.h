#pragma once
// MESSAGE MOUNT_STATUS PACKING

#define MAVLINK_MSG_ID_MOUNT_STATUS 158


typedef struct __mavlink_mount_status_t {
 int32_t pointing_a; /*< [cdeg] Pitch.*/
 int32_t pointing_b; /*< [cdeg] Roll.*/
 int32_t pointing_c; /*< [cdeg] Yaw.*/
 uint8_t target_system; /*<  System ID.*/
 uint8_t target_component; /*<  Component ID.*/
 uint8_t mount_mode; /*<  Mount operating mode.*/
} mavlink_mount_status_t;

#define MAVLINK_MSG_ID_MOUNT_STATUS_LEN 15
#define MAVLINK_MSG_ID_MOUNT_STATUS_MIN_LEN 14
#define MAVLINK_MSG_ID_158_LEN 15
#define MAVLINK_MSG_ID_158_MIN_LEN 14

#define MAVLINK_MSG_ID_MOUNT_STATUS_CRC 134
#define MAVLINK_MSG_ID_158_CRC 134



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MOUNT_STATUS { \
    158, \
    "MOUNT_STATUS", \
    6, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_mount_status_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_mount_status_t, target_component) }, \
         { "pointing_a", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_mount_status_t, pointing_a) }, \
         { "pointing_b", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_mount_status_t, pointing_b) }, \
         { "pointing_c", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_mount_status_t, pointing_c) }, \
         { "mount_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_mount_status_t, mount_mode) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MOUNT_STATUS { \
    "MOUNT_STATUS", \
    6, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_mount_status_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_mount_status_t, target_component) }, \
         { "pointing_a", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_mount_status_t, pointing_a) }, \
         { "pointing_b", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_mount_status_t, pointing_b) }, \
         { "pointing_c", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_mount_status_t, pointing_c) }, \
         { "mount_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_mount_status_t, mount_mode) }, \
         } \
}
#endif

/**
 * @brief Pack a mount_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param pointing_a [cdeg] Pitch.
 * @param pointing_b [cdeg] Roll.
 * @param pointing_c [cdeg] Yaw.
 * @param mount_mode  Mount operating mode.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mount_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, int32_t pointing_a, int32_t pointing_b, int32_t pointing_c, uint8_t mount_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOUNT_STATUS_LEN];
    _mav_put_int32_t(buf, 0, pointing_a);
    _mav_put_int32_t(buf, 4, pointing_b);
    _mav_put_int32_t(buf, 8, pointing_c);
    _mav_put_uint8_t(buf, 12, target_system);
    _mav_put_uint8_t(buf, 13, target_component);
    _mav_put_uint8_t(buf, 14, mount_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOUNT_STATUS_LEN);
#else
    mavlink_mount_status_t packet;
    packet.pointing_a = pointing_a;
    packet.pointing_b = pointing_b;
    packet.pointing_c = pointing_c;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.mount_mode = mount_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOUNT_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOUNT_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MOUNT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MOUNT_STATUS_LEN, MAVLINK_MSG_ID_MOUNT_STATUS_CRC);
}

/**
 * @brief Pack a mount_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param pointing_a [cdeg] Pitch.
 * @param pointing_b [cdeg] Roll.
 * @param pointing_c [cdeg] Yaw.
 * @param mount_mode  Mount operating mode.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mount_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, int32_t pointing_a, int32_t pointing_b, int32_t pointing_c, uint8_t mount_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOUNT_STATUS_LEN];
    _mav_put_int32_t(buf, 0, pointing_a);
    _mav_put_int32_t(buf, 4, pointing_b);
    _mav_put_int32_t(buf, 8, pointing_c);
    _mav_put_uint8_t(buf, 12, target_system);
    _mav_put_uint8_t(buf, 13, target_component);
    _mav_put_uint8_t(buf, 14, mount_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOUNT_STATUS_LEN);
#else
    mavlink_mount_status_t packet;
    packet.pointing_a = pointing_a;
    packet.pointing_b = pointing_b;
    packet.pointing_c = pointing_c;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.mount_mode = mount_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOUNT_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOUNT_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MOUNT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MOUNT_STATUS_LEN, MAVLINK_MSG_ID_MOUNT_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MOUNT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MOUNT_STATUS_LEN);
#endif
}

/**
 * @brief Pack a mount_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param pointing_a [cdeg] Pitch.
 * @param pointing_b [cdeg] Roll.
 * @param pointing_c [cdeg] Yaw.
 * @param mount_mode  Mount operating mode.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mount_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,int32_t pointing_a,int32_t pointing_b,int32_t pointing_c,uint8_t mount_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOUNT_STATUS_LEN];
    _mav_put_int32_t(buf, 0, pointing_a);
    _mav_put_int32_t(buf, 4, pointing_b);
    _mav_put_int32_t(buf, 8, pointing_c);
    _mav_put_uint8_t(buf, 12, target_system);
    _mav_put_uint8_t(buf, 13, target_component);
    _mav_put_uint8_t(buf, 14, mount_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOUNT_STATUS_LEN);
#else
    mavlink_mount_status_t packet;
    packet.pointing_a = pointing_a;
    packet.pointing_b = pointing_b;
    packet.pointing_c = pointing_c;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.mount_mode = mount_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOUNT_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOUNT_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MOUNT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MOUNT_STATUS_LEN, MAVLINK_MSG_ID_MOUNT_STATUS_CRC);
}

/**
 * @brief Encode a mount_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mount_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mount_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mount_status_t* mount_status)
{
    return mavlink_msg_mount_status_pack(system_id, component_id, msg, mount_status->target_system, mount_status->target_component, mount_status->pointing_a, mount_status->pointing_b, mount_status->pointing_c, mount_status->mount_mode);
}

/**
 * @brief Encode a mount_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mount_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mount_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mount_status_t* mount_status)
{
    return mavlink_msg_mount_status_pack_chan(system_id, component_id, chan, msg, mount_status->target_system, mount_status->target_component, mount_status->pointing_a, mount_status->pointing_b, mount_status->pointing_c, mount_status->mount_mode);
}

/**
 * @brief Encode a mount_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param mount_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mount_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_mount_status_t* mount_status)
{
    return mavlink_msg_mount_status_pack_status(system_id, component_id, _status, msg,  mount_status->target_system, mount_status->target_component, mount_status->pointing_a, mount_status->pointing_b, mount_status->pointing_c, mount_status->mount_mode);
}

/**
 * @brief Send a mount_status message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param pointing_a [cdeg] Pitch.
 * @param pointing_b [cdeg] Roll.
 * @param pointing_c [cdeg] Yaw.
 * @param mount_mode  Mount operating mode.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mount_status_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, int32_t pointing_a, int32_t pointing_b, int32_t pointing_c, uint8_t mount_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOUNT_STATUS_LEN];
    _mav_put_int32_t(buf, 0, pointing_a);
    _mav_put_int32_t(buf, 4, pointing_b);
    _mav_put_int32_t(buf, 8, pointing_c);
    _mav_put_uint8_t(buf, 12, target_system);
    _mav_put_uint8_t(buf, 13, target_component);
    _mav_put_uint8_t(buf, 14, mount_mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOUNT_STATUS, buf, MAVLINK_MSG_ID_MOUNT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MOUNT_STATUS_LEN, MAVLINK_MSG_ID_MOUNT_STATUS_CRC);
#else
    mavlink_mount_status_t packet;
    packet.pointing_a = pointing_a;
    packet.pointing_b = pointing_b;
    packet.pointing_c = pointing_c;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.mount_mode = mount_mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOUNT_STATUS, (const char *)&packet, MAVLINK_MSG_ID_MOUNT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MOUNT_STATUS_LEN, MAVLINK_MSG_ID_MOUNT_STATUS_CRC);
#endif
}

/**
 * @brief Send a mount_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mount_status_send_struct(mavlink_channel_t chan, const mavlink_mount_status_t* mount_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mount_status_send(chan, mount_status->target_system, mount_status->target_component, mount_status->pointing_a, mount_status->pointing_b, mount_status->pointing_c, mount_status->mount_mode);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOUNT_STATUS, (const char *)mount_status, MAVLINK_MSG_ID_MOUNT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MOUNT_STATUS_LEN, MAVLINK_MSG_ID_MOUNT_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_MOUNT_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mount_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, int32_t pointing_a, int32_t pointing_b, int32_t pointing_c, uint8_t mount_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, pointing_a);
    _mav_put_int32_t(buf, 4, pointing_b);
    _mav_put_int32_t(buf, 8, pointing_c);
    _mav_put_uint8_t(buf, 12, target_system);
    _mav_put_uint8_t(buf, 13, target_component);
    _mav_put_uint8_t(buf, 14, mount_mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOUNT_STATUS, buf, MAVLINK_MSG_ID_MOUNT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MOUNT_STATUS_LEN, MAVLINK_MSG_ID_MOUNT_STATUS_CRC);
#else
    mavlink_mount_status_t *packet = (mavlink_mount_status_t *)msgbuf;
    packet->pointing_a = pointing_a;
    packet->pointing_b = pointing_b;
    packet->pointing_c = pointing_c;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->mount_mode = mount_mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOUNT_STATUS, (const char *)packet, MAVLINK_MSG_ID_MOUNT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MOUNT_STATUS_LEN, MAVLINK_MSG_ID_MOUNT_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE MOUNT_STATUS UNPACKING


/**
 * @brief Get field target_system from mount_status message
 *
 * @return  System ID.
 */
static inline uint8_t mavlink_msg_mount_status_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field target_component from mount_status message
 *
 * @return  Component ID.
 */
static inline uint8_t mavlink_msg_mount_status_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field pointing_a from mount_status message
 *
 * @return [cdeg] Pitch.
 */
static inline int32_t mavlink_msg_mount_status_get_pointing_a(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field pointing_b from mount_status message
 *
 * @return [cdeg] Roll.
 */
static inline int32_t mavlink_msg_mount_status_get_pointing_b(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field pointing_c from mount_status message
 *
 * @return [cdeg] Yaw.
 */
static inline int32_t mavlink_msg_mount_status_get_pointing_c(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field mount_mode from mount_status message
 *
 * @return  Mount operating mode.
 */
static inline uint8_t mavlink_msg_mount_status_get_mount_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Decode a mount_status message into a struct
 *
 * @param msg The message to decode
 * @param mount_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_mount_status_decode(const mavlink_message_t* msg, mavlink_mount_status_t* mount_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mount_status->pointing_a = mavlink_msg_mount_status_get_pointing_a(msg);
    mount_status->pointing_b = mavlink_msg_mount_status_get_pointing_b(msg);
    mount_status->pointing_c = mavlink_msg_mount_status_get_pointing_c(msg);
    mount_status->target_system = mavlink_msg_mount_status_get_target_system(msg);
    mount_status->target_component = mavlink_msg_mount_status_get_target_component(msg);
    mount_status->mount_mode = mavlink_msg_mount_status_get_mount_mode(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MOUNT_STATUS_LEN? msg->len : MAVLINK_MSG_ID_MOUNT_STATUS_LEN;
        memset(mount_status, 0, MAVLINK_MSG_ID_MOUNT_STATUS_LEN);
    memcpy(mount_status, _MAV_PAYLOAD(msg), len);
#endif
}
