#pragma once
// MESSAGE STORM32_GIMBAL_MANAGER_INFORMATION PACKING

#define MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION 60010


typedef struct __mavlink_storm32_gimbal_manager_information_t {
 uint32_t device_cap_flags; /*<  Gimbal device capability flags.*/
 uint32_t manager_cap_flags; /*<  Gimbal manager capability flags.*/
 float roll_min; /*< [rad] Hardware minimum roll angle (positive: roll to the right, NaN if unknown).*/
 float roll_max; /*< [rad] Hardware maximum roll angle (positive: roll to the right, NaN if unknown).*/
 float pitch_min; /*< [rad] Hardware minimum pitch/tilt angle (positive: tilt up, NaN if unknown).*/
 float pitch_max; /*< [rad] Hardware maximum pitch/tilt angle (positive: tilt up, NaN if unknown).*/
 float yaw_min; /*< [rad] Hardware minimum yaw/pan angle (positive: pan to the right, relative to the vehicle/gimbal base, NaN if unknown).*/
 float yaw_max; /*< [rad] Hardware maximum yaw/pan angle (positive: pan to the right, relative to the vehicle/gimbal base, NaN if unknown).*/
 uint8_t gimbal_id; /*<  Gimbal ID (component ID or 1-6 for non-MAVLink gimbal) that this gimbal manager is responsible for.*/
} mavlink_storm32_gimbal_manager_information_t;

#define MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_LEN 33
#define MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_MIN_LEN 33
#define MAVLINK_MSG_ID_60010_LEN 33
#define MAVLINK_MSG_ID_60010_MIN_LEN 33

#define MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_CRC 208
#define MAVLINK_MSG_ID_60010_CRC 208



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_STORM32_GIMBAL_MANAGER_INFORMATION { \
    60010, \
    "STORM32_GIMBAL_MANAGER_INFORMATION", \
    9, \
    {  { "gimbal_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_storm32_gimbal_manager_information_t, gimbal_id) }, \
         { "device_cap_flags", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_storm32_gimbal_manager_information_t, device_cap_flags) }, \
         { "manager_cap_flags", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_storm32_gimbal_manager_information_t, manager_cap_flags) }, \
         { "roll_min", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_storm32_gimbal_manager_information_t, roll_min) }, \
         { "roll_max", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_storm32_gimbal_manager_information_t, roll_max) }, \
         { "pitch_min", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_storm32_gimbal_manager_information_t, pitch_min) }, \
         { "pitch_max", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_storm32_gimbal_manager_information_t, pitch_max) }, \
         { "yaw_min", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_storm32_gimbal_manager_information_t, yaw_min) }, \
         { "yaw_max", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_storm32_gimbal_manager_information_t, yaw_max) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_STORM32_GIMBAL_MANAGER_INFORMATION { \
    "STORM32_GIMBAL_MANAGER_INFORMATION", \
    9, \
    {  { "gimbal_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_storm32_gimbal_manager_information_t, gimbal_id) }, \
         { "device_cap_flags", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_storm32_gimbal_manager_information_t, device_cap_flags) }, \
         { "manager_cap_flags", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_storm32_gimbal_manager_information_t, manager_cap_flags) }, \
         { "roll_min", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_storm32_gimbal_manager_information_t, roll_min) }, \
         { "roll_max", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_storm32_gimbal_manager_information_t, roll_max) }, \
         { "pitch_min", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_storm32_gimbal_manager_information_t, pitch_min) }, \
         { "pitch_max", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_storm32_gimbal_manager_information_t, pitch_max) }, \
         { "yaw_min", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_storm32_gimbal_manager_information_t, yaw_min) }, \
         { "yaw_max", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_storm32_gimbal_manager_information_t, yaw_max) }, \
         } \
}
#endif

/**
 * @brief Pack a storm32_gimbal_manager_information message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param gimbal_id  Gimbal ID (component ID or 1-6 for non-MAVLink gimbal) that this gimbal manager is responsible for.
 * @param device_cap_flags  Gimbal device capability flags.
 * @param manager_cap_flags  Gimbal manager capability flags.
 * @param roll_min [rad] Hardware minimum roll angle (positive: roll to the right, NaN if unknown).
 * @param roll_max [rad] Hardware maximum roll angle (positive: roll to the right, NaN if unknown).
 * @param pitch_min [rad] Hardware minimum pitch/tilt angle (positive: tilt up, NaN if unknown).
 * @param pitch_max [rad] Hardware maximum pitch/tilt angle (positive: tilt up, NaN if unknown).
 * @param yaw_min [rad] Hardware minimum yaw/pan angle (positive: pan to the right, relative to the vehicle/gimbal base, NaN if unknown).
 * @param yaw_max [rad] Hardware maximum yaw/pan angle (positive: pan to the right, relative to the vehicle/gimbal base, NaN if unknown).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_information_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t gimbal_id, uint32_t device_cap_flags, uint32_t manager_cap_flags, float roll_min, float roll_max, float pitch_min, float pitch_max, float yaw_min, float yaw_max)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_LEN];
    _mav_put_uint32_t(buf, 0, device_cap_flags);
    _mav_put_uint32_t(buf, 4, manager_cap_flags);
    _mav_put_float(buf, 8, roll_min);
    _mav_put_float(buf, 12, roll_max);
    _mav_put_float(buf, 16, pitch_min);
    _mav_put_float(buf, 20, pitch_max);
    _mav_put_float(buf, 24, yaw_min);
    _mav_put_float(buf, 28, yaw_max);
    _mav_put_uint8_t(buf, 32, gimbal_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_LEN);
#else
    mavlink_storm32_gimbal_manager_information_t packet;
    packet.device_cap_flags = device_cap_flags;
    packet.manager_cap_flags = manager_cap_flags;
    packet.roll_min = roll_min;
    packet.roll_max = roll_max;
    packet.pitch_min = pitch_min;
    packet.pitch_max = pitch_max;
    packet.yaw_min = yaw_min;
    packet.yaw_max = yaw_max;
    packet.gimbal_id = gimbal_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_CRC);
}

/**
 * @brief Pack a storm32_gimbal_manager_information message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param gimbal_id  Gimbal ID (component ID or 1-6 for non-MAVLink gimbal) that this gimbal manager is responsible for.
 * @param device_cap_flags  Gimbal device capability flags.
 * @param manager_cap_flags  Gimbal manager capability flags.
 * @param roll_min [rad] Hardware minimum roll angle (positive: roll to the right, NaN if unknown).
 * @param roll_max [rad] Hardware maximum roll angle (positive: roll to the right, NaN if unknown).
 * @param pitch_min [rad] Hardware minimum pitch/tilt angle (positive: tilt up, NaN if unknown).
 * @param pitch_max [rad] Hardware maximum pitch/tilt angle (positive: tilt up, NaN if unknown).
 * @param yaw_min [rad] Hardware minimum yaw/pan angle (positive: pan to the right, relative to the vehicle/gimbal base, NaN if unknown).
 * @param yaw_max [rad] Hardware maximum yaw/pan angle (positive: pan to the right, relative to the vehicle/gimbal base, NaN if unknown).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_information_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t gimbal_id, uint32_t device_cap_flags, uint32_t manager_cap_flags, float roll_min, float roll_max, float pitch_min, float pitch_max, float yaw_min, float yaw_max)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_LEN];
    _mav_put_uint32_t(buf, 0, device_cap_flags);
    _mav_put_uint32_t(buf, 4, manager_cap_flags);
    _mav_put_float(buf, 8, roll_min);
    _mav_put_float(buf, 12, roll_max);
    _mav_put_float(buf, 16, pitch_min);
    _mav_put_float(buf, 20, pitch_max);
    _mav_put_float(buf, 24, yaw_min);
    _mav_put_float(buf, 28, yaw_max);
    _mav_put_uint8_t(buf, 32, gimbal_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_LEN);
#else
    mavlink_storm32_gimbal_manager_information_t packet;
    packet.device_cap_flags = device_cap_flags;
    packet.manager_cap_flags = manager_cap_flags;
    packet.roll_min = roll_min;
    packet.roll_max = roll_max;
    packet.pitch_min = pitch_min;
    packet.pitch_max = pitch_max;
    packet.yaw_min = yaw_min;
    packet.yaw_max = yaw_max;
    packet.gimbal_id = gimbal_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_LEN);
#endif
}

/**
 * @brief Pack a storm32_gimbal_manager_information message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_id  Gimbal ID (component ID or 1-6 for non-MAVLink gimbal) that this gimbal manager is responsible for.
 * @param device_cap_flags  Gimbal device capability flags.
 * @param manager_cap_flags  Gimbal manager capability flags.
 * @param roll_min [rad] Hardware minimum roll angle (positive: roll to the right, NaN if unknown).
 * @param roll_max [rad] Hardware maximum roll angle (positive: roll to the right, NaN if unknown).
 * @param pitch_min [rad] Hardware minimum pitch/tilt angle (positive: tilt up, NaN if unknown).
 * @param pitch_max [rad] Hardware maximum pitch/tilt angle (positive: tilt up, NaN if unknown).
 * @param yaw_min [rad] Hardware minimum yaw/pan angle (positive: pan to the right, relative to the vehicle/gimbal base, NaN if unknown).
 * @param yaw_max [rad] Hardware maximum yaw/pan angle (positive: pan to the right, relative to the vehicle/gimbal base, NaN if unknown).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_information_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t gimbal_id,uint32_t device_cap_flags,uint32_t manager_cap_flags,float roll_min,float roll_max,float pitch_min,float pitch_max,float yaw_min,float yaw_max)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_LEN];
    _mav_put_uint32_t(buf, 0, device_cap_flags);
    _mav_put_uint32_t(buf, 4, manager_cap_flags);
    _mav_put_float(buf, 8, roll_min);
    _mav_put_float(buf, 12, roll_max);
    _mav_put_float(buf, 16, pitch_min);
    _mav_put_float(buf, 20, pitch_max);
    _mav_put_float(buf, 24, yaw_min);
    _mav_put_float(buf, 28, yaw_max);
    _mav_put_uint8_t(buf, 32, gimbal_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_LEN);
#else
    mavlink_storm32_gimbal_manager_information_t packet;
    packet.device_cap_flags = device_cap_flags;
    packet.manager_cap_flags = manager_cap_flags;
    packet.roll_min = roll_min;
    packet.roll_max = roll_max;
    packet.pitch_min = pitch_min;
    packet.pitch_max = pitch_max;
    packet.yaw_min = yaw_min;
    packet.yaw_max = yaw_max;
    packet.gimbal_id = gimbal_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_CRC);
}

/**
 * @brief Encode a storm32_gimbal_manager_information struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param storm32_gimbal_manager_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_information_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_storm32_gimbal_manager_information_t* storm32_gimbal_manager_information)
{
    return mavlink_msg_storm32_gimbal_manager_information_pack(system_id, component_id, msg, storm32_gimbal_manager_information->gimbal_id, storm32_gimbal_manager_information->device_cap_flags, storm32_gimbal_manager_information->manager_cap_flags, storm32_gimbal_manager_information->roll_min, storm32_gimbal_manager_information->roll_max, storm32_gimbal_manager_information->pitch_min, storm32_gimbal_manager_information->pitch_max, storm32_gimbal_manager_information->yaw_min, storm32_gimbal_manager_information->yaw_max);
}

/**
 * @brief Encode a storm32_gimbal_manager_information struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param storm32_gimbal_manager_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_information_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_storm32_gimbal_manager_information_t* storm32_gimbal_manager_information)
{
    return mavlink_msg_storm32_gimbal_manager_information_pack_chan(system_id, component_id, chan, msg, storm32_gimbal_manager_information->gimbal_id, storm32_gimbal_manager_information->device_cap_flags, storm32_gimbal_manager_information->manager_cap_flags, storm32_gimbal_manager_information->roll_min, storm32_gimbal_manager_information->roll_max, storm32_gimbal_manager_information->pitch_min, storm32_gimbal_manager_information->pitch_max, storm32_gimbal_manager_information->yaw_min, storm32_gimbal_manager_information->yaw_max);
}

/**
 * @brief Encode a storm32_gimbal_manager_information struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param storm32_gimbal_manager_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_information_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_storm32_gimbal_manager_information_t* storm32_gimbal_manager_information)
{
    return mavlink_msg_storm32_gimbal_manager_information_pack_status(system_id, component_id, _status, msg,  storm32_gimbal_manager_information->gimbal_id, storm32_gimbal_manager_information->device_cap_flags, storm32_gimbal_manager_information->manager_cap_flags, storm32_gimbal_manager_information->roll_min, storm32_gimbal_manager_information->roll_max, storm32_gimbal_manager_information->pitch_min, storm32_gimbal_manager_information->pitch_max, storm32_gimbal_manager_information->yaw_min, storm32_gimbal_manager_information->yaw_max);
}

/**
 * @brief Send a storm32_gimbal_manager_information message
 * @param chan MAVLink channel to send the message
 *
 * @param gimbal_id  Gimbal ID (component ID or 1-6 for non-MAVLink gimbal) that this gimbal manager is responsible for.
 * @param device_cap_flags  Gimbal device capability flags.
 * @param manager_cap_flags  Gimbal manager capability flags.
 * @param roll_min [rad] Hardware minimum roll angle (positive: roll to the right, NaN if unknown).
 * @param roll_max [rad] Hardware maximum roll angle (positive: roll to the right, NaN if unknown).
 * @param pitch_min [rad] Hardware minimum pitch/tilt angle (positive: tilt up, NaN if unknown).
 * @param pitch_max [rad] Hardware maximum pitch/tilt angle (positive: tilt up, NaN if unknown).
 * @param yaw_min [rad] Hardware minimum yaw/pan angle (positive: pan to the right, relative to the vehicle/gimbal base, NaN if unknown).
 * @param yaw_max [rad] Hardware maximum yaw/pan angle (positive: pan to the right, relative to the vehicle/gimbal base, NaN if unknown).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_storm32_gimbal_manager_information_send(mavlink_channel_t chan, uint8_t gimbal_id, uint32_t device_cap_flags, uint32_t manager_cap_flags, float roll_min, float roll_max, float pitch_min, float pitch_max, float yaw_min, float yaw_max)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_LEN];
    _mav_put_uint32_t(buf, 0, device_cap_flags);
    _mav_put_uint32_t(buf, 4, manager_cap_flags);
    _mav_put_float(buf, 8, roll_min);
    _mav_put_float(buf, 12, roll_max);
    _mav_put_float(buf, 16, pitch_min);
    _mav_put_float(buf, 20, pitch_max);
    _mav_put_float(buf, 24, yaw_min);
    _mav_put_float(buf, 28, yaw_max);
    _mav_put_uint8_t(buf, 32, gimbal_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION, buf, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_CRC);
#else
    mavlink_storm32_gimbal_manager_information_t packet;
    packet.device_cap_flags = device_cap_flags;
    packet.manager_cap_flags = manager_cap_flags;
    packet.roll_min = roll_min;
    packet.roll_max = roll_max;
    packet.pitch_min = pitch_min;
    packet.pitch_max = pitch_max;
    packet.yaw_min = yaw_min;
    packet.yaw_max = yaw_max;
    packet.gimbal_id = gimbal_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION, (const char *)&packet, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_CRC);
#endif
}

/**
 * @brief Send a storm32_gimbal_manager_information message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_storm32_gimbal_manager_information_send_struct(mavlink_channel_t chan, const mavlink_storm32_gimbal_manager_information_t* storm32_gimbal_manager_information)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_storm32_gimbal_manager_information_send(chan, storm32_gimbal_manager_information->gimbal_id, storm32_gimbal_manager_information->device_cap_flags, storm32_gimbal_manager_information->manager_cap_flags, storm32_gimbal_manager_information->roll_min, storm32_gimbal_manager_information->roll_max, storm32_gimbal_manager_information->pitch_min, storm32_gimbal_manager_information->pitch_max, storm32_gimbal_manager_information->yaw_min, storm32_gimbal_manager_information->yaw_max);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION, (const char *)storm32_gimbal_manager_information, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_CRC);
#endif
}

#if MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_storm32_gimbal_manager_information_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t gimbal_id, uint32_t device_cap_flags, uint32_t manager_cap_flags, float roll_min, float roll_max, float pitch_min, float pitch_max, float yaw_min, float yaw_max)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, device_cap_flags);
    _mav_put_uint32_t(buf, 4, manager_cap_flags);
    _mav_put_float(buf, 8, roll_min);
    _mav_put_float(buf, 12, roll_max);
    _mav_put_float(buf, 16, pitch_min);
    _mav_put_float(buf, 20, pitch_max);
    _mav_put_float(buf, 24, yaw_min);
    _mav_put_float(buf, 28, yaw_max);
    _mav_put_uint8_t(buf, 32, gimbal_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION, buf, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_CRC);
#else
    mavlink_storm32_gimbal_manager_information_t *packet = (mavlink_storm32_gimbal_manager_information_t *)msgbuf;
    packet->device_cap_flags = device_cap_flags;
    packet->manager_cap_flags = manager_cap_flags;
    packet->roll_min = roll_min;
    packet->roll_max = roll_max;
    packet->pitch_min = pitch_min;
    packet->pitch_max = pitch_max;
    packet->yaw_min = yaw_min;
    packet->yaw_max = yaw_max;
    packet->gimbal_id = gimbal_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION, (const char *)packet, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_CRC);
#endif
}
#endif

#endif

// MESSAGE STORM32_GIMBAL_MANAGER_INFORMATION UNPACKING


/**
 * @brief Get field gimbal_id from storm32_gimbal_manager_information message
 *
 * @return  Gimbal ID (component ID or 1-6 for non-MAVLink gimbal) that this gimbal manager is responsible for.
 */
static inline uint8_t mavlink_msg_storm32_gimbal_manager_information_get_gimbal_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field device_cap_flags from storm32_gimbal_manager_information message
 *
 * @return  Gimbal device capability flags.
 */
static inline uint32_t mavlink_msg_storm32_gimbal_manager_information_get_device_cap_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field manager_cap_flags from storm32_gimbal_manager_information message
 *
 * @return  Gimbal manager capability flags.
 */
static inline uint32_t mavlink_msg_storm32_gimbal_manager_information_get_manager_cap_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field roll_min from storm32_gimbal_manager_information message
 *
 * @return [rad] Hardware minimum roll angle (positive: roll to the right, NaN if unknown).
 */
static inline float mavlink_msg_storm32_gimbal_manager_information_get_roll_min(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field roll_max from storm32_gimbal_manager_information message
 *
 * @return [rad] Hardware maximum roll angle (positive: roll to the right, NaN if unknown).
 */
static inline float mavlink_msg_storm32_gimbal_manager_information_get_roll_max(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field pitch_min from storm32_gimbal_manager_information message
 *
 * @return [rad] Hardware minimum pitch/tilt angle (positive: tilt up, NaN if unknown).
 */
static inline float mavlink_msg_storm32_gimbal_manager_information_get_pitch_min(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field pitch_max from storm32_gimbal_manager_information message
 *
 * @return [rad] Hardware maximum pitch/tilt angle (positive: tilt up, NaN if unknown).
 */
static inline float mavlink_msg_storm32_gimbal_manager_information_get_pitch_max(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field yaw_min from storm32_gimbal_manager_information message
 *
 * @return [rad] Hardware minimum yaw/pan angle (positive: pan to the right, relative to the vehicle/gimbal base, NaN if unknown).
 */
static inline float mavlink_msg_storm32_gimbal_manager_information_get_yaw_min(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field yaw_max from storm32_gimbal_manager_information message
 *
 * @return [rad] Hardware maximum yaw/pan angle (positive: pan to the right, relative to the vehicle/gimbal base, NaN if unknown).
 */
static inline float mavlink_msg_storm32_gimbal_manager_information_get_yaw_max(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a storm32_gimbal_manager_information message into a struct
 *
 * @param msg The message to decode
 * @param storm32_gimbal_manager_information C-struct to decode the message contents into
 */
static inline void mavlink_msg_storm32_gimbal_manager_information_decode(const mavlink_message_t* msg, mavlink_storm32_gimbal_manager_information_t* storm32_gimbal_manager_information)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    storm32_gimbal_manager_information->device_cap_flags = mavlink_msg_storm32_gimbal_manager_information_get_device_cap_flags(msg);
    storm32_gimbal_manager_information->manager_cap_flags = mavlink_msg_storm32_gimbal_manager_information_get_manager_cap_flags(msg);
    storm32_gimbal_manager_information->roll_min = mavlink_msg_storm32_gimbal_manager_information_get_roll_min(msg);
    storm32_gimbal_manager_information->roll_max = mavlink_msg_storm32_gimbal_manager_information_get_roll_max(msg);
    storm32_gimbal_manager_information->pitch_min = mavlink_msg_storm32_gimbal_manager_information_get_pitch_min(msg);
    storm32_gimbal_manager_information->pitch_max = mavlink_msg_storm32_gimbal_manager_information_get_pitch_max(msg);
    storm32_gimbal_manager_information->yaw_min = mavlink_msg_storm32_gimbal_manager_information_get_yaw_min(msg);
    storm32_gimbal_manager_information->yaw_max = mavlink_msg_storm32_gimbal_manager_information_get_yaw_max(msg);
    storm32_gimbal_manager_information->gimbal_id = mavlink_msg_storm32_gimbal_manager_information_get_gimbal_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_LEN? msg->len : MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_LEN;
        memset(storm32_gimbal_manager_information, 0, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_LEN);
    memcpy(storm32_gimbal_manager_information, _MAV_PAYLOAD(msg), len);
#endif
}
