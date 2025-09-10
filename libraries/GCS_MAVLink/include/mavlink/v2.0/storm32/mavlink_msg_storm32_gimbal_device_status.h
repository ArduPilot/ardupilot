#pragma once
// MESSAGE STORM32_GIMBAL_DEVICE_STATUS PACKING

#define MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS 60001


typedef struct __mavlink_storm32_gimbal_device_status_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 float q[4]; /*<  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation). The frame depends on the STORM32_GIMBAL_DEVICE_FLAGS_YAW_ABSOLUTE flag.*/
 float angular_velocity_x; /*< [rad/s] X component of angular velocity (NaN if unknown).*/
 float angular_velocity_y; /*< [rad/s] Y component of angular velocity (NaN if unknown).*/
 float angular_velocity_z; /*< [rad/s] Z component of angular velocity (the frame depends on the STORM32_GIMBAL_DEVICE_FLAGS_YAW_ABSOLUTE flag, NaN if unknown).*/
 float yaw_absolute; /*< [deg] Yaw in absolute frame relative to Earth's North, north is 0 (NaN if unknown).*/
 uint16_t flags; /*<  Gimbal device flags currently applied.*/
 uint16_t failure_flags; /*<  Failure flags (0 for no failure).*/
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
} mavlink_storm32_gimbal_device_status_t;

#define MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_LEN 42
#define MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_MIN_LEN 42
#define MAVLINK_MSG_ID_60001_LEN 42
#define MAVLINK_MSG_ID_60001_MIN_LEN 42

#define MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_CRC 186
#define MAVLINK_MSG_ID_60001_CRC 186

#define MAVLINK_MSG_STORM32_GIMBAL_DEVICE_STATUS_FIELD_Q_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_STORM32_GIMBAL_DEVICE_STATUS { \
    60001, \
    "STORM32_GIMBAL_DEVICE_STATUS", \
    10, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_storm32_gimbal_device_status_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 41, offsetof(mavlink_storm32_gimbal_device_status_t, target_component) }, \
         { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_storm32_gimbal_device_status_t, time_boot_ms) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_storm32_gimbal_device_status_t, flags) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 4, 4, offsetof(mavlink_storm32_gimbal_device_status_t, q) }, \
         { "angular_velocity_x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_storm32_gimbal_device_status_t, angular_velocity_x) }, \
         { "angular_velocity_y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_storm32_gimbal_device_status_t, angular_velocity_y) }, \
         { "angular_velocity_z", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_storm32_gimbal_device_status_t, angular_velocity_z) }, \
         { "yaw_absolute", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_storm32_gimbal_device_status_t, yaw_absolute) }, \
         { "failure_flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 38, offsetof(mavlink_storm32_gimbal_device_status_t, failure_flags) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_STORM32_GIMBAL_DEVICE_STATUS { \
    "STORM32_GIMBAL_DEVICE_STATUS", \
    10, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_storm32_gimbal_device_status_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 41, offsetof(mavlink_storm32_gimbal_device_status_t, target_component) }, \
         { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_storm32_gimbal_device_status_t, time_boot_ms) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_storm32_gimbal_device_status_t, flags) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 4, 4, offsetof(mavlink_storm32_gimbal_device_status_t, q) }, \
         { "angular_velocity_x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_storm32_gimbal_device_status_t, angular_velocity_x) }, \
         { "angular_velocity_y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_storm32_gimbal_device_status_t, angular_velocity_y) }, \
         { "angular_velocity_z", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_storm32_gimbal_device_status_t, angular_velocity_z) }, \
         { "yaw_absolute", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_storm32_gimbal_device_status_t, yaw_absolute) }, \
         { "failure_flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 38, offsetof(mavlink_storm32_gimbal_device_status_t, failure_flags) }, \
         } \
}
#endif

/**
 * @brief Pack a storm32_gimbal_device_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param flags  Gimbal device flags currently applied.
 * @param q  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation). The frame depends on the STORM32_GIMBAL_DEVICE_FLAGS_YAW_ABSOLUTE flag.
 * @param angular_velocity_x [rad/s] X component of angular velocity (NaN if unknown).
 * @param angular_velocity_y [rad/s] Y component of angular velocity (NaN if unknown).
 * @param angular_velocity_z [rad/s] Z component of angular velocity (the frame depends on the STORM32_GIMBAL_DEVICE_FLAGS_YAW_ABSOLUTE flag, NaN if unknown).
 * @param yaw_absolute [deg] Yaw in absolute frame relative to Earth's North, north is 0 (NaN if unknown).
 * @param failure_flags  Failure flags (0 for no failure).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_storm32_gimbal_device_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint32_t time_boot_ms, uint16_t flags, const float *q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z, float yaw_absolute, uint16_t failure_flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 20, angular_velocity_x);
    _mav_put_float(buf, 24, angular_velocity_y);
    _mav_put_float(buf, 28, angular_velocity_z);
    _mav_put_float(buf, 32, yaw_absolute);
    _mav_put_uint16_t(buf, 36, flags);
    _mav_put_uint16_t(buf, 38, failure_flags);
    _mav_put_uint8_t(buf, 40, target_system);
    _mav_put_uint8_t(buf, 41, target_component);
    _mav_put_float_array(buf, 4, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_LEN);
#else
    mavlink_storm32_gimbal_device_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.angular_velocity_x = angular_velocity_x;
    packet.angular_velocity_y = angular_velocity_y;
    packet.angular_velocity_z = angular_velocity_z;
    packet.yaw_absolute = yaw_absolute;
    packet.flags = flags;
    packet.failure_flags = failure_flags;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_CRC);
}

/**
 * @brief Pack a storm32_gimbal_device_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param flags  Gimbal device flags currently applied.
 * @param q  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation). The frame depends on the STORM32_GIMBAL_DEVICE_FLAGS_YAW_ABSOLUTE flag.
 * @param angular_velocity_x [rad/s] X component of angular velocity (NaN if unknown).
 * @param angular_velocity_y [rad/s] Y component of angular velocity (NaN if unknown).
 * @param angular_velocity_z [rad/s] Z component of angular velocity (the frame depends on the STORM32_GIMBAL_DEVICE_FLAGS_YAW_ABSOLUTE flag, NaN if unknown).
 * @param yaw_absolute [deg] Yaw in absolute frame relative to Earth's North, north is 0 (NaN if unknown).
 * @param failure_flags  Failure flags (0 for no failure).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_storm32_gimbal_device_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint32_t time_boot_ms, uint16_t flags, const float *q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z, float yaw_absolute, uint16_t failure_flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 20, angular_velocity_x);
    _mav_put_float(buf, 24, angular_velocity_y);
    _mav_put_float(buf, 28, angular_velocity_z);
    _mav_put_float(buf, 32, yaw_absolute);
    _mav_put_uint16_t(buf, 36, flags);
    _mav_put_uint16_t(buf, 38, failure_flags);
    _mav_put_uint8_t(buf, 40, target_system);
    _mav_put_uint8_t(buf, 41, target_component);
    _mav_put_float_array(buf, 4, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_LEN);
#else
    mavlink_storm32_gimbal_device_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.angular_velocity_x = angular_velocity_x;
    packet.angular_velocity_y = angular_velocity_y;
    packet.angular_velocity_z = angular_velocity_z;
    packet.yaw_absolute = yaw_absolute;
    packet.flags = flags;
    packet.failure_flags = failure_flags;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_LEN);
#endif
}

/**
 * @brief Pack a storm32_gimbal_device_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param flags  Gimbal device flags currently applied.
 * @param q  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation). The frame depends on the STORM32_GIMBAL_DEVICE_FLAGS_YAW_ABSOLUTE flag.
 * @param angular_velocity_x [rad/s] X component of angular velocity (NaN if unknown).
 * @param angular_velocity_y [rad/s] Y component of angular velocity (NaN if unknown).
 * @param angular_velocity_z [rad/s] Z component of angular velocity (the frame depends on the STORM32_GIMBAL_DEVICE_FLAGS_YAW_ABSOLUTE flag, NaN if unknown).
 * @param yaw_absolute [deg] Yaw in absolute frame relative to Earth's North, north is 0 (NaN if unknown).
 * @param failure_flags  Failure flags (0 for no failure).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_storm32_gimbal_device_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint32_t time_boot_ms,uint16_t flags,const float *q,float angular_velocity_x,float angular_velocity_y,float angular_velocity_z,float yaw_absolute,uint16_t failure_flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 20, angular_velocity_x);
    _mav_put_float(buf, 24, angular_velocity_y);
    _mav_put_float(buf, 28, angular_velocity_z);
    _mav_put_float(buf, 32, yaw_absolute);
    _mav_put_uint16_t(buf, 36, flags);
    _mav_put_uint16_t(buf, 38, failure_flags);
    _mav_put_uint8_t(buf, 40, target_system);
    _mav_put_uint8_t(buf, 41, target_component);
    _mav_put_float_array(buf, 4, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_LEN);
#else
    mavlink_storm32_gimbal_device_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.angular_velocity_x = angular_velocity_x;
    packet.angular_velocity_y = angular_velocity_y;
    packet.angular_velocity_z = angular_velocity_z;
    packet.yaw_absolute = yaw_absolute;
    packet.flags = flags;
    packet.failure_flags = failure_flags;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_CRC);
}

/**
 * @brief Encode a storm32_gimbal_device_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param storm32_gimbal_device_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_storm32_gimbal_device_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_storm32_gimbal_device_status_t* storm32_gimbal_device_status)
{
    return mavlink_msg_storm32_gimbal_device_status_pack(system_id, component_id, msg, storm32_gimbal_device_status->target_system, storm32_gimbal_device_status->target_component, storm32_gimbal_device_status->time_boot_ms, storm32_gimbal_device_status->flags, storm32_gimbal_device_status->q, storm32_gimbal_device_status->angular_velocity_x, storm32_gimbal_device_status->angular_velocity_y, storm32_gimbal_device_status->angular_velocity_z, storm32_gimbal_device_status->yaw_absolute, storm32_gimbal_device_status->failure_flags);
}

/**
 * @brief Encode a storm32_gimbal_device_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param storm32_gimbal_device_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_storm32_gimbal_device_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_storm32_gimbal_device_status_t* storm32_gimbal_device_status)
{
    return mavlink_msg_storm32_gimbal_device_status_pack_chan(system_id, component_id, chan, msg, storm32_gimbal_device_status->target_system, storm32_gimbal_device_status->target_component, storm32_gimbal_device_status->time_boot_ms, storm32_gimbal_device_status->flags, storm32_gimbal_device_status->q, storm32_gimbal_device_status->angular_velocity_x, storm32_gimbal_device_status->angular_velocity_y, storm32_gimbal_device_status->angular_velocity_z, storm32_gimbal_device_status->yaw_absolute, storm32_gimbal_device_status->failure_flags);
}

/**
 * @brief Encode a storm32_gimbal_device_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param storm32_gimbal_device_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_storm32_gimbal_device_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_storm32_gimbal_device_status_t* storm32_gimbal_device_status)
{
    return mavlink_msg_storm32_gimbal_device_status_pack_status(system_id, component_id, _status, msg,  storm32_gimbal_device_status->target_system, storm32_gimbal_device_status->target_component, storm32_gimbal_device_status->time_boot_ms, storm32_gimbal_device_status->flags, storm32_gimbal_device_status->q, storm32_gimbal_device_status->angular_velocity_x, storm32_gimbal_device_status->angular_velocity_y, storm32_gimbal_device_status->angular_velocity_z, storm32_gimbal_device_status->yaw_absolute, storm32_gimbal_device_status->failure_flags);
}

/**
 * @brief Send a storm32_gimbal_device_status message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param flags  Gimbal device flags currently applied.
 * @param q  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation). The frame depends on the STORM32_GIMBAL_DEVICE_FLAGS_YAW_ABSOLUTE flag.
 * @param angular_velocity_x [rad/s] X component of angular velocity (NaN if unknown).
 * @param angular_velocity_y [rad/s] Y component of angular velocity (NaN if unknown).
 * @param angular_velocity_z [rad/s] Z component of angular velocity (the frame depends on the STORM32_GIMBAL_DEVICE_FLAGS_YAW_ABSOLUTE flag, NaN if unknown).
 * @param yaw_absolute [deg] Yaw in absolute frame relative to Earth's North, north is 0 (NaN if unknown).
 * @param failure_flags  Failure flags (0 for no failure).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_storm32_gimbal_device_status_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint32_t time_boot_ms, uint16_t flags, const float *q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z, float yaw_absolute, uint16_t failure_flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 20, angular_velocity_x);
    _mav_put_float(buf, 24, angular_velocity_y);
    _mav_put_float(buf, 28, angular_velocity_z);
    _mav_put_float(buf, 32, yaw_absolute);
    _mav_put_uint16_t(buf, 36, flags);
    _mav_put_uint16_t(buf, 38, failure_flags);
    _mav_put_uint8_t(buf, 40, target_system);
    _mav_put_uint8_t(buf, 41, target_component);
    _mav_put_float_array(buf, 4, q, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS, buf, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_CRC);
#else
    mavlink_storm32_gimbal_device_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.angular_velocity_x = angular_velocity_x;
    packet.angular_velocity_y = angular_velocity_y;
    packet.angular_velocity_z = angular_velocity_z;
    packet.yaw_absolute = yaw_absolute;
    packet.flags = flags;
    packet.failure_flags = failure_flags;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS, (const char *)&packet, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_CRC);
#endif
}

/**
 * @brief Send a storm32_gimbal_device_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_storm32_gimbal_device_status_send_struct(mavlink_channel_t chan, const mavlink_storm32_gimbal_device_status_t* storm32_gimbal_device_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_storm32_gimbal_device_status_send(chan, storm32_gimbal_device_status->target_system, storm32_gimbal_device_status->target_component, storm32_gimbal_device_status->time_boot_ms, storm32_gimbal_device_status->flags, storm32_gimbal_device_status->q, storm32_gimbal_device_status->angular_velocity_x, storm32_gimbal_device_status->angular_velocity_y, storm32_gimbal_device_status->angular_velocity_z, storm32_gimbal_device_status->yaw_absolute, storm32_gimbal_device_status->failure_flags);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS, (const char *)storm32_gimbal_device_status, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_storm32_gimbal_device_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint32_t time_boot_ms, uint16_t flags, const float *q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z, float yaw_absolute, uint16_t failure_flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 20, angular_velocity_x);
    _mav_put_float(buf, 24, angular_velocity_y);
    _mav_put_float(buf, 28, angular_velocity_z);
    _mav_put_float(buf, 32, yaw_absolute);
    _mav_put_uint16_t(buf, 36, flags);
    _mav_put_uint16_t(buf, 38, failure_flags);
    _mav_put_uint8_t(buf, 40, target_system);
    _mav_put_uint8_t(buf, 41, target_component);
    _mav_put_float_array(buf, 4, q, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS, buf, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_CRC);
#else
    mavlink_storm32_gimbal_device_status_t *packet = (mavlink_storm32_gimbal_device_status_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->angular_velocity_x = angular_velocity_x;
    packet->angular_velocity_y = angular_velocity_y;
    packet->angular_velocity_z = angular_velocity_z;
    packet->yaw_absolute = yaw_absolute;
    packet->flags = flags;
    packet->failure_flags = failure_flags;
    packet->target_system = target_system;
    packet->target_component = target_component;
    mav_array_memcpy(packet->q, q, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS, (const char *)packet, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE STORM32_GIMBAL_DEVICE_STATUS UNPACKING


/**
 * @brief Get field target_system from storm32_gimbal_device_status message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_storm32_gimbal_device_status_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  40);
}

/**
 * @brief Get field target_component from storm32_gimbal_device_status message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_storm32_gimbal_device_status_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  41);
}

/**
 * @brief Get field time_boot_ms from storm32_gimbal_device_status message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_storm32_gimbal_device_status_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field flags from storm32_gimbal_device_status message
 *
 * @return  Gimbal device flags currently applied.
 */
static inline uint16_t mavlink_msg_storm32_gimbal_device_status_get_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  36);
}

/**
 * @brief Get field q from storm32_gimbal_device_status message
 *
 * @return  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation). The frame depends on the STORM32_GIMBAL_DEVICE_FLAGS_YAW_ABSOLUTE flag.
 */
static inline uint16_t mavlink_msg_storm32_gimbal_device_status_get_q(const mavlink_message_t* msg, float *q)
{
    return _MAV_RETURN_float_array(msg, q, 4,  4);
}

/**
 * @brief Get field angular_velocity_x from storm32_gimbal_device_status message
 *
 * @return [rad/s] X component of angular velocity (NaN if unknown).
 */
static inline float mavlink_msg_storm32_gimbal_device_status_get_angular_velocity_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field angular_velocity_y from storm32_gimbal_device_status message
 *
 * @return [rad/s] Y component of angular velocity (NaN if unknown).
 */
static inline float mavlink_msg_storm32_gimbal_device_status_get_angular_velocity_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field angular_velocity_z from storm32_gimbal_device_status message
 *
 * @return [rad/s] Z component of angular velocity (the frame depends on the STORM32_GIMBAL_DEVICE_FLAGS_YAW_ABSOLUTE flag, NaN if unknown).
 */
static inline float mavlink_msg_storm32_gimbal_device_status_get_angular_velocity_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field yaw_absolute from storm32_gimbal_device_status message
 *
 * @return [deg] Yaw in absolute frame relative to Earth's North, north is 0 (NaN if unknown).
 */
static inline float mavlink_msg_storm32_gimbal_device_status_get_yaw_absolute(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field failure_flags from storm32_gimbal_device_status message
 *
 * @return  Failure flags (0 for no failure).
 */
static inline uint16_t mavlink_msg_storm32_gimbal_device_status_get_failure_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  38);
}

/**
 * @brief Decode a storm32_gimbal_device_status message into a struct
 *
 * @param msg The message to decode
 * @param storm32_gimbal_device_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_storm32_gimbal_device_status_decode(const mavlink_message_t* msg, mavlink_storm32_gimbal_device_status_t* storm32_gimbal_device_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    storm32_gimbal_device_status->time_boot_ms = mavlink_msg_storm32_gimbal_device_status_get_time_boot_ms(msg);
    mavlink_msg_storm32_gimbal_device_status_get_q(msg, storm32_gimbal_device_status->q);
    storm32_gimbal_device_status->angular_velocity_x = mavlink_msg_storm32_gimbal_device_status_get_angular_velocity_x(msg);
    storm32_gimbal_device_status->angular_velocity_y = mavlink_msg_storm32_gimbal_device_status_get_angular_velocity_y(msg);
    storm32_gimbal_device_status->angular_velocity_z = mavlink_msg_storm32_gimbal_device_status_get_angular_velocity_z(msg);
    storm32_gimbal_device_status->yaw_absolute = mavlink_msg_storm32_gimbal_device_status_get_yaw_absolute(msg);
    storm32_gimbal_device_status->flags = mavlink_msg_storm32_gimbal_device_status_get_flags(msg);
    storm32_gimbal_device_status->failure_flags = mavlink_msg_storm32_gimbal_device_status_get_failure_flags(msg);
    storm32_gimbal_device_status->target_system = mavlink_msg_storm32_gimbal_device_status_get_target_system(msg);
    storm32_gimbal_device_status->target_component = mavlink_msg_storm32_gimbal_device_status_get_target_component(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_LEN? msg->len : MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_LEN;
        memset(storm32_gimbal_device_status, 0, MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS_LEN);
    memcpy(storm32_gimbal_device_status, _MAV_PAYLOAD(msg), len);
#endif
}
