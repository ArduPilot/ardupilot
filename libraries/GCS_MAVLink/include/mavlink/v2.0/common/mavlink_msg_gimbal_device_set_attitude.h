#pragma once
// MESSAGE GIMBAL_DEVICE_SET_ATTITUDE PACKING

#define MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE 284


typedef struct __mavlink_gimbal_device_set_attitude_t {
 float q[4]; /*<  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation). The frame is described in the message description. Set fields to NaN to be ignored.*/
 float angular_velocity_x; /*< [rad/s] X component of angular velocity (positive: rolling to the right). The frame is described in the message description. NaN to be ignored.*/
 float angular_velocity_y; /*< [rad/s] Y component of angular velocity (positive: pitching up). The frame is described in the message description. NaN to be ignored.*/
 float angular_velocity_z; /*< [rad/s] Z component of angular velocity (positive: yawing to the right). The frame is described in the message description. NaN to be ignored.*/
 uint16_t flags; /*<  Low level gimbal flags.*/
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
} mavlink_gimbal_device_set_attitude_t;

#define MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_LEN 32
#define MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_MIN_LEN 32
#define MAVLINK_MSG_ID_284_LEN 32
#define MAVLINK_MSG_ID_284_MIN_LEN 32

#define MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_CRC 99
#define MAVLINK_MSG_ID_284_CRC 99

#define MAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_FIELD_Q_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GIMBAL_DEVICE_SET_ATTITUDE { \
    284, \
    "GIMBAL_DEVICE_SET_ATTITUDE", \
    7, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_gimbal_device_set_attitude_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_gimbal_device_set_attitude_t, target_component) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_gimbal_device_set_attitude_t, flags) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 4, 0, offsetof(mavlink_gimbal_device_set_attitude_t, q) }, \
         { "angular_velocity_x", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_gimbal_device_set_attitude_t, angular_velocity_x) }, \
         { "angular_velocity_y", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_gimbal_device_set_attitude_t, angular_velocity_y) }, \
         { "angular_velocity_z", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_gimbal_device_set_attitude_t, angular_velocity_z) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GIMBAL_DEVICE_SET_ATTITUDE { \
    "GIMBAL_DEVICE_SET_ATTITUDE", \
    7, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_gimbal_device_set_attitude_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_gimbal_device_set_attitude_t, target_component) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_gimbal_device_set_attitude_t, flags) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 4, 0, offsetof(mavlink_gimbal_device_set_attitude_t, q) }, \
         { "angular_velocity_x", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_gimbal_device_set_attitude_t, angular_velocity_x) }, \
         { "angular_velocity_y", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_gimbal_device_set_attitude_t, angular_velocity_y) }, \
         { "angular_velocity_z", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_gimbal_device_set_attitude_t, angular_velocity_z) }, \
         } \
}
#endif

/**
 * @brief Pack a gimbal_device_set_attitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param flags  Low level gimbal flags.
 * @param q  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation). The frame is described in the message description. Set fields to NaN to be ignored.
 * @param angular_velocity_x [rad/s] X component of angular velocity (positive: rolling to the right). The frame is described in the message description. NaN to be ignored.
 * @param angular_velocity_y [rad/s] Y component of angular velocity (positive: pitching up). The frame is described in the message description. NaN to be ignored.
 * @param angular_velocity_z [rad/s] Z component of angular velocity (positive: yawing to the right). The frame is described in the message description. NaN to be ignored.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_device_set_attitude_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint16_t flags, const float *q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_LEN];
    _mav_put_float(buf, 16, angular_velocity_x);
    _mav_put_float(buf, 20, angular_velocity_y);
    _mav_put_float(buf, 24, angular_velocity_z);
    _mav_put_uint16_t(buf, 28, flags);
    _mav_put_uint8_t(buf, 30, target_system);
    _mav_put_uint8_t(buf, 31, target_component);
    _mav_put_float_array(buf, 0, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_LEN);
#else
    mavlink_gimbal_device_set_attitude_t packet;
    packet.angular_velocity_x = angular_velocity_x;
    packet.angular_velocity_y = angular_velocity_y;
    packet.angular_velocity_z = angular_velocity_z;
    packet.flags = flags;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_CRC);
}

/**
 * @brief Pack a gimbal_device_set_attitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param flags  Low level gimbal flags.
 * @param q  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation). The frame is described in the message description. Set fields to NaN to be ignored.
 * @param angular_velocity_x [rad/s] X component of angular velocity (positive: rolling to the right). The frame is described in the message description. NaN to be ignored.
 * @param angular_velocity_y [rad/s] Y component of angular velocity (positive: pitching up). The frame is described in the message description. NaN to be ignored.
 * @param angular_velocity_z [rad/s] Z component of angular velocity (positive: yawing to the right). The frame is described in the message description. NaN to be ignored.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_device_set_attitude_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint16_t flags, const float *q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_LEN];
    _mav_put_float(buf, 16, angular_velocity_x);
    _mav_put_float(buf, 20, angular_velocity_y);
    _mav_put_float(buf, 24, angular_velocity_z);
    _mav_put_uint16_t(buf, 28, flags);
    _mav_put_uint8_t(buf, 30, target_system);
    _mav_put_uint8_t(buf, 31, target_component);
    _mav_put_float_array(buf, 0, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_LEN);
#else
    mavlink_gimbal_device_set_attitude_t packet;
    packet.angular_velocity_x = angular_velocity_x;
    packet.angular_velocity_y = angular_velocity_y;
    packet.angular_velocity_z = angular_velocity_z;
    packet.flags = flags;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_LEN);
#endif
}

/**
 * @brief Pack a gimbal_device_set_attitude message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param flags  Low level gimbal flags.
 * @param q  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation). The frame is described in the message description. Set fields to NaN to be ignored.
 * @param angular_velocity_x [rad/s] X component of angular velocity (positive: rolling to the right). The frame is described in the message description. NaN to be ignored.
 * @param angular_velocity_y [rad/s] Y component of angular velocity (positive: pitching up). The frame is described in the message description. NaN to be ignored.
 * @param angular_velocity_z [rad/s] Z component of angular velocity (positive: yawing to the right). The frame is described in the message description. NaN to be ignored.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_device_set_attitude_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint16_t flags,const float *q,float angular_velocity_x,float angular_velocity_y,float angular_velocity_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_LEN];
    _mav_put_float(buf, 16, angular_velocity_x);
    _mav_put_float(buf, 20, angular_velocity_y);
    _mav_put_float(buf, 24, angular_velocity_z);
    _mav_put_uint16_t(buf, 28, flags);
    _mav_put_uint8_t(buf, 30, target_system);
    _mav_put_uint8_t(buf, 31, target_component);
    _mav_put_float_array(buf, 0, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_LEN);
#else
    mavlink_gimbal_device_set_attitude_t packet;
    packet.angular_velocity_x = angular_velocity_x;
    packet.angular_velocity_y = angular_velocity_y;
    packet.angular_velocity_z = angular_velocity_z;
    packet.flags = flags;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_CRC);
}

/**
 * @brief Encode a gimbal_device_set_attitude struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_device_set_attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_device_set_attitude_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gimbal_device_set_attitude_t* gimbal_device_set_attitude)
{
    return mavlink_msg_gimbal_device_set_attitude_pack(system_id, component_id, msg, gimbal_device_set_attitude->target_system, gimbal_device_set_attitude->target_component, gimbal_device_set_attitude->flags, gimbal_device_set_attitude->q, gimbal_device_set_attitude->angular_velocity_x, gimbal_device_set_attitude->angular_velocity_y, gimbal_device_set_attitude->angular_velocity_z);
}

/**
 * @brief Encode a gimbal_device_set_attitude struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_device_set_attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_device_set_attitude_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gimbal_device_set_attitude_t* gimbal_device_set_attitude)
{
    return mavlink_msg_gimbal_device_set_attitude_pack_chan(system_id, component_id, chan, msg, gimbal_device_set_attitude->target_system, gimbal_device_set_attitude->target_component, gimbal_device_set_attitude->flags, gimbal_device_set_attitude->q, gimbal_device_set_attitude->angular_velocity_x, gimbal_device_set_attitude->angular_velocity_y, gimbal_device_set_attitude->angular_velocity_z);
}

/**
 * @brief Encode a gimbal_device_set_attitude struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_device_set_attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_device_set_attitude_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_gimbal_device_set_attitude_t* gimbal_device_set_attitude)
{
    return mavlink_msg_gimbal_device_set_attitude_pack_status(system_id, component_id, _status, msg,  gimbal_device_set_attitude->target_system, gimbal_device_set_attitude->target_component, gimbal_device_set_attitude->flags, gimbal_device_set_attitude->q, gimbal_device_set_attitude->angular_velocity_x, gimbal_device_set_attitude->angular_velocity_y, gimbal_device_set_attitude->angular_velocity_z);
}

/**
 * @brief Send a gimbal_device_set_attitude message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param flags  Low level gimbal flags.
 * @param q  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation). The frame is described in the message description. Set fields to NaN to be ignored.
 * @param angular_velocity_x [rad/s] X component of angular velocity (positive: rolling to the right). The frame is described in the message description. NaN to be ignored.
 * @param angular_velocity_y [rad/s] Y component of angular velocity (positive: pitching up). The frame is described in the message description. NaN to be ignored.
 * @param angular_velocity_z [rad/s] Z component of angular velocity (positive: yawing to the right). The frame is described in the message description. NaN to be ignored.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gimbal_device_set_attitude_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint16_t flags, const float *q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_LEN];
    _mav_put_float(buf, 16, angular_velocity_x);
    _mav_put_float(buf, 20, angular_velocity_y);
    _mav_put_float(buf, 24, angular_velocity_z);
    _mav_put_uint16_t(buf, 28, flags);
    _mav_put_uint8_t(buf, 30, target_system);
    _mav_put_uint8_t(buf, 31, target_component);
    _mav_put_float_array(buf, 0, q, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE, buf, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_CRC);
#else
    mavlink_gimbal_device_set_attitude_t packet;
    packet.angular_velocity_x = angular_velocity_x;
    packet.angular_velocity_y = angular_velocity_y;
    packet.angular_velocity_z = angular_velocity_z;
    packet.flags = flags;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE, (const char *)&packet, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_CRC);
#endif
}

/**
 * @brief Send a gimbal_device_set_attitude message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gimbal_device_set_attitude_send_struct(mavlink_channel_t chan, const mavlink_gimbal_device_set_attitude_t* gimbal_device_set_attitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gimbal_device_set_attitude_send(chan, gimbal_device_set_attitude->target_system, gimbal_device_set_attitude->target_component, gimbal_device_set_attitude->flags, gimbal_device_set_attitude->q, gimbal_device_set_attitude->angular_velocity_x, gimbal_device_set_attitude->angular_velocity_y, gimbal_device_set_attitude->angular_velocity_z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE, (const char *)gimbal_device_set_attitude, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_CRC);
#endif
}

#if MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gimbal_device_set_attitude_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint16_t flags, const float *q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 16, angular_velocity_x);
    _mav_put_float(buf, 20, angular_velocity_y);
    _mav_put_float(buf, 24, angular_velocity_z);
    _mav_put_uint16_t(buf, 28, flags);
    _mav_put_uint8_t(buf, 30, target_system);
    _mav_put_uint8_t(buf, 31, target_component);
    _mav_put_float_array(buf, 0, q, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE, buf, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_CRC);
#else
    mavlink_gimbal_device_set_attitude_t *packet = (mavlink_gimbal_device_set_attitude_t *)msgbuf;
    packet->angular_velocity_x = angular_velocity_x;
    packet->angular_velocity_y = angular_velocity_y;
    packet->angular_velocity_z = angular_velocity_z;
    packet->flags = flags;
    packet->target_system = target_system;
    packet->target_component = target_component;
    mav_array_memcpy(packet->q, q, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE, (const char *)packet, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_CRC);
#endif
}
#endif

#endif

// MESSAGE GIMBAL_DEVICE_SET_ATTITUDE UNPACKING


/**
 * @brief Get field target_system from gimbal_device_set_attitude message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_gimbal_device_set_attitude_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  30);
}

/**
 * @brief Get field target_component from gimbal_device_set_attitude message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_gimbal_device_set_attitude_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  31);
}

/**
 * @brief Get field flags from gimbal_device_set_attitude message
 *
 * @return  Low level gimbal flags.
 */
static inline uint16_t mavlink_msg_gimbal_device_set_attitude_get_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  28);
}

/**
 * @brief Get field q from gimbal_device_set_attitude message
 *
 * @return  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation). The frame is described in the message description. Set fields to NaN to be ignored.
 */
static inline uint16_t mavlink_msg_gimbal_device_set_attitude_get_q(const mavlink_message_t* msg, float *q)
{
    return _MAV_RETURN_float_array(msg, q, 4,  0);
}

/**
 * @brief Get field angular_velocity_x from gimbal_device_set_attitude message
 *
 * @return [rad/s] X component of angular velocity (positive: rolling to the right). The frame is described in the message description. NaN to be ignored.
 */
static inline float mavlink_msg_gimbal_device_set_attitude_get_angular_velocity_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field angular_velocity_y from gimbal_device_set_attitude message
 *
 * @return [rad/s] Y component of angular velocity (positive: pitching up). The frame is described in the message description. NaN to be ignored.
 */
static inline float mavlink_msg_gimbal_device_set_attitude_get_angular_velocity_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field angular_velocity_z from gimbal_device_set_attitude message
 *
 * @return [rad/s] Z component of angular velocity (positive: yawing to the right). The frame is described in the message description. NaN to be ignored.
 */
static inline float mavlink_msg_gimbal_device_set_attitude_get_angular_velocity_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a gimbal_device_set_attitude message into a struct
 *
 * @param msg The message to decode
 * @param gimbal_device_set_attitude C-struct to decode the message contents into
 */
static inline void mavlink_msg_gimbal_device_set_attitude_decode(const mavlink_message_t* msg, mavlink_gimbal_device_set_attitude_t* gimbal_device_set_attitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gimbal_device_set_attitude_get_q(msg, gimbal_device_set_attitude->q);
    gimbal_device_set_attitude->angular_velocity_x = mavlink_msg_gimbal_device_set_attitude_get_angular_velocity_x(msg);
    gimbal_device_set_attitude->angular_velocity_y = mavlink_msg_gimbal_device_set_attitude_get_angular_velocity_y(msg);
    gimbal_device_set_attitude->angular_velocity_z = mavlink_msg_gimbal_device_set_attitude_get_angular_velocity_z(msg);
    gimbal_device_set_attitude->flags = mavlink_msg_gimbal_device_set_attitude_get_flags(msg);
    gimbal_device_set_attitude->target_system = mavlink_msg_gimbal_device_set_attitude_get_target_system(msg);
    gimbal_device_set_attitude->target_component = mavlink_msg_gimbal_device_set_attitude_get_target_component(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_LEN? msg->len : MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_LEN;
        memset(gimbal_device_set_attitude, 0, MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_LEN);
    memcpy(gimbal_device_set_attitude, _MAV_PAYLOAD(msg), len);
#endif
}
