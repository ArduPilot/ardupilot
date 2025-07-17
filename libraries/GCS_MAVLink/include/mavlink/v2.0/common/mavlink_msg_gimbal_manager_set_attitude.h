#pragma once
// MESSAGE GIMBAL_MANAGER_SET_ATTITUDE PACKING

#define MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE 282


typedef struct __mavlink_gimbal_manager_set_attitude_t {
 uint32_t flags; /*<  High level gimbal manager flags to use.*/
 float q[4]; /*<  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation, the frame is depends on whether the flag GIMBAL_MANAGER_FLAGS_YAW_LOCK is set)*/
 float angular_velocity_x; /*< [rad/s] X component of angular velocity, positive is rolling to the right, NaN to be ignored.*/
 float angular_velocity_y; /*< [rad/s] Y component of angular velocity, positive is pitching up, NaN to be ignored.*/
 float angular_velocity_z; /*< [rad/s] Z component of angular velocity, positive is yawing to the right, NaN to be ignored.*/
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
 uint8_t gimbal_device_id; /*<  Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).*/
} mavlink_gimbal_manager_set_attitude_t;

#define MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_LEN 35
#define MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_MIN_LEN 35
#define MAVLINK_MSG_ID_282_LEN 35
#define MAVLINK_MSG_ID_282_MIN_LEN 35

#define MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_CRC 123
#define MAVLINK_MSG_ID_282_CRC 123

#define MAVLINK_MSG_GIMBAL_MANAGER_SET_ATTITUDE_FIELD_Q_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GIMBAL_MANAGER_SET_ATTITUDE { \
    282, \
    "GIMBAL_MANAGER_SET_ATTITUDE", \
    8, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_gimbal_manager_set_attitude_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_gimbal_manager_set_attitude_t, target_component) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_gimbal_manager_set_attitude_t, flags) }, \
         { "gimbal_device_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_gimbal_manager_set_attitude_t, gimbal_device_id) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 4, 4, offsetof(mavlink_gimbal_manager_set_attitude_t, q) }, \
         { "angular_velocity_x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_gimbal_manager_set_attitude_t, angular_velocity_x) }, \
         { "angular_velocity_y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_gimbal_manager_set_attitude_t, angular_velocity_y) }, \
         { "angular_velocity_z", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_gimbal_manager_set_attitude_t, angular_velocity_z) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GIMBAL_MANAGER_SET_ATTITUDE { \
    "GIMBAL_MANAGER_SET_ATTITUDE", \
    8, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_gimbal_manager_set_attitude_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_gimbal_manager_set_attitude_t, target_component) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_gimbal_manager_set_attitude_t, flags) }, \
         { "gimbal_device_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_gimbal_manager_set_attitude_t, gimbal_device_id) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 4, 4, offsetof(mavlink_gimbal_manager_set_attitude_t, q) }, \
         { "angular_velocity_x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_gimbal_manager_set_attitude_t, angular_velocity_x) }, \
         { "angular_velocity_y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_gimbal_manager_set_attitude_t, angular_velocity_y) }, \
         { "angular_velocity_z", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_gimbal_manager_set_attitude_t, angular_velocity_z) }, \
         } \
}
#endif

/**
 * @brief Pack a gimbal_manager_set_attitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param flags  High level gimbal manager flags to use.
 * @param gimbal_device_id  Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
 * @param q  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation, the frame is depends on whether the flag GIMBAL_MANAGER_FLAGS_YAW_LOCK is set)
 * @param angular_velocity_x [rad/s] X component of angular velocity, positive is rolling to the right, NaN to be ignored.
 * @param angular_velocity_y [rad/s] Y component of angular velocity, positive is pitching up, NaN to be ignored.
 * @param angular_velocity_z [rad/s] Z component of angular velocity, positive is yawing to the right, NaN to be ignored.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_manager_set_attitude_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint32_t flags, uint8_t gimbal_device_id, const float *q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_LEN];
    _mav_put_uint32_t(buf, 0, flags);
    _mav_put_float(buf, 20, angular_velocity_x);
    _mav_put_float(buf, 24, angular_velocity_y);
    _mav_put_float(buf, 28, angular_velocity_z);
    _mav_put_uint8_t(buf, 32, target_system);
    _mav_put_uint8_t(buf, 33, target_component);
    _mav_put_uint8_t(buf, 34, gimbal_device_id);
    _mav_put_float_array(buf, 4, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_LEN);
#else
    mavlink_gimbal_manager_set_attitude_t packet;
    packet.flags = flags;
    packet.angular_velocity_x = angular_velocity_x;
    packet.angular_velocity_y = angular_velocity_y;
    packet.angular_velocity_z = angular_velocity_z;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.gimbal_device_id = gimbal_device_id;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_CRC);
}

/**
 * @brief Pack a gimbal_manager_set_attitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param flags  High level gimbal manager flags to use.
 * @param gimbal_device_id  Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
 * @param q  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation, the frame is depends on whether the flag GIMBAL_MANAGER_FLAGS_YAW_LOCK is set)
 * @param angular_velocity_x [rad/s] X component of angular velocity, positive is rolling to the right, NaN to be ignored.
 * @param angular_velocity_y [rad/s] Y component of angular velocity, positive is pitching up, NaN to be ignored.
 * @param angular_velocity_z [rad/s] Z component of angular velocity, positive is yawing to the right, NaN to be ignored.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_manager_set_attitude_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint32_t flags, uint8_t gimbal_device_id, const float *q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_LEN];
    _mav_put_uint32_t(buf, 0, flags);
    _mav_put_float(buf, 20, angular_velocity_x);
    _mav_put_float(buf, 24, angular_velocity_y);
    _mav_put_float(buf, 28, angular_velocity_z);
    _mav_put_uint8_t(buf, 32, target_system);
    _mav_put_uint8_t(buf, 33, target_component);
    _mav_put_uint8_t(buf, 34, gimbal_device_id);
    _mav_put_float_array(buf, 4, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_LEN);
#else
    mavlink_gimbal_manager_set_attitude_t packet;
    packet.flags = flags;
    packet.angular_velocity_x = angular_velocity_x;
    packet.angular_velocity_y = angular_velocity_y;
    packet.angular_velocity_z = angular_velocity_z;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.gimbal_device_id = gimbal_device_id;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_LEN);
#endif
}

/**
 * @brief Pack a gimbal_manager_set_attitude message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param flags  High level gimbal manager flags to use.
 * @param gimbal_device_id  Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
 * @param q  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation, the frame is depends on whether the flag GIMBAL_MANAGER_FLAGS_YAW_LOCK is set)
 * @param angular_velocity_x [rad/s] X component of angular velocity, positive is rolling to the right, NaN to be ignored.
 * @param angular_velocity_y [rad/s] Y component of angular velocity, positive is pitching up, NaN to be ignored.
 * @param angular_velocity_z [rad/s] Z component of angular velocity, positive is yawing to the right, NaN to be ignored.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_manager_set_attitude_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint32_t flags,uint8_t gimbal_device_id,const float *q,float angular_velocity_x,float angular_velocity_y,float angular_velocity_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_LEN];
    _mav_put_uint32_t(buf, 0, flags);
    _mav_put_float(buf, 20, angular_velocity_x);
    _mav_put_float(buf, 24, angular_velocity_y);
    _mav_put_float(buf, 28, angular_velocity_z);
    _mav_put_uint8_t(buf, 32, target_system);
    _mav_put_uint8_t(buf, 33, target_component);
    _mav_put_uint8_t(buf, 34, gimbal_device_id);
    _mav_put_float_array(buf, 4, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_LEN);
#else
    mavlink_gimbal_manager_set_attitude_t packet;
    packet.flags = flags;
    packet.angular_velocity_x = angular_velocity_x;
    packet.angular_velocity_y = angular_velocity_y;
    packet.angular_velocity_z = angular_velocity_z;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.gimbal_device_id = gimbal_device_id;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_CRC);
}

/**
 * @brief Encode a gimbal_manager_set_attitude struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_manager_set_attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_manager_set_attitude_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gimbal_manager_set_attitude_t* gimbal_manager_set_attitude)
{
    return mavlink_msg_gimbal_manager_set_attitude_pack(system_id, component_id, msg, gimbal_manager_set_attitude->target_system, gimbal_manager_set_attitude->target_component, gimbal_manager_set_attitude->flags, gimbal_manager_set_attitude->gimbal_device_id, gimbal_manager_set_attitude->q, gimbal_manager_set_attitude->angular_velocity_x, gimbal_manager_set_attitude->angular_velocity_y, gimbal_manager_set_attitude->angular_velocity_z);
}

/**
 * @brief Encode a gimbal_manager_set_attitude struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_manager_set_attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_manager_set_attitude_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gimbal_manager_set_attitude_t* gimbal_manager_set_attitude)
{
    return mavlink_msg_gimbal_manager_set_attitude_pack_chan(system_id, component_id, chan, msg, gimbal_manager_set_attitude->target_system, gimbal_manager_set_attitude->target_component, gimbal_manager_set_attitude->flags, gimbal_manager_set_attitude->gimbal_device_id, gimbal_manager_set_attitude->q, gimbal_manager_set_attitude->angular_velocity_x, gimbal_manager_set_attitude->angular_velocity_y, gimbal_manager_set_attitude->angular_velocity_z);
}

/**
 * @brief Encode a gimbal_manager_set_attitude struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_manager_set_attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_manager_set_attitude_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_gimbal_manager_set_attitude_t* gimbal_manager_set_attitude)
{
    return mavlink_msg_gimbal_manager_set_attitude_pack_status(system_id, component_id, _status, msg,  gimbal_manager_set_attitude->target_system, gimbal_manager_set_attitude->target_component, gimbal_manager_set_attitude->flags, gimbal_manager_set_attitude->gimbal_device_id, gimbal_manager_set_attitude->q, gimbal_manager_set_attitude->angular_velocity_x, gimbal_manager_set_attitude->angular_velocity_y, gimbal_manager_set_attitude->angular_velocity_z);
}

/**
 * @brief Send a gimbal_manager_set_attitude message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param flags  High level gimbal manager flags to use.
 * @param gimbal_device_id  Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
 * @param q  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation, the frame is depends on whether the flag GIMBAL_MANAGER_FLAGS_YAW_LOCK is set)
 * @param angular_velocity_x [rad/s] X component of angular velocity, positive is rolling to the right, NaN to be ignored.
 * @param angular_velocity_y [rad/s] Y component of angular velocity, positive is pitching up, NaN to be ignored.
 * @param angular_velocity_z [rad/s] Z component of angular velocity, positive is yawing to the right, NaN to be ignored.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gimbal_manager_set_attitude_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint32_t flags, uint8_t gimbal_device_id, const float *q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_LEN];
    _mav_put_uint32_t(buf, 0, flags);
    _mav_put_float(buf, 20, angular_velocity_x);
    _mav_put_float(buf, 24, angular_velocity_y);
    _mav_put_float(buf, 28, angular_velocity_z);
    _mav_put_uint8_t(buf, 32, target_system);
    _mav_put_uint8_t(buf, 33, target_component);
    _mav_put_uint8_t(buf, 34, gimbal_device_id);
    _mav_put_float_array(buf, 4, q, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE, buf, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_CRC);
#else
    mavlink_gimbal_manager_set_attitude_t packet;
    packet.flags = flags;
    packet.angular_velocity_x = angular_velocity_x;
    packet.angular_velocity_y = angular_velocity_y;
    packet.angular_velocity_z = angular_velocity_z;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.gimbal_device_id = gimbal_device_id;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE, (const char *)&packet, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_CRC);
#endif
}

/**
 * @brief Send a gimbal_manager_set_attitude message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gimbal_manager_set_attitude_send_struct(mavlink_channel_t chan, const mavlink_gimbal_manager_set_attitude_t* gimbal_manager_set_attitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gimbal_manager_set_attitude_send(chan, gimbal_manager_set_attitude->target_system, gimbal_manager_set_attitude->target_component, gimbal_manager_set_attitude->flags, gimbal_manager_set_attitude->gimbal_device_id, gimbal_manager_set_attitude->q, gimbal_manager_set_attitude->angular_velocity_x, gimbal_manager_set_attitude->angular_velocity_y, gimbal_manager_set_attitude->angular_velocity_z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE, (const char *)gimbal_manager_set_attitude, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_CRC);
#endif
}

#if MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gimbal_manager_set_attitude_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint32_t flags, uint8_t gimbal_device_id, const float *q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, flags);
    _mav_put_float(buf, 20, angular_velocity_x);
    _mav_put_float(buf, 24, angular_velocity_y);
    _mav_put_float(buf, 28, angular_velocity_z);
    _mav_put_uint8_t(buf, 32, target_system);
    _mav_put_uint8_t(buf, 33, target_component);
    _mav_put_uint8_t(buf, 34, gimbal_device_id);
    _mav_put_float_array(buf, 4, q, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE, buf, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_CRC);
#else
    mavlink_gimbal_manager_set_attitude_t *packet = (mavlink_gimbal_manager_set_attitude_t *)msgbuf;
    packet->flags = flags;
    packet->angular_velocity_x = angular_velocity_x;
    packet->angular_velocity_y = angular_velocity_y;
    packet->angular_velocity_z = angular_velocity_z;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->gimbal_device_id = gimbal_device_id;
    mav_array_memcpy(packet->q, q, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE, (const char *)packet, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_CRC);
#endif
}
#endif

#endif

// MESSAGE GIMBAL_MANAGER_SET_ATTITUDE UNPACKING


/**
 * @brief Get field target_system from gimbal_manager_set_attitude message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_gimbal_manager_set_attitude_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field target_component from gimbal_manager_set_attitude message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_gimbal_manager_set_attitude_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  33);
}

/**
 * @brief Get field flags from gimbal_manager_set_attitude message
 *
 * @return  High level gimbal manager flags to use.
 */
static inline uint32_t mavlink_msg_gimbal_manager_set_attitude_get_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field gimbal_device_id from gimbal_manager_set_attitude message
 *
 * @return  Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
 */
static inline uint8_t mavlink_msg_gimbal_manager_set_attitude_get_gimbal_device_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field q from gimbal_manager_set_attitude message
 *
 * @return  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation, the frame is depends on whether the flag GIMBAL_MANAGER_FLAGS_YAW_LOCK is set)
 */
static inline uint16_t mavlink_msg_gimbal_manager_set_attitude_get_q(const mavlink_message_t* msg, float *q)
{
    return _MAV_RETURN_float_array(msg, q, 4,  4);
}

/**
 * @brief Get field angular_velocity_x from gimbal_manager_set_attitude message
 *
 * @return [rad/s] X component of angular velocity, positive is rolling to the right, NaN to be ignored.
 */
static inline float mavlink_msg_gimbal_manager_set_attitude_get_angular_velocity_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field angular_velocity_y from gimbal_manager_set_attitude message
 *
 * @return [rad/s] Y component of angular velocity, positive is pitching up, NaN to be ignored.
 */
static inline float mavlink_msg_gimbal_manager_set_attitude_get_angular_velocity_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field angular_velocity_z from gimbal_manager_set_attitude message
 *
 * @return [rad/s] Z component of angular velocity, positive is yawing to the right, NaN to be ignored.
 */
static inline float mavlink_msg_gimbal_manager_set_attitude_get_angular_velocity_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a gimbal_manager_set_attitude message into a struct
 *
 * @param msg The message to decode
 * @param gimbal_manager_set_attitude C-struct to decode the message contents into
 */
static inline void mavlink_msg_gimbal_manager_set_attitude_decode(const mavlink_message_t* msg, mavlink_gimbal_manager_set_attitude_t* gimbal_manager_set_attitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gimbal_manager_set_attitude->flags = mavlink_msg_gimbal_manager_set_attitude_get_flags(msg);
    mavlink_msg_gimbal_manager_set_attitude_get_q(msg, gimbal_manager_set_attitude->q);
    gimbal_manager_set_attitude->angular_velocity_x = mavlink_msg_gimbal_manager_set_attitude_get_angular_velocity_x(msg);
    gimbal_manager_set_attitude->angular_velocity_y = mavlink_msg_gimbal_manager_set_attitude_get_angular_velocity_y(msg);
    gimbal_manager_set_attitude->angular_velocity_z = mavlink_msg_gimbal_manager_set_attitude_get_angular_velocity_z(msg);
    gimbal_manager_set_attitude->target_system = mavlink_msg_gimbal_manager_set_attitude_get_target_system(msg);
    gimbal_manager_set_attitude->target_component = mavlink_msg_gimbal_manager_set_attitude_get_target_component(msg);
    gimbal_manager_set_attitude->gimbal_device_id = mavlink_msg_gimbal_manager_set_attitude_get_gimbal_device_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_LEN? msg->len : MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_LEN;
        memset(gimbal_manager_set_attitude, 0, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_LEN);
    memcpy(gimbal_manager_set_attitude, _MAV_PAYLOAD(msg), len);
#endif
}
