#pragma once
// MESSAGE STORM32_GIMBAL_MANAGER_CONTROL PACKING

#define MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL 60012


typedef struct __mavlink_storm32_gimbal_manager_control_t {
 float q[4]; /*<  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation, the frame is determined by the GIMBAL_MANAGER_FLAGS_ABSOLUTE_YAW flag, set first element to NaN to be ignored).*/
 float angular_velocity_x; /*< [rad/s] X component of angular velocity (positive: roll to the right, NaN to be ignored).*/
 float angular_velocity_y; /*< [rad/s] Y component of angular velocity (positive: tilt up, NaN to be ignored).*/
 float angular_velocity_z; /*< [rad/s] Z component of angular velocity (positive: pan to the right, the frame is determined by the STORM32_GIMBAL_DEVICE_FLAGS_YAW_ABSOLUTE flag, NaN to be ignored).*/
 uint16_t device_flags; /*<  Gimbal device flags (UINT16_MAX to be ignored).*/
 uint16_t manager_flags; /*<  Gimbal manager flags (0 to be ignored).*/
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
 uint8_t gimbal_id; /*<  Gimbal ID of the gimbal manager to address (component ID or 1-6 for non-MAVLink gimbal, 0 for all gimbals, send command multiple times for more than one but not all gimbals).*/
 uint8_t client; /*<  Client which is contacting the gimbal manager (must be set).*/
} mavlink_storm32_gimbal_manager_control_t;

#define MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_LEN 36
#define MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_MIN_LEN 36
#define MAVLINK_MSG_ID_60012_LEN 36
#define MAVLINK_MSG_ID_60012_MIN_LEN 36

#define MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_CRC 99
#define MAVLINK_MSG_ID_60012_CRC 99

#define MAVLINK_MSG_STORM32_GIMBAL_MANAGER_CONTROL_FIELD_Q_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_STORM32_GIMBAL_MANAGER_CONTROL { \
    60012, \
    "STORM32_GIMBAL_MANAGER_CONTROL", \
    10, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_storm32_gimbal_manager_control_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_storm32_gimbal_manager_control_t, target_component) }, \
         { "gimbal_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_storm32_gimbal_manager_control_t, gimbal_id) }, \
         { "client", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_storm32_gimbal_manager_control_t, client) }, \
         { "device_flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_storm32_gimbal_manager_control_t, device_flags) }, \
         { "manager_flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 30, offsetof(mavlink_storm32_gimbal_manager_control_t, manager_flags) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 4, 0, offsetof(mavlink_storm32_gimbal_manager_control_t, q) }, \
         { "angular_velocity_x", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_storm32_gimbal_manager_control_t, angular_velocity_x) }, \
         { "angular_velocity_y", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_storm32_gimbal_manager_control_t, angular_velocity_y) }, \
         { "angular_velocity_z", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_storm32_gimbal_manager_control_t, angular_velocity_z) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_STORM32_GIMBAL_MANAGER_CONTROL { \
    "STORM32_GIMBAL_MANAGER_CONTROL", \
    10, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_storm32_gimbal_manager_control_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_storm32_gimbal_manager_control_t, target_component) }, \
         { "gimbal_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_storm32_gimbal_manager_control_t, gimbal_id) }, \
         { "client", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_storm32_gimbal_manager_control_t, client) }, \
         { "device_flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_storm32_gimbal_manager_control_t, device_flags) }, \
         { "manager_flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 30, offsetof(mavlink_storm32_gimbal_manager_control_t, manager_flags) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 4, 0, offsetof(mavlink_storm32_gimbal_manager_control_t, q) }, \
         { "angular_velocity_x", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_storm32_gimbal_manager_control_t, angular_velocity_x) }, \
         { "angular_velocity_y", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_storm32_gimbal_manager_control_t, angular_velocity_y) }, \
         { "angular_velocity_z", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_storm32_gimbal_manager_control_t, angular_velocity_z) }, \
         } \
}
#endif

/**
 * @brief Pack a storm32_gimbal_manager_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param gimbal_id  Gimbal ID of the gimbal manager to address (component ID or 1-6 for non-MAVLink gimbal, 0 for all gimbals, send command multiple times for more than one but not all gimbals).
 * @param client  Client which is contacting the gimbal manager (must be set).
 * @param device_flags  Gimbal device flags (UINT16_MAX to be ignored).
 * @param manager_flags  Gimbal manager flags (0 to be ignored).
 * @param q  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation, the frame is determined by the GIMBAL_MANAGER_FLAGS_ABSOLUTE_YAW flag, set first element to NaN to be ignored).
 * @param angular_velocity_x [rad/s] X component of angular velocity (positive: roll to the right, NaN to be ignored).
 * @param angular_velocity_y [rad/s] Y component of angular velocity (positive: tilt up, NaN to be ignored).
 * @param angular_velocity_z [rad/s] Z component of angular velocity (positive: pan to the right, the frame is determined by the STORM32_GIMBAL_DEVICE_FLAGS_YAW_ABSOLUTE flag, NaN to be ignored).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_control_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t gimbal_id, uint8_t client, uint16_t device_flags, uint16_t manager_flags, const float *q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_LEN];
    _mav_put_float(buf, 16, angular_velocity_x);
    _mav_put_float(buf, 20, angular_velocity_y);
    _mav_put_float(buf, 24, angular_velocity_z);
    _mav_put_uint16_t(buf, 28, device_flags);
    _mav_put_uint16_t(buf, 30, manager_flags);
    _mav_put_uint8_t(buf, 32, target_system);
    _mav_put_uint8_t(buf, 33, target_component);
    _mav_put_uint8_t(buf, 34, gimbal_id);
    _mav_put_uint8_t(buf, 35, client);
    _mav_put_float_array(buf, 0, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_LEN);
#else
    mavlink_storm32_gimbal_manager_control_t packet;
    packet.angular_velocity_x = angular_velocity_x;
    packet.angular_velocity_y = angular_velocity_y;
    packet.angular_velocity_z = angular_velocity_z;
    packet.device_flags = device_flags;
    packet.manager_flags = manager_flags;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.gimbal_id = gimbal_id;
    packet.client = client;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_CRC);
}

/**
 * @brief Pack a storm32_gimbal_manager_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param gimbal_id  Gimbal ID of the gimbal manager to address (component ID or 1-6 for non-MAVLink gimbal, 0 for all gimbals, send command multiple times for more than one but not all gimbals).
 * @param client  Client which is contacting the gimbal manager (must be set).
 * @param device_flags  Gimbal device flags (UINT16_MAX to be ignored).
 * @param manager_flags  Gimbal manager flags (0 to be ignored).
 * @param q  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation, the frame is determined by the GIMBAL_MANAGER_FLAGS_ABSOLUTE_YAW flag, set first element to NaN to be ignored).
 * @param angular_velocity_x [rad/s] X component of angular velocity (positive: roll to the right, NaN to be ignored).
 * @param angular_velocity_y [rad/s] Y component of angular velocity (positive: tilt up, NaN to be ignored).
 * @param angular_velocity_z [rad/s] Z component of angular velocity (positive: pan to the right, the frame is determined by the STORM32_GIMBAL_DEVICE_FLAGS_YAW_ABSOLUTE flag, NaN to be ignored).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_control_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t gimbal_id, uint8_t client, uint16_t device_flags, uint16_t manager_flags, const float *q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_LEN];
    _mav_put_float(buf, 16, angular_velocity_x);
    _mav_put_float(buf, 20, angular_velocity_y);
    _mav_put_float(buf, 24, angular_velocity_z);
    _mav_put_uint16_t(buf, 28, device_flags);
    _mav_put_uint16_t(buf, 30, manager_flags);
    _mav_put_uint8_t(buf, 32, target_system);
    _mav_put_uint8_t(buf, 33, target_component);
    _mav_put_uint8_t(buf, 34, gimbal_id);
    _mav_put_uint8_t(buf, 35, client);
    _mav_put_float_array(buf, 0, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_LEN);
#else
    mavlink_storm32_gimbal_manager_control_t packet;
    packet.angular_velocity_x = angular_velocity_x;
    packet.angular_velocity_y = angular_velocity_y;
    packet.angular_velocity_z = angular_velocity_z;
    packet.device_flags = device_flags;
    packet.manager_flags = manager_flags;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.gimbal_id = gimbal_id;
    packet.client = client;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_LEN);
#endif
}

/**
 * @brief Pack a storm32_gimbal_manager_control message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param gimbal_id  Gimbal ID of the gimbal manager to address (component ID or 1-6 for non-MAVLink gimbal, 0 for all gimbals, send command multiple times for more than one but not all gimbals).
 * @param client  Client which is contacting the gimbal manager (must be set).
 * @param device_flags  Gimbal device flags (UINT16_MAX to be ignored).
 * @param manager_flags  Gimbal manager flags (0 to be ignored).
 * @param q  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation, the frame is determined by the GIMBAL_MANAGER_FLAGS_ABSOLUTE_YAW flag, set first element to NaN to be ignored).
 * @param angular_velocity_x [rad/s] X component of angular velocity (positive: roll to the right, NaN to be ignored).
 * @param angular_velocity_y [rad/s] Y component of angular velocity (positive: tilt up, NaN to be ignored).
 * @param angular_velocity_z [rad/s] Z component of angular velocity (positive: pan to the right, the frame is determined by the STORM32_GIMBAL_DEVICE_FLAGS_YAW_ABSOLUTE flag, NaN to be ignored).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_control_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint8_t gimbal_id,uint8_t client,uint16_t device_flags,uint16_t manager_flags,const float *q,float angular_velocity_x,float angular_velocity_y,float angular_velocity_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_LEN];
    _mav_put_float(buf, 16, angular_velocity_x);
    _mav_put_float(buf, 20, angular_velocity_y);
    _mav_put_float(buf, 24, angular_velocity_z);
    _mav_put_uint16_t(buf, 28, device_flags);
    _mav_put_uint16_t(buf, 30, manager_flags);
    _mav_put_uint8_t(buf, 32, target_system);
    _mav_put_uint8_t(buf, 33, target_component);
    _mav_put_uint8_t(buf, 34, gimbal_id);
    _mav_put_uint8_t(buf, 35, client);
    _mav_put_float_array(buf, 0, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_LEN);
#else
    mavlink_storm32_gimbal_manager_control_t packet;
    packet.angular_velocity_x = angular_velocity_x;
    packet.angular_velocity_y = angular_velocity_y;
    packet.angular_velocity_z = angular_velocity_z;
    packet.device_flags = device_flags;
    packet.manager_flags = manager_flags;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.gimbal_id = gimbal_id;
    packet.client = client;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_CRC);
}

/**
 * @brief Encode a storm32_gimbal_manager_control struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param storm32_gimbal_manager_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_control_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_storm32_gimbal_manager_control_t* storm32_gimbal_manager_control)
{
    return mavlink_msg_storm32_gimbal_manager_control_pack(system_id, component_id, msg, storm32_gimbal_manager_control->target_system, storm32_gimbal_manager_control->target_component, storm32_gimbal_manager_control->gimbal_id, storm32_gimbal_manager_control->client, storm32_gimbal_manager_control->device_flags, storm32_gimbal_manager_control->manager_flags, storm32_gimbal_manager_control->q, storm32_gimbal_manager_control->angular_velocity_x, storm32_gimbal_manager_control->angular_velocity_y, storm32_gimbal_manager_control->angular_velocity_z);
}

/**
 * @brief Encode a storm32_gimbal_manager_control struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param storm32_gimbal_manager_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_control_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_storm32_gimbal_manager_control_t* storm32_gimbal_manager_control)
{
    return mavlink_msg_storm32_gimbal_manager_control_pack_chan(system_id, component_id, chan, msg, storm32_gimbal_manager_control->target_system, storm32_gimbal_manager_control->target_component, storm32_gimbal_manager_control->gimbal_id, storm32_gimbal_manager_control->client, storm32_gimbal_manager_control->device_flags, storm32_gimbal_manager_control->manager_flags, storm32_gimbal_manager_control->q, storm32_gimbal_manager_control->angular_velocity_x, storm32_gimbal_manager_control->angular_velocity_y, storm32_gimbal_manager_control->angular_velocity_z);
}

/**
 * @brief Encode a storm32_gimbal_manager_control struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param storm32_gimbal_manager_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_control_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_storm32_gimbal_manager_control_t* storm32_gimbal_manager_control)
{
    return mavlink_msg_storm32_gimbal_manager_control_pack_status(system_id, component_id, _status, msg,  storm32_gimbal_manager_control->target_system, storm32_gimbal_manager_control->target_component, storm32_gimbal_manager_control->gimbal_id, storm32_gimbal_manager_control->client, storm32_gimbal_manager_control->device_flags, storm32_gimbal_manager_control->manager_flags, storm32_gimbal_manager_control->q, storm32_gimbal_manager_control->angular_velocity_x, storm32_gimbal_manager_control->angular_velocity_y, storm32_gimbal_manager_control->angular_velocity_z);
}

/**
 * @brief Send a storm32_gimbal_manager_control message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param gimbal_id  Gimbal ID of the gimbal manager to address (component ID or 1-6 for non-MAVLink gimbal, 0 for all gimbals, send command multiple times for more than one but not all gimbals).
 * @param client  Client which is contacting the gimbal manager (must be set).
 * @param device_flags  Gimbal device flags (UINT16_MAX to be ignored).
 * @param manager_flags  Gimbal manager flags (0 to be ignored).
 * @param q  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation, the frame is determined by the GIMBAL_MANAGER_FLAGS_ABSOLUTE_YAW flag, set first element to NaN to be ignored).
 * @param angular_velocity_x [rad/s] X component of angular velocity (positive: roll to the right, NaN to be ignored).
 * @param angular_velocity_y [rad/s] Y component of angular velocity (positive: tilt up, NaN to be ignored).
 * @param angular_velocity_z [rad/s] Z component of angular velocity (positive: pan to the right, the frame is determined by the STORM32_GIMBAL_DEVICE_FLAGS_YAW_ABSOLUTE flag, NaN to be ignored).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_storm32_gimbal_manager_control_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t gimbal_id, uint8_t client, uint16_t device_flags, uint16_t manager_flags, const float *q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_LEN];
    _mav_put_float(buf, 16, angular_velocity_x);
    _mav_put_float(buf, 20, angular_velocity_y);
    _mav_put_float(buf, 24, angular_velocity_z);
    _mav_put_uint16_t(buf, 28, device_flags);
    _mav_put_uint16_t(buf, 30, manager_flags);
    _mav_put_uint8_t(buf, 32, target_system);
    _mav_put_uint8_t(buf, 33, target_component);
    _mav_put_uint8_t(buf, 34, gimbal_id);
    _mav_put_uint8_t(buf, 35, client);
    _mav_put_float_array(buf, 0, q, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL, buf, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_CRC);
#else
    mavlink_storm32_gimbal_manager_control_t packet;
    packet.angular_velocity_x = angular_velocity_x;
    packet.angular_velocity_y = angular_velocity_y;
    packet.angular_velocity_z = angular_velocity_z;
    packet.device_flags = device_flags;
    packet.manager_flags = manager_flags;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.gimbal_id = gimbal_id;
    packet.client = client;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL, (const char *)&packet, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_CRC);
#endif
}

/**
 * @brief Send a storm32_gimbal_manager_control message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_storm32_gimbal_manager_control_send_struct(mavlink_channel_t chan, const mavlink_storm32_gimbal_manager_control_t* storm32_gimbal_manager_control)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_storm32_gimbal_manager_control_send(chan, storm32_gimbal_manager_control->target_system, storm32_gimbal_manager_control->target_component, storm32_gimbal_manager_control->gimbal_id, storm32_gimbal_manager_control->client, storm32_gimbal_manager_control->device_flags, storm32_gimbal_manager_control->manager_flags, storm32_gimbal_manager_control->q, storm32_gimbal_manager_control->angular_velocity_x, storm32_gimbal_manager_control->angular_velocity_y, storm32_gimbal_manager_control->angular_velocity_z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL, (const char *)storm32_gimbal_manager_control, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_CRC);
#endif
}

#if MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_storm32_gimbal_manager_control_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t gimbal_id, uint8_t client, uint16_t device_flags, uint16_t manager_flags, const float *q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 16, angular_velocity_x);
    _mav_put_float(buf, 20, angular_velocity_y);
    _mav_put_float(buf, 24, angular_velocity_z);
    _mav_put_uint16_t(buf, 28, device_flags);
    _mav_put_uint16_t(buf, 30, manager_flags);
    _mav_put_uint8_t(buf, 32, target_system);
    _mav_put_uint8_t(buf, 33, target_component);
    _mav_put_uint8_t(buf, 34, gimbal_id);
    _mav_put_uint8_t(buf, 35, client);
    _mav_put_float_array(buf, 0, q, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL, buf, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_CRC);
#else
    mavlink_storm32_gimbal_manager_control_t *packet = (mavlink_storm32_gimbal_manager_control_t *)msgbuf;
    packet->angular_velocity_x = angular_velocity_x;
    packet->angular_velocity_y = angular_velocity_y;
    packet->angular_velocity_z = angular_velocity_z;
    packet->device_flags = device_flags;
    packet->manager_flags = manager_flags;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->gimbal_id = gimbal_id;
    packet->client = client;
    mav_array_memcpy(packet->q, q, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL, (const char *)packet, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_CRC);
#endif
}
#endif

#endif

// MESSAGE STORM32_GIMBAL_MANAGER_CONTROL UNPACKING


/**
 * @brief Get field target_system from storm32_gimbal_manager_control message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_storm32_gimbal_manager_control_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field target_component from storm32_gimbal_manager_control message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_storm32_gimbal_manager_control_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  33);
}

/**
 * @brief Get field gimbal_id from storm32_gimbal_manager_control message
 *
 * @return  Gimbal ID of the gimbal manager to address (component ID or 1-6 for non-MAVLink gimbal, 0 for all gimbals, send command multiple times for more than one but not all gimbals).
 */
static inline uint8_t mavlink_msg_storm32_gimbal_manager_control_get_gimbal_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field client from storm32_gimbal_manager_control message
 *
 * @return  Client which is contacting the gimbal manager (must be set).
 */
static inline uint8_t mavlink_msg_storm32_gimbal_manager_control_get_client(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  35);
}

/**
 * @brief Get field device_flags from storm32_gimbal_manager_control message
 *
 * @return  Gimbal device flags (UINT16_MAX to be ignored).
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_control_get_device_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  28);
}

/**
 * @brief Get field manager_flags from storm32_gimbal_manager_control message
 *
 * @return  Gimbal manager flags (0 to be ignored).
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_control_get_manager_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  30);
}

/**
 * @brief Get field q from storm32_gimbal_manager_control message
 *
 * @return  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation, the frame is determined by the GIMBAL_MANAGER_FLAGS_ABSOLUTE_YAW flag, set first element to NaN to be ignored).
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_control_get_q(const mavlink_message_t* msg, float *q)
{
    return _MAV_RETURN_float_array(msg, q, 4,  0);
}

/**
 * @brief Get field angular_velocity_x from storm32_gimbal_manager_control message
 *
 * @return [rad/s] X component of angular velocity (positive: roll to the right, NaN to be ignored).
 */
static inline float mavlink_msg_storm32_gimbal_manager_control_get_angular_velocity_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field angular_velocity_y from storm32_gimbal_manager_control message
 *
 * @return [rad/s] Y component of angular velocity (positive: tilt up, NaN to be ignored).
 */
static inline float mavlink_msg_storm32_gimbal_manager_control_get_angular_velocity_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field angular_velocity_z from storm32_gimbal_manager_control message
 *
 * @return [rad/s] Z component of angular velocity (positive: pan to the right, the frame is determined by the STORM32_GIMBAL_DEVICE_FLAGS_YAW_ABSOLUTE flag, NaN to be ignored).
 */
static inline float mavlink_msg_storm32_gimbal_manager_control_get_angular_velocity_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a storm32_gimbal_manager_control message into a struct
 *
 * @param msg The message to decode
 * @param storm32_gimbal_manager_control C-struct to decode the message contents into
 */
static inline void mavlink_msg_storm32_gimbal_manager_control_decode(const mavlink_message_t* msg, mavlink_storm32_gimbal_manager_control_t* storm32_gimbal_manager_control)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_storm32_gimbal_manager_control_get_q(msg, storm32_gimbal_manager_control->q);
    storm32_gimbal_manager_control->angular_velocity_x = mavlink_msg_storm32_gimbal_manager_control_get_angular_velocity_x(msg);
    storm32_gimbal_manager_control->angular_velocity_y = mavlink_msg_storm32_gimbal_manager_control_get_angular_velocity_y(msg);
    storm32_gimbal_manager_control->angular_velocity_z = mavlink_msg_storm32_gimbal_manager_control_get_angular_velocity_z(msg);
    storm32_gimbal_manager_control->device_flags = mavlink_msg_storm32_gimbal_manager_control_get_device_flags(msg);
    storm32_gimbal_manager_control->manager_flags = mavlink_msg_storm32_gimbal_manager_control_get_manager_flags(msg);
    storm32_gimbal_manager_control->target_system = mavlink_msg_storm32_gimbal_manager_control_get_target_system(msg);
    storm32_gimbal_manager_control->target_component = mavlink_msg_storm32_gimbal_manager_control_get_target_component(msg);
    storm32_gimbal_manager_control->gimbal_id = mavlink_msg_storm32_gimbal_manager_control_get_gimbal_id(msg);
    storm32_gimbal_manager_control->client = mavlink_msg_storm32_gimbal_manager_control_get_client(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_LEN? msg->len : MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_LEN;
        memset(storm32_gimbal_manager_control, 0, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_LEN);
    memcpy(storm32_gimbal_manager_control, _MAV_PAYLOAD(msg), len);
#endif
}
