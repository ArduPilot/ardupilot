#pragma once
// MESSAGE GIMBAL_REPORT PACKING

#define MAVLINK_MSG_ID_GIMBAL_REPORT 200


typedef struct __mavlink_gimbal_report_t {
 float delta_time; /*< [s] Time since last update.*/
 float delta_angle_x; /*< [rad] Delta angle X.*/
 float delta_angle_y; /*< [rad] Delta angle Y.*/
 float delta_angle_z; /*< [rad] Delta angle X.*/
 float delta_velocity_x; /*< [m/s] Delta velocity X.*/
 float delta_velocity_y; /*< [m/s] Delta velocity Y.*/
 float delta_velocity_z; /*< [m/s] Delta velocity Z.*/
 float joint_roll; /*< [rad] Joint ROLL.*/
 float joint_el; /*< [rad] Joint EL.*/
 float joint_az; /*< [rad] Joint AZ.*/
 uint8_t target_system; /*<  System ID.*/
 uint8_t target_component; /*<  Component ID.*/
} mavlink_gimbal_report_t;

#define MAVLINK_MSG_ID_GIMBAL_REPORT_LEN 42
#define MAVLINK_MSG_ID_GIMBAL_REPORT_MIN_LEN 42
#define MAVLINK_MSG_ID_200_LEN 42
#define MAVLINK_MSG_ID_200_MIN_LEN 42

#define MAVLINK_MSG_ID_GIMBAL_REPORT_CRC 134
#define MAVLINK_MSG_ID_200_CRC 134



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GIMBAL_REPORT { \
    200, \
    "GIMBAL_REPORT", \
    12, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_gimbal_report_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 41, offsetof(mavlink_gimbal_report_t, target_component) }, \
         { "delta_time", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_gimbal_report_t, delta_time) }, \
         { "delta_angle_x", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_gimbal_report_t, delta_angle_x) }, \
         { "delta_angle_y", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_gimbal_report_t, delta_angle_y) }, \
         { "delta_angle_z", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_gimbal_report_t, delta_angle_z) }, \
         { "delta_velocity_x", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_gimbal_report_t, delta_velocity_x) }, \
         { "delta_velocity_y", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_gimbal_report_t, delta_velocity_y) }, \
         { "delta_velocity_z", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_gimbal_report_t, delta_velocity_z) }, \
         { "joint_roll", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_gimbal_report_t, joint_roll) }, \
         { "joint_el", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_gimbal_report_t, joint_el) }, \
         { "joint_az", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_gimbal_report_t, joint_az) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GIMBAL_REPORT { \
    "GIMBAL_REPORT", \
    12, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_gimbal_report_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 41, offsetof(mavlink_gimbal_report_t, target_component) }, \
         { "delta_time", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_gimbal_report_t, delta_time) }, \
         { "delta_angle_x", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_gimbal_report_t, delta_angle_x) }, \
         { "delta_angle_y", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_gimbal_report_t, delta_angle_y) }, \
         { "delta_angle_z", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_gimbal_report_t, delta_angle_z) }, \
         { "delta_velocity_x", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_gimbal_report_t, delta_velocity_x) }, \
         { "delta_velocity_y", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_gimbal_report_t, delta_velocity_y) }, \
         { "delta_velocity_z", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_gimbal_report_t, delta_velocity_z) }, \
         { "joint_roll", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_gimbal_report_t, joint_roll) }, \
         { "joint_el", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_gimbal_report_t, joint_el) }, \
         { "joint_az", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_gimbal_report_t, joint_az) }, \
         } \
}
#endif

/**
 * @brief Pack a gimbal_report message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param delta_time [s] Time since last update.
 * @param delta_angle_x [rad] Delta angle X.
 * @param delta_angle_y [rad] Delta angle Y.
 * @param delta_angle_z [rad] Delta angle X.
 * @param delta_velocity_x [m/s] Delta velocity X.
 * @param delta_velocity_y [m/s] Delta velocity Y.
 * @param delta_velocity_z [m/s] Delta velocity Z.
 * @param joint_roll [rad] Joint ROLL.
 * @param joint_el [rad] Joint EL.
 * @param joint_az [rad] Joint AZ.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_report_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, float delta_time, float delta_angle_x, float delta_angle_y, float delta_angle_z, float delta_velocity_x, float delta_velocity_y, float delta_velocity_z, float joint_roll, float joint_el, float joint_az)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_REPORT_LEN];
    _mav_put_float(buf, 0, delta_time);
    _mav_put_float(buf, 4, delta_angle_x);
    _mav_put_float(buf, 8, delta_angle_y);
    _mav_put_float(buf, 12, delta_angle_z);
    _mav_put_float(buf, 16, delta_velocity_x);
    _mav_put_float(buf, 20, delta_velocity_y);
    _mav_put_float(buf, 24, delta_velocity_z);
    _mav_put_float(buf, 28, joint_roll);
    _mav_put_float(buf, 32, joint_el);
    _mav_put_float(buf, 36, joint_az);
    _mav_put_uint8_t(buf, 40, target_system);
    _mav_put_uint8_t(buf, 41, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_REPORT_LEN);
#else
    mavlink_gimbal_report_t packet;
    packet.delta_time = delta_time;
    packet.delta_angle_x = delta_angle_x;
    packet.delta_angle_y = delta_angle_y;
    packet.delta_angle_z = delta_angle_z;
    packet.delta_velocity_x = delta_velocity_x;
    packet.delta_velocity_y = delta_velocity_y;
    packet.delta_velocity_z = delta_velocity_z;
    packet.joint_roll = joint_roll;
    packet.joint_el = joint_el;
    packet.joint_az = joint_az;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_REPORT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GIMBAL_REPORT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GIMBAL_REPORT_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_REPORT_LEN, MAVLINK_MSG_ID_GIMBAL_REPORT_CRC);
}

/**
 * @brief Pack a gimbal_report message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param delta_time [s] Time since last update.
 * @param delta_angle_x [rad] Delta angle X.
 * @param delta_angle_y [rad] Delta angle Y.
 * @param delta_angle_z [rad] Delta angle X.
 * @param delta_velocity_x [m/s] Delta velocity X.
 * @param delta_velocity_y [m/s] Delta velocity Y.
 * @param delta_velocity_z [m/s] Delta velocity Z.
 * @param joint_roll [rad] Joint ROLL.
 * @param joint_el [rad] Joint EL.
 * @param joint_az [rad] Joint AZ.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_report_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,float delta_time,float delta_angle_x,float delta_angle_y,float delta_angle_z,float delta_velocity_x,float delta_velocity_y,float delta_velocity_z,float joint_roll,float joint_el,float joint_az)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_REPORT_LEN];
    _mav_put_float(buf, 0, delta_time);
    _mav_put_float(buf, 4, delta_angle_x);
    _mav_put_float(buf, 8, delta_angle_y);
    _mav_put_float(buf, 12, delta_angle_z);
    _mav_put_float(buf, 16, delta_velocity_x);
    _mav_put_float(buf, 20, delta_velocity_y);
    _mav_put_float(buf, 24, delta_velocity_z);
    _mav_put_float(buf, 28, joint_roll);
    _mav_put_float(buf, 32, joint_el);
    _mav_put_float(buf, 36, joint_az);
    _mav_put_uint8_t(buf, 40, target_system);
    _mav_put_uint8_t(buf, 41, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_REPORT_LEN);
#else
    mavlink_gimbal_report_t packet;
    packet.delta_time = delta_time;
    packet.delta_angle_x = delta_angle_x;
    packet.delta_angle_y = delta_angle_y;
    packet.delta_angle_z = delta_angle_z;
    packet.delta_velocity_x = delta_velocity_x;
    packet.delta_velocity_y = delta_velocity_y;
    packet.delta_velocity_z = delta_velocity_z;
    packet.joint_roll = joint_roll;
    packet.joint_el = joint_el;
    packet.joint_az = joint_az;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_REPORT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GIMBAL_REPORT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GIMBAL_REPORT_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_REPORT_LEN, MAVLINK_MSG_ID_GIMBAL_REPORT_CRC);
}

/**
 * @brief Encode a gimbal_report struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_report C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_report_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gimbal_report_t* gimbal_report)
{
    return mavlink_msg_gimbal_report_pack(system_id, component_id, msg, gimbal_report->target_system, gimbal_report->target_component, gimbal_report->delta_time, gimbal_report->delta_angle_x, gimbal_report->delta_angle_y, gimbal_report->delta_angle_z, gimbal_report->delta_velocity_x, gimbal_report->delta_velocity_y, gimbal_report->delta_velocity_z, gimbal_report->joint_roll, gimbal_report->joint_el, gimbal_report->joint_az);
}

/**
 * @brief Encode a gimbal_report struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_report C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_report_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gimbal_report_t* gimbal_report)
{
    return mavlink_msg_gimbal_report_pack_chan(system_id, component_id, chan, msg, gimbal_report->target_system, gimbal_report->target_component, gimbal_report->delta_time, gimbal_report->delta_angle_x, gimbal_report->delta_angle_y, gimbal_report->delta_angle_z, gimbal_report->delta_velocity_x, gimbal_report->delta_velocity_y, gimbal_report->delta_velocity_z, gimbal_report->joint_roll, gimbal_report->joint_el, gimbal_report->joint_az);
}

/**
 * @brief Send a gimbal_report message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param delta_time [s] Time since last update.
 * @param delta_angle_x [rad] Delta angle X.
 * @param delta_angle_y [rad] Delta angle Y.
 * @param delta_angle_z [rad] Delta angle X.
 * @param delta_velocity_x [m/s] Delta velocity X.
 * @param delta_velocity_y [m/s] Delta velocity Y.
 * @param delta_velocity_z [m/s] Delta velocity Z.
 * @param joint_roll [rad] Joint ROLL.
 * @param joint_el [rad] Joint EL.
 * @param joint_az [rad] Joint AZ.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gimbal_report_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, float delta_time, float delta_angle_x, float delta_angle_y, float delta_angle_z, float delta_velocity_x, float delta_velocity_y, float delta_velocity_z, float joint_roll, float joint_el, float joint_az)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_REPORT_LEN];
    _mav_put_float(buf, 0, delta_time);
    _mav_put_float(buf, 4, delta_angle_x);
    _mav_put_float(buf, 8, delta_angle_y);
    _mav_put_float(buf, 12, delta_angle_z);
    _mav_put_float(buf, 16, delta_velocity_x);
    _mav_put_float(buf, 20, delta_velocity_y);
    _mav_put_float(buf, 24, delta_velocity_z);
    _mav_put_float(buf, 28, joint_roll);
    _mav_put_float(buf, 32, joint_el);
    _mav_put_float(buf, 36, joint_az);
    _mav_put_uint8_t(buf, 40, target_system);
    _mav_put_uint8_t(buf, 41, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_REPORT, buf, MAVLINK_MSG_ID_GIMBAL_REPORT_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_REPORT_LEN, MAVLINK_MSG_ID_GIMBAL_REPORT_CRC);
#else
    mavlink_gimbal_report_t packet;
    packet.delta_time = delta_time;
    packet.delta_angle_x = delta_angle_x;
    packet.delta_angle_y = delta_angle_y;
    packet.delta_angle_z = delta_angle_z;
    packet.delta_velocity_x = delta_velocity_x;
    packet.delta_velocity_y = delta_velocity_y;
    packet.delta_velocity_z = delta_velocity_z;
    packet.joint_roll = joint_roll;
    packet.joint_el = joint_el;
    packet.joint_az = joint_az;
    packet.target_system = target_system;
    packet.target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_REPORT, (const char *)&packet, MAVLINK_MSG_ID_GIMBAL_REPORT_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_REPORT_LEN, MAVLINK_MSG_ID_GIMBAL_REPORT_CRC);
#endif
}

/**
 * @brief Send a gimbal_report message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gimbal_report_send_struct(mavlink_channel_t chan, const mavlink_gimbal_report_t* gimbal_report)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gimbal_report_send(chan, gimbal_report->target_system, gimbal_report->target_component, gimbal_report->delta_time, gimbal_report->delta_angle_x, gimbal_report->delta_angle_y, gimbal_report->delta_angle_z, gimbal_report->delta_velocity_x, gimbal_report->delta_velocity_y, gimbal_report->delta_velocity_z, gimbal_report->joint_roll, gimbal_report->joint_el, gimbal_report->joint_az);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_REPORT, (const char *)gimbal_report, MAVLINK_MSG_ID_GIMBAL_REPORT_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_REPORT_LEN, MAVLINK_MSG_ID_GIMBAL_REPORT_CRC);
#endif
}

#if MAVLINK_MSG_ID_GIMBAL_REPORT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gimbal_report_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, float delta_time, float delta_angle_x, float delta_angle_y, float delta_angle_z, float delta_velocity_x, float delta_velocity_y, float delta_velocity_z, float joint_roll, float joint_el, float joint_az)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, delta_time);
    _mav_put_float(buf, 4, delta_angle_x);
    _mav_put_float(buf, 8, delta_angle_y);
    _mav_put_float(buf, 12, delta_angle_z);
    _mav_put_float(buf, 16, delta_velocity_x);
    _mav_put_float(buf, 20, delta_velocity_y);
    _mav_put_float(buf, 24, delta_velocity_z);
    _mav_put_float(buf, 28, joint_roll);
    _mav_put_float(buf, 32, joint_el);
    _mav_put_float(buf, 36, joint_az);
    _mav_put_uint8_t(buf, 40, target_system);
    _mav_put_uint8_t(buf, 41, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_REPORT, buf, MAVLINK_MSG_ID_GIMBAL_REPORT_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_REPORT_LEN, MAVLINK_MSG_ID_GIMBAL_REPORT_CRC);
#else
    mavlink_gimbal_report_t *packet = (mavlink_gimbal_report_t *)msgbuf;
    packet->delta_time = delta_time;
    packet->delta_angle_x = delta_angle_x;
    packet->delta_angle_y = delta_angle_y;
    packet->delta_angle_z = delta_angle_z;
    packet->delta_velocity_x = delta_velocity_x;
    packet->delta_velocity_y = delta_velocity_y;
    packet->delta_velocity_z = delta_velocity_z;
    packet->joint_roll = joint_roll;
    packet->joint_el = joint_el;
    packet->joint_az = joint_az;
    packet->target_system = target_system;
    packet->target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_REPORT, (const char *)packet, MAVLINK_MSG_ID_GIMBAL_REPORT_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_REPORT_LEN, MAVLINK_MSG_ID_GIMBAL_REPORT_CRC);
#endif
}
#endif

#endif

// MESSAGE GIMBAL_REPORT UNPACKING


/**
 * @brief Get field target_system from gimbal_report message
 *
 * @return  System ID.
 */
static inline uint8_t mavlink_msg_gimbal_report_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  40);
}

/**
 * @brief Get field target_component from gimbal_report message
 *
 * @return  Component ID.
 */
static inline uint8_t mavlink_msg_gimbal_report_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  41);
}

/**
 * @brief Get field delta_time from gimbal_report message
 *
 * @return [s] Time since last update.
 */
static inline float mavlink_msg_gimbal_report_get_delta_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field delta_angle_x from gimbal_report message
 *
 * @return [rad] Delta angle X.
 */
static inline float mavlink_msg_gimbal_report_get_delta_angle_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field delta_angle_y from gimbal_report message
 *
 * @return [rad] Delta angle Y.
 */
static inline float mavlink_msg_gimbal_report_get_delta_angle_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field delta_angle_z from gimbal_report message
 *
 * @return [rad] Delta angle X.
 */
static inline float mavlink_msg_gimbal_report_get_delta_angle_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field delta_velocity_x from gimbal_report message
 *
 * @return [m/s] Delta velocity X.
 */
static inline float mavlink_msg_gimbal_report_get_delta_velocity_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field delta_velocity_y from gimbal_report message
 *
 * @return [m/s] Delta velocity Y.
 */
static inline float mavlink_msg_gimbal_report_get_delta_velocity_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field delta_velocity_z from gimbal_report message
 *
 * @return [m/s] Delta velocity Z.
 */
static inline float mavlink_msg_gimbal_report_get_delta_velocity_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field joint_roll from gimbal_report message
 *
 * @return [rad] Joint ROLL.
 */
static inline float mavlink_msg_gimbal_report_get_joint_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field joint_el from gimbal_report message
 *
 * @return [rad] Joint EL.
 */
static inline float mavlink_msg_gimbal_report_get_joint_el(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field joint_az from gimbal_report message
 *
 * @return [rad] Joint AZ.
 */
static inline float mavlink_msg_gimbal_report_get_joint_az(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Decode a gimbal_report message into a struct
 *
 * @param msg The message to decode
 * @param gimbal_report C-struct to decode the message contents into
 */
static inline void mavlink_msg_gimbal_report_decode(const mavlink_message_t* msg, mavlink_gimbal_report_t* gimbal_report)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gimbal_report->delta_time = mavlink_msg_gimbal_report_get_delta_time(msg);
    gimbal_report->delta_angle_x = mavlink_msg_gimbal_report_get_delta_angle_x(msg);
    gimbal_report->delta_angle_y = mavlink_msg_gimbal_report_get_delta_angle_y(msg);
    gimbal_report->delta_angle_z = mavlink_msg_gimbal_report_get_delta_angle_z(msg);
    gimbal_report->delta_velocity_x = mavlink_msg_gimbal_report_get_delta_velocity_x(msg);
    gimbal_report->delta_velocity_y = mavlink_msg_gimbal_report_get_delta_velocity_y(msg);
    gimbal_report->delta_velocity_z = mavlink_msg_gimbal_report_get_delta_velocity_z(msg);
    gimbal_report->joint_roll = mavlink_msg_gimbal_report_get_joint_roll(msg);
    gimbal_report->joint_el = mavlink_msg_gimbal_report_get_joint_el(msg);
    gimbal_report->joint_az = mavlink_msg_gimbal_report_get_joint_az(msg);
    gimbal_report->target_system = mavlink_msg_gimbal_report_get_target_system(msg);
    gimbal_report->target_component = mavlink_msg_gimbal_report_get_target_component(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GIMBAL_REPORT_LEN? msg->len : MAVLINK_MSG_ID_GIMBAL_REPORT_LEN;
        memset(gimbal_report, 0, MAVLINK_MSG_ID_GIMBAL_REPORT_LEN);
    memcpy(gimbal_report, _MAV_PAYLOAD(msg), len);
#endif
}
