#pragma once
// MESSAGE STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW PACKING

#define MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW 60013


typedef struct __mavlink_storm32_gimbal_manager_control_pitchyaw_t {
 float pitch; /*< [rad] Pitch/tilt angle (positive: tilt up, NaN to be ignored).*/
 float yaw; /*< [rad] Yaw/pan angle (positive: pan the right, the frame is determined by the STORM32_GIMBAL_DEVICE_FLAGS_YAW_ABSOLUTE flag, NaN to be ignored).*/
 float pitch_rate; /*< [rad/s] Pitch/tilt angular rate (positive: tilt up, NaN to be ignored).*/
 float yaw_rate; /*< [rad/s] Yaw/pan angular rate (positive: pan to the right, the frame is determined by the STORM32_GIMBAL_DEVICE_FLAGS_YAW_ABSOLUTE flag, NaN to be ignored).*/
 uint16_t device_flags; /*<  Gimbal device flags (UINT16_MAX to be ignored).*/
 uint16_t manager_flags; /*<  Gimbal manager flags (0 to be ignored).*/
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
 uint8_t gimbal_id; /*<  Gimbal ID of the gimbal manager to address (component ID or 1-6 for non-MAVLink gimbal, 0 for all gimbals, send command multiple times for more than one but not all gimbals).*/
 uint8_t client; /*<  Client which is contacting the gimbal manager (must be set).*/
} mavlink_storm32_gimbal_manager_control_pitchyaw_t;

#define MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_LEN 24
#define MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_MIN_LEN 24
#define MAVLINK_MSG_ID_60013_LEN 24
#define MAVLINK_MSG_ID_60013_MIN_LEN 24

#define MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_CRC 129
#define MAVLINK_MSG_ID_60013_CRC 129



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW { \
    60013, \
    "STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW", \
    10, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_storm32_gimbal_manager_control_pitchyaw_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_storm32_gimbal_manager_control_pitchyaw_t, target_component) }, \
         { "gimbal_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_storm32_gimbal_manager_control_pitchyaw_t, gimbal_id) }, \
         { "client", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_storm32_gimbal_manager_control_pitchyaw_t, client) }, \
         { "device_flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_storm32_gimbal_manager_control_pitchyaw_t, device_flags) }, \
         { "manager_flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_storm32_gimbal_manager_control_pitchyaw_t, manager_flags) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_storm32_gimbal_manager_control_pitchyaw_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_storm32_gimbal_manager_control_pitchyaw_t, yaw) }, \
         { "pitch_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_storm32_gimbal_manager_control_pitchyaw_t, pitch_rate) }, \
         { "yaw_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_storm32_gimbal_manager_control_pitchyaw_t, yaw_rate) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW { \
    "STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW", \
    10, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_storm32_gimbal_manager_control_pitchyaw_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_storm32_gimbal_manager_control_pitchyaw_t, target_component) }, \
         { "gimbal_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_storm32_gimbal_manager_control_pitchyaw_t, gimbal_id) }, \
         { "client", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_storm32_gimbal_manager_control_pitchyaw_t, client) }, \
         { "device_flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_storm32_gimbal_manager_control_pitchyaw_t, device_flags) }, \
         { "manager_flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_storm32_gimbal_manager_control_pitchyaw_t, manager_flags) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_storm32_gimbal_manager_control_pitchyaw_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_storm32_gimbal_manager_control_pitchyaw_t, yaw) }, \
         { "pitch_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_storm32_gimbal_manager_control_pitchyaw_t, pitch_rate) }, \
         { "yaw_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_storm32_gimbal_manager_control_pitchyaw_t, yaw_rate) }, \
         } \
}
#endif

/**
 * @brief Pack a storm32_gimbal_manager_control_pitchyaw message
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
 * @param pitch [rad] Pitch/tilt angle (positive: tilt up, NaN to be ignored).
 * @param yaw [rad] Yaw/pan angle (positive: pan the right, the frame is determined by the STORM32_GIMBAL_DEVICE_FLAGS_YAW_ABSOLUTE flag, NaN to be ignored).
 * @param pitch_rate [rad/s] Pitch/tilt angular rate (positive: tilt up, NaN to be ignored).
 * @param yaw_rate [rad/s] Yaw/pan angular rate (positive: pan to the right, the frame is determined by the STORM32_GIMBAL_DEVICE_FLAGS_YAW_ABSOLUTE flag, NaN to be ignored).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_control_pitchyaw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t gimbal_id, uint8_t client, uint16_t device_flags, uint16_t manager_flags, float pitch, float yaw, float pitch_rate, float yaw_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_LEN];
    _mav_put_float(buf, 0, pitch);
    _mav_put_float(buf, 4, yaw);
    _mav_put_float(buf, 8, pitch_rate);
    _mav_put_float(buf, 12, yaw_rate);
    _mav_put_uint16_t(buf, 16, device_flags);
    _mav_put_uint16_t(buf, 18, manager_flags);
    _mav_put_uint8_t(buf, 20, target_system);
    _mav_put_uint8_t(buf, 21, target_component);
    _mav_put_uint8_t(buf, 22, gimbal_id);
    _mav_put_uint8_t(buf, 23, client);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_LEN);
#else
    mavlink_storm32_gimbal_manager_control_pitchyaw_t packet;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.pitch_rate = pitch_rate;
    packet.yaw_rate = yaw_rate;
    packet.device_flags = device_flags;
    packet.manager_flags = manager_flags;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.gimbal_id = gimbal_id;
    packet.client = client;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_CRC);
}

/**
 * @brief Pack a storm32_gimbal_manager_control_pitchyaw message
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
 * @param pitch [rad] Pitch/tilt angle (positive: tilt up, NaN to be ignored).
 * @param yaw [rad] Yaw/pan angle (positive: pan the right, the frame is determined by the STORM32_GIMBAL_DEVICE_FLAGS_YAW_ABSOLUTE flag, NaN to be ignored).
 * @param pitch_rate [rad/s] Pitch/tilt angular rate (positive: tilt up, NaN to be ignored).
 * @param yaw_rate [rad/s] Yaw/pan angular rate (positive: pan to the right, the frame is determined by the STORM32_GIMBAL_DEVICE_FLAGS_YAW_ABSOLUTE flag, NaN to be ignored).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_control_pitchyaw_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t gimbal_id, uint8_t client, uint16_t device_flags, uint16_t manager_flags, float pitch, float yaw, float pitch_rate, float yaw_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_LEN];
    _mav_put_float(buf, 0, pitch);
    _mav_put_float(buf, 4, yaw);
    _mav_put_float(buf, 8, pitch_rate);
    _mav_put_float(buf, 12, yaw_rate);
    _mav_put_uint16_t(buf, 16, device_flags);
    _mav_put_uint16_t(buf, 18, manager_flags);
    _mav_put_uint8_t(buf, 20, target_system);
    _mav_put_uint8_t(buf, 21, target_component);
    _mav_put_uint8_t(buf, 22, gimbal_id);
    _mav_put_uint8_t(buf, 23, client);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_LEN);
#else
    mavlink_storm32_gimbal_manager_control_pitchyaw_t packet;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.pitch_rate = pitch_rate;
    packet.yaw_rate = yaw_rate;
    packet.device_flags = device_flags;
    packet.manager_flags = manager_flags;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.gimbal_id = gimbal_id;
    packet.client = client;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_LEN);
#endif
}

/**
 * @brief Pack a storm32_gimbal_manager_control_pitchyaw message on a channel
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
 * @param pitch [rad] Pitch/tilt angle (positive: tilt up, NaN to be ignored).
 * @param yaw [rad] Yaw/pan angle (positive: pan the right, the frame is determined by the STORM32_GIMBAL_DEVICE_FLAGS_YAW_ABSOLUTE flag, NaN to be ignored).
 * @param pitch_rate [rad/s] Pitch/tilt angular rate (positive: tilt up, NaN to be ignored).
 * @param yaw_rate [rad/s] Yaw/pan angular rate (positive: pan to the right, the frame is determined by the STORM32_GIMBAL_DEVICE_FLAGS_YAW_ABSOLUTE flag, NaN to be ignored).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_control_pitchyaw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint8_t gimbal_id,uint8_t client,uint16_t device_flags,uint16_t manager_flags,float pitch,float yaw,float pitch_rate,float yaw_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_LEN];
    _mav_put_float(buf, 0, pitch);
    _mav_put_float(buf, 4, yaw);
    _mav_put_float(buf, 8, pitch_rate);
    _mav_put_float(buf, 12, yaw_rate);
    _mav_put_uint16_t(buf, 16, device_flags);
    _mav_put_uint16_t(buf, 18, manager_flags);
    _mav_put_uint8_t(buf, 20, target_system);
    _mav_put_uint8_t(buf, 21, target_component);
    _mav_put_uint8_t(buf, 22, gimbal_id);
    _mav_put_uint8_t(buf, 23, client);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_LEN);
#else
    mavlink_storm32_gimbal_manager_control_pitchyaw_t packet;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.pitch_rate = pitch_rate;
    packet.yaw_rate = yaw_rate;
    packet.device_flags = device_flags;
    packet.manager_flags = manager_flags;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.gimbal_id = gimbal_id;
    packet.client = client;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_CRC);
}

/**
 * @brief Encode a storm32_gimbal_manager_control_pitchyaw struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param storm32_gimbal_manager_control_pitchyaw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_control_pitchyaw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_storm32_gimbal_manager_control_pitchyaw_t* storm32_gimbal_manager_control_pitchyaw)
{
    return mavlink_msg_storm32_gimbal_manager_control_pitchyaw_pack(system_id, component_id, msg, storm32_gimbal_manager_control_pitchyaw->target_system, storm32_gimbal_manager_control_pitchyaw->target_component, storm32_gimbal_manager_control_pitchyaw->gimbal_id, storm32_gimbal_manager_control_pitchyaw->client, storm32_gimbal_manager_control_pitchyaw->device_flags, storm32_gimbal_manager_control_pitchyaw->manager_flags, storm32_gimbal_manager_control_pitchyaw->pitch, storm32_gimbal_manager_control_pitchyaw->yaw, storm32_gimbal_manager_control_pitchyaw->pitch_rate, storm32_gimbal_manager_control_pitchyaw->yaw_rate);
}

/**
 * @brief Encode a storm32_gimbal_manager_control_pitchyaw struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param storm32_gimbal_manager_control_pitchyaw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_control_pitchyaw_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_storm32_gimbal_manager_control_pitchyaw_t* storm32_gimbal_manager_control_pitchyaw)
{
    return mavlink_msg_storm32_gimbal_manager_control_pitchyaw_pack_chan(system_id, component_id, chan, msg, storm32_gimbal_manager_control_pitchyaw->target_system, storm32_gimbal_manager_control_pitchyaw->target_component, storm32_gimbal_manager_control_pitchyaw->gimbal_id, storm32_gimbal_manager_control_pitchyaw->client, storm32_gimbal_manager_control_pitchyaw->device_flags, storm32_gimbal_manager_control_pitchyaw->manager_flags, storm32_gimbal_manager_control_pitchyaw->pitch, storm32_gimbal_manager_control_pitchyaw->yaw, storm32_gimbal_manager_control_pitchyaw->pitch_rate, storm32_gimbal_manager_control_pitchyaw->yaw_rate);
}

/**
 * @brief Encode a storm32_gimbal_manager_control_pitchyaw struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param storm32_gimbal_manager_control_pitchyaw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_control_pitchyaw_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_storm32_gimbal_manager_control_pitchyaw_t* storm32_gimbal_manager_control_pitchyaw)
{
    return mavlink_msg_storm32_gimbal_manager_control_pitchyaw_pack_status(system_id, component_id, _status, msg,  storm32_gimbal_manager_control_pitchyaw->target_system, storm32_gimbal_manager_control_pitchyaw->target_component, storm32_gimbal_manager_control_pitchyaw->gimbal_id, storm32_gimbal_manager_control_pitchyaw->client, storm32_gimbal_manager_control_pitchyaw->device_flags, storm32_gimbal_manager_control_pitchyaw->manager_flags, storm32_gimbal_manager_control_pitchyaw->pitch, storm32_gimbal_manager_control_pitchyaw->yaw, storm32_gimbal_manager_control_pitchyaw->pitch_rate, storm32_gimbal_manager_control_pitchyaw->yaw_rate);
}

/**
 * @brief Send a storm32_gimbal_manager_control_pitchyaw message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param gimbal_id  Gimbal ID of the gimbal manager to address (component ID or 1-6 for non-MAVLink gimbal, 0 for all gimbals, send command multiple times for more than one but not all gimbals).
 * @param client  Client which is contacting the gimbal manager (must be set).
 * @param device_flags  Gimbal device flags (UINT16_MAX to be ignored).
 * @param manager_flags  Gimbal manager flags (0 to be ignored).
 * @param pitch [rad] Pitch/tilt angle (positive: tilt up, NaN to be ignored).
 * @param yaw [rad] Yaw/pan angle (positive: pan the right, the frame is determined by the STORM32_GIMBAL_DEVICE_FLAGS_YAW_ABSOLUTE flag, NaN to be ignored).
 * @param pitch_rate [rad/s] Pitch/tilt angular rate (positive: tilt up, NaN to be ignored).
 * @param yaw_rate [rad/s] Yaw/pan angular rate (positive: pan to the right, the frame is determined by the STORM32_GIMBAL_DEVICE_FLAGS_YAW_ABSOLUTE flag, NaN to be ignored).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_storm32_gimbal_manager_control_pitchyaw_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t gimbal_id, uint8_t client, uint16_t device_flags, uint16_t manager_flags, float pitch, float yaw, float pitch_rate, float yaw_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_LEN];
    _mav_put_float(buf, 0, pitch);
    _mav_put_float(buf, 4, yaw);
    _mav_put_float(buf, 8, pitch_rate);
    _mav_put_float(buf, 12, yaw_rate);
    _mav_put_uint16_t(buf, 16, device_flags);
    _mav_put_uint16_t(buf, 18, manager_flags);
    _mav_put_uint8_t(buf, 20, target_system);
    _mav_put_uint8_t(buf, 21, target_component);
    _mav_put_uint8_t(buf, 22, gimbal_id);
    _mav_put_uint8_t(buf, 23, client);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW, buf, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_CRC);
#else
    mavlink_storm32_gimbal_manager_control_pitchyaw_t packet;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.pitch_rate = pitch_rate;
    packet.yaw_rate = yaw_rate;
    packet.device_flags = device_flags;
    packet.manager_flags = manager_flags;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.gimbal_id = gimbal_id;
    packet.client = client;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW, (const char *)&packet, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_CRC);
#endif
}

/**
 * @brief Send a storm32_gimbal_manager_control_pitchyaw message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_storm32_gimbal_manager_control_pitchyaw_send_struct(mavlink_channel_t chan, const mavlink_storm32_gimbal_manager_control_pitchyaw_t* storm32_gimbal_manager_control_pitchyaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_storm32_gimbal_manager_control_pitchyaw_send(chan, storm32_gimbal_manager_control_pitchyaw->target_system, storm32_gimbal_manager_control_pitchyaw->target_component, storm32_gimbal_manager_control_pitchyaw->gimbal_id, storm32_gimbal_manager_control_pitchyaw->client, storm32_gimbal_manager_control_pitchyaw->device_flags, storm32_gimbal_manager_control_pitchyaw->manager_flags, storm32_gimbal_manager_control_pitchyaw->pitch, storm32_gimbal_manager_control_pitchyaw->yaw, storm32_gimbal_manager_control_pitchyaw->pitch_rate, storm32_gimbal_manager_control_pitchyaw->yaw_rate);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW, (const char *)storm32_gimbal_manager_control_pitchyaw, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_CRC);
#endif
}

#if MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_storm32_gimbal_manager_control_pitchyaw_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t gimbal_id, uint8_t client, uint16_t device_flags, uint16_t manager_flags, float pitch, float yaw, float pitch_rate, float yaw_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, pitch);
    _mav_put_float(buf, 4, yaw);
    _mav_put_float(buf, 8, pitch_rate);
    _mav_put_float(buf, 12, yaw_rate);
    _mav_put_uint16_t(buf, 16, device_flags);
    _mav_put_uint16_t(buf, 18, manager_flags);
    _mav_put_uint8_t(buf, 20, target_system);
    _mav_put_uint8_t(buf, 21, target_component);
    _mav_put_uint8_t(buf, 22, gimbal_id);
    _mav_put_uint8_t(buf, 23, client);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW, buf, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_CRC);
#else
    mavlink_storm32_gimbal_manager_control_pitchyaw_t *packet = (mavlink_storm32_gimbal_manager_control_pitchyaw_t *)msgbuf;
    packet->pitch = pitch;
    packet->yaw = yaw;
    packet->pitch_rate = pitch_rate;
    packet->yaw_rate = yaw_rate;
    packet->device_flags = device_flags;
    packet->manager_flags = manager_flags;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->gimbal_id = gimbal_id;
    packet->client = client;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW, (const char *)packet, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_CRC);
#endif
}
#endif

#endif

// MESSAGE STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW UNPACKING


/**
 * @brief Get field target_system from storm32_gimbal_manager_control_pitchyaw message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_storm32_gimbal_manager_control_pitchyaw_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field target_component from storm32_gimbal_manager_control_pitchyaw message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_storm32_gimbal_manager_control_pitchyaw_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Get field gimbal_id from storm32_gimbal_manager_control_pitchyaw message
 *
 * @return  Gimbal ID of the gimbal manager to address (component ID or 1-6 for non-MAVLink gimbal, 0 for all gimbals, send command multiple times for more than one but not all gimbals).
 */
static inline uint8_t mavlink_msg_storm32_gimbal_manager_control_pitchyaw_get_gimbal_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Get field client from storm32_gimbal_manager_control_pitchyaw message
 *
 * @return  Client which is contacting the gimbal manager (must be set).
 */
static inline uint8_t mavlink_msg_storm32_gimbal_manager_control_pitchyaw_get_client(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  23);
}

/**
 * @brief Get field device_flags from storm32_gimbal_manager_control_pitchyaw message
 *
 * @return  Gimbal device flags (UINT16_MAX to be ignored).
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_control_pitchyaw_get_device_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field manager_flags from storm32_gimbal_manager_control_pitchyaw message
 *
 * @return  Gimbal manager flags (0 to be ignored).
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_control_pitchyaw_get_manager_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  18);
}

/**
 * @brief Get field pitch from storm32_gimbal_manager_control_pitchyaw message
 *
 * @return [rad] Pitch/tilt angle (positive: tilt up, NaN to be ignored).
 */
static inline float mavlink_msg_storm32_gimbal_manager_control_pitchyaw_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field yaw from storm32_gimbal_manager_control_pitchyaw message
 *
 * @return [rad] Yaw/pan angle (positive: pan the right, the frame is determined by the STORM32_GIMBAL_DEVICE_FLAGS_YAW_ABSOLUTE flag, NaN to be ignored).
 */
static inline float mavlink_msg_storm32_gimbal_manager_control_pitchyaw_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field pitch_rate from storm32_gimbal_manager_control_pitchyaw message
 *
 * @return [rad/s] Pitch/tilt angular rate (positive: tilt up, NaN to be ignored).
 */
static inline float mavlink_msg_storm32_gimbal_manager_control_pitchyaw_get_pitch_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field yaw_rate from storm32_gimbal_manager_control_pitchyaw message
 *
 * @return [rad/s] Yaw/pan angular rate (positive: pan to the right, the frame is determined by the STORM32_GIMBAL_DEVICE_FLAGS_YAW_ABSOLUTE flag, NaN to be ignored).
 */
static inline float mavlink_msg_storm32_gimbal_manager_control_pitchyaw_get_yaw_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a storm32_gimbal_manager_control_pitchyaw message into a struct
 *
 * @param msg The message to decode
 * @param storm32_gimbal_manager_control_pitchyaw C-struct to decode the message contents into
 */
static inline void mavlink_msg_storm32_gimbal_manager_control_pitchyaw_decode(const mavlink_message_t* msg, mavlink_storm32_gimbal_manager_control_pitchyaw_t* storm32_gimbal_manager_control_pitchyaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    storm32_gimbal_manager_control_pitchyaw->pitch = mavlink_msg_storm32_gimbal_manager_control_pitchyaw_get_pitch(msg);
    storm32_gimbal_manager_control_pitchyaw->yaw = mavlink_msg_storm32_gimbal_manager_control_pitchyaw_get_yaw(msg);
    storm32_gimbal_manager_control_pitchyaw->pitch_rate = mavlink_msg_storm32_gimbal_manager_control_pitchyaw_get_pitch_rate(msg);
    storm32_gimbal_manager_control_pitchyaw->yaw_rate = mavlink_msg_storm32_gimbal_manager_control_pitchyaw_get_yaw_rate(msg);
    storm32_gimbal_manager_control_pitchyaw->device_flags = mavlink_msg_storm32_gimbal_manager_control_pitchyaw_get_device_flags(msg);
    storm32_gimbal_manager_control_pitchyaw->manager_flags = mavlink_msg_storm32_gimbal_manager_control_pitchyaw_get_manager_flags(msg);
    storm32_gimbal_manager_control_pitchyaw->target_system = mavlink_msg_storm32_gimbal_manager_control_pitchyaw_get_target_system(msg);
    storm32_gimbal_manager_control_pitchyaw->target_component = mavlink_msg_storm32_gimbal_manager_control_pitchyaw_get_target_component(msg);
    storm32_gimbal_manager_control_pitchyaw->gimbal_id = mavlink_msg_storm32_gimbal_manager_control_pitchyaw_get_gimbal_id(msg);
    storm32_gimbal_manager_control_pitchyaw->client = mavlink_msg_storm32_gimbal_manager_control_pitchyaw_get_client(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_LEN? msg->len : MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_LEN;
        memset(storm32_gimbal_manager_control_pitchyaw, 0, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW_LEN);
    memcpy(storm32_gimbal_manager_control_pitchyaw, _MAV_PAYLOAD(msg), len);
#endif
}
