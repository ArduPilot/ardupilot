#pragma once
// MESSAGE GIMBAL_MANAGER_SET_PITCHYAW PACKING

#define MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW 287


typedef struct __mavlink_gimbal_manager_set_pitchyaw_t {
 uint32_t flags; /*<  High level gimbal manager flags to use.*/
 float pitch; /*< [rad] Pitch angle (positive: up, negative: down, NaN to be ignored).*/
 float yaw; /*< [rad] Yaw angle (positive: to the right, negative: to the left, NaN to be ignored).*/
 float pitch_rate; /*< [rad/s] Pitch angular rate (positive: up, negative: down, NaN to be ignored).*/
 float yaw_rate; /*< [rad/s] Yaw angular rate (positive: to the right, negative: to the left, NaN to be ignored).*/
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
 uint8_t gimbal_device_id; /*<  Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).*/
} mavlink_gimbal_manager_set_pitchyaw_t;

#define MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_LEN 23
#define MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_MIN_LEN 23
#define MAVLINK_MSG_ID_287_LEN 23
#define MAVLINK_MSG_ID_287_MIN_LEN 23

#define MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_CRC 1
#define MAVLINK_MSG_ID_287_CRC 1



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GIMBAL_MANAGER_SET_PITCHYAW { \
    287, \
    "GIMBAL_MANAGER_SET_PITCHYAW", \
    8, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_gimbal_manager_set_pitchyaw_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_gimbal_manager_set_pitchyaw_t, target_component) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_gimbal_manager_set_pitchyaw_t, flags) }, \
         { "gimbal_device_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_gimbal_manager_set_pitchyaw_t, gimbal_device_id) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_gimbal_manager_set_pitchyaw_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_gimbal_manager_set_pitchyaw_t, yaw) }, \
         { "pitch_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_gimbal_manager_set_pitchyaw_t, pitch_rate) }, \
         { "yaw_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_gimbal_manager_set_pitchyaw_t, yaw_rate) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GIMBAL_MANAGER_SET_PITCHYAW { \
    "GIMBAL_MANAGER_SET_PITCHYAW", \
    8, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_gimbal_manager_set_pitchyaw_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_gimbal_manager_set_pitchyaw_t, target_component) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_gimbal_manager_set_pitchyaw_t, flags) }, \
         { "gimbal_device_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_gimbal_manager_set_pitchyaw_t, gimbal_device_id) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_gimbal_manager_set_pitchyaw_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_gimbal_manager_set_pitchyaw_t, yaw) }, \
         { "pitch_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_gimbal_manager_set_pitchyaw_t, pitch_rate) }, \
         { "yaw_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_gimbal_manager_set_pitchyaw_t, yaw_rate) }, \
         } \
}
#endif

/**
 * @brief Pack a gimbal_manager_set_pitchyaw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param flags  High level gimbal manager flags to use.
 * @param gimbal_device_id  Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
 * @param pitch [rad] Pitch angle (positive: up, negative: down, NaN to be ignored).
 * @param yaw [rad] Yaw angle (positive: to the right, negative: to the left, NaN to be ignored).
 * @param pitch_rate [rad/s] Pitch angular rate (positive: up, negative: down, NaN to be ignored).
 * @param yaw_rate [rad/s] Yaw angular rate (positive: to the right, negative: to the left, NaN to be ignored).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_manager_set_pitchyaw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint32_t flags, uint8_t gimbal_device_id, float pitch, float yaw, float pitch_rate, float yaw_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_LEN];
    _mav_put_uint32_t(buf, 0, flags);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);
    _mav_put_float(buf, 12, pitch_rate);
    _mav_put_float(buf, 16, yaw_rate);
    _mav_put_uint8_t(buf, 20, target_system);
    _mav_put_uint8_t(buf, 21, target_component);
    _mav_put_uint8_t(buf, 22, gimbal_device_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_LEN);
#else
    mavlink_gimbal_manager_set_pitchyaw_t packet;
    packet.flags = flags;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.pitch_rate = pitch_rate;
    packet.yaw_rate = yaw_rate;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.gimbal_device_id = gimbal_device_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_CRC);
}

/**
 * @brief Pack a gimbal_manager_set_pitchyaw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param flags  High level gimbal manager flags to use.
 * @param gimbal_device_id  Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
 * @param pitch [rad] Pitch angle (positive: up, negative: down, NaN to be ignored).
 * @param yaw [rad] Yaw angle (positive: to the right, negative: to the left, NaN to be ignored).
 * @param pitch_rate [rad/s] Pitch angular rate (positive: up, negative: down, NaN to be ignored).
 * @param yaw_rate [rad/s] Yaw angular rate (positive: to the right, negative: to the left, NaN to be ignored).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_manager_set_pitchyaw_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint32_t flags, uint8_t gimbal_device_id, float pitch, float yaw, float pitch_rate, float yaw_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_LEN];
    _mav_put_uint32_t(buf, 0, flags);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);
    _mav_put_float(buf, 12, pitch_rate);
    _mav_put_float(buf, 16, yaw_rate);
    _mav_put_uint8_t(buf, 20, target_system);
    _mav_put_uint8_t(buf, 21, target_component);
    _mav_put_uint8_t(buf, 22, gimbal_device_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_LEN);
#else
    mavlink_gimbal_manager_set_pitchyaw_t packet;
    packet.flags = flags;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.pitch_rate = pitch_rate;
    packet.yaw_rate = yaw_rate;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.gimbal_device_id = gimbal_device_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_LEN);
#endif
}

/**
 * @brief Pack a gimbal_manager_set_pitchyaw message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param flags  High level gimbal manager flags to use.
 * @param gimbal_device_id  Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
 * @param pitch [rad] Pitch angle (positive: up, negative: down, NaN to be ignored).
 * @param yaw [rad] Yaw angle (positive: to the right, negative: to the left, NaN to be ignored).
 * @param pitch_rate [rad/s] Pitch angular rate (positive: up, negative: down, NaN to be ignored).
 * @param yaw_rate [rad/s] Yaw angular rate (positive: to the right, negative: to the left, NaN to be ignored).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_manager_set_pitchyaw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint32_t flags,uint8_t gimbal_device_id,float pitch,float yaw,float pitch_rate,float yaw_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_LEN];
    _mav_put_uint32_t(buf, 0, flags);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);
    _mav_put_float(buf, 12, pitch_rate);
    _mav_put_float(buf, 16, yaw_rate);
    _mav_put_uint8_t(buf, 20, target_system);
    _mav_put_uint8_t(buf, 21, target_component);
    _mav_put_uint8_t(buf, 22, gimbal_device_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_LEN);
#else
    mavlink_gimbal_manager_set_pitchyaw_t packet;
    packet.flags = flags;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.pitch_rate = pitch_rate;
    packet.yaw_rate = yaw_rate;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.gimbal_device_id = gimbal_device_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_CRC);
}

/**
 * @brief Encode a gimbal_manager_set_pitchyaw struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_manager_set_pitchyaw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_manager_set_pitchyaw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gimbal_manager_set_pitchyaw_t* gimbal_manager_set_pitchyaw)
{
    return mavlink_msg_gimbal_manager_set_pitchyaw_pack(system_id, component_id, msg, gimbal_manager_set_pitchyaw->target_system, gimbal_manager_set_pitchyaw->target_component, gimbal_manager_set_pitchyaw->flags, gimbal_manager_set_pitchyaw->gimbal_device_id, gimbal_manager_set_pitchyaw->pitch, gimbal_manager_set_pitchyaw->yaw, gimbal_manager_set_pitchyaw->pitch_rate, gimbal_manager_set_pitchyaw->yaw_rate);
}

/**
 * @brief Encode a gimbal_manager_set_pitchyaw struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_manager_set_pitchyaw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_manager_set_pitchyaw_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gimbal_manager_set_pitchyaw_t* gimbal_manager_set_pitchyaw)
{
    return mavlink_msg_gimbal_manager_set_pitchyaw_pack_chan(system_id, component_id, chan, msg, gimbal_manager_set_pitchyaw->target_system, gimbal_manager_set_pitchyaw->target_component, gimbal_manager_set_pitchyaw->flags, gimbal_manager_set_pitchyaw->gimbal_device_id, gimbal_manager_set_pitchyaw->pitch, gimbal_manager_set_pitchyaw->yaw, gimbal_manager_set_pitchyaw->pitch_rate, gimbal_manager_set_pitchyaw->yaw_rate);
}

/**
 * @brief Encode a gimbal_manager_set_pitchyaw struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_manager_set_pitchyaw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_manager_set_pitchyaw_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_gimbal_manager_set_pitchyaw_t* gimbal_manager_set_pitchyaw)
{
    return mavlink_msg_gimbal_manager_set_pitchyaw_pack_status(system_id, component_id, _status, msg,  gimbal_manager_set_pitchyaw->target_system, gimbal_manager_set_pitchyaw->target_component, gimbal_manager_set_pitchyaw->flags, gimbal_manager_set_pitchyaw->gimbal_device_id, gimbal_manager_set_pitchyaw->pitch, gimbal_manager_set_pitchyaw->yaw, gimbal_manager_set_pitchyaw->pitch_rate, gimbal_manager_set_pitchyaw->yaw_rate);
}

/**
 * @brief Send a gimbal_manager_set_pitchyaw message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param flags  High level gimbal manager flags to use.
 * @param gimbal_device_id  Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
 * @param pitch [rad] Pitch angle (positive: up, negative: down, NaN to be ignored).
 * @param yaw [rad] Yaw angle (positive: to the right, negative: to the left, NaN to be ignored).
 * @param pitch_rate [rad/s] Pitch angular rate (positive: up, negative: down, NaN to be ignored).
 * @param yaw_rate [rad/s] Yaw angular rate (positive: to the right, negative: to the left, NaN to be ignored).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gimbal_manager_set_pitchyaw_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint32_t flags, uint8_t gimbal_device_id, float pitch, float yaw, float pitch_rate, float yaw_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_LEN];
    _mav_put_uint32_t(buf, 0, flags);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);
    _mav_put_float(buf, 12, pitch_rate);
    _mav_put_float(buf, 16, yaw_rate);
    _mav_put_uint8_t(buf, 20, target_system);
    _mav_put_uint8_t(buf, 21, target_component);
    _mav_put_uint8_t(buf, 22, gimbal_device_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW, buf, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_CRC);
#else
    mavlink_gimbal_manager_set_pitchyaw_t packet;
    packet.flags = flags;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.pitch_rate = pitch_rate;
    packet.yaw_rate = yaw_rate;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.gimbal_device_id = gimbal_device_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW, (const char *)&packet, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_CRC);
#endif
}

/**
 * @brief Send a gimbal_manager_set_pitchyaw message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gimbal_manager_set_pitchyaw_send_struct(mavlink_channel_t chan, const mavlink_gimbal_manager_set_pitchyaw_t* gimbal_manager_set_pitchyaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gimbal_manager_set_pitchyaw_send(chan, gimbal_manager_set_pitchyaw->target_system, gimbal_manager_set_pitchyaw->target_component, gimbal_manager_set_pitchyaw->flags, gimbal_manager_set_pitchyaw->gimbal_device_id, gimbal_manager_set_pitchyaw->pitch, gimbal_manager_set_pitchyaw->yaw, gimbal_manager_set_pitchyaw->pitch_rate, gimbal_manager_set_pitchyaw->yaw_rate);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW, (const char *)gimbal_manager_set_pitchyaw, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_CRC);
#endif
}

#if MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gimbal_manager_set_pitchyaw_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint32_t flags, uint8_t gimbal_device_id, float pitch, float yaw, float pitch_rate, float yaw_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, flags);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);
    _mav_put_float(buf, 12, pitch_rate);
    _mav_put_float(buf, 16, yaw_rate);
    _mav_put_uint8_t(buf, 20, target_system);
    _mav_put_uint8_t(buf, 21, target_component);
    _mav_put_uint8_t(buf, 22, gimbal_device_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW, buf, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_CRC);
#else
    mavlink_gimbal_manager_set_pitchyaw_t *packet = (mavlink_gimbal_manager_set_pitchyaw_t *)msgbuf;
    packet->flags = flags;
    packet->pitch = pitch;
    packet->yaw = yaw;
    packet->pitch_rate = pitch_rate;
    packet->yaw_rate = yaw_rate;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->gimbal_device_id = gimbal_device_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW, (const char *)packet, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_LEN, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_CRC);
#endif
}
#endif

#endif

// MESSAGE GIMBAL_MANAGER_SET_PITCHYAW UNPACKING


/**
 * @brief Get field target_system from gimbal_manager_set_pitchyaw message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_gimbal_manager_set_pitchyaw_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field target_component from gimbal_manager_set_pitchyaw message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_gimbal_manager_set_pitchyaw_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Get field flags from gimbal_manager_set_pitchyaw message
 *
 * @return  High level gimbal manager flags to use.
 */
static inline uint32_t mavlink_msg_gimbal_manager_set_pitchyaw_get_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field gimbal_device_id from gimbal_manager_set_pitchyaw message
 *
 * @return  Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
 */
static inline uint8_t mavlink_msg_gimbal_manager_set_pitchyaw_get_gimbal_device_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Get field pitch from gimbal_manager_set_pitchyaw message
 *
 * @return [rad] Pitch angle (positive: up, negative: down, NaN to be ignored).
 */
static inline float mavlink_msg_gimbal_manager_set_pitchyaw_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field yaw from gimbal_manager_set_pitchyaw message
 *
 * @return [rad] Yaw angle (positive: to the right, negative: to the left, NaN to be ignored).
 */
static inline float mavlink_msg_gimbal_manager_set_pitchyaw_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field pitch_rate from gimbal_manager_set_pitchyaw message
 *
 * @return [rad/s] Pitch angular rate (positive: up, negative: down, NaN to be ignored).
 */
static inline float mavlink_msg_gimbal_manager_set_pitchyaw_get_pitch_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field yaw_rate from gimbal_manager_set_pitchyaw message
 *
 * @return [rad/s] Yaw angular rate (positive: to the right, negative: to the left, NaN to be ignored).
 */
static inline float mavlink_msg_gimbal_manager_set_pitchyaw_get_yaw_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Decode a gimbal_manager_set_pitchyaw message into a struct
 *
 * @param msg The message to decode
 * @param gimbal_manager_set_pitchyaw C-struct to decode the message contents into
 */
static inline void mavlink_msg_gimbal_manager_set_pitchyaw_decode(const mavlink_message_t* msg, mavlink_gimbal_manager_set_pitchyaw_t* gimbal_manager_set_pitchyaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gimbal_manager_set_pitchyaw->flags = mavlink_msg_gimbal_manager_set_pitchyaw_get_flags(msg);
    gimbal_manager_set_pitchyaw->pitch = mavlink_msg_gimbal_manager_set_pitchyaw_get_pitch(msg);
    gimbal_manager_set_pitchyaw->yaw = mavlink_msg_gimbal_manager_set_pitchyaw_get_yaw(msg);
    gimbal_manager_set_pitchyaw->pitch_rate = mavlink_msg_gimbal_manager_set_pitchyaw_get_pitch_rate(msg);
    gimbal_manager_set_pitchyaw->yaw_rate = mavlink_msg_gimbal_manager_set_pitchyaw_get_yaw_rate(msg);
    gimbal_manager_set_pitchyaw->target_system = mavlink_msg_gimbal_manager_set_pitchyaw_get_target_system(msg);
    gimbal_manager_set_pitchyaw->target_component = mavlink_msg_gimbal_manager_set_pitchyaw_get_target_component(msg);
    gimbal_manager_set_pitchyaw->gimbal_device_id = mavlink_msg_gimbal_manager_set_pitchyaw_get_gimbal_device_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_LEN? msg->len : MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_LEN;
        memset(gimbal_manager_set_pitchyaw, 0, MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW_LEN);
    memcpy(gimbal_manager_set_pitchyaw, _MAV_PAYLOAD(msg), len);
#endif
}
