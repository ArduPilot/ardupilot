#pragma once
// MESSAGE STORM32_GIMBAL_MANAGER_PROFILE PACKING

#define MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE 60015


typedef struct __mavlink_storm32_gimbal_manager_profile_t {
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
 uint8_t gimbal_id; /*<  Gimbal ID of the gimbal manager to address (component ID or 1-6 for non-MAVLink gimbal, 0 for all gimbals, send command multiple times for more than one but not all gimbals).*/
 uint8_t profile; /*<  Profile to be applied (0 = default).*/
 uint8_t priorities[8]; /*<  Priorities for custom profile.*/
 uint8_t profile_flags; /*<  Profile flags for custom profile (0 = default).*/
 uint8_t rc_timeout; /*<  Rc timeouts for custom profile (0 = infinite, in uints of 100 ms).*/
 uint8_t timeouts[8]; /*<  Timeouts for custom profile (0 = infinite, in uints of 100 ms).*/
} mavlink_storm32_gimbal_manager_profile_t;

#define MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_LEN 22
#define MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_MIN_LEN 22
#define MAVLINK_MSG_ID_60015_LEN 22
#define MAVLINK_MSG_ID_60015_MIN_LEN 22

#define MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_CRC 78
#define MAVLINK_MSG_ID_60015_CRC 78

#define MAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_FIELD_PRIORITIES_LEN 8
#define MAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_FIELD_TIMEOUTS_LEN 8

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_STORM32_GIMBAL_MANAGER_PROFILE { \
    60015, \
    "STORM32_GIMBAL_MANAGER_PROFILE", \
    8, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_storm32_gimbal_manager_profile_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_storm32_gimbal_manager_profile_t, target_component) }, \
         { "gimbal_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_storm32_gimbal_manager_profile_t, gimbal_id) }, \
         { "profile", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_storm32_gimbal_manager_profile_t, profile) }, \
         { "priorities", NULL, MAVLINK_TYPE_UINT8_T, 8, 4, offsetof(mavlink_storm32_gimbal_manager_profile_t, priorities) }, \
         { "profile_flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_storm32_gimbal_manager_profile_t, profile_flags) }, \
         { "rc_timeout", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_storm32_gimbal_manager_profile_t, rc_timeout) }, \
         { "timeouts", NULL, MAVLINK_TYPE_UINT8_T, 8, 14, offsetof(mavlink_storm32_gimbal_manager_profile_t, timeouts) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_STORM32_GIMBAL_MANAGER_PROFILE { \
    "STORM32_GIMBAL_MANAGER_PROFILE", \
    8, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_storm32_gimbal_manager_profile_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_storm32_gimbal_manager_profile_t, target_component) }, \
         { "gimbal_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_storm32_gimbal_manager_profile_t, gimbal_id) }, \
         { "profile", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_storm32_gimbal_manager_profile_t, profile) }, \
         { "priorities", NULL, MAVLINK_TYPE_UINT8_T, 8, 4, offsetof(mavlink_storm32_gimbal_manager_profile_t, priorities) }, \
         { "profile_flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_storm32_gimbal_manager_profile_t, profile_flags) }, \
         { "rc_timeout", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_storm32_gimbal_manager_profile_t, rc_timeout) }, \
         { "timeouts", NULL, MAVLINK_TYPE_UINT8_T, 8, 14, offsetof(mavlink_storm32_gimbal_manager_profile_t, timeouts) }, \
         } \
}
#endif

/**
 * @brief Pack a storm32_gimbal_manager_profile message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param gimbal_id  Gimbal ID of the gimbal manager to address (component ID or 1-6 for non-MAVLink gimbal, 0 for all gimbals, send command multiple times for more than one but not all gimbals).
 * @param profile  Profile to be applied (0 = default).
 * @param priorities  Priorities for custom profile.
 * @param profile_flags  Profile flags for custom profile (0 = default).
 * @param rc_timeout  Rc timeouts for custom profile (0 = infinite, in uints of 100 ms).
 * @param timeouts  Timeouts for custom profile (0 = infinite, in uints of 100 ms).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_profile_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t gimbal_id, uint8_t profile, const uint8_t *priorities, uint8_t profile_flags, uint8_t rc_timeout, const uint8_t *timeouts)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, gimbal_id);
    _mav_put_uint8_t(buf, 3, profile);
    _mav_put_uint8_t(buf, 12, profile_flags);
    _mav_put_uint8_t(buf, 13, rc_timeout);
    _mav_put_uint8_t_array(buf, 4, priorities, 8);
    _mav_put_uint8_t_array(buf, 14, timeouts, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_LEN);
#else
    mavlink_storm32_gimbal_manager_profile_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.gimbal_id = gimbal_id;
    packet.profile = profile;
    packet.profile_flags = profile_flags;
    packet.rc_timeout = rc_timeout;
    mav_array_memcpy(packet.priorities, priorities, sizeof(uint8_t)*8);
    mav_array_memcpy(packet.timeouts, timeouts, sizeof(uint8_t)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_CRC);
}

/**
 * @brief Pack a storm32_gimbal_manager_profile message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param gimbal_id  Gimbal ID of the gimbal manager to address (component ID or 1-6 for non-MAVLink gimbal, 0 for all gimbals, send command multiple times for more than one but not all gimbals).
 * @param profile  Profile to be applied (0 = default).
 * @param priorities  Priorities for custom profile.
 * @param profile_flags  Profile flags for custom profile (0 = default).
 * @param rc_timeout  Rc timeouts for custom profile (0 = infinite, in uints of 100 ms).
 * @param timeouts  Timeouts for custom profile (0 = infinite, in uints of 100 ms).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_profile_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t gimbal_id, uint8_t profile, const uint8_t *priorities, uint8_t profile_flags, uint8_t rc_timeout, const uint8_t *timeouts)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, gimbal_id);
    _mav_put_uint8_t(buf, 3, profile);
    _mav_put_uint8_t(buf, 12, profile_flags);
    _mav_put_uint8_t(buf, 13, rc_timeout);
    _mav_put_uint8_t_array(buf, 4, priorities, 8);
    _mav_put_uint8_t_array(buf, 14, timeouts, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_LEN);
#else
    mavlink_storm32_gimbal_manager_profile_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.gimbal_id = gimbal_id;
    packet.profile = profile;
    packet.profile_flags = profile_flags;
    packet.rc_timeout = rc_timeout;
    mav_array_memcpy(packet.priorities, priorities, sizeof(uint8_t)*8);
    mav_array_memcpy(packet.timeouts, timeouts, sizeof(uint8_t)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_LEN);
#endif
}

/**
 * @brief Pack a storm32_gimbal_manager_profile message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param gimbal_id  Gimbal ID of the gimbal manager to address (component ID or 1-6 for non-MAVLink gimbal, 0 for all gimbals, send command multiple times for more than one but not all gimbals).
 * @param profile  Profile to be applied (0 = default).
 * @param priorities  Priorities for custom profile.
 * @param profile_flags  Profile flags for custom profile (0 = default).
 * @param rc_timeout  Rc timeouts for custom profile (0 = infinite, in uints of 100 ms).
 * @param timeouts  Timeouts for custom profile (0 = infinite, in uints of 100 ms).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_profile_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint8_t gimbal_id,uint8_t profile,const uint8_t *priorities,uint8_t profile_flags,uint8_t rc_timeout,const uint8_t *timeouts)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, gimbal_id);
    _mav_put_uint8_t(buf, 3, profile);
    _mav_put_uint8_t(buf, 12, profile_flags);
    _mav_put_uint8_t(buf, 13, rc_timeout);
    _mav_put_uint8_t_array(buf, 4, priorities, 8);
    _mav_put_uint8_t_array(buf, 14, timeouts, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_LEN);
#else
    mavlink_storm32_gimbal_manager_profile_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.gimbal_id = gimbal_id;
    packet.profile = profile;
    packet.profile_flags = profile_flags;
    packet.rc_timeout = rc_timeout;
    mav_array_memcpy(packet.priorities, priorities, sizeof(uint8_t)*8);
    mav_array_memcpy(packet.timeouts, timeouts, sizeof(uint8_t)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_CRC);
}

/**
 * @brief Encode a storm32_gimbal_manager_profile struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param storm32_gimbal_manager_profile C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_profile_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_storm32_gimbal_manager_profile_t* storm32_gimbal_manager_profile)
{
    return mavlink_msg_storm32_gimbal_manager_profile_pack(system_id, component_id, msg, storm32_gimbal_manager_profile->target_system, storm32_gimbal_manager_profile->target_component, storm32_gimbal_manager_profile->gimbal_id, storm32_gimbal_manager_profile->profile, storm32_gimbal_manager_profile->priorities, storm32_gimbal_manager_profile->profile_flags, storm32_gimbal_manager_profile->rc_timeout, storm32_gimbal_manager_profile->timeouts);
}

/**
 * @brief Encode a storm32_gimbal_manager_profile struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param storm32_gimbal_manager_profile C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_profile_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_storm32_gimbal_manager_profile_t* storm32_gimbal_manager_profile)
{
    return mavlink_msg_storm32_gimbal_manager_profile_pack_chan(system_id, component_id, chan, msg, storm32_gimbal_manager_profile->target_system, storm32_gimbal_manager_profile->target_component, storm32_gimbal_manager_profile->gimbal_id, storm32_gimbal_manager_profile->profile, storm32_gimbal_manager_profile->priorities, storm32_gimbal_manager_profile->profile_flags, storm32_gimbal_manager_profile->rc_timeout, storm32_gimbal_manager_profile->timeouts);
}

/**
 * @brief Encode a storm32_gimbal_manager_profile struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param storm32_gimbal_manager_profile C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_profile_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_storm32_gimbal_manager_profile_t* storm32_gimbal_manager_profile)
{
    return mavlink_msg_storm32_gimbal_manager_profile_pack_status(system_id, component_id, _status, msg,  storm32_gimbal_manager_profile->target_system, storm32_gimbal_manager_profile->target_component, storm32_gimbal_manager_profile->gimbal_id, storm32_gimbal_manager_profile->profile, storm32_gimbal_manager_profile->priorities, storm32_gimbal_manager_profile->profile_flags, storm32_gimbal_manager_profile->rc_timeout, storm32_gimbal_manager_profile->timeouts);
}

/**
 * @brief Send a storm32_gimbal_manager_profile message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param gimbal_id  Gimbal ID of the gimbal manager to address (component ID or 1-6 for non-MAVLink gimbal, 0 for all gimbals, send command multiple times for more than one but not all gimbals).
 * @param profile  Profile to be applied (0 = default).
 * @param priorities  Priorities for custom profile.
 * @param profile_flags  Profile flags for custom profile (0 = default).
 * @param rc_timeout  Rc timeouts for custom profile (0 = infinite, in uints of 100 ms).
 * @param timeouts  Timeouts for custom profile (0 = infinite, in uints of 100 ms).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_storm32_gimbal_manager_profile_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t gimbal_id, uint8_t profile, const uint8_t *priorities, uint8_t profile_flags, uint8_t rc_timeout, const uint8_t *timeouts)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, gimbal_id);
    _mav_put_uint8_t(buf, 3, profile);
    _mav_put_uint8_t(buf, 12, profile_flags);
    _mav_put_uint8_t(buf, 13, rc_timeout);
    _mav_put_uint8_t_array(buf, 4, priorities, 8);
    _mav_put_uint8_t_array(buf, 14, timeouts, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE, buf, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_CRC);
#else
    mavlink_storm32_gimbal_manager_profile_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.gimbal_id = gimbal_id;
    packet.profile = profile;
    packet.profile_flags = profile_flags;
    packet.rc_timeout = rc_timeout;
    mav_array_memcpy(packet.priorities, priorities, sizeof(uint8_t)*8);
    mav_array_memcpy(packet.timeouts, timeouts, sizeof(uint8_t)*8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE, (const char *)&packet, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_CRC);
#endif
}

/**
 * @brief Send a storm32_gimbal_manager_profile message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_storm32_gimbal_manager_profile_send_struct(mavlink_channel_t chan, const mavlink_storm32_gimbal_manager_profile_t* storm32_gimbal_manager_profile)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_storm32_gimbal_manager_profile_send(chan, storm32_gimbal_manager_profile->target_system, storm32_gimbal_manager_profile->target_component, storm32_gimbal_manager_profile->gimbal_id, storm32_gimbal_manager_profile->profile, storm32_gimbal_manager_profile->priorities, storm32_gimbal_manager_profile->profile_flags, storm32_gimbal_manager_profile->rc_timeout, storm32_gimbal_manager_profile->timeouts);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE, (const char *)storm32_gimbal_manager_profile, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_CRC);
#endif
}

#if MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_storm32_gimbal_manager_profile_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t gimbal_id, uint8_t profile, const uint8_t *priorities, uint8_t profile_flags, uint8_t rc_timeout, const uint8_t *timeouts)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, gimbal_id);
    _mav_put_uint8_t(buf, 3, profile);
    _mav_put_uint8_t(buf, 12, profile_flags);
    _mav_put_uint8_t(buf, 13, rc_timeout);
    _mav_put_uint8_t_array(buf, 4, priorities, 8);
    _mav_put_uint8_t_array(buf, 14, timeouts, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE, buf, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_CRC);
#else
    mavlink_storm32_gimbal_manager_profile_t *packet = (mavlink_storm32_gimbal_manager_profile_t *)msgbuf;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->gimbal_id = gimbal_id;
    packet->profile = profile;
    packet->profile_flags = profile_flags;
    packet->rc_timeout = rc_timeout;
    mav_array_memcpy(packet->priorities, priorities, sizeof(uint8_t)*8);
    mav_array_memcpy(packet->timeouts, timeouts, sizeof(uint8_t)*8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE, (const char *)packet, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_CRC);
#endif
}
#endif

#endif

// MESSAGE STORM32_GIMBAL_MANAGER_PROFILE UNPACKING


/**
 * @brief Get field target_system from storm32_gimbal_manager_profile message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_storm32_gimbal_manager_profile_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field target_component from storm32_gimbal_manager_profile message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_storm32_gimbal_manager_profile_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field gimbal_id from storm32_gimbal_manager_profile message
 *
 * @return  Gimbal ID of the gimbal manager to address (component ID or 1-6 for non-MAVLink gimbal, 0 for all gimbals, send command multiple times for more than one but not all gimbals).
 */
static inline uint8_t mavlink_msg_storm32_gimbal_manager_profile_get_gimbal_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field profile from storm32_gimbal_manager_profile message
 *
 * @return  Profile to be applied (0 = default).
 */
static inline uint8_t mavlink_msg_storm32_gimbal_manager_profile_get_profile(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field priorities from storm32_gimbal_manager_profile message
 *
 * @return  Priorities for custom profile.
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_profile_get_priorities(const mavlink_message_t* msg, uint8_t *priorities)
{
    return _MAV_RETURN_uint8_t_array(msg, priorities, 8,  4);
}

/**
 * @brief Get field profile_flags from storm32_gimbal_manager_profile message
 *
 * @return  Profile flags for custom profile (0 = default).
 */
static inline uint8_t mavlink_msg_storm32_gimbal_manager_profile_get_profile_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field rc_timeout from storm32_gimbal_manager_profile message
 *
 * @return  Rc timeouts for custom profile (0 = infinite, in uints of 100 ms).
 */
static inline uint8_t mavlink_msg_storm32_gimbal_manager_profile_get_rc_timeout(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field timeouts from storm32_gimbal_manager_profile message
 *
 * @return  Timeouts for custom profile (0 = infinite, in uints of 100 ms).
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_profile_get_timeouts(const mavlink_message_t* msg, uint8_t *timeouts)
{
    return _MAV_RETURN_uint8_t_array(msg, timeouts, 8,  14);
}

/**
 * @brief Decode a storm32_gimbal_manager_profile message into a struct
 *
 * @param msg The message to decode
 * @param storm32_gimbal_manager_profile C-struct to decode the message contents into
 */
static inline void mavlink_msg_storm32_gimbal_manager_profile_decode(const mavlink_message_t* msg, mavlink_storm32_gimbal_manager_profile_t* storm32_gimbal_manager_profile)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    storm32_gimbal_manager_profile->target_system = mavlink_msg_storm32_gimbal_manager_profile_get_target_system(msg);
    storm32_gimbal_manager_profile->target_component = mavlink_msg_storm32_gimbal_manager_profile_get_target_component(msg);
    storm32_gimbal_manager_profile->gimbal_id = mavlink_msg_storm32_gimbal_manager_profile_get_gimbal_id(msg);
    storm32_gimbal_manager_profile->profile = mavlink_msg_storm32_gimbal_manager_profile_get_profile(msg);
    mavlink_msg_storm32_gimbal_manager_profile_get_priorities(msg, storm32_gimbal_manager_profile->priorities);
    storm32_gimbal_manager_profile->profile_flags = mavlink_msg_storm32_gimbal_manager_profile_get_profile_flags(msg);
    storm32_gimbal_manager_profile->rc_timeout = mavlink_msg_storm32_gimbal_manager_profile_get_rc_timeout(msg);
    mavlink_msg_storm32_gimbal_manager_profile_get_timeouts(msg, storm32_gimbal_manager_profile->timeouts);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_LEN? msg->len : MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_LEN;
        memset(storm32_gimbal_manager_profile, 0, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_LEN);
    memcpy(storm32_gimbal_manager_profile, _MAV_PAYLOAD(msg), len);
#endif
}
