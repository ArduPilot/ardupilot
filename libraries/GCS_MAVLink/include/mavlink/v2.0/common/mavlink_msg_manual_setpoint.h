#pragma once
// MESSAGE MANUAL_SETPOINT PACKING

#define MAVLINK_MSG_ID_MANUAL_SETPOINT 81


typedef struct __mavlink_manual_setpoint_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 float roll; /*< [rad/s] Desired roll rate*/
 float pitch; /*< [rad/s] Desired pitch rate*/
 float yaw; /*< [rad/s] Desired yaw rate*/
 float thrust; /*<  Collective thrust, normalized to 0 .. 1*/
 uint8_t mode_switch; /*<  Flight mode switch position, 0.. 255*/
 uint8_t manual_override_switch; /*<  Override mode switch position, 0.. 255*/
} mavlink_manual_setpoint_t;

#define MAVLINK_MSG_ID_MANUAL_SETPOINT_LEN 22
#define MAVLINK_MSG_ID_MANUAL_SETPOINT_MIN_LEN 22
#define MAVLINK_MSG_ID_81_LEN 22
#define MAVLINK_MSG_ID_81_MIN_LEN 22

#define MAVLINK_MSG_ID_MANUAL_SETPOINT_CRC 106
#define MAVLINK_MSG_ID_81_CRC 106



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MANUAL_SETPOINT { \
    81, \
    "MANUAL_SETPOINT", \
    7, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_manual_setpoint_t, time_boot_ms) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_manual_setpoint_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_manual_setpoint_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_manual_setpoint_t, yaw) }, \
         { "thrust", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_manual_setpoint_t, thrust) }, \
         { "mode_switch", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_manual_setpoint_t, mode_switch) }, \
         { "manual_override_switch", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_manual_setpoint_t, manual_override_switch) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MANUAL_SETPOINT { \
    "MANUAL_SETPOINT", \
    7, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_manual_setpoint_t, time_boot_ms) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_manual_setpoint_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_manual_setpoint_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_manual_setpoint_t, yaw) }, \
         { "thrust", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_manual_setpoint_t, thrust) }, \
         { "mode_switch", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_manual_setpoint_t, mode_switch) }, \
         { "manual_override_switch", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_manual_setpoint_t, manual_override_switch) }, \
         } \
}
#endif

/**
 * @brief Pack a manual_setpoint message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param roll [rad/s] Desired roll rate
 * @param pitch [rad/s] Desired pitch rate
 * @param yaw [rad/s] Desired yaw rate
 * @param thrust  Collective thrust, normalized to 0 .. 1
 * @param mode_switch  Flight mode switch position, 0.. 255
 * @param manual_override_switch  Override mode switch position, 0.. 255
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_manual_setpoint_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, float roll, float pitch, float yaw, float thrust, uint8_t mode_switch, uint8_t manual_override_switch)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MANUAL_SETPOINT_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, roll);
    _mav_put_float(buf, 8, pitch);
    _mav_put_float(buf, 12, yaw);
    _mav_put_float(buf, 16, thrust);
    _mav_put_uint8_t(buf, 20, mode_switch);
    _mav_put_uint8_t(buf, 21, manual_override_switch);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MANUAL_SETPOINT_LEN);
#else
    mavlink_manual_setpoint_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.thrust = thrust;
    packet.mode_switch = mode_switch;
    packet.manual_override_switch = manual_override_switch;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MANUAL_SETPOINT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MANUAL_SETPOINT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MANUAL_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_MANUAL_SETPOINT_LEN, MAVLINK_MSG_ID_MANUAL_SETPOINT_CRC);
}

/**
 * @brief Pack a manual_setpoint message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param roll [rad/s] Desired roll rate
 * @param pitch [rad/s] Desired pitch rate
 * @param yaw [rad/s] Desired yaw rate
 * @param thrust  Collective thrust, normalized to 0 .. 1
 * @param mode_switch  Flight mode switch position, 0.. 255
 * @param manual_override_switch  Override mode switch position, 0.. 255
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_manual_setpoint_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t time_boot_ms, float roll, float pitch, float yaw, float thrust, uint8_t mode_switch, uint8_t manual_override_switch)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MANUAL_SETPOINT_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, roll);
    _mav_put_float(buf, 8, pitch);
    _mav_put_float(buf, 12, yaw);
    _mav_put_float(buf, 16, thrust);
    _mav_put_uint8_t(buf, 20, mode_switch);
    _mav_put_uint8_t(buf, 21, manual_override_switch);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MANUAL_SETPOINT_LEN);
#else
    mavlink_manual_setpoint_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.thrust = thrust;
    packet.mode_switch = mode_switch;
    packet.manual_override_switch = manual_override_switch;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MANUAL_SETPOINT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MANUAL_SETPOINT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MANUAL_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_MANUAL_SETPOINT_LEN, MAVLINK_MSG_ID_MANUAL_SETPOINT_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MANUAL_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_MANUAL_SETPOINT_LEN);
#endif
}

/**
 * @brief Pack a manual_setpoint message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param roll [rad/s] Desired roll rate
 * @param pitch [rad/s] Desired pitch rate
 * @param yaw [rad/s] Desired yaw rate
 * @param thrust  Collective thrust, normalized to 0 .. 1
 * @param mode_switch  Flight mode switch position, 0.. 255
 * @param manual_override_switch  Override mode switch position, 0.. 255
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_manual_setpoint_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,float roll,float pitch,float yaw,float thrust,uint8_t mode_switch,uint8_t manual_override_switch)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MANUAL_SETPOINT_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, roll);
    _mav_put_float(buf, 8, pitch);
    _mav_put_float(buf, 12, yaw);
    _mav_put_float(buf, 16, thrust);
    _mav_put_uint8_t(buf, 20, mode_switch);
    _mav_put_uint8_t(buf, 21, manual_override_switch);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MANUAL_SETPOINT_LEN);
#else
    mavlink_manual_setpoint_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.thrust = thrust;
    packet.mode_switch = mode_switch;
    packet.manual_override_switch = manual_override_switch;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MANUAL_SETPOINT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MANUAL_SETPOINT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MANUAL_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_MANUAL_SETPOINT_LEN, MAVLINK_MSG_ID_MANUAL_SETPOINT_CRC);
}

/**
 * @brief Encode a manual_setpoint struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param manual_setpoint C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_manual_setpoint_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_manual_setpoint_t* manual_setpoint)
{
    return mavlink_msg_manual_setpoint_pack(system_id, component_id, msg, manual_setpoint->time_boot_ms, manual_setpoint->roll, manual_setpoint->pitch, manual_setpoint->yaw, manual_setpoint->thrust, manual_setpoint->mode_switch, manual_setpoint->manual_override_switch);
}

/**
 * @brief Encode a manual_setpoint struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param manual_setpoint C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_manual_setpoint_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_manual_setpoint_t* manual_setpoint)
{
    return mavlink_msg_manual_setpoint_pack_chan(system_id, component_id, chan, msg, manual_setpoint->time_boot_ms, manual_setpoint->roll, manual_setpoint->pitch, manual_setpoint->yaw, manual_setpoint->thrust, manual_setpoint->mode_switch, manual_setpoint->manual_override_switch);
}

/**
 * @brief Encode a manual_setpoint struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param manual_setpoint C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_manual_setpoint_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_manual_setpoint_t* manual_setpoint)
{
    return mavlink_msg_manual_setpoint_pack_status(system_id, component_id, _status, msg,  manual_setpoint->time_boot_ms, manual_setpoint->roll, manual_setpoint->pitch, manual_setpoint->yaw, manual_setpoint->thrust, manual_setpoint->mode_switch, manual_setpoint->manual_override_switch);
}

/**
 * @brief Send a manual_setpoint message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param roll [rad/s] Desired roll rate
 * @param pitch [rad/s] Desired pitch rate
 * @param yaw [rad/s] Desired yaw rate
 * @param thrust  Collective thrust, normalized to 0 .. 1
 * @param mode_switch  Flight mode switch position, 0.. 255
 * @param manual_override_switch  Override mode switch position, 0.. 255
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_manual_setpoint_send(mavlink_channel_t chan, uint32_t time_boot_ms, float roll, float pitch, float yaw, float thrust, uint8_t mode_switch, uint8_t manual_override_switch)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MANUAL_SETPOINT_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, roll);
    _mav_put_float(buf, 8, pitch);
    _mav_put_float(buf, 12, yaw);
    _mav_put_float(buf, 16, thrust);
    _mav_put_uint8_t(buf, 20, mode_switch);
    _mav_put_uint8_t(buf, 21, manual_override_switch);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MANUAL_SETPOINT, buf, MAVLINK_MSG_ID_MANUAL_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_MANUAL_SETPOINT_LEN, MAVLINK_MSG_ID_MANUAL_SETPOINT_CRC);
#else
    mavlink_manual_setpoint_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.thrust = thrust;
    packet.mode_switch = mode_switch;
    packet.manual_override_switch = manual_override_switch;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MANUAL_SETPOINT, (const char *)&packet, MAVLINK_MSG_ID_MANUAL_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_MANUAL_SETPOINT_LEN, MAVLINK_MSG_ID_MANUAL_SETPOINT_CRC);
#endif
}

/**
 * @brief Send a manual_setpoint message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_manual_setpoint_send_struct(mavlink_channel_t chan, const mavlink_manual_setpoint_t* manual_setpoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_manual_setpoint_send(chan, manual_setpoint->time_boot_ms, manual_setpoint->roll, manual_setpoint->pitch, manual_setpoint->yaw, manual_setpoint->thrust, manual_setpoint->mode_switch, manual_setpoint->manual_override_switch);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MANUAL_SETPOINT, (const char *)manual_setpoint, MAVLINK_MSG_ID_MANUAL_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_MANUAL_SETPOINT_LEN, MAVLINK_MSG_ID_MANUAL_SETPOINT_CRC);
#endif
}

#if MAVLINK_MSG_ID_MANUAL_SETPOINT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_manual_setpoint_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, float roll, float pitch, float yaw, float thrust, uint8_t mode_switch, uint8_t manual_override_switch)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, roll);
    _mav_put_float(buf, 8, pitch);
    _mav_put_float(buf, 12, yaw);
    _mav_put_float(buf, 16, thrust);
    _mav_put_uint8_t(buf, 20, mode_switch);
    _mav_put_uint8_t(buf, 21, manual_override_switch);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MANUAL_SETPOINT, buf, MAVLINK_MSG_ID_MANUAL_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_MANUAL_SETPOINT_LEN, MAVLINK_MSG_ID_MANUAL_SETPOINT_CRC);
#else
    mavlink_manual_setpoint_t *packet = (mavlink_manual_setpoint_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;
    packet->thrust = thrust;
    packet->mode_switch = mode_switch;
    packet->manual_override_switch = manual_override_switch;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MANUAL_SETPOINT, (const char *)packet, MAVLINK_MSG_ID_MANUAL_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_MANUAL_SETPOINT_LEN, MAVLINK_MSG_ID_MANUAL_SETPOINT_CRC);
#endif
}
#endif

#endif

// MESSAGE MANUAL_SETPOINT UNPACKING


/**
 * @brief Get field time_boot_ms from manual_setpoint message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_manual_setpoint_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field roll from manual_setpoint message
 *
 * @return [rad/s] Desired roll rate
 */
static inline float mavlink_msg_manual_setpoint_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field pitch from manual_setpoint message
 *
 * @return [rad/s] Desired pitch rate
 */
static inline float mavlink_msg_manual_setpoint_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field yaw from manual_setpoint message
 *
 * @return [rad/s] Desired yaw rate
 */
static inline float mavlink_msg_manual_setpoint_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field thrust from manual_setpoint message
 *
 * @return  Collective thrust, normalized to 0 .. 1
 */
static inline float mavlink_msg_manual_setpoint_get_thrust(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field mode_switch from manual_setpoint message
 *
 * @return  Flight mode switch position, 0.. 255
 */
static inline uint8_t mavlink_msg_manual_setpoint_get_mode_switch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field manual_override_switch from manual_setpoint message
 *
 * @return  Override mode switch position, 0.. 255
 */
static inline uint8_t mavlink_msg_manual_setpoint_get_manual_override_switch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Decode a manual_setpoint message into a struct
 *
 * @param msg The message to decode
 * @param manual_setpoint C-struct to decode the message contents into
 */
static inline void mavlink_msg_manual_setpoint_decode(const mavlink_message_t* msg, mavlink_manual_setpoint_t* manual_setpoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    manual_setpoint->time_boot_ms = mavlink_msg_manual_setpoint_get_time_boot_ms(msg);
    manual_setpoint->roll = mavlink_msg_manual_setpoint_get_roll(msg);
    manual_setpoint->pitch = mavlink_msg_manual_setpoint_get_pitch(msg);
    manual_setpoint->yaw = mavlink_msg_manual_setpoint_get_yaw(msg);
    manual_setpoint->thrust = mavlink_msg_manual_setpoint_get_thrust(msg);
    manual_setpoint->mode_switch = mavlink_msg_manual_setpoint_get_mode_switch(msg);
    manual_setpoint->manual_override_switch = mavlink_msg_manual_setpoint_get_manual_override_switch(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MANUAL_SETPOINT_LEN? msg->len : MAVLINK_MSG_ID_MANUAL_SETPOINT_LEN;
        memset(manual_setpoint, 0, MAVLINK_MSG_ID_MANUAL_SETPOINT_LEN);
    memcpy(manual_setpoint, _MAV_PAYLOAD(msg), len);
#endif
}
