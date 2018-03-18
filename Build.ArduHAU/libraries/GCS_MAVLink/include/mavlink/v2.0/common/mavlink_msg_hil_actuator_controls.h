#pragma once
// MESSAGE HIL_ACTUATOR_CONTROLS PACKING

#define MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS 93

MAVPACKED(
typedef struct __mavlink_hil_actuator_controls_t {
 uint64_t time_usec; /*< Timestamp (microseconds since UNIX epoch or microseconds since system boot)*/
 uint64_t flags; /*< Flags as bitfield, reserved for future use.*/
 float controls[16]; /*< Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.*/
 uint8_t mode; /*< System mode (MAV_MODE), includes arming state.*/
}) mavlink_hil_actuator_controls_t;

#define MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN 81
#define MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_MIN_LEN 81
#define MAVLINK_MSG_ID_93_LEN 81
#define MAVLINK_MSG_ID_93_MIN_LEN 81

#define MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_CRC 47
#define MAVLINK_MSG_ID_93_CRC 47

#define MAVLINK_MSG_HIL_ACTUATOR_CONTROLS_FIELD_CONTROLS_LEN 16

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_HIL_ACTUATOR_CONTROLS { \
    93, \
    "HIL_ACTUATOR_CONTROLS", \
    4, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_hil_actuator_controls_t, time_usec) }, \
         { "controls", NULL, MAVLINK_TYPE_FLOAT, 16, 16, offsetof(mavlink_hil_actuator_controls_t, controls) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 80, offsetof(mavlink_hil_actuator_controls_t, mode) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_hil_actuator_controls_t, flags) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_HIL_ACTUATOR_CONTROLS { \
    "HIL_ACTUATOR_CONTROLS", \
    4, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_hil_actuator_controls_t, time_usec) }, \
         { "controls", NULL, MAVLINK_TYPE_FLOAT, 16, 16, offsetof(mavlink_hil_actuator_controls_t, controls) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 80, offsetof(mavlink_hil_actuator_controls_t, mode) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_hil_actuator_controls_t, flags) }, \
         } \
}
#endif

/**
 * @brief Pack a hil_actuator_controls message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param controls Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
 * @param mode System mode (MAV_MODE), includes arming state.
 * @param flags Flags as bitfield, reserved for future use.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hil_actuator_controls_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, const float *controls, uint8_t mode, uint64_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint64_t(buf, 8, flags);
    _mav_put_uint8_t(buf, 80, mode);
    _mav_put_float_array(buf, 16, controls, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN);
#else
    mavlink_hil_actuator_controls_t packet;
    packet.time_usec = time_usec;
    packet.flags = flags;
    packet.mode = mode;
    mav_array_memcpy(packet.controls, controls, sizeof(float)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_MIN_LEN, MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN, MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_CRC);
}

/**
 * @brief Pack a hil_actuator_controls message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param controls Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
 * @param mode System mode (MAV_MODE), includes arming state.
 * @param flags Flags as bitfield, reserved for future use.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hil_actuator_controls_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,const float *controls,uint8_t mode,uint64_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint64_t(buf, 8, flags);
    _mav_put_uint8_t(buf, 80, mode);
    _mav_put_float_array(buf, 16, controls, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN);
#else
    mavlink_hil_actuator_controls_t packet;
    packet.time_usec = time_usec;
    packet.flags = flags;
    packet.mode = mode;
    mav_array_memcpy(packet.controls, controls, sizeof(float)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_MIN_LEN, MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN, MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_CRC);
}

/**
 * @brief Encode a hil_actuator_controls struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param hil_actuator_controls C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hil_actuator_controls_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_hil_actuator_controls_t* hil_actuator_controls)
{
    return mavlink_msg_hil_actuator_controls_pack(system_id, component_id, msg, hil_actuator_controls->time_usec, hil_actuator_controls->controls, hil_actuator_controls->mode, hil_actuator_controls->flags);
}

/**
 * @brief Encode a hil_actuator_controls struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param hil_actuator_controls C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hil_actuator_controls_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_hil_actuator_controls_t* hil_actuator_controls)
{
    return mavlink_msg_hil_actuator_controls_pack_chan(system_id, component_id, chan, msg, hil_actuator_controls->time_usec, hil_actuator_controls->controls, hil_actuator_controls->mode, hil_actuator_controls->flags);
}

/**
 * @brief Send a hil_actuator_controls message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param controls Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
 * @param mode System mode (MAV_MODE), includes arming state.
 * @param flags Flags as bitfield, reserved for future use.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_hil_actuator_controls_send(mavlink_channel_t chan, uint64_t time_usec, const float *controls, uint8_t mode, uint64_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint64_t(buf, 8, flags);
    _mav_put_uint8_t(buf, 80, mode);
    _mav_put_float_array(buf, 16, controls, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS, buf, MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_MIN_LEN, MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN, MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_CRC);
#else
    mavlink_hil_actuator_controls_t packet;
    packet.time_usec = time_usec;
    packet.flags = flags;
    packet.mode = mode;
    mav_array_memcpy(packet.controls, controls, sizeof(float)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS, (const char *)&packet, MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_MIN_LEN, MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN, MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_CRC);
#endif
}

/**
 * @brief Send a hil_actuator_controls message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_hil_actuator_controls_send_struct(mavlink_channel_t chan, const mavlink_hil_actuator_controls_t* hil_actuator_controls)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_hil_actuator_controls_send(chan, hil_actuator_controls->time_usec, hil_actuator_controls->controls, hil_actuator_controls->mode, hil_actuator_controls->flags);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS, (const char *)hil_actuator_controls, MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_MIN_LEN, MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN, MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_CRC);
#endif
}

#if MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_hil_actuator_controls_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, const float *controls, uint8_t mode, uint64_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint64_t(buf, 8, flags);
    _mav_put_uint8_t(buf, 80, mode);
    _mav_put_float_array(buf, 16, controls, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS, buf, MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_MIN_LEN, MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN, MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_CRC);
#else
    mavlink_hil_actuator_controls_t *packet = (mavlink_hil_actuator_controls_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->flags = flags;
    packet->mode = mode;
    mav_array_memcpy(packet->controls, controls, sizeof(float)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS, (const char *)packet, MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_MIN_LEN, MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN, MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_CRC);
#endif
}
#endif

#endif

// MESSAGE HIL_ACTUATOR_CONTROLS UNPACKING


/**
 * @brief Get field time_usec from hil_actuator_controls message
 *
 * @return Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 */
static inline uint64_t mavlink_msg_hil_actuator_controls_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field controls from hil_actuator_controls message
 *
 * @return Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
 */
static inline uint16_t mavlink_msg_hil_actuator_controls_get_controls(const mavlink_message_t* msg, float *controls)
{
    return _MAV_RETURN_float_array(msg, controls, 16,  16);
}

/**
 * @brief Get field mode from hil_actuator_controls message
 *
 * @return System mode (MAV_MODE), includes arming state.
 */
static inline uint8_t mavlink_msg_hil_actuator_controls_get_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  80);
}

/**
 * @brief Get field flags from hil_actuator_controls message
 *
 * @return Flags as bitfield, reserved for future use.
 */
static inline uint64_t mavlink_msg_hil_actuator_controls_get_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Decode a hil_actuator_controls message into a struct
 *
 * @param msg The message to decode
 * @param hil_actuator_controls C-struct to decode the message contents into
 */
static inline void mavlink_msg_hil_actuator_controls_decode(const mavlink_message_t* msg, mavlink_hil_actuator_controls_t* hil_actuator_controls)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    hil_actuator_controls->time_usec = mavlink_msg_hil_actuator_controls_get_time_usec(msg);
    hil_actuator_controls->flags = mavlink_msg_hil_actuator_controls_get_flags(msg);
    mavlink_msg_hil_actuator_controls_get_controls(msg, hil_actuator_controls->controls);
    hil_actuator_controls->mode = mavlink_msg_hil_actuator_controls_get_mode(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN? msg->len : MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN;
        memset(hil_actuator_controls, 0, MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN);
    memcpy(hil_actuator_controls, _MAV_PAYLOAD(msg), len);
#endif
}
