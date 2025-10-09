#pragma once
// MESSAGE MOUNT_CONTROL PACKING

#define MAVLINK_MSG_ID_MOUNT_CONTROL 157


typedef struct __mavlink_mount_control_t {
 int32_t input_a; /*<  Pitch (centi-degrees) or lat (degE7), depending on mount mode.*/
 int32_t input_b; /*<  Roll (centi-degrees) or lon (degE7) depending on mount mode.*/
 int32_t input_c; /*<  Yaw (centi-degrees) or alt (cm) depending on mount mode.*/
 uint8_t target_system; /*<  System ID.*/
 uint8_t target_component; /*<  Component ID.*/
 uint8_t save_position; /*<  If "1" it will save current trimmed position on EEPROM (just valid for NEUTRAL and LANDING).*/
} mavlink_mount_control_t;

#define MAVLINK_MSG_ID_MOUNT_CONTROL_LEN 15
#define MAVLINK_MSG_ID_MOUNT_CONTROL_MIN_LEN 15
#define MAVLINK_MSG_ID_157_LEN 15
#define MAVLINK_MSG_ID_157_MIN_LEN 15

#define MAVLINK_MSG_ID_MOUNT_CONTROL_CRC 21
#define MAVLINK_MSG_ID_157_CRC 21



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MOUNT_CONTROL { \
    157, \
    "MOUNT_CONTROL", \
    6, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_mount_control_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_mount_control_t, target_component) }, \
         { "input_a", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_mount_control_t, input_a) }, \
         { "input_b", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_mount_control_t, input_b) }, \
         { "input_c", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_mount_control_t, input_c) }, \
         { "save_position", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_mount_control_t, save_position) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MOUNT_CONTROL { \
    "MOUNT_CONTROL", \
    6, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_mount_control_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_mount_control_t, target_component) }, \
         { "input_a", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_mount_control_t, input_a) }, \
         { "input_b", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_mount_control_t, input_b) }, \
         { "input_c", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_mount_control_t, input_c) }, \
         { "save_position", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_mount_control_t, save_position) }, \
         } \
}
#endif

/**
 * @brief Pack a mount_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param input_a  Pitch (centi-degrees) or lat (degE7), depending on mount mode.
 * @param input_b  Roll (centi-degrees) or lon (degE7) depending on mount mode.
 * @param input_c  Yaw (centi-degrees) or alt (cm) depending on mount mode.
 * @param save_position  If "1" it will save current trimmed position on EEPROM (just valid for NEUTRAL and LANDING).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mount_control_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, int32_t input_a, int32_t input_b, int32_t input_c, uint8_t save_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOUNT_CONTROL_LEN];
    _mav_put_int32_t(buf, 0, input_a);
    _mav_put_int32_t(buf, 4, input_b);
    _mav_put_int32_t(buf, 8, input_c);
    _mav_put_uint8_t(buf, 12, target_system);
    _mav_put_uint8_t(buf, 13, target_component);
    _mav_put_uint8_t(buf, 14, save_position);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOUNT_CONTROL_LEN);
#else
    mavlink_mount_control_t packet;
    packet.input_a = input_a;
    packet.input_b = input_b;
    packet.input_c = input_c;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.save_position = save_position;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOUNT_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOUNT_CONTROL;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MOUNT_CONTROL_MIN_LEN, MAVLINK_MSG_ID_MOUNT_CONTROL_LEN, MAVLINK_MSG_ID_MOUNT_CONTROL_CRC);
}

/**
 * @brief Pack a mount_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param input_a  Pitch (centi-degrees) or lat (degE7), depending on mount mode.
 * @param input_b  Roll (centi-degrees) or lon (degE7) depending on mount mode.
 * @param input_c  Yaw (centi-degrees) or alt (cm) depending on mount mode.
 * @param save_position  If "1" it will save current trimmed position on EEPROM (just valid for NEUTRAL and LANDING).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mount_control_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, int32_t input_a, int32_t input_b, int32_t input_c, uint8_t save_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOUNT_CONTROL_LEN];
    _mav_put_int32_t(buf, 0, input_a);
    _mav_put_int32_t(buf, 4, input_b);
    _mav_put_int32_t(buf, 8, input_c);
    _mav_put_uint8_t(buf, 12, target_system);
    _mav_put_uint8_t(buf, 13, target_component);
    _mav_put_uint8_t(buf, 14, save_position);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOUNT_CONTROL_LEN);
#else
    mavlink_mount_control_t packet;
    packet.input_a = input_a;
    packet.input_b = input_b;
    packet.input_c = input_c;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.save_position = save_position;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOUNT_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOUNT_CONTROL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MOUNT_CONTROL_MIN_LEN, MAVLINK_MSG_ID_MOUNT_CONTROL_LEN, MAVLINK_MSG_ID_MOUNT_CONTROL_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MOUNT_CONTROL_MIN_LEN, MAVLINK_MSG_ID_MOUNT_CONTROL_LEN);
#endif
}

/**
 * @brief Pack a mount_control message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param input_a  Pitch (centi-degrees) or lat (degE7), depending on mount mode.
 * @param input_b  Roll (centi-degrees) or lon (degE7) depending on mount mode.
 * @param input_c  Yaw (centi-degrees) or alt (cm) depending on mount mode.
 * @param save_position  If "1" it will save current trimmed position on EEPROM (just valid for NEUTRAL and LANDING).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mount_control_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,int32_t input_a,int32_t input_b,int32_t input_c,uint8_t save_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOUNT_CONTROL_LEN];
    _mav_put_int32_t(buf, 0, input_a);
    _mav_put_int32_t(buf, 4, input_b);
    _mav_put_int32_t(buf, 8, input_c);
    _mav_put_uint8_t(buf, 12, target_system);
    _mav_put_uint8_t(buf, 13, target_component);
    _mav_put_uint8_t(buf, 14, save_position);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOUNT_CONTROL_LEN);
#else
    mavlink_mount_control_t packet;
    packet.input_a = input_a;
    packet.input_b = input_b;
    packet.input_c = input_c;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.save_position = save_position;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOUNT_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOUNT_CONTROL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MOUNT_CONTROL_MIN_LEN, MAVLINK_MSG_ID_MOUNT_CONTROL_LEN, MAVLINK_MSG_ID_MOUNT_CONTROL_CRC);
}

/**
 * @brief Encode a mount_control struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mount_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mount_control_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mount_control_t* mount_control)
{
    return mavlink_msg_mount_control_pack(system_id, component_id, msg, mount_control->target_system, mount_control->target_component, mount_control->input_a, mount_control->input_b, mount_control->input_c, mount_control->save_position);
}

/**
 * @brief Encode a mount_control struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mount_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mount_control_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mount_control_t* mount_control)
{
    return mavlink_msg_mount_control_pack_chan(system_id, component_id, chan, msg, mount_control->target_system, mount_control->target_component, mount_control->input_a, mount_control->input_b, mount_control->input_c, mount_control->save_position);
}

/**
 * @brief Encode a mount_control struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param mount_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mount_control_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_mount_control_t* mount_control)
{
    return mavlink_msg_mount_control_pack_status(system_id, component_id, _status, msg,  mount_control->target_system, mount_control->target_component, mount_control->input_a, mount_control->input_b, mount_control->input_c, mount_control->save_position);
}

/**
 * @brief Send a mount_control message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param input_a  Pitch (centi-degrees) or lat (degE7), depending on mount mode.
 * @param input_b  Roll (centi-degrees) or lon (degE7) depending on mount mode.
 * @param input_c  Yaw (centi-degrees) or alt (cm) depending on mount mode.
 * @param save_position  If "1" it will save current trimmed position on EEPROM (just valid for NEUTRAL and LANDING).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mount_control_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, int32_t input_a, int32_t input_b, int32_t input_c, uint8_t save_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOUNT_CONTROL_LEN];
    _mav_put_int32_t(buf, 0, input_a);
    _mav_put_int32_t(buf, 4, input_b);
    _mav_put_int32_t(buf, 8, input_c);
    _mav_put_uint8_t(buf, 12, target_system);
    _mav_put_uint8_t(buf, 13, target_component);
    _mav_put_uint8_t(buf, 14, save_position);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOUNT_CONTROL, buf, MAVLINK_MSG_ID_MOUNT_CONTROL_MIN_LEN, MAVLINK_MSG_ID_MOUNT_CONTROL_LEN, MAVLINK_MSG_ID_MOUNT_CONTROL_CRC);
#else
    mavlink_mount_control_t packet;
    packet.input_a = input_a;
    packet.input_b = input_b;
    packet.input_c = input_c;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.save_position = save_position;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOUNT_CONTROL, (const char *)&packet, MAVLINK_MSG_ID_MOUNT_CONTROL_MIN_LEN, MAVLINK_MSG_ID_MOUNT_CONTROL_LEN, MAVLINK_MSG_ID_MOUNT_CONTROL_CRC);
#endif
}

/**
 * @brief Send a mount_control message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mount_control_send_struct(mavlink_channel_t chan, const mavlink_mount_control_t* mount_control)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mount_control_send(chan, mount_control->target_system, mount_control->target_component, mount_control->input_a, mount_control->input_b, mount_control->input_c, mount_control->save_position);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOUNT_CONTROL, (const char *)mount_control, MAVLINK_MSG_ID_MOUNT_CONTROL_MIN_LEN, MAVLINK_MSG_ID_MOUNT_CONTROL_LEN, MAVLINK_MSG_ID_MOUNT_CONTROL_CRC);
#endif
}

#if MAVLINK_MSG_ID_MOUNT_CONTROL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mount_control_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, int32_t input_a, int32_t input_b, int32_t input_c, uint8_t save_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, input_a);
    _mav_put_int32_t(buf, 4, input_b);
    _mav_put_int32_t(buf, 8, input_c);
    _mav_put_uint8_t(buf, 12, target_system);
    _mav_put_uint8_t(buf, 13, target_component);
    _mav_put_uint8_t(buf, 14, save_position);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOUNT_CONTROL, buf, MAVLINK_MSG_ID_MOUNT_CONTROL_MIN_LEN, MAVLINK_MSG_ID_MOUNT_CONTROL_LEN, MAVLINK_MSG_ID_MOUNT_CONTROL_CRC);
#else
    mavlink_mount_control_t *packet = (mavlink_mount_control_t *)msgbuf;
    packet->input_a = input_a;
    packet->input_b = input_b;
    packet->input_c = input_c;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->save_position = save_position;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOUNT_CONTROL, (const char *)packet, MAVLINK_MSG_ID_MOUNT_CONTROL_MIN_LEN, MAVLINK_MSG_ID_MOUNT_CONTROL_LEN, MAVLINK_MSG_ID_MOUNT_CONTROL_CRC);
#endif
}
#endif

#endif

// MESSAGE MOUNT_CONTROL UNPACKING


/**
 * @brief Get field target_system from mount_control message
 *
 * @return  System ID.
 */
static inline uint8_t mavlink_msg_mount_control_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field target_component from mount_control message
 *
 * @return  Component ID.
 */
static inline uint8_t mavlink_msg_mount_control_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field input_a from mount_control message
 *
 * @return  Pitch (centi-degrees) or lat (degE7), depending on mount mode.
 */
static inline int32_t mavlink_msg_mount_control_get_input_a(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field input_b from mount_control message
 *
 * @return  Roll (centi-degrees) or lon (degE7) depending on mount mode.
 */
static inline int32_t mavlink_msg_mount_control_get_input_b(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field input_c from mount_control message
 *
 * @return  Yaw (centi-degrees) or alt (cm) depending on mount mode.
 */
static inline int32_t mavlink_msg_mount_control_get_input_c(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field save_position from mount_control message
 *
 * @return  If "1" it will save current trimmed position on EEPROM (just valid for NEUTRAL and LANDING).
 */
static inline uint8_t mavlink_msg_mount_control_get_save_position(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Decode a mount_control message into a struct
 *
 * @param msg The message to decode
 * @param mount_control C-struct to decode the message contents into
 */
static inline void mavlink_msg_mount_control_decode(const mavlink_message_t* msg, mavlink_mount_control_t* mount_control)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mount_control->input_a = mavlink_msg_mount_control_get_input_a(msg);
    mount_control->input_b = mavlink_msg_mount_control_get_input_b(msg);
    mount_control->input_c = mavlink_msg_mount_control_get_input_c(msg);
    mount_control->target_system = mavlink_msg_mount_control_get_target_system(msg);
    mount_control->target_component = mavlink_msg_mount_control_get_target_component(msg);
    mount_control->save_position = mavlink_msg_mount_control_get_save_position(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MOUNT_CONTROL_LEN? msg->len : MAVLINK_MSG_ID_MOUNT_CONTROL_LEN;
        memset(mount_control, 0, MAVLINK_MSG_ID_MOUNT_CONTROL_LEN);
    memcpy(mount_control, _MAV_PAYLOAD(msg), len);
#endif
}
