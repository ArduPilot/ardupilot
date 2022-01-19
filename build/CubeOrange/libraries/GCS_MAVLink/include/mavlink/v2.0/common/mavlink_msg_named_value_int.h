#pragma once
// MESSAGE NAMED_VALUE_INT PACKING

#define MAVLINK_MSG_ID_NAMED_VALUE_INT 252


typedef struct __mavlink_named_value_int_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 int32_t value; /*<  Signed integer value*/
 char name[10]; /*<  Name of the debug variable*/
} mavlink_named_value_int_t;

#define MAVLINK_MSG_ID_NAMED_VALUE_INT_LEN 18
#define MAVLINK_MSG_ID_NAMED_VALUE_INT_MIN_LEN 18
#define MAVLINK_MSG_ID_252_LEN 18
#define MAVLINK_MSG_ID_252_MIN_LEN 18

#define MAVLINK_MSG_ID_NAMED_VALUE_INT_CRC 44
#define MAVLINK_MSG_ID_252_CRC 44

#define MAVLINK_MSG_NAMED_VALUE_INT_FIELD_NAME_LEN 10

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_NAMED_VALUE_INT { \
    252, \
    "NAMED_VALUE_INT", \
    3, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_named_value_int_t, time_boot_ms) }, \
         { "name", NULL, MAVLINK_TYPE_CHAR, 10, 8, offsetof(mavlink_named_value_int_t, name) }, \
         { "value", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_named_value_int_t, value) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_NAMED_VALUE_INT { \
    "NAMED_VALUE_INT", \
    3, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_named_value_int_t, time_boot_ms) }, \
         { "name", NULL, MAVLINK_TYPE_CHAR, 10, 8, offsetof(mavlink_named_value_int_t, name) }, \
         { "value", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_named_value_int_t, value) }, \
         } \
}
#endif

/**
 * @brief Pack a named_value_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param name  Name of the debug variable
 * @param value  Signed integer value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_named_value_int_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, const char *name, int32_t value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAMED_VALUE_INT_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, value);
    _mav_put_char_array(buf, 8, name, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAMED_VALUE_INT_LEN);
#else
    mavlink_named_value_int_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.value = value;
    mav_array_memcpy(packet.name, name, sizeof(char)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAMED_VALUE_INT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAMED_VALUE_INT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NAMED_VALUE_INT_MIN_LEN, MAVLINK_MSG_ID_NAMED_VALUE_INT_LEN, MAVLINK_MSG_ID_NAMED_VALUE_INT_CRC);
}

/**
 * @brief Pack a named_value_int message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param name  Name of the debug variable
 * @param value  Signed integer value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_named_value_int_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,const char *name,int32_t value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAMED_VALUE_INT_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, value);
    _mav_put_char_array(buf, 8, name, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAMED_VALUE_INT_LEN);
#else
    mavlink_named_value_int_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.value = value;
    mav_array_memcpy(packet.name, name, sizeof(char)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAMED_VALUE_INT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAMED_VALUE_INT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NAMED_VALUE_INT_MIN_LEN, MAVLINK_MSG_ID_NAMED_VALUE_INT_LEN, MAVLINK_MSG_ID_NAMED_VALUE_INT_CRC);
}

/**
 * @brief Encode a named_value_int struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param named_value_int C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_named_value_int_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_named_value_int_t* named_value_int)
{
    return mavlink_msg_named_value_int_pack(system_id, component_id, msg, named_value_int->time_boot_ms, named_value_int->name, named_value_int->value);
}

/**
 * @brief Encode a named_value_int struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param named_value_int C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_named_value_int_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_named_value_int_t* named_value_int)
{
    return mavlink_msg_named_value_int_pack_chan(system_id, component_id, chan, msg, named_value_int->time_boot_ms, named_value_int->name, named_value_int->value);
}

/**
 * @brief Send a named_value_int message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param name  Name of the debug variable
 * @param value  Signed integer value
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_named_value_int_send(mavlink_channel_t chan, uint32_t time_boot_ms, const char *name, int32_t value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAMED_VALUE_INT_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, value);
    _mav_put_char_array(buf, 8, name, 10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAMED_VALUE_INT, buf, MAVLINK_MSG_ID_NAMED_VALUE_INT_MIN_LEN, MAVLINK_MSG_ID_NAMED_VALUE_INT_LEN, MAVLINK_MSG_ID_NAMED_VALUE_INT_CRC);
#else
    mavlink_named_value_int_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.value = value;
    mav_array_memcpy(packet.name, name, sizeof(char)*10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAMED_VALUE_INT, (const char *)&packet, MAVLINK_MSG_ID_NAMED_VALUE_INT_MIN_LEN, MAVLINK_MSG_ID_NAMED_VALUE_INT_LEN, MAVLINK_MSG_ID_NAMED_VALUE_INT_CRC);
#endif
}

/**
 * @brief Send a named_value_int message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_named_value_int_send_struct(mavlink_channel_t chan, const mavlink_named_value_int_t* named_value_int)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_named_value_int_send(chan, named_value_int->time_boot_ms, named_value_int->name, named_value_int->value);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAMED_VALUE_INT, (const char *)named_value_int, MAVLINK_MSG_ID_NAMED_VALUE_INT_MIN_LEN, MAVLINK_MSG_ID_NAMED_VALUE_INT_LEN, MAVLINK_MSG_ID_NAMED_VALUE_INT_CRC);
#endif
}

#if MAVLINK_MSG_ID_NAMED_VALUE_INT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_named_value_int_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, const char *name, int32_t value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, value);
    _mav_put_char_array(buf, 8, name, 10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAMED_VALUE_INT, buf, MAVLINK_MSG_ID_NAMED_VALUE_INT_MIN_LEN, MAVLINK_MSG_ID_NAMED_VALUE_INT_LEN, MAVLINK_MSG_ID_NAMED_VALUE_INT_CRC);
#else
    mavlink_named_value_int_t *packet = (mavlink_named_value_int_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->value = value;
    mav_array_memcpy(packet->name, name, sizeof(char)*10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAMED_VALUE_INT, (const char *)packet, MAVLINK_MSG_ID_NAMED_VALUE_INT_MIN_LEN, MAVLINK_MSG_ID_NAMED_VALUE_INT_LEN, MAVLINK_MSG_ID_NAMED_VALUE_INT_CRC);
#endif
}
#endif

#endif

// MESSAGE NAMED_VALUE_INT UNPACKING


/**
 * @brief Get field time_boot_ms from named_value_int message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_named_value_int_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field name from named_value_int message
 *
 * @return  Name of the debug variable
 */
static inline uint16_t mavlink_msg_named_value_int_get_name(const mavlink_message_t* msg, char *name)
{
    return _MAV_RETURN_char_array(msg, name, 10,  8);
}

/**
 * @brief Get field value from named_value_int message
 *
 * @return  Signed integer value
 */
static inline int32_t mavlink_msg_named_value_int_get_value(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Decode a named_value_int message into a struct
 *
 * @param msg The message to decode
 * @param named_value_int C-struct to decode the message contents into
 */
static inline void mavlink_msg_named_value_int_decode(const mavlink_message_t* msg, mavlink_named_value_int_t* named_value_int)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    named_value_int->time_boot_ms = mavlink_msg_named_value_int_get_time_boot_ms(msg);
    named_value_int->value = mavlink_msg_named_value_int_get_value(msg);
    mavlink_msg_named_value_int_get_name(msg, named_value_int->name);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_NAMED_VALUE_INT_LEN? msg->len : MAVLINK_MSG_ID_NAMED_VALUE_INT_LEN;
        memset(named_value_int, 0, MAVLINK_MSG_ID_NAMED_VALUE_INT_LEN);
    memcpy(named_value_int, _MAV_PAYLOAD(msg), len);
#endif
}
