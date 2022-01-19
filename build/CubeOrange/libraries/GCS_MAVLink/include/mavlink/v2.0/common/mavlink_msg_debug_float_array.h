#pragma once
// MESSAGE DEBUG_FLOAT_ARRAY PACKING

#define MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY 350


typedef struct __mavlink_debug_float_array_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 uint16_t array_id; /*<  Unique ID used to discriminate between arrays*/
 char name[10]; /*<  Name, for human-friendly display in a Ground Control Station*/
 float data[58]; /*<  data*/
} mavlink_debug_float_array_t;

#define MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN 252
#define MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_MIN_LEN 20
#define MAVLINK_MSG_ID_350_LEN 252
#define MAVLINK_MSG_ID_350_MIN_LEN 20

#define MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_CRC 232
#define MAVLINK_MSG_ID_350_CRC 232

#define MAVLINK_MSG_DEBUG_FLOAT_ARRAY_FIELD_NAME_LEN 10
#define MAVLINK_MSG_DEBUG_FLOAT_ARRAY_FIELD_DATA_LEN 58

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DEBUG_FLOAT_ARRAY { \
    350, \
    "DEBUG_FLOAT_ARRAY", \
    4, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_debug_float_array_t, time_usec) }, \
         { "name", NULL, MAVLINK_TYPE_CHAR, 10, 10, offsetof(mavlink_debug_float_array_t, name) }, \
         { "array_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_debug_float_array_t, array_id) }, \
         { "data", NULL, MAVLINK_TYPE_FLOAT, 58, 20, offsetof(mavlink_debug_float_array_t, data) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DEBUG_FLOAT_ARRAY { \
    "DEBUG_FLOAT_ARRAY", \
    4, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_debug_float_array_t, time_usec) }, \
         { "name", NULL, MAVLINK_TYPE_CHAR, 10, 10, offsetof(mavlink_debug_float_array_t, name) }, \
         { "array_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_debug_float_array_t, array_id) }, \
         { "data", NULL, MAVLINK_TYPE_FLOAT, 58, 20, offsetof(mavlink_debug_float_array_t, data) }, \
         } \
}
#endif

/**
 * @brief Pack a debug_float_array message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param name  Name, for human-friendly display in a Ground Control Station
 * @param array_id  Unique ID used to discriminate between arrays
 * @param data  data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_debug_float_array_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, const char *name, uint16_t array_id, const float *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint16_t(buf, 8, array_id);
    _mav_put_char_array(buf, 10, name, 10);
    _mav_put_float_array(buf, 20, data, 58);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN);
#else
    mavlink_debug_float_array_t packet;
    packet.time_usec = time_usec;
    packet.array_id = array_id;
    mav_array_memcpy(packet.name, name, sizeof(char)*10);
    mav_array_memcpy(packet.data, data, sizeof(float)*58);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_MIN_LEN, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_CRC);
}

/**
 * @brief Pack a debug_float_array message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param name  Name, for human-friendly display in a Ground Control Station
 * @param array_id  Unique ID used to discriminate between arrays
 * @param data  data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_debug_float_array_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,const char *name,uint16_t array_id,const float *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint16_t(buf, 8, array_id);
    _mav_put_char_array(buf, 10, name, 10);
    _mav_put_float_array(buf, 20, data, 58);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN);
#else
    mavlink_debug_float_array_t packet;
    packet.time_usec = time_usec;
    packet.array_id = array_id;
    mav_array_memcpy(packet.name, name, sizeof(char)*10);
    mav_array_memcpy(packet.data, data, sizeof(float)*58);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_MIN_LEN, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_CRC);
}

/**
 * @brief Encode a debug_float_array struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param debug_float_array C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_debug_float_array_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_debug_float_array_t* debug_float_array)
{
    return mavlink_msg_debug_float_array_pack(system_id, component_id, msg, debug_float_array->time_usec, debug_float_array->name, debug_float_array->array_id, debug_float_array->data);
}

/**
 * @brief Encode a debug_float_array struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param debug_float_array C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_debug_float_array_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_debug_float_array_t* debug_float_array)
{
    return mavlink_msg_debug_float_array_pack_chan(system_id, component_id, chan, msg, debug_float_array->time_usec, debug_float_array->name, debug_float_array->array_id, debug_float_array->data);
}

/**
 * @brief Send a debug_float_array message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param name  Name, for human-friendly display in a Ground Control Station
 * @param array_id  Unique ID used to discriminate between arrays
 * @param data  data
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_debug_float_array_send(mavlink_channel_t chan, uint64_t time_usec, const char *name, uint16_t array_id, const float *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint16_t(buf, 8, array_id);
    _mav_put_char_array(buf, 10, name, 10);
    _mav_put_float_array(buf, 20, data, 58);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY, buf, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_MIN_LEN, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_CRC);
#else
    mavlink_debug_float_array_t packet;
    packet.time_usec = time_usec;
    packet.array_id = array_id;
    mav_array_memcpy(packet.name, name, sizeof(char)*10);
    mav_array_memcpy(packet.data, data, sizeof(float)*58);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY, (const char *)&packet, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_MIN_LEN, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_CRC);
#endif
}

/**
 * @brief Send a debug_float_array message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_debug_float_array_send_struct(mavlink_channel_t chan, const mavlink_debug_float_array_t* debug_float_array)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_debug_float_array_send(chan, debug_float_array->time_usec, debug_float_array->name, debug_float_array->array_id, debug_float_array->data);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY, (const char *)debug_float_array, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_MIN_LEN, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_CRC);
#endif
}

#if MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_debug_float_array_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, const char *name, uint16_t array_id, const float *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint16_t(buf, 8, array_id);
    _mav_put_char_array(buf, 10, name, 10);
    _mav_put_float_array(buf, 20, data, 58);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY, buf, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_MIN_LEN, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_CRC);
#else
    mavlink_debug_float_array_t *packet = (mavlink_debug_float_array_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->array_id = array_id;
    mav_array_memcpy(packet->name, name, sizeof(char)*10);
    mav_array_memcpy(packet->data, data, sizeof(float)*58);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY, (const char *)packet, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_MIN_LEN, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_CRC);
#endif
}
#endif

#endif

// MESSAGE DEBUG_FLOAT_ARRAY UNPACKING


/**
 * @brief Get field time_usec from debug_float_array message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
static inline uint64_t mavlink_msg_debug_float_array_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field name from debug_float_array message
 *
 * @return  Name, for human-friendly display in a Ground Control Station
 */
static inline uint16_t mavlink_msg_debug_float_array_get_name(const mavlink_message_t* msg, char *name)
{
    return _MAV_RETURN_char_array(msg, name, 10,  10);
}

/**
 * @brief Get field array_id from debug_float_array message
 *
 * @return  Unique ID used to discriminate between arrays
 */
static inline uint16_t mavlink_msg_debug_float_array_get_array_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field data from debug_float_array message
 *
 * @return  data
 */
static inline uint16_t mavlink_msg_debug_float_array_get_data(const mavlink_message_t* msg, float *data)
{
    return _MAV_RETURN_float_array(msg, data, 58,  20);
}

/**
 * @brief Decode a debug_float_array message into a struct
 *
 * @param msg The message to decode
 * @param debug_float_array C-struct to decode the message contents into
 */
static inline void mavlink_msg_debug_float_array_decode(const mavlink_message_t* msg, mavlink_debug_float_array_t* debug_float_array)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    debug_float_array->time_usec = mavlink_msg_debug_float_array_get_time_usec(msg);
    debug_float_array->array_id = mavlink_msg_debug_float_array_get_array_id(msg);
    mavlink_msg_debug_float_array_get_name(msg, debug_float_array->name);
    mavlink_msg_debug_float_array_get_data(msg, debug_float_array->data);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN? msg->len : MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN;
        memset(debug_float_array, 0, MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN);
    memcpy(debug_float_array, _MAV_PAYLOAD(msg), len);
#endif
}
