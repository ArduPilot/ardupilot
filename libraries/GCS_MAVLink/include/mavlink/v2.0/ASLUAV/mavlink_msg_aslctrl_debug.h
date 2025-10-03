#pragma once
// MESSAGE ASLCTRL_DEBUG PACKING

#define MAVLINK_MSG_ID_ASLCTRL_DEBUG 8005


typedef struct __mavlink_aslctrl_debug_t {
 uint32_t i32_1; /*<   Debug data*/
 float f_1; /*<   Debug data */
 float f_2; /*<   Debug data*/
 float f_3; /*<   Debug data*/
 float f_4; /*<   Debug data*/
 float f_5; /*<   Debug data*/
 float f_6; /*<   Debug data*/
 float f_7; /*<   Debug data*/
 float f_8; /*<   Debug data*/
 uint8_t i8_1; /*<   Debug data*/
 uint8_t i8_2; /*<   Debug data*/
} mavlink_aslctrl_debug_t;

#define MAVLINK_MSG_ID_ASLCTRL_DEBUG_LEN 38
#define MAVLINK_MSG_ID_ASLCTRL_DEBUG_MIN_LEN 38
#define MAVLINK_MSG_ID_8005_LEN 38
#define MAVLINK_MSG_ID_8005_MIN_LEN 38

#define MAVLINK_MSG_ID_ASLCTRL_DEBUG_CRC 251
#define MAVLINK_MSG_ID_8005_CRC 251



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ASLCTRL_DEBUG { \
    8005, \
    "ASLCTRL_DEBUG", \
    11, \
    {  { "i32_1", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_aslctrl_debug_t, i32_1) }, \
         { "i8_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_aslctrl_debug_t, i8_1) }, \
         { "i8_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_aslctrl_debug_t, i8_2) }, \
         { "f_1", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_aslctrl_debug_t, f_1) }, \
         { "f_2", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_aslctrl_debug_t, f_2) }, \
         { "f_3", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_aslctrl_debug_t, f_3) }, \
         { "f_4", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_aslctrl_debug_t, f_4) }, \
         { "f_5", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_aslctrl_debug_t, f_5) }, \
         { "f_6", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_aslctrl_debug_t, f_6) }, \
         { "f_7", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_aslctrl_debug_t, f_7) }, \
         { "f_8", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_aslctrl_debug_t, f_8) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ASLCTRL_DEBUG { \
    "ASLCTRL_DEBUG", \
    11, \
    {  { "i32_1", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_aslctrl_debug_t, i32_1) }, \
         { "i8_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_aslctrl_debug_t, i8_1) }, \
         { "i8_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_aslctrl_debug_t, i8_2) }, \
         { "f_1", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_aslctrl_debug_t, f_1) }, \
         { "f_2", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_aslctrl_debug_t, f_2) }, \
         { "f_3", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_aslctrl_debug_t, f_3) }, \
         { "f_4", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_aslctrl_debug_t, f_4) }, \
         { "f_5", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_aslctrl_debug_t, f_5) }, \
         { "f_6", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_aslctrl_debug_t, f_6) }, \
         { "f_7", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_aslctrl_debug_t, f_7) }, \
         { "f_8", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_aslctrl_debug_t, f_8) }, \
         } \
}
#endif

/**
 * @brief Pack a aslctrl_debug message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param i32_1   Debug data
 * @param i8_1   Debug data
 * @param i8_2   Debug data
 * @param f_1   Debug data 
 * @param f_2   Debug data
 * @param f_3   Debug data
 * @param f_4   Debug data
 * @param f_5   Debug data
 * @param f_6   Debug data
 * @param f_7   Debug data
 * @param f_8   Debug data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_aslctrl_debug_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t i32_1, uint8_t i8_1, uint8_t i8_2, float f_1, float f_2, float f_3, float f_4, float f_5, float f_6, float f_7, float f_8)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ASLCTRL_DEBUG_LEN];
    _mav_put_uint32_t(buf, 0, i32_1);
    _mav_put_float(buf, 4, f_1);
    _mav_put_float(buf, 8, f_2);
    _mav_put_float(buf, 12, f_3);
    _mav_put_float(buf, 16, f_4);
    _mav_put_float(buf, 20, f_5);
    _mav_put_float(buf, 24, f_6);
    _mav_put_float(buf, 28, f_7);
    _mav_put_float(buf, 32, f_8);
    _mav_put_uint8_t(buf, 36, i8_1);
    _mav_put_uint8_t(buf, 37, i8_2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ASLCTRL_DEBUG_LEN);
#else
    mavlink_aslctrl_debug_t packet;
    packet.i32_1 = i32_1;
    packet.f_1 = f_1;
    packet.f_2 = f_2;
    packet.f_3 = f_3;
    packet.f_4 = f_4;
    packet.f_5 = f_5;
    packet.f_6 = f_6;
    packet.f_7 = f_7;
    packet.f_8 = f_8;
    packet.i8_1 = i8_1;
    packet.i8_2 = i8_2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ASLCTRL_DEBUG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ASLCTRL_DEBUG;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ASLCTRL_DEBUG_MIN_LEN, MAVLINK_MSG_ID_ASLCTRL_DEBUG_LEN, MAVLINK_MSG_ID_ASLCTRL_DEBUG_CRC);
}

/**
 * @brief Pack a aslctrl_debug message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param i32_1   Debug data
 * @param i8_1   Debug data
 * @param i8_2   Debug data
 * @param f_1   Debug data 
 * @param f_2   Debug data
 * @param f_3   Debug data
 * @param f_4   Debug data
 * @param f_5   Debug data
 * @param f_6   Debug data
 * @param f_7   Debug data
 * @param f_8   Debug data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_aslctrl_debug_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t i32_1, uint8_t i8_1, uint8_t i8_2, float f_1, float f_2, float f_3, float f_4, float f_5, float f_6, float f_7, float f_8)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ASLCTRL_DEBUG_LEN];
    _mav_put_uint32_t(buf, 0, i32_1);
    _mav_put_float(buf, 4, f_1);
    _mav_put_float(buf, 8, f_2);
    _mav_put_float(buf, 12, f_3);
    _mav_put_float(buf, 16, f_4);
    _mav_put_float(buf, 20, f_5);
    _mav_put_float(buf, 24, f_6);
    _mav_put_float(buf, 28, f_7);
    _mav_put_float(buf, 32, f_8);
    _mav_put_uint8_t(buf, 36, i8_1);
    _mav_put_uint8_t(buf, 37, i8_2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ASLCTRL_DEBUG_LEN);
#else
    mavlink_aslctrl_debug_t packet;
    packet.i32_1 = i32_1;
    packet.f_1 = f_1;
    packet.f_2 = f_2;
    packet.f_3 = f_3;
    packet.f_4 = f_4;
    packet.f_5 = f_5;
    packet.f_6 = f_6;
    packet.f_7 = f_7;
    packet.f_8 = f_8;
    packet.i8_1 = i8_1;
    packet.i8_2 = i8_2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ASLCTRL_DEBUG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ASLCTRL_DEBUG;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ASLCTRL_DEBUG_MIN_LEN, MAVLINK_MSG_ID_ASLCTRL_DEBUG_LEN, MAVLINK_MSG_ID_ASLCTRL_DEBUG_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ASLCTRL_DEBUG_MIN_LEN, MAVLINK_MSG_ID_ASLCTRL_DEBUG_LEN);
#endif
}

/**
 * @brief Pack a aslctrl_debug message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param i32_1   Debug data
 * @param i8_1   Debug data
 * @param i8_2   Debug data
 * @param f_1   Debug data 
 * @param f_2   Debug data
 * @param f_3   Debug data
 * @param f_4   Debug data
 * @param f_5   Debug data
 * @param f_6   Debug data
 * @param f_7   Debug data
 * @param f_8   Debug data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_aslctrl_debug_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t i32_1,uint8_t i8_1,uint8_t i8_2,float f_1,float f_2,float f_3,float f_4,float f_5,float f_6,float f_7,float f_8)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ASLCTRL_DEBUG_LEN];
    _mav_put_uint32_t(buf, 0, i32_1);
    _mav_put_float(buf, 4, f_1);
    _mav_put_float(buf, 8, f_2);
    _mav_put_float(buf, 12, f_3);
    _mav_put_float(buf, 16, f_4);
    _mav_put_float(buf, 20, f_5);
    _mav_put_float(buf, 24, f_6);
    _mav_put_float(buf, 28, f_7);
    _mav_put_float(buf, 32, f_8);
    _mav_put_uint8_t(buf, 36, i8_1);
    _mav_put_uint8_t(buf, 37, i8_2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ASLCTRL_DEBUG_LEN);
#else
    mavlink_aslctrl_debug_t packet;
    packet.i32_1 = i32_1;
    packet.f_1 = f_1;
    packet.f_2 = f_2;
    packet.f_3 = f_3;
    packet.f_4 = f_4;
    packet.f_5 = f_5;
    packet.f_6 = f_6;
    packet.f_7 = f_7;
    packet.f_8 = f_8;
    packet.i8_1 = i8_1;
    packet.i8_2 = i8_2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ASLCTRL_DEBUG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ASLCTRL_DEBUG;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ASLCTRL_DEBUG_MIN_LEN, MAVLINK_MSG_ID_ASLCTRL_DEBUG_LEN, MAVLINK_MSG_ID_ASLCTRL_DEBUG_CRC);
}

/**
 * @brief Encode a aslctrl_debug struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param aslctrl_debug C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_aslctrl_debug_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_aslctrl_debug_t* aslctrl_debug)
{
    return mavlink_msg_aslctrl_debug_pack(system_id, component_id, msg, aslctrl_debug->i32_1, aslctrl_debug->i8_1, aslctrl_debug->i8_2, aslctrl_debug->f_1, aslctrl_debug->f_2, aslctrl_debug->f_3, aslctrl_debug->f_4, aslctrl_debug->f_5, aslctrl_debug->f_6, aslctrl_debug->f_7, aslctrl_debug->f_8);
}

/**
 * @brief Encode a aslctrl_debug struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param aslctrl_debug C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_aslctrl_debug_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_aslctrl_debug_t* aslctrl_debug)
{
    return mavlink_msg_aslctrl_debug_pack_chan(system_id, component_id, chan, msg, aslctrl_debug->i32_1, aslctrl_debug->i8_1, aslctrl_debug->i8_2, aslctrl_debug->f_1, aslctrl_debug->f_2, aslctrl_debug->f_3, aslctrl_debug->f_4, aslctrl_debug->f_5, aslctrl_debug->f_6, aslctrl_debug->f_7, aslctrl_debug->f_8);
}

/**
 * @brief Encode a aslctrl_debug struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param aslctrl_debug C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_aslctrl_debug_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_aslctrl_debug_t* aslctrl_debug)
{
    return mavlink_msg_aslctrl_debug_pack_status(system_id, component_id, _status, msg,  aslctrl_debug->i32_1, aslctrl_debug->i8_1, aslctrl_debug->i8_2, aslctrl_debug->f_1, aslctrl_debug->f_2, aslctrl_debug->f_3, aslctrl_debug->f_4, aslctrl_debug->f_5, aslctrl_debug->f_6, aslctrl_debug->f_7, aslctrl_debug->f_8);
}

/**
 * @brief Send a aslctrl_debug message
 * @param chan MAVLink channel to send the message
 *
 * @param i32_1   Debug data
 * @param i8_1   Debug data
 * @param i8_2   Debug data
 * @param f_1   Debug data 
 * @param f_2   Debug data
 * @param f_3   Debug data
 * @param f_4   Debug data
 * @param f_5   Debug data
 * @param f_6   Debug data
 * @param f_7   Debug data
 * @param f_8   Debug data
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_aslctrl_debug_send(mavlink_channel_t chan, uint32_t i32_1, uint8_t i8_1, uint8_t i8_2, float f_1, float f_2, float f_3, float f_4, float f_5, float f_6, float f_7, float f_8)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ASLCTRL_DEBUG_LEN];
    _mav_put_uint32_t(buf, 0, i32_1);
    _mav_put_float(buf, 4, f_1);
    _mav_put_float(buf, 8, f_2);
    _mav_put_float(buf, 12, f_3);
    _mav_put_float(buf, 16, f_4);
    _mav_put_float(buf, 20, f_5);
    _mav_put_float(buf, 24, f_6);
    _mav_put_float(buf, 28, f_7);
    _mav_put_float(buf, 32, f_8);
    _mav_put_uint8_t(buf, 36, i8_1);
    _mav_put_uint8_t(buf, 37, i8_2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ASLCTRL_DEBUG, buf, MAVLINK_MSG_ID_ASLCTRL_DEBUG_MIN_LEN, MAVLINK_MSG_ID_ASLCTRL_DEBUG_LEN, MAVLINK_MSG_ID_ASLCTRL_DEBUG_CRC);
#else
    mavlink_aslctrl_debug_t packet;
    packet.i32_1 = i32_1;
    packet.f_1 = f_1;
    packet.f_2 = f_2;
    packet.f_3 = f_3;
    packet.f_4 = f_4;
    packet.f_5 = f_5;
    packet.f_6 = f_6;
    packet.f_7 = f_7;
    packet.f_8 = f_8;
    packet.i8_1 = i8_1;
    packet.i8_2 = i8_2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ASLCTRL_DEBUG, (const char *)&packet, MAVLINK_MSG_ID_ASLCTRL_DEBUG_MIN_LEN, MAVLINK_MSG_ID_ASLCTRL_DEBUG_LEN, MAVLINK_MSG_ID_ASLCTRL_DEBUG_CRC);
#endif
}

/**
 * @brief Send a aslctrl_debug message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_aslctrl_debug_send_struct(mavlink_channel_t chan, const mavlink_aslctrl_debug_t* aslctrl_debug)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_aslctrl_debug_send(chan, aslctrl_debug->i32_1, aslctrl_debug->i8_1, aslctrl_debug->i8_2, aslctrl_debug->f_1, aslctrl_debug->f_2, aslctrl_debug->f_3, aslctrl_debug->f_4, aslctrl_debug->f_5, aslctrl_debug->f_6, aslctrl_debug->f_7, aslctrl_debug->f_8);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ASLCTRL_DEBUG, (const char *)aslctrl_debug, MAVLINK_MSG_ID_ASLCTRL_DEBUG_MIN_LEN, MAVLINK_MSG_ID_ASLCTRL_DEBUG_LEN, MAVLINK_MSG_ID_ASLCTRL_DEBUG_CRC);
#endif
}

#if MAVLINK_MSG_ID_ASLCTRL_DEBUG_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_aslctrl_debug_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t i32_1, uint8_t i8_1, uint8_t i8_2, float f_1, float f_2, float f_3, float f_4, float f_5, float f_6, float f_7, float f_8)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, i32_1);
    _mav_put_float(buf, 4, f_1);
    _mav_put_float(buf, 8, f_2);
    _mav_put_float(buf, 12, f_3);
    _mav_put_float(buf, 16, f_4);
    _mav_put_float(buf, 20, f_5);
    _mav_put_float(buf, 24, f_6);
    _mav_put_float(buf, 28, f_7);
    _mav_put_float(buf, 32, f_8);
    _mav_put_uint8_t(buf, 36, i8_1);
    _mav_put_uint8_t(buf, 37, i8_2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ASLCTRL_DEBUG, buf, MAVLINK_MSG_ID_ASLCTRL_DEBUG_MIN_LEN, MAVLINK_MSG_ID_ASLCTRL_DEBUG_LEN, MAVLINK_MSG_ID_ASLCTRL_DEBUG_CRC);
#else
    mavlink_aslctrl_debug_t *packet = (mavlink_aslctrl_debug_t *)msgbuf;
    packet->i32_1 = i32_1;
    packet->f_1 = f_1;
    packet->f_2 = f_2;
    packet->f_3 = f_3;
    packet->f_4 = f_4;
    packet->f_5 = f_5;
    packet->f_6 = f_6;
    packet->f_7 = f_7;
    packet->f_8 = f_8;
    packet->i8_1 = i8_1;
    packet->i8_2 = i8_2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ASLCTRL_DEBUG, (const char *)packet, MAVLINK_MSG_ID_ASLCTRL_DEBUG_MIN_LEN, MAVLINK_MSG_ID_ASLCTRL_DEBUG_LEN, MAVLINK_MSG_ID_ASLCTRL_DEBUG_CRC);
#endif
}
#endif

#endif

// MESSAGE ASLCTRL_DEBUG UNPACKING


/**
 * @brief Get field i32_1 from aslctrl_debug message
 *
 * @return   Debug data
 */
static inline uint32_t mavlink_msg_aslctrl_debug_get_i32_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field i8_1 from aslctrl_debug message
 *
 * @return   Debug data
 */
static inline uint8_t mavlink_msg_aslctrl_debug_get_i8_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Get field i8_2 from aslctrl_debug message
 *
 * @return   Debug data
 */
static inline uint8_t mavlink_msg_aslctrl_debug_get_i8_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  37);
}

/**
 * @brief Get field f_1 from aslctrl_debug message
 *
 * @return   Debug data 
 */
static inline float mavlink_msg_aslctrl_debug_get_f_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field f_2 from aslctrl_debug message
 *
 * @return   Debug data
 */
static inline float mavlink_msg_aslctrl_debug_get_f_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field f_3 from aslctrl_debug message
 *
 * @return   Debug data
 */
static inline float mavlink_msg_aslctrl_debug_get_f_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field f_4 from aslctrl_debug message
 *
 * @return   Debug data
 */
static inline float mavlink_msg_aslctrl_debug_get_f_4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field f_5 from aslctrl_debug message
 *
 * @return   Debug data
 */
static inline float mavlink_msg_aslctrl_debug_get_f_5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field f_6 from aslctrl_debug message
 *
 * @return   Debug data
 */
static inline float mavlink_msg_aslctrl_debug_get_f_6(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field f_7 from aslctrl_debug message
 *
 * @return   Debug data
 */
static inline float mavlink_msg_aslctrl_debug_get_f_7(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field f_8 from aslctrl_debug message
 *
 * @return   Debug data
 */
static inline float mavlink_msg_aslctrl_debug_get_f_8(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Decode a aslctrl_debug message into a struct
 *
 * @param msg The message to decode
 * @param aslctrl_debug C-struct to decode the message contents into
 */
static inline void mavlink_msg_aslctrl_debug_decode(const mavlink_message_t* msg, mavlink_aslctrl_debug_t* aslctrl_debug)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    aslctrl_debug->i32_1 = mavlink_msg_aslctrl_debug_get_i32_1(msg);
    aslctrl_debug->f_1 = mavlink_msg_aslctrl_debug_get_f_1(msg);
    aslctrl_debug->f_2 = mavlink_msg_aslctrl_debug_get_f_2(msg);
    aslctrl_debug->f_3 = mavlink_msg_aslctrl_debug_get_f_3(msg);
    aslctrl_debug->f_4 = mavlink_msg_aslctrl_debug_get_f_4(msg);
    aslctrl_debug->f_5 = mavlink_msg_aslctrl_debug_get_f_5(msg);
    aslctrl_debug->f_6 = mavlink_msg_aslctrl_debug_get_f_6(msg);
    aslctrl_debug->f_7 = mavlink_msg_aslctrl_debug_get_f_7(msg);
    aslctrl_debug->f_8 = mavlink_msg_aslctrl_debug_get_f_8(msg);
    aslctrl_debug->i8_1 = mavlink_msg_aslctrl_debug_get_i8_1(msg);
    aslctrl_debug->i8_2 = mavlink_msg_aslctrl_debug_get_i8_2(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ASLCTRL_DEBUG_LEN? msg->len : MAVLINK_MSG_ID_ASLCTRL_DEBUG_LEN;
        memset(aslctrl_debug, 0, MAVLINK_MSG_ID_ASLCTRL_DEBUG_LEN);
    memcpy(aslctrl_debug, _MAV_PAYLOAD(msg), len);
#endif
}
