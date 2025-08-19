#pragma once
// MESSAGE ARRAY_TEST_7 PACKING

#define MAVLINK_MSG_ID_ARRAY_TEST_7 17157


typedef struct __mavlink_array_test_7_t {
 double ar_d[2]; /*<  Value array*/
 float ar_f[2]; /*<  Value array*/
 uint32_t ar_u32[2]; /*<  Value array*/
 int32_t ar_i32[2]; /*<  Value array*/
 uint16_t ar_u16[2]; /*<  Value array*/
 int16_t ar_i16[2]; /*<  Value array*/
 uint8_t ar_u8[2]; /*<  Value array*/
 int8_t ar_i8[2]; /*<  Value array*/
 char ar_c[32]; /*<  Value array*/
} mavlink_array_test_7_t;

#define MAVLINK_MSG_ID_ARRAY_TEST_7_LEN 84
#define MAVLINK_MSG_ID_ARRAY_TEST_7_MIN_LEN 84
#define MAVLINK_MSG_ID_17157_LEN 84
#define MAVLINK_MSG_ID_17157_MIN_LEN 84

#define MAVLINK_MSG_ID_ARRAY_TEST_7_CRC 187
#define MAVLINK_MSG_ID_17157_CRC 187

#define MAVLINK_MSG_ARRAY_TEST_7_FIELD_AR_D_LEN 2
#define MAVLINK_MSG_ARRAY_TEST_7_FIELD_AR_F_LEN 2
#define MAVLINK_MSG_ARRAY_TEST_7_FIELD_AR_U32_LEN 2
#define MAVLINK_MSG_ARRAY_TEST_7_FIELD_AR_I32_LEN 2
#define MAVLINK_MSG_ARRAY_TEST_7_FIELD_AR_U16_LEN 2
#define MAVLINK_MSG_ARRAY_TEST_7_FIELD_AR_I16_LEN 2
#define MAVLINK_MSG_ARRAY_TEST_7_FIELD_AR_U8_LEN 2
#define MAVLINK_MSG_ARRAY_TEST_7_FIELD_AR_I8_LEN 2
#define MAVLINK_MSG_ARRAY_TEST_7_FIELD_AR_C_LEN 32

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ARRAY_TEST_7 { \
    17157, \
    "ARRAY_TEST_7", \
    9, \
    {  { "ar_d", NULL, MAVLINK_TYPE_DOUBLE, 2, 0, offsetof(mavlink_array_test_7_t, ar_d) }, \
         { "ar_f", NULL, MAVLINK_TYPE_FLOAT, 2, 16, offsetof(mavlink_array_test_7_t, ar_f) }, \
         { "ar_u32", NULL, MAVLINK_TYPE_UINT32_T, 2, 24, offsetof(mavlink_array_test_7_t, ar_u32) }, \
         { "ar_i32", NULL, MAVLINK_TYPE_INT32_T, 2, 32, offsetof(mavlink_array_test_7_t, ar_i32) }, \
         { "ar_u16", NULL, MAVLINK_TYPE_UINT16_T, 2, 40, offsetof(mavlink_array_test_7_t, ar_u16) }, \
         { "ar_i16", NULL, MAVLINK_TYPE_INT16_T, 2, 44, offsetof(mavlink_array_test_7_t, ar_i16) }, \
         { "ar_u8", NULL, MAVLINK_TYPE_UINT8_T, 2, 48, offsetof(mavlink_array_test_7_t, ar_u8) }, \
         { "ar_i8", NULL, MAVLINK_TYPE_INT8_T, 2, 50, offsetof(mavlink_array_test_7_t, ar_i8) }, \
         { "ar_c", NULL, MAVLINK_TYPE_CHAR, 32, 52, offsetof(mavlink_array_test_7_t, ar_c) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ARRAY_TEST_7 { \
    "ARRAY_TEST_7", \
    9, \
    {  { "ar_d", NULL, MAVLINK_TYPE_DOUBLE, 2, 0, offsetof(mavlink_array_test_7_t, ar_d) }, \
         { "ar_f", NULL, MAVLINK_TYPE_FLOAT, 2, 16, offsetof(mavlink_array_test_7_t, ar_f) }, \
         { "ar_u32", NULL, MAVLINK_TYPE_UINT32_T, 2, 24, offsetof(mavlink_array_test_7_t, ar_u32) }, \
         { "ar_i32", NULL, MAVLINK_TYPE_INT32_T, 2, 32, offsetof(mavlink_array_test_7_t, ar_i32) }, \
         { "ar_u16", NULL, MAVLINK_TYPE_UINT16_T, 2, 40, offsetof(mavlink_array_test_7_t, ar_u16) }, \
         { "ar_i16", NULL, MAVLINK_TYPE_INT16_T, 2, 44, offsetof(mavlink_array_test_7_t, ar_i16) }, \
         { "ar_u8", NULL, MAVLINK_TYPE_UINT8_T, 2, 48, offsetof(mavlink_array_test_7_t, ar_u8) }, \
         { "ar_i8", NULL, MAVLINK_TYPE_INT8_T, 2, 50, offsetof(mavlink_array_test_7_t, ar_i8) }, \
         { "ar_c", NULL, MAVLINK_TYPE_CHAR, 32, 52, offsetof(mavlink_array_test_7_t, ar_c) }, \
         } \
}
#endif

/**
 * @brief Pack a array_test_7 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param ar_d  Value array
 * @param ar_f  Value array
 * @param ar_u32  Value array
 * @param ar_i32  Value array
 * @param ar_u16  Value array
 * @param ar_i16  Value array
 * @param ar_u8  Value array
 * @param ar_i8  Value array
 * @param ar_c  Value array
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_array_test_7_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const double *ar_d, const float *ar_f, const uint32_t *ar_u32, const int32_t *ar_i32, const uint16_t *ar_u16, const int16_t *ar_i16, const uint8_t *ar_u8, const int8_t *ar_i8, const char *ar_c)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ARRAY_TEST_7_LEN];

    _mav_put_double_array(buf, 0, ar_d, 2);
    _mav_put_float_array(buf, 16, ar_f, 2);
    _mav_put_uint32_t_array(buf, 24, ar_u32, 2);
    _mav_put_int32_t_array(buf, 32, ar_i32, 2);
    _mav_put_uint16_t_array(buf, 40, ar_u16, 2);
    _mav_put_int16_t_array(buf, 44, ar_i16, 2);
    _mav_put_uint8_t_array(buf, 48, ar_u8, 2);
    _mav_put_int8_t_array(buf, 50, ar_i8, 2);
    _mav_put_char_array(buf, 52, ar_c, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARRAY_TEST_7_LEN);
#else
    mavlink_array_test_7_t packet;

    mav_array_memcpy(packet.ar_d, ar_d, sizeof(double)*2);
    mav_array_memcpy(packet.ar_f, ar_f, sizeof(float)*2);
    mav_array_memcpy(packet.ar_u32, ar_u32, sizeof(uint32_t)*2);
    mav_array_memcpy(packet.ar_i32, ar_i32, sizeof(int32_t)*2);
    mav_array_memcpy(packet.ar_u16, ar_u16, sizeof(uint16_t)*2);
    mav_array_memcpy(packet.ar_i16, ar_i16, sizeof(int16_t)*2);
    mav_array_memcpy(packet.ar_u8, ar_u8, sizeof(uint8_t)*2);
    mav_array_memcpy(packet.ar_i8, ar_i8, sizeof(int8_t)*2);
    mav_array_memcpy(packet.ar_c, ar_c, sizeof(char)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARRAY_TEST_7_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ARRAY_TEST_7;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ARRAY_TEST_7_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_7_LEN, MAVLINK_MSG_ID_ARRAY_TEST_7_CRC);
}

/**
 * @brief Pack a array_test_7 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param ar_d  Value array
 * @param ar_f  Value array
 * @param ar_u32  Value array
 * @param ar_i32  Value array
 * @param ar_u16  Value array
 * @param ar_i16  Value array
 * @param ar_u8  Value array
 * @param ar_i8  Value array
 * @param ar_c  Value array
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_array_test_7_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               const double *ar_d, const float *ar_f, const uint32_t *ar_u32, const int32_t *ar_i32, const uint16_t *ar_u16, const int16_t *ar_i16, const uint8_t *ar_u8, const int8_t *ar_i8, const char *ar_c)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ARRAY_TEST_7_LEN];

    _mav_put_double_array(buf, 0, ar_d, 2);
    _mav_put_float_array(buf, 16, ar_f, 2);
    _mav_put_uint32_t_array(buf, 24, ar_u32, 2);
    _mav_put_int32_t_array(buf, 32, ar_i32, 2);
    _mav_put_uint16_t_array(buf, 40, ar_u16, 2);
    _mav_put_int16_t_array(buf, 44, ar_i16, 2);
    _mav_put_uint8_t_array(buf, 48, ar_u8, 2);
    _mav_put_int8_t_array(buf, 50, ar_i8, 2);
    _mav_put_char_array(buf, 52, ar_c, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARRAY_TEST_7_LEN);
#else
    mavlink_array_test_7_t packet;

    mav_array_memcpy(packet.ar_d, ar_d, sizeof(double)*2);
    mav_array_memcpy(packet.ar_f, ar_f, sizeof(float)*2);
    mav_array_memcpy(packet.ar_u32, ar_u32, sizeof(uint32_t)*2);
    mav_array_memcpy(packet.ar_i32, ar_i32, sizeof(int32_t)*2);
    mav_array_memcpy(packet.ar_u16, ar_u16, sizeof(uint16_t)*2);
    mav_array_memcpy(packet.ar_i16, ar_i16, sizeof(int16_t)*2);
    mav_array_memcpy(packet.ar_u8, ar_u8, sizeof(uint8_t)*2);
    mav_array_memcpy(packet.ar_i8, ar_i8, sizeof(int8_t)*2);
    mav_array_memcpy(packet.ar_c, ar_c, sizeof(char)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARRAY_TEST_7_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ARRAY_TEST_7;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ARRAY_TEST_7_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_7_LEN, MAVLINK_MSG_ID_ARRAY_TEST_7_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ARRAY_TEST_7_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_7_LEN);
#endif
}

/**
 * @brief Pack a array_test_7 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ar_d  Value array
 * @param ar_f  Value array
 * @param ar_u32  Value array
 * @param ar_i32  Value array
 * @param ar_u16  Value array
 * @param ar_i16  Value array
 * @param ar_u8  Value array
 * @param ar_i8  Value array
 * @param ar_c  Value array
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_array_test_7_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const double *ar_d,const float *ar_f,const uint32_t *ar_u32,const int32_t *ar_i32,const uint16_t *ar_u16,const int16_t *ar_i16,const uint8_t *ar_u8,const int8_t *ar_i8,const char *ar_c)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ARRAY_TEST_7_LEN];

    _mav_put_double_array(buf, 0, ar_d, 2);
    _mav_put_float_array(buf, 16, ar_f, 2);
    _mav_put_uint32_t_array(buf, 24, ar_u32, 2);
    _mav_put_int32_t_array(buf, 32, ar_i32, 2);
    _mav_put_uint16_t_array(buf, 40, ar_u16, 2);
    _mav_put_int16_t_array(buf, 44, ar_i16, 2);
    _mav_put_uint8_t_array(buf, 48, ar_u8, 2);
    _mav_put_int8_t_array(buf, 50, ar_i8, 2);
    _mav_put_char_array(buf, 52, ar_c, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARRAY_TEST_7_LEN);
#else
    mavlink_array_test_7_t packet;

    mav_array_memcpy(packet.ar_d, ar_d, sizeof(double)*2);
    mav_array_memcpy(packet.ar_f, ar_f, sizeof(float)*2);
    mav_array_memcpy(packet.ar_u32, ar_u32, sizeof(uint32_t)*2);
    mav_array_memcpy(packet.ar_i32, ar_i32, sizeof(int32_t)*2);
    mav_array_memcpy(packet.ar_u16, ar_u16, sizeof(uint16_t)*2);
    mav_array_memcpy(packet.ar_i16, ar_i16, sizeof(int16_t)*2);
    mav_array_memcpy(packet.ar_u8, ar_u8, sizeof(uint8_t)*2);
    mav_array_memcpy(packet.ar_i8, ar_i8, sizeof(int8_t)*2);
    mav_array_memcpy(packet.ar_c, ar_c, sizeof(char)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARRAY_TEST_7_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ARRAY_TEST_7;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ARRAY_TEST_7_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_7_LEN, MAVLINK_MSG_ID_ARRAY_TEST_7_CRC);
}

/**
 * @brief Encode a array_test_7 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param array_test_7 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_array_test_7_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_array_test_7_t* array_test_7)
{
    return mavlink_msg_array_test_7_pack(system_id, component_id, msg, array_test_7->ar_d, array_test_7->ar_f, array_test_7->ar_u32, array_test_7->ar_i32, array_test_7->ar_u16, array_test_7->ar_i16, array_test_7->ar_u8, array_test_7->ar_i8, array_test_7->ar_c);
}

/**
 * @brief Encode a array_test_7 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param array_test_7 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_array_test_7_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_array_test_7_t* array_test_7)
{
    return mavlink_msg_array_test_7_pack_chan(system_id, component_id, chan, msg, array_test_7->ar_d, array_test_7->ar_f, array_test_7->ar_u32, array_test_7->ar_i32, array_test_7->ar_u16, array_test_7->ar_i16, array_test_7->ar_u8, array_test_7->ar_i8, array_test_7->ar_c);
}

/**
 * @brief Encode a array_test_7 struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param array_test_7 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_array_test_7_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_array_test_7_t* array_test_7)
{
    return mavlink_msg_array_test_7_pack_status(system_id, component_id, _status, msg,  array_test_7->ar_d, array_test_7->ar_f, array_test_7->ar_u32, array_test_7->ar_i32, array_test_7->ar_u16, array_test_7->ar_i16, array_test_7->ar_u8, array_test_7->ar_i8, array_test_7->ar_c);
}

/**
 * @brief Send a array_test_7 message
 * @param chan MAVLink channel to send the message
 *
 * @param ar_d  Value array
 * @param ar_f  Value array
 * @param ar_u32  Value array
 * @param ar_i32  Value array
 * @param ar_u16  Value array
 * @param ar_i16  Value array
 * @param ar_u8  Value array
 * @param ar_i8  Value array
 * @param ar_c  Value array
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_array_test_7_send(mavlink_channel_t chan, const double *ar_d, const float *ar_f, const uint32_t *ar_u32, const int32_t *ar_i32, const uint16_t *ar_u16, const int16_t *ar_i16, const uint8_t *ar_u8, const int8_t *ar_i8, const char *ar_c)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ARRAY_TEST_7_LEN];

    _mav_put_double_array(buf, 0, ar_d, 2);
    _mav_put_float_array(buf, 16, ar_f, 2);
    _mav_put_uint32_t_array(buf, 24, ar_u32, 2);
    _mav_put_int32_t_array(buf, 32, ar_i32, 2);
    _mav_put_uint16_t_array(buf, 40, ar_u16, 2);
    _mav_put_int16_t_array(buf, 44, ar_i16, 2);
    _mav_put_uint8_t_array(buf, 48, ar_u8, 2);
    _mav_put_int8_t_array(buf, 50, ar_i8, 2);
    _mav_put_char_array(buf, 52, ar_c, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARRAY_TEST_7, buf, MAVLINK_MSG_ID_ARRAY_TEST_7_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_7_LEN, MAVLINK_MSG_ID_ARRAY_TEST_7_CRC);
#else
    mavlink_array_test_7_t packet;

    mav_array_memcpy(packet.ar_d, ar_d, sizeof(double)*2);
    mav_array_memcpy(packet.ar_f, ar_f, sizeof(float)*2);
    mav_array_memcpy(packet.ar_u32, ar_u32, sizeof(uint32_t)*2);
    mav_array_memcpy(packet.ar_i32, ar_i32, sizeof(int32_t)*2);
    mav_array_memcpy(packet.ar_u16, ar_u16, sizeof(uint16_t)*2);
    mav_array_memcpy(packet.ar_i16, ar_i16, sizeof(int16_t)*2);
    mav_array_memcpy(packet.ar_u8, ar_u8, sizeof(uint8_t)*2);
    mav_array_memcpy(packet.ar_i8, ar_i8, sizeof(int8_t)*2);
    mav_array_memcpy(packet.ar_c, ar_c, sizeof(char)*32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARRAY_TEST_7, (const char *)&packet, MAVLINK_MSG_ID_ARRAY_TEST_7_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_7_LEN, MAVLINK_MSG_ID_ARRAY_TEST_7_CRC);
#endif
}

/**
 * @brief Send a array_test_7 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_array_test_7_send_struct(mavlink_channel_t chan, const mavlink_array_test_7_t* array_test_7)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_array_test_7_send(chan, array_test_7->ar_d, array_test_7->ar_f, array_test_7->ar_u32, array_test_7->ar_i32, array_test_7->ar_u16, array_test_7->ar_i16, array_test_7->ar_u8, array_test_7->ar_i8, array_test_7->ar_c);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARRAY_TEST_7, (const char *)array_test_7, MAVLINK_MSG_ID_ARRAY_TEST_7_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_7_LEN, MAVLINK_MSG_ID_ARRAY_TEST_7_CRC);
#endif
}

#if MAVLINK_MSG_ID_ARRAY_TEST_7_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_array_test_7_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const double *ar_d, const float *ar_f, const uint32_t *ar_u32, const int32_t *ar_i32, const uint16_t *ar_u16, const int16_t *ar_i16, const uint8_t *ar_u8, const int8_t *ar_i8, const char *ar_c)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_double_array(buf, 0, ar_d, 2);
    _mav_put_float_array(buf, 16, ar_f, 2);
    _mav_put_uint32_t_array(buf, 24, ar_u32, 2);
    _mav_put_int32_t_array(buf, 32, ar_i32, 2);
    _mav_put_uint16_t_array(buf, 40, ar_u16, 2);
    _mav_put_int16_t_array(buf, 44, ar_i16, 2);
    _mav_put_uint8_t_array(buf, 48, ar_u8, 2);
    _mav_put_int8_t_array(buf, 50, ar_i8, 2);
    _mav_put_char_array(buf, 52, ar_c, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARRAY_TEST_7, buf, MAVLINK_MSG_ID_ARRAY_TEST_7_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_7_LEN, MAVLINK_MSG_ID_ARRAY_TEST_7_CRC);
#else
    mavlink_array_test_7_t *packet = (mavlink_array_test_7_t *)msgbuf;

    mav_array_memcpy(packet->ar_d, ar_d, sizeof(double)*2);
    mav_array_memcpy(packet->ar_f, ar_f, sizeof(float)*2);
    mav_array_memcpy(packet->ar_u32, ar_u32, sizeof(uint32_t)*2);
    mav_array_memcpy(packet->ar_i32, ar_i32, sizeof(int32_t)*2);
    mav_array_memcpy(packet->ar_u16, ar_u16, sizeof(uint16_t)*2);
    mav_array_memcpy(packet->ar_i16, ar_i16, sizeof(int16_t)*2);
    mav_array_memcpy(packet->ar_u8, ar_u8, sizeof(uint8_t)*2);
    mav_array_memcpy(packet->ar_i8, ar_i8, sizeof(int8_t)*2);
    mav_array_memcpy(packet->ar_c, ar_c, sizeof(char)*32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARRAY_TEST_7, (const char *)packet, MAVLINK_MSG_ID_ARRAY_TEST_7_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_7_LEN, MAVLINK_MSG_ID_ARRAY_TEST_7_CRC);
#endif
}
#endif

#endif

// MESSAGE ARRAY_TEST_7 UNPACKING


/**
 * @brief Get field ar_d from array_test_7 message
 *
 * @return  Value array
 */
static inline uint16_t mavlink_msg_array_test_7_get_ar_d(const mavlink_message_t* msg, double *ar_d)
{
    return _MAV_RETURN_double_array(msg, ar_d, 2,  0);
}

/**
 * @brief Get field ar_f from array_test_7 message
 *
 * @return  Value array
 */
static inline uint16_t mavlink_msg_array_test_7_get_ar_f(const mavlink_message_t* msg, float *ar_f)
{
    return _MAV_RETURN_float_array(msg, ar_f, 2,  16);
}

/**
 * @brief Get field ar_u32 from array_test_7 message
 *
 * @return  Value array
 */
static inline uint16_t mavlink_msg_array_test_7_get_ar_u32(const mavlink_message_t* msg, uint32_t *ar_u32)
{
    return _MAV_RETURN_uint32_t_array(msg, ar_u32, 2,  24);
}

/**
 * @brief Get field ar_i32 from array_test_7 message
 *
 * @return  Value array
 */
static inline uint16_t mavlink_msg_array_test_7_get_ar_i32(const mavlink_message_t* msg, int32_t *ar_i32)
{
    return _MAV_RETURN_int32_t_array(msg, ar_i32, 2,  32);
}

/**
 * @brief Get field ar_u16 from array_test_7 message
 *
 * @return  Value array
 */
static inline uint16_t mavlink_msg_array_test_7_get_ar_u16(const mavlink_message_t* msg, uint16_t *ar_u16)
{
    return _MAV_RETURN_uint16_t_array(msg, ar_u16, 2,  40);
}

/**
 * @brief Get field ar_i16 from array_test_7 message
 *
 * @return  Value array
 */
static inline uint16_t mavlink_msg_array_test_7_get_ar_i16(const mavlink_message_t* msg, int16_t *ar_i16)
{
    return _MAV_RETURN_int16_t_array(msg, ar_i16, 2,  44);
}

/**
 * @brief Get field ar_u8 from array_test_7 message
 *
 * @return  Value array
 */
static inline uint16_t mavlink_msg_array_test_7_get_ar_u8(const mavlink_message_t* msg, uint8_t *ar_u8)
{
    return _MAV_RETURN_uint8_t_array(msg, ar_u8, 2,  48);
}

/**
 * @brief Get field ar_i8 from array_test_7 message
 *
 * @return  Value array
 */
static inline uint16_t mavlink_msg_array_test_7_get_ar_i8(const mavlink_message_t* msg, int8_t *ar_i8)
{
    return _MAV_RETURN_int8_t_array(msg, ar_i8, 2,  50);
}

/**
 * @brief Get field ar_c from array_test_7 message
 *
 * @return  Value array
 */
static inline uint16_t mavlink_msg_array_test_7_get_ar_c(const mavlink_message_t* msg, char *ar_c)
{
    return _MAV_RETURN_char_array(msg, ar_c, 32,  52);
}

/**
 * @brief Decode a array_test_7 message into a struct
 *
 * @param msg The message to decode
 * @param array_test_7 C-struct to decode the message contents into
 */
static inline void mavlink_msg_array_test_7_decode(const mavlink_message_t* msg, mavlink_array_test_7_t* array_test_7)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_array_test_7_get_ar_d(msg, array_test_7->ar_d);
    mavlink_msg_array_test_7_get_ar_f(msg, array_test_7->ar_f);
    mavlink_msg_array_test_7_get_ar_u32(msg, array_test_7->ar_u32);
    mavlink_msg_array_test_7_get_ar_i32(msg, array_test_7->ar_i32);
    mavlink_msg_array_test_7_get_ar_u16(msg, array_test_7->ar_u16);
    mavlink_msg_array_test_7_get_ar_i16(msg, array_test_7->ar_i16);
    mavlink_msg_array_test_7_get_ar_u8(msg, array_test_7->ar_u8);
    mavlink_msg_array_test_7_get_ar_i8(msg, array_test_7->ar_i8);
    mavlink_msg_array_test_7_get_ar_c(msg, array_test_7->ar_c);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ARRAY_TEST_7_LEN? msg->len : MAVLINK_MSG_ID_ARRAY_TEST_7_LEN;
        memset(array_test_7, 0, MAVLINK_MSG_ID_ARRAY_TEST_7_LEN);
    memcpy(array_test_7, _MAV_PAYLOAD(msg), len);
#endif
}
