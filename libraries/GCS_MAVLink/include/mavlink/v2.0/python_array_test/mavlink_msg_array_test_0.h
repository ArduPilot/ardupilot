#pragma once
// MESSAGE ARRAY_TEST_0 PACKING

#define MAVLINK_MSG_ID_ARRAY_TEST_0 17150


typedef struct __mavlink_array_test_0_t {
 uint32_t ar_u32[4]; /*<  Value array*/
 uint16_t ar_u16[4]; /*<  Value array*/
 uint8_t v1; /*<  Stub field*/
 int8_t ar_i8[4]; /*<  Value array*/
 uint8_t ar_u8[4]; /*<  Value array*/
} mavlink_array_test_0_t;

#define MAVLINK_MSG_ID_ARRAY_TEST_0_LEN 33
#define MAVLINK_MSG_ID_ARRAY_TEST_0_MIN_LEN 33
#define MAVLINK_MSG_ID_17150_LEN 33
#define MAVLINK_MSG_ID_17150_MIN_LEN 33

#define MAVLINK_MSG_ID_ARRAY_TEST_0_CRC 26
#define MAVLINK_MSG_ID_17150_CRC 26

#define MAVLINK_MSG_ARRAY_TEST_0_FIELD_AR_U32_LEN 4
#define MAVLINK_MSG_ARRAY_TEST_0_FIELD_AR_U16_LEN 4
#define MAVLINK_MSG_ARRAY_TEST_0_FIELD_AR_I8_LEN 4
#define MAVLINK_MSG_ARRAY_TEST_0_FIELD_AR_U8_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ARRAY_TEST_0 { \
    17150, \
    "ARRAY_TEST_0", \
    5, \
    {  { "v1", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_array_test_0_t, v1) }, \
         { "ar_i8", NULL, MAVLINK_TYPE_INT8_T, 4, 25, offsetof(mavlink_array_test_0_t, ar_i8) }, \
         { "ar_u8", NULL, MAVLINK_TYPE_UINT8_T, 4, 29, offsetof(mavlink_array_test_0_t, ar_u8) }, \
         { "ar_u16", NULL, MAVLINK_TYPE_UINT16_T, 4, 16, offsetof(mavlink_array_test_0_t, ar_u16) }, \
         { "ar_u32", NULL, MAVLINK_TYPE_UINT32_T, 4, 0, offsetof(mavlink_array_test_0_t, ar_u32) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ARRAY_TEST_0 { \
    "ARRAY_TEST_0", \
    5, \
    {  { "v1", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_array_test_0_t, v1) }, \
         { "ar_i8", NULL, MAVLINK_TYPE_INT8_T, 4, 25, offsetof(mavlink_array_test_0_t, ar_i8) }, \
         { "ar_u8", NULL, MAVLINK_TYPE_UINT8_T, 4, 29, offsetof(mavlink_array_test_0_t, ar_u8) }, \
         { "ar_u16", NULL, MAVLINK_TYPE_UINT16_T, 4, 16, offsetof(mavlink_array_test_0_t, ar_u16) }, \
         { "ar_u32", NULL, MAVLINK_TYPE_UINT32_T, 4, 0, offsetof(mavlink_array_test_0_t, ar_u32) }, \
         } \
}
#endif

/**
 * @brief Pack a array_test_0 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param v1  Stub field
 * @param ar_i8  Value array
 * @param ar_u8  Value array
 * @param ar_u16  Value array
 * @param ar_u32  Value array
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_array_test_0_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t v1, const int8_t *ar_i8, const uint8_t *ar_u8, const uint16_t *ar_u16, const uint32_t *ar_u32)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ARRAY_TEST_0_LEN];
    _mav_put_uint8_t(buf, 24, v1);
    _mav_put_uint32_t_array(buf, 0, ar_u32, 4);
    _mav_put_uint16_t_array(buf, 16, ar_u16, 4);
    _mav_put_int8_t_array(buf, 25, ar_i8, 4);
    _mav_put_uint8_t_array(buf, 29, ar_u8, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARRAY_TEST_0_LEN);
#else
    mavlink_array_test_0_t packet;
    packet.v1 = v1;
    mav_array_memcpy(packet.ar_u32, ar_u32, sizeof(uint32_t)*4);
    mav_array_memcpy(packet.ar_u16, ar_u16, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.ar_i8, ar_i8, sizeof(int8_t)*4);
    mav_array_memcpy(packet.ar_u8, ar_u8, sizeof(uint8_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARRAY_TEST_0_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ARRAY_TEST_0;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ARRAY_TEST_0_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_0_LEN, MAVLINK_MSG_ID_ARRAY_TEST_0_CRC);
}

/**
 * @brief Pack a array_test_0 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param v1  Stub field
 * @param ar_i8  Value array
 * @param ar_u8  Value array
 * @param ar_u16  Value array
 * @param ar_u32  Value array
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_array_test_0_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t v1, const int8_t *ar_i8, const uint8_t *ar_u8, const uint16_t *ar_u16, const uint32_t *ar_u32)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ARRAY_TEST_0_LEN];
    _mav_put_uint8_t(buf, 24, v1);
    _mav_put_uint32_t_array(buf, 0, ar_u32, 4);
    _mav_put_uint16_t_array(buf, 16, ar_u16, 4);
    _mav_put_int8_t_array(buf, 25, ar_i8, 4);
    _mav_put_uint8_t_array(buf, 29, ar_u8, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARRAY_TEST_0_LEN);
#else
    mavlink_array_test_0_t packet;
    packet.v1 = v1;
    mav_array_memcpy(packet.ar_u32, ar_u32, sizeof(uint32_t)*4);
    mav_array_memcpy(packet.ar_u16, ar_u16, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.ar_i8, ar_i8, sizeof(int8_t)*4);
    mav_array_memcpy(packet.ar_u8, ar_u8, sizeof(uint8_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARRAY_TEST_0_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ARRAY_TEST_0;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ARRAY_TEST_0_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_0_LEN, MAVLINK_MSG_ID_ARRAY_TEST_0_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ARRAY_TEST_0_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_0_LEN);
#endif
}

/**
 * @brief Pack a array_test_0 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param v1  Stub field
 * @param ar_i8  Value array
 * @param ar_u8  Value array
 * @param ar_u16  Value array
 * @param ar_u32  Value array
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_array_test_0_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t v1,const int8_t *ar_i8,const uint8_t *ar_u8,const uint16_t *ar_u16,const uint32_t *ar_u32)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ARRAY_TEST_0_LEN];
    _mav_put_uint8_t(buf, 24, v1);
    _mav_put_uint32_t_array(buf, 0, ar_u32, 4);
    _mav_put_uint16_t_array(buf, 16, ar_u16, 4);
    _mav_put_int8_t_array(buf, 25, ar_i8, 4);
    _mav_put_uint8_t_array(buf, 29, ar_u8, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARRAY_TEST_0_LEN);
#else
    mavlink_array_test_0_t packet;
    packet.v1 = v1;
    mav_array_memcpy(packet.ar_u32, ar_u32, sizeof(uint32_t)*4);
    mav_array_memcpy(packet.ar_u16, ar_u16, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.ar_i8, ar_i8, sizeof(int8_t)*4);
    mav_array_memcpy(packet.ar_u8, ar_u8, sizeof(uint8_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARRAY_TEST_0_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ARRAY_TEST_0;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ARRAY_TEST_0_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_0_LEN, MAVLINK_MSG_ID_ARRAY_TEST_0_CRC);
}

/**
 * @brief Encode a array_test_0 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param array_test_0 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_array_test_0_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_array_test_0_t* array_test_0)
{
    return mavlink_msg_array_test_0_pack(system_id, component_id, msg, array_test_0->v1, array_test_0->ar_i8, array_test_0->ar_u8, array_test_0->ar_u16, array_test_0->ar_u32);
}

/**
 * @brief Encode a array_test_0 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param array_test_0 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_array_test_0_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_array_test_0_t* array_test_0)
{
    return mavlink_msg_array_test_0_pack_chan(system_id, component_id, chan, msg, array_test_0->v1, array_test_0->ar_i8, array_test_0->ar_u8, array_test_0->ar_u16, array_test_0->ar_u32);
}

/**
 * @brief Encode a array_test_0 struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param array_test_0 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_array_test_0_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_array_test_0_t* array_test_0)
{
    return mavlink_msg_array_test_0_pack_status(system_id, component_id, _status, msg,  array_test_0->v1, array_test_0->ar_i8, array_test_0->ar_u8, array_test_0->ar_u16, array_test_0->ar_u32);
}

/**
 * @brief Send a array_test_0 message
 * @param chan MAVLink channel to send the message
 *
 * @param v1  Stub field
 * @param ar_i8  Value array
 * @param ar_u8  Value array
 * @param ar_u16  Value array
 * @param ar_u32  Value array
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_array_test_0_send(mavlink_channel_t chan, uint8_t v1, const int8_t *ar_i8, const uint8_t *ar_u8, const uint16_t *ar_u16, const uint32_t *ar_u32)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ARRAY_TEST_0_LEN];
    _mav_put_uint8_t(buf, 24, v1);
    _mav_put_uint32_t_array(buf, 0, ar_u32, 4);
    _mav_put_uint16_t_array(buf, 16, ar_u16, 4);
    _mav_put_int8_t_array(buf, 25, ar_i8, 4);
    _mav_put_uint8_t_array(buf, 29, ar_u8, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARRAY_TEST_0, buf, MAVLINK_MSG_ID_ARRAY_TEST_0_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_0_LEN, MAVLINK_MSG_ID_ARRAY_TEST_0_CRC);
#else
    mavlink_array_test_0_t packet;
    packet.v1 = v1;
    mav_array_memcpy(packet.ar_u32, ar_u32, sizeof(uint32_t)*4);
    mav_array_memcpy(packet.ar_u16, ar_u16, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.ar_i8, ar_i8, sizeof(int8_t)*4);
    mav_array_memcpy(packet.ar_u8, ar_u8, sizeof(uint8_t)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARRAY_TEST_0, (const char *)&packet, MAVLINK_MSG_ID_ARRAY_TEST_0_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_0_LEN, MAVLINK_MSG_ID_ARRAY_TEST_0_CRC);
#endif
}

/**
 * @brief Send a array_test_0 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_array_test_0_send_struct(mavlink_channel_t chan, const mavlink_array_test_0_t* array_test_0)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_array_test_0_send(chan, array_test_0->v1, array_test_0->ar_i8, array_test_0->ar_u8, array_test_0->ar_u16, array_test_0->ar_u32);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARRAY_TEST_0, (const char *)array_test_0, MAVLINK_MSG_ID_ARRAY_TEST_0_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_0_LEN, MAVLINK_MSG_ID_ARRAY_TEST_0_CRC);
#endif
}

#if MAVLINK_MSG_ID_ARRAY_TEST_0_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_array_test_0_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t v1, const int8_t *ar_i8, const uint8_t *ar_u8, const uint16_t *ar_u16, const uint32_t *ar_u32)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 24, v1);
    _mav_put_uint32_t_array(buf, 0, ar_u32, 4);
    _mav_put_uint16_t_array(buf, 16, ar_u16, 4);
    _mav_put_int8_t_array(buf, 25, ar_i8, 4);
    _mav_put_uint8_t_array(buf, 29, ar_u8, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARRAY_TEST_0, buf, MAVLINK_MSG_ID_ARRAY_TEST_0_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_0_LEN, MAVLINK_MSG_ID_ARRAY_TEST_0_CRC);
#else
    mavlink_array_test_0_t *packet = (mavlink_array_test_0_t *)msgbuf;
    packet->v1 = v1;
    mav_array_memcpy(packet->ar_u32, ar_u32, sizeof(uint32_t)*4);
    mav_array_memcpy(packet->ar_u16, ar_u16, sizeof(uint16_t)*4);
    mav_array_memcpy(packet->ar_i8, ar_i8, sizeof(int8_t)*4);
    mav_array_memcpy(packet->ar_u8, ar_u8, sizeof(uint8_t)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARRAY_TEST_0, (const char *)packet, MAVLINK_MSG_ID_ARRAY_TEST_0_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_0_LEN, MAVLINK_MSG_ID_ARRAY_TEST_0_CRC);
#endif
}
#endif

#endif

// MESSAGE ARRAY_TEST_0 UNPACKING


/**
 * @brief Get field v1 from array_test_0 message
 *
 * @return  Stub field
 */
static inline uint8_t mavlink_msg_array_test_0_get_v1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field ar_i8 from array_test_0 message
 *
 * @return  Value array
 */
static inline uint16_t mavlink_msg_array_test_0_get_ar_i8(const mavlink_message_t* msg, int8_t *ar_i8)
{
    return _MAV_RETURN_int8_t_array(msg, ar_i8, 4,  25);
}

/**
 * @brief Get field ar_u8 from array_test_0 message
 *
 * @return  Value array
 */
static inline uint16_t mavlink_msg_array_test_0_get_ar_u8(const mavlink_message_t* msg, uint8_t *ar_u8)
{
    return _MAV_RETURN_uint8_t_array(msg, ar_u8, 4,  29);
}

/**
 * @brief Get field ar_u16 from array_test_0 message
 *
 * @return  Value array
 */
static inline uint16_t mavlink_msg_array_test_0_get_ar_u16(const mavlink_message_t* msg, uint16_t *ar_u16)
{
    return _MAV_RETURN_uint16_t_array(msg, ar_u16, 4,  16);
}

/**
 * @brief Get field ar_u32 from array_test_0 message
 *
 * @return  Value array
 */
static inline uint16_t mavlink_msg_array_test_0_get_ar_u32(const mavlink_message_t* msg, uint32_t *ar_u32)
{
    return _MAV_RETURN_uint32_t_array(msg, ar_u32, 4,  0);
}

/**
 * @brief Decode a array_test_0 message into a struct
 *
 * @param msg The message to decode
 * @param array_test_0 C-struct to decode the message contents into
 */
static inline void mavlink_msg_array_test_0_decode(const mavlink_message_t* msg, mavlink_array_test_0_t* array_test_0)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_array_test_0_get_ar_u32(msg, array_test_0->ar_u32);
    mavlink_msg_array_test_0_get_ar_u16(msg, array_test_0->ar_u16);
    array_test_0->v1 = mavlink_msg_array_test_0_get_v1(msg);
    mavlink_msg_array_test_0_get_ar_i8(msg, array_test_0->ar_i8);
    mavlink_msg_array_test_0_get_ar_u8(msg, array_test_0->ar_u8);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ARRAY_TEST_0_LEN? msg->len : MAVLINK_MSG_ID_ARRAY_TEST_0_LEN;
        memset(array_test_0, 0, MAVLINK_MSG_ID_ARRAY_TEST_0_LEN);
    memcpy(array_test_0, _MAV_PAYLOAD(msg), len);
#endif
}
