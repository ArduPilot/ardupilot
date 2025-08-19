#pragma once
// MESSAGE ARRAY_TEST_3 PACKING

#define MAVLINK_MSG_ID_ARRAY_TEST_3 17153


typedef struct __mavlink_array_test_3_t {
 uint32_t ar_u32[4]; /*<  Value array*/
 uint8_t v; /*<  Stub field*/
} mavlink_array_test_3_t;

#define MAVLINK_MSG_ID_ARRAY_TEST_3_LEN 17
#define MAVLINK_MSG_ID_ARRAY_TEST_3_MIN_LEN 17
#define MAVLINK_MSG_ID_17153_LEN 17
#define MAVLINK_MSG_ID_17153_MIN_LEN 17

#define MAVLINK_MSG_ID_ARRAY_TEST_3_CRC 19
#define MAVLINK_MSG_ID_17153_CRC 19

#define MAVLINK_MSG_ARRAY_TEST_3_FIELD_AR_U32_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ARRAY_TEST_3 { \
    17153, \
    "ARRAY_TEST_3", \
    2, \
    {  { "v", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_array_test_3_t, v) }, \
         { "ar_u32", NULL, MAVLINK_TYPE_UINT32_T, 4, 0, offsetof(mavlink_array_test_3_t, ar_u32) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ARRAY_TEST_3 { \
    "ARRAY_TEST_3", \
    2, \
    {  { "v", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_array_test_3_t, v) }, \
         { "ar_u32", NULL, MAVLINK_TYPE_UINT32_T, 4, 0, offsetof(mavlink_array_test_3_t, ar_u32) }, \
         } \
}
#endif

/**
 * @brief Pack a array_test_3 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param v  Stub field
 * @param ar_u32  Value array
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_array_test_3_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t v, const uint32_t *ar_u32)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ARRAY_TEST_3_LEN];
    _mav_put_uint8_t(buf, 16, v);
    _mav_put_uint32_t_array(buf, 0, ar_u32, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARRAY_TEST_3_LEN);
#else
    mavlink_array_test_3_t packet;
    packet.v = v;
    mav_array_memcpy(packet.ar_u32, ar_u32, sizeof(uint32_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARRAY_TEST_3_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ARRAY_TEST_3;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ARRAY_TEST_3_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_3_LEN, MAVLINK_MSG_ID_ARRAY_TEST_3_CRC);
}

/**
 * @brief Pack a array_test_3 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param v  Stub field
 * @param ar_u32  Value array
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_array_test_3_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t v, const uint32_t *ar_u32)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ARRAY_TEST_3_LEN];
    _mav_put_uint8_t(buf, 16, v);
    _mav_put_uint32_t_array(buf, 0, ar_u32, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARRAY_TEST_3_LEN);
#else
    mavlink_array_test_3_t packet;
    packet.v = v;
    mav_array_memcpy(packet.ar_u32, ar_u32, sizeof(uint32_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARRAY_TEST_3_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ARRAY_TEST_3;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ARRAY_TEST_3_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_3_LEN, MAVLINK_MSG_ID_ARRAY_TEST_3_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ARRAY_TEST_3_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_3_LEN);
#endif
}

/**
 * @brief Pack a array_test_3 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param v  Stub field
 * @param ar_u32  Value array
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_array_test_3_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t v,const uint32_t *ar_u32)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ARRAY_TEST_3_LEN];
    _mav_put_uint8_t(buf, 16, v);
    _mav_put_uint32_t_array(buf, 0, ar_u32, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARRAY_TEST_3_LEN);
#else
    mavlink_array_test_3_t packet;
    packet.v = v;
    mav_array_memcpy(packet.ar_u32, ar_u32, sizeof(uint32_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARRAY_TEST_3_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ARRAY_TEST_3;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ARRAY_TEST_3_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_3_LEN, MAVLINK_MSG_ID_ARRAY_TEST_3_CRC);
}

/**
 * @brief Encode a array_test_3 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param array_test_3 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_array_test_3_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_array_test_3_t* array_test_3)
{
    return mavlink_msg_array_test_3_pack(system_id, component_id, msg, array_test_3->v, array_test_3->ar_u32);
}

/**
 * @brief Encode a array_test_3 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param array_test_3 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_array_test_3_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_array_test_3_t* array_test_3)
{
    return mavlink_msg_array_test_3_pack_chan(system_id, component_id, chan, msg, array_test_3->v, array_test_3->ar_u32);
}

/**
 * @brief Encode a array_test_3 struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param array_test_3 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_array_test_3_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_array_test_3_t* array_test_3)
{
    return mavlink_msg_array_test_3_pack_status(system_id, component_id, _status, msg,  array_test_3->v, array_test_3->ar_u32);
}

/**
 * @brief Send a array_test_3 message
 * @param chan MAVLink channel to send the message
 *
 * @param v  Stub field
 * @param ar_u32  Value array
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_array_test_3_send(mavlink_channel_t chan, uint8_t v, const uint32_t *ar_u32)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ARRAY_TEST_3_LEN];
    _mav_put_uint8_t(buf, 16, v);
    _mav_put_uint32_t_array(buf, 0, ar_u32, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARRAY_TEST_3, buf, MAVLINK_MSG_ID_ARRAY_TEST_3_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_3_LEN, MAVLINK_MSG_ID_ARRAY_TEST_3_CRC);
#else
    mavlink_array_test_3_t packet;
    packet.v = v;
    mav_array_memcpy(packet.ar_u32, ar_u32, sizeof(uint32_t)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARRAY_TEST_3, (const char *)&packet, MAVLINK_MSG_ID_ARRAY_TEST_3_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_3_LEN, MAVLINK_MSG_ID_ARRAY_TEST_3_CRC);
#endif
}

/**
 * @brief Send a array_test_3 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_array_test_3_send_struct(mavlink_channel_t chan, const mavlink_array_test_3_t* array_test_3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_array_test_3_send(chan, array_test_3->v, array_test_3->ar_u32);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARRAY_TEST_3, (const char *)array_test_3, MAVLINK_MSG_ID_ARRAY_TEST_3_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_3_LEN, MAVLINK_MSG_ID_ARRAY_TEST_3_CRC);
#endif
}

#if MAVLINK_MSG_ID_ARRAY_TEST_3_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_array_test_3_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t v, const uint32_t *ar_u32)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 16, v);
    _mav_put_uint32_t_array(buf, 0, ar_u32, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARRAY_TEST_3, buf, MAVLINK_MSG_ID_ARRAY_TEST_3_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_3_LEN, MAVLINK_MSG_ID_ARRAY_TEST_3_CRC);
#else
    mavlink_array_test_3_t *packet = (mavlink_array_test_3_t *)msgbuf;
    packet->v = v;
    mav_array_memcpy(packet->ar_u32, ar_u32, sizeof(uint32_t)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARRAY_TEST_3, (const char *)packet, MAVLINK_MSG_ID_ARRAY_TEST_3_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_3_LEN, MAVLINK_MSG_ID_ARRAY_TEST_3_CRC);
#endif
}
#endif

#endif

// MESSAGE ARRAY_TEST_3 UNPACKING


/**
 * @brief Get field v from array_test_3 message
 *
 * @return  Stub field
 */
static inline uint8_t mavlink_msg_array_test_3_get_v(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field ar_u32 from array_test_3 message
 *
 * @return  Value array
 */
static inline uint16_t mavlink_msg_array_test_3_get_ar_u32(const mavlink_message_t* msg, uint32_t *ar_u32)
{
    return _MAV_RETURN_uint32_t_array(msg, ar_u32, 4,  0);
}

/**
 * @brief Decode a array_test_3 message into a struct
 *
 * @param msg The message to decode
 * @param array_test_3 C-struct to decode the message contents into
 */
static inline void mavlink_msg_array_test_3_decode(const mavlink_message_t* msg, mavlink_array_test_3_t* array_test_3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_array_test_3_get_ar_u32(msg, array_test_3->ar_u32);
    array_test_3->v = mavlink_msg_array_test_3_get_v(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ARRAY_TEST_3_LEN? msg->len : MAVLINK_MSG_ID_ARRAY_TEST_3_LEN;
        memset(array_test_3, 0, MAVLINK_MSG_ID_ARRAY_TEST_3_LEN);
    memcpy(array_test_3, _MAV_PAYLOAD(msg), len);
#endif
}
