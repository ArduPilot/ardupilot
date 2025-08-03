#pragma once
// MESSAGE ARRAY_TEST_8 PACKING

#define MAVLINK_MSG_ID_ARRAY_TEST_8 17158


typedef struct __mavlink_array_test_8_t {
 double ar_d[2]; /*<  Value array*/
 uint32_t v3; /*<  Stub field*/
 uint16_t ar_u16[2]; /*<  Value array*/
} mavlink_array_test_8_t;

#define MAVLINK_MSG_ID_ARRAY_TEST_8_LEN 24
#define MAVLINK_MSG_ID_ARRAY_TEST_8_MIN_LEN 24
#define MAVLINK_MSG_ID_17158_LEN 24
#define MAVLINK_MSG_ID_17158_MIN_LEN 24

#define MAVLINK_MSG_ID_ARRAY_TEST_8_CRC 106
#define MAVLINK_MSG_ID_17158_CRC 106

#define MAVLINK_MSG_ARRAY_TEST_8_FIELD_AR_D_LEN 2
#define MAVLINK_MSG_ARRAY_TEST_8_FIELD_AR_U16_LEN 2

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ARRAY_TEST_8 { \
    17158, \
    "ARRAY_TEST_8", \
    3, \
    {  { "v3", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_array_test_8_t, v3) }, \
         { "ar_d", NULL, MAVLINK_TYPE_DOUBLE, 2, 0, offsetof(mavlink_array_test_8_t, ar_d) }, \
         { "ar_u16", NULL, MAVLINK_TYPE_UINT16_T, 2, 20, offsetof(mavlink_array_test_8_t, ar_u16) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ARRAY_TEST_8 { \
    "ARRAY_TEST_8", \
    3, \
    {  { "v3", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_array_test_8_t, v3) }, \
         { "ar_d", NULL, MAVLINK_TYPE_DOUBLE, 2, 0, offsetof(mavlink_array_test_8_t, ar_d) }, \
         { "ar_u16", NULL, MAVLINK_TYPE_UINT16_T, 2, 20, offsetof(mavlink_array_test_8_t, ar_u16) }, \
         } \
}
#endif

/**
 * @brief Pack a array_test_8 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param v3  Stub field
 * @param ar_d  Value array
 * @param ar_u16  Value array
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_array_test_8_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t v3, const double *ar_d, const uint16_t *ar_u16)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ARRAY_TEST_8_LEN];
    _mav_put_uint32_t(buf, 16, v3);
    _mav_put_double_array(buf, 0, ar_d, 2);
    _mav_put_uint16_t_array(buf, 20, ar_u16, 2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARRAY_TEST_8_LEN);
#else
    mavlink_array_test_8_t packet;
    packet.v3 = v3;
    mav_array_memcpy(packet.ar_d, ar_d, sizeof(double)*2);
    mav_array_memcpy(packet.ar_u16, ar_u16, sizeof(uint16_t)*2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARRAY_TEST_8_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ARRAY_TEST_8;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ARRAY_TEST_8_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_8_LEN, MAVLINK_MSG_ID_ARRAY_TEST_8_CRC);
}

/**
 * @brief Pack a array_test_8 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param v3  Stub field
 * @param ar_d  Value array
 * @param ar_u16  Value array
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_array_test_8_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t v3, const double *ar_d, const uint16_t *ar_u16)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ARRAY_TEST_8_LEN];
    _mav_put_uint32_t(buf, 16, v3);
    _mav_put_double_array(buf, 0, ar_d, 2);
    _mav_put_uint16_t_array(buf, 20, ar_u16, 2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARRAY_TEST_8_LEN);
#else
    mavlink_array_test_8_t packet;
    packet.v3 = v3;
    mav_array_memcpy(packet.ar_d, ar_d, sizeof(double)*2);
    mav_array_memcpy(packet.ar_u16, ar_u16, sizeof(uint16_t)*2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARRAY_TEST_8_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ARRAY_TEST_8;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ARRAY_TEST_8_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_8_LEN, MAVLINK_MSG_ID_ARRAY_TEST_8_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ARRAY_TEST_8_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_8_LEN);
#endif
}

/**
 * @brief Pack a array_test_8 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param v3  Stub field
 * @param ar_d  Value array
 * @param ar_u16  Value array
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_array_test_8_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t v3,const double *ar_d,const uint16_t *ar_u16)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ARRAY_TEST_8_LEN];
    _mav_put_uint32_t(buf, 16, v3);
    _mav_put_double_array(buf, 0, ar_d, 2);
    _mav_put_uint16_t_array(buf, 20, ar_u16, 2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARRAY_TEST_8_LEN);
#else
    mavlink_array_test_8_t packet;
    packet.v3 = v3;
    mav_array_memcpy(packet.ar_d, ar_d, sizeof(double)*2);
    mav_array_memcpy(packet.ar_u16, ar_u16, sizeof(uint16_t)*2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARRAY_TEST_8_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ARRAY_TEST_8;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ARRAY_TEST_8_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_8_LEN, MAVLINK_MSG_ID_ARRAY_TEST_8_CRC);
}

/**
 * @brief Encode a array_test_8 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param array_test_8 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_array_test_8_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_array_test_8_t* array_test_8)
{
    return mavlink_msg_array_test_8_pack(system_id, component_id, msg, array_test_8->v3, array_test_8->ar_d, array_test_8->ar_u16);
}

/**
 * @brief Encode a array_test_8 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param array_test_8 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_array_test_8_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_array_test_8_t* array_test_8)
{
    return mavlink_msg_array_test_8_pack_chan(system_id, component_id, chan, msg, array_test_8->v3, array_test_8->ar_d, array_test_8->ar_u16);
}

/**
 * @brief Encode a array_test_8 struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param array_test_8 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_array_test_8_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_array_test_8_t* array_test_8)
{
    return mavlink_msg_array_test_8_pack_status(system_id, component_id, _status, msg,  array_test_8->v3, array_test_8->ar_d, array_test_8->ar_u16);
}

/**
 * @brief Send a array_test_8 message
 * @param chan MAVLink channel to send the message
 *
 * @param v3  Stub field
 * @param ar_d  Value array
 * @param ar_u16  Value array
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_array_test_8_send(mavlink_channel_t chan, uint32_t v3, const double *ar_d, const uint16_t *ar_u16)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ARRAY_TEST_8_LEN];
    _mav_put_uint32_t(buf, 16, v3);
    _mav_put_double_array(buf, 0, ar_d, 2);
    _mav_put_uint16_t_array(buf, 20, ar_u16, 2);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARRAY_TEST_8, buf, MAVLINK_MSG_ID_ARRAY_TEST_8_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_8_LEN, MAVLINK_MSG_ID_ARRAY_TEST_8_CRC);
#else
    mavlink_array_test_8_t packet;
    packet.v3 = v3;
    mav_array_memcpy(packet.ar_d, ar_d, sizeof(double)*2);
    mav_array_memcpy(packet.ar_u16, ar_u16, sizeof(uint16_t)*2);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARRAY_TEST_8, (const char *)&packet, MAVLINK_MSG_ID_ARRAY_TEST_8_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_8_LEN, MAVLINK_MSG_ID_ARRAY_TEST_8_CRC);
#endif
}

/**
 * @brief Send a array_test_8 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_array_test_8_send_struct(mavlink_channel_t chan, const mavlink_array_test_8_t* array_test_8)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_array_test_8_send(chan, array_test_8->v3, array_test_8->ar_d, array_test_8->ar_u16);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARRAY_TEST_8, (const char *)array_test_8, MAVLINK_MSG_ID_ARRAY_TEST_8_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_8_LEN, MAVLINK_MSG_ID_ARRAY_TEST_8_CRC);
#endif
}

#if MAVLINK_MSG_ID_ARRAY_TEST_8_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_array_test_8_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t v3, const double *ar_d, const uint16_t *ar_u16)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 16, v3);
    _mav_put_double_array(buf, 0, ar_d, 2);
    _mav_put_uint16_t_array(buf, 20, ar_u16, 2);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARRAY_TEST_8, buf, MAVLINK_MSG_ID_ARRAY_TEST_8_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_8_LEN, MAVLINK_MSG_ID_ARRAY_TEST_8_CRC);
#else
    mavlink_array_test_8_t *packet = (mavlink_array_test_8_t *)msgbuf;
    packet->v3 = v3;
    mav_array_memcpy(packet->ar_d, ar_d, sizeof(double)*2);
    mav_array_memcpy(packet->ar_u16, ar_u16, sizeof(uint16_t)*2);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARRAY_TEST_8, (const char *)packet, MAVLINK_MSG_ID_ARRAY_TEST_8_MIN_LEN, MAVLINK_MSG_ID_ARRAY_TEST_8_LEN, MAVLINK_MSG_ID_ARRAY_TEST_8_CRC);
#endif
}
#endif

#endif

// MESSAGE ARRAY_TEST_8 UNPACKING


/**
 * @brief Get field v3 from array_test_8 message
 *
 * @return  Stub field
 */
static inline uint32_t mavlink_msg_array_test_8_get_v3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  16);
}

/**
 * @brief Get field ar_d from array_test_8 message
 *
 * @return  Value array
 */
static inline uint16_t mavlink_msg_array_test_8_get_ar_d(const mavlink_message_t* msg, double *ar_d)
{
    return _MAV_RETURN_double_array(msg, ar_d, 2,  0);
}

/**
 * @brief Get field ar_u16 from array_test_8 message
 *
 * @return  Value array
 */
static inline uint16_t mavlink_msg_array_test_8_get_ar_u16(const mavlink_message_t* msg, uint16_t *ar_u16)
{
    return _MAV_RETURN_uint16_t_array(msg, ar_u16, 2,  20);
}

/**
 * @brief Decode a array_test_8 message into a struct
 *
 * @param msg The message to decode
 * @param array_test_8 C-struct to decode the message contents into
 */
static inline void mavlink_msg_array_test_8_decode(const mavlink_message_t* msg, mavlink_array_test_8_t* array_test_8)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_array_test_8_get_ar_d(msg, array_test_8->ar_d);
    array_test_8->v3 = mavlink_msg_array_test_8_get_v3(msg);
    mavlink_msg_array_test_8_get_ar_u16(msg, array_test_8->ar_u16);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ARRAY_TEST_8_LEN? msg->len : MAVLINK_MSG_ID_ARRAY_TEST_8_LEN;
        memset(array_test_8, 0, MAVLINK_MSG_ID_ARRAY_TEST_8_LEN);
    memcpy(array_test_8, _MAV_PAYLOAD(msg), len);
#endif
}
