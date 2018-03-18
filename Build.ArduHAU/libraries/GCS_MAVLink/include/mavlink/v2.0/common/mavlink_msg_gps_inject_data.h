#pragma once
// MESSAGE GPS_INJECT_DATA PACKING

#define MAVLINK_MSG_ID_GPS_INJECT_DATA 123

MAVPACKED(
typedef struct __mavlink_gps_inject_data_t {
 uint8_t target_system; /*< System ID*/
 uint8_t target_component; /*< Component ID*/
 uint8_t len; /*< data length*/
 uint8_t data[110]; /*< raw data (110 is enough for 12 satellites of RTCMv2)*/
}) mavlink_gps_inject_data_t;

#define MAVLINK_MSG_ID_GPS_INJECT_DATA_LEN 113
#define MAVLINK_MSG_ID_GPS_INJECT_DATA_MIN_LEN 113
#define MAVLINK_MSG_ID_123_LEN 113
#define MAVLINK_MSG_ID_123_MIN_LEN 113

#define MAVLINK_MSG_ID_GPS_INJECT_DATA_CRC 250
#define MAVLINK_MSG_ID_123_CRC 250

#define MAVLINK_MSG_GPS_INJECT_DATA_FIELD_DATA_LEN 110

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GPS_INJECT_DATA { \
    123, \
    "GPS_INJECT_DATA", \
    4, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_gps_inject_data_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_gps_inject_data_t, target_component) }, \
         { "len", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_gps_inject_data_t, len) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 110, 3, offsetof(mavlink_gps_inject_data_t, data) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GPS_INJECT_DATA { \
    "GPS_INJECT_DATA", \
    4, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_gps_inject_data_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_gps_inject_data_t, target_component) }, \
         { "len", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_gps_inject_data_t, len) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 110, 3, offsetof(mavlink_gps_inject_data_t, data) }, \
         } \
}
#endif

/**
 * @brief Pack a gps_inject_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param len data length
 * @param data raw data (110 is enough for 12 satellites of RTCMv2)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_inject_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t len, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS_INJECT_DATA_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, len);
    _mav_put_uint8_t_array(buf, 3, data, 110);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS_INJECT_DATA_LEN);
#else
    mavlink_gps_inject_data_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.len = len;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*110);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GPS_INJECT_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GPS_INJECT_DATA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GPS_INJECT_DATA_MIN_LEN, MAVLINK_MSG_ID_GPS_INJECT_DATA_LEN, MAVLINK_MSG_ID_GPS_INJECT_DATA_CRC);
}

/**
 * @brief Pack a gps_inject_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param len data length
 * @param data raw data (110 is enough for 12 satellites of RTCMv2)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_inject_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint8_t len,const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS_INJECT_DATA_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, len);
    _mav_put_uint8_t_array(buf, 3, data, 110);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS_INJECT_DATA_LEN);
#else
    mavlink_gps_inject_data_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.len = len;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*110);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GPS_INJECT_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GPS_INJECT_DATA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GPS_INJECT_DATA_MIN_LEN, MAVLINK_MSG_ID_GPS_INJECT_DATA_LEN, MAVLINK_MSG_ID_GPS_INJECT_DATA_CRC);
}

/**
 * @brief Encode a gps_inject_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gps_inject_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_inject_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gps_inject_data_t* gps_inject_data)
{
    return mavlink_msg_gps_inject_data_pack(system_id, component_id, msg, gps_inject_data->target_system, gps_inject_data->target_component, gps_inject_data->len, gps_inject_data->data);
}

/**
 * @brief Encode a gps_inject_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gps_inject_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_inject_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gps_inject_data_t* gps_inject_data)
{
    return mavlink_msg_gps_inject_data_pack_chan(system_id, component_id, chan, msg, gps_inject_data->target_system, gps_inject_data->target_component, gps_inject_data->len, gps_inject_data->data);
}

/**
 * @brief Send a gps_inject_data message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param len data length
 * @param data raw data (110 is enough for 12 satellites of RTCMv2)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gps_inject_data_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t len, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS_INJECT_DATA_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, len);
    _mav_put_uint8_t_array(buf, 3, data, 110);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_INJECT_DATA, buf, MAVLINK_MSG_ID_GPS_INJECT_DATA_MIN_LEN, MAVLINK_MSG_ID_GPS_INJECT_DATA_LEN, MAVLINK_MSG_ID_GPS_INJECT_DATA_CRC);
#else
    mavlink_gps_inject_data_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.len = len;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*110);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_INJECT_DATA, (const char *)&packet, MAVLINK_MSG_ID_GPS_INJECT_DATA_MIN_LEN, MAVLINK_MSG_ID_GPS_INJECT_DATA_LEN, MAVLINK_MSG_ID_GPS_INJECT_DATA_CRC);
#endif
}

/**
 * @brief Send a gps_inject_data message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gps_inject_data_send_struct(mavlink_channel_t chan, const mavlink_gps_inject_data_t* gps_inject_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gps_inject_data_send(chan, gps_inject_data->target_system, gps_inject_data->target_component, gps_inject_data->len, gps_inject_data->data);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_INJECT_DATA, (const char *)gps_inject_data, MAVLINK_MSG_ID_GPS_INJECT_DATA_MIN_LEN, MAVLINK_MSG_ID_GPS_INJECT_DATA_LEN, MAVLINK_MSG_ID_GPS_INJECT_DATA_CRC);
#endif
}

#if MAVLINK_MSG_ID_GPS_INJECT_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gps_inject_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t len, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, len);
    _mav_put_uint8_t_array(buf, 3, data, 110);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_INJECT_DATA, buf, MAVLINK_MSG_ID_GPS_INJECT_DATA_MIN_LEN, MAVLINK_MSG_ID_GPS_INJECT_DATA_LEN, MAVLINK_MSG_ID_GPS_INJECT_DATA_CRC);
#else
    mavlink_gps_inject_data_t *packet = (mavlink_gps_inject_data_t *)msgbuf;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->len = len;
    mav_array_memcpy(packet->data, data, sizeof(uint8_t)*110);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_INJECT_DATA, (const char *)packet, MAVLINK_MSG_ID_GPS_INJECT_DATA_MIN_LEN, MAVLINK_MSG_ID_GPS_INJECT_DATA_LEN, MAVLINK_MSG_ID_GPS_INJECT_DATA_CRC);
#endif
}
#endif

#endif

// MESSAGE GPS_INJECT_DATA UNPACKING


/**
 * @brief Get field target_system from gps_inject_data message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_gps_inject_data_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field target_component from gps_inject_data message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_gps_inject_data_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field len from gps_inject_data message
 *
 * @return data length
 */
static inline uint8_t mavlink_msg_gps_inject_data_get_len(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field data from gps_inject_data message
 *
 * @return raw data (110 is enough for 12 satellites of RTCMv2)
 */
static inline uint16_t mavlink_msg_gps_inject_data_get_data(const mavlink_message_t* msg, uint8_t *data)
{
    return _MAV_RETURN_uint8_t_array(msg, data, 110,  3);
}

/**
 * @brief Decode a gps_inject_data message into a struct
 *
 * @param msg The message to decode
 * @param gps_inject_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_gps_inject_data_decode(const mavlink_message_t* msg, mavlink_gps_inject_data_t* gps_inject_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gps_inject_data->target_system = mavlink_msg_gps_inject_data_get_target_system(msg);
    gps_inject_data->target_component = mavlink_msg_gps_inject_data_get_target_component(msg);
    gps_inject_data->len = mavlink_msg_gps_inject_data_get_len(msg);
    mavlink_msg_gps_inject_data_get_data(msg, gps_inject_data->data);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GPS_INJECT_DATA_LEN? msg->len : MAVLINK_MSG_ID_GPS_INJECT_DATA_LEN;
        memset(gps_inject_data, 0, MAVLINK_MSG_ID_GPS_INJECT_DATA_LEN);
    memcpy(gps_inject_data, _MAV_PAYLOAD(msg), len);
#endif
}
