#pragma once
// MESSAGE GPS_RTCM_DATA PACKING

#define MAVLINK_MSG_ID_GPS_RTCM_DATA 233

MAVPACKED(
typedef struct __mavlink_gps_rtcm_data_t {
 uint8_t flags; /*< LSB: 1 means message is fragmented, next 2 bits are the fragment ID, the remaining 5 bits are used for the sequence ID. Messages are only to be flushed to the GPS when the entire message has been reconstructed on the autopilot. The fragment ID specifies which order the fragments should be assembled into a buffer, while the sequence ID is used to detect a mismatch between different buffers. The buffer is considered fully reconstructed when either all 4 fragments are present, or all the fragments before the first fragment with a non full payload is received. This management is used to ensure that normal GPS operation doesn't corrupt RTCM data, and to recover from a unreliable transport delivery order.*/
 uint8_t len; /*< data length*/
 uint8_t data[180]; /*< RTCM message (may be fragmented)*/
}) mavlink_gps_rtcm_data_t;

#define MAVLINK_MSG_ID_GPS_RTCM_DATA_LEN 182
#define MAVLINK_MSG_ID_GPS_RTCM_DATA_MIN_LEN 182
#define MAVLINK_MSG_ID_233_LEN 182
#define MAVLINK_MSG_ID_233_MIN_LEN 182

#define MAVLINK_MSG_ID_GPS_RTCM_DATA_CRC 35
#define MAVLINK_MSG_ID_233_CRC 35

#define MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN 180

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GPS_RTCM_DATA { \
    233, \
    "GPS_RTCM_DATA", \
    3, \
    {  { "flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_gps_rtcm_data_t, flags) }, \
         { "len", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_gps_rtcm_data_t, len) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 180, 2, offsetof(mavlink_gps_rtcm_data_t, data) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GPS_RTCM_DATA { \
    "GPS_RTCM_DATA", \
    3, \
    {  { "flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_gps_rtcm_data_t, flags) }, \
         { "len", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_gps_rtcm_data_t, len) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 180, 2, offsetof(mavlink_gps_rtcm_data_t, data) }, \
         } \
}
#endif

/**
 * @brief Pack a gps_rtcm_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param flags LSB: 1 means message is fragmented, next 2 bits are the fragment ID, the remaining 5 bits are used for the sequence ID. Messages are only to be flushed to the GPS when the entire message has been reconstructed on the autopilot. The fragment ID specifies which order the fragments should be assembled into a buffer, while the sequence ID is used to detect a mismatch between different buffers. The buffer is considered fully reconstructed when either all 4 fragments are present, or all the fragments before the first fragment with a non full payload is received. This management is used to ensure that normal GPS operation doesn't corrupt RTCM data, and to recover from a unreliable transport delivery order.
 * @param len data length
 * @param data RTCM message (may be fragmented)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_rtcm_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t flags, uint8_t len, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS_RTCM_DATA_LEN];
    _mav_put_uint8_t(buf, 0, flags);
    _mav_put_uint8_t(buf, 1, len);
    _mav_put_uint8_t_array(buf, 2, data, 180);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS_RTCM_DATA_LEN);
#else
    mavlink_gps_rtcm_data_t packet;
    packet.flags = flags;
    packet.len = len;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*180);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GPS_RTCM_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GPS_RTCM_DATA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GPS_RTCM_DATA_MIN_LEN, MAVLINK_MSG_ID_GPS_RTCM_DATA_LEN, MAVLINK_MSG_ID_GPS_RTCM_DATA_CRC);
}

/**
 * @brief Pack a gps_rtcm_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param flags LSB: 1 means message is fragmented, next 2 bits are the fragment ID, the remaining 5 bits are used for the sequence ID. Messages are only to be flushed to the GPS when the entire message has been reconstructed on the autopilot. The fragment ID specifies which order the fragments should be assembled into a buffer, while the sequence ID is used to detect a mismatch between different buffers. The buffer is considered fully reconstructed when either all 4 fragments are present, or all the fragments before the first fragment with a non full payload is received. This management is used to ensure that normal GPS operation doesn't corrupt RTCM data, and to recover from a unreliable transport delivery order.
 * @param len data length
 * @param data RTCM message (may be fragmented)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_rtcm_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t flags,uint8_t len,const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS_RTCM_DATA_LEN];
    _mav_put_uint8_t(buf, 0, flags);
    _mav_put_uint8_t(buf, 1, len);
    _mav_put_uint8_t_array(buf, 2, data, 180);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS_RTCM_DATA_LEN);
#else
    mavlink_gps_rtcm_data_t packet;
    packet.flags = flags;
    packet.len = len;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*180);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GPS_RTCM_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GPS_RTCM_DATA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GPS_RTCM_DATA_MIN_LEN, MAVLINK_MSG_ID_GPS_RTCM_DATA_LEN, MAVLINK_MSG_ID_GPS_RTCM_DATA_CRC);
}

/**
 * @brief Encode a gps_rtcm_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gps_rtcm_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_rtcm_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gps_rtcm_data_t* gps_rtcm_data)
{
    return mavlink_msg_gps_rtcm_data_pack(system_id, component_id, msg, gps_rtcm_data->flags, gps_rtcm_data->len, gps_rtcm_data->data);
}

/**
 * @brief Encode a gps_rtcm_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gps_rtcm_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_rtcm_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gps_rtcm_data_t* gps_rtcm_data)
{
    return mavlink_msg_gps_rtcm_data_pack_chan(system_id, component_id, chan, msg, gps_rtcm_data->flags, gps_rtcm_data->len, gps_rtcm_data->data);
}

/**
 * @brief Send a gps_rtcm_data message
 * @param chan MAVLink channel to send the message
 *
 * @param flags LSB: 1 means message is fragmented, next 2 bits are the fragment ID, the remaining 5 bits are used for the sequence ID. Messages are only to be flushed to the GPS when the entire message has been reconstructed on the autopilot. The fragment ID specifies which order the fragments should be assembled into a buffer, while the sequence ID is used to detect a mismatch between different buffers. The buffer is considered fully reconstructed when either all 4 fragments are present, or all the fragments before the first fragment with a non full payload is received. This management is used to ensure that normal GPS operation doesn't corrupt RTCM data, and to recover from a unreliable transport delivery order.
 * @param len data length
 * @param data RTCM message (may be fragmented)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gps_rtcm_data_send(mavlink_channel_t chan, uint8_t flags, uint8_t len, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS_RTCM_DATA_LEN];
    _mav_put_uint8_t(buf, 0, flags);
    _mav_put_uint8_t(buf, 1, len);
    _mav_put_uint8_t_array(buf, 2, data, 180);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_RTCM_DATA, buf, MAVLINK_MSG_ID_GPS_RTCM_DATA_MIN_LEN, MAVLINK_MSG_ID_GPS_RTCM_DATA_LEN, MAVLINK_MSG_ID_GPS_RTCM_DATA_CRC);
#else
    mavlink_gps_rtcm_data_t packet;
    packet.flags = flags;
    packet.len = len;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*180);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_RTCM_DATA, (const char *)&packet, MAVLINK_MSG_ID_GPS_RTCM_DATA_MIN_LEN, MAVLINK_MSG_ID_GPS_RTCM_DATA_LEN, MAVLINK_MSG_ID_GPS_RTCM_DATA_CRC);
#endif
}

/**
 * @brief Send a gps_rtcm_data message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gps_rtcm_data_send_struct(mavlink_channel_t chan, const mavlink_gps_rtcm_data_t* gps_rtcm_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gps_rtcm_data_send(chan, gps_rtcm_data->flags, gps_rtcm_data->len, gps_rtcm_data->data);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_RTCM_DATA, (const char *)gps_rtcm_data, MAVLINK_MSG_ID_GPS_RTCM_DATA_MIN_LEN, MAVLINK_MSG_ID_GPS_RTCM_DATA_LEN, MAVLINK_MSG_ID_GPS_RTCM_DATA_CRC);
#endif
}

#if MAVLINK_MSG_ID_GPS_RTCM_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gps_rtcm_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t flags, uint8_t len, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, flags);
    _mav_put_uint8_t(buf, 1, len);
    _mav_put_uint8_t_array(buf, 2, data, 180);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_RTCM_DATA, buf, MAVLINK_MSG_ID_GPS_RTCM_DATA_MIN_LEN, MAVLINK_MSG_ID_GPS_RTCM_DATA_LEN, MAVLINK_MSG_ID_GPS_RTCM_DATA_CRC);
#else
    mavlink_gps_rtcm_data_t *packet = (mavlink_gps_rtcm_data_t *)msgbuf;
    packet->flags = flags;
    packet->len = len;
    mav_array_memcpy(packet->data, data, sizeof(uint8_t)*180);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_RTCM_DATA, (const char *)packet, MAVLINK_MSG_ID_GPS_RTCM_DATA_MIN_LEN, MAVLINK_MSG_ID_GPS_RTCM_DATA_LEN, MAVLINK_MSG_ID_GPS_RTCM_DATA_CRC);
#endif
}
#endif

#endif

// MESSAGE GPS_RTCM_DATA UNPACKING


/**
 * @brief Get field flags from gps_rtcm_data message
 *
 * @return LSB: 1 means message is fragmented, next 2 bits are the fragment ID, the remaining 5 bits are used for the sequence ID. Messages are only to be flushed to the GPS when the entire message has been reconstructed on the autopilot. The fragment ID specifies which order the fragments should be assembled into a buffer, while the sequence ID is used to detect a mismatch between different buffers. The buffer is considered fully reconstructed when either all 4 fragments are present, or all the fragments before the first fragment with a non full payload is received. This management is used to ensure that normal GPS operation doesn't corrupt RTCM data, and to recover from a unreliable transport delivery order.
 */
static inline uint8_t mavlink_msg_gps_rtcm_data_get_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field len from gps_rtcm_data message
 *
 * @return data length
 */
static inline uint8_t mavlink_msg_gps_rtcm_data_get_len(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field data from gps_rtcm_data message
 *
 * @return RTCM message (may be fragmented)
 */
static inline uint16_t mavlink_msg_gps_rtcm_data_get_data(const mavlink_message_t* msg, uint8_t *data)
{
    return _MAV_RETURN_uint8_t_array(msg, data, 180,  2);
}

/**
 * @brief Decode a gps_rtcm_data message into a struct
 *
 * @param msg The message to decode
 * @param gps_rtcm_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_gps_rtcm_data_decode(const mavlink_message_t* msg, mavlink_gps_rtcm_data_t* gps_rtcm_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gps_rtcm_data->flags = mavlink_msg_gps_rtcm_data_get_flags(msg);
    gps_rtcm_data->len = mavlink_msg_gps_rtcm_data_get_len(msg);
    mavlink_msg_gps_rtcm_data_get_data(msg, gps_rtcm_data->data);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GPS_RTCM_DATA_LEN? msg->len : MAVLINK_MSG_ID_GPS_RTCM_DATA_LEN;
        memset(gps_rtcm_data, 0, MAVLINK_MSG_ID_GPS_RTCM_DATA_LEN);
    memcpy(gps_rtcm_data, _MAV_PAYLOAD(msg), len);
#endif
}
