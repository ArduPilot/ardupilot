#pragma once
// MESSAGE SENSORPOD_STATUS PACKING

#define MAVLINK_MSG_ID_SENSORPOD_STATUS 8012


typedef struct __mavlink_sensorpod_status_t {
 uint64_t timestamp; /*< [ms] Timestamp in linuxtime (since 1.1.1970)*/
 uint16_t free_space; /*<  Free space available in recordings directory in [Gb] * 1e2*/
 uint8_t visensor_rate_1; /*<  Rate of ROS topic 1*/
 uint8_t visensor_rate_2; /*<  Rate of ROS topic 2*/
 uint8_t visensor_rate_3; /*<  Rate of ROS topic 3*/
 uint8_t visensor_rate_4; /*<  Rate of ROS topic 4*/
 uint8_t recording_nodes_count; /*<  Number of recording nodes*/
 uint8_t cpu_temp; /*< [degC] Temperature of sensorpod CPU in*/
} mavlink_sensorpod_status_t;

#define MAVLINK_MSG_ID_SENSORPOD_STATUS_LEN 16
#define MAVLINK_MSG_ID_SENSORPOD_STATUS_MIN_LEN 16
#define MAVLINK_MSG_ID_8012_LEN 16
#define MAVLINK_MSG_ID_8012_MIN_LEN 16

#define MAVLINK_MSG_ID_SENSORPOD_STATUS_CRC 54
#define MAVLINK_MSG_ID_8012_CRC 54



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SENSORPOD_STATUS { \
    8012, \
    "SENSORPOD_STATUS", \
    8, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sensorpod_status_t, timestamp) }, \
         { "visensor_rate_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_sensorpod_status_t, visensor_rate_1) }, \
         { "visensor_rate_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_sensorpod_status_t, visensor_rate_2) }, \
         { "visensor_rate_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_sensorpod_status_t, visensor_rate_3) }, \
         { "visensor_rate_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_sensorpod_status_t, visensor_rate_4) }, \
         { "recording_nodes_count", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_sensorpod_status_t, recording_nodes_count) }, \
         { "cpu_temp", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_sensorpod_status_t, cpu_temp) }, \
         { "free_space", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_sensorpod_status_t, free_space) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SENSORPOD_STATUS { \
    "SENSORPOD_STATUS", \
    8, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sensorpod_status_t, timestamp) }, \
         { "visensor_rate_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_sensorpod_status_t, visensor_rate_1) }, \
         { "visensor_rate_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_sensorpod_status_t, visensor_rate_2) }, \
         { "visensor_rate_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_sensorpod_status_t, visensor_rate_3) }, \
         { "visensor_rate_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_sensorpod_status_t, visensor_rate_4) }, \
         { "recording_nodes_count", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_sensorpod_status_t, recording_nodes_count) }, \
         { "cpu_temp", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_sensorpod_status_t, cpu_temp) }, \
         { "free_space", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_sensorpod_status_t, free_space) }, \
         } \
}
#endif

/**
 * @brief Pack a sensorpod_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [ms] Timestamp in linuxtime (since 1.1.1970)
 * @param visensor_rate_1  Rate of ROS topic 1
 * @param visensor_rate_2  Rate of ROS topic 2
 * @param visensor_rate_3  Rate of ROS topic 3
 * @param visensor_rate_4  Rate of ROS topic 4
 * @param recording_nodes_count  Number of recording nodes
 * @param cpu_temp [degC] Temperature of sensorpod CPU in
 * @param free_space  Free space available in recordings directory in [Gb] * 1e2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sensorpod_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t visensor_rate_1, uint8_t visensor_rate_2, uint8_t visensor_rate_3, uint8_t visensor_rate_4, uint8_t recording_nodes_count, uint8_t cpu_temp, uint16_t free_space)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENSORPOD_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint16_t(buf, 8, free_space);
    _mav_put_uint8_t(buf, 10, visensor_rate_1);
    _mav_put_uint8_t(buf, 11, visensor_rate_2);
    _mav_put_uint8_t(buf, 12, visensor_rate_3);
    _mav_put_uint8_t(buf, 13, visensor_rate_4);
    _mav_put_uint8_t(buf, 14, recording_nodes_count);
    _mav_put_uint8_t(buf, 15, cpu_temp);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENSORPOD_STATUS_LEN);
#else
    mavlink_sensorpod_status_t packet;
    packet.timestamp = timestamp;
    packet.free_space = free_space;
    packet.visensor_rate_1 = visensor_rate_1;
    packet.visensor_rate_2 = visensor_rate_2;
    packet.visensor_rate_3 = visensor_rate_3;
    packet.visensor_rate_4 = visensor_rate_4;
    packet.recording_nodes_count = recording_nodes_count;
    packet.cpu_temp = cpu_temp;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENSORPOD_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENSORPOD_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SENSORPOD_STATUS_MIN_LEN, MAVLINK_MSG_ID_SENSORPOD_STATUS_LEN, MAVLINK_MSG_ID_SENSORPOD_STATUS_CRC);
}

/**
 * @brief Pack a sensorpod_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [ms] Timestamp in linuxtime (since 1.1.1970)
 * @param visensor_rate_1  Rate of ROS topic 1
 * @param visensor_rate_2  Rate of ROS topic 2
 * @param visensor_rate_3  Rate of ROS topic 3
 * @param visensor_rate_4  Rate of ROS topic 4
 * @param recording_nodes_count  Number of recording nodes
 * @param cpu_temp [degC] Temperature of sensorpod CPU in
 * @param free_space  Free space available in recordings directory in [Gb] * 1e2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sensorpod_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t visensor_rate_1, uint8_t visensor_rate_2, uint8_t visensor_rate_3, uint8_t visensor_rate_4, uint8_t recording_nodes_count, uint8_t cpu_temp, uint16_t free_space)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENSORPOD_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint16_t(buf, 8, free_space);
    _mav_put_uint8_t(buf, 10, visensor_rate_1);
    _mav_put_uint8_t(buf, 11, visensor_rate_2);
    _mav_put_uint8_t(buf, 12, visensor_rate_3);
    _mav_put_uint8_t(buf, 13, visensor_rate_4);
    _mav_put_uint8_t(buf, 14, recording_nodes_count);
    _mav_put_uint8_t(buf, 15, cpu_temp);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENSORPOD_STATUS_LEN);
#else
    mavlink_sensorpod_status_t packet;
    packet.timestamp = timestamp;
    packet.free_space = free_space;
    packet.visensor_rate_1 = visensor_rate_1;
    packet.visensor_rate_2 = visensor_rate_2;
    packet.visensor_rate_3 = visensor_rate_3;
    packet.visensor_rate_4 = visensor_rate_4;
    packet.recording_nodes_count = recording_nodes_count;
    packet.cpu_temp = cpu_temp;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENSORPOD_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENSORPOD_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SENSORPOD_STATUS_MIN_LEN, MAVLINK_MSG_ID_SENSORPOD_STATUS_LEN, MAVLINK_MSG_ID_SENSORPOD_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SENSORPOD_STATUS_MIN_LEN, MAVLINK_MSG_ID_SENSORPOD_STATUS_LEN);
#endif
}

/**
 * @brief Pack a sensorpod_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [ms] Timestamp in linuxtime (since 1.1.1970)
 * @param visensor_rate_1  Rate of ROS topic 1
 * @param visensor_rate_2  Rate of ROS topic 2
 * @param visensor_rate_3  Rate of ROS topic 3
 * @param visensor_rate_4  Rate of ROS topic 4
 * @param recording_nodes_count  Number of recording nodes
 * @param cpu_temp [degC] Temperature of sensorpod CPU in
 * @param free_space  Free space available in recordings directory in [Gb] * 1e2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sensorpod_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint8_t visensor_rate_1,uint8_t visensor_rate_2,uint8_t visensor_rate_3,uint8_t visensor_rate_4,uint8_t recording_nodes_count,uint8_t cpu_temp,uint16_t free_space)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENSORPOD_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint16_t(buf, 8, free_space);
    _mav_put_uint8_t(buf, 10, visensor_rate_1);
    _mav_put_uint8_t(buf, 11, visensor_rate_2);
    _mav_put_uint8_t(buf, 12, visensor_rate_3);
    _mav_put_uint8_t(buf, 13, visensor_rate_4);
    _mav_put_uint8_t(buf, 14, recording_nodes_count);
    _mav_put_uint8_t(buf, 15, cpu_temp);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENSORPOD_STATUS_LEN);
#else
    mavlink_sensorpod_status_t packet;
    packet.timestamp = timestamp;
    packet.free_space = free_space;
    packet.visensor_rate_1 = visensor_rate_1;
    packet.visensor_rate_2 = visensor_rate_2;
    packet.visensor_rate_3 = visensor_rate_3;
    packet.visensor_rate_4 = visensor_rate_4;
    packet.recording_nodes_count = recording_nodes_count;
    packet.cpu_temp = cpu_temp;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENSORPOD_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENSORPOD_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SENSORPOD_STATUS_MIN_LEN, MAVLINK_MSG_ID_SENSORPOD_STATUS_LEN, MAVLINK_MSG_ID_SENSORPOD_STATUS_CRC);
}

/**
 * @brief Encode a sensorpod_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sensorpod_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sensorpod_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sensorpod_status_t* sensorpod_status)
{
    return mavlink_msg_sensorpod_status_pack(system_id, component_id, msg, sensorpod_status->timestamp, sensorpod_status->visensor_rate_1, sensorpod_status->visensor_rate_2, sensorpod_status->visensor_rate_3, sensorpod_status->visensor_rate_4, sensorpod_status->recording_nodes_count, sensorpod_status->cpu_temp, sensorpod_status->free_space);
}

/**
 * @brief Encode a sensorpod_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sensorpod_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sensorpod_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sensorpod_status_t* sensorpod_status)
{
    return mavlink_msg_sensorpod_status_pack_chan(system_id, component_id, chan, msg, sensorpod_status->timestamp, sensorpod_status->visensor_rate_1, sensorpod_status->visensor_rate_2, sensorpod_status->visensor_rate_3, sensorpod_status->visensor_rate_4, sensorpod_status->recording_nodes_count, sensorpod_status->cpu_temp, sensorpod_status->free_space);
}

/**
 * @brief Encode a sensorpod_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param sensorpod_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sensorpod_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_sensorpod_status_t* sensorpod_status)
{
    return mavlink_msg_sensorpod_status_pack_status(system_id, component_id, _status, msg,  sensorpod_status->timestamp, sensorpod_status->visensor_rate_1, sensorpod_status->visensor_rate_2, sensorpod_status->visensor_rate_3, sensorpod_status->visensor_rate_4, sensorpod_status->recording_nodes_count, sensorpod_status->cpu_temp, sensorpod_status->free_space);
}

/**
 * @brief Send a sensorpod_status message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [ms] Timestamp in linuxtime (since 1.1.1970)
 * @param visensor_rate_1  Rate of ROS topic 1
 * @param visensor_rate_2  Rate of ROS topic 2
 * @param visensor_rate_3  Rate of ROS topic 3
 * @param visensor_rate_4  Rate of ROS topic 4
 * @param recording_nodes_count  Number of recording nodes
 * @param cpu_temp [degC] Temperature of sensorpod CPU in
 * @param free_space  Free space available in recordings directory in [Gb] * 1e2
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sensorpod_status_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t visensor_rate_1, uint8_t visensor_rate_2, uint8_t visensor_rate_3, uint8_t visensor_rate_4, uint8_t recording_nodes_count, uint8_t cpu_temp, uint16_t free_space)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENSORPOD_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint16_t(buf, 8, free_space);
    _mav_put_uint8_t(buf, 10, visensor_rate_1);
    _mav_put_uint8_t(buf, 11, visensor_rate_2);
    _mav_put_uint8_t(buf, 12, visensor_rate_3);
    _mav_put_uint8_t(buf, 13, visensor_rate_4);
    _mav_put_uint8_t(buf, 14, recording_nodes_count);
    _mav_put_uint8_t(buf, 15, cpu_temp);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSORPOD_STATUS, buf, MAVLINK_MSG_ID_SENSORPOD_STATUS_MIN_LEN, MAVLINK_MSG_ID_SENSORPOD_STATUS_LEN, MAVLINK_MSG_ID_SENSORPOD_STATUS_CRC);
#else
    mavlink_sensorpod_status_t packet;
    packet.timestamp = timestamp;
    packet.free_space = free_space;
    packet.visensor_rate_1 = visensor_rate_1;
    packet.visensor_rate_2 = visensor_rate_2;
    packet.visensor_rate_3 = visensor_rate_3;
    packet.visensor_rate_4 = visensor_rate_4;
    packet.recording_nodes_count = recording_nodes_count;
    packet.cpu_temp = cpu_temp;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSORPOD_STATUS, (const char *)&packet, MAVLINK_MSG_ID_SENSORPOD_STATUS_MIN_LEN, MAVLINK_MSG_ID_SENSORPOD_STATUS_LEN, MAVLINK_MSG_ID_SENSORPOD_STATUS_CRC);
#endif
}

/**
 * @brief Send a sensorpod_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sensorpod_status_send_struct(mavlink_channel_t chan, const mavlink_sensorpod_status_t* sensorpod_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sensorpod_status_send(chan, sensorpod_status->timestamp, sensorpod_status->visensor_rate_1, sensorpod_status->visensor_rate_2, sensorpod_status->visensor_rate_3, sensorpod_status->visensor_rate_4, sensorpod_status->recording_nodes_count, sensorpod_status->cpu_temp, sensorpod_status->free_space);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSORPOD_STATUS, (const char *)sensorpod_status, MAVLINK_MSG_ID_SENSORPOD_STATUS_MIN_LEN, MAVLINK_MSG_ID_SENSORPOD_STATUS_LEN, MAVLINK_MSG_ID_SENSORPOD_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_SENSORPOD_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sensorpod_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t visensor_rate_1, uint8_t visensor_rate_2, uint8_t visensor_rate_3, uint8_t visensor_rate_4, uint8_t recording_nodes_count, uint8_t cpu_temp, uint16_t free_space)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint16_t(buf, 8, free_space);
    _mav_put_uint8_t(buf, 10, visensor_rate_1);
    _mav_put_uint8_t(buf, 11, visensor_rate_2);
    _mav_put_uint8_t(buf, 12, visensor_rate_3);
    _mav_put_uint8_t(buf, 13, visensor_rate_4);
    _mav_put_uint8_t(buf, 14, recording_nodes_count);
    _mav_put_uint8_t(buf, 15, cpu_temp);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSORPOD_STATUS, buf, MAVLINK_MSG_ID_SENSORPOD_STATUS_MIN_LEN, MAVLINK_MSG_ID_SENSORPOD_STATUS_LEN, MAVLINK_MSG_ID_SENSORPOD_STATUS_CRC);
#else
    mavlink_sensorpod_status_t *packet = (mavlink_sensorpod_status_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->free_space = free_space;
    packet->visensor_rate_1 = visensor_rate_1;
    packet->visensor_rate_2 = visensor_rate_2;
    packet->visensor_rate_3 = visensor_rate_3;
    packet->visensor_rate_4 = visensor_rate_4;
    packet->recording_nodes_count = recording_nodes_count;
    packet->cpu_temp = cpu_temp;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSORPOD_STATUS, (const char *)packet, MAVLINK_MSG_ID_SENSORPOD_STATUS_MIN_LEN, MAVLINK_MSG_ID_SENSORPOD_STATUS_LEN, MAVLINK_MSG_ID_SENSORPOD_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE SENSORPOD_STATUS UNPACKING


/**
 * @brief Get field timestamp from sensorpod_status message
 *
 * @return [ms] Timestamp in linuxtime (since 1.1.1970)
 */
static inline uint64_t mavlink_msg_sensorpod_status_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field visensor_rate_1 from sensorpod_status message
 *
 * @return  Rate of ROS topic 1
 */
static inline uint8_t mavlink_msg_sensorpod_status_get_visensor_rate_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field visensor_rate_2 from sensorpod_status message
 *
 * @return  Rate of ROS topic 2
 */
static inline uint8_t mavlink_msg_sensorpod_status_get_visensor_rate_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field visensor_rate_3 from sensorpod_status message
 *
 * @return  Rate of ROS topic 3
 */
static inline uint8_t mavlink_msg_sensorpod_status_get_visensor_rate_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field visensor_rate_4 from sensorpod_status message
 *
 * @return  Rate of ROS topic 4
 */
static inline uint8_t mavlink_msg_sensorpod_status_get_visensor_rate_4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field recording_nodes_count from sensorpod_status message
 *
 * @return  Number of recording nodes
 */
static inline uint8_t mavlink_msg_sensorpod_status_get_recording_nodes_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field cpu_temp from sensorpod_status message
 *
 * @return [degC] Temperature of sensorpod CPU in
 */
static inline uint8_t mavlink_msg_sensorpod_status_get_cpu_temp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  15);
}

/**
 * @brief Get field free_space from sensorpod_status message
 *
 * @return  Free space available in recordings directory in [Gb] * 1e2
 */
static inline uint16_t mavlink_msg_sensorpod_status_get_free_space(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Decode a sensorpod_status message into a struct
 *
 * @param msg The message to decode
 * @param sensorpod_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_sensorpod_status_decode(const mavlink_message_t* msg, mavlink_sensorpod_status_t* sensorpod_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sensorpod_status->timestamp = mavlink_msg_sensorpod_status_get_timestamp(msg);
    sensorpod_status->free_space = mavlink_msg_sensorpod_status_get_free_space(msg);
    sensorpod_status->visensor_rate_1 = mavlink_msg_sensorpod_status_get_visensor_rate_1(msg);
    sensorpod_status->visensor_rate_2 = mavlink_msg_sensorpod_status_get_visensor_rate_2(msg);
    sensorpod_status->visensor_rate_3 = mavlink_msg_sensorpod_status_get_visensor_rate_3(msg);
    sensorpod_status->visensor_rate_4 = mavlink_msg_sensorpod_status_get_visensor_rate_4(msg);
    sensorpod_status->recording_nodes_count = mavlink_msg_sensorpod_status_get_recording_nodes_count(msg);
    sensorpod_status->cpu_temp = mavlink_msg_sensorpod_status_get_cpu_temp(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SENSORPOD_STATUS_LEN? msg->len : MAVLINK_MSG_ID_SENSORPOD_STATUS_LEN;
        memset(sensorpod_status, 0, MAVLINK_MSG_ID_SENSORPOD_STATUS_LEN);
    memcpy(sensorpod_status, _MAV_PAYLOAD(msg), len);
#endif
}
