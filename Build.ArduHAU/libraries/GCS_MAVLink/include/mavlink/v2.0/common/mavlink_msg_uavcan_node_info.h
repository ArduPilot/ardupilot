#pragma once
// MESSAGE UAVCAN_NODE_INFO PACKING

#define MAVLINK_MSG_ID_UAVCAN_NODE_INFO 311

MAVPACKED(
typedef struct __mavlink_uavcan_node_info_t {
 uint64_t time_usec; /*< Timestamp (microseconds since UNIX epoch or microseconds since system boot)*/
 uint32_t uptime_sec; /*< The number of seconds since the start-up of the node.*/
 uint32_t sw_vcs_commit; /*< Version control system (VCS) revision identifier (e.g. git short commit hash). Zero if unknown.*/
 char name[80]; /*< Node name string. For example, "sapog.px4.io".*/
 uint8_t hw_version_major; /*< Hardware major version number.*/
 uint8_t hw_version_minor; /*< Hardware minor version number.*/
 uint8_t hw_unique_id[16]; /*< Hardware unique 128-bit ID.*/
 uint8_t sw_version_major; /*< Software major version number.*/
 uint8_t sw_version_minor; /*< Software minor version number.*/
}) mavlink_uavcan_node_info_t;

#define MAVLINK_MSG_ID_UAVCAN_NODE_INFO_LEN 116
#define MAVLINK_MSG_ID_UAVCAN_NODE_INFO_MIN_LEN 116
#define MAVLINK_MSG_ID_311_LEN 116
#define MAVLINK_MSG_ID_311_MIN_LEN 116

#define MAVLINK_MSG_ID_UAVCAN_NODE_INFO_CRC 95
#define MAVLINK_MSG_ID_311_CRC 95

#define MAVLINK_MSG_UAVCAN_NODE_INFO_FIELD_NAME_LEN 80
#define MAVLINK_MSG_UAVCAN_NODE_INFO_FIELD_HW_UNIQUE_ID_LEN 16

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_UAVCAN_NODE_INFO { \
    311, \
    "UAVCAN_NODE_INFO", \
    9, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_uavcan_node_info_t, time_usec) }, \
         { "uptime_sec", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_uavcan_node_info_t, uptime_sec) }, \
         { "name", NULL, MAVLINK_TYPE_CHAR, 80, 16, offsetof(mavlink_uavcan_node_info_t, name) }, \
         { "hw_version_major", NULL, MAVLINK_TYPE_UINT8_T, 0, 96, offsetof(mavlink_uavcan_node_info_t, hw_version_major) }, \
         { "hw_version_minor", NULL, MAVLINK_TYPE_UINT8_T, 0, 97, offsetof(mavlink_uavcan_node_info_t, hw_version_minor) }, \
         { "hw_unique_id", NULL, MAVLINK_TYPE_UINT8_T, 16, 98, offsetof(mavlink_uavcan_node_info_t, hw_unique_id) }, \
         { "sw_version_major", NULL, MAVLINK_TYPE_UINT8_T, 0, 114, offsetof(mavlink_uavcan_node_info_t, sw_version_major) }, \
         { "sw_version_minor", NULL, MAVLINK_TYPE_UINT8_T, 0, 115, offsetof(mavlink_uavcan_node_info_t, sw_version_minor) }, \
         { "sw_vcs_commit", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_uavcan_node_info_t, sw_vcs_commit) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_UAVCAN_NODE_INFO { \
    "UAVCAN_NODE_INFO", \
    9, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_uavcan_node_info_t, time_usec) }, \
         { "uptime_sec", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_uavcan_node_info_t, uptime_sec) }, \
         { "name", NULL, MAVLINK_TYPE_CHAR, 80, 16, offsetof(mavlink_uavcan_node_info_t, name) }, \
         { "hw_version_major", NULL, MAVLINK_TYPE_UINT8_T, 0, 96, offsetof(mavlink_uavcan_node_info_t, hw_version_major) }, \
         { "hw_version_minor", NULL, MAVLINK_TYPE_UINT8_T, 0, 97, offsetof(mavlink_uavcan_node_info_t, hw_version_minor) }, \
         { "hw_unique_id", NULL, MAVLINK_TYPE_UINT8_T, 16, 98, offsetof(mavlink_uavcan_node_info_t, hw_unique_id) }, \
         { "sw_version_major", NULL, MAVLINK_TYPE_UINT8_T, 0, 114, offsetof(mavlink_uavcan_node_info_t, sw_version_major) }, \
         { "sw_version_minor", NULL, MAVLINK_TYPE_UINT8_T, 0, 115, offsetof(mavlink_uavcan_node_info_t, sw_version_minor) }, \
         { "sw_vcs_commit", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_uavcan_node_info_t, sw_vcs_commit) }, \
         } \
}
#endif

/**
 * @brief Pack a uavcan_node_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param uptime_sec The number of seconds since the start-up of the node.
 * @param name Node name string. For example, "sapog.px4.io".
 * @param hw_version_major Hardware major version number.
 * @param hw_version_minor Hardware minor version number.
 * @param hw_unique_id Hardware unique 128-bit ID.
 * @param sw_version_major Software major version number.
 * @param sw_version_minor Software minor version number.
 * @param sw_vcs_commit Version control system (VCS) revision identifier (e.g. git short commit hash). Zero if unknown.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uavcan_node_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, uint32_t uptime_sec, const char *name, uint8_t hw_version_major, uint8_t hw_version_minor, const uint8_t *hw_unique_id, uint8_t sw_version_major, uint8_t sw_version_minor, uint32_t sw_vcs_commit)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVCAN_NODE_INFO_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, uptime_sec);
    _mav_put_uint32_t(buf, 12, sw_vcs_commit);
    _mav_put_uint8_t(buf, 96, hw_version_major);
    _mav_put_uint8_t(buf, 97, hw_version_minor);
    _mav_put_uint8_t(buf, 114, sw_version_major);
    _mav_put_uint8_t(buf, 115, sw_version_minor);
    _mav_put_char_array(buf, 16, name, 80);
    _mav_put_uint8_t_array(buf, 98, hw_unique_id, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAVCAN_NODE_INFO_LEN);
#else
    mavlink_uavcan_node_info_t packet;
    packet.time_usec = time_usec;
    packet.uptime_sec = uptime_sec;
    packet.sw_vcs_commit = sw_vcs_commit;
    packet.hw_version_major = hw_version_major;
    packet.hw_version_minor = hw_version_minor;
    packet.sw_version_major = sw_version_major;
    packet.sw_version_minor = sw_version_minor;
    mav_array_memcpy(packet.name, name, sizeof(char)*80);
    mav_array_memcpy(packet.hw_unique_id, hw_unique_id, sizeof(uint8_t)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAVCAN_NODE_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAVCAN_NODE_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UAVCAN_NODE_INFO_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_NODE_INFO_LEN, MAVLINK_MSG_ID_UAVCAN_NODE_INFO_CRC);
}

/**
 * @brief Pack a uavcan_node_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param uptime_sec The number of seconds since the start-up of the node.
 * @param name Node name string. For example, "sapog.px4.io".
 * @param hw_version_major Hardware major version number.
 * @param hw_version_minor Hardware minor version number.
 * @param hw_unique_id Hardware unique 128-bit ID.
 * @param sw_version_major Software major version number.
 * @param sw_version_minor Software minor version number.
 * @param sw_vcs_commit Version control system (VCS) revision identifier (e.g. git short commit hash). Zero if unknown.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uavcan_node_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,uint32_t uptime_sec,const char *name,uint8_t hw_version_major,uint8_t hw_version_minor,const uint8_t *hw_unique_id,uint8_t sw_version_major,uint8_t sw_version_minor,uint32_t sw_vcs_commit)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVCAN_NODE_INFO_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, uptime_sec);
    _mav_put_uint32_t(buf, 12, sw_vcs_commit);
    _mav_put_uint8_t(buf, 96, hw_version_major);
    _mav_put_uint8_t(buf, 97, hw_version_minor);
    _mav_put_uint8_t(buf, 114, sw_version_major);
    _mav_put_uint8_t(buf, 115, sw_version_minor);
    _mav_put_char_array(buf, 16, name, 80);
    _mav_put_uint8_t_array(buf, 98, hw_unique_id, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAVCAN_NODE_INFO_LEN);
#else
    mavlink_uavcan_node_info_t packet;
    packet.time_usec = time_usec;
    packet.uptime_sec = uptime_sec;
    packet.sw_vcs_commit = sw_vcs_commit;
    packet.hw_version_major = hw_version_major;
    packet.hw_version_minor = hw_version_minor;
    packet.sw_version_major = sw_version_major;
    packet.sw_version_minor = sw_version_minor;
    mav_array_memcpy(packet.name, name, sizeof(char)*80);
    mav_array_memcpy(packet.hw_unique_id, hw_unique_id, sizeof(uint8_t)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAVCAN_NODE_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAVCAN_NODE_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UAVCAN_NODE_INFO_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_NODE_INFO_LEN, MAVLINK_MSG_ID_UAVCAN_NODE_INFO_CRC);
}

/**
 * @brief Encode a uavcan_node_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param uavcan_node_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uavcan_node_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_uavcan_node_info_t* uavcan_node_info)
{
    return mavlink_msg_uavcan_node_info_pack(system_id, component_id, msg, uavcan_node_info->time_usec, uavcan_node_info->uptime_sec, uavcan_node_info->name, uavcan_node_info->hw_version_major, uavcan_node_info->hw_version_minor, uavcan_node_info->hw_unique_id, uavcan_node_info->sw_version_major, uavcan_node_info->sw_version_minor, uavcan_node_info->sw_vcs_commit);
}

/**
 * @brief Encode a uavcan_node_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param uavcan_node_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uavcan_node_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_uavcan_node_info_t* uavcan_node_info)
{
    return mavlink_msg_uavcan_node_info_pack_chan(system_id, component_id, chan, msg, uavcan_node_info->time_usec, uavcan_node_info->uptime_sec, uavcan_node_info->name, uavcan_node_info->hw_version_major, uavcan_node_info->hw_version_minor, uavcan_node_info->hw_unique_id, uavcan_node_info->sw_version_major, uavcan_node_info->sw_version_minor, uavcan_node_info->sw_vcs_commit);
}

/**
 * @brief Send a uavcan_node_info message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param uptime_sec The number of seconds since the start-up of the node.
 * @param name Node name string. For example, "sapog.px4.io".
 * @param hw_version_major Hardware major version number.
 * @param hw_version_minor Hardware minor version number.
 * @param hw_unique_id Hardware unique 128-bit ID.
 * @param sw_version_major Software major version number.
 * @param sw_version_minor Software minor version number.
 * @param sw_vcs_commit Version control system (VCS) revision identifier (e.g. git short commit hash). Zero if unknown.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_uavcan_node_info_send(mavlink_channel_t chan, uint64_t time_usec, uint32_t uptime_sec, const char *name, uint8_t hw_version_major, uint8_t hw_version_minor, const uint8_t *hw_unique_id, uint8_t sw_version_major, uint8_t sw_version_minor, uint32_t sw_vcs_commit)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVCAN_NODE_INFO_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, uptime_sec);
    _mav_put_uint32_t(buf, 12, sw_vcs_commit);
    _mav_put_uint8_t(buf, 96, hw_version_major);
    _mav_put_uint8_t(buf, 97, hw_version_minor);
    _mav_put_uint8_t(buf, 114, sw_version_major);
    _mav_put_uint8_t(buf, 115, sw_version_minor);
    _mav_put_char_array(buf, 16, name, 80);
    _mav_put_uint8_t_array(buf, 98, hw_unique_id, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVCAN_NODE_INFO, buf, MAVLINK_MSG_ID_UAVCAN_NODE_INFO_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_NODE_INFO_LEN, MAVLINK_MSG_ID_UAVCAN_NODE_INFO_CRC);
#else
    mavlink_uavcan_node_info_t packet;
    packet.time_usec = time_usec;
    packet.uptime_sec = uptime_sec;
    packet.sw_vcs_commit = sw_vcs_commit;
    packet.hw_version_major = hw_version_major;
    packet.hw_version_minor = hw_version_minor;
    packet.sw_version_major = sw_version_major;
    packet.sw_version_minor = sw_version_minor;
    mav_array_memcpy(packet.name, name, sizeof(char)*80);
    mav_array_memcpy(packet.hw_unique_id, hw_unique_id, sizeof(uint8_t)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVCAN_NODE_INFO, (const char *)&packet, MAVLINK_MSG_ID_UAVCAN_NODE_INFO_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_NODE_INFO_LEN, MAVLINK_MSG_ID_UAVCAN_NODE_INFO_CRC);
#endif
}

/**
 * @brief Send a uavcan_node_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_uavcan_node_info_send_struct(mavlink_channel_t chan, const mavlink_uavcan_node_info_t* uavcan_node_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_uavcan_node_info_send(chan, uavcan_node_info->time_usec, uavcan_node_info->uptime_sec, uavcan_node_info->name, uavcan_node_info->hw_version_major, uavcan_node_info->hw_version_minor, uavcan_node_info->hw_unique_id, uavcan_node_info->sw_version_major, uavcan_node_info->sw_version_minor, uavcan_node_info->sw_vcs_commit);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVCAN_NODE_INFO, (const char *)uavcan_node_info, MAVLINK_MSG_ID_UAVCAN_NODE_INFO_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_NODE_INFO_LEN, MAVLINK_MSG_ID_UAVCAN_NODE_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_UAVCAN_NODE_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_uavcan_node_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint32_t uptime_sec, const char *name, uint8_t hw_version_major, uint8_t hw_version_minor, const uint8_t *hw_unique_id, uint8_t sw_version_major, uint8_t sw_version_minor, uint32_t sw_vcs_commit)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, uptime_sec);
    _mav_put_uint32_t(buf, 12, sw_vcs_commit);
    _mav_put_uint8_t(buf, 96, hw_version_major);
    _mav_put_uint8_t(buf, 97, hw_version_minor);
    _mav_put_uint8_t(buf, 114, sw_version_major);
    _mav_put_uint8_t(buf, 115, sw_version_minor);
    _mav_put_char_array(buf, 16, name, 80);
    _mav_put_uint8_t_array(buf, 98, hw_unique_id, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVCAN_NODE_INFO, buf, MAVLINK_MSG_ID_UAVCAN_NODE_INFO_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_NODE_INFO_LEN, MAVLINK_MSG_ID_UAVCAN_NODE_INFO_CRC);
#else
    mavlink_uavcan_node_info_t *packet = (mavlink_uavcan_node_info_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->uptime_sec = uptime_sec;
    packet->sw_vcs_commit = sw_vcs_commit;
    packet->hw_version_major = hw_version_major;
    packet->hw_version_minor = hw_version_minor;
    packet->sw_version_major = sw_version_major;
    packet->sw_version_minor = sw_version_minor;
    mav_array_memcpy(packet->name, name, sizeof(char)*80);
    mav_array_memcpy(packet->hw_unique_id, hw_unique_id, sizeof(uint8_t)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVCAN_NODE_INFO, (const char *)packet, MAVLINK_MSG_ID_UAVCAN_NODE_INFO_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_NODE_INFO_LEN, MAVLINK_MSG_ID_UAVCAN_NODE_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE UAVCAN_NODE_INFO UNPACKING


/**
 * @brief Get field time_usec from uavcan_node_info message
 *
 * @return Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 */
static inline uint64_t mavlink_msg_uavcan_node_info_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field uptime_sec from uavcan_node_info message
 *
 * @return The number of seconds since the start-up of the node.
 */
static inline uint32_t mavlink_msg_uavcan_node_info_get_uptime_sec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field name from uavcan_node_info message
 *
 * @return Node name string. For example, "sapog.px4.io".
 */
static inline uint16_t mavlink_msg_uavcan_node_info_get_name(const mavlink_message_t* msg, char *name)
{
    return _MAV_RETURN_char_array(msg, name, 80,  16);
}

/**
 * @brief Get field hw_version_major from uavcan_node_info message
 *
 * @return Hardware major version number.
 */
static inline uint8_t mavlink_msg_uavcan_node_info_get_hw_version_major(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  96);
}

/**
 * @brief Get field hw_version_minor from uavcan_node_info message
 *
 * @return Hardware minor version number.
 */
static inline uint8_t mavlink_msg_uavcan_node_info_get_hw_version_minor(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  97);
}

/**
 * @brief Get field hw_unique_id from uavcan_node_info message
 *
 * @return Hardware unique 128-bit ID.
 */
static inline uint16_t mavlink_msg_uavcan_node_info_get_hw_unique_id(const mavlink_message_t* msg, uint8_t *hw_unique_id)
{
    return _MAV_RETURN_uint8_t_array(msg, hw_unique_id, 16,  98);
}

/**
 * @brief Get field sw_version_major from uavcan_node_info message
 *
 * @return Software major version number.
 */
static inline uint8_t mavlink_msg_uavcan_node_info_get_sw_version_major(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  114);
}

/**
 * @brief Get field sw_version_minor from uavcan_node_info message
 *
 * @return Software minor version number.
 */
static inline uint8_t mavlink_msg_uavcan_node_info_get_sw_version_minor(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  115);
}

/**
 * @brief Get field sw_vcs_commit from uavcan_node_info message
 *
 * @return Version control system (VCS) revision identifier (e.g. git short commit hash). Zero if unknown.
 */
static inline uint32_t mavlink_msg_uavcan_node_info_get_sw_vcs_commit(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Decode a uavcan_node_info message into a struct
 *
 * @param msg The message to decode
 * @param uavcan_node_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_uavcan_node_info_decode(const mavlink_message_t* msg, mavlink_uavcan_node_info_t* uavcan_node_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    uavcan_node_info->time_usec = mavlink_msg_uavcan_node_info_get_time_usec(msg);
    uavcan_node_info->uptime_sec = mavlink_msg_uavcan_node_info_get_uptime_sec(msg);
    uavcan_node_info->sw_vcs_commit = mavlink_msg_uavcan_node_info_get_sw_vcs_commit(msg);
    mavlink_msg_uavcan_node_info_get_name(msg, uavcan_node_info->name);
    uavcan_node_info->hw_version_major = mavlink_msg_uavcan_node_info_get_hw_version_major(msg);
    uavcan_node_info->hw_version_minor = mavlink_msg_uavcan_node_info_get_hw_version_minor(msg);
    mavlink_msg_uavcan_node_info_get_hw_unique_id(msg, uavcan_node_info->hw_unique_id);
    uavcan_node_info->sw_version_major = mavlink_msg_uavcan_node_info_get_sw_version_major(msg);
    uavcan_node_info->sw_version_minor = mavlink_msg_uavcan_node_info_get_sw_version_minor(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_UAVCAN_NODE_INFO_LEN? msg->len : MAVLINK_MSG_ID_UAVCAN_NODE_INFO_LEN;
        memset(uavcan_node_info, 0, MAVLINK_MSG_ID_UAVCAN_NODE_INFO_LEN);
    memcpy(uavcan_node_info, _MAV_PAYLOAD(msg), len);
#endif
}
