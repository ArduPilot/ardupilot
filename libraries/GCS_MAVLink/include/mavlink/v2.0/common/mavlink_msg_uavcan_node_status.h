#pragma once
// MESSAGE UAVCAN_NODE_STATUS PACKING

#define MAVLINK_MSG_ID_UAVCAN_NODE_STATUS 310


typedef struct __mavlink_uavcan_node_status_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 uint32_t uptime_sec; /*< [s] Time since the start-up of the node.*/
 uint16_t vendor_specific_status_code; /*<  Vendor-specific status information.*/
 uint8_t health; /*<  Generalized node health status.*/
 uint8_t mode; /*<  Generalized operating mode.*/
 uint8_t sub_mode; /*<  Not used currently.*/
} mavlink_uavcan_node_status_t;

#define MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_LEN 17
#define MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_MIN_LEN 17
#define MAVLINK_MSG_ID_310_LEN 17
#define MAVLINK_MSG_ID_310_MIN_LEN 17

#define MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_CRC 28
#define MAVLINK_MSG_ID_310_CRC 28



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_UAVCAN_NODE_STATUS { \
    310, \
    "UAVCAN_NODE_STATUS", \
    6, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_uavcan_node_status_t, time_usec) }, \
         { "uptime_sec", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_uavcan_node_status_t, uptime_sec) }, \
         { "health", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_uavcan_node_status_t, health) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_uavcan_node_status_t, mode) }, \
         { "sub_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_uavcan_node_status_t, sub_mode) }, \
         { "vendor_specific_status_code", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_uavcan_node_status_t, vendor_specific_status_code) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_UAVCAN_NODE_STATUS { \
    "UAVCAN_NODE_STATUS", \
    6, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_uavcan_node_status_t, time_usec) }, \
         { "uptime_sec", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_uavcan_node_status_t, uptime_sec) }, \
         { "health", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_uavcan_node_status_t, health) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_uavcan_node_status_t, mode) }, \
         { "sub_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_uavcan_node_status_t, sub_mode) }, \
         { "vendor_specific_status_code", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_uavcan_node_status_t, vendor_specific_status_code) }, \
         } \
}
#endif

/**
 * @brief Pack a uavcan_node_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param uptime_sec [s] Time since the start-up of the node.
 * @param health  Generalized node health status.
 * @param mode  Generalized operating mode.
 * @param sub_mode  Not used currently.
 * @param vendor_specific_status_code  Vendor-specific status information.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uavcan_node_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, uint32_t uptime_sec, uint8_t health, uint8_t mode, uint8_t sub_mode, uint16_t vendor_specific_status_code)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, uptime_sec);
    _mav_put_uint16_t(buf, 12, vendor_specific_status_code);
    _mav_put_uint8_t(buf, 14, health);
    _mav_put_uint8_t(buf, 15, mode);
    _mav_put_uint8_t(buf, 16, sub_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_LEN);
#else
    mavlink_uavcan_node_status_t packet;
    packet.time_usec = time_usec;
    packet.uptime_sec = uptime_sec;
    packet.vendor_specific_status_code = vendor_specific_status_code;
    packet.health = health;
    packet.mode = mode;
    packet.sub_mode = sub_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAVCAN_NODE_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_LEN, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_CRC);
}

/**
 * @brief Pack a uavcan_node_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param uptime_sec [s] Time since the start-up of the node.
 * @param health  Generalized node health status.
 * @param mode  Generalized operating mode.
 * @param sub_mode  Not used currently.
 * @param vendor_specific_status_code  Vendor-specific status information.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uavcan_node_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t time_usec, uint32_t uptime_sec, uint8_t health, uint8_t mode, uint8_t sub_mode, uint16_t vendor_specific_status_code)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, uptime_sec);
    _mav_put_uint16_t(buf, 12, vendor_specific_status_code);
    _mav_put_uint8_t(buf, 14, health);
    _mav_put_uint8_t(buf, 15, mode);
    _mav_put_uint8_t(buf, 16, sub_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_LEN);
#else
    mavlink_uavcan_node_status_t packet;
    packet.time_usec = time_usec;
    packet.uptime_sec = uptime_sec;
    packet.vendor_specific_status_code = vendor_specific_status_code;
    packet.health = health;
    packet.mode = mode;
    packet.sub_mode = sub_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAVCAN_NODE_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_LEN, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_LEN);
#endif
}

/**
 * @brief Pack a uavcan_node_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param uptime_sec [s] Time since the start-up of the node.
 * @param health  Generalized node health status.
 * @param mode  Generalized operating mode.
 * @param sub_mode  Not used currently.
 * @param vendor_specific_status_code  Vendor-specific status information.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uavcan_node_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,uint32_t uptime_sec,uint8_t health,uint8_t mode,uint8_t sub_mode,uint16_t vendor_specific_status_code)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, uptime_sec);
    _mav_put_uint16_t(buf, 12, vendor_specific_status_code);
    _mav_put_uint8_t(buf, 14, health);
    _mav_put_uint8_t(buf, 15, mode);
    _mav_put_uint8_t(buf, 16, sub_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_LEN);
#else
    mavlink_uavcan_node_status_t packet;
    packet.time_usec = time_usec;
    packet.uptime_sec = uptime_sec;
    packet.vendor_specific_status_code = vendor_specific_status_code;
    packet.health = health;
    packet.mode = mode;
    packet.sub_mode = sub_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAVCAN_NODE_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_LEN, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_CRC);
}

/**
 * @brief Encode a uavcan_node_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param uavcan_node_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uavcan_node_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_uavcan_node_status_t* uavcan_node_status)
{
    return mavlink_msg_uavcan_node_status_pack(system_id, component_id, msg, uavcan_node_status->time_usec, uavcan_node_status->uptime_sec, uavcan_node_status->health, uavcan_node_status->mode, uavcan_node_status->sub_mode, uavcan_node_status->vendor_specific_status_code);
}

/**
 * @brief Encode a uavcan_node_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param uavcan_node_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uavcan_node_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_uavcan_node_status_t* uavcan_node_status)
{
    return mavlink_msg_uavcan_node_status_pack_chan(system_id, component_id, chan, msg, uavcan_node_status->time_usec, uavcan_node_status->uptime_sec, uavcan_node_status->health, uavcan_node_status->mode, uavcan_node_status->sub_mode, uavcan_node_status->vendor_specific_status_code);
}

/**
 * @brief Encode a uavcan_node_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param uavcan_node_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uavcan_node_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_uavcan_node_status_t* uavcan_node_status)
{
    return mavlink_msg_uavcan_node_status_pack_status(system_id, component_id, _status, msg,  uavcan_node_status->time_usec, uavcan_node_status->uptime_sec, uavcan_node_status->health, uavcan_node_status->mode, uavcan_node_status->sub_mode, uavcan_node_status->vendor_specific_status_code);
}

/**
 * @brief Send a uavcan_node_status message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param uptime_sec [s] Time since the start-up of the node.
 * @param health  Generalized node health status.
 * @param mode  Generalized operating mode.
 * @param sub_mode  Not used currently.
 * @param vendor_specific_status_code  Vendor-specific status information.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_uavcan_node_status_send(mavlink_channel_t chan, uint64_t time_usec, uint32_t uptime_sec, uint8_t health, uint8_t mode, uint8_t sub_mode, uint16_t vendor_specific_status_code)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, uptime_sec);
    _mav_put_uint16_t(buf, 12, vendor_specific_status_code);
    _mav_put_uint8_t(buf, 14, health);
    _mav_put_uint8_t(buf, 15, mode);
    _mav_put_uint8_t(buf, 16, sub_mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS, buf, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_LEN, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_CRC);
#else
    mavlink_uavcan_node_status_t packet;
    packet.time_usec = time_usec;
    packet.uptime_sec = uptime_sec;
    packet.vendor_specific_status_code = vendor_specific_status_code;
    packet.health = health;
    packet.mode = mode;
    packet.sub_mode = sub_mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS, (const char *)&packet, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_LEN, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_CRC);
#endif
}

/**
 * @brief Send a uavcan_node_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_uavcan_node_status_send_struct(mavlink_channel_t chan, const mavlink_uavcan_node_status_t* uavcan_node_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_uavcan_node_status_send(chan, uavcan_node_status->time_usec, uavcan_node_status->uptime_sec, uavcan_node_status->health, uavcan_node_status->mode, uavcan_node_status->sub_mode, uavcan_node_status->vendor_specific_status_code);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS, (const char *)uavcan_node_status, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_LEN, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_uavcan_node_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint32_t uptime_sec, uint8_t health, uint8_t mode, uint8_t sub_mode, uint16_t vendor_specific_status_code)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, uptime_sec);
    _mav_put_uint16_t(buf, 12, vendor_specific_status_code);
    _mav_put_uint8_t(buf, 14, health);
    _mav_put_uint8_t(buf, 15, mode);
    _mav_put_uint8_t(buf, 16, sub_mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS, buf, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_LEN, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_CRC);
#else
    mavlink_uavcan_node_status_t *packet = (mavlink_uavcan_node_status_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->uptime_sec = uptime_sec;
    packet->vendor_specific_status_code = vendor_specific_status_code;
    packet->health = health;
    packet->mode = mode;
    packet->sub_mode = sub_mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS, (const char *)packet, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_LEN, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE UAVCAN_NODE_STATUS UNPACKING


/**
 * @brief Get field time_usec from uavcan_node_status message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
static inline uint64_t mavlink_msg_uavcan_node_status_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field uptime_sec from uavcan_node_status message
 *
 * @return [s] Time since the start-up of the node.
 */
static inline uint32_t mavlink_msg_uavcan_node_status_get_uptime_sec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field health from uavcan_node_status message
 *
 * @return  Generalized node health status.
 */
static inline uint8_t mavlink_msg_uavcan_node_status_get_health(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field mode from uavcan_node_status message
 *
 * @return  Generalized operating mode.
 */
static inline uint8_t mavlink_msg_uavcan_node_status_get_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  15);
}

/**
 * @brief Get field sub_mode from uavcan_node_status message
 *
 * @return  Not used currently.
 */
static inline uint8_t mavlink_msg_uavcan_node_status_get_sub_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field vendor_specific_status_code from uavcan_node_status message
 *
 * @return  Vendor-specific status information.
 */
static inline uint16_t mavlink_msg_uavcan_node_status_get_vendor_specific_status_code(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Decode a uavcan_node_status message into a struct
 *
 * @param msg The message to decode
 * @param uavcan_node_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_uavcan_node_status_decode(const mavlink_message_t* msg, mavlink_uavcan_node_status_t* uavcan_node_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    uavcan_node_status->time_usec = mavlink_msg_uavcan_node_status_get_time_usec(msg);
    uavcan_node_status->uptime_sec = mavlink_msg_uavcan_node_status_get_uptime_sec(msg);
    uavcan_node_status->vendor_specific_status_code = mavlink_msg_uavcan_node_status_get_vendor_specific_status_code(msg);
    uavcan_node_status->health = mavlink_msg_uavcan_node_status_get_health(msg);
    uavcan_node_status->mode = mavlink_msg_uavcan_node_status_get_mode(msg);
    uavcan_node_status->sub_mode = mavlink_msg_uavcan_node_status_get_sub_mode(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_LEN? msg->len : MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_LEN;
        memset(uavcan_node_status, 0, MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_LEN);
    memcpy(uavcan_node_status, _MAV_PAYLOAD(msg), len);
#endif
}
