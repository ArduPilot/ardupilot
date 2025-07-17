#pragma once
// MESSAGE UALBERTA_SYS_STATUS PACKING

#define MAVLINK_MSG_ID_UALBERTA_SYS_STATUS 222


typedef struct __mavlink_ualberta_sys_status_t {
 uint8_t mode; /*<  System mode, see UALBERTA_AUTOPILOT_MODE ENUM*/
 uint8_t nav_mode; /*<  Navigation mode, see UALBERTA_NAV_MODE ENUM*/
 uint8_t pilot; /*<  Pilot mode, see UALBERTA_PILOT_MODE*/
} mavlink_ualberta_sys_status_t;

#define MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_LEN 3
#define MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_MIN_LEN 3
#define MAVLINK_MSG_ID_222_LEN 3
#define MAVLINK_MSG_ID_222_MIN_LEN 3

#define MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_CRC 15
#define MAVLINK_MSG_ID_222_CRC 15



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_UALBERTA_SYS_STATUS { \
    222, \
    "UALBERTA_SYS_STATUS", \
    3, \
    {  { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_ualberta_sys_status_t, mode) }, \
         { "nav_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_ualberta_sys_status_t, nav_mode) }, \
         { "pilot", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_ualberta_sys_status_t, pilot) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_UALBERTA_SYS_STATUS { \
    "UALBERTA_SYS_STATUS", \
    3, \
    {  { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_ualberta_sys_status_t, mode) }, \
         { "nav_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_ualberta_sys_status_t, nav_mode) }, \
         { "pilot", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_ualberta_sys_status_t, pilot) }, \
         } \
}
#endif

/**
 * @brief Pack a ualberta_sys_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param mode  System mode, see UALBERTA_AUTOPILOT_MODE ENUM
 * @param nav_mode  Navigation mode, see UALBERTA_NAV_MODE ENUM
 * @param pilot  Pilot mode, see UALBERTA_PILOT_MODE
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ualberta_sys_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t mode, uint8_t nav_mode, uint8_t pilot)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_LEN];
    _mav_put_uint8_t(buf, 0, mode);
    _mav_put_uint8_t(buf, 1, nav_mode);
    _mav_put_uint8_t(buf, 2, pilot);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_LEN);
#else
    mavlink_ualberta_sys_status_t packet;
    packet.mode = mode;
    packet.nav_mode = nav_mode;
    packet.pilot = pilot;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UALBERTA_SYS_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_MIN_LEN, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_LEN, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_CRC);
}

/**
 * @brief Pack a ualberta_sys_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param mode  System mode, see UALBERTA_AUTOPILOT_MODE ENUM
 * @param nav_mode  Navigation mode, see UALBERTA_NAV_MODE ENUM
 * @param pilot  Pilot mode, see UALBERTA_PILOT_MODE
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ualberta_sys_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t mode, uint8_t nav_mode, uint8_t pilot)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_LEN];
    _mav_put_uint8_t(buf, 0, mode);
    _mav_put_uint8_t(buf, 1, nav_mode);
    _mav_put_uint8_t(buf, 2, pilot);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_LEN);
#else
    mavlink_ualberta_sys_status_t packet;
    packet.mode = mode;
    packet.nav_mode = nav_mode;
    packet.pilot = pilot;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UALBERTA_SYS_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_MIN_LEN, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_LEN, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_MIN_LEN, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_LEN);
#endif
}

/**
 * @brief Pack a ualberta_sys_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mode  System mode, see UALBERTA_AUTOPILOT_MODE ENUM
 * @param nav_mode  Navigation mode, see UALBERTA_NAV_MODE ENUM
 * @param pilot  Pilot mode, see UALBERTA_PILOT_MODE
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ualberta_sys_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t mode,uint8_t nav_mode,uint8_t pilot)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_LEN];
    _mav_put_uint8_t(buf, 0, mode);
    _mav_put_uint8_t(buf, 1, nav_mode);
    _mav_put_uint8_t(buf, 2, pilot);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_LEN);
#else
    mavlink_ualberta_sys_status_t packet;
    packet.mode = mode;
    packet.nav_mode = nav_mode;
    packet.pilot = pilot;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UALBERTA_SYS_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_MIN_LEN, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_LEN, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_CRC);
}

/**
 * @brief Encode a ualberta_sys_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ualberta_sys_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ualberta_sys_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ualberta_sys_status_t* ualberta_sys_status)
{
    return mavlink_msg_ualberta_sys_status_pack(system_id, component_id, msg, ualberta_sys_status->mode, ualberta_sys_status->nav_mode, ualberta_sys_status->pilot);
}

/**
 * @brief Encode a ualberta_sys_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ualberta_sys_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ualberta_sys_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ualberta_sys_status_t* ualberta_sys_status)
{
    return mavlink_msg_ualberta_sys_status_pack_chan(system_id, component_id, chan, msg, ualberta_sys_status->mode, ualberta_sys_status->nav_mode, ualberta_sys_status->pilot);
}

/**
 * @brief Encode a ualberta_sys_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param ualberta_sys_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ualberta_sys_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_ualberta_sys_status_t* ualberta_sys_status)
{
    return mavlink_msg_ualberta_sys_status_pack_status(system_id, component_id, _status, msg,  ualberta_sys_status->mode, ualberta_sys_status->nav_mode, ualberta_sys_status->pilot);
}

/**
 * @brief Send a ualberta_sys_status message
 * @param chan MAVLink channel to send the message
 *
 * @param mode  System mode, see UALBERTA_AUTOPILOT_MODE ENUM
 * @param nav_mode  Navigation mode, see UALBERTA_NAV_MODE ENUM
 * @param pilot  Pilot mode, see UALBERTA_PILOT_MODE
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ualberta_sys_status_send(mavlink_channel_t chan, uint8_t mode, uint8_t nav_mode, uint8_t pilot)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_LEN];
    _mav_put_uint8_t(buf, 0, mode);
    _mav_put_uint8_t(buf, 1, nav_mode);
    _mav_put_uint8_t(buf, 2, pilot);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS, buf, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_MIN_LEN, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_LEN, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_CRC);
#else
    mavlink_ualberta_sys_status_t packet;
    packet.mode = mode;
    packet.nav_mode = nav_mode;
    packet.pilot = pilot;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS, (const char *)&packet, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_MIN_LEN, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_LEN, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_CRC);
#endif
}

/**
 * @brief Send a ualberta_sys_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ualberta_sys_status_send_struct(mavlink_channel_t chan, const mavlink_ualberta_sys_status_t* ualberta_sys_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ualberta_sys_status_send(chan, ualberta_sys_status->mode, ualberta_sys_status->nav_mode, ualberta_sys_status->pilot);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS, (const char *)ualberta_sys_status, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_MIN_LEN, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_LEN, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ualberta_sys_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t mode, uint8_t nav_mode, uint8_t pilot)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, mode);
    _mav_put_uint8_t(buf, 1, nav_mode);
    _mav_put_uint8_t(buf, 2, pilot);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS, buf, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_MIN_LEN, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_LEN, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_CRC);
#else
    mavlink_ualberta_sys_status_t *packet = (mavlink_ualberta_sys_status_t *)msgbuf;
    packet->mode = mode;
    packet->nav_mode = nav_mode;
    packet->pilot = pilot;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS, (const char *)packet, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_MIN_LEN, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_LEN, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE UALBERTA_SYS_STATUS UNPACKING


/**
 * @brief Get field mode from ualberta_sys_status message
 *
 * @return  System mode, see UALBERTA_AUTOPILOT_MODE ENUM
 */
static inline uint8_t mavlink_msg_ualberta_sys_status_get_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field nav_mode from ualberta_sys_status message
 *
 * @return  Navigation mode, see UALBERTA_NAV_MODE ENUM
 */
static inline uint8_t mavlink_msg_ualberta_sys_status_get_nav_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field pilot from ualberta_sys_status message
 *
 * @return  Pilot mode, see UALBERTA_PILOT_MODE
 */
static inline uint8_t mavlink_msg_ualberta_sys_status_get_pilot(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a ualberta_sys_status message into a struct
 *
 * @param msg The message to decode
 * @param ualberta_sys_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_ualberta_sys_status_decode(const mavlink_message_t* msg, mavlink_ualberta_sys_status_t* ualberta_sys_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    ualberta_sys_status->mode = mavlink_msg_ualberta_sys_status_get_mode(msg);
    ualberta_sys_status->nav_mode = mavlink_msg_ualberta_sys_status_get_nav_mode(msg);
    ualberta_sys_status->pilot = mavlink_msg_ualberta_sys_status_get_pilot(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_LEN? msg->len : MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_LEN;
        memset(ualberta_sys_status, 0, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_LEN);
    memcpy(ualberta_sys_status, _MAV_PAYLOAD(msg), len);
#endif
}
