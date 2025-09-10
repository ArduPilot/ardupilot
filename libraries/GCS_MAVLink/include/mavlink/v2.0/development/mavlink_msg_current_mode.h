#pragma once
// MESSAGE CURRENT_MODE PACKING

#define MAVLINK_MSG_ID_CURRENT_MODE 436


typedef struct __mavlink_current_mode_t {
 uint32_t custom_mode; /*<  A bitfield for use for autopilot-specific flags*/
 uint32_t intended_custom_mode; /*<  The custom_mode of the mode that was last commanded by the user (for example, with MAV_CMD_DO_SET_STANDARD_MODE, MAV_CMD_DO_SET_MODE or via RC). This should usually be the same as custom_mode. It will be different if the vehicle is unable to enter the intended mode, or has left that mode due to a failsafe condition. 0 indicates the intended custom mode is unknown/not supplied*/
 uint8_t standard_mode; /*<  Standard mode.*/
} mavlink_current_mode_t;

#define MAVLINK_MSG_ID_CURRENT_MODE_LEN 9
#define MAVLINK_MSG_ID_CURRENT_MODE_MIN_LEN 9
#define MAVLINK_MSG_ID_436_LEN 9
#define MAVLINK_MSG_ID_436_MIN_LEN 9

#define MAVLINK_MSG_ID_CURRENT_MODE_CRC 193
#define MAVLINK_MSG_ID_436_CRC 193



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CURRENT_MODE { \
    436, \
    "CURRENT_MODE", \
    3, \
    {  { "standard_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_current_mode_t, standard_mode) }, \
         { "custom_mode", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_current_mode_t, custom_mode) }, \
         { "intended_custom_mode", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_current_mode_t, intended_custom_mode) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CURRENT_MODE { \
    "CURRENT_MODE", \
    3, \
    {  { "standard_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_current_mode_t, standard_mode) }, \
         { "custom_mode", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_current_mode_t, custom_mode) }, \
         { "intended_custom_mode", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_current_mode_t, intended_custom_mode) }, \
         } \
}
#endif

/**
 * @brief Pack a current_mode message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param standard_mode  Standard mode.
 * @param custom_mode  A bitfield for use for autopilot-specific flags
 * @param intended_custom_mode  The custom_mode of the mode that was last commanded by the user (for example, with MAV_CMD_DO_SET_STANDARD_MODE, MAV_CMD_DO_SET_MODE or via RC). This should usually be the same as custom_mode. It will be different if the vehicle is unable to enter the intended mode, or has left that mode due to a failsafe condition. 0 indicates the intended custom mode is unknown/not supplied
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_current_mode_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t standard_mode, uint32_t custom_mode, uint32_t intended_custom_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CURRENT_MODE_LEN];
    _mav_put_uint32_t(buf, 0, custom_mode);
    _mav_put_uint32_t(buf, 4, intended_custom_mode);
    _mav_put_uint8_t(buf, 8, standard_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CURRENT_MODE_LEN);
#else
    mavlink_current_mode_t packet;
    packet.custom_mode = custom_mode;
    packet.intended_custom_mode = intended_custom_mode;
    packet.standard_mode = standard_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CURRENT_MODE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CURRENT_MODE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CURRENT_MODE_MIN_LEN, MAVLINK_MSG_ID_CURRENT_MODE_LEN, MAVLINK_MSG_ID_CURRENT_MODE_CRC);
}

/**
 * @brief Pack a current_mode message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param standard_mode  Standard mode.
 * @param custom_mode  A bitfield for use for autopilot-specific flags
 * @param intended_custom_mode  The custom_mode of the mode that was last commanded by the user (for example, with MAV_CMD_DO_SET_STANDARD_MODE, MAV_CMD_DO_SET_MODE or via RC). This should usually be the same as custom_mode. It will be different if the vehicle is unable to enter the intended mode, or has left that mode due to a failsafe condition. 0 indicates the intended custom mode is unknown/not supplied
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_current_mode_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t standard_mode, uint32_t custom_mode, uint32_t intended_custom_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CURRENT_MODE_LEN];
    _mav_put_uint32_t(buf, 0, custom_mode);
    _mav_put_uint32_t(buf, 4, intended_custom_mode);
    _mav_put_uint8_t(buf, 8, standard_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CURRENT_MODE_LEN);
#else
    mavlink_current_mode_t packet;
    packet.custom_mode = custom_mode;
    packet.intended_custom_mode = intended_custom_mode;
    packet.standard_mode = standard_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CURRENT_MODE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CURRENT_MODE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CURRENT_MODE_MIN_LEN, MAVLINK_MSG_ID_CURRENT_MODE_LEN, MAVLINK_MSG_ID_CURRENT_MODE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CURRENT_MODE_MIN_LEN, MAVLINK_MSG_ID_CURRENT_MODE_LEN);
#endif
}

/**
 * @brief Pack a current_mode message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param standard_mode  Standard mode.
 * @param custom_mode  A bitfield for use for autopilot-specific flags
 * @param intended_custom_mode  The custom_mode of the mode that was last commanded by the user (for example, with MAV_CMD_DO_SET_STANDARD_MODE, MAV_CMD_DO_SET_MODE or via RC). This should usually be the same as custom_mode. It will be different if the vehicle is unable to enter the intended mode, or has left that mode due to a failsafe condition. 0 indicates the intended custom mode is unknown/not supplied
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_current_mode_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t standard_mode,uint32_t custom_mode,uint32_t intended_custom_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CURRENT_MODE_LEN];
    _mav_put_uint32_t(buf, 0, custom_mode);
    _mav_put_uint32_t(buf, 4, intended_custom_mode);
    _mav_put_uint8_t(buf, 8, standard_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CURRENT_MODE_LEN);
#else
    mavlink_current_mode_t packet;
    packet.custom_mode = custom_mode;
    packet.intended_custom_mode = intended_custom_mode;
    packet.standard_mode = standard_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CURRENT_MODE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CURRENT_MODE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CURRENT_MODE_MIN_LEN, MAVLINK_MSG_ID_CURRENT_MODE_LEN, MAVLINK_MSG_ID_CURRENT_MODE_CRC);
}

/**
 * @brief Encode a current_mode struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param current_mode C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_current_mode_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_current_mode_t* current_mode)
{
    return mavlink_msg_current_mode_pack(system_id, component_id, msg, current_mode->standard_mode, current_mode->custom_mode, current_mode->intended_custom_mode);
}

/**
 * @brief Encode a current_mode struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param current_mode C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_current_mode_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_current_mode_t* current_mode)
{
    return mavlink_msg_current_mode_pack_chan(system_id, component_id, chan, msg, current_mode->standard_mode, current_mode->custom_mode, current_mode->intended_custom_mode);
}

/**
 * @brief Encode a current_mode struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param current_mode C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_current_mode_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_current_mode_t* current_mode)
{
    return mavlink_msg_current_mode_pack_status(system_id, component_id, _status, msg,  current_mode->standard_mode, current_mode->custom_mode, current_mode->intended_custom_mode);
}

/**
 * @brief Send a current_mode message
 * @param chan MAVLink channel to send the message
 *
 * @param standard_mode  Standard mode.
 * @param custom_mode  A bitfield for use for autopilot-specific flags
 * @param intended_custom_mode  The custom_mode of the mode that was last commanded by the user (for example, with MAV_CMD_DO_SET_STANDARD_MODE, MAV_CMD_DO_SET_MODE or via RC). This should usually be the same as custom_mode. It will be different if the vehicle is unable to enter the intended mode, or has left that mode due to a failsafe condition. 0 indicates the intended custom mode is unknown/not supplied
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_current_mode_send(mavlink_channel_t chan, uint8_t standard_mode, uint32_t custom_mode, uint32_t intended_custom_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CURRENT_MODE_LEN];
    _mav_put_uint32_t(buf, 0, custom_mode);
    _mav_put_uint32_t(buf, 4, intended_custom_mode);
    _mav_put_uint8_t(buf, 8, standard_mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CURRENT_MODE, buf, MAVLINK_MSG_ID_CURRENT_MODE_MIN_LEN, MAVLINK_MSG_ID_CURRENT_MODE_LEN, MAVLINK_MSG_ID_CURRENT_MODE_CRC);
#else
    mavlink_current_mode_t packet;
    packet.custom_mode = custom_mode;
    packet.intended_custom_mode = intended_custom_mode;
    packet.standard_mode = standard_mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CURRENT_MODE, (const char *)&packet, MAVLINK_MSG_ID_CURRENT_MODE_MIN_LEN, MAVLINK_MSG_ID_CURRENT_MODE_LEN, MAVLINK_MSG_ID_CURRENT_MODE_CRC);
#endif
}

/**
 * @brief Send a current_mode message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_current_mode_send_struct(mavlink_channel_t chan, const mavlink_current_mode_t* current_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_current_mode_send(chan, current_mode->standard_mode, current_mode->custom_mode, current_mode->intended_custom_mode);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CURRENT_MODE, (const char *)current_mode, MAVLINK_MSG_ID_CURRENT_MODE_MIN_LEN, MAVLINK_MSG_ID_CURRENT_MODE_LEN, MAVLINK_MSG_ID_CURRENT_MODE_CRC);
#endif
}

#if MAVLINK_MSG_ID_CURRENT_MODE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_current_mode_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t standard_mode, uint32_t custom_mode, uint32_t intended_custom_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, custom_mode);
    _mav_put_uint32_t(buf, 4, intended_custom_mode);
    _mav_put_uint8_t(buf, 8, standard_mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CURRENT_MODE, buf, MAVLINK_MSG_ID_CURRENT_MODE_MIN_LEN, MAVLINK_MSG_ID_CURRENT_MODE_LEN, MAVLINK_MSG_ID_CURRENT_MODE_CRC);
#else
    mavlink_current_mode_t *packet = (mavlink_current_mode_t *)msgbuf;
    packet->custom_mode = custom_mode;
    packet->intended_custom_mode = intended_custom_mode;
    packet->standard_mode = standard_mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CURRENT_MODE, (const char *)packet, MAVLINK_MSG_ID_CURRENT_MODE_MIN_LEN, MAVLINK_MSG_ID_CURRENT_MODE_LEN, MAVLINK_MSG_ID_CURRENT_MODE_CRC);
#endif
}
#endif

#endif

// MESSAGE CURRENT_MODE UNPACKING


/**
 * @brief Get field standard_mode from current_mode message
 *
 * @return  Standard mode.
 */
static inline uint8_t mavlink_msg_current_mode_get_standard_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field custom_mode from current_mode message
 *
 * @return  A bitfield for use for autopilot-specific flags
 */
static inline uint32_t mavlink_msg_current_mode_get_custom_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field intended_custom_mode from current_mode message
 *
 * @return  The custom_mode of the mode that was last commanded by the user (for example, with MAV_CMD_DO_SET_STANDARD_MODE, MAV_CMD_DO_SET_MODE or via RC). This should usually be the same as custom_mode. It will be different if the vehicle is unable to enter the intended mode, or has left that mode due to a failsafe condition. 0 indicates the intended custom mode is unknown/not supplied
 */
static inline uint32_t mavlink_msg_current_mode_get_intended_custom_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Decode a current_mode message into a struct
 *
 * @param msg The message to decode
 * @param current_mode C-struct to decode the message contents into
 */
static inline void mavlink_msg_current_mode_decode(const mavlink_message_t* msg, mavlink_current_mode_t* current_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    current_mode->custom_mode = mavlink_msg_current_mode_get_custom_mode(msg);
    current_mode->intended_custom_mode = mavlink_msg_current_mode_get_intended_custom_mode(msg);
    current_mode->standard_mode = mavlink_msg_current_mode_get_standard_mode(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CURRENT_MODE_LEN? msg->len : MAVLINK_MSG_ID_CURRENT_MODE_LEN;
        memset(current_mode, 0, MAVLINK_MSG_ID_CURRENT_MODE_LEN);
    memcpy(current_mode, _MAV_PAYLOAD(msg), len);
#endif
}
