#pragma once
// MESSAGE RADIO_RC_CHANNELS PACKING

#define MAVLINK_MSG_ID_RADIO_RC_CHANNELS 420

MAVPACKED(
typedef struct __mavlink_radio_rc_channels_t {
 uint32_t time_last_update_ms; /*< [ms] Time when the data in the channels field were last updated (time since boot in the receiver's time domain).*/
 uint16_t flags; /*<  Radio RC channels status flags.*/
 uint8_t target_system; /*<  System ID (ID of target system, normally flight controller).*/
 uint8_t target_component; /*<  Component ID (normally 0 for broadcast).*/
 uint8_t count; /*<  Total number of RC channels being received. This can be larger than 32, indicating that more channels are available but not given in this message.*/
 int16_t channels[32]; /*<  RC channels.
        Channel values are in centered 13 bit format. Range is -4096 to 4096, center is 0. Conversion to PWM is x * 5/32 + 1500.
        Channels with indexes equal or above count should be set to 0, to benefit from MAVLink's trailing-zero trimming.*/
}) mavlink_radio_rc_channels_t;

#define MAVLINK_MSG_ID_RADIO_RC_CHANNELS_LEN 73
#define MAVLINK_MSG_ID_RADIO_RC_CHANNELS_MIN_LEN 9
#define MAVLINK_MSG_ID_420_LEN 73
#define MAVLINK_MSG_ID_420_MIN_LEN 9

#define MAVLINK_MSG_ID_RADIO_RC_CHANNELS_CRC 20
#define MAVLINK_MSG_ID_420_CRC 20

#define MAVLINK_MSG_RADIO_RC_CHANNELS_FIELD_CHANNELS_LEN 32

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RADIO_RC_CHANNELS { \
    420, \
    "RADIO_RC_CHANNELS", \
    6, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_radio_rc_channels_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_radio_rc_channels_t, target_component) }, \
         { "time_last_update_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_radio_rc_channels_t, time_last_update_ms) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_radio_rc_channels_t, flags) }, \
         { "count", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_radio_rc_channels_t, count) }, \
         { "channels", NULL, MAVLINK_TYPE_INT16_T, 32, 9, offsetof(mavlink_radio_rc_channels_t, channels) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RADIO_RC_CHANNELS { \
    "RADIO_RC_CHANNELS", \
    6, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_radio_rc_channels_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_radio_rc_channels_t, target_component) }, \
         { "time_last_update_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_radio_rc_channels_t, time_last_update_ms) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_radio_rc_channels_t, flags) }, \
         { "count", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_radio_rc_channels_t, count) }, \
         { "channels", NULL, MAVLINK_TYPE_INT16_T, 32, 9, offsetof(mavlink_radio_rc_channels_t, channels) }, \
         } \
}
#endif

/**
 * @brief Pack a radio_rc_channels message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID (ID of target system, normally flight controller).
 * @param target_component  Component ID (normally 0 for broadcast).
 * @param time_last_update_ms [ms] Time when the data in the channels field were last updated (time since boot in the receiver's time domain).
 * @param flags  Radio RC channels status flags.
 * @param count  Total number of RC channels being received. This can be larger than 32, indicating that more channels are available but not given in this message.
 * @param channels  RC channels.
        Channel values are in centered 13 bit format. Range is -4096 to 4096, center is 0. Conversion to PWM is x * 5/32 + 1500.
        Channels with indexes equal or above count should be set to 0, to benefit from MAVLink's trailing-zero trimming.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radio_rc_channels_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint32_t time_last_update_ms, uint16_t flags, uint8_t count, const int16_t *channels)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RADIO_RC_CHANNELS_LEN];
    _mav_put_uint32_t(buf, 0, time_last_update_ms);
    _mav_put_uint16_t(buf, 4, flags);
    _mav_put_uint8_t(buf, 6, target_system);
    _mav_put_uint8_t(buf, 7, target_component);
    _mav_put_uint8_t(buf, 8, count);
    _mav_put_int16_t_array(buf, 9, channels, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RADIO_RC_CHANNELS_LEN);
#else
    mavlink_radio_rc_channels_t packet;
    packet.time_last_update_ms = time_last_update_ms;
    packet.flags = flags;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.count = count;
    mav_array_memcpy(packet.channels, channels, sizeof(int16_t)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RADIO_RC_CHANNELS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RADIO_RC_CHANNELS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RADIO_RC_CHANNELS_MIN_LEN, MAVLINK_MSG_ID_RADIO_RC_CHANNELS_LEN, MAVLINK_MSG_ID_RADIO_RC_CHANNELS_CRC);
}

/**
 * @brief Pack a radio_rc_channels message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID (ID of target system, normally flight controller).
 * @param target_component  Component ID (normally 0 for broadcast).
 * @param time_last_update_ms [ms] Time when the data in the channels field were last updated (time since boot in the receiver's time domain).
 * @param flags  Radio RC channels status flags.
 * @param count  Total number of RC channels being received. This can be larger than 32, indicating that more channels are available but not given in this message.
 * @param channels  RC channels.
        Channel values are in centered 13 bit format. Range is -4096 to 4096, center is 0. Conversion to PWM is x * 5/32 + 1500.
        Channels with indexes equal or above count should be set to 0, to benefit from MAVLink's trailing-zero trimming.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radio_rc_channels_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint32_t time_last_update_ms, uint16_t flags, uint8_t count, const int16_t *channels)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RADIO_RC_CHANNELS_LEN];
    _mav_put_uint32_t(buf, 0, time_last_update_ms);
    _mav_put_uint16_t(buf, 4, flags);
    _mav_put_uint8_t(buf, 6, target_system);
    _mav_put_uint8_t(buf, 7, target_component);
    _mav_put_uint8_t(buf, 8, count);
    _mav_put_int16_t_array(buf, 9, channels, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RADIO_RC_CHANNELS_LEN);
#else
    mavlink_radio_rc_channels_t packet;
    packet.time_last_update_ms = time_last_update_ms;
    packet.flags = flags;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.count = count;
    mav_array_memcpy(packet.channels, channels, sizeof(int16_t)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RADIO_RC_CHANNELS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RADIO_RC_CHANNELS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_RADIO_RC_CHANNELS_MIN_LEN, MAVLINK_MSG_ID_RADIO_RC_CHANNELS_LEN, MAVLINK_MSG_ID_RADIO_RC_CHANNELS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_RADIO_RC_CHANNELS_MIN_LEN, MAVLINK_MSG_ID_RADIO_RC_CHANNELS_LEN);
#endif
}

/**
 * @brief Pack a radio_rc_channels message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID (ID of target system, normally flight controller).
 * @param target_component  Component ID (normally 0 for broadcast).
 * @param time_last_update_ms [ms] Time when the data in the channels field were last updated (time since boot in the receiver's time domain).
 * @param flags  Radio RC channels status flags.
 * @param count  Total number of RC channels being received. This can be larger than 32, indicating that more channels are available but not given in this message.
 * @param channels  RC channels.
        Channel values are in centered 13 bit format. Range is -4096 to 4096, center is 0. Conversion to PWM is x * 5/32 + 1500.
        Channels with indexes equal or above count should be set to 0, to benefit from MAVLink's trailing-zero trimming.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radio_rc_channels_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint32_t time_last_update_ms,uint16_t flags,uint8_t count,const int16_t *channels)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RADIO_RC_CHANNELS_LEN];
    _mav_put_uint32_t(buf, 0, time_last_update_ms);
    _mav_put_uint16_t(buf, 4, flags);
    _mav_put_uint8_t(buf, 6, target_system);
    _mav_put_uint8_t(buf, 7, target_component);
    _mav_put_uint8_t(buf, 8, count);
    _mav_put_int16_t_array(buf, 9, channels, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RADIO_RC_CHANNELS_LEN);
#else
    mavlink_radio_rc_channels_t packet;
    packet.time_last_update_ms = time_last_update_ms;
    packet.flags = flags;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.count = count;
    mav_array_memcpy(packet.channels, channels, sizeof(int16_t)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RADIO_RC_CHANNELS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RADIO_RC_CHANNELS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RADIO_RC_CHANNELS_MIN_LEN, MAVLINK_MSG_ID_RADIO_RC_CHANNELS_LEN, MAVLINK_MSG_ID_RADIO_RC_CHANNELS_CRC);
}

/**
 * @brief Encode a radio_rc_channels struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param radio_rc_channels C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_radio_rc_channels_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_radio_rc_channels_t* radio_rc_channels)
{
    return mavlink_msg_radio_rc_channels_pack(system_id, component_id, msg, radio_rc_channels->target_system, radio_rc_channels->target_component, radio_rc_channels->time_last_update_ms, radio_rc_channels->flags, radio_rc_channels->count, radio_rc_channels->channels);
}

/**
 * @brief Encode a radio_rc_channels struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param radio_rc_channels C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_radio_rc_channels_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_radio_rc_channels_t* radio_rc_channels)
{
    return mavlink_msg_radio_rc_channels_pack_chan(system_id, component_id, chan, msg, radio_rc_channels->target_system, radio_rc_channels->target_component, radio_rc_channels->time_last_update_ms, radio_rc_channels->flags, radio_rc_channels->count, radio_rc_channels->channels);
}

/**
 * @brief Encode a radio_rc_channels struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param radio_rc_channels C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_radio_rc_channels_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_radio_rc_channels_t* radio_rc_channels)
{
    return mavlink_msg_radio_rc_channels_pack_status(system_id, component_id, _status, msg,  radio_rc_channels->target_system, radio_rc_channels->target_component, radio_rc_channels->time_last_update_ms, radio_rc_channels->flags, radio_rc_channels->count, radio_rc_channels->channels);
}

/**
 * @brief Send a radio_rc_channels message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID (ID of target system, normally flight controller).
 * @param target_component  Component ID (normally 0 for broadcast).
 * @param time_last_update_ms [ms] Time when the data in the channels field were last updated (time since boot in the receiver's time domain).
 * @param flags  Radio RC channels status flags.
 * @param count  Total number of RC channels being received. This can be larger than 32, indicating that more channels are available but not given in this message.
 * @param channels  RC channels.
        Channel values are in centered 13 bit format. Range is -4096 to 4096, center is 0. Conversion to PWM is x * 5/32 + 1500.
        Channels with indexes equal or above count should be set to 0, to benefit from MAVLink's trailing-zero trimming.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_radio_rc_channels_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint32_t time_last_update_ms, uint16_t flags, uint8_t count, const int16_t *channels)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RADIO_RC_CHANNELS_LEN];
    _mav_put_uint32_t(buf, 0, time_last_update_ms);
    _mav_put_uint16_t(buf, 4, flags);
    _mav_put_uint8_t(buf, 6, target_system);
    _mav_put_uint8_t(buf, 7, target_component);
    _mav_put_uint8_t(buf, 8, count);
    _mav_put_int16_t_array(buf, 9, channels, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADIO_RC_CHANNELS, buf, MAVLINK_MSG_ID_RADIO_RC_CHANNELS_MIN_LEN, MAVLINK_MSG_ID_RADIO_RC_CHANNELS_LEN, MAVLINK_MSG_ID_RADIO_RC_CHANNELS_CRC);
#else
    mavlink_radio_rc_channels_t packet;
    packet.time_last_update_ms = time_last_update_ms;
    packet.flags = flags;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.count = count;
    mav_array_memcpy(packet.channels, channels, sizeof(int16_t)*32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADIO_RC_CHANNELS, (const char *)&packet, MAVLINK_MSG_ID_RADIO_RC_CHANNELS_MIN_LEN, MAVLINK_MSG_ID_RADIO_RC_CHANNELS_LEN, MAVLINK_MSG_ID_RADIO_RC_CHANNELS_CRC);
#endif
}

/**
 * @brief Send a radio_rc_channels message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_radio_rc_channels_send_struct(mavlink_channel_t chan, const mavlink_radio_rc_channels_t* radio_rc_channels)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_radio_rc_channels_send(chan, radio_rc_channels->target_system, radio_rc_channels->target_component, radio_rc_channels->time_last_update_ms, radio_rc_channels->flags, radio_rc_channels->count, radio_rc_channels->channels);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADIO_RC_CHANNELS, (const char *)radio_rc_channels, MAVLINK_MSG_ID_RADIO_RC_CHANNELS_MIN_LEN, MAVLINK_MSG_ID_RADIO_RC_CHANNELS_LEN, MAVLINK_MSG_ID_RADIO_RC_CHANNELS_CRC);
#endif
}

#if MAVLINK_MSG_ID_RADIO_RC_CHANNELS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_radio_rc_channels_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint32_t time_last_update_ms, uint16_t flags, uint8_t count, const int16_t *channels)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_last_update_ms);
    _mav_put_uint16_t(buf, 4, flags);
    _mav_put_uint8_t(buf, 6, target_system);
    _mav_put_uint8_t(buf, 7, target_component);
    _mav_put_uint8_t(buf, 8, count);
    _mav_put_int16_t_array(buf, 9, channels, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADIO_RC_CHANNELS, buf, MAVLINK_MSG_ID_RADIO_RC_CHANNELS_MIN_LEN, MAVLINK_MSG_ID_RADIO_RC_CHANNELS_LEN, MAVLINK_MSG_ID_RADIO_RC_CHANNELS_CRC);
#else
    mavlink_radio_rc_channels_t *packet = (mavlink_radio_rc_channels_t *)msgbuf;
    packet->time_last_update_ms = time_last_update_ms;
    packet->flags = flags;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->count = count;
    mav_array_memcpy(packet->channels, channels, sizeof(int16_t)*32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADIO_RC_CHANNELS, (const char *)packet, MAVLINK_MSG_ID_RADIO_RC_CHANNELS_MIN_LEN, MAVLINK_MSG_ID_RADIO_RC_CHANNELS_LEN, MAVLINK_MSG_ID_RADIO_RC_CHANNELS_CRC);
#endif
}
#endif

#endif

// MESSAGE RADIO_RC_CHANNELS UNPACKING


/**
 * @brief Get field target_system from radio_rc_channels message
 *
 * @return  System ID (ID of target system, normally flight controller).
 */
static inline uint8_t mavlink_msg_radio_rc_channels_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field target_component from radio_rc_channels message
 *
 * @return  Component ID (normally 0 for broadcast).
 */
static inline uint8_t mavlink_msg_radio_rc_channels_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field time_last_update_ms from radio_rc_channels message
 *
 * @return [ms] Time when the data in the channels field were last updated (time since boot in the receiver's time domain).
 */
static inline uint32_t mavlink_msg_radio_rc_channels_get_time_last_update_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field flags from radio_rc_channels message
 *
 * @return  Radio RC channels status flags.
 */
static inline uint16_t mavlink_msg_radio_rc_channels_get_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field count from radio_rc_channels message
 *
 * @return  Total number of RC channels being received. This can be larger than 32, indicating that more channels are available but not given in this message.
 */
static inline uint8_t mavlink_msg_radio_rc_channels_get_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field channels from radio_rc_channels message
 *
 * @return  RC channels.
        Channel values are in centered 13 bit format. Range is -4096 to 4096, center is 0. Conversion to PWM is x * 5/32 + 1500.
        Channels with indexes equal or above count should be set to 0, to benefit from MAVLink's trailing-zero trimming.
 */
static inline uint16_t mavlink_msg_radio_rc_channels_get_channels(const mavlink_message_t* msg, int16_t *channels)
{
    return _MAV_RETURN_int16_t_array(msg, channels, 32,  9);
}

/**
 * @brief Decode a radio_rc_channels message into a struct
 *
 * @param msg The message to decode
 * @param radio_rc_channels C-struct to decode the message contents into
 */
static inline void mavlink_msg_radio_rc_channels_decode(const mavlink_message_t* msg, mavlink_radio_rc_channels_t* radio_rc_channels)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    radio_rc_channels->time_last_update_ms = mavlink_msg_radio_rc_channels_get_time_last_update_ms(msg);
    radio_rc_channels->flags = mavlink_msg_radio_rc_channels_get_flags(msg);
    radio_rc_channels->target_system = mavlink_msg_radio_rc_channels_get_target_system(msg);
    radio_rc_channels->target_component = mavlink_msg_radio_rc_channels_get_target_component(msg);
    radio_rc_channels->count = mavlink_msg_radio_rc_channels_get_count(msg);
    mavlink_msg_radio_rc_channels_get_channels(msg, radio_rc_channels->channels);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RADIO_RC_CHANNELS_LEN? msg->len : MAVLINK_MSG_ID_RADIO_RC_CHANNELS_LEN;
        memset(radio_rc_channels, 0, MAVLINK_MSG_ID_RADIO_RC_CHANNELS_LEN);
    memcpy(radio_rc_channels, _MAV_PAYLOAD(msg), len);
#endif
}
