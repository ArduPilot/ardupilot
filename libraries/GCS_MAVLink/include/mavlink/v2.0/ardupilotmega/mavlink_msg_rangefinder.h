#pragma once
// MESSAGE RANGEFINDER PACKING

#define MAVLINK_MSG_ID_RANGEFINDER 173


typedef struct __mavlink_rangefinder_t {
 float distance; /*< [m] Distance.*/
 float voltage; /*< [V] Raw voltage if available, zero otherwise.*/
} mavlink_rangefinder_t;

#define MAVLINK_MSG_ID_RANGEFINDER_LEN 8
#define MAVLINK_MSG_ID_RANGEFINDER_MIN_LEN 8
#define MAVLINK_MSG_ID_173_LEN 8
#define MAVLINK_MSG_ID_173_MIN_LEN 8

#define MAVLINK_MSG_ID_RANGEFINDER_CRC 83
#define MAVLINK_MSG_ID_173_CRC 83



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RANGEFINDER { \
    173, \
    "RANGEFINDER", \
    2, \
    {  { "distance", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_rangefinder_t, distance) }, \
         { "voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_rangefinder_t, voltage) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RANGEFINDER { \
    "RANGEFINDER", \
    2, \
    {  { "distance", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_rangefinder_t, distance) }, \
         { "voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_rangefinder_t, voltage) }, \
         } \
}
#endif

/**
 * @brief Pack a rangefinder message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param distance [m] Distance.
 * @param voltage [V] Raw voltage if available, zero otherwise.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rangefinder_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float distance, float voltage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RANGEFINDER_LEN];
    _mav_put_float(buf, 0, distance);
    _mav_put_float(buf, 4, voltage);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RANGEFINDER_LEN);
#else
    mavlink_rangefinder_t packet;
    packet.distance = distance;
    packet.voltage = voltage;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RANGEFINDER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RANGEFINDER;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RANGEFINDER_MIN_LEN, MAVLINK_MSG_ID_RANGEFINDER_LEN, MAVLINK_MSG_ID_RANGEFINDER_CRC);
}

/**
 * @brief Pack a rangefinder message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param distance [m] Distance.
 * @param voltage [V] Raw voltage if available, zero otherwise.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rangefinder_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float distance, float voltage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RANGEFINDER_LEN];
    _mav_put_float(buf, 0, distance);
    _mav_put_float(buf, 4, voltage);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RANGEFINDER_LEN);
#else
    mavlink_rangefinder_t packet;
    packet.distance = distance;
    packet.voltage = voltage;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RANGEFINDER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RANGEFINDER;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_RANGEFINDER_MIN_LEN, MAVLINK_MSG_ID_RANGEFINDER_LEN, MAVLINK_MSG_ID_RANGEFINDER_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_RANGEFINDER_MIN_LEN, MAVLINK_MSG_ID_RANGEFINDER_LEN);
#endif
}

/**
 * @brief Pack a rangefinder message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param distance [m] Distance.
 * @param voltage [V] Raw voltage if available, zero otherwise.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rangefinder_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float distance,float voltage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RANGEFINDER_LEN];
    _mav_put_float(buf, 0, distance);
    _mav_put_float(buf, 4, voltage);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RANGEFINDER_LEN);
#else
    mavlink_rangefinder_t packet;
    packet.distance = distance;
    packet.voltage = voltage;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RANGEFINDER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RANGEFINDER;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RANGEFINDER_MIN_LEN, MAVLINK_MSG_ID_RANGEFINDER_LEN, MAVLINK_MSG_ID_RANGEFINDER_CRC);
}

/**
 * @brief Encode a rangefinder struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rangefinder C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rangefinder_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rangefinder_t* rangefinder)
{
    return mavlink_msg_rangefinder_pack(system_id, component_id, msg, rangefinder->distance, rangefinder->voltage);
}

/**
 * @brief Encode a rangefinder struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rangefinder C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rangefinder_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rangefinder_t* rangefinder)
{
    return mavlink_msg_rangefinder_pack_chan(system_id, component_id, chan, msg, rangefinder->distance, rangefinder->voltage);
}

/**
 * @brief Encode a rangefinder struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param rangefinder C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rangefinder_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_rangefinder_t* rangefinder)
{
    return mavlink_msg_rangefinder_pack_status(system_id, component_id, _status, msg,  rangefinder->distance, rangefinder->voltage);
}

/**
 * @brief Send a rangefinder message
 * @param chan MAVLink channel to send the message
 *
 * @param distance [m] Distance.
 * @param voltage [V] Raw voltage if available, zero otherwise.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rangefinder_send(mavlink_channel_t chan, float distance, float voltage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RANGEFINDER_LEN];
    _mav_put_float(buf, 0, distance);
    _mav_put_float(buf, 4, voltage);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RANGEFINDER, buf, MAVLINK_MSG_ID_RANGEFINDER_MIN_LEN, MAVLINK_MSG_ID_RANGEFINDER_LEN, MAVLINK_MSG_ID_RANGEFINDER_CRC);
#else
    mavlink_rangefinder_t packet;
    packet.distance = distance;
    packet.voltage = voltage;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RANGEFINDER, (const char *)&packet, MAVLINK_MSG_ID_RANGEFINDER_MIN_LEN, MAVLINK_MSG_ID_RANGEFINDER_LEN, MAVLINK_MSG_ID_RANGEFINDER_CRC);
#endif
}

/**
 * @brief Send a rangefinder message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_rangefinder_send_struct(mavlink_channel_t chan, const mavlink_rangefinder_t* rangefinder)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_rangefinder_send(chan, rangefinder->distance, rangefinder->voltage);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RANGEFINDER, (const char *)rangefinder, MAVLINK_MSG_ID_RANGEFINDER_MIN_LEN, MAVLINK_MSG_ID_RANGEFINDER_LEN, MAVLINK_MSG_ID_RANGEFINDER_CRC);
#endif
}

#if MAVLINK_MSG_ID_RANGEFINDER_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rangefinder_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float distance, float voltage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, distance);
    _mav_put_float(buf, 4, voltage);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RANGEFINDER, buf, MAVLINK_MSG_ID_RANGEFINDER_MIN_LEN, MAVLINK_MSG_ID_RANGEFINDER_LEN, MAVLINK_MSG_ID_RANGEFINDER_CRC);
#else
    mavlink_rangefinder_t *packet = (mavlink_rangefinder_t *)msgbuf;
    packet->distance = distance;
    packet->voltage = voltage;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RANGEFINDER, (const char *)packet, MAVLINK_MSG_ID_RANGEFINDER_MIN_LEN, MAVLINK_MSG_ID_RANGEFINDER_LEN, MAVLINK_MSG_ID_RANGEFINDER_CRC);
#endif
}
#endif

#endif

// MESSAGE RANGEFINDER UNPACKING


/**
 * @brief Get field distance from rangefinder message
 *
 * @return [m] Distance.
 */
static inline float mavlink_msg_rangefinder_get_distance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field voltage from rangefinder message
 *
 * @return [V] Raw voltage if available, zero otherwise.
 */
static inline float mavlink_msg_rangefinder_get_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a rangefinder message into a struct
 *
 * @param msg The message to decode
 * @param rangefinder C-struct to decode the message contents into
 */
static inline void mavlink_msg_rangefinder_decode(const mavlink_message_t* msg, mavlink_rangefinder_t* rangefinder)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    rangefinder->distance = mavlink_msg_rangefinder_get_distance(msg);
    rangefinder->voltage = mavlink_msg_rangefinder_get_voltage(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RANGEFINDER_LEN? msg->len : MAVLINK_MSG_ID_RANGEFINDER_LEN;
        memset(rangefinder, 0, MAVLINK_MSG_ID_RANGEFINDER_LEN);
    memcpy(rangefinder, _MAV_PAYLOAD(msg), len);
#endif
}
