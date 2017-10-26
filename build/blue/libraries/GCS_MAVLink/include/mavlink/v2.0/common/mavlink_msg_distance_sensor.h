#pragma once
// MESSAGE DISTANCE_SENSOR PACKING

#define MAVLINK_MSG_ID_DISTANCE_SENSOR 132

MAVPACKED(
typedef struct __mavlink_distance_sensor_t {
 uint32_t time_boot_ms; /*< Time since system boot*/
 uint16_t min_distance; /*< Minimum distance the sensor can measure in centimeters*/
 uint16_t max_distance; /*< Maximum distance the sensor can measure in centimeters*/
 uint16_t current_distance; /*< Current distance reading*/
 uint8_t type; /*< Type from MAV_DISTANCE_SENSOR enum.*/
 uint8_t id; /*< Onboard ID of the sensor*/
 uint8_t orientation; /*< Direction the sensor faces from MAV_SENSOR_ORIENTATION enum.*/
 uint8_t covariance; /*< Measurement covariance in centimeters, 0 for unknown / invalid readings*/
}) mavlink_distance_sensor_t;

#define MAVLINK_MSG_ID_DISTANCE_SENSOR_LEN 14
#define MAVLINK_MSG_ID_DISTANCE_SENSOR_MIN_LEN 14
#define MAVLINK_MSG_ID_132_LEN 14
#define MAVLINK_MSG_ID_132_MIN_LEN 14

#define MAVLINK_MSG_ID_DISTANCE_SENSOR_CRC 85
#define MAVLINK_MSG_ID_132_CRC 85



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DISTANCE_SENSOR { \
    132, \
    "DISTANCE_SENSOR", \
    8, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_distance_sensor_t, time_boot_ms) }, \
         { "min_distance", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_distance_sensor_t, min_distance) }, \
         { "max_distance", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_distance_sensor_t, max_distance) }, \
         { "current_distance", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_distance_sensor_t, current_distance) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_distance_sensor_t, type) }, \
         { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_distance_sensor_t, id) }, \
         { "orientation", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_distance_sensor_t, orientation) }, \
         { "covariance", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_distance_sensor_t, covariance) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DISTANCE_SENSOR { \
    "DISTANCE_SENSOR", \
    8, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_distance_sensor_t, time_boot_ms) }, \
         { "min_distance", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_distance_sensor_t, min_distance) }, \
         { "max_distance", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_distance_sensor_t, max_distance) }, \
         { "current_distance", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_distance_sensor_t, current_distance) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_distance_sensor_t, type) }, \
         { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_distance_sensor_t, id) }, \
         { "orientation", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_distance_sensor_t, orientation) }, \
         { "covariance", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_distance_sensor_t, covariance) }, \
         } \
}
#endif

/**
 * @brief Pack a distance_sensor message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Time since system boot
 * @param min_distance Minimum distance the sensor can measure in centimeters
 * @param max_distance Maximum distance the sensor can measure in centimeters
 * @param current_distance Current distance reading
 * @param type Type from MAV_DISTANCE_SENSOR enum.
 * @param id Onboard ID of the sensor
 * @param orientation Direction the sensor faces from MAV_SENSOR_ORIENTATION enum.
 * @param covariance Measurement covariance in centimeters, 0 for unknown / invalid readings
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_distance_sensor_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint16_t min_distance, uint16_t max_distance, uint16_t current_distance, uint8_t type, uint8_t id, uint8_t orientation, uint8_t covariance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DISTANCE_SENSOR_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint16_t(buf, 4, min_distance);
    _mav_put_uint16_t(buf, 6, max_distance);
    _mav_put_uint16_t(buf, 8, current_distance);
    _mav_put_uint8_t(buf, 10, type);
    _mav_put_uint8_t(buf, 11, id);
    _mav_put_uint8_t(buf, 12, orientation);
    _mav_put_uint8_t(buf, 13, covariance);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DISTANCE_SENSOR_LEN);
#else
    mavlink_distance_sensor_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.min_distance = min_distance;
    packet.max_distance = max_distance;
    packet.current_distance = current_distance;
    packet.type = type;
    packet.id = id;
    packet.orientation = orientation;
    packet.covariance = covariance;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DISTANCE_SENSOR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DISTANCE_SENSOR;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DISTANCE_SENSOR_MIN_LEN, MAVLINK_MSG_ID_DISTANCE_SENSOR_LEN, MAVLINK_MSG_ID_DISTANCE_SENSOR_CRC);
}

/**
 * @brief Pack a distance_sensor message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Time since system boot
 * @param min_distance Minimum distance the sensor can measure in centimeters
 * @param max_distance Maximum distance the sensor can measure in centimeters
 * @param current_distance Current distance reading
 * @param type Type from MAV_DISTANCE_SENSOR enum.
 * @param id Onboard ID of the sensor
 * @param orientation Direction the sensor faces from MAV_SENSOR_ORIENTATION enum.
 * @param covariance Measurement covariance in centimeters, 0 for unknown / invalid readings
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_distance_sensor_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,uint16_t min_distance,uint16_t max_distance,uint16_t current_distance,uint8_t type,uint8_t id,uint8_t orientation,uint8_t covariance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DISTANCE_SENSOR_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint16_t(buf, 4, min_distance);
    _mav_put_uint16_t(buf, 6, max_distance);
    _mav_put_uint16_t(buf, 8, current_distance);
    _mav_put_uint8_t(buf, 10, type);
    _mav_put_uint8_t(buf, 11, id);
    _mav_put_uint8_t(buf, 12, orientation);
    _mav_put_uint8_t(buf, 13, covariance);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DISTANCE_SENSOR_LEN);
#else
    mavlink_distance_sensor_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.min_distance = min_distance;
    packet.max_distance = max_distance;
    packet.current_distance = current_distance;
    packet.type = type;
    packet.id = id;
    packet.orientation = orientation;
    packet.covariance = covariance;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DISTANCE_SENSOR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DISTANCE_SENSOR;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DISTANCE_SENSOR_MIN_LEN, MAVLINK_MSG_ID_DISTANCE_SENSOR_LEN, MAVLINK_MSG_ID_DISTANCE_SENSOR_CRC);
}

/**
 * @brief Encode a distance_sensor struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param distance_sensor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_distance_sensor_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_distance_sensor_t* distance_sensor)
{
    return mavlink_msg_distance_sensor_pack(system_id, component_id, msg, distance_sensor->time_boot_ms, distance_sensor->min_distance, distance_sensor->max_distance, distance_sensor->current_distance, distance_sensor->type, distance_sensor->id, distance_sensor->orientation, distance_sensor->covariance);
}

/**
 * @brief Encode a distance_sensor struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param distance_sensor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_distance_sensor_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_distance_sensor_t* distance_sensor)
{
    return mavlink_msg_distance_sensor_pack_chan(system_id, component_id, chan, msg, distance_sensor->time_boot_ms, distance_sensor->min_distance, distance_sensor->max_distance, distance_sensor->current_distance, distance_sensor->type, distance_sensor->id, distance_sensor->orientation, distance_sensor->covariance);
}

/**
 * @brief Send a distance_sensor message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Time since system boot
 * @param min_distance Minimum distance the sensor can measure in centimeters
 * @param max_distance Maximum distance the sensor can measure in centimeters
 * @param current_distance Current distance reading
 * @param type Type from MAV_DISTANCE_SENSOR enum.
 * @param id Onboard ID of the sensor
 * @param orientation Direction the sensor faces from MAV_SENSOR_ORIENTATION enum.
 * @param covariance Measurement covariance in centimeters, 0 for unknown / invalid readings
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_distance_sensor_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint16_t min_distance, uint16_t max_distance, uint16_t current_distance, uint8_t type, uint8_t id, uint8_t orientation, uint8_t covariance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DISTANCE_SENSOR_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint16_t(buf, 4, min_distance);
    _mav_put_uint16_t(buf, 6, max_distance);
    _mav_put_uint16_t(buf, 8, current_distance);
    _mav_put_uint8_t(buf, 10, type);
    _mav_put_uint8_t(buf, 11, id);
    _mav_put_uint8_t(buf, 12, orientation);
    _mav_put_uint8_t(buf, 13, covariance);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DISTANCE_SENSOR, buf, MAVLINK_MSG_ID_DISTANCE_SENSOR_MIN_LEN, MAVLINK_MSG_ID_DISTANCE_SENSOR_LEN, MAVLINK_MSG_ID_DISTANCE_SENSOR_CRC);
#else
    mavlink_distance_sensor_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.min_distance = min_distance;
    packet.max_distance = max_distance;
    packet.current_distance = current_distance;
    packet.type = type;
    packet.id = id;
    packet.orientation = orientation;
    packet.covariance = covariance;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DISTANCE_SENSOR, (const char *)&packet, MAVLINK_MSG_ID_DISTANCE_SENSOR_MIN_LEN, MAVLINK_MSG_ID_DISTANCE_SENSOR_LEN, MAVLINK_MSG_ID_DISTANCE_SENSOR_CRC);
#endif
}

/**
 * @brief Send a distance_sensor message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_distance_sensor_send_struct(mavlink_channel_t chan, const mavlink_distance_sensor_t* distance_sensor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_distance_sensor_send(chan, distance_sensor->time_boot_ms, distance_sensor->min_distance, distance_sensor->max_distance, distance_sensor->current_distance, distance_sensor->type, distance_sensor->id, distance_sensor->orientation, distance_sensor->covariance);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DISTANCE_SENSOR, (const char *)distance_sensor, MAVLINK_MSG_ID_DISTANCE_SENSOR_MIN_LEN, MAVLINK_MSG_ID_DISTANCE_SENSOR_LEN, MAVLINK_MSG_ID_DISTANCE_SENSOR_CRC);
#endif
}

#if MAVLINK_MSG_ID_DISTANCE_SENSOR_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_distance_sensor_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint16_t min_distance, uint16_t max_distance, uint16_t current_distance, uint8_t type, uint8_t id, uint8_t orientation, uint8_t covariance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint16_t(buf, 4, min_distance);
    _mav_put_uint16_t(buf, 6, max_distance);
    _mav_put_uint16_t(buf, 8, current_distance);
    _mav_put_uint8_t(buf, 10, type);
    _mav_put_uint8_t(buf, 11, id);
    _mav_put_uint8_t(buf, 12, orientation);
    _mav_put_uint8_t(buf, 13, covariance);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DISTANCE_SENSOR, buf, MAVLINK_MSG_ID_DISTANCE_SENSOR_MIN_LEN, MAVLINK_MSG_ID_DISTANCE_SENSOR_LEN, MAVLINK_MSG_ID_DISTANCE_SENSOR_CRC);
#else
    mavlink_distance_sensor_t *packet = (mavlink_distance_sensor_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->min_distance = min_distance;
    packet->max_distance = max_distance;
    packet->current_distance = current_distance;
    packet->type = type;
    packet->id = id;
    packet->orientation = orientation;
    packet->covariance = covariance;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DISTANCE_SENSOR, (const char *)packet, MAVLINK_MSG_ID_DISTANCE_SENSOR_MIN_LEN, MAVLINK_MSG_ID_DISTANCE_SENSOR_LEN, MAVLINK_MSG_ID_DISTANCE_SENSOR_CRC);
#endif
}
#endif

#endif

// MESSAGE DISTANCE_SENSOR UNPACKING


/**
 * @brief Get field time_boot_ms from distance_sensor message
 *
 * @return Time since system boot
 */
static inline uint32_t mavlink_msg_distance_sensor_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field min_distance from distance_sensor message
 *
 * @return Minimum distance the sensor can measure in centimeters
 */
static inline uint16_t mavlink_msg_distance_sensor_get_min_distance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field max_distance from distance_sensor message
 *
 * @return Maximum distance the sensor can measure in centimeters
 */
static inline uint16_t mavlink_msg_distance_sensor_get_max_distance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field current_distance from distance_sensor message
 *
 * @return Current distance reading
 */
static inline uint16_t mavlink_msg_distance_sensor_get_current_distance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field type from distance_sensor message
 *
 * @return Type from MAV_DISTANCE_SENSOR enum.
 */
static inline uint8_t mavlink_msg_distance_sensor_get_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field id from distance_sensor message
 *
 * @return Onboard ID of the sensor
 */
static inline uint8_t mavlink_msg_distance_sensor_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field orientation from distance_sensor message
 *
 * @return Direction the sensor faces from MAV_SENSOR_ORIENTATION enum.
 */
static inline uint8_t mavlink_msg_distance_sensor_get_orientation(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field covariance from distance_sensor message
 *
 * @return Measurement covariance in centimeters, 0 for unknown / invalid readings
 */
static inline uint8_t mavlink_msg_distance_sensor_get_covariance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Decode a distance_sensor message into a struct
 *
 * @param msg The message to decode
 * @param distance_sensor C-struct to decode the message contents into
 */
static inline void mavlink_msg_distance_sensor_decode(const mavlink_message_t* msg, mavlink_distance_sensor_t* distance_sensor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    distance_sensor->time_boot_ms = mavlink_msg_distance_sensor_get_time_boot_ms(msg);
    distance_sensor->min_distance = mavlink_msg_distance_sensor_get_min_distance(msg);
    distance_sensor->max_distance = mavlink_msg_distance_sensor_get_max_distance(msg);
    distance_sensor->current_distance = mavlink_msg_distance_sensor_get_current_distance(msg);
    distance_sensor->type = mavlink_msg_distance_sensor_get_type(msg);
    distance_sensor->id = mavlink_msg_distance_sensor_get_id(msg);
    distance_sensor->orientation = mavlink_msg_distance_sensor_get_orientation(msg);
    distance_sensor->covariance = mavlink_msg_distance_sensor_get_covariance(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DISTANCE_SENSOR_LEN? msg->len : MAVLINK_MSG_ID_DISTANCE_SENSOR_LEN;
        memset(distance_sensor, 0, MAVLINK_MSG_ID_DISTANCE_SENSOR_LEN);
    memcpy(distance_sensor, _MAV_PAYLOAD(msg), len);
#endif
}
