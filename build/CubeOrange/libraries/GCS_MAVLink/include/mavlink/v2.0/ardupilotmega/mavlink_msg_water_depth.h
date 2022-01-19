#pragma once
// MESSAGE WATER_DEPTH PACKING

#define MAVLINK_MSG_ID_WATER_DEPTH 11038


typedef struct __mavlink_water_depth_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot)*/
 int32_t lat; /*< [degE7] Latitude*/
 int32_t lng; /*< [degE7] Longitude*/
 float alt; /*< [m] Altitude (MSL) of vehicle*/
 float roll; /*< [rad] Roll angle*/
 float pitch; /*< [rad] Pitch angle*/
 float yaw; /*< [rad] Yaw angle*/
 float distance; /*< [m] Distance (uncorrected)*/
 float temperature; /*< [degC] Water temperature*/
 uint8_t id; /*<  Onboard ID of the sensor*/
 uint8_t healthy; /*<  Sensor data healthy (0=unhealthy, 1=healthy)*/
} mavlink_water_depth_t;

#define MAVLINK_MSG_ID_WATER_DEPTH_LEN 38
#define MAVLINK_MSG_ID_WATER_DEPTH_MIN_LEN 38
#define MAVLINK_MSG_ID_11038_LEN 38
#define MAVLINK_MSG_ID_11038_MIN_LEN 38

#define MAVLINK_MSG_ID_WATER_DEPTH_CRC 47
#define MAVLINK_MSG_ID_11038_CRC 47



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_WATER_DEPTH { \
    11038, \
    "WATER_DEPTH", \
    11, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_water_depth_t, time_boot_ms) }, \
         { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_water_depth_t, id) }, \
         { "healthy", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_water_depth_t, healthy) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_water_depth_t, lat) }, \
         { "lng", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_water_depth_t, lng) }, \
         { "alt", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_water_depth_t, alt) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_water_depth_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_water_depth_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_water_depth_t, yaw) }, \
         { "distance", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_water_depth_t, distance) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_water_depth_t, temperature) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_WATER_DEPTH { \
    "WATER_DEPTH", \
    11, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_water_depth_t, time_boot_ms) }, \
         { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_water_depth_t, id) }, \
         { "healthy", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_water_depth_t, healthy) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_water_depth_t, lat) }, \
         { "lng", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_water_depth_t, lng) }, \
         { "alt", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_water_depth_t, alt) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_water_depth_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_water_depth_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_water_depth_t, yaw) }, \
         { "distance", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_water_depth_t, distance) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_water_depth_t, temperature) }, \
         } \
}
#endif

/**
 * @brief Pack a water_depth message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot)
 * @param id  Onboard ID of the sensor
 * @param healthy  Sensor data healthy (0=unhealthy, 1=healthy)
 * @param lat [degE7] Latitude
 * @param lng [degE7] Longitude
 * @param alt [m] Altitude (MSL) of vehicle
 * @param roll [rad] Roll angle
 * @param pitch [rad] Pitch angle
 * @param yaw [rad] Yaw angle
 * @param distance [m] Distance (uncorrected)
 * @param temperature [degC] Water temperature
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_water_depth_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint8_t id, uint8_t healthy, int32_t lat, int32_t lng, float alt, float roll, float pitch, float yaw, float distance, float temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WATER_DEPTH_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lng);
    _mav_put_float(buf, 12, alt);
    _mav_put_float(buf, 16, roll);
    _mav_put_float(buf, 20, pitch);
    _mav_put_float(buf, 24, yaw);
    _mav_put_float(buf, 28, distance);
    _mav_put_float(buf, 32, temperature);
    _mav_put_uint8_t(buf, 36, id);
    _mav_put_uint8_t(buf, 37, healthy);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WATER_DEPTH_LEN);
#else
    mavlink_water_depth_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.lat = lat;
    packet.lng = lng;
    packet.alt = alt;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.distance = distance;
    packet.temperature = temperature;
    packet.id = id;
    packet.healthy = healthy;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WATER_DEPTH_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WATER_DEPTH;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_WATER_DEPTH_MIN_LEN, MAVLINK_MSG_ID_WATER_DEPTH_LEN, MAVLINK_MSG_ID_WATER_DEPTH_CRC);
}

/**
 * @brief Pack a water_depth message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot)
 * @param id  Onboard ID of the sensor
 * @param healthy  Sensor data healthy (0=unhealthy, 1=healthy)
 * @param lat [degE7] Latitude
 * @param lng [degE7] Longitude
 * @param alt [m] Altitude (MSL) of vehicle
 * @param roll [rad] Roll angle
 * @param pitch [rad] Pitch angle
 * @param yaw [rad] Yaw angle
 * @param distance [m] Distance (uncorrected)
 * @param temperature [degC] Water temperature
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_water_depth_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,uint8_t id,uint8_t healthy,int32_t lat,int32_t lng,float alt,float roll,float pitch,float yaw,float distance,float temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WATER_DEPTH_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lng);
    _mav_put_float(buf, 12, alt);
    _mav_put_float(buf, 16, roll);
    _mav_put_float(buf, 20, pitch);
    _mav_put_float(buf, 24, yaw);
    _mav_put_float(buf, 28, distance);
    _mav_put_float(buf, 32, temperature);
    _mav_put_uint8_t(buf, 36, id);
    _mav_put_uint8_t(buf, 37, healthy);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WATER_DEPTH_LEN);
#else
    mavlink_water_depth_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.lat = lat;
    packet.lng = lng;
    packet.alt = alt;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.distance = distance;
    packet.temperature = temperature;
    packet.id = id;
    packet.healthy = healthy;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WATER_DEPTH_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WATER_DEPTH;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_WATER_DEPTH_MIN_LEN, MAVLINK_MSG_ID_WATER_DEPTH_LEN, MAVLINK_MSG_ID_WATER_DEPTH_CRC);
}

/**
 * @brief Encode a water_depth struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param water_depth C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_water_depth_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_water_depth_t* water_depth)
{
    return mavlink_msg_water_depth_pack(system_id, component_id, msg, water_depth->time_boot_ms, water_depth->id, water_depth->healthy, water_depth->lat, water_depth->lng, water_depth->alt, water_depth->roll, water_depth->pitch, water_depth->yaw, water_depth->distance, water_depth->temperature);
}

/**
 * @brief Encode a water_depth struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param water_depth C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_water_depth_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_water_depth_t* water_depth)
{
    return mavlink_msg_water_depth_pack_chan(system_id, component_id, chan, msg, water_depth->time_boot_ms, water_depth->id, water_depth->healthy, water_depth->lat, water_depth->lng, water_depth->alt, water_depth->roll, water_depth->pitch, water_depth->yaw, water_depth->distance, water_depth->temperature);
}

/**
 * @brief Send a water_depth message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot)
 * @param id  Onboard ID of the sensor
 * @param healthy  Sensor data healthy (0=unhealthy, 1=healthy)
 * @param lat [degE7] Latitude
 * @param lng [degE7] Longitude
 * @param alt [m] Altitude (MSL) of vehicle
 * @param roll [rad] Roll angle
 * @param pitch [rad] Pitch angle
 * @param yaw [rad] Yaw angle
 * @param distance [m] Distance (uncorrected)
 * @param temperature [degC] Water temperature
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_water_depth_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t id, uint8_t healthy, int32_t lat, int32_t lng, float alt, float roll, float pitch, float yaw, float distance, float temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WATER_DEPTH_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lng);
    _mav_put_float(buf, 12, alt);
    _mav_put_float(buf, 16, roll);
    _mav_put_float(buf, 20, pitch);
    _mav_put_float(buf, 24, yaw);
    _mav_put_float(buf, 28, distance);
    _mav_put_float(buf, 32, temperature);
    _mav_put_uint8_t(buf, 36, id);
    _mav_put_uint8_t(buf, 37, healthy);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WATER_DEPTH, buf, MAVLINK_MSG_ID_WATER_DEPTH_MIN_LEN, MAVLINK_MSG_ID_WATER_DEPTH_LEN, MAVLINK_MSG_ID_WATER_DEPTH_CRC);
#else
    mavlink_water_depth_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.lat = lat;
    packet.lng = lng;
    packet.alt = alt;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.distance = distance;
    packet.temperature = temperature;
    packet.id = id;
    packet.healthy = healthy;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WATER_DEPTH, (const char *)&packet, MAVLINK_MSG_ID_WATER_DEPTH_MIN_LEN, MAVLINK_MSG_ID_WATER_DEPTH_LEN, MAVLINK_MSG_ID_WATER_DEPTH_CRC);
#endif
}

/**
 * @brief Send a water_depth message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_water_depth_send_struct(mavlink_channel_t chan, const mavlink_water_depth_t* water_depth)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_water_depth_send(chan, water_depth->time_boot_ms, water_depth->id, water_depth->healthy, water_depth->lat, water_depth->lng, water_depth->alt, water_depth->roll, water_depth->pitch, water_depth->yaw, water_depth->distance, water_depth->temperature);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WATER_DEPTH, (const char *)water_depth, MAVLINK_MSG_ID_WATER_DEPTH_MIN_LEN, MAVLINK_MSG_ID_WATER_DEPTH_LEN, MAVLINK_MSG_ID_WATER_DEPTH_CRC);
#endif
}

#if MAVLINK_MSG_ID_WATER_DEPTH_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_water_depth_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint8_t id, uint8_t healthy, int32_t lat, int32_t lng, float alt, float roll, float pitch, float yaw, float distance, float temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lng);
    _mav_put_float(buf, 12, alt);
    _mav_put_float(buf, 16, roll);
    _mav_put_float(buf, 20, pitch);
    _mav_put_float(buf, 24, yaw);
    _mav_put_float(buf, 28, distance);
    _mav_put_float(buf, 32, temperature);
    _mav_put_uint8_t(buf, 36, id);
    _mav_put_uint8_t(buf, 37, healthy);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WATER_DEPTH, buf, MAVLINK_MSG_ID_WATER_DEPTH_MIN_LEN, MAVLINK_MSG_ID_WATER_DEPTH_LEN, MAVLINK_MSG_ID_WATER_DEPTH_CRC);
#else
    mavlink_water_depth_t *packet = (mavlink_water_depth_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->lat = lat;
    packet->lng = lng;
    packet->alt = alt;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;
    packet->distance = distance;
    packet->temperature = temperature;
    packet->id = id;
    packet->healthy = healthy;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WATER_DEPTH, (const char *)packet, MAVLINK_MSG_ID_WATER_DEPTH_MIN_LEN, MAVLINK_MSG_ID_WATER_DEPTH_LEN, MAVLINK_MSG_ID_WATER_DEPTH_CRC);
#endif
}
#endif

#endif

// MESSAGE WATER_DEPTH UNPACKING


/**
 * @brief Get field time_boot_ms from water_depth message
 *
 * @return [ms] Timestamp (time since system boot)
 */
static inline uint32_t mavlink_msg_water_depth_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field id from water_depth message
 *
 * @return  Onboard ID of the sensor
 */
static inline uint8_t mavlink_msg_water_depth_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Get field healthy from water_depth message
 *
 * @return  Sensor data healthy (0=unhealthy, 1=healthy)
 */
static inline uint8_t mavlink_msg_water_depth_get_healthy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  37);
}

/**
 * @brief Get field lat from water_depth message
 *
 * @return [degE7] Latitude
 */
static inline int32_t mavlink_msg_water_depth_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field lng from water_depth message
 *
 * @return [degE7] Longitude
 */
static inline int32_t mavlink_msg_water_depth_get_lng(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field alt from water_depth message
 *
 * @return [m] Altitude (MSL) of vehicle
 */
static inline float mavlink_msg_water_depth_get_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field roll from water_depth message
 *
 * @return [rad] Roll angle
 */
static inline float mavlink_msg_water_depth_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field pitch from water_depth message
 *
 * @return [rad] Pitch angle
 */
static inline float mavlink_msg_water_depth_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field yaw from water_depth message
 *
 * @return [rad] Yaw angle
 */
static inline float mavlink_msg_water_depth_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field distance from water_depth message
 *
 * @return [m] Distance (uncorrected)
 */
static inline float mavlink_msg_water_depth_get_distance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field temperature from water_depth message
 *
 * @return [degC] Water temperature
 */
static inline float mavlink_msg_water_depth_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Decode a water_depth message into a struct
 *
 * @param msg The message to decode
 * @param water_depth C-struct to decode the message contents into
 */
static inline void mavlink_msg_water_depth_decode(const mavlink_message_t* msg, mavlink_water_depth_t* water_depth)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    water_depth->time_boot_ms = mavlink_msg_water_depth_get_time_boot_ms(msg);
    water_depth->lat = mavlink_msg_water_depth_get_lat(msg);
    water_depth->lng = mavlink_msg_water_depth_get_lng(msg);
    water_depth->alt = mavlink_msg_water_depth_get_alt(msg);
    water_depth->roll = mavlink_msg_water_depth_get_roll(msg);
    water_depth->pitch = mavlink_msg_water_depth_get_pitch(msg);
    water_depth->yaw = mavlink_msg_water_depth_get_yaw(msg);
    water_depth->distance = mavlink_msg_water_depth_get_distance(msg);
    water_depth->temperature = mavlink_msg_water_depth_get_temperature(msg);
    water_depth->id = mavlink_msg_water_depth_get_id(msg);
    water_depth->healthy = mavlink_msg_water_depth_get_healthy(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_WATER_DEPTH_LEN? msg->len : MAVLINK_MSG_ID_WATER_DEPTH_LEN;
        memset(water_depth, 0, MAVLINK_MSG_ID_WATER_DEPTH_LEN);
    memcpy(water_depth, _MAV_PAYLOAD(msg), len);
#endif
}
