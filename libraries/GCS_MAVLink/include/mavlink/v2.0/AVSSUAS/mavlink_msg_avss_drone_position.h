#pragma once
// MESSAGE AVSS_DRONE_POSITION PACKING

#define MAVLINK_MSG_ID_AVSS_DRONE_POSITION 60051


typedef struct __mavlink_avss_drone_position_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since FC boot).*/
 int32_t lat; /*< [degE7] Latitude, expressed*/
 int32_t lon; /*< [degE7] Longitude, expressed*/
 int32_t alt; /*< [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.*/
 float ground_alt; /*< [m] Altitude above ground, This altitude is measured by a ultrasound, Laser rangefinder or millimeter-wave radar*/
 float barometer_alt; /*< [m] This altitude is measured by a barometer*/
} mavlink_avss_drone_position_t;

#define MAVLINK_MSG_ID_AVSS_DRONE_POSITION_LEN 24
#define MAVLINK_MSG_ID_AVSS_DRONE_POSITION_MIN_LEN 24
#define MAVLINK_MSG_ID_60051_LEN 24
#define MAVLINK_MSG_ID_60051_MIN_LEN 24

#define MAVLINK_MSG_ID_AVSS_DRONE_POSITION_CRC 245
#define MAVLINK_MSG_ID_60051_CRC 245



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AVSS_DRONE_POSITION { \
    60051, \
    "AVSS_DRONE_POSITION", \
    6, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_avss_drone_position_t, time_boot_ms) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_avss_drone_position_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_avss_drone_position_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_avss_drone_position_t, alt) }, \
         { "ground_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_avss_drone_position_t, ground_alt) }, \
         { "barometer_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_avss_drone_position_t, barometer_alt) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AVSS_DRONE_POSITION { \
    "AVSS_DRONE_POSITION", \
    6, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_avss_drone_position_t, time_boot_ms) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_avss_drone_position_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_avss_drone_position_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_avss_drone_position_t, alt) }, \
         { "ground_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_avss_drone_position_t, ground_alt) }, \
         { "barometer_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_avss_drone_position_t, barometer_alt) }, \
         } \
}
#endif

/**
 * @brief Pack a avss_drone_position message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since FC boot).
 * @param lat [degE7] Latitude, expressed
 * @param lon [degE7] Longitude, expressed
 * @param alt [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
 * @param ground_alt [m] Altitude above ground, This altitude is measured by a ultrasound, Laser rangefinder or millimeter-wave radar
 * @param barometer_alt [m] This altitude is measured by a barometer
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_avss_drone_position_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, float ground_alt, float barometer_alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AVSS_DRONE_POSITION_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_int32_t(buf, 12, alt);
    _mav_put_float(buf, 16, ground_alt);
    _mav_put_float(buf, 20, barometer_alt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AVSS_DRONE_POSITION_LEN);
#else
    mavlink_avss_drone_position_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.ground_alt = ground_alt;
    packet.barometer_alt = barometer_alt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AVSS_DRONE_POSITION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AVSS_DRONE_POSITION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AVSS_DRONE_POSITION_MIN_LEN, MAVLINK_MSG_ID_AVSS_DRONE_POSITION_LEN, MAVLINK_MSG_ID_AVSS_DRONE_POSITION_CRC);
}

/**
 * @brief Pack a avss_drone_position message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since FC boot).
 * @param lat [degE7] Latitude, expressed
 * @param lon [degE7] Longitude, expressed
 * @param alt [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
 * @param ground_alt [m] Altitude above ground, This altitude is measured by a ultrasound, Laser rangefinder or millimeter-wave radar
 * @param barometer_alt [m] This altitude is measured by a barometer
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_avss_drone_position_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, float ground_alt, float barometer_alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AVSS_DRONE_POSITION_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_int32_t(buf, 12, alt);
    _mav_put_float(buf, 16, ground_alt);
    _mav_put_float(buf, 20, barometer_alt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AVSS_DRONE_POSITION_LEN);
#else
    mavlink_avss_drone_position_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.ground_alt = ground_alt;
    packet.barometer_alt = barometer_alt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AVSS_DRONE_POSITION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AVSS_DRONE_POSITION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AVSS_DRONE_POSITION_MIN_LEN, MAVLINK_MSG_ID_AVSS_DRONE_POSITION_LEN, MAVLINK_MSG_ID_AVSS_DRONE_POSITION_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AVSS_DRONE_POSITION_MIN_LEN, MAVLINK_MSG_ID_AVSS_DRONE_POSITION_LEN);
#endif
}

/**
 * @brief Pack a avss_drone_position message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since FC boot).
 * @param lat [degE7] Latitude, expressed
 * @param lon [degE7] Longitude, expressed
 * @param alt [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
 * @param ground_alt [m] Altitude above ground, This altitude is measured by a ultrasound, Laser rangefinder or millimeter-wave radar
 * @param barometer_alt [m] This altitude is measured by a barometer
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_avss_drone_position_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,int32_t lat,int32_t lon,int32_t alt,float ground_alt,float barometer_alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AVSS_DRONE_POSITION_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_int32_t(buf, 12, alt);
    _mav_put_float(buf, 16, ground_alt);
    _mav_put_float(buf, 20, barometer_alt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AVSS_DRONE_POSITION_LEN);
#else
    mavlink_avss_drone_position_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.ground_alt = ground_alt;
    packet.barometer_alt = barometer_alt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AVSS_DRONE_POSITION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AVSS_DRONE_POSITION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AVSS_DRONE_POSITION_MIN_LEN, MAVLINK_MSG_ID_AVSS_DRONE_POSITION_LEN, MAVLINK_MSG_ID_AVSS_DRONE_POSITION_CRC);
}

/**
 * @brief Encode a avss_drone_position struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param avss_drone_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_avss_drone_position_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_avss_drone_position_t* avss_drone_position)
{
    return mavlink_msg_avss_drone_position_pack(system_id, component_id, msg, avss_drone_position->time_boot_ms, avss_drone_position->lat, avss_drone_position->lon, avss_drone_position->alt, avss_drone_position->ground_alt, avss_drone_position->barometer_alt);
}

/**
 * @brief Encode a avss_drone_position struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param avss_drone_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_avss_drone_position_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_avss_drone_position_t* avss_drone_position)
{
    return mavlink_msg_avss_drone_position_pack_chan(system_id, component_id, chan, msg, avss_drone_position->time_boot_ms, avss_drone_position->lat, avss_drone_position->lon, avss_drone_position->alt, avss_drone_position->ground_alt, avss_drone_position->barometer_alt);
}

/**
 * @brief Encode a avss_drone_position struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param avss_drone_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_avss_drone_position_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_avss_drone_position_t* avss_drone_position)
{
    return mavlink_msg_avss_drone_position_pack_status(system_id, component_id, _status, msg,  avss_drone_position->time_boot_ms, avss_drone_position->lat, avss_drone_position->lon, avss_drone_position->alt, avss_drone_position->ground_alt, avss_drone_position->barometer_alt);
}

/**
 * @brief Send a avss_drone_position message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since FC boot).
 * @param lat [degE7] Latitude, expressed
 * @param lon [degE7] Longitude, expressed
 * @param alt [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
 * @param ground_alt [m] Altitude above ground, This altitude is measured by a ultrasound, Laser rangefinder or millimeter-wave radar
 * @param barometer_alt [m] This altitude is measured by a barometer
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_avss_drone_position_send(mavlink_channel_t chan, uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, float ground_alt, float barometer_alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AVSS_DRONE_POSITION_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_int32_t(buf, 12, alt);
    _mav_put_float(buf, 16, ground_alt);
    _mav_put_float(buf, 20, barometer_alt);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AVSS_DRONE_POSITION, buf, MAVLINK_MSG_ID_AVSS_DRONE_POSITION_MIN_LEN, MAVLINK_MSG_ID_AVSS_DRONE_POSITION_LEN, MAVLINK_MSG_ID_AVSS_DRONE_POSITION_CRC);
#else
    mavlink_avss_drone_position_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.ground_alt = ground_alt;
    packet.barometer_alt = barometer_alt;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AVSS_DRONE_POSITION, (const char *)&packet, MAVLINK_MSG_ID_AVSS_DRONE_POSITION_MIN_LEN, MAVLINK_MSG_ID_AVSS_DRONE_POSITION_LEN, MAVLINK_MSG_ID_AVSS_DRONE_POSITION_CRC);
#endif
}

/**
 * @brief Send a avss_drone_position message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_avss_drone_position_send_struct(mavlink_channel_t chan, const mavlink_avss_drone_position_t* avss_drone_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_avss_drone_position_send(chan, avss_drone_position->time_boot_ms, avss_drone_position->lat, avss_drone_position->lon, avss_drone_position->alt, avss_drone_position->ground_alt, avss_drone_position->barometer_alt);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AVSS_DRONE_POSITION, (const char *)avss_drone_position, MAVLINK_MSG_ID_AVSS_DRONE_POSITION_MIN_LEN, MAVLINK_MSG_ID_AVSS_DRONE_POSITION_LEN, MAVLINK_MSG_ID_AVSS_DRONE_POSITION_CRC);
#endif
}

#if MAVLINK_MSG_ID_AVSS_DRONE_POSITION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_avss_drone_position_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, float ground_alt, float barometer_alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_int32_t(buf, 12, alt);
    _mav_put_float(buf, 16, ground_alt);
    _mav_put_float(buf, 20, barometer_alt);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AVSS_DRONE_POSITION, buf, MAVLINK_MSG_ID_AVSS_DRONE_POSITION_MIN_LEN, MAVLINK_MSG_ID_AVSS_DRONE_POSITION_LEN, MAVLINK_MSG_ID_AVSS_DRONE_POSITION_CRC);
#else
    mavlink_avss_drone_position_t *packet = (mavlink_avss_drone_position_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt = alt;
    packet->ground_alt = ground_alt;
    packet->barometer_alt = barometer_alt;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AVSS_DRONE_POSITION, (const char *)packet, MAVLINK_MSG_ID_AVSS_DRONE_POSITION_MIN_LEN, MAVLINK_MSG_ID_AVSS_DRONE_POSITION_LEN, MAVLINK_MSG_ID_AVSS_DRONE_POSITION_CRC);
#endif
}
#endif

#endif

// MESSAGE AVSS_DRONE_POSITION UNPACKING


/**
 * @brief Get field time_boot_ms from avss_drone_position message
 *
 * @return [ms] Timestamp (time since FC boot).
 */
static inline uint32_t mavlink_msg_avss_drone_position_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field lat from avss_drone_position message
 *
 * @return [degE7] Latitude, expressed
 */
static inline int32_t mavlink_msg_avss_drone_position_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field lon from avss_drone_position message
 *
 * @return [degE7] Longitude, expressed
 */
static inline int32_t mavlink_msg_avss_drone_position_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field alt from avss_drone_position message
 *
 * @return [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
 */
static inline int32_t mavlink_msg_avss_drone_position_get_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field ground_alt from avss_drone_position message
 *
 * @return [m] Altitude above ground, This altitude is measured by a ultrasound, Laser rangefinder or millimeter-wave radar
 */
static inline float mavlink_msg_avss_drone_position_get_ground_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field barometer_alt from avss_drone_position message
 *
 * @return [m] This altitude is measured by a barometer
 */
static inline float mavlink_msg_avss_drone_position_get_barometer_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a avss_drone_position message into a struct
 *
 * @param msg The message to decode
 * @param avss_drone_position C-struct to decode the message contents into
 */
static inline void mavlink_msg_avss_drone_position_decode(const mavlink_message_t* msg, mavlink_avss_drone_position_t* avss_drone_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    avss_drone_position->time_boot_ms = mavlink_msg_avss_drone_position_get_time_boot_ms(msg);
    avss_drone_position->lat = mavlink_msg_avss_drone_position_get_lat(msg);
    avss_drone_position->lon = mavlink_msg_avss_drone_position_get_lon(msg);
    avss_drone_position->alt = mavlink_msg_avss_drone_position_get_alt(msg);
    avss_drone_position->ground_alt = mavlink_msg_avss_drone_position_get_ground_alt(msg);
    avss_drone_position->barometer_alt = mavlink_msg_avss_drone_position_get_barometer_alt(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AVSS_DRONE_POSITION_LEN? msg->len : MAVLINK_MSG_ID_AVSS_DRONE_POSITION_LEN;
        memset(avss_drone_position, 0, MAVLINK_MSG_ID_AVSS_DRONE_POSITION_LEN);
    memcpy(avss_drone_position, _MAV_PAYLOAD(msg), len);
#endif
}
