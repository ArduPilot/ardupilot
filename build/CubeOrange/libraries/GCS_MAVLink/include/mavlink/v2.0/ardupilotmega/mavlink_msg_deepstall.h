#pragma once
// MESSAGE DEEPSTALL PACKING

#define MAVLINK_MSG_ID_DEEPSTALL 195


typedef struct __mavlink_deepstall_t {
 int32_t landing_lat; /*< [degE7] Landing latitude.*/
 int32_t landing_lon; /*< [degE7] Landing longitude.*/
 int32_t path_lat; /*< [degE7] Final heading start point, latitude.*/
 int32_t path_lon; /*< [degE7] Final heading start point, longitude.*/
 int32_t arc_entry_lat; /*< [degE7] Arc entry point, latitude.*/
 int32_t arc_entry_lon; /*< [degE7] Arc entry point, longitude.*/
 float altitude; /*< [m] Altitude.*/
 float expected_travel_distance; /*< [m] Distance the aircraft expects to travel during the deepstall.*/
 float cross_track_error; /*< [m] Deepstall cross track error (only valid when in DEEPSTALL_STAGE_LAND).*/
 uint8_t stage; /*<  Deepstall stage.*/
} mavlink_deepstall_t;

#define MAVLINK_MSG_ID_DEEPSTALL_LEN 37
#define MAVLINK_MSG_ID_DEEPSTALL_MIN_LEN 37
#define MAVLINK_MSG_ID_195_LEN 37
#define MAVLINK_MSG_ID_195_MIN_LEN 37

#define MAVLINK_MSG_ID_DEEPSTALL_CRC 120
#define MAVLINK_MSG_ID_195_CRC 120



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DEEPSTALL { \
    195, \
    "DEEPSTALL", \
    10, \
    {  { "landing_lat", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_deepstall_t, landing_lat) }, \
         { "landing_lon", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_deepstall_t, landing_lon) }, \
         { "path_lat", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_deepstall_t, path_lat) }, \
         { "path_lon", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_deepstall_t, path_lon) }, \
         { "arc_entry_lat", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_deepstall_t, arc_entry_lat) }, \
         { "arc_entry_lon", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_deepstall_t, arc_entry_lon) }, \
         { "altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_deepstall_t, altitude) }, \
         { "expected_travel_distance", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_deepstall_t, expected_travel_distance) }, \
         { "cross_track_error", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_deepstall_t, cross_track_error) }, \
         { "stage", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_deepstall_t, stage) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DEEPSTALL { \
    "DEEPSTALL", \
    10, \
    {  { "landing_lat", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_deepstall_t, landing_lat) }, \
         { "landing_lon", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_deepstall_t, landing_lon) }, \
         { "path_lat", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_deepstall_t, path_lat) }, \
         { "path_lon", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_deepstall_t, path_lon) }, \
         { "arc_entry_lat", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_deepstall_t, arc_entry_lat) }, \
         { "arc_entry_lon", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_deepstall_t, arc_entry_lon) }, \
         { "altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_deepstall_t, altitude) }, \
         { "expected_travel_distance", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_deepstall_t, expected_travel_distance) }, \
         { "cross_track_error", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_deepstall_t, cross_track_error) }, \
         { "stage", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_deepstall_t, stage) }, \
         } \
}
#endif

/**
 * @brief Pack a deepstall message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param landing_lat [degE7] Landing latitude.
 * @param landing_lon [degE7] Landing longitude.
 * @param path_lat [degE7] Final heading start point, latitude.
 * @param path_lon [degE7] Final heading start point, longitude.
 * @param arc_entry_lat [degE7] Arc entry point, latitude.
 * @param arc_entry_lon [degE7] Arc entry point, longitude.
 * @param altitude [m] Altitude.
 * @param expected_travel_distance [m] Distance the aircraft expects to travel during the deepstall.
 * @param cross_track_error [m] Deepstall cross track error (only valid when in DEEPSTALL_STAGE_LAND).
 * @param stage  Deepstall stage.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_deepstall_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t landing_lat, int32_t landing_lon, int32_t path_lat, int32_t path_lon, int32_t arc_entry_lat, int32_t arc_entry_lon, float altitude, float expected_travel_distance, float cross_track_error, uint8_t stage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DEEPSTALL_LEN];
    _mav_put_int32_t(buf, 0, landing_lat);
    _mav_put_int32_t(buf, 4, landing_lon);
    _mav_put_int32_t(buf, 8, path_lat);
    _mav_put_int32_t(buf, 12, path_lon);
    _mav_put_int32_t(buf, 16, arc_entry_lat);
    _mav_put_int32_t(buf, 20, arc_entry_lon);
    _mav_put_float(buf, 24, altitude);
    _mav_put_float(buf, 28, expected_travel_distance);
    _mav_put_float(buf, 32, cross_track_error);
    _mav_put_uint8_t(buf, 36, stage);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DEEPSTALL_LEN);
#else
    mavlink_deepstall_t packet;
    packet.landing_lat = landing_lat;
    packet.landing_lon = landing_lon;
    packet.path_lat = path_lat;
    packet.path_lon = path_lon;
    packet.arc_entry_lat = arc_entry_lat;
    packet.arc_entry_lon = arc_entry_lon;
    packet.altitude = altitude;
    packet.expected_travel_distance = expected_travel_distance;
    packet.cross_track_error = cross_track_error;
    packet.stage = stage;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DEEPSTALL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DEEPSTALL;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DEEPSTALL_MIN_LEN, MAVLINK_MSG_ID_DEEPSTALL_LEN, MAVLINK_MSG_ID_DEEPSTALL_CRC);
}

/**
 * @brief Pack a deepstall message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param landing_lat [degE7] Landing latitude.
 * @param landing_lon [degE7] Landing longitude.
 * @param path_lat [degE7] Final heading start point, latitude.
 * @param path_lon [degE7] Final heading start point, longitude.
 * @param arc_entry_lat [degE7] Arc entry point, latitude.
 * @param arc_entry_lon [degE7] Arc entry point, longitude.
 * @param altitude [m] Altitude.
 * @param expected_travel_distance [m] Distance the aircraft expects to travel during the deepstall.
 * @param cross_track_error [m] Deepstall cross track error (only valid when in DEEPSTALL_STAGE_LAND).
 * @param stage  Deepstall stage.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_deepstall_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t landing_lat,int32_t landing_lon,int32_t path_lat,int32_t path_lon,int32_t arc_entry_lat,int32_t arc_entry_lon,float altitude,float expected_travel_distance,float cross_track_error,uint8_t stage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DEEPSTALL_LEN];
    _mav_put_int32_t(buf, 0, landing_lat);
    _mav_put_int32_t(buf, 4, landing_lon);
    _mav_put_int32_t(buf, 8, path_lat);
    _mav_put_int32_t(buf, 12, path_lon);
    _mav_put_int32_t(buf, 16, arc_entry_lat);
    _mav_put_int32_t(buf, 20, arc_entry_lon);
    _mav_put_float(buf, 24, altitude);
    _mav_put_float(buf, 28, expected_travel_distance);
    _mav_put_float(buf, 32, cross_track_error);
    _mav_put_uint8_t(buf, 36, stage);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DEEPSTALL_LEN);
#else
    mavlink_deepstall_t packet;
    packet.landing_lat = landing_lat;
    packet.landing_lon = landing_lon;
    packet.path_lat = path_lat;
    packet.path_lon = path_lon;
    packet.arc_entry_lat = arc_entry_lat;
    packet.arc_entry_lon = arc_entry_lon;
    packet.altitude = altitude;
    packet.expected_travel_distance = expected_travel_distance;
    packet.cross_track_error = cross_track_error;
    packet.stage = stage;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DEEPSTALL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DEEPSTALL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DEEPSTALL_MIN_LEN, MAVLINK_MSG_ID_DEEPSTALL_LEN, MAVLINK_MSG_ID_DEEPSTALL_CRC);
}

/**
 * @brief Encode a deepstall struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param deepstall C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_deepstall_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_deepstall_t* deepstall)
{
    return mavlink_msg_deepstall_pack(system_id, component_id, msg, deepstall->landing_lat, deepstall->landing_lon, deepstall->path_lat, deepstall->path_lon, deepstall->arc_entry_lat, deepstall->arc_entry_lon, deepstall->altitude, deepstall->expected_travel_distance, deepstall->cross_track_error, deepstall->stage);
}

/**
 * @brief Encode a deepstall struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param deepstall C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_deepstall_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_deepstall_t* deepstall)
{
    return mavlink_msg_deepstall_pack_chan(system_id, component_id, chan, msg, deepstall->landing_lat, deepstall->landing_lon, deepstall->path_lat, deepstall->path_lon, deepstall->arc_entry_lat, deepstall->arc_entry_lon, deepstall->altitude, deepstall->expected_travel_distance, deepstall->cross_track_error, deepstall->stage);
}

/**
 * @brief Send a deepstall message
 * @param chan MAVLink channel to send the message
 *
 * @param landing_lat [degE7] Landing latitude.
 * @param landing_lon [degE7] Landing longitude.
 * @param path_lat [degE7] Final heading start point, latitude.
 * @param path_lon [degE7] Final heading start point, longitude.
 * @param arc_entry_lat [degE7] Arc entry point, latitude.
 * @param arc_entry_lon [degE7] Arc entry point, longitude.
 * @param altitude [m] Altitude.
 * @param expected_travel_distance [m] Distance the aircraft expects to travel during the deepstall.
 * @param cross_track_error [m] Deepstall cross track error (only valid when in DEEPSTALL_STAGE_LAND).
 * @param stage  Deepstall stage.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_deepstall_send(mavlink_channel_t chan, int32_t landing_lat, int32_t landing_lon, int32_t path_lat, int32_t path_lon, int32_t arc_entry_lat, int32_t arc_entry_lon, float altitude, float expected_travel_distance, float cross_track_error, uint8_t stage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DEEPSTALL_LEN];
    _mav_put_int32_t(buf, 0, landing_lat);
    _mav_put_int32_t(buf, 4, landing_lon);
    _mav_put_int32_t(buf, 8, path_lat);
    _mav_put_int32_t(buf, 12, path_lon);
    _mav_put_int32_t(buf, 16, arc_entry_lat);
    _mav_put_int32_t(buf, 20, arc_entry_lon);
    _mav_put_float(buf, 24, altitude);
    _mav_put_float(buf, 28, expected_travel_distance);
    _mav_put_float(buf, 32, cross_track_error);
    _mav_put_uint8_t(buf, 36, stage);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEEPSTALL, buf, MAVLINK_MSG_ID_DEEPSTALL_MIN_LEN, MAVLINK_MSG_ID_DEEPSTALL_LEN, MAVLINK_MSG_ID_DEEPSTALL_CRC);
#else
    mavlink_deepstall_t packet;
    packet.landing_lat = landing_lat;
    packet.landing_lon = landing_lon;
    packet.path_lat = path_lat;
    packet.path_lon = path_lon;
    packet.arc_entry_lat = arc_entry_lat;
    packet.arc_entry_lon = arc_entry_lon;
    packet.altitude = altitude;
    packet.expected_travel_distance = expected_travel_distance;
    packet.cross_track_error = cross_track_error;
    packet.stage = stage;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEEPSTALL, (const char *)&packet, MAVLINK_MSG_ID_DEEPSTALL_MIN_LEN, MAVLINK_MSG_ID_DEEPSTALL_LEN, MAVLINK_MSG_ID_DEEPSTALL_CRC);
#endif
}

/**
 * @brief Send a deepstall message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_deepstall_send_struct(mavlink_channel_t chan, const mavlink_deepstall_t* deepstall)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_deepstall_send(chan, deepstall->landing_lat, deepstall->landing_lon, deepstall->path_lat, deepstall->path_lon, deepstall->arc_entry_lat, deepstall->arc_entry_lon, deepstall->altitude, deepstall->expected_travel_distance, deepstall->cross_track_error, deepstall->stage);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEEPSTALL, (const char *)deepstall, MAVLINK_MSG_ID_DEEPSTALL_MIN_LEN, MAVLINK_MSG_ID_DEEPSTALL_LEN, MAVLINK_MSG_ID_DEEPSTALL_CRC);
#endif
}

#if MAVLINK_MSG_ID_DEEPSTALL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_deepstall_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t landing_lat, int32_t landing_lon, int32_t path_lat, int32_t path_lon, int32_t arc_entry_lat, int32_t arc_entry_lon, float altitude, float expected_travel_distance, float cross_track_error, uint8_t stage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, landing_lat);
    _mav_put_int32_t(buf, 4, landing_lon);
    _mav_put_int32_t(buf, 8, path_lat);
    _mav_put_int32_t(buf, 12, path_lon);
    _mav_put_int32_t(buf, 16, arc_entry_lat);
    _mav_put_int32_t(buf, 20, arc_entry_lon);
    _mav_put_float(buf, 24, altitude);
    _mav_put_float(buf, 28, expected_travel_distance);
    _mav_put_float(buf, 32, cross_track_error);
    _mav_put_uint8_t(buf, 36, stage);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEEPSTALL, buf, MAVLINK_MSG_ID_DEEPSTALL_MIN_LEN, MAVLINK_MSG_ID_DEEPSTALL_LEN, MAVLINK_MSG_ID_DEEPSTALL_CRC);
#else
    mavlink_deepstall_t *packet = (mavlink_deepstall_t *)msgbuf;
    packet->landing_lat = landing_lat;
    packet->landing_lon = landing_lon;
    packet->path_lat = path_lat;
    packet->path_lon = path_lon;
    packet->arc_entry_lat = arc_entry_lat;
    packet->arc_entry_lon = arc_entry_lon;
    packet->altitude = altitude;
    packet->expected_travel_distance = expected_travel_distance;
    packet->cross_track_error = cross_track_error;
    packet->stage = stage;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEEPSTALL, (const char *)packet, MAVLINK_MSG_ID_DEEPSTALL_MIN_LEN, MAVLINK_MSG_ID_DEEPSTALL_LEN, MAVLINK_MSG_ID_DEEPSTALL_CRC);
#endif
}
#endif

#endif

// MESSAGE DEEPSTALL UNPACKING


/**
 * @brief Get field landing_lat from deepstall message
 *
 * @return [degE7] Landing latitude.
 */
static inline int32_t mavlink_msg_deepstall_get_landing_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field landing_lon from deepstall message
 *
 * @return [degE7] Landing longitude.
 */
static inline int32_t mavlink_msg_deepstall_get_landing_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field path_lat from deepstall message
 *
 * @return [degE7] Final heading start point, latitude.
 */
static inline int32_t mavlink_msg_deepstall_get_path_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field path_lon from deepstall message
 *
 * @return [degE7] Final heading start point, longitude.
 */
static inline int32_t mavlink_msg_deepstall_get_path_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field arc_entry_lat from deepstall message
 *
 * @return [degE7] Arc entry point, latitude.
 */
static inline int32_t mavlink_msg_deepstall_get_arc_entry_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field arc_entry_lon from deepstall message
 *
 * @return [degE7] Arc entry point, longitude.
 */
static inline int32_t mavlink_msg_deepstall_get_arc_entry_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  20);
}

/**
 * @brief Get field altitude from deepstall message
 *
 * @return [m] Altitude.
 */
static inline float mavlink_msg_deepstall_get_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field expected_travel_distance from deepstall message
 *
 * @return [m] Distance the aircraft expects to travel during the deepstall.
 */
static inline float mavlink_msg_deepstall_get_expected_travel_distance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field cross_track_error from deepstall message
 *
 * @return [m] Deepstall cross track error (only valid when in DEEPSTALL_STAGE_LAND).
 */
static inline float mavlink_msg_deepstall_get_cross_track_error(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field stage from deepstall message
 *
 * @return  Deepstall stage.
 */
static inline uint8_t mavlink_msg_deepstall_get_stage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Decode a deepstall message into a struct
 *
 * @param msg The message to decode
 * @param deepstall C-struct to decode the message contents into
 */
static inline void mavlink_msg_deepstall_decode(const mavlink_message_t* msg, mavlink_deepstall_t* deepstall)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    deepstall->landing_lat = mavlink_msg_deepstall_get_landing_lat(msg);
    deepstall->landing_lon = mavlink_msg_deepstall_get_landing_lon(msg);
    deepstall->path_lat = mavlink_msg_deepstall_get_path_lat(msg);
    deepstall->path_lon = mavlink_msg_deepstall_get_path_lon(msg);
    deepstall->arc_entry_lat = mavlink_msg_deepstall_get_arc_entry_lat(msg);
    deepstall->arc_entry_lon = mavlink_msg_deepstall_get_arc_entry_lon(msg);
    deepstall->altitude = mavlink_msg_deepstall_get_altitude(msg);
    deepstall->expected_travel_distance = mavlink_msg_deepstall_get_expected_travel_distance(msg);
    deepstall->cross_track_error = mavlink_msg_deepstall_get_cross_track_error(msg);
    deepstall->stage = mavlink_msg_deepstall_get_stage(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DEEPSTALL_LEN? msg->len : MAVLINK_MSG_ID_DEEPSTALL_LEN;
        memset(deepstall, 0, MAVLINK_MSG_ID_DEEPSTALL_LEN);
    memcpy(deepstall, _MAV_PAYLOAD(msg), len);
#endif
}
