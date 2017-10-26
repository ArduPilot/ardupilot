#pragma once
// MESSAGE HOME_POSITION PACKING

#define MAVLINK_MSG_ID_HOME_POSITION 242

MAVPACKED(
typedef struct __mavlink_home_position_t {
 int32_t latitude; /*< Latitude (WGS84), in degrees * 1E7*/
 int32_t longitude; /*< Longitude (WGS84, in degrees * 1E7*/
 int32_t altitude; /*< Altitude (AMSL), in meters * 1000 (positive for up)*/
 float x; /*< Local X position of this position in the local coordinate frame*/
 float y; /*< Local Y position of this position in the local coordinate frame*/
 float z; /*< Local Z position of this position in the local coordinate frame*/
 float q[4]; /*< World to surface normal and heading transformation of the takeoff position. Used to indicate the heading and slope of the ground*/
 float approach_x; /*< Local X position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.*/
 float approach_y; /*< Local Y position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.*/
 float approach_z; /*< Local Z position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.*/
}) mavlink_home_position_t;

#define MAVLINK_MSG_ID_HOME_POSITION_LEN 52
#define MAVLINK_MSG_ID_HOME_POSITION_MIN_LEN 52
#define MAVLINK_MSG_ID_242_LEN 52
#define MAVLINK_MSG_ID_242_MIN_LEN 52

#define MAVLINK_MSG_ID_HOME_POSITION_CRC 104
#define MAVLINK_MSG_ID_242_CRC 104

#define MAVLINK_MSG_HOME_POSITION_FIELD_Q_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_HOME_POSITION { \
    242, \
    "HOME_POSITION", \
    10, \
    {  { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_home_position_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_home_position_t, longitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_home_position_t, altitude) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_home_position_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_home_position_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_home_position_t, z) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 4, 24, offsetof(mavlink_home_position_t, q) }, \
         { "approach_x", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_home_position_t, approach_x) }, \
         { "approach_y", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_home_position_t, approach_y) }, \
         { "approach_z", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_home_position_t, approach_z) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_HOME_POSITION { \
    "HOME_POSITION", \
    10, \
    {  { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_home_position_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_home_position_t, longitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_home_position_t, altitude) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_home_position_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_home_position_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_home_position_t, z) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 4, 24, offsetof(mavlink_home_position_t, q) }, \
         { "approach_x", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_home_position_t, approach_x) }, \
         { "approach_y", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_home_position_t, approach_y) }, \
         { "approach_z", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_home_position_t, approach_z) }, \
         } \
}
#endif

/**
 * @brief Pack a home_position message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param latitude Latitude (WGS84), in degrees * 1E7
 * @param longitude Longitude (WGS84, in degrees * 1E7
 * @param altitude Altitude (AMSL), in meters * 1000 (positive for up)
 * @param x Local X position of this position in the local coordinate frame
 * @param y Local Y position of this position in the local coordinate frame
 * @param z Local Z position of this position in the local coordinate frame
 * @param q World to surface normal and heading transformation of the takeoff position. Used to indicate the heading and slope of the ground
 * @param approach_x Local X position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
 * @param approach_y Local Y position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
 * @param approach_z Local Z position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_home_position_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t latitude, int32_t longitude, int32_t altitude, float x, float y, float z, const float *q, float approach_x, float approach_y, float approach_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HOME_POSITION_LEN];
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_int32_t(buf, 8, altitude);
    _mav_put_float(buf, 12, x);
    _mav_put_float(buf, 16, y);
    _mav_put_float(buf, 20, z);
    _mav_put_float(buf, 40, approach_x);
    _mav_put_float(buf, 44, approach_y);
    _mav_put_float(buf, 48, approach_z);
    _mav_put_float_array(buf, 24, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HOME_POSITION_LEN);
#else
    mavlink_home_position_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.approach_x = approach_x;
    packet.approach_y = approach_y;
    packet.approach_z = approach_z;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HOME_POSITION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HOME_POSITION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HOME_POSITION_MIN_LEN, MAVLINK_MSG_ID_HOME_POSITION_LEN, MAVLINK_MSG_ID_HOME_POSITION_CRC);
}

/**
 * @brief Pack a home_position message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param latitude Latitude (WGS84), in degrees * 1E7
 * @param longitude Longitude (WGS84, in degrees * 1E7
 * @param altitude Altitude (AMSL), in meters * 1000 (positive for up)
 * @param x Local X position of this position in the local coordinate frame
 * @param y Local Y position of this position in the local coordinate frame
 * @param z Local Z position of this position in the local coordinate frame
 * @param q World to surface normal and heading transformation of the takeoff position. Used to indicate the heading and slope of the ground
 * @param approach_x Local X position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
 * @param approach_y Local Y position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
 * @param approach_z Local Z position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_home_position_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t latitude,int32_t longitude,int32_t altitude,float x,float y,float z,const float *q,float approach_x,float approach_y,float approach_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HOME_POSITION_LEN];
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_int32_t(buf, 8, altitude);
    _mav_put_float(buf, 12, x);
    _mav_put_float(buf, 16, y);
    _mav_put_float(buf, 20, z);
    _mav_put_float(buf, 40, approach_x);
    _mav_put_float(buf, 44, approach_y);
    _mav_put_float(buf, 48, approach_z);
    _mav_put_float_array(buf, 24, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HOME_POSITION_LEN);
#else
    mavlink_home_position_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.approach_x = approach_x;
    packet.approach_y = approach_y;
    packet.approach_z = approach_z;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HOME_POSITION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HOME_POSITION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HOME_POSITION_MIN_LEN, MAVLINK_MSG_ID_HOME_POSITION_LEN, MAVLINK_MSG_ID_HOME_POSITION_CRC);
}

/**
 * @brief Encode a home_position struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param home_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_home_position_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_home_position_t* home_position)
{
    return mavlink_msg_home_position_pack(system_id, component_id, msg, home_position->latitude, home_position->longitude, home_position->altitude, home_position->x, home_position->y, home_position->z, home_position->q, home_position->approach_x, home_position->approach_y, home_position->approach_z);
}

/**
 * @brief Encode a home_position struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param home_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_home_position_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_home_position_t* home_position)
{
    return mavlink_msg_home_position_pack_chan(system_id, component_id, chan, msg, home_position->latitude, home_position->longitude, home_position->altitude, home_position->x, home_position->y, home_position->z, home_position->q, home_position->approach_x, home_position->approach_y, home_position->approach_z);
}

/**
 * @brief Send a home_position message
 * @param chan MAVLink channel to send the message
 *
 * @param latitude Latitude (WGS84), in degrees * 1E7
 * @param longitude Longitude (WGS84, in degrees * 1E7
 * @param altitude Altitude (AMSL), in meters * 1000 (positive for up)
 * @param x Local X position of this position in the local coordinate frame
 * @param y Local Y position of this position in the local coordinate frame
 * @param z Local Z position of this position in the local coordinate frame
 * @param q World to surface normal and heading transformation of the takeoff position. Used to indicate the heading and slope of the ground
 * @param approach_x Local X position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
 * @param approach_y Local Y position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
 * @param approach_z Local Z position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_home_position_send(mavlink_channel_t chan, int32_t latitude, int32_t longitude, int32_t altitude, float x, float y, float z, const float *q, float approach_x, float approach_y, float approach_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HOME_POSITION_LEN];
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_int32_t(buf, 8, altitude);
    _mav_put_float(buf, 12, x);
    _mav_put_float(buf, 16, y);
    _mav_put_float(buf, 20, z);
    _mav_put_float(buf, 40, approach_x);
    _mav_put_float(buf, 44, approach_y);
    _mav_put_float(buf, 48, approach_z);
    _mav_put_float_array(buf, 24, q, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HOME_POSITION, buf, MAVLINK_MSG_ID_HOME_POSITION_MIN_LEN, MAVLINK_MSG_ID_HOME_POSITION_LEN, MAVLINK_MSG_ID_HOME_POSITION_CRC);
#else
    mavlink_home_position_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.approach_x = approach_x;
    packet.approach_y = approach_y;
    packet.approach_z = approach_z;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HOME_POSITION, (const char *)&packet, MAVLINK_MSG_ID_HOME_POSITION_MIN_LEN, MAVLINK_MSG_ID_HOME_POSITION_LEN, MAVLINK_MSG_ID_HOME_POSITION_CRC);
#endif
}

/**
 * @brief Send a home_position message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_home_position_send_struct(mavlink_channel_t chan, const mavlink_home_position_t* home_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_home_position_send(chan, home_position->latitude, home_position->longitude, home_position->altitude, home_position->x, home_position->y, home_position->z, home_position->q, home_position->approach_x, home_position->approach_y, home_position->approach_z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HOME_POSITION, (const char *)home_position, MAVLINK_MSG_ID_HOME_POSITION_MIN_LEN, MAVLINK_MSG_ID_HOME_POSITION_LEN, MAVLINK_MSG_ID_HOME_POSITION_CRC);
#endif
}

#if MAVLINK_MSG_ID_HOME_POSITION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_home_position_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t latitude, int32_t longitude, int32_t altitude, float x, float y, float z, const float *q, float approach_x, float approach_y, float approach_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_int32_t(buf, 8, altitude);
    _mav_put_float(buf, 12, x);
    _mav_put_float(buf, 16, y);
    _mav_put_float(buf, 20, z);
    _mav_put_float(buf, 40, approach_x);
    _mav_put_float(buf, 44, approach_y);
    _mav_put_float(buf, 48, approach_z);
    _mav_put_float_array(buf, 24, q, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HOME_POSITION, buf, MAVLINK_MSG_ID_HOME_POSITION_MIN_LEN, MAVLINK_MSG_ID_HOME_POSITION_LEN, MAVLINK_MSG_ID_HOME_POSITION_CRC);
#else
    mavlink_home_position_t *packet = (mavlink_home_position_t *)msgbuf;
    packet->latitude = latitude;
    packet->longitude = longitude;
    packet->altitude = altitude;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->approach_x = approach_x;
    packet->approach_y = approach_y;
    packet->approach_z = approach_z;
    mav_array_memcpy(packet->q, q, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HOME_POSITION, (const char *)packet, MAVLINK_MSG_ID_HOME_POSITION_MIN_LEN, MAVLINK_MSG_ID_HOME_POSITION_LEN, MAVLINK_MSG_ID_HOME_POSITION_CRC);
#endif
}
#endif

#endif

// MESSAGE HOME_POSITION UNPACKING


/**
 * @brief Get field latitude from home_position message
 *
 * @return Latitude (WGS84), in degrees * 1E7
 */
static inline int32_t mavlink_msg_home_position_get_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field longitude from home_position message
 *
 * @return Longitude (WGS84, in degrees * 1E7
 */
static inline int32_t mavlink_msg_home_position_get_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field altitude from home_position message
 *
 * @return Altitude (AMSL), in meters * 1000 (positive for up)
 */
static inline int32_t mavlink_msg_home_position_get_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field x from home_position message
 *
 * @return Local X position of this position in the local coordinate frame
 */
static inline float mavlink_msg_home_position_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field y from home_position message
 *
 * @return Local Y position of this position in the local coordinate frame
 */
static inline float mavlink_msg_home_position_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field z from home_position message
 *
 * @return Local Z position of this position in the local coordinate frame
 */
static inline float mavlink_msg_home_position_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field q from home_position message
 *
 * @return World to surface normal and heading transformation of the takeoff position. Used to indicate the heading and slope of the ground
 */
static inline uint16_t mavlink_msg_home_position_get_q(const mavlink_message_t* msg, float *q)
{
    return _MAV_RETURN_float_array(msg, q, 4,  24);
}

/**
 * @brief Get field approach_x from home_position message
 *
 * @return Local X position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
 */
static inline float mavlink_msg_home_position_get_approach_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field approach_y from home_position message
 *
 * @return Local Y position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
 */
static inline float mavlink_msg_home_position_get_approach_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field approach_z from home_position message
 *
 * @return Local Z position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
 */
static inline float mavlink_msg_home_position_get_approach_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Decode a home_position message into a struct
 *
 * @param msg The message to decode
 * @param home_position C-struct to decode the message contents into
 */
static inline void mavlink_msg_home_position_decode(const mavlink_message_t* msg, mavlink_home_position_t* home_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    home_position->latitude = mavlink_msg_home_position_get_latitude(msg);
    home_position->longitude = mavlink_msg_home_position_get_longitude(msg);
    home_position->altitude = mavlink_msg_home_position_get_altitude(msg);
    home_position->x = mavlink_msg_home_position_get_x(msg);
    home_position->y = mavlink_msg_home_position_get_y(msg);
    home_position->z = mavlink_msg_home_position_get_z(msg);
    mavlink_msg_home_position_get_q(msg, home_position->q);
    home_position->approach_x = mavlink_msg_home_position_get_approach_x(msg);
    home_position->approach_y = mavlink_msg_home_position_get_approach_y(msg);
    home_position->approach_z = mavlink_msg_home_position_get_approach_z(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_HOME_POSITION_LEN? msg->len : MAVLINK_MSG_ID_HOME_POSITION_LEN;
        memset(home_position, 0, MAVLINK_MSG_ID_HOME_POSITION_LEN);
    memcpy(home_position, _MAV_PAYLOAD(msg), len);
#endif
}
