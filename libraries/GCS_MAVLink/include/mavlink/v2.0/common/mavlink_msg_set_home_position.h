#pragma once
// MESSAGE SET_HOME_POSITION PACKING

#define MAVLINK_MSG_ID_SET_HOME_POSITION 243

MAVPACKED(
typedef struct __mavlink_set_home_position_t {
 int32_t latitude; /*< [degE7] Latitude (WGS84)*/
 int32_t longitude; /*< [degE7] Longitude (WGS84)*/
 int32_t altitude; /*< [mm] Altitude (MSL). Positive for up.*/
 float x; /*< [m] Local X position of this position in the local coordinate frame*/
 float y; /*< [m] Local Y position of this position in the local coordinate frame*/
 float z; /*< [m] Local Z position of this position in the local coordinate frame*/
 float q[4]; /*<  World to surface normal and heading transformation of the takeoff position. Used to indicate the heading and slope of the ground*/
 float approach_x; /*< [m] Local X position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.*/
 float approach_y; /*< [m] Local Y position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.*/
 float approach_z; /*< [m] Local Z position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.*/
 uint8_t target_system; /*<  System ID.*/
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
}) mavlink_set_home_position_t;

#define MAVLINK_MSG_ID_SET_HOME_POSITION_LEN 61
#define MAVLINK_MSG_ID_SET_HOME_POSITION_MIN_LEN 53
#define MAVLINK_MSG_ID_243_LEN 61
#define MAVLINK_MSG_ID_243_MIN_LEN 53

#define MAVLINK_MSG_ID_SET_HOME_POSITION_CRC 85
#define MAVLINK_MSG_ID_243_CRC 85

#define MAVLINK_MSG_SET_HOME_POSITION_FIELD_Q_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SET_HOME_POSITION { \
    243, \
    "SET_HOME_POSITION", \
    12, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 52, offsetof(mavlink_set_home_position_t, target_system) }, \
         { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_set_home_position_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_set_home_position_t, longitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_set_home_position_t, altitude) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_set_home_position_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_set_home_position_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_set_home_position_t, z) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 4, 24, offsetof(mavlink_set_home_position_t, q) }, \
         { "approach_x", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_set_home_position_t, approach_x) }, \
         { "approach_y", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_set_home_position_t, approach_y) }, \
         { "approach_z", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_set_home_position_t, approach_z) }, \
         { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 53, offsetof(mavlink_set_home_position_t, time_usec) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SET_HOME_POSITION { \
    "SET_HOME_POSITION", \
    12, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 52, offsetof(mavlink_set_home_position_t, target_system) }, \
         { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_set_home_position_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_set_home_position_t, longitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_set_home_position_t, altitude) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_set_home_position_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_set_home_position_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_set_home_position_t, z) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 4, 24, offsetof(mavlink_set_home_position_t, q) }, \
         { "approach_x", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_set_home_position_t, approach_x) }, \
         { "approach_y", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_set_home_position_t, approach_y) }, \
         { "approach_z", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_set_home_position_t, approach_z) }, \
         { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 53, offsetof(mavlink_set_home_position_t, time_usec) }, \
         } \
}
#endif

/**
 * @brief Pack a set_home_position message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param latitude [degE7] Latitude (WGS84)
 * @param longitude [degE7] Longitude (WGS84)
 * @param altitude [mm] Altitude (MSL). Positive for up.
 * @param x [m] Local X position of this position in the local coordinate frame
 * @param y [m] Local Y position of this position in the local coordinate frame
 * @param z [m] Local Z position of this position in the local coordinate frame
 * @param q  World to surface normal and heading transformation of the takeoff position. Used to indicate the heading and slope of the ground
 * @param approach_x [m] Local X position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
 * @param approach_y [m] Local Y position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
 * @param approach_z [m] Local Z position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_home_position_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, int32_t latitude, int32_t longitude, int32_t altitude, float x, float y, float z, const float *q, float approach_x, float approach_y, float approach_z, uint64_t time_usec)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_HOME_POSITION_LEN];
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_int32_t(buf, 8, altitude);
    _mav_put_float(buf, 12, x);
    _mav_put_float(buf, 16, y);
    _mav_put_float(buf, 20, z);
    _mav_put_float(buf, 40, approach_x);
    _mav_put_float(buf, 44, approach_y);
    _mav_put_float(buf, 48, approach_z);
    _mav_put_uint8_t(buf, 52, target_system);
    _mav_put_uint64_t(buf, 53, time_usec);
    _mav_put_float_array(buf, 24, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_HOME_POSITION_LEN);
#else
    mavlink_set_home_position_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.approach_x = approach_x;
    packet.approach_y = approach_y;
    packet.approach_z = approach_z;
    packet.target_system = target_system;
    packet.time_usec = time_usec;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_HOME_POSITION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_HOME_POSITION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_HOME_POSITION_MIN_LEN, MAVLINK_MSG_ID_SET_HOME_POSITION_LEN, MAVLINK_MSG_ID_SET_HOME_POSITION_CRC);
}

/**
 * @brief Pack a set_home_position message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param latitude [degE7] Latitude (WGS84)
 * @param longitude [degE7] Longitude (WGS84)
 * @param altitude [mm] Altitude (MSL). Positive for up.
 * @param x [m] Local X position of this position in the local coordinate frame
 * @param y [m] Local Y position of this position in the local coordinate frame
 * @param z [m] Local Z position of this position in the local coordinate frame
 * @param q  World to surface normal and heading transformation of the takeoff position. Used to indicate the heading and slope of the ground
 * @param approach_x [m] Local X position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
 * @param approach_y [m] Local Y position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
 * @param approach_z [m] Local Z position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_home_position_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, int32_t latitude, int32_t longitude, int32_t altitude, float x, float y, float z, const float *q, float approach_x, float approach_y, float approach_z, uint64_t time_usec)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_HOME_POSITION_LEN];
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_int32_t(buf, 8, altitude);
    _mav_put_float(buf, 12, x);
    _mav_put_float(buf, 16, y);
    _mav_put_float(buf, 20, z);
    _mav_put_float(buf, 40, approach_x);
    _mav_put_float(buf, 44, approach_y);
    _mav_put_float(buf, 48, approach_z);
    _mav_put_uint8_t(buf, 52, target_system);
    _mav_put_uint64_t(buf, 53, time_usec);
    _mav_put_float_array(buf, 24, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_HOME_POSITION_LEN);
#else
    mavlink_set_home_position_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.approach_x = approach_x;
    packet.approach_y = approach_y;
    packet.approach_z = approach_z;
    packet.target_system = target_system;
    packet.time_usec = time_usec;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_HOME_POSITION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_HOME_POSITION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SET_HOME_POSITION_MIN_LEN, MAVLINK_MSG_ID_SET_HOME_POSITION_LEN, MAVLINK_MSG_ID_SET_HOME_POSITION_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SET_HOME_POSITION_MIN_LEN, MAVLINK_MSG_ID_SET_HOME_POSITION_LEN);
#endif
}

/**
 * @brief Pack a set_home_position message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID.
 * @param latitude [degE7] Latitude (WGS84)
 * @param longitude [degE7] Longitude (WGS84)
 * @param altitude [mm] Altitude (MSL). Positive for up.
 * @param x [m] Local X position of this position in the local coordinate frame
 * @param y [m] Local Y position of this position in the local coordinate frame
 * @param z [m] Local Z position of this position in the local coordinate frame
 * @param q  World to surface normal and heading transformation of the takeoff position. Used to indicate the heading and slope of the ground
 * @param approach_x [m] Local X position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
 * @param approach_y [m] Local Y position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
 * @param approach_z [m] Local Z position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_home_position_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,int32_t latitude,int32_t longitude,int32_t altitude,float x,float y,float z,const float *q,float approach_x,float approach_y,float approach_z,uint64_t time_usec)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_HOME_POSITION_LEN];
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_int32_t(buf, 8, altitude);
    _mav_put_float(buf, 12, x);
    _mav_put_float(buf, 16, y);
    _mav_put_float(buf, 20, z);
    _mav_put_float(buf, 40, approach_x);
    _mav_put_float(buf, 44, approach_y);
    _mav_put_float(buf, 48, approach_z);
    _mav_put_uint8_t(buf, 52, target_system);
    _mav_put_uint64_t(buf, 53, time_usec);
    _mav_put_float_array(buf, 24, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_HOME_POSITION_LEN);
#else
    mavlink_set_home_position_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.approach_x = approach_x;
    packet.approach_y = approach_y;
    packet.approach_z = approach_z;
    packet.target_system = target_system;
    packet.time_usec = time_usec;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_HOME_POSITION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_HOME_POSITION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_HOME_POSITION_MIN_LEN, MAVLINK_MSG_ID_SET_HOME_POSITION_LEN, MAVLINK_MSG_ID_SET_HOME_POSITION_CRC);
}

/**
 * @brief Encode a set_home_position struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_home_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_home_position_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_home_position_t* set_home_position)
{
    return mavlink_msg_set_home_position_pack(system_id, component_id, msg, set_home_position->target_system, set_home_position->latitude, set_home_position->longitude, set_home_position->altitude, set_home_position->x, set_home_position->y, set_home_position->z, set_home_position->q, set_home_position->approach_x, set_home_position->approach_y, set_home_position->approach_z, set_home_position->time_usec);
}

/**
 * @brief Encode a set_home_position struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_home_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_home_position_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_home_position_t* set_home_position)
{
    return mavlink_msg_set_home_position_pack_chan(system_id, component_id, chan, msg, set_home_position->target_system, set_home_position->latitude, set_home_position->longitude, set_home_position->altitude, set_home_position->x, set_home_position->y, set_home_position->z, set_home_position->q, set_home_position->approach_x, set_home_position->approach_y, set_home_position->approach_z, set_home_position->time_usec);
}

/**
 * @brief Encode a set_home_position struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param set_home_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_home_position_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_set_home_position_t* set_home_position)
{
    return mavlink_msg_set_home_position_pack_status(system_id, component_id, _status, msg,  set_home_position->target_system, set_home_position->latitude, set_home_position->longitude, set_home_position->altitude, set_home_position->x, set_home_position->y, set_home_position->z, set_home_position->q, set_home_position->approach_x, set_home_position->approach_y, set_home_position->approach_z, set_home_position->time_usec);
}

/**
 * @brief Send a set_home_position message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID.
 * @param latitude [degE7] Latitude (WGS84)
 * @param longitude [degE7] Longitude (WGS84)
 * @param altitude [mm] Altitude (MSL). Positive for up.
 * @param x [m] Local X position of this position in the local coordinate frame
 * @param y [m] Local Y position of this position in the local coordinate frame
 * @param z [m] Local Z position of this position in the local coordinate frame
 * @param q  World to surface normal and heading transformation of the takeoff position. Used to indicate the heading and slope of the ground
 * @param approach_x [m] Local X position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
 * @param approach_y [m] Local Y position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
 * @param approach_z [m] Local Z position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_home_position_send(mavlink_channel_t chan, uint8_t target_system, int32_t latitude, int32_t longitude, int32_t altitude, float x, float y, float z, const float *q, float approach_x, float approach_y, float approach_z, uint64_t time_usec)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_HOME_POSITION_LEN];
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_int32_t(buf, 8, altitude);
    _mav_put_float(buf, 12, x);
    _mav_put_float(buf, 16, y);
    _mav_put_float(buf, 20, z);
    _mav_put_float(buf, 40, approach_x);
    _mav_put_float(buf, 44, approach_y);
    _mav_put_float(buf, 48, approach_z);
    _mav_put_uint8_t(buf, 52, target_system);
    _mav_put_uint64_t(buf, 53, time_usec);
    _mav_put_float_array(buf, 24, q, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_HOME_POSITION, buf, MAVLINK_MSG_ID_SET_HOME_POSITION_MIN_LEN, MAVLINK_MSG_ID_SET_HOME_POSITION_LEN, MAVLINK_MSG_ID_SET_HOME_POSITION_CRC);
#else
    mavlink_set_home_position_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.approach_x = approach_x;
    packet.approach_y = approach_y;
    packet.approach_z = approach_z;
    packet.target_system = target_system;
    packet.time_usec = time_usec;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_HOME_POSITION, (const char *)&packet, MAVLINK_MSG_ID_SET_HOME_POSITION_MIN_LEN, MAVLINK_MSG_ID_SET_HOME_POSITION_LEN, MAVLINK_MSG_ID_SET_HOME_POSITION_CRC);
#endif
}

/**
 * @brief Send a set_home_position message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_set_home_position_send_struct(mavlink_channel_t chan, const mavlink_set_home_position_t* set_home_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_set_home_position_send(chan, set_home_position->target_system, set_home_position->latitude, set_home_position->longitude, set_home_position->altitude, set_home_position->x, set_home_position->y, set_home_position->z, set_home_position->q, set_home_position->approach_x, set_home_position->approach_y, set_home_position->approach_z, set_home_position->time_usec);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_HOME_POSITION, (const char *)set_home_position, MAVLINK_MSG_ID_SET_HOME_POSITION_MIN_LEN, MAVLINK_MSG_ID_SET_HOME_POSITION_LEN, MAVLINK_MSG_ID_SET_HOME_POSITION_CRC);
#endif
}

#if MAVLINK_MSG_ID_SET_HOME_POSITION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_home_position_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, int32_t latitude, int32_t longitude, int32_t altitude, float x, float y, float z, const float *q, float approach_x, float approach_y, float approach_z, uint64_t time_usec)
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
    _mav_put_uint8_t(buf, 52, target_system);
    _mav_put_uint64_t(buf, 53, time_usec);
    _mav_put_float_array(buf, 24, q, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_HOME_POSITION, buf, MAVLINK_MSG_ID_SET_HOME_POSITION_MIN_LEN, MAVLINK_MSG_ID_SET_HOME_POSITION_LEN, MAVLINK_MSG_ID_SET_HOME_POSITION_CRC);
#else
    mavlink_set_home_position_t *packet = (mavlink_set_home_position_t *)msgbuf;
    packet->latitude = latitude;
    packet->longitude = longitude;
    packet->altitude = altitude;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->approach_x = approach_x;
    packet->approach_y = approach_y;
    packet->approach_z = approach_z;
    packet->target_system = target_system;
    packet->time_usec = time_usec;
    mav_array_memcpy(packet->q, q, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_HOME_POSITION, (const char *)packet, MAVLINK_MSG_ID_SET_HOME_POSITION_MIN_LEN, MAVLINK_MSG_ID_SET_HOME_POSITION_LEN, MAVLINK_MSG_ID_SET_HOME_POSITION_CRC);
#endif
}
#endif

#endif

// MESSAGE SET_HOME_POSITION UNPACKING


/**
 * @brief Get field target_system from set_home_position message
 *
 * @return  System ID.
 */
static inline uint8_t mavlink_msg_set_home_position_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  52);
}

/**
 * @brief Get field latitude from set_home_position message
 *
 * @return [degE7] Latitude (WGS84)
 */
static inline int32_t mavlink_msg_set_home_position_get_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field longitude from set_home_position message
 *
 * @return [degE7] Longitude (WGS84)
 */
static inline int32_t mavlink_msg_set_home_position_get_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field altitude from set_home_position message
 *
 * @return [mm] Altitude (MSL). Positive for up.
 */
static inline int32_t mavlink_msg_set_home_position_get_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field x from set_home_position message
 *
 * @return [m] Local X position of this position in the local coordinate frame
 */
static inline float mavlink_msg_set_home_position_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field y from set_home_position message
 *
 * @return [m] Local Y position of this position in the local coordinate frame
 */
static inline float mavlink_msg_set_home_position_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field z from set_home_position message
 *
 * @return [m] Local Z position of this position in the local coordinate frame
 */
static inline float mavlink_msg_set_home_position_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field q from set_home_position message
 *
 * @return  World to surface normal and heading transformation of the takeoff position. Used to indicate the heading and slope of the ground
 */
static inline uint16_t mavlink_msg_set_home_position_get_q(const mavlink_message_t* msg, float *q)
{
    return _MAV_RETURN_float_array(msg, q, 4,  24);
}

/**
 * @brief Get field approach_x from set_home_position message
 *
 * @return [m] Local X position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
 */
static inline float mavlink_msg_set_home_position_get_approach_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field approach_y from set_home_position message
 *
 * @return [m] Local Y position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
 */
static inline float mavlink_msg_set_home_position_get_approach_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field approach_z from set_home_position message
 *
 * @return [m] Local Z position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
 */
static inline float mavlink_msg_set_home_position_get_approach_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field time_usec from set_home_position message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
static inline uint64_t mavlink_msg_set_home_position_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  53);
}

/**
 * @brief Decode a set_home_position message into a struct
 *
 * @param msg The message to decode
 * @param set_home_position C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_home_position_decode(const mavlink_message_t* msg, mavlink_set_home_position_t* set_home_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    set_home_position->latitude = mavlink_msg_set_home_position_get_latitude(msg);
    set_home_position->longitude = mavlink_msg_set_home_position_get_longitude(msg);
    set_home_position->altitude = mavlink_msg_set_home_position_get_altitude(msg);
    set_home_position->x = mavlink_msg_set_home_position_get_x(msg);
    set_home_position->y = mavlink_msg_set_home_position_get_y(msg);
    set_home_position->z = mavlink_msg_set_home_position_get_z(msg);
    mavlink_msg_set_home_position_get_q(msg, set_home_position->q);
    set_home_position->approach_x = mavlink_msg_set_home_position_get_approach_x(msg);
    set_home_position->approach_y = mavlink_msg_set_home_position_get_approach_y(msg);
    set_home_position->approach_z = mavlink_msg_set_home_position_get_approach_z(msg);
    set_home_position->target_system = mavlink_msg_set_home_position_get_target_system(msg);
    set_home_position->time_usec = mavlink_msg_set_home_position_get_time_usec(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SET_HOME_POSITION_LEN? msg->len : MAVLINK_MSG_ID_SET_HOME_POSITION_LEN;
        memset(set_home_position, 0, MAVLINK_MSG_ID_SET_HOME_POSITION_LEN);
    memcpy(set_home_position, _MAV_PAYLOAD(msg), len);
#endif
}
