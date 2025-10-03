#pragma once
// MESSAGE SET_POSITION_TARGET_GLOBAL_INT PACKING

#define MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT 86


typedef struct __mavlink_set_position_target_global_int_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot). The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.*/
 int32_t lat_int; /*< [degE7] Latitude in WGS84 frame*/
 int32_t lon_int; /*< [degE7] Longitude in WGS84 frame*/
 float alt; /*< [m] Altitude (MSL, Relative to home, or AGL - depending on frame)*/
 float vx; /*< [m/s] X velocity in NED frame*/
 float vy; /*< [m/s] Y velocity in NED frame*/
 float vz; /*< [m/s] Z velocity in NED frame*/
 float afx; /*< [m/s/s] X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N*/
 float afy; /*< [m/s/s] Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N*/
 float afz; /*< [m/s/s] Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N*/
 float yaw; /*< [rad] yaw setpoint*/
 float yaw_rate; /*< [rad/s] yaw rate setpoint*/
 uint16_t type_mask; /*<  Bitmap to indicate which dimensions should be ignored by the vehicle.*/
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
 uint8_t coordinate_frame; /*<  Valid options are: MAV_FRAME_GLOBAL = 0, MAV_FRAME_GLOBAL_RELATIVE_ALT = 3, MAV_FRAME_GLOBAL_TERRAIN_ALT = 10 (MAV_FRAME_GLOBAL_INT, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT are allowed synonyms, but have been deprecated)*/
} mavlink_set_position_target_global_int_t;

#define MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_LEN 53
#define MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_MIN_LEN 53
#define MAVLINK_MSG_ID_86_LEN 53
#define MAVLINK_MSG_ID_86_MIN_LEN 53

#define MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_CRC 5
#define MAVLINK_MSG_ID_86_CRC 5



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SET_POSITION_TARGET_GLOBAL_INT { \
    86, \
    "SET_POSITION_TARGET_GLOBAL_INT", \
    16, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_set_position_target_global_int_t, time_boot_ms) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 50, offsetof(mavlink_set_position_target_global_int_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 51, offsetof(mavlink_set_position_target_global_int_t, target_component) }, \
         { "coordinate_frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 52, offsetof(mavlink_set_position_target_global_int_t, coordinate_frame) }, \
         { "type_mask", NULL, MAVLINK_TYPE_UINT16_T, 0, 48, offsetof(mavlink_set_position_target_global_int_t, type_mask) }, \
         { "lat_int", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_set_position_target_global_int_t, lat_int) }, \
         { "lon_int", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_set_position_target_global_int_t, lon_int) }, \
         { "alt", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_set_position_target_global_int_t, alt) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_set_position_target_global_int_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_set_position_target_global_int_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_set_position_target_global_int_t, vz) }, \
         { "afx", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_set_position_target_global_int_t, afx) }, \
         { "afy", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_set_position_target_global_int_t, afy) }, \
         { "afz", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_set_position_target_global_int_t, afz) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_set_position_target_global_int_t, yaw) }, \
         { "yaw_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_set_position_target_global_int_t, yaw_rate) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SET_POSITION_TARGET_GLOBAL_INT { \
    "SET_POSITION_TARGET_GLOBAL_INT", \
    16, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_set_position_target_global_int_t, time_boot_ms) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 50, offsetof(mavlink_set_position_target_global_int_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 51, offsetof(mavlink_set_position_target_global_int_t, target_component) }, \
         { "coordinate_frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 52, offsetof(mavlink_set_position_target_global_int_t, coordinate_frame) }, \
         { "type_mask", NULL, MAVLINK_TYPE_UINT16_T, 0, 48, offsetof(mavlink_set_position_target_global_int_t, type_mask) }, \
         { "lat_int", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_set_position_target_global_int_t, lat_int) }, \
         { "lon_int", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_set_position_target_global_int_t, lon_int) }, \
         { "alt", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_set_position_target_global_int_t, alt) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_set_position_target_global_int_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_set_position_target_global_int_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_set_position_target_global_int_t, vz) }, \
         { "afx", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_set_position_target_global_int_t, afx) }, \
         { "afy", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_set_position_target_global_int_t, afy) }, \
         { "afz", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_set_position_target_global_int_t, afz) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_set_position_target_global_int_t, yaw) }, \
         { "yaw_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_set_position_target_global_int_t, yaw_rate) }, \
         } \
}
#endif

/**
 * @brief Pack a set_position_target_global_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot). The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param coordinate_frame  Valid options are: MAV_FRAME_GLOBAL = 0, MAV_FRAME_GLOBAL_RELATIVE_ALT = 3, MAV_FRAME_GLOBAL_TERRAIN_ALT = 10 (MAV_FRAME_GLOBAL_INT, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT are allowed synonyms, but have been deprecated)
 * @param type_mask  Bitmap to indicate which dimensions should be ignored by the vehicle.
 * @param lat_int [degE7] Latitude in WGS84 frame
 * @param lon_int [degE7] Longitude in WGS84 frame
 * @param alt [m] Altitude (MSL, Relative to home, or AGL - depending on frame)
 * @param vx [m/s] X velocity in NED frame
 * @param vy [m/s] Y velocity in NED frame
 * @param vz [m/s] Z velocity in NED frame
 * @param afx [m/s/s] X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param afy [m/s/s] Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param afz [m/s/s] Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param yaw [rad] yaw setpoint
 * @param yaw_rate [rad/s] yaw rate setpoint
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_position_target_global_int_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, uint16_t type_mask, int32_t lat_int, int32_t lon_int, float alt, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, lat_int);
    _mav_put_int32_t(buf, 8, lon_int);
    _mav_put_float(buf, 12, alt);
    _mav_put_float(buf, 16, vx);
    _mav_put_float(buf, 20, vy);
    _mav_put_float(buf, 24, vz);
    _mav_put_float(buf, 28, afx);
    _mav_put_float(buf, 32, afy);
    _mav_put_float(buf, 36, afz);
    _mav_put_float(buf, 40, yaw);
    _mav_put_float(buf, 44, yaw_rate);
    _mav_put_uint16_t(buf, 48, type_mask);
    _mav_put_uint8_t(buf, 50, target_system);
    _mav_put_uint8_t(buf, 51, target_component);
    _mav_put_uint8_t(buf, 52, coordinate_frame);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_LEN);
#else
    mavlink_set_position_target_global_int_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.lat_int = lat_int;
    packet.lon_int = lon_int;
    packet.alt = alt;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.afx = afx;
    packet.afy = afy;
    packet.afz = afz;
    packet.yaw = yaw;
    packet.yaw_rate = yaw_rate;
    packet.type_mask = type_mask;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.coordinate_frame = coordinate_frame;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_MIN_LEN, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_LEN, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_CRC);
}

/**
 * @brief Pack a set_position_target_global_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot). The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param coordinate_frame  Valid options are: MAV_FRAME_GLOBAL = 0, MAV_FRAME_GLOBAL_RELATIVE_ALT = 3, MAV_FRAME_GLOBAL_TERRAIN_ALT = 10 (MAV_FRAME_GLOBAL_INT, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT are allowed synonyms, but have been deprecated)
 * @param type_mask  Bitmap to indicate which dimensions should be ignored by the vehicle.
 * @param lat_int [degE7] Latitude in WGS84 frame
 * @param lon_int [degE7] Longitude in WGS84 frame
 * @param alt [m] Altitude (MSL, Relative to home, or AGL - depending on frame)
 * @param vx [m/s] X velocity in NED frame
 * @param vy [m/s] Y velocity in NED frame
 * @param vz [m/s] Z velocity in NED frame
 * @param afx [m/s/s] X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param afy [m/s/s] Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param afz [m/s/s] Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param yaw [rad] yaw setpoint
 * @param yaw_rate [rad/s] yaw rate setpoint
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_position_target_global_int_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, uint16_t type_mask, int32_t lat_int, int32_t lon_int, float alt, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, lat_int);
    _mav_put_int32_t(buf, 8, lon_int);
    _mav_put_float(buf, 12, alt);
    _mav_put_float(buf, 16, vx);
    _mav_put_float(buf, 20, vy);
    _mav_put_float(buf, 24, vz);
    _mav_put_float(buf, 28, afx);
    _mav_put_float(buf, 32, afy);
    _mav_put_float(buf, 36, afz);
    _mav_put_float(buf, 40, yaw);
    _mav_put_float(buf, 44, yaw_rate);
    _mav_put_uint16_t(buf, 48, type_mask);
    _mav_put_uint8_t(buf, 50, target_system);
    _mav_put_uint8_t(buf, 51, target_component);
    _mav_put_uint8_t(buf, 52, coordinate_frame);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_LEN);
#else
    mavlink_set_position_target_global_int_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.lat_int = lat_int;
    packet.lon_int = lon_int;
    packet.alt = alt;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.afx = afx;
    packet.afy = afy;
    packet.afz = afz;
    packet.yaw = yaw;
    packet.yaw_rate = yaw_rate;
    packet.type_mask = type_mask;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.coordinate_frame = coordinate_frame;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_MIN_LEN, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_LEN, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_MIN_LEN, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_LEN);
#endif
}

/**
 * @brief Pack a set_position_target_global_int message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot). The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param coordinate_frame  Valid options are: MAV_FRAME_GLOBAL = 0, MAV_FRAME_GLOBAL_RELATIVE_ALT = 3, MAV_FRAME_GLOBAL_TERRAIN_ALT = 10 (MAV_FRAME_GLOBAL_INT, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT are allowed synonyms, but have been deprecated)
 * @param type_mask  Bitmap to indicate which dimensions should be ignored by the vehicle.
 * @param lat_int [degE7] Latitude in WGS84 frame
 * @param lon_int [degE7] Longitude in WGS84 frame
 * @param alt [m] Altitude (MSL, Relative to home, or AGL - depending on frame)
 * @param vx [m/s] X velocity in NED frame
 * @param vy [m/s] Y velocity in NED frame
 * @param vz [m/s] Z velocity in NED frame
 * @param afx [m/s/s] X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param afy [m/s/s] Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param afz [m/s/s] Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param yaw [rad] yaw setpoint
 * @param yaw_rate [rad/s] yaw rate setpoint
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_position_target_global_int_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,uint8_t target_system,uint8_t target_component,uint8_t coordinate_frame,uint16_t type_mask,int32_t lat_int,int32_t lon_int,float alt,float vx,float vy,float vz,float afx,float afy,float afz,float yaw,float yaw_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, lat_int);
    _mav_put_int32_t(buf, 8, lon_int);
    _mav_put_float(buf, 12, alt);
    _mav_put_float(buf, 16, vx);
    _mav_put_float(buf, 20, vy);
    _mav_put_float(buf, 24, vz);
    _mav_put_float(buf, 28, afx);
    _mav_put_float(buf, 32, afy);
    _mav_put_float(buf, 36, afz);
    _mav_put_float(buf, 40, yaw);
    _mav_put_float(buf, 44, yaw_rate);
    _mav_put_uint16_t(buf, 48, type_mask);
    _mav_put_uint8_t(buf, 50, target_system);
    _mav_put_uint8_t(buf, 51, target_component);
    _mav_put_uint8_t(buf, 52, coordinate_frame);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_LEN);
#else
    mavlink_set_position_target_global_int_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.lat_int = lat_int;
    packet.lon_int = lon_int;
    packet.alt = alt;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.afx = afx;
    packet.afy = afy;
    packet.afz = afz;
    packet.yaw = yaw;
    packet.yaw_rate = yaw_rate;
    packet.type_mask = type_mask;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.coordinate_frame = coordinate_frame;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_MIN_LEN, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_LEN, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_CRC);
}

/**
 * @brief Encode a set_position_target_global_int struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_position_target_global_int C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_position_target_global_int_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_position_target_global_int_t* set_position_target_global_int)
{
    return mavlink_msg_set_position_target_global_int_pack(system_id, component_id, msg, set_position_target_global_int->time_boot_ms, set_position_target_global_int->target_system, set_position_target_global_int->target_component, set_position_target_global_int->coordinate_frame, set_position_target_global_int->type_mask, set_position_target_global_int->lat_int, set_position_target_global_int->lon_int, set_position_target_global_int->alt, set_position_target_global_int->vx, set_position_target_global_int->vy, set_position_target_global_int->vz, set_position_target_global_int->afx, set_position_target_global_int->afy, set_position_target_global_int->afz, set_position_target_global_int->yaw, set_position_target_global_int->yaw_rate);
}

/**
 * @brief Encode a set_position_target_global_int struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_position_target_global_int C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_position_target_global_int_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_position_target_global_int_t* set_position_target_global_int)
{
    return mavlink_msg_set_position_target_global_int_pack_chan(system_id, component_id, chan, msg, set_position_target_global_int->time_boot_ms, set_position_target_global_int->target_system, set_position_target_global_int->target_component, set_position_target_global_int->coordinate_frame, set_position_target_global_int->type_mask, set_position_target_global_int->lat_int, set_position_target_global_int->lon_int, set_position_target_global_int->alt, set_position_target_global_int->vx, set_position_target_global_int->vy, set_position_target_global_int->vz, set_position_target_global_int->afx, set_position_target_global_int->afy, set_position_target_global_int->afz, set_position_target_global_int->yaw, set_position_target_global_int->yaw_rate);
}

/**
 * @brief Encode a set_position_target_global_int struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param set_position_target_global_int C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_position_target_global_int_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_set_position_target_global_int_t* set_position_target_global_int)
{
    return mavlink_msg_set_position_target_global_int_pack_status(system_id, component_id, _status, msg,  set_position_target_global_int->time_boot_ms, set_position_target_global_int->target_system, set_position_target_global_int->target_component, set_position_target_global_int->coordinate_frame, set_position_target_global_int->type_mask, set_position_target_global_int->lat_int, set_position_target_global_int->lon_int, set_position_target_global_int->alt, set_position_target_global_int->vx, set_position_target_global_int->vy, set_position_target_global_int->vz, set_position_target_global_int->afx, set_position_target_global_int->afy, set_position_target_global_int->afz, set_position_target_global_int->yaw, set_position_target_global_int->yaw_rate);
}

/**
 * @brief Send a set_position_target_global_int message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot). The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param coordinate_frame  Valid options are: MAV_FRAME_GLOBAL = 0, MAV_FRAME_GLOBAL_RELATIVE_ALT = 3, MAV_FRAME_GLOBAL_TERRAIN_ALT = 10 (MAV_FRAME_GLOBAL_INT, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT are allowed synonyms, but have been deprecated)
 * @param type_mask  Bitmap to indicate which dimensions should be ignored by the vehicle.
 * @param lat_int [degE7] Latitude in WGS84 frame
 * @param lon_int [degE7] Longitude in WGS84 frame
 * @param alt [m] Altitude (MSL, Relative to home, or AGL - depending on frame)
 * @param vx [m/s] X velocity in NED frame
 * @param vy [m/s] Y velocity in NED frame
 * @param vz [m/s] Z velocity in NED frame
 * @param afx [m/s/s] X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param afy [m/s/s] Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param afz [m/s/s] Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param yaw [rad] yaw setpoint
 * @param yaw_rate [rad/s] yaw rate setpoint
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_position_target_global_int_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, uint16_t type_mask, int32_t lat_int, int32_t lon_int, float alt, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, lat_int);
    _mav_put_int32_t(buf, 8, lon_int);
    _mav_put_float(buf, 12, alt);
    _mav_put_float(buf, 16, vx);
    _mav_put_float(buf, 20, vy);
    _mav_put_float(buf, 24, vz);
    _mav_put_float(buf, 28, afx);
    _mav_put_float(buf, 32, afy);
    _mav_put_float(buf, 36, afz);
    _mav_put_float(buf, 40, yaw);
    _mav_put_float(buf, 44, yaw_rate);
    _mav_put_uint16_t(buf, 48, type_mask);
    _mav_put_uint8_t(buf, 50, target_system);
    _mav_put_uint8_t(buf, 51, target_component);
    _mav_put_uint8_t(buf, 52, coordinate_frame);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT, buf, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_MIN_LEN, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_LEN, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_CRC);
#else
    mavlink_set_position_target_global_int_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.lat_int = lat_int;
    packet.lon_int = lon_int;
    packet.alt = alt;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.afx = afx;
    packet.afy = afy;
    packet.afz = afz;
    packet.yaw = yaw;
    packet.yaw_rate = yaw_rate;
    packet.type_mask = type_mask;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.coordinate_frame = coordinate_frame;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT, (const char *)&packet, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_MIN_LEN, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_LEN, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_CRC);
#endif
}

/**
 * @brief Send a set_position_target_global_int message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_set_position_target_global_int_send_struct(mavlink_channel_t chan, const mavlink_set_position_target_global_int_t* set_position_target_global_int)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_set_position_target_global_int_send(chan, set_position_target_global_int->time_boot_ms, set_position_target_global_int->target_system, set_position_target_global_int->target_component, set_position_target_global_int->coordinate_frame, set_position_target_global_int->type_mask, set_position_target_global_int->lat_int, set_position_target_global_int->lon_int, set_position_target_global_int->alt, set_position_target_global_int->vx, set_position_target_global_int->vy, set_position_target_global_int->vz, set_position_target_global_int->afx, set_position_target_global_int->afy, set_position_target_global_int->afz, set_position_target_global_int->yaw, set_position_target_global_int->yaw_rate);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT, (const char *)set_position_target_global_int, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_MIN_LEN, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_LEN, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_CRC);
#endif
}

#if MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_position_target_global_int_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, uint16_t type_mask, int32_t lat_int, int32_t lon_int, float alt, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, lat_int);
    _mav_put_int32_t(buf, 8, lon_int);
    _mav_put_float(buf, 12, alt);
    _mav_put_float(buf, 16, vx);
    _mav_put_float(buf, 20, vy);
    _mav_put_float(buf, 24, vz);
    _mav_put_float(buf, 28, afx);
    _mav_put_float(buf, 32, afy);
    _mav_put_float(buf, 36, afz);
    _mav_put_float(buf, 40, yaw);
    _mav_put_float(buf, 44, yaw_rate);
    _mav_put_uint16_t(buf, 48, type_mask);
    _mav_put_uint8_t(buf, 50, target_system);
    _mav_put_uint8_t(buf, 51, target_component);
    _mav_put_uint8_t(buf, 52, coordinate_frame);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT, buf, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_MIN_LEN, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_LEN, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_CRC);
#else
    mavlink_set_position_target_global_int_t *packet = (mavlink_set_position_target_global_int_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->lat_int = lat_int;
    packet->lon_int = lon_int;
    packet->alt = alt;
    packet->vx = vx;
    packet->vy = vy;
    packet->vz = vz;
    packet->afx = afx;
    packet->afy = afy;
    packet->afz = afz;
    packet->yaw = yaw;
    packet->yaw_rate = yaw_rate;
    packet->type_mask = type_mask;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->coordinate_frame = coordinate_frame;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT, (const char *)packet, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_MIN_LEN, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_LEN, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_CRC);
#endif
}
#endif

#endif

// MESSAGE SET_POSITION_TARGET_GLOBAL_INT UNPACKING


/**
 * @brief Get field time_boot_ms from set_position_target_global_int message
 *
 * @return [ms] Timestamp (time since system boot). The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
 */
static inline uint32_t mavlink_msg_set_position_target_global_int_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field target_system from set_position_target_global_int message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_set_position_target_global_int_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  50);
}

/**
 * @brief Get field target_component from set_position_target_global_int message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_set_position_target_global_int_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  51);
}

/**
 * @brief Get field coordinate_frame from set_position_target_global_int message
 *
 * @return  Valid options are: MAV_FRAME_GLOBAL = 0, MAV_FRAME_GLOBAL_RELATIVE_ALT = 3, MAV_FRAME_GLOBAL_TERRAIN_ALT = 10 (MAV_FRAME_GLOBAL_INT, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT are allowed synonyms, but have been deprecated)
 */
static inline uint8_t mavlink_msg_set_position_target_global_int_get_coordinate_frame(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  52);
}

/**
 * @brief Get field type_mask from set_position_target_global_int message
 *
 * @return  Bitmap to indicate which dimensions should be ignored by the vehicle.
 */
static inline uint16_t mavlink_msg_set_position_target_global_int_get_type_mask(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  48);
}

/**
 * @brief Get field lat_int from set_position_target_global_int message
 *
 * @return [degE7] Latitude in WGS84 frame
 */
static inline int32_t mavlink_msg_set_position_target_global_int_get_lat_int(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field lon_int from set_position_target_global_int message
 *
 * @return [degE7] Longitude in WGS84 frame
 */
static inline int32_t mavlink_msg_set_position_target_global_int_get_lon_int(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field alt from set_position_target_global_int message
 *
 * @return [m] Altitude (MSL, Relative to home, or AGL - depending on frame)
 */
static inline float mavlink_msg_set_position_target_global_int_get_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field vx from set_position_target_global_int message
 *
 * @return [m/s] X velocity in NED frame
 */
static inline float mavlink_msg_set_position_target_global_int_get_vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field vy from set_position_target_global_int message
 *
 * @return [m/s] Y velocity in NED frame
 */
static inline float mavlink_msg_set_position_target_global_int_get_vy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field vz from set_position_target_global_int message
 *
 * @return [m/s] Z velocity in NED frame
 */
static inline float mavlink_msg_set_position_target_global_int_get_vz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field afx from set_position_target_global_int message
 *
 * @return [m/s/s] X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 */
static inline float mavlink_msg_set_position_target_global_int_get_afx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field afy from set_position_target_global_int message
 *
 * @return [m/s/s] Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 */
static inline float mavlink_msg_set_position_target_global_int_get_afy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field afz from set_position_target_global_int message
 *
 * @return [m/s/s] Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 */
static inline float mavlink_msg_set_position_target_global_int_get_afz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field yaw from set_position_target_global_int message
 *
 * @return [rad] yaw setpoint
 */
static inline float mavlink_msg_set_position_target_global_int_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field yaw_rate from set_position_target_global_int message
 *
 * @return [rad/s] yaw rate setpoint
 */
static inline float mavlink_msg_set_position_target_global_int_get_yaw_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Decode a set_position_target_global_int message into a struct
 *
 * @param msg The message to decode
 * @param set_position_target_global_int C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_position_target_global_int_decode(const mavlink_message_t* msg, mavlink_set_position_target_global_int_t* set_position_target_global_int)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    set_position_target_global_int->time_boot_ms = mavlink_msg_set_position_target_global_int_get_time_boot_ms(msg);
    set_position_target_global_int->lat_int = mavlink_msg_set_position_target_global_int_get_lat_int(msg);
    set_position_target_global_int->lon_int = mavlink_msg_set_position_target_global_int_get_lon_int(msg);
    set_position_target_global_int->alt = mavlink_msg_set_position_target_global_int_get_alt(msg);
    set_position_target_global_int->vx = mavlink_msg_set_position_target_global_int_get_vx(msg);
    set_position_target_global_int->vy = mavlink_msg_set_position_target_global_int_get_vy(msg);
    set_position_target_global_int->vz = mavlink_msg_set_position_target_global_int_get_vz(msg);
    set_position_target_global_int->afx = mavlink_msg_set_position_target_global_int_get_afx(msg);
    set_position_target_global_int->afy = mavlink_msg_set_position_target_global_int_get_afy(msg);
    set_position_target_global_int->afz = mavlink_msg_set_position_target_global_int_get_afz(msg);
    set_position_target_global_int->yaw = mavlink_msg_set_position_target_global_int_get_yaw(msg);
    set_position_target_global_int->yaw_rate = mavlink_msg_set_position_target_global_int_get_yaw_rate(msg);
    set_position_target_global_int->type_mask = mavlink_msg_set_position_target_global_int_get_type_mask(msg);
    set_position_target_global_int->target_system = mavlink_msg_set_position_target_global_int_get_target_system(msg);
    set_position_target_global_int->target_component = mavlink_msg_set_position_target_global_int_get_target_component(msg);
    set_position_target_global_int->coordinate_frame = mavlink_msg_set_position_target_global_int_get_coordinate_frame(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_LEN? msg->len : MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_LEN;
        memset(set_position_target_global_int, 0, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_LEN);
    memcpy(set_position_target_global_int, _MAV_PAYLOAD(msg), len);
#endif
}
