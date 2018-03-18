#pragma once
// MESSAGE SET_POSITION_TARGET_LOCAL_NED PACKING

#define MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED 84

MAVPACKED(
typedef struct __mavlink_set_position_target_local_ned_t {
 uint32_t time_boot_ms; /*< Timestamp in milliseconds since system boot*/
 float x; /*< X Position in NED frame in meters*/
 float y; /*< Y Position in NED frame in meters*/
 float z; /*< Z Position in NED frame in meters (note, altitude is negative in NED)*/
 float vx; /*< X velocity in NED frame in meter / s*/
 float vy; /*< Y velocity in NED frame in meter / s*/
 float vz; /*< Z velocity in NED frame in meter / s*/
 float afx; /*< X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N*/
 float afy; /*< Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N*/
 float afz; /*< Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N*/
 float yaw; /*< yaw setpoint in rad*/
 float yaw_rate; /*< yaw rate setpoint in rad/s*/
 uint16_t type_mask; /*< Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate*/
 uint8_t target_system; /*< System ID*/
 uint8_t target_component; /*< Component ID*/
 uint8_t coordinate_frame; /*< Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9*/
}) mavlink_set_position_target_local_ned_t;

#define MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_LEN 53
#define MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_MIN_LEN 53
#define MAVLINK_MSG_ID_84_LEN 53
#define MAVLINK_MSG_ID_84_MIN_LEN 53

#define MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_CRC 143
#define MAVLINK_MSG_ID_84_CRC 143



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SET_POSITION_TARGET_LOCAL_NED { \
    84, \
    "SET_POSITION_TARGET_LOCAL_NED", \
    16, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_set_position_target_local_ned_t, time_boot_ms) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 50, offsetof(mavlink_set_position_target_local_ned_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 51, offsetof(mavlink_set_position_target_local_ned_t, target_component) }, \
         { "coordinate_frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 52, offsetof(mavlink_set_position_target_local_ned_t, coordinate_frame) }, \
         { "type_mask", NULL, MAVLINK_TYPE_UINT16_T, 0, 48, offsetof(mavlink_set_position_target_local_ned_t, type_mask) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_set_position_target_local_ned_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_set_position_target_local_ned_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_set_position_target_local_ned_t, z) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_set_position_target_local_ned_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_set_position_target_local_ned_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_set_position_target_local_ned_t, vz) }, \
         { "afx", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_set_position_target_local_ned_t, afx) }, \
         { "afy", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_set_position_target_local_ned_t, afy) }, \
         { "afz", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_set_position_target_local_ned_t, afz) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_set_position_target_local_ned_t, yaw) }, \
         { "yaw_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_set_position_target_local_ned_t, yaw_rate) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SET_POSITION_TARGET_LOCAL_NED { \
    "SET_POSITION_TARGET_LOCAL_NED", \
    16, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_set_position_target_local_ned_t, time_boot_ms) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 50, offsetof(mavlink_set_position_target_local_ned_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 51, offsetof(mavlink_set_position_target_local_ned_t, target_component) }, \
         { "coordinate_frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 52, offsetof(mavlink_set_position_target_local_ned_t, coordinate_frame) }, \
         { "type_mask", NULL, MAVLINK_TYPE_UINT16_T, 0, 48, offsetof(mavlink_set_position_target_local_ned_t, type_mask) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_set_position_target_local_ned_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_set_position_target_local_ned_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_set_position_target_local_ned_t, z) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_set_position_target_local_ned_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_set_position_target_local_ned_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_set_position_target_local_ned_t, vz) }, \
         { "afx", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_set_position_target_local_ned_t, afx) }, \
         { "afy", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_set_position_target_local_ned_t, afy) }, \
         { "afz", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_set_position_target_local_ned_t, afz) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_set_position_target_local_ned_t, yaw) }, \
         { "yaw_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_set_position_target_local_ned_t, yaw_rate) }, \
         } \
}
#endif

/**
 * @brief Pack a set_position_target_local_ned message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp in milliseconds since system boot
 * @param target_system System ID
 * @param target_component Component ID
 * @param coordinate_frame Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9
 * @param type_mask Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
 * @param x X Position in NED frame in meters
 * @param y Y Position in NED frame in meters
 * @param z Z Position in NED frame in meters (note, altitude is negative in NED)
 * @param vx X velocity in NED frame in meter / s
 * @param vy Y velocity in NED frame in meter / s
 * @param vz Z velocity in NED frame in meter / s
 * @param afx X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param afy Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param afz Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param yaw yaw setpoint in rad
 * @param yaw_rate yaw rate setpoint in rad/s
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_position_target_local_ned_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, uint16_t type_mask, float x, float y, float z, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, x);
    _mav_put_float(buf, 8, y);
    _mav_put_float(buf, 12, z);
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

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_LEN);
#else
    mavlink_set_position_target_local_ned_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.x = x;
    packet.y = y;
    packet.z = z;
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

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_MIN_LEN, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_LEN, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_CRC);
}

/**
 * @brief Pack a set_position_target_local_ned message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp in milliseconds since system boot
 * @param target_system System ID
 * @param target_component Component ID
 * @param coordinate_frame Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9
 * @param type_mask Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
 * @param x X Position in NED frame in meters
 * @param y Y Position in NED frame in meters
 * @param z Z Position in NED frame in meters (note, altitude is negative in NED)
 * @param vx X velocity in NED frame in meter / s
 * @param vy Y velocity in NED frame in meter / s
 * @param vz Z velocity in NED frame in meter / s
 * @param afx X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param afy Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param afz Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param yaw yaw setpoint in rad
 * @param yaw_rate yaw rate setpoint in rad/s
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_position_target_local_ned_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,uint8_t target_system,uint8_t target_component,uint8_t coordinate_frame,uint16_t type_mask,float x,float y,float z,float vx,float vy,float vz,float afx,float afy,float afz,float yaw,float yaw_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, x);
    _mav_put_float(buf, 8, y);
    _mav_put_float(buf, 12, z);
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

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_LEN);
#else
    mavlink_set_position_target_local_ned_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.x = x;
    packet.y = y;
    packet.z = z;
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

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_MIN_LEN, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_LEN, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_CRC);
}

/**
 * @brief Encode a set_position_target_local_ned struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_position_target_local_ned C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_position_target_local_ned_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_position_target_local_ned_t* set_position_target_local_ned)
{
    return mavlink_msg_set_position_target_local_ned_pack(system_id, component_id, msg, set_position_target_local_ned->time_boot_ms, set_position_target_local_ned->target_system, set_position_target_local_ned->target_component, set_position_target_local_ned->coordinate_frame, set_position_target_local_ned->type_mask, set_position_target_local_ned->x, set_position_target_local_ned->y, set_position_target_local_ned->z, set_position_target_local_ned->vx, set_position_target_local_ned->vy, set_position_target_local_ned->vz, set_position_target_local_ned->afx, set_position_target_local_ned->afy, set_position_target_local_ned->afz, set_position_target_local_ned->yaw, set_position_target_local_ned->yaw_rate);
}

/**
 * @brief Encode a set_position_target_local_ned struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_position_target_local_ned C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_position_target_local_ned_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_position_target_local_ned_t* set_position_target_local_ned)
{
    return mavlink_msg_set_position_target_local_ned_pack_chan(system_id, component_id, chan, msg, set_position_target_local_ned->time_boot_ms, set_position_target_local_ned->target_system, set_position_target_local_ned->target_component, set_position_target_local_ned->coordinate_frame, set_position_target_local_ned->type_mask, set_position_target_local_ned->x, set_position_target_local_ned->y, set_position_target_local_ned->z, set_position_target_local_ned->vx, set_position_target_local_ned->vy, set_position_target_local_ned->vz, set_position_target_local_ned->afx, set_position_target_local_ned->afy, set_position_target_local_ned->afz, set_position_target_local_ned->yaw, set_position_target_local_ned->yaw_rate);
}

/**
 * @brief Send a set_position_target_local_ned message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp in milliseconds since system boot
 * @param target_system System ID
 * @param target_component Component ID
 * @param coordinate_frame Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9
 * @param type_mask Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
 * @param x X Position in NED frame in meters
 * @param y Y Position in NED frame in meters
 * @param z Z Position in NED frame in meters (note, altitude is negative in NED)
 * @param vx X velocity in NED frame in meter / s
 * @param vy Y velocity in NED frame in meter / s
 * @param vz Z velocity in NED frame in meter / s
 * @param afx X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param afy Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param afz Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param yaw yaw setpoint in rad
 * @param yaw_rate yaw rate setpoint in rad/s
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_position_target_local_ned_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, uint16_t type_mask, float x, float y, float z, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, x);
    _mav_put_float(buf, 8, y);
    _mav_put_float(buf, 12, z);
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

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED, buf, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_MIN_LEN, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_LEN, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_CRC);
#else
    mavlink_set_position_target_local_ned_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.x = x;
    packet.y = y;
    packet.z = z;
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

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED, (const char *)&packet, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_MIN_LEN, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_LEN, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_CRC);
#endif
}

/**
 * @brief Send a set_position_target_local_ned message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_set_position_target_local_ned_send_struct(mavlink_channel_t chan, const mavlink_set_position_target_local_ned_t* set_position_target_local_ned)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_set_position_target_local_ned_send(chan, set_position_target_local_ned->time_boot_ms, set_position_target_local_ned->target_system, set_position_target_local_ned->target_component, set_position_target_local_ned->coordinate_frame, set_position_target_local_ned->type_mask, set_position_target_local_ned->x, set_position_target_local_ned->y, set_position_target_local_ned->z, set_position_target_local_ned->vx, set_position_target_local_ned->vy, set_position_target_local_ned->vz, set_position_target_local_ned->afx, set_position_target_local_ned->afy, set_position_target_local_ned->afz, set_position_target_local_ned->yaw, set_position_target_local_ned->yaw_rate);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED, (const char *)set_position_target_local_ned, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_MIN_LEN, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_LEN, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_CRC);
#endif
}

#if MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_position_target_local_ned_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, uint16_t type_mask, float x, float y, float z, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, x);
    _mav_put_float(buf, 8, y);
    _mav_put_float(buf, 12, z);
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

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED, buf, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_MIN_LEN, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_LEN, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_CRC);
#else
    mavlink_set_position_target_local_ned_t *packet = (mavlink_set_position_target_local_ned_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->x = x;
    packet->y = y;
    packet->z = z;
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

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED, (const char *)packet, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_MIN_LEN, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_LEN, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_CRC);
#endif
}
#endif

#endif

// MESSAGE SET_POSITION_TARGET_LOCAL_NED UNPACKING


/**
 * @brief Get field time_boot_ms from set_position_target_local_ned message
 *
 * @return Timestamp in milliseconds since system boot
 */
static inline uint32_t mavlink_msg_set_position_target_local_ned_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field target_system from set_position_target_local_ned message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_set_position_target_local_ned_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  50);
}

/**
 * @brief Get field target_component from set_position_target_local_ned message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_set_position_target_local_ned_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  51);
}

/**
 * @brief Get field coordinate_frame from set_position_target_local_ned message
 *
 * @return Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9
 */
static inline uint8_t mavlink_msg_set_position_target_local_ned_get_coordinate_frame(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  52);
}

/**
 * @brief Get field type_mask from set_position_target_local_ned message
 *
 * @return Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
 */
static inline uint16_t mavlink_msg_set_position_target_local_ned_get_type_mask(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  48);
}

/**
 * @brief Get field x from set_position_target_local_ned message
 *
 * @return X Position in NED frame in meters
 */
static inline float mavlink_msg_set_position_target_local_ned_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field y from set_position_target_local_ned message
 *
 * @return Y Position in NED frame in meters
 */
static inline float mavlink_msg_set_position_target_local_ned_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field z from set_position_target_local_ned message
 *
 * @return Z Position in NED frame in meters (note, altitude is negative in NED)
 */
static inline float mavlink_msg_set_position_target_local_ned_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field vx from set_position_target_local_ned message
 *
 * @return X velocity in NED frame in meter / s
 */
static inline float mavlink_msg_set_position_target_local_ned_get_vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field vy from set_position_target_local_ned message
 *
 * @return Y velocity in NED frame in meter / s
 */
static inline float mavlink_msg_set_position_target_local_ned_get_vy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field vz from set_position_target_local_ned message
 *
 * @return Z velocity in NED frame in meter / s
 */
static inline float mavlink_msg_set_position_target_local_ned_get_vz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field afx from set_position_target_local_ned message
 *
 * @return X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 */
static inline float mavlink_msg_set_position_target_local_ned_get_afx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field afy from set_position_target_local_ned message
 *
 * @return Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 */
static inline float mavlink_msg_set_position_target_local_ned_get_afy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field afz from set_position_target_local_ned message
 *
 * @return Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 */
static inline float mavlink_msg_set_position_target_local_ned_get_afz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field yaw from set_position_target_local_ned message
 *
 * @return yaw setpoint in rad
 */
static inline float mavlink_msg_set_position_target_local_ned_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field yaw_rate from set_position_target_local_ned message
 *
 * @return yaw rate setpoint in rad/s
 */
static inline float mavlink_msg_set_position_target_local_ned_get_yaw_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Decode a set_position_target_local_ned message into a struct
 *
 * @param msg The message to decode
 * @param set_position_target_local_ned C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_position_target_local_ned_decode(const mavlink_message_t* msg, mavlink_set_position_target_local_ned_t* set_position_target_local_ned)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    set_position_target_local_ned->time_boot_ms = mavlink_msg_set_position_target_local_ned_get_time_boot_ms(msg);
    set_position_target_local_ned->x = mavlink_msg_set_position_target_local_ned_get_x(msg);
    set_position_target_local_ned->y = mavlink_msg_set_position_target_local_ned_get_y(msg);
    set_position_target_local_ned->z = mavlink_msg_set_position_target_local_ned_get_z(msg);
    set_position_target_local_ned->vx = mavlink_msg_set_position_target_local_ned_get_vx(msg);
    set_position_target_local_ned->vy = mavlink_msg_set_position_target_local_ned_get_vy(msg);
    set_position_target_local_ned->vz = mavlink_msg_set_position_target_local_ned_get_vz(msg);
    set_position_target_local_ned->afx = mavlink_msg_set_position_target_local_ned_get_afx(msg);
    set_position_target_local_ned->afy = mavlink_msg_set_position_target_local_ned_get_afy(msg);
    set_position_target_local_ned->afz = mavlink_msg_set_position_target_local_ned_get_afz(msg);
    set_position_target_local_ned->yaw = mavlink_msg_set_position_target_local_ned_get_yaw(msg);
    set_position_target_local_ned->yaw_rate = mavlink_msg_set_position_target_local_ned_get_yaw_rate(msg);
    set_position_target_local_ned->type_mask = mavlink_msg_set_position_target_local_ned_get_type_mask(msg);
    set_position_target_local_ned->target_system = mavlink_msg_set_position_target_local_ned_get_target_system(msg);
    set_position_target_local_ned->target_component = mavlink_msg_set_position_target_local_ned_get_target_component(msg);
    set_position_target_local_ned->coordinate_frame = mavlink_msg_set_position_target_local_ned_get_coordinate_frame(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_LEN? msg->len : MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_LEN;
        memset(set_position_target_local_ned, 0, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_LEN);
    memcpy(set_position_target_local_ned, _MAV_PAYLOAD(msg), len);
#endif
}
