#pragma once
// MESSAGE TRAJECTORY_REPRESENTATION_WAYPOINTS PACKING

#define MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS 332


typedef struct __mavlink_trajectory_representation_waypoints_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 float pos_x[5]; /*< [m] X-coordinate of waypoint, set to NaN if not being used*/
 float pos_y[5]; /*< [m] Y-coordinate of waypoint, set to NaN if not being used*/
 float pos_z[5]; /*< [m] Z-coordinate of waypoint, set to NaN if not being used*/
 float vel_x[5]; /*< [m/s] X-velocity of waypoint, set to NaN if not being used*/
 float vel_y[5]; /*< [m/s] Y-velocity of waypoint, set to NaN if not being used*/
 float vel_z[5]; /*< [m/s] Z-velocity of waypoint, set to NaN if not being used*/
 float acc_x[5]; /*< [m/s/s] X-acceleration of waypoint, set to NaN if not being used*/
 float acc_y[5]; /*< [m/s/s] Y-acceleration of waypoint, set to NaN if not being used*/
 float acc_z[5]; /*< [m/s/s] Z-acceleration of waypoint, set to NaN if not being used*/
 float pos_yaw[5]; /*< [rad] Yaw angle, set to NaN if not being used*/
 float vel_yaw[5]; /*< [rad/s] Yaw rate, set to NaN if not being used*/
 uint16_t command[5]; /*<  MAV_CMD command id of waypoint, set to UINT16_MAX if not being used.*/
 uint8_t valid_points; /*<  Number of valid points (up-to 5 waypoints are possible)*/
} mavlink_trajectory_representation_waypoints_t;

#define MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_LEN 239
#define MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_MIN_LEN 239
#define MAVLINK_MSG_ID_332_LEN 239
#define MAVLINK_MSG_ID_332_MIN_LEN 239

#define MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_CRC 236
#define MAVLINK_MSG_ID_332_CRC 236

#define MAVLINK_MSG_TRAJECTORY_REPRESENTATION_WAYPOINTS_FIELD_POS_X_LEN 5
#define MAVLINK_MSG_TRAJECTORY_REPRESENTATION_WAYPOINTS_FIELD_POS_Y_LEN 5
#define MAVLINK_MSG_TRAJECTORY_REPRESENTATION_WAYPOINTS_FIELD_POS_Z_LEN 5
#define MAVLINK_MSG_TRAJECTORY_REPRESENTATION_WAYPOINTS_FIELD_VEL_X_LEN 5
#define MAVLINK_MSG_TRAJECTORY_REPRESENTATION_WAYPOINTS_FIELD_VEL_Y_LEN 5
#define MAVLINK_MSG_TRAJECTORY_REPRESENTATION_WAYPOINTS_FIELD_VEL_Z_LEN 5
#define MAVLINK_MSG_TRAJECTORY_REPRESENTATION_WAYPOINTS_FIELD_ACC_X_LEN 5
#define MAVLINK_MSG_TRAJECTORY_REPRESENTATION_WAYPOINTS_FIELD_ACC_Y_LEN 5
#define MAVLINK_MSG_TRAJECTORY_REPRESENTATION_WAYPOINTS_FIELD_ACC_Z_LEN 5
#define MAVLINK_MSG_TRAJECTORY_REPRESENTATION_WAYPOINTS_FIELD_POS_YAW_LEN 5
#define MAVLINK_MSG_TRAJECTORY_REPRESENTATION_WAYPOINTS_FIELD_VEL_YAW_LEN 5
#define MAVLINK_MSG_TRAJECTORY_REPRESENTATION_WAYPOINTS_FIELD_COMMAND_LEN 5

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TRAJECTORY_REPRESENTATION_WAYPOINTS { \
    332, \
    "TRAJECTORY_REPRESENTATION_WAYPOINTS", \
    14, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_trajectory_representation_waypoints_t, time_usec) }, \
         { "valid_points", NULL, MAVLINK_TYPE_UINT8_T, 0, 238, offsetof(mavlink_trajectory_representation_waypoints_t, valid_points) }, \
         { "pos_x", NULL, MAVLINK_TYPE_FLOAT, 5, 8, offsetof(mavlink_trajectory_representation_waypoints_t, pos_x) }, \
         { "pos_y", NULL, MAVLINK_TYPE_FLOAT, 5, 28, offsetof(mavlink_trajectory_representation_waypoints_t, pos_y) }, \
         { "pos_z", NULL, MAVLINK_TYPE_FLOAT, 5, 48, offsetof(mavlink_trajectory_representation_waypoints_t, pos_z) }, \
         { "vel_x", NULL, MAVLINK_TYPE_FLOAT, 5, 68, offsetof(mavlink_trajectory_representation_waypoints_t, vel_x) }, \
         { "vel_y", NULL, MAVLINK_TYPE_FLOAT, 5, 88, offsetof(mavlink_trajectory_representation_waypoints_t, vel_y) }, \
         { "vel_z", NULL, MAVLINK_TYPE_FLOAT, 5, 108, offsetof(mavlink_trajectory_representation_waypoints_t, vel_z) }, \
         { "acc_x", NULL, MAVLINK_TYPE_FLOAT, 5, 128, offsetof(mavlink_trajectory_representation_waypoints_t, acc_x) }, \
         { "acc_y", NULL, MAVLINK_TYPE_FLOAT, 5, 148, offsetof(mavlink_trajectory_representation_waypoints_t, acc_y) }, \
         { "acc_z", NULL, MAVLINK_TYPE_FLOAT, 5, 168, offsetof(mavlink_trajectory_representation_waypoints_t, acc_z) }, \
         { "pos_yaw", NULL, MAVLINK_TYPE_FLOAT, 5, 188, offsetof(mavlink_trajectory_representation_waypoints_t, pos_yaw) }, \
         { "vel_yaw", NULL, MAVLINK_TYPE_FLOAT, 5, 208, offsetof(mavlink_trajectory_representation_waypoints_t, vel_yaw) }, \
         { "command", NULL, MAVLINK_TYPE_UINT16_T, 5, 228, offsetof(mavlink_trajectory_representation_waypoints_t, command) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TRAJECTORY_REPRESENTATION_WAYPOINTS { \
    "TRAJECTORY_REPRESENTATION_WAYPOINTS", \
    14, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_trajectory_representation_waypoints_t, time_usec) }, \
         { "valid_points", NULL, MAVLINK_TYPE_UINT8_T, 0, 238, offsetof(mavlink_trajectory_representation_waypoints_t, valid_points) }, \
         { "pos_x", NULL, MAVLINK_TYPE_FLOAT, 5, 8, offsetof(mavlink_trajectory_representation_waypoints_t, pos_x) }, \
         { "pos_y", NULL, MAVLINK_TYPE_FLOAT, 5, 28, offsetof(mavlink_trajectory_representation_waypoints_t, pos_y) }, \
         { "pos_z", NULL, MAVLINK_TYPE_FLOAT, 5, 48, offsetof(mavlink_trajectory_representation_waypoints_t, pos_z) }, \
         { "vel_x", NULL, MAVLINK_TYPE_FLOAT, 5, 68, offsetof(mavlink_trajectory_representation_waypoints_t, vel_x) }, \
         { "vel_y", NULL, MAVLINK_TYPE_FLOAT, 5, 88, offsetof(mavlink_trajectory_representation_waypoints_t, vel_y) }, \
         { "vel_z", NULL, MAVLINK_TYPE_FLOAT, 5, 108, offsetof(mavlink_trajectory_representation_waypoints_t, vel_z) }, \
         { "acc_x", NULL, MAVLINK_TYPE_FLOAT, 5, 128, offsetof(mavlink_trajectory_representation_waypoints_t, acc_x) }, \
         { "acc_y", NULL, MAVLINK_TYPE_FLOAT, 5, 148, offsetof(mavlink_trajectory_representation_waypoints_t, acc_y) }, \
         { "acc_z", NULL, MAVLINK_TYPE_FLOAT, 5, 168, offsetof(mavlink_trajectory_representation_waypoints_t, acc_z) }, \
         { "pos_yaw", NULL, MAVLINK_TYPE_FLOAT, 5, 188, offsetof(mavlink_trajectory_representation_waypoints_t, pos_yaw) }, \
         { "vel_yaw", NULL, MAVLINK_TYPE_FLOAT, 5, 208, offsetof(mavlink_trajectory_representation_waypoints_t, vel_yaw) }, \
         { "command", NULL, MAVLINK_TYPE_UINT16_T, 5, 228, offsetof(mavlink_trajectory_representation_waypoints_t, command) }, \
         } \
}
#endif

/**
 * @brief Pack a trajectory_representation_waypoints message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param valid_points  Number of valid points (up-to 5 waypoints are possible)
 * @param pos_x [m] X-coordinate of waypoint, set to NaN if not being used
 * @param pos_y [m] Y-coordinate of waypoint, set to NaN if not being used
 * @param pos_z [m] Z-coordinate of waypoint, set to NaN if not being used
 * @param vel_x [m/s] X-velocity of waypoint, set to NaN if not being used
 * @param vel_y [m/s] Y-velocity of waypoint, set to NaN if not being used
 * @param vel_z [m/s] Z-velocity of waypoint, set to NaN if not being used
 * @param acc_x [m/s/s] X-acceleration of waypoint, set to NaN if not being used
 * @param acc_y [m/s/s] Y-acceleration of waypoint, set to NaN if not being used
 * @param acc_z [m/s/s] Z-acceleration of waypoint, set to NaN if not being used
 * @param pos_yaw [rad] Yaw angle, set to NaN if not being used
 * @param vel_yaw [rad/s] Yaw rate, set to NaN if not being used
 * @param command  MAV_CMD command id of waypoint, set to UINT16_MAX if not being used.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_trajectory_representation_waypoints_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, uint8_t valid_points, const float *pos_x, const float *pos_y, const float *pos_z, const float *vel_x, const float *vel_y, const float *vel_z, const float *acc_x, const float *acc_y, const float *acc_z, const float *pos_yaw, const float *vel_yaw, const uint16_t *command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 238, valid_points);
    _mav_put_float_array(buf, 8, pos_x, 5);
    _mav_put_float_array(buf, 28, pos_y, 5);
    _mav_put_float_array(buf, 48, pos_z, 5);
    _mav_put_float_array(buf, 68, vel_x, 5);
    _mav_put_float_array(buf, 88, vel_y, 5);
    _mav_put_float_array(buf, 108, vel_z, 5);
    _mav_put_float_array(buf, 128, acc_x, 5);
    _mav_put_float_array(buf, 148, acc_y, 5);
    _mav_put_float_array(buf, 168, acc_z, 5);
    _mav_put_float_array(buf, 188, pos_yaw, 5);
    _mav_put_float_array(buf, 208, vel_yaw, 5);
    _mav_put_uint16_t_array(buf, 228, command, 5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_LEN);
#else
    mavlink_trajectory_representation_waypoints_t packet;
    packet.time_usec = time_usec;
    packet.valid_points = valid_points;
    mav_array_memcpy(packet.pos_x, pos_x, sizeof(float)*5);
    mav_array_memcpy(packet.pos_y, pos_y, sizeof(float)*5);
    mav_array_memcpy(packet.pos_z, pos_z, sizeof(float)*5);
    mav_array_memcpy(packet.vel_x, vel_x, sizeof(float)*5);
    mav_array_memcpy(packet.vel_y, vel_y, sizeof(float)*5);
    mav_array_memcpy(packet.vel_z, vel_z, sizeof(float)*5);
    mav_array_memcpy(packet.acc_x, acc_x, sizeof(float)*5);
    mav_array_memcpy(packet.acc_y, acc_y, sizeof(float)*5);
    mav_array_memcpy(packet.acc_z, acc_z, sizeof(float)*5);
    mav_array_memcpy(packet.pos_yaw, pos_yaw, sizeof(float)*5);
    mav_array_memcpy(packet.vel_yaw, vel_yaw, sizeof(float)*5);
    mav_array_memcpy(packet.command, command, sizeof(uint16_t)*5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_LEN, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_CRC);
}

/**
 * @brief Pack a trajectory_representation_waypoints message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param valid_points  Number of valid points (up-to 5 waypoints are possible)
 * @param pos_x [m] X-coordinate of waypoint, set to NaN if not being used
 * @param pos_y [m] Y-coordinate of waypoint, set to NaN if not being used
 * @param pos_z [m] Z-coordinate of waypoint, set to NaN if not being used
 * @param vel_x [m/s] X-velocity of waypoint, set to NaN if not being used
 * @param vel_y [m/s] Y-velocity of waypoint, set to NaN if not being used
 * @param vel_z [m/s] Z-velocity of waypoint, set to NaN if not being used
 * @param acc_x [m/s/s] X-acceleration of waypoint, set to NaN if not being used
 * @param acc_y [m/s/s] Y-acceleration of waypoint, set to NaN if not being used
 * @param acc_z [m/s/s] Z-acceleration of waypoint, set to NaN if not being used
 * @param pos_yaw [rad] Yaw angle, set to NaN if not being used
 * @param vel_yaw [rad/s] Yaw rate, set to NaN if not being used
 * @param command  MAV_CMD command id of waypoint, set to UINT16_MAX if not being used.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_trajectory_representation_waypoints_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t time_usec, uint8_t valid_points, const float *pos_x, const float *pos_y, const float *pos_z, const float *vel_x, const float *vel_y, const float *vel_z, const float *acc_x, const float *acc_y, const float *acc_z, const float *pos_yaw, const float *vel_yaw, const uint16_t *command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 238, valid_points);
    _mav_put_float_array(buf, 8, pos_x, 5);
    _mav_put_float_array(buf, 28, pos_y, 5);
    _mav_put_float_array(buf, 48, pos_z, 5);
    _mav_put_float_array(buf, 68, vel_x, 5);
    _mav_put_float_array(buf, 88, vel_y, 5);
    _mav_put_float_array(buf, 108, vel_z, 5);
    _mav_put_float_array(buf, 128, acc_x, 5);
    _mav_put_float_array(buf, 148, acc_y, 5);
    _mav_put_float_array(buf, 168, acc_z, 5);
    _mav_put_float_array(buf, 188, pos_yaw, 5);
    _mav_put_float_array(buf, 208, vel_yaw, 5);
    _mav_put_uint16_t_array(buf, 228, command, 5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_LEN);
#else
    mavlink_trajectory_representation_waypoints_t packet;
    packet.time_usec = time_usec;
    packet.valid_points = valid_points;
    mav_array_memcpy(packet.pos_x, pos_x, sizeof(float)*5);
    mav_array_memcpy(packet.pos_y, pos_y, sizeof(float)*5);
    mav_array_memcpy(packet.pos_z, pos_z, sizeof(float)*5);
    mav_array_memcpy(packet.vel_x, vel_x, sizeof(float)*5);
    mav_array_memcpy(packet.vel_y, vel_y, sizeof(float)*5);
    mav_array_memcpy(packet.vel_z, vel_z, sizeof(float)*5);
    mav_array_memcpy(packet.acc_x, acc_x, sizeof(float)*5);
    mav_array_memcpy(packet.acc_y, acc_y, sizeof(float)*5);
    mav_array_memcpy(packet.acc_z, acc_z, sizeof(float)*5);
    mav_array_memcpy(packet.pos_yaw, pos_yaw, sizeof(float)*5);
    mav_array_memcpy(packet.vel_yaw, vel_yaw, sizeof(float)*5);
    mav_array_memcpy(packet.command, command, sizeof(uint16_t)*5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_LEN, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_LEN);
#endif
}

/**
 * @brief Pack a trajectory_representation_waypoints message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param valid_points  Number of valid points (up-to 5 waypoints are possible)
 * @param pos_x [m] X-coordinate of waypoint, set to NaN if not being used
 * @param pos_y [m] Y-coordinate of waypoint, set to NaN if not being used
 * @param pos_z [m] Z-coordinate of waypoint, set to NaN if not being used
 * @param vel_x [m/s] X-velocity of waypoint, set to NaN if not being used
 * @param vel_y [m/s] Y-velocity of waypoint, set to NaN if not being used
 * @param vel_z [m/s] Z-velocity of waypoint, set to NaN if not being used
 * @param acc_x [m/s/s] X-acceleration of waypoint, set to NaN if not being used
 * @param acc_y [m/s/s] Y-acceleration of waypoint, set to NaN if not being used
 * @param acc_z [m/s/s] Z-acceleration of waypoint, set to NaN if not being used
 * @param pos_yaw [rad] Yaw angle, set to NaN if not being used
 * @param vel_yaw [rad/s] Yaw rate, set to NaN if not being used
 * @param command  MAV_CMD command id of waypoint, set to UINT16_MAX if not being used.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_trajectory_representation_waypoints_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,uint8_t valid_points,const float *pos_x,const float *pos_y,const float *pos_z,const float *vel_x,const float *vel_y,const float *vel_z,const float *acc_x,const float *acc_y,const float *acc_z,const float *pos_yaw,const float *vel_yaw,const uint16_t *command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 238, valid_points);
    _mav_put_float_array(buf, 8, pos_x, 5);
    _mav_put_float_array(buf, 28, pos_y, 5);
    _mav_put_float_array(buf, 48, pos_z, 5);
    _mav_put_float_array(buf, 68, vel_x, 5);
    _mav_put_float_array(buf, 88, vel_y, 5);
    _mav_put_float_array(buf, 108, vel_z, 5);
    _mav_put_float_array(buf, 128, acc_x, 5);
    _mav_put_float_array(buf, 148, acc_y, 5);
    _mav_put_float_array(buf, 168, acc_z, 5);
    _mav_put_float_array(buf, 188, pos_yaw, 5);
    _mav_put_float_array(buf, 208, vel_yaw, 5);
    _mav_put_uint16_t_array(buf, 228, command, 5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_LEN);
#else
    mavlink_trajectory_representation_waypoints_t packet;
    packet.time_usec = time_usec;
    packet.valid_points = valid_points;
    mav_array_memcpy(packet.pos_x, pos_x, sizeof(float)*5);
    mav_array_memcpy(packet.pos_y, pos_y, sizeof(float)*5);
    mav_array_memcpy(packet.pos_z, pos_z, sizeof(float)*5);
    mav_array_memcpy(packet.vel_x, vel_x, sizeof(float)*5);
    mav_array_memcpy(packet.vel_y, vel_y, sizeof(float)*5);
    mav_array_memcpy(packet.vel_z, vel_z, sizeof(float)*5);
    mav_array_memcpy(packet.acc_x, acc_x, sizeof(float)*5);
    mav_array_memcpy(packet.acc_y, acc_y, sizeof(float)*5);
    mav_array_memcpy(packet.acc_z, acc_z, sizeof(float)*5);
    mav_array_memcpy(packet.pos_yaw, pos_yaw, sizeof(float)*5);
    mav_array_memcpy(packet.vel_yaw, vel_yaw, sizeof(float)*5);
    mav_array_memcpy(packet.command, command, sizeof(uint16_t)*5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_LEN, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_CRC);
}

/**
 * @brief Encode a trajectory_representation_waypoints struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param trajectory_representation_waypoints C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_trajectory_representation_waypoints_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_trajectory_representation_waypoints_t* trajectory_representation_waypoints)
{
    return mavlink_msg_trajectory_representation_waypoints_pack(system_id, component_id, msg, trajectory_representation_waypoints->time_usec, trajectory_representation_waypoints->valid_points, trajectory_representation_waypoints->pos_x, trajectory_representation_waypoints->pos_y, trajectory_representation_waypoints->pos_z, trajectory_representation_waypoints->vel_x, trajectory_representation_waypoints->vel_y, trajectory_representation_waypoints->vel_z, trajectory_representation_waypoints->acc_x, trajectory_representation_waypoints->acc_y, trajectory_representation_waypoints->acc_z, trajectory_representation_waypoints->pos_yaw, trajectory_representation_waypoints->vel_yaw, trajectory_representation_waypoints->command);
}

/**
 * @brief Encode a trajectory_representation_waypoints struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param trajectory_representation_waypoints C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_trajectory_representation_waypoints_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_trajectory_representation_waypoints_t* trajectory_representation_waypoints)
{
    return mavlink_msg_trajectory_representation_waypoints_pack_chan(system_id, component_id, chan, msg, trajectory_representation_waypoints->time_usec, trajectory_representation_waypoints->valid_points, trajectory_representation_waypoints->pos_x, trajectory_representation_waypoints->pos_y, trajectory_representation_waypoints->pos_z, trajectory_representation_waypoints->vel_x, trajectory_representation_waypoints->vel_y, trajectory_representation_waypoints->vel_z, trajectory_representation_waypoints->acc_x, trajectory_representation_waypoints->acc_y, trajectory_representation_waypoints->acc_z, trajectory_representation_waypoints->pos_yaw, trajectory_representation_waypoints->vel_yaw, trajectory_representation_waypoints->command);
}

/**
 * @brief Encode a trajectory_representation_waypoints struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param trajectory_representation_waypoints C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_trajectory_representation_waypoints_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_trajectory_representation_waypoints_t* trajectory_representation_waypoints)
{
    return mavlink_msg_trajectory_representation_waypoints_pack_status(system_id, component_id, _status, msg,  trajectory_representation_waypoints->time_usec, trajectory_representation_waypoints->valid_points, trajectory_representation_waypoints->pos_x, trajectory_representation_waypoints->pos_y, trajectory_representation_waypoints->pos_z, trajectory_representation_waypoints->vel_x, trajectory_representation_waypoints->vel_y, trajectory_representation_waypoints->vel_z, trajectory_representation_waypoints->acc_x, trajectory_representation_waypoints->acc_y, trajectory_representation_waypoints->acc_z, trajectory_representation_waypoints->pos_yaw, trajectory_representation_waypoints->vel_yaw, trajectory_representation_waypoints->command);
}

/**
 * @brief Send a trajectory_representation_waypoints message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param valid_points  Number of valid points (up-to 5 waypoints are possible)
 * @param pos_x [m] X-coordinate of waypoint, set to NaN if not being used
 * @param pos_y [m] Y-coordinate of waypoint, set to NaN if not being used
 * @param pos_z [m] Z-coordinate of waypoint, set to NaN if not being used
 * @param vel_x [m/s] X-velocity of waypoint, set to NaN if not being used
 * @param vel_y [m/s] Y-velocity of waypoint, set to NaN if not being used
 * @param vel_z [m/s] Z-velocity of waypoint, set to NaN if not being used
 * @param acc_x [m/s/s] X-acceleration of waypoint, set to NaN if not being used
 * @param acc_y [m/s/s] Y-acceleration of waypoint, set to NaN if not being used
 * @param acc_z [m/s/s] Z-acceleration of waypoint, set to NaN if not being used
 * @param pos_yaw [rad] Yaw angle, set to NaN if not being used
 * @param vel_yaw [rad/s] Yaw rate, set to NaN if not being used
 * @param command  MAV_CMD command id of waypoint, set to UINT16_MAX if not being used.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_trajectory_representation_waypoints_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t valid_points, const float *pos_x, const float *pos_y, const float *pos_z, const float *vel_x, const float *vel_y, const float *vel_z, const float *acc_x, const float *acc_y, const float *acc_z, const float *pos_yaw, const float *vel_yaw, const uint16_t *command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 238, valid_points);
    _mav_put_float_array(buf, 8, pos_x, 5);
    _mav_put_float_array(buf, 28, pos_y, 5);
    _mav_put_float_array(buf, 48, pos_z, 5);
    _mav_put_float_array(buf, 68, vel_x, 5);
    _mav_put_float_array(buf, 88, vel_y, 5);
    _mav_put_float_array(buf, 108, vel_z, 5);
    _mav_put_float_array(buf, 128, acc_x, 5);
    _mav_put_float_array(buf, 148, acc_y, 5);
    _mav_put_float_array(buf, 168, acc_z, 5);
    _mav_put_float_array(buf, 188, pos_yaw, 5);
    _mav_put_float_array(buf, 208, vel_yaw, 5);
    _mav_put_uint16_t_array(buf, 228, command, 5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS, buf, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_LEN, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_CRC);
#else
    mavlink_trajectory_representation_waypoints_t packet;
    packet.time_usec = time_usec;
    packet.valid_points = valid_points;
    mav_array_memcpy(packet.pos_x, pos_x, sizeof(float)*5);
    mav_array_memcpy(packet.pos_y, pos_y, sizeof(float)*5);
    mav_array_memcpy(packet.pos_z, pos_z, sizeof(float)*5);
    mav_array_memcpy(packet.vel_x, vel_x, sizeof(float)*5);
    mav_array_memcpy(packet.vel_y, vel_y, sizeof(float)*5);
    mav_array_memcpy(packet.vel_z, vel_z, sizeof(float)*5);
    mav_array_memcpy(packet.acc_x, acc_x, sizeof(float)*5);
    mav_array_memcpy(packet.acc_y, acc_y, sizeof(float)*5);
    mav_array_memcpy(packet.acc_z, acc_z, sizeof(float)*5);
    mav_array_memcpy(packet.pos_yaw, pos_yaw, sizeof(float)*5);
    mav_array_memcpy(packet.vel_yaw, vel_yaw, sizeof(float)*5);
    mav_array_memcpy(packet.command, command, sizeof(uint16_t)*5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS, (const char *)&packet, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_LEN, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_CRC);
#endif
}

/**
 * @brief Send a trajectory_representation_waypoints message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_trajectory_representation_waypoints_send_struct(mavlink_channel_t chan, const mavlink_trajectory_representation_waypoints_t* trajectory_representation_waypoints)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_trajectory_representation_waypoints_send(chan, trajectory_representation_waypoints->time_usec, trajectory_representation_waypoints->valid_points, trajectory_representation_waypoints->pos_x, trajectory_representation_waypoints->pos_y, trajectory_representation_waypoints->pos_z, trajectory_representation_waypoints->vel_x, trajectory_representation_waypoints->vel_y, trajectory_representation_waypoints->vel_z, trajectory_representation_waypoints->acc_x, trajectory_representation_waypoints->acc_y, trajectory_representation_waypoints->acc_z, trajectory_representation_waypoints->pos_yaw, trajectory_representation_waypoints->vel_yaw, trajectory_representation_waypoints->command);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS, (const char *)trajectory_representation_waypoints, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_LEN, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_CRC);
#endif
}

#if MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_trajectory_representation_waypoints_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint8_t valid_points, const float *pos_x, const float *pos_y, const float *pos_z, const float *vel_x, const float *vel_y, const float *vel_z, const float *acc_x, const float *acc_y, const float *acc_z, const float *pos_yaw, const float *vel_yaw, const uint16_t *command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 238, valid_points);
    _mav_put_float_array(buf, 8, pos_x, 5);
    _mav_put_float_array(buf, 28, pos_y, 5);
    _mav_put_float_array(buf, 48, pos_z, 5);
    _mav_put_float_array(buf, 68, vel_x, 5);
    _mav_put_float_array(buf, 88, vel_y, 5);
    _mav_put_float_array(buf, 108, vel_z, 5);
    _mav_put_float_array(buf, 128, acc_x, 5);
    _mav_put_float_array(buf, 148, acc_y, 5);
    _mav_put_float_array(buf, 168, acc_z, 5);
    _mav_put_float_array(buf, 188, pos_yaw, 5);
    _mav_put_float_array(buf, 208, vel_yaw, 5);
    _mav_put_uint16_t_array(buf, 228, command, 5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS, buf, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_LEN, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_CRC);
#else
    mavlink_trajectory_representation_waypoints_t *packet = (mavlink_trajectory_representation_waypoints_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->valid_points = valid_points;
    mav_array_memcpy(packet->pos_x, pos_x, sizeof(float)*5);
    mav_array_memcpy(packet->pos_y, pos_y, sizeof(float)*5);
    mav_array_memcpy(packet->pos_z, pos_z, sizeof(float)*5);
    mav_array_memcpy(packet->vel_x, vel_x, sizeof(float)*5);
    mav_array_memcpy(packet->vel_y, vel_y, sizeof(float)*5);
    mav_array_memcpy(packet->vel_z, vel_z, sizeof(float)*5);
    mav_array_memcpy(packet->acc_x, acc_x, sizeof(float)*5);
    mav_array_memcpy(packet->acc_y, acc_y, sizeof(float)*5);
    mav_array_memcpy(packet->acc_z, acc_z, sizeof(float)*5);
    mav_array_memcpy(packet->pos_yaw, pos_yaw, sizeof(float)*5);
    mav_array_memcpy(packet->vel_yaw, vel_yaw, sizeof(float)*5);
    mav_array_memcpy(packet->command, command, sizeof(uint16_t)*5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS, (const char *)packet, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_LEN, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_CRC);
#endif
}
#endif

#endif

// MESSAGE TRAJECTORY_REPRESENTATION_WAYPOINTS UNPACKING


/**
 * @brief Get field time_usec from trajectory_representation_waypoints message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
static inline uint64_t mavlink_msg_trajectory_representation_waypoints_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field valid_points from trajectory_representation_waypoints message
 *
 * @return  Number of valid points (up-to 5 waypoints are possible)
 */
static inline uint8_t mavlink_msg_trajectory_representation_waypoints_get_valid_points(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  238);
}

/**
 * @brief Get field pos_x from trajectory_representation_waypoints message
 *
 * @return [m] X-coordinate of waypoint, set to NaN if not being used
 */
static inline uint16_t mavlink_msg_trajectory_representation_waypoints_get_pos_x(const mavlink_message_t* msg, float *pos_x)
{
    return _MAV_RETURN_float_array(msg, pos_x, 5,  8);
}

/**
 * @brief Get field pos_y from trajectory_representation_waypoints message
 *
 * @return [m] Y-coordinate of waypoint, set to NaN if not being used
 */
static inline uint16_t mavlink_msg_trajectory_representation_waypoints_get_pos_y(const mavlink_message_t* msg, float *pos_y)
{
    return _MAV_RETURN_float_array(msg, pos_y, 5,  28);
}

/**
 * @brief Get field pos_z from trajectory_representation_waypoints message
 *
 * @return [m] Z-coordinate of waypoint, set to NaN if not being used
 */
static inline uint16_t mavlink_msg_trajectory_representation_waypoints_get_pos_z(const mavlink_message_t* msg, float *pos_z)
{
    return _MAV_RETURN_float_array(msg, pos_z, 5,  48);
}

/**
 * @brief Get field vel_x from trajectory_representation_waypoints message
 *
 * @return [m/s] X-velocity of waypoint, set to NaN if not being used
 */
static inline uint16_t mavlink_msg_trajectory_representation_waypoints_get_vel_x(const mavlink_message_t* msg, float *vel_x)
{
    return _MAV_RETURN_float_array(msg, vel_x, 5,  68);
}

/**
 * @brief Get field vel_y from trajectory_representation_waypoints message
 *
 * @return [m/s] Y-velocity of waypoint, set to NaN if not being used
 */
static inline uint16_t mavlink_msg_trajectory_representation_waypoints_get_vel_y(const mavlink_message_t* msg, float *vel_y)
{
    return _MAV_RETURN_float_array(msg, vel_y, 5,  88);
}

/**
 * @brief Get field vel_z from trajectory_representation_waypoints message
 *
 * @return [m/s] Z-velocity of waypoint, set to NaN if not being used
 */
static inline uint16_t mavlink_msg_trajectory_representation_waypoints_get_vel_z(const mavlink_message_t* msg, float *vel_z)
{
    return _MAV_RETURN_float_array(msg, vel_z, 5,  108);
}

/**
 * @brief Get field acc_x from trajectory_representation_waypoints message
 *
 * @return [m/s/s] X-acceleration of waypoint, set to NaN if not being used
 */
static inline uint16_t mavlink_msg_trajectory_representation_waypoints_get_acc_x(const mavlink_message_t* msg, float *acc_x)
{
    return _MAV_RETURN_float_array(msg, acc_x, 5,  128);
}

/**
 * @brief Get field acc_y from trajectory_representation_waypoints message
 *
 * @return [m/s/s] Y-acceleration of waypoint, set to NaN if not being used
 */
static inline uint16_t mavlink_msg_trajectory_representation_waypoints_get_acc_y(const mavlink_message_t* msg, float *acc_y)
{
    return _MAV_RETURN_float_array(msg, acc_y, 5,  148);
}

/**
 * @brief Get field acc_z from trajectory_representation_waypoints message
 *
 * @return [m/s/s] Z-acceleration of waypoint, set to NaN if not being used
 */
static inline uint16_t mavlink_msg_trajectory_representation_waypoints_get_acc_z(const mavlink_message_t* msg, float *acc_z)
{
    return _MAV_RETURN_float_array(msg, acc_z, 5,  168);
}

/**
 * @brief Get field pos_yaw from trajectory_representation_waypoints message
 *
 * @return [rad] Yaw angle, set to NaN if not being used
 */
static inline uint16_t mavlink_msg_trajectory_representation_waypoints_get_pos_yaw(const mavlink_message_t* msg, float *pos_yaw)
{
    return _MAV_RETURN_float_array(msg, pos_yaw, 5,  188);
}

/**
 * @brief Get field vel_yaw from trajectory_representation_waypoints message
 *
 * @return [rad/s] Yaw rate, set to NaN if not being used
 */
static inline uint16_t mavlink_msg_trajectory_representation_waypoints_get_vel_yaw(const mavlink_message_t* msg, float *vel_yaw)
{
    return _MAV_RETURN_float_array(msg, vel_yaw, 5,  208);
}

/**
 * @brief Get field command from trajectory_representation_waypoints message
 *
 * @return  MAV_CMD command id of waypoint, set to UINT16_MAX if not being used.
 */
static inline uint16_t mavlink_msg_trajectory_representation_waypoints_get_command(const mavlink_message_t* msg, uint16_t *command)
{
    return _MAV_RETURN_uint16_t_array(msg, command, 5,  228);
}

/**
 * @brief Decode a trajectory_representation_waypoints message into a struct
 *
 * @param msg The message to decode
 * @param trajectory_representation_waypoints C-struct to decode the message contents into
 */
static inline void mavlink_msg_trajectory_representation_waypoints_decode(const mavlink_message_t* msg, mavlink_trajectory_representation_waypoints_t* trajectory_representation_waypoints)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    trajectory_representation_waypoints->time_usec = mavlink_msg_trajectory_representation_waypoints_get_time_usec(msg);
    mavlink_msg_trajectory_representation_waypoints_get_pos_x(msg, trajectory_representation_waypoints->pos_x);
    mavlink_msg_trajectory_representation_waypoints_get_pos_y(msg, trajectory_representation_waypoints->pos_y);
    mavlink_msg_trajectory_representation_waypoints_get_pos_z(msg, trajectory_representation_waypoints->pos_z);
    mavlink_msg_trajectory_representation_waypoints_get_vel_x(msg, trajectory_representation_waypoints->vel_x);
    mavlink_msg_trajectory_representation_waypoints_get_vel_y(msg, trajectory_representation_waypoints->vel_y);
    mavlink_msg_trajectory_representation_waypoints_get_vel_z(msg, trajectory_representation_waypoints->vel_z);
    mavlink_msg_trajectory_representation_waypoints_get_acc_x(msg, trajectory_representation_waypoints->acc_x);
    mavlink_msg_trajectory_representation_waypoints_get_acc_y(msg, trajectory_representation_waypoints->acc_y);
    mavlink_msg_trajectory_representation_waypoints_get_acc_z(msg, trajectory_representation_waypoints->acc_z);
    mavlink_msg_trajectory_representation_waypoints_get_pos_yaw(msg, trajectory_representation_waypoints->pos_yaw);
    mavlink_msg_trajectory_representation_waypoints_get_vel_yaw(msg, trajectory_representation_waypoints->vel_yaw);
    mavlink_msg_trajectory_representation_waypoints_get_command(msg, trajectory_representation_waypoints->command);
    trajectory_representation_waypoints->valid_points = mavlink_msg_trajectory_representation_waypoints_get_valid_points(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_LEN? msg->len : MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_LEN;
        memset(trajectory_representation_waypoints, 0, MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_LEN);
    memcpy(trajectory_representation_waypoints, _MAV_PAYLOAD(msg), len);
#endif
}
