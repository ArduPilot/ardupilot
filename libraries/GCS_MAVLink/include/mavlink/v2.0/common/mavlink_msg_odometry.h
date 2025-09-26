#pragma once
// MESSAGE ODOMETRY PACKING

#define MAVLINK_MSG_ID_ODOMETRY 331


typedef struct __mavlink_odometry_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 float x; /*< [m] X Position*/
 float y; /*< [m] Y Position*/
 float z; /*< [m] Z Position*/
 float q[4]; /*<  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)*/
 float vx; /*< [m/s] X linear speed*/
 float vy; /*< [m/s] Y linear speed*/
 float vz; /*< [m/s] Z linear speed*/
 float rollspeed; /*< [rad/s] Roll angular speed*/
 float pitchspeed; /*< [rad/s] Pitch angular speed*/
 float yawspeed; /*< [rad/s] Yaw angular speed*/
 float pose_covariance[21]; /*<  Row-major representation of a 6x6 pose cross-covariance matrix upper right triangle (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.*/
 float velocity_covariance[21]; /*<  Row-major representation of a 6x6 velocity cross-covariance matrix upper right triangle (states: vx, vy, vz, rollspeed, pitchspeed, yawspeed; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.*/
 uint8_t frame_id; /*<  Coordinate frame of reference for the pose data.*/
 uint8_t child_frame_id; /*<  Coordinate frame of reference for the velocity in free space (twist) data.*/
 uint8_t reset_counter; /*<  Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions (position, velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM system detects a loop-closure and the estimate jumps.*/
 uint8_t estimator_type; /*<  Type of estimator that is providing the odometry.*/
 int8_t quality; /*< [%] Optional odometry quality metric as a percentage. -1 = odometry has failed, 0 = unknown/unset quality, 1 = worst quality, 100 = best quality*/
} mavlink_odometry_t;

#define MAVLINK_MSG_ID_ODOMETRY_LEN 233
#define MAVLINK_MSG_ID_ODOMETRY_MIN_LEN 230
#define MAVLINK_MSG_ID_331_LEN 233
#define MAVLINK_MSG_ID_331_MIN_LEN 230

#define MAVLINK_MSG_ID_ODOMETRY_CRC 91
#define MAVLINK_MSG_ID_331_CRC 91

#define MAVLINK_MSG_ODOMETRY_FIELD_Q_LEN 4
#define MAVLINK_MSG_ODOMETRY_FIELD_POSE_COVARIANCE_LEN 21
#define MAVLINK_MSG_ODOMETRY_FIELD_VELOCITY_COVARIANCE_LEN 21

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ODOMETRY { \
    331, \
    "ODOMETRY", \
    18, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_odometry_t, time_usec) }, \
         { "frame_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 228, offsetof(mavlink_odometry_t, frame_id) }, \
         { "child_frame_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 229, offsetof(mavlink_odometry_t, child_frame_id) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_odometry_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_odometry_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_odometry_t, z) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 4, 20, offsetof(mavlink_odometry_t, q) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_odometry_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_odometry_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_odometry_t, vz) }, \
         { "rollspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_odometry_t, rollspeed) }, \
         { "pitchspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_odometry_t, pitchspeed) }, \
         { "yawspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_odometry_t, yawspeed) }, \
         { "pose_covariance", NULL, MAVLINK_TYPE_FLOAT, 21, 60, offsetof(mavlink_odometry_t, pose_covariance) }, \
         { "velocity_covariance", NULL, MAVLINK_TYPE_FLOAT, 21, 144, offsetof(mavlink_odometry_t, velocity_covariance) }, \
         { "reset_counter", NULL, MAVLINK_TYPE_UINT8_T, 0, 230, offsetof(mavlink_odometry_t, reset_counter) }, \
         { "estimator_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 231, offsetof(mavlink_odometry_t, estimator_type) }, \
         { "quality", NULL, MAVLINK_TYPE_INT8_T, 0, 232, offsetof(mavlink_odometry_t, quality) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ODOMETRY { \
    "ODOMETRY", \
    18, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_odometry_t, time_usec) }, \
         { "frame_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 228, offsetof(mavlink_odometry_t, frame_id) }, \
         { "child_frame_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 229, offsetof(mavlink_odometry_t, child_frame_id) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_odometry_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_odometry_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_odometry_t, z) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 4, 20, offsetof(mavlink_odometry_t, q) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_odometry_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_odometry_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_odometry_t, vz) }, \
         { "rollspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_odometry_t, rollspeed) }, \
         { "pitchspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_odometry_t, pitchspeed) }, \
         { "yawspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_odometry_t, yawspeed) }, \
         { "pose_covariance", NULL, MAVLINK_TYPE_FLOAT, 21, 60, offsetof(mavlink_odometry_t, pose_covariance) }, \
         { "velocity_covariance", NULL, MAVLINK_TYPE_FLOAT, 21, 144, offsetof(mavlink_odometry_t, velocity_covariance) }, \
         { "reset_counter", NULL, MAVLINK_TYPE_UINT8_T, 0, 230, offsetof(mavlink_odometry_t, reset_counter) }, \
         { "estimator_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 231, offsetof(mavlink_odometry_t, estimator_type) }, \
         { "quality", NULL, MAVLINK_TYPE_INT8_T, 0, 232, offsetof(mavlink_odometry_t, quality) }, \
         } \
}
#endif

/**
 * @brief Pack a odometry message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param frame_id  Coordinate frame of reference for the pose data.
 * @param child_frame_id  Coordinate frame of reference for the velocity in free space (twist) data.
 * @param x [m] X Position
 * @param y [m] Y Position
 * @param z [m] Z Position
 * @param q  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
 * @param vx [m/s] X linear speed
 * @param vy [m/s] Y linear speed
 * @param vz [m/s] Z linear speed
 * @param rollspeed [rad/s] Roll angular speed
 * @param pitchspeed [rad/s] Pitch angular speed
 * @param yawspeed [rad/s] Yaw angular speed
 * @param pose_covariance  Row-major representation of a 6x6 pose cross-covariance matrix upper right triangle (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.
 * @param velocity_covariance  Row-major representation of a 6x6 velocity cross-covariance matrix upper right triangle (states: vx, vy, vz, rollspeed, pitchspeed, yawspeed; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.
 * @param reset_counter  Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions (position, velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM system detects a loop-closure and the estimate jumps.
 * @param estimator_type  Type of estimator that is providing the odometry.
 * @param quality [%] Optional odometry quality metric as a percentage. -1 = odometry has failed, 0 = unknown/unset quality, 1 = worst quality, 100 = best quality
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_odometry_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, uint8_t frame_id, uint8_t child_frame_id, float x, float y, float z, const float *q, float vx, float vy, float vz, float rollspeed, float pitchspeed, float yawspeed, const float *pose_covariance, const float *velocity_covariance, uint8_t reset_counter, uint8_t estimator_type, int8_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ODOMETRY_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_float(buf, 36, vx);
    _mav_put_float(buf, 40, vy);
    _mav_put_float(buf, 44, vz);
    _mav_put_float(buf, 48, rollspeed);
    _mav_put_float(buf, 52, pitchspeed);
    _mav_put_float(buf, 56, yawspeed);
    _mav_put_uint8_t(buf, 228, frame_id);
    _mav_put_uint8_t(buf, 229, child_frame_id);
    _mav_put_uint8_t(buf, 230, reset_counter);
    _mav_put_uint8_t(buf, 231, estimator_type);
    _mav_put_int8_t(buf, 232, quality);
    _mav_put_float_array(buf, 20, q, 4);
    _mav_put_float_array(buf, 60, pose_covariance, 21);
    _mav_put_float_array(buf, 144, velocity_covariance, 21);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ODOMETRY_LEN);
#else
    mavlink_odometry_t packet;
    packet.time_usec = time_usec;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;
    packet.frame_id = frame_id;
    packet.child_frame_id = child_frame_id;
    packet.reset_counter = reset_counter;
    packet.estimator_type = estimator_type;
    packet.quality = quality;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
    mav_array_memcpy(packet.pose_covariance, pose_covariance, sizeof(float)*21);
    mav_array_memcpy(packet.velocity_covariance, velocity_covariance, sizeof(float)*21);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ODOMETRY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ODOMETRY;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ODOMETRY_MIN_LEN, MAVLINK_MSG_ID_ODOMETRY_LEN, MAVLINK_MSG_ID_ODOMETRY_CRC);
}

/**
 * @brief Pack a odometry message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param frame_id  Coordinate frame of reference for the pose data.
 * @param child_frame_id  Coordinate frame of reference for the velocity in free space (twist) data.
 * @param x [m] X Position
 * @param y [m] Y Position
 * @param z [m] Z Position
 * @param q  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
 * @param vx [m/s] X linear speed
 * @param vy [m/s] Y linear speed
 * @param vz [m/s] Z linear speed
 * @param rollspeed [rad/s] Roll angular speed
 * @param pitchspeed [rad/s] Pitch angular speed
 * @param yawspeed [rad/s] Yaw angular speed
 * @param pose_covariance  Row-major representation of a 6x6 pose cross-covariance matrix upper right triangle (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.
 * @param velocity_covariance  Row-major representation of a 6x6 velocity cross-covariance matrix upper right triangle (states: vx, vy, vz, rollspeed, pitchspeed, yawspeed; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.
 * @param reset_counter  Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions (position, velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM system detects a loop-closure and the estimate jumps.
 * @param estimator_type  Type of estimator that is providing the odometry.
 * @param quality [%] Optional odometry quality metric as a percentage. -1 = odometry has failed, 0 = unknown/unset quality, 1 = worst quality, 100 = best quality
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_odometry_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t time_usec, uint8_t frame_id, uint8_t child_frame_id, float x, float y, float z, const float *q, float vx, float vy, float vz, float rollspeed, float pitchspeed, float yawspeed, const float *pose_covariance, const float *velocity_covariance, uint8_t reset_counter, uint8_t estimator_type, int8_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ODOMETRY_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_float(buf, 36, vx);
    _mav_put_float(buf, 40, vy);
    _mav_put_float(buf, 44, vz);
    _mav_put_float(buf, 48, rollspeed);
    _mav_put_float(buf, 52, pitchspeed);
    _mav_put_float(buf, 56, yawspeed);
    _mav_put_uint8_t(buf, 228, frame_id);
    _mav_put_uint8_t(buf, 229, child_frame_id);
    _mav_put_uint8_t(buf, 230, reset_counter);
    _mav_put_uint8_t(buf, 231, estimator_type);
    _mav_put_int8_t(buf, 232, quality);
    _mav_put_float_array(buf, 20, q, 4);
    _mav_put_float_array(buf, 60, pose_covariance, 21);
    _mav_put_float_array(buf, 144, velocity_covariance, 21);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ODOMETRY_LEN);
#else
    mavlink_odometry_t packet;
    packet.time_usec = time_usec;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;
    packet.frame_id = frame_id;
    packet.child_frame_id = child_frame_id;
    packet.reset_counter = reset_counter;
    packet.estimator_type = estimator_type;
    packet.quality = quality;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
    mav_array_memcpy(packet.pose_covariance, pose_covariance, sizeof(float)*21);
    mav_array_memcpy(packet.velocity_covariance, velocity_covariance, sizeof(float)*21);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ODOMETRY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ODOMETRY;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ODOMETRY_MIN_LEN, MAVLINK_MSG_ID_ODOMETRY_LEN, MAVLINK_MSG_ID_ODOMETRY_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ODOMETRY_MIN_LEN, MAVLINK_MSG_ID_ODOMETRY_LEN);
#endif
}

/**
 * @brief Pack a odometry message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param frame_id  Coordinate frame of reference for the pose data.
 * @param child_frame_id  Coordinate frame of reference for the velocity in free space (twist) data.
 * @param x [m] X Position
 * @param y [m] Y Position
 * @param z [m] Z Position
 * @param q  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
 * @param vx [m/s] X linear speed
 * @param vy [m/s] Y linear speed
 * @param vz [m/s] Z linear speed
 * @param rollspeed [rad/s] Roll angular speed
 * @param pitchspeed [rad/s] Pitch angular speed
 * @param yawspeed [rad/s] Yaw angular speed
 * @param pose_covariance  Row-major representation of a 6x6 pose cross-covariance matrix upper right triangle (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.
 * @param velocity_covariance  Row-major representation of a 6x6 velocity cross-covariance matrix upper right triangle (states: vx, vy, vz, rollspeed, pitchspeed, yawspeed; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.
 * @param reset_counter  Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions (position, velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM system detects a loop-closure and the estimate jumps.
 * @param estimator_type  Type of estimator that is providing the odometry.
 * @param quality [%] Optional odometry quality metric as a percentage. -1 = odometry has failed, 0 = unknown/unset quality, 1 = worst quality, 100 = best quality
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_odometry_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,uint8_t frame_id,uint8_t child_frame_id,float x,float y,float z,const float *q,float vx,float vy,float vz,float rollspeed,float pitchspeed,float yawspeed,const float *pose_covariance,const float *velocity_covariance,uint8_t reset_counter,uint8_t estimator_type,int8_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ODOMETRY_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_float(buf, 36, vx);
    _mav_put_float(buf, 40, vy);
    _mav_put_float(buf, 44, vz);
    _mav_put_float(buf, 48, rollspeed);
    _mav_put_float(buf, 52, pitchspeed);
    _mav_put_float(buf, 56, yawspeed);
    _mav_put_uint8_t(buf, 228, frame_id);
    _mav_put_uint8_t(buf, 229, child_frame_id);
    _mav_put_uint8_t(buf, 230, reset_counter);
    _mav_put_uint8_t(buf, 231, estimator_type);
    _mav_put_int8_t(buf, 232, quality);
    _mav_put_float_array(buf, 20, q, 4);
    _mav_put_float_array(buf, 60, pose_covariance, 21);
    _mav_put_float_array(buf, 144, velocity_covariance, 21);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ODOMETRY_LEN);
#else
    mavlink_odometry_t packet;
    packet.time_usec = time_usec;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;
    packet.frame_id = frame_id;
    packet.child_frame_id = child_frame_id;
    packet.reset_counter = reset_counter;
    packet.estimator_type = estimator_type;
    packet.quality = quality;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
    mav_array_memcpy(packet.pose_covariance, pose_covariance, sizeof(float)*21);
    mav_array_memcpy(packet.velocity_covariance, velocity_covariance, sizeof(float)*21);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ODOMETRY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ODOMETRY;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ODOMETRY_MIN_LEN, MAVLINK_MSG_ID_ODOMETRY_LEN, MAVLINK_MSG_ID_ODOMETRY_CRC);
}

/**
 * @brief Encode a odometry struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param odometry C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_odometry_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_odometry_t* odometry)
{
    return mavlink_msg_odometry_pack(system_id, component_id, msg, odometry->time_usec, odometry->frame_id, odometry->child_frame_id, odometry->x, odometry->y, odometry->z, odometry->q, odometry->vx, odometry->vy, odometry->vz, odometry->rollspeed, odometry->pitchspeed, odometry->yawspeed, odometry->pose_covariance, odometry->velocity_covariance, odometry->reset_counter, odometry->estimator_type, odometry->quality);
}

/**
 * @brief Encode a odometry struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param odometry C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_odometry_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_odometry_t* odometry)
{
    return mavlink_msg_odometry_pack_chan(system_id, component_id, chan, msg, odometry->time_usec, odometry->frame_id, odometry->child_frame_id, odometry->x, odometry->y, odometry->z, odometry->q, odometry->vx, odometry->vy, odometry->vz, odometry->rollspeed, odometry->pitchspeed, odometry->yawspeed, odometry->pose_covariance, odometry->velocity_covariance, odometry->reset_counter, odometry->estimator_type, odometry->quality);
}

/**
 * @brief Encode a odometry struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param odometry C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_odometry_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_odometry_t* odometry)
{
    return mavlink_msg_odometry_pack_status(system_id, component_id, _status, msg,  odometry->time_usec, odometry->frame_id, odometry->child_frame_id, odometry->x, odometry->y, odometry->z, odometry->q, odometry->vx, odometry->vy, odometry->vz, odometry->rollspeed, odometry->pitchspeed, odometry->yawspeed, odometry->pose_covariance, odometry->velocity_covariance, odometry->reset_counter, odometry->estimator_type, odometry->quality);
}

/**
 * @brief Send a odometry message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param frame_id  Coordinate frame of reference for the pose data.
 * @param child_frame_id  Coordinate frame of reference for the velocity in free space (twist) data.
 * @param x [m] X Position
 * @param y [m] Y Position
 * @param z [m] Z Position
 * @param q  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
 * @param vx [m/s] X linear speed
 * @param vy [m/s] Y linear speed
 * @param vz [m/s] Z linear speed
 * @param rollspeed [rad/s] Roll angular speed
 * @param pitchspeed [rad/s] Pitch angular speed
 * @param yawspeed [rad/s] Yaw angular speed
 * @param pose_covariance  Row-major representation of a 6x6 pose cross-covariance matrix upper right triangle (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.
 * @param velocity_covariance  Row-major representation of a 6x6 velocity cross-covariance matrix upper right triangle (states: vx, vy, vz, rollspeed, pitchspeed, yawspeed; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.
 * @param reset_counter  Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions (position, velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM system detects a loop-closure and the estimate jumps.
 * @param estimator_type  Type of estimator that is providing the odometry.
 * @param quality [%] Optional odometry quality metric as a percentage. -1 = odometry has failed, 0 = unknown/unset quality, 1 = worst quality, 100 = best quality
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_odometry_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t frame_id, uint8_t child_frame_id, float x, float y, float z, const float *q, float vx, float vy, float vz, float rollspeed, float pitchspeed, float yawspeed, const float *pose_covariance, const float *velocity_covariance, uint8_t reset_counter, uint8_t estimator_type, int8_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ODOMETRY_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_float(buf, 36, vx);
    _mav_put_float(buf, 40, vy);
    _mav_put_float(buf, 44, vz);
    _mav_put_float(buf, 48, rollspeed);
    _mav_put_float(buf, 52, pitchspeed);
    _mav_put_float(buf, 56, yawspeed);
    _mav_put_uint8_t(buf, 228, frame_id);
    _mav_put_uint8_t(buf, 229, child_frame_id);
    _mav_put_uint8_t(buf, 230, reset_counter);
    _mav_put_uint8_t(buf, 231, estimator_type);
    _mav_put_int8_t(buf, 232, quality);
    _mav_put_float_array(buf, 20, q, 4);
    _mav_put_float_array(buf, 60, pose_covariance, 21);
    _mav_put_float_array(buf, 144, velocity_covariance, 21);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ODOMETRY, buf, MAVLINK_MSG_ID_ODOMETRY_MIN_LEN, MAVLINK_MSG_ID_ODOMETRY_LEN, MAVLINK_MSG_ID_ODOMETRY_CRC);
#else
    mavlink_odometry_t packet;
    packet.time_usec = time_usec;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;
    packet.frame_id = frame_id;
    packet.child_frame_id = child_frame_id;
    packet.reset_counter = reset_counter;
    packet.estimator_type = estimator_type;
    packet.quality = quality;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
    mav_array_memcpy(packet.pose_covariance, pose_covariance, sizeof(float)*21);
    mav_array_memcpy(packet.velocity_covariance, velocity_covariance, sizeof(float)*21);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ODOMETRY, (const char *)&packet, MAVLINK_MSG_ID_ODOMETRY_MIN_LEN, MAVLINK_MSG_ID_ODOMETRY_LEN, MAVLINK_MSG_ID_ODOMETRY_CRC);
#endif
}

/**
 * @brief Send a odometry message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_odometry_send_struct(mavlink_channel_t chan, const mavlink_odometry_t* odometry)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_odometry_send(chan, odometry->time_usec, odometry->frame_id, odometry->child_frame_id, odometry->x, odometry->y, odometry->z, odometry->q, odometry->vx, odometry->vy, odometry->vz, odometry->rollspeed, odometry->pitchspeed, odometry->yawspeed, odometry->pose_covariance, odometry->velocity_covariance, odometry->reset_counter, odometry->estimator_type, odometry->quality);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ODOMETRY, (const char *)odometry, MAVLINK_MSG_ID_ODOMETRY_MIN_LEN, MAVLINK_MSG_ID_ODOMETRY_LEN, MAVLINK_MSG_ID_ODOMETRY_CRC);
#endif
}

#if MAVLINK_MSG_ID_ODOMETRY_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_odometry_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint8_t frame_id, uint8_t child_frame_id, float x, float y, float z, const float *q, float vx, float vy, float vz, float rollspeed, float pitchspeed, float yawspeed, const float *pose_covariance, const float *velocity_covariance, uint8_t reset_counter, uint8_t estimator_type, int8_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_float(buf, 36, vx);
    _mav_put_float(buf, 40, vy);
    _mav_put_float(buf, 44, vz);
    _mav_put_float(buf, 48, rollspeed);
    _mav_put_float(buf, 52, pitchspeed);
    _mav_put_float(buf, 56, yawspeed);
    _mav_put_uint8_t(buf, 228, frame_id);
    _mav_put_uint8_t(buf, 229, child_frame_id);
    _mav_put_uint8_t(buf, 230, reset_counter);
    _mav_put_uint8_t(buf, 231, estimator_type);
    _mav_put_int8_t(buf, 232, quality);
    _mav_put_float_array(buf, 20, q, 4);
    _mav_put_float_array(buf, 60, pose_covariance, 21);
    _mav_put_float_array(buf, 144, velocity_covariance, 21);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ODOMETRY, buf, MAVLINK_MSG_ID_ODOMETRY_MIN_LEN, MAVLINK_MSG_ID_ODOMETRY_LEN, MAVLINK_MSG_ID_ODOMETRY_CRC);
#else
    mavlink_odometry_t *packet = (mavlink_odometry_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->vx = vx;
    packet->vy = vy;
    packet->vz = vz;
    packet->rollspeed = rollspeed;
    packet->pitchspeed = pitchspeed;
    packet->yawspeed = yawspeed;
    packet->frame_id = frame_id;
    packet->child_frame_id = child_frame_id;
    packet->reset_counter = reset_counter;
    packet->estimator_type = estimator_type;
    packet->quality = quality;
    mav_array_memcpy(packet->q, q, sizeof(float)*4);
    mav_array_memcpy(packet->pose_covariance, pose_covariance, sizeof(float)*21);
    mav_array_memcpy(packet->velocity_covariance, velocity_covariance, sizeof(float)*21);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ODOMETRY, (const char *)packet, MAVLINK_MSG_ID_ODOMETRY_MIN_LEN, MAVLINK_MSG_ID_ODOMETRY_LEN, MAVLINK_MSG_ID_ODOMETRY_CRC);
#endif
}
#endif

#endif

// MESSAGE ODOMETRY UNPACKING


/**
 * @brief Get field time_usec from odometry message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
static inline uint64_t mavlink_msg_odometry_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field frame_id from odometry message
 *
 * @return  Coordinate frame of reference for the pose data.
 */
static inline uint8_t mavlink_msg_odometry_get_frame_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  228);
}

/**
 * @brief Get field child_frame_id from odometry message
 *
 * @return  Coordinate frame of reference for the velocity in free space (twist) data.
 */
static inline uint8_t mavlink_msg_odometry_get_child_frame_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  229);
}

/**
 * @brief Get field x from odometry message
 *
 * @return [m] X Position
 */
static inline float mavlink_msg_odometry_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field y from odometry message
 *
 * @return [m] Y Position
 */
static inline float mavlink_msg_odometry_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field z from odometry message
 *
 * @return [m] Z Position
 */
static inline float mavlink_msg_odometry_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field q from odometry message
 *
 * @return  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
 */
static inline uint16_t mavlink_msg_odometry_get_q(const mavlink_message_t* msg, float *q)
{
    return _MAV_RETURN_float_array(msg, q, 4,  20);
}

/**
 * @brief Get field vx from odometry message
 *
 * @return [m/s] X linear speed
 */
static inline float mavlink_msg_odometry_get_vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field vy from odometry message
 *
 * @return [m/s] Y linear speed
 */
static inline float mavlink_msg_odometry_get_vy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field vz from odometry message
 *
 * @return [m/s] Z linear speed
 */
static inline float mavlink_msg_odometry_get_vz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field rollspeed from odometry message
 *
 * @return [rad/s] Roll angular speed
 */
static inline float mavlink_msg_odometry_get_rollspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field pitchspeed from odometry message
 *
 * @return [rad/s] Pitch angular speed
 */
static inline float mavlink_msg_odometry_get_pitchspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field yawspeed from odometry message
 *
 * @return [rad/s] Yaw angular speed
 */
static inline float mavlink_msg_odometry_get_yawspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field pose_covariance from odometry message
 *
 * @return  Row-major representation of a 6x6 pose cross-covariance matrix upper right triangle (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.
 */
static inline uint16_t mavlink_msg_odometry_get_pose_covariance(const mavlink_message_t* msg, float *pose_covariance)
{
    return _MAV_RETURN_float_array(msg, pose_covariance, 21,  60);
}

/**
 * @brief Get field velocity_covariance from odometry message
 *
 * @return  Row-major representation of a 6x6 velocity cross-covariance matrix upper right triangle (states: vx, vy, vz, rollspeed, pitchspeed, yawspeed; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.
 */
static inline uint16_t mavlink_msg_odometry_get_velocity_covariance(const mavlink_message_t* msg, float *velocity_covariance)
{
    return _MAV_RETURN_float_array(msg, velocity_covariance, 21,  144);
}

/**
 * @brief Get field reset_counter from odometry message
 *
 * @return  Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions (position, velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM system detects a loop-closure and the estimate jumps.
 */
static inline uint8_t mavlink_msg_odometry_get_reset_counter(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  230);
}

/**
 * @brief Get field estimator_type from odometry message
 *
 * @return  Type of estimator that is providing the odometry.
 */
static inline uint8_t mavlink_msg_odometry_get_estimator_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  231);
}

/**
 * @brief Get field quality from odometry message
 *
 * @return [%] Optional odometry quality metric as a percentage. -1 = odometry has failed, 0 = unknown/unset quality, 1 = worst quality, 100 = best quality
 */
static inline int8_t mavlink_msg_odometry_get_quality(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  232);
}

/**
 * @brief Decode a odometry message into a struct
 *
 * @param msg The message to decode
 * @param odometry C-struct to decode the message contents into
 */
static inline void mavlink_msg_odometry_decode(const mavlink_message_t* msg, mavlink_odometry_t* odometry)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    odometry->time_usec = mavlink_msg_odometry_get_time_usec(msg);
    odometry->x = mavlink_msg_odometry_get_x(msg);
    odometry->y = mavlink_msg_odometry_get_y(msg);
    odometry->z = mavlink_msg_odometry_get_z(msg);
    mavlink_msg_odometry_get_q(msg, odometry->q);
    odometry->vx = mavlink_msg_odometry_get_vx(msg);
    odometry->vy = mavlink_msg_odometry_get_vy(msg);
    odometry->vz = mavlink_msg_odometry_get_vz(msg);
    odometry->rollspeed = mavlink_msg_odometry_get_rollspeed(msg);
    odometry->pitchspeed = mavlink_msg_odometry_get_pitchspeed(msg);
    odometry->yawspeed = mavlink_msg_odometry_get_yawspeed(msg);
    mavlink_msg_odometry_get_pose_covariance(msg, odometry->pose_covariance);
    mavlink_msg_odometry_get_velocity_covariance(msg, odometry->velocity_covariance);
    odometry->frame_id = mavlink_msg_odometry_get_frame_id(msg);
    odometry->child_frame_id = mavlink_msg_odometry_get_child_frame_id(msg);
    odometry->reset_counter = mavlink_msg_odometry_get_reset_counter(msg);
    odometry->estimator_type = mavlink_msg_odometry_get_estimator_type(msg);
    odometry->quality = mavlink_msg_odometry_get_quality(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ODOMETRY_LEN? msg->len : MAVLINK_MSG_ID_ODOMETRY_LEN;
        memset(odometry, 0, MAVLINK_MSG_ID_ODOMETRY_LEN);
    memcpy(odometry, _MAV_PAYLOAD(msg), len);
#endif
}
