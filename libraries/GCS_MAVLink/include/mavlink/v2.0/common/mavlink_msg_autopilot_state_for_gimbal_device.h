#pragma once
// MESSAGE AUTOPILOT_STATE_FOR_GIMBAL_DEVICE PACKING

#define MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE 286

MAVPACKED(
typedef struct __mavlink_autopilot_state_for_gimbal_device_t {
 uint64_t time_boot_us; /*< [us] Timestamp (time since system boot).*/
 float q[4]; /*<  Quaternion components of autopilot attitude: w, x, y, z (1 0 0 0 is the null-rotation, Hamilton convention).*/
 uint32_t q_estimated_delay_us; /*< [us] Estimated delay of the attitude data. 0 if unknown.*/
 float vx; /*< [m/s] X Speed in NED (North, East, Down). NAN if unknown.*/
 float vy; /*< [m/s] Y Speed in NED (North, East, Down). NAN if unknown.*/
 float vz; /*< [m/s] Z Speed in NED (North, East, Down). NAN if unknown.*/
 uint32_t v_estimated_delay_us; /*< [us] Estimated delay of the speed data. 0 if unknown.*/
 float feed_forward_angular_velocity_z; /*< [rad/s] Feed forward Z component of angular velocity (positive: yawing to the right). NaN to be ignored. This is to indicate if the autopilot is actively yawing.*/
 uint16_t estimator_status; /*<  Bitmap indicating which estimator outputs are valid.*/
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
 uint8_t landed_state; /*<  The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.*/
 float angular_velocity_z; /*< [rad/s] Z component of angular velocity in NED (North, East, Down). NaN if unknown.*/
}) mavlink_autopilot_state_for_gimbal_device_t;

#define MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_LEN 57
#define MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_MIN_LEN 53
#define MAVLINK_MSG_ID_286_LEN 57
#define MAVLINK_MSG_ID_286_MIN_LEN 53

#define MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_CRC 210
#define MAVLINK_MSG_ID_286_CRC 210

#define MAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FIELD_Q_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE { \
    286, \
    "AUTOPILOT_STATE_FOR_GIMBAL_DEVICE", \
    13, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 50, offsetof(mavlink_autopilot_state_for_gimbal_device_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 51, offsetof(mavlink_autopilot_state_for_gimbal_device_t, target_component) }, \
         { "time_boot_us", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_autopilot_state_for_gimbal_device_t, time_boot_us) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 4, 8, offsetof(mavlink_autopilot_state_for_gimbal_device_t, q) }, \
         { "q_estimated_delay_us", NULL, MAVLINK_TYPE_UINT32_T, 0, 24, offsetof(mavlink_autopilot_state_for_gimbal_device_t, q_estimated_delay_us) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_autopilot_state_for_gimbal_device_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_autopilot_state_for_gimbal_device_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_autopilot_state_for_gimbal_device_t, vz) }, \
         { "v_estimated_delay_us", NULL, MAVLINK_TYPE_UINT32_T, 0, 40, offsetof(mavlink_autopilot_state_for_gimbal_device_t, v_estimated_delay_us) }, \
         { "feed_forward_angular_velocity_z", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_autopilot_state_for_gimbal_device_t, feed_forward_angular_velocity_z) }, \
         { "estimator_status", NULL, MAVLINK_TYPE_UINT16_T, 0, 48, offsetof(mavlink_autopilot_state_for_gimbal_device_t, estimator_status) }, \
         { "landed_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 52, offsetof(mavlink_autopilot_state_for_gimbal_device_t, landed_state) }, \
         { "angular_velocity_z", NULL, MAVLINK_TYPE_FLOAT, 0, 53, offsetof(mavlink_autopilot_state_for_gimbal_device_t, angular_velocity_z) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE { \
    "AUTOPILOT_STATE_FOR_GIMBAL_DEVICE", \
    13, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 50, offsetof(mavlink_autopilot_state_for_gimbal_device_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 51, offsetof(mavlink_autopilot_state_for_gimbal_device_t, target_component) }, \
         { "time_boot_us", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_autopilot_state_for_gimbal_device_t, time_boot_us) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 4, 8, offsetof(mavlink_autopilot_state_for_gimbal_device_t, q) }, \
         { "q_estimated_delay_us", NULL, MAVLINK_TYPE_UINT32_T, 0, 24, offsetof(mavlink_autopilot_state_for_gimbal_device_t, q_estimated_delay_us) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_autopilot_state_for_gimbal_device_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_autopilot_state_for_gimbal_device_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_autopilot_state_for_gimbal_device_t, vz) }, \
         { "v_estimated_delay_us", NULL, MAVLINK_TYPE_UINT32_T, 0, 40, offsetof(mavlink_autopilot_state_for_gimbal_device_t, v_estimated_delay_us) }, \
         { "feed_forward_angular_velocity_z", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_autopilot_state_for_gimbal_device_t, feed_forward_angular_velocity_z) }, \
         { "estimator_status", NULL, MAVLINK_TYPE_UINT16_T, 0, 48, offsetof(mavlink_autopilot_state_for_gimbal_device_t, estimator_status) }, \
         { "landed_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 52, offsetof(mavlink_autopilot_state_for_gimbal_device_t, landed_state) }, \
         { "angular_velocity_z", NULL, MAVLINK_TYPE_FLOAT, 0, 53, offsetof(mavlink_autopilot_state_for_gimbal_device_t, angular_velocity_z) }, \
         } \
}
#endif

/**
 * @brief Pack a autopilot_state_for_gimbal_device message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param time_boot_us [us] Timestamp (time since system boot).
 * @param q  Quaternion components of autopilot attitude: w, x, y, z (1 0 0 0 is the null-rotation, Hamilton convention).
 * @param q_estimated_delay_us [us] Estimated delay of the attitude data. 0 if unknown.
 * @param vx [m/s] X Speed in NED (North, East, Down). NAN if unknown.
 * @param vy [m/s] Y Speed in NED (North, East, Down). NAN if unknown.
 * @param vz [m/s] Z Speed in NED (North, East, Down). NAN if unknown.
 * @param v_estimated_delay_us [us] Estimated delay of the speed data. 0 if unknown.
 * @param feed_forward_angular_velocity_z [rad/s] Feed forward Z component of angular velocity (positive: yawing to the right). NaN to be ignored. This is to indicate if the autopilot is actively yawing.
 * @param estimator_status  Bitmap indicating which estimator outputs are valid.
 * @param landed_state  The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
 * @param angular_velocity_z [rad/s] Z component of angular velocity in NED (North, East, Down). NaN if unknown.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_autopilot_state_for_gimbal_device_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint64_t time_boot_us, const float *q, uint32_t q_estimated_delay_us, float vx, float vy, float vz, uint32_t v_estimated_delay_us, float feed_forward_angular_velocity_z, uint16_t estimator_status, uint8_t landed_state, float angular_velocity_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_LEN];
    _mav_put_uint64_t(buf, 0, time_boot_us);
    _mav_put_uint32_t(buf, 24, q_estimated_delay_us);
    _mav_put_float(buf, 28, vx);
    _mav_put_float(buf, 32, vy);
    _mav_put_float(buf, 36, vz);
    _mav_put_uint32_t(buf, 40, v_estimated_delay_us);
    _mav_put_float(buf, 44, feed_forward_angular_velocity_z);
    _mav_put_uint16_t(buf, 48, estimator_status);
    _mav_put_uint8_t(buf, 50, target_system);
    _mav_put_uint8_t(buf, 51, target_component);
    _mav_put_uint8_t(buf, 52, landed_state);
    _mav_put_float(buf, 53, angular_velocity_z);
    _mav_put_float_array(buf, 8, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_LEN);
#else
    mavlink_autopilot_state_for_gimbal_device_t packet;
    packet.time_boot_us = time_boot_us;
    packet.q_estimated_delay_us = q_estimated_delay_us;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.v_estimated_delay_us = v_estimated_delay_us;
    packet.feed_forward_angular_velocity_z = feed_forward_angular_velocity_z;
    packet.estimator_status = estimator_status;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.landed_state = landed_state;
    packet.angular_velocity_z = angular_velocity_z;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_MIN_LEN, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_LEN, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_CRC);
}

/**
 * @brief Pack a autopilot_state_for_gimbal_device message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param time_boot_us [us] Timestamp (time since system boot).
 * @param q  Quaternion components of autopilot attitude: w, x, y, z (1 0 0 0 is the null-rotation, Hamilton convention).
 * @param q_estimated_delay_us [us] Estimated delay of the attitude data. 0 if unknown.
 * @param vx [m/s] X Speed in NED (North, East, Down). NAN if unknown.
 * @param vy [m/s] Y Speed in NED (North, East, Down). NAN if unknown.
 * @param vz [m/s] Z Speed in NED (North, East, Down). NAN if unknown.
 * @param v_estimated_delay_us [us] Estimated delay of the speed data. 0 if unknown.
 * @param feed_forward_angular_velocity_z [rad/s] Feed forward Z component of angular velocity (positive: yawing to the right). NaN to be ignored. This is to indicate if the autopilot is actively yawing.
 * @param estimator_status  Bitmap indicating which estimator outputs are valid.
 * @param landed_state  The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
 * @param angular_velocity_z [rad/s] Z component of angular velocity in NED (North, East, Down). NaN if unknown.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_autopilot_state_for_gimbal_device_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint64_t time_boot_us, const float *q, uint32_t q_estimated_delay_us, float vx, float vy, float vz, uint32_t v_estimated_delay_us, float feed_forward_angular_velocity_z, uint16_t estimator_status, uint8_t landed_state, float angular_velocity_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_LEN];
    _mav_put_uint64_t(buf, 0, time_boot_us);
    _mav_put_uint32_t(buf, 24, q_estimated_delay_us);
    _mav_put_float(buf, 28, vx);
    _mav_put_float(buf, 32, vy);
    _mav_put_float(buf, 36, vz);
    _mav_put_uint32_t(buf, 40, v_estimated_delay_us);
    _mav_put_float(buf, 44, feed_forward_angular_velocity_z);
    _mav_put_uint16_t(buf, 48, estimator_status);
    _mav_put_uint8_t(buf, 50, target_system);
    _mav_put_uint8_t(buf, 51, target_component);
    _mav_put_uint8_t(buf, 52, landed_state);
    _mav_put_float(buf, 53, angular_velocity_z);
    _mav_put_float_array(buf, 8, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_LEN);
#else
    mavlink_autopilot_state_for_gimbal_device_t packet;
    packet.time_boot_us = time_boot_us;
    packet.q_estimated_delay_us = q_estimated_delay_us;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.v_estimated_delay_us = v_estimated_delay_us;
    packet.feed_forward_angular_velocity_z = feed_forward_angular_velocity_z;
    packet.estimator_status = estimator_status;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.landed_state = landed_state;
    packet.angular_velocity_z = angular_velocity_z;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_MIN_LEN, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_LEN, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_MIN_LEN, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_LEN);
#endif
}

/**
 * @brief Pack a autopilot_state_for_gimbal_device message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param time_boot_us [us] Timestamp (time since system boot).
 * @param q  Quaternion components of autopilot attitude: w, x, y, z (1 0 0 0 is the null-rotation, Hamilton convention).
 * @param q_estimated_delay_us [us] Estimated delay of the attitude data. 0 if unknown.
 * @param vx [m/s] X Speed in NED (North, East, Down). NAN if unknown.
 * @param vy [m/s] Y Speed in NED (North, East, Down). NAN if unknown.
 * @param vz [m/s] Z Speed in NED (North, East, Down). NAN if unknown.
 * @param v_estimated_delay_us [us] Estimated delay of the speed data. 0 if unknown.
 * @param feed_forward_angular_velocity_z [rad/s] Feed forward Z component of angular velocity (positive: yawing to the right). NaN to be ignored. This is to indicate if the autopilot is actively yawing.
 * @param estimator_status  Bitmap indicating which estimator outputs are valid.
 * @param landed_state  The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
 * @param angular_velocity_z [rad/s] Z component of angular velocity in NED (North, East, Down). NaN if unknown.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_autopilot_state_for_gimbal_device_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint64_t time_boot_us,const float *q,uint32_t q_estimated_delay_us,float vx,float vy,float vz,uint32_t v_estimated_delay_us,float feed_forward_angular_velocity_z,uint16_t estimator_status,uint8_t landed_state,float angular_velocity_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_LEN];
    _mav_put_uint64_t(buf, 0, time_boot_us);
    _mav_put_uint32_t(buf, 24, q_estimated_delay_us);
    _mav_put_float(buf, 28, vx);
    _mav_put_float(buf, 32, vy);
    _mav_put_float(buf, 36, vz);
    _mav_put_uint32_t(buf, 40, v_estimated_delay_us);
    _mav_put_float(buf, 44, feed_forward_angular_velocity_z);
    _mav_put_uint16_t(buf, 48, estimator_status);
    _mav_put_uint8_t(buf, 50, target_system);
    _mav_put_uint8_t(buf, 51, target_component);
    _mav_put_uint8_t(buf, 52, landed_state);
    _mav_put_float(buf, 53, angular_velocity_z);
    _mav_put_float_array(buf, 8, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_LEN);
#else
    mavlink_autopilot_state_for_gimbal_device_t packet;
    packet.time_boot_us = time_boot_us;
    packet.q_estimated_delay_us = q_estimated_delay_us;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.v_estimated_delay_us = v_estimated_delay_us;
    packet.feed_forward_angular_velocity_z = feed_forward_angular_velocity_z;
    packet.estimator_status = estimator_status;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.landed_state = landed_state;
    packet.angular_velocity_z = angular_velocity_z;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_MIN_LEN, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_LEN, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_CRC);
}

/**
 * @brief Encode a autopilot_state_for_gimbal_device struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param autopilot_state_for_gimbal_device C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_autopilot_state_for_gimbal_device_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_autopilot_state_for_gimbal_device_t* autopilot_state_for_gimbal_device)
{
    return mavlink_msg_autopilot_state_for_gimbal_device_pack(system_id, component_id, msg, autopilot_state_for_gimbal_device->target_system, autopilot_state_for_gimbal_device->target_component, autopilot_state_for_gimbal_device->time_boot_us, autopilot_state_for_gimbal_device->q, autopilot_state_for_gimbal_device->q_estimated_delay_us, autopilot_state_for_gimbal_device->vx, autopilot_state_for_gimbal_device->vy, autopilot_state_for_gimbal_device->vz, autopilot_state_for_gimbal_device->v_estimated_delay_us, autopilot_state_for_gimbal_device->feed_forward_angular_velocity_z, autopilot_state_for_gimbal_device->estimator_status, autopilot_state_for_gimbal_device->landed_state, autopilot_state_for_gimbal_device->angular_velocity_z);
}

/**
 * @brief Encode a autopilot_state_for_gimbal_device struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param autopilot_state_for_gimbal_device C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_autopilot_state_for_gimbal_device_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_autopilot_state_for_gimbal_device_t* autopilot_state_for_gimbal_device)
{
    return mavlink_msg_autopilot_state_for_gimbal_device_pack_chan(system_id, component_id, chan, msg, autopilot_state_for_gimbal_device->target_system, autopilot_state_for_gimbal_device->target_component, autopilot_state_for_gimbal_device->time_boot_us, autopilot_state_for_gimbal_device->q, autopilot_state_for_gimbal_device->q_estimated_delay_us, autopilot_state_for_gimbal_device->vx, autopilot_state_for_gimbal_device->vy, autopilot_state_for_gimbal_device->vz, autopilot_state_for_gimbal_device->v_estimated_delay_us, autopilot_state_for_gimbal_device->feed_forward_angular_velocity_z, autopilot_state_for_gimbal_device->estimator_status, autopilot_state_for_gimbal_device->landed_state, autopilot_state_for_gimbal_device->angular_velocity_z);
}

/**
 * @brief Encode a autopilot_state_for_gimbal_device struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param autopilot_state_for_gimbal_device C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_autopilot_state_for_gimbal_device_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_autopilot_state_for_gimbal_device_t* autopilot_state_for_gimbal_device)
{
    return mavlink_msg_autopilot_state_for_gimbal_device_pack_status(system_id, component_id, _status, msg,  autopilot_state_for_gimbal_device->target_system, autopilot_state_for_gimbal_device->target_component, autopilot_state_for_gimbal_device->time_boot_us, autopilot_state_for_gimbal_device->q, autopilot_state_for_gimbal_device->q_estimated_delay_us, autopilot_state_for_gimbal_device->vx, autopilot_state_for_gimbal_device->vy, autopilot_state_for_gimbal_device->vz, autopilot_state_for_gimbal_device->v_estimated_delay_us, autopilot_state_for_gimbal_device->feed_forward_angular_velocity_z, autopilot_state_for_gimbal_device->estimator_status, autopilot_state_for_gimbal_device->landed_state, autopilot_state_for_gimbal_device->angular_velocity_z);
}

/**
 * @brief Send a autopilot_state_for_gimbal_device message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param time_boot_us [us] Timestamp (time since system boot).
 * @param q  Quaternion components of autopilot attitude: w, x, y, z (1 0 0 0 is the null-rotation, Hamilton convention).
 * @param q_estimated_delay_us [us] Estimated delay of the attitude data. 0 if unknown.
 * @param vx [m/s] X Speed in NED (North, East, Down). NAN if unknown.
 * @param vy [m/s] Y Speed in NED (North, East, Down). NAN if unknown.
 * @param vz [m/s] Z Speed in NED (North, East, Down). NAN if unknown.
 * @param v_estimated_delay_us [us] Estimated delay of the speed data. 0 if unknown.
 * @param feed_forward_angular_velocity_z [rad/s] Feed forward Z component of angular velocity (positive: yawing to the right). NaN to be ignored. This is to indicate if the autopilot is actively yawing.
 * @param estimator_status  Bitmap indicating which estimator outputs are valid.
 * @param landed_state  The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
 * @param angular_velocity_z [rad/s] Z component of angular velocity in NED (North, East, Down). NaN if unknown.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_autopilot_state_for_gimbal_device_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint64_t time_boot_us, const float *q, uint32_t q_estimated_delay_us, float vx, float vy, float vz, uint32_t v_estimated_delay_us, float feed_forward_angular_velocity_z, uint16_t estimator_status, uint8_t landed_state, float angular_velocity_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_LEN];
    _mav_put_uint64_t(buf, 0, time_boot_us);
    _mav_put_uint32_t(buf, 24, q_estimated_delay_us);
    _mav_put_float(buf, 28, vx);
    _mav_put_float(buf, 32, vy);
    _mav_put_float(buf, 36, vz);
    _mav_put_uint32_t(buf, 40, v_estimated_delay_us);
    _mav_put_float(buf, 44, feed_forward_angular_velocity_z);
    _mav_put_uint16_t(buf, 48, estimator_status);
    _mav_put_uint8_t(buf, 50, target_system);
    _mav_put_uint8_t(buf, 51, target_component);
    _mav_put_uint8_t(buf, 52, landed_state);
    _mav_put_float(buf, 53, angular_velocity_z);
    _mav_put_float_array(buf, 8, q, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE, buf, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_MIN_LEN, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_LEN, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_CRC);
#else
    mavlink_autopilot_state_for_gimbal_device_t packet;
    packet.time_boot_us = time_boot_us;
    packet.q_estimated_delay_us = q_estimated_delay_us;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.v_estimated_delay_us = v_estimated_delay_us;
    packet.feed_forward_angular_velocity_z = feed_forward_angular_velocity_z;
    packet.estimator_status = estimator_status;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.landed_state = landed_state;
    packet.angular_velocity_z = angular_velocity_z;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE, (const char *)&packet, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_MIN_LEN, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_LEN, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_CRC);
#endif
}

/**
 * @brief Send a autopilot_state_for_gimbal_device message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_autopilot_state_for_gimbal_device_send_struct(mavlink_channel_t chan, const mavlink_autopilot_state_for_gimbal_device_t* autopilot_state_for_gimbal_device)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_autopilot_state_for_gimbal_device_send(chan, autopilot_state_for_gimbal_device->target_system, autopilot_state_for_gimbal_device->target_component, autopilot_state_for_gimbal_device->time_boot_us, autopilot_state_for_gimbal_device->q, autopilot_state_for_gimbal_device->q_estimated_delay_us, autopilot_state_for_gimbal_device->vx, autopilot_state_for_gimbal_device->vy, autopilot_state_for_gimbal_device->vz, autopilot_state_for_gimbal_device->v_estimated_delay_us, autopilot_state_for_gimbal_device->feed_forward_angular_velocity_z, autopilot_state_for_gimbal_device->estimator_status, autopilot_state_for_gimbal_device->landed_state, autopilot_state_for_gimbal_device->angular_velocity_z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE, (const char *)autopilot_state_for_gimbal_device, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_MIN_LEN, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_LEN, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_CRC);
#endif
}

#if MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_autopilot_state_for_gimbal_device_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint64_t time_boot_us, const float *q, uint32_t q_estimated_delay_us, float vx, float vy, float vz, uint32_t v_estimated_delay_us, float feed_forward_angular_velocity_z, uint16_t estimator_status, uint8_t landed_state, float angular_velocity_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_boot_us);
    _mav_put_uint32_t(buf, 24, q_estimated_delay_us);
    _mav_put_float(buf, 28, vx);
    _mav_put_float(buf, 32, vy);
    _mav_put_float(buf, 36, vz);
    _mav_put_uint32_t(buf, 40, v_estimated_delay_us);
    _mav_put_float(buf, 44, feed_forward_angular_velocity_z);
    _mav_put_uint16_t(buf, 48, estimator_status);
    _mav_put_uint8_t(buf, 50, target_system);
    _mav_put_uint8_t(buf, 51, target_component);
    _mav_put_uint8_t(buf, 52, landed_state);
    _mav_put_float(buf, 53, angular_velocity_z);
    _mav_put_float_array(buf, 8, q, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE, buf, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_MIN_LEN, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_LEN, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_CRC);
#else
    mavlink_autopilot_state_for_gimbal_device_t *packet = (mavlink_autopilot_state_for_gimbal_device_t *)msgbuf;
    packet->time_boot_us = time_boot_us;
    packet->q_estimated_delay_us = q_estimated_delay_us;
    packet->vx = vx;
    packet->vy = vy;
    packet->vz = vz;
    packet->v_estimated_delay_us = v_estimated_delay_us;
    packet->feed_forward_angular_velocity_z = feed_forward_angular_velocity_z;
    packet->estimator_status = estimator_status;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->landed_state = landed_state;
    packet->angular_velocity_z = angular_velocity_z;
    mav_array_memcpy(packet->q, q, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE, (const char *)packet, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_MIN_LEN, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_LEN, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_CRC);
#endif
}
#endif

#endif

// MESSAGE AUTOPILOT_STATE_FOR_GIMBAL_DEVICE UNPACKING


/**
 * @brief Get field target_system from autopilot_state_for_gimbal_device message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_autopilot_state_for_gimbal_device_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  50);
}

/**
 * @brief Get field target_component from autopilot_state_for_gimbal_device message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_autopilot_state_for_gimbal_device_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  51);
}

/**
 * @brief Get field time_boot_us from autopilot_state_for_gimbal_device message
 *
 * @return [us] Timestamp (time since system boot).
 */
static inline uint64_t mavlink_msg_autopilot_state_for_gimbal_device_get_time_boot_us(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field q from autopilot_state_for_gimbal_device message
 *
 * @return  Quaternion components of autopilot attitude: w, x, y, z (1 0 0 0 is the null-rotation, Hamilton convention).
 */
static inline uint16_t mavlink_msg_autopilot_state_for_gimbal_device_get_q(const mavlink_message_t* msg, float *q)
{
    return _MAV_RETURN_float_array(msg, q, 4,  8);
}

/**
 * @brief Get field q_estimated_delay_us from autopilot_state_for_gimbal_device message
 *
 * @return [us] Estimated delay of the attitude data. 0 if unknown.
 */
static inline uint32_t mavlink_msg_autopilot_state_for_gimbal_device_get_q_estimated_delay_us(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  24);
}

/**
 * @brief Get field vx from autopilot_state_for_gimbal_device message
 *
 * @return [m/s] X Speed in NED (North, East, Down). NAN if unknown.
 */
static inline float mavlink_msg_autopilot_state_for_gimbal_device_get_vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field vy from autopilot_state_for_gimbal_device message
 *
 * @return [m/s] Y Speed in NED (North, East, Down). NAN if unknown.
 */
static inline float mavlink_msg_autopilot_state_for_gimbal_device_get_vy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field vz from autopilot_state_for_gimbal_device message
 *
 * @return [m/s] Z Speed in NED (North, East, Down). NAN if unknown.
 */
static inline float mavlink_msg_autopilot_state_for_gimbal_device_get_vz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field v_estimated_delay_us from autopilot_state_for_gimbal_device message
 *
 * @return [us] Estimated delay of the speed data. 0 if unknown.
 */
static inline uint32_t mavlink_msg_autopilot_state_for_gimbal_device_get_v_estimated_delay_us(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  40);
}

/**
 * @brief Get field feed_forward_angular_velocity_z from autopilot_state_for_gimbal_device message
 *
 * @return [rad/s] Feed forward Z component of angular velocity (positive: yawing to the right). NaN to be ignored. This is to indicate if the autopilot is actively yawing.
 */
static inline float mavlink_msg_autopilot_state_for_gimbal_device_get_feed_forward_angular_velocity_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field estimator_status from autopilot_state_for_gimbal_device message
 *
 * @return  Bitmap indicating which estimator outputs are valid.
 */
static inline uint16_t mavlink_msg_autopilot_state_for_gimbal_device_get_estimator_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  48);
}

/**
 * @brief Get field landed_state from autopilot_state_for_gimbal_device message
 *
 * @return  The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
 */
static inline uint8_t mavlink_msg_autopilot_state_for_gimbal_device_get_landed_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  52);
}

/**
 * @brief Get field angular_velocity_z from autopilot_state_for_gimbal_device message
 *
 * @return [rad/s] Z component of angular velocity in NED (North, East, Down). NaN if unknown.
 */
static inline float mavlink_msg_autopilot_state_for_gimbal_device_get_angular_velocity_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  53);
}

/**
 * @brief Decode a autopilot_state_for_gimbal_device message into a struct
 *
 * @param msg The message to decode
 * @param autopilot_state_for_gimbal_device C-struct to decode the message contents into
 */
static inline void mavlink_msg_autopilot_state_for_gimbal_device_decode(const mavlink_message_t* msg, mavlink_autopilot_state_for_gimbal_device_t* autopilot_state_for_gimbal_device)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    autopilot_state_for_gimbal_device->time_boot_us = mavlink_msg_autopilot_state_for_gimbal_device_get_time_boot_us(msg);
    mavlink_msg_autopilot_state_for_gimbal_device_get_q(msg, autopilot_state_for_gimbal_device->q);
    autopilot_state_for_gimbal_device->q_estimated_delay_us = mavlink_msg_autopilot_state_for_gimbal_device_get_q_estimated_delay_us(msg);
    autopilot_state_for_gimbal_device->vx = mavlink_msg_autopilot_state_for_gimbal_device_get_vx(msg);
    autopilot_state_for_gimbal_device->vy = mavlink_msg_autopilot_state_for_gimbal_device_get_vy(msg);
    autopilot_state_for_gimbal_device->vz = mavlink_msg_autopilot_state_for_gimbal_device_get_vz(msg);
    autopilot_state_for_gimbal_device->v_estimated_delay_us = mavlink_msg_autopilot_state_for_gimbal_device_get_v_estimated_delay_us(msg);
    autopilot_state_for_gimbal_device->feed_forward_angular_velocity_z = mavlink_msg_autopilot_state_for_gimbal_device_get_feed_forward_angular_velocity_z(msg);
    autopilot_state_for_gimbal_device->estimator_status = mavlink_msg_autopilot_state_for_gimbal_device_get_estimator_status(msg);
    autopilot_state_for_gimbal_device->target_system = mavlink_msg_autopilot_state_for_gimbal_device_get_target_system(msg);
    autopilot_state_for_gimbal_device->target_component = mavlink_msg_autopilot_state_for_gimbal_device_get_target_component(msg);
    autopilot_state_for_gimbal_device->landed_state = mavlink_msg_autopilot_state_for_gimbal_device_get_landed_state(msg);
    autopilot_state_for_gimbal_device->angular_velocity_z = mavlink_msg_autopilot_state_for_gimbal_device_get_angular_velocity_z(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_LEN? msg->len : MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_LEN;
        memset(autopilot_state_for_gimbal_device, 0, MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_LEN);
    memcpy(autopilot_state_for_gimbal_device, _MAV_PAYLOAD(msg), len);
#endif
}
