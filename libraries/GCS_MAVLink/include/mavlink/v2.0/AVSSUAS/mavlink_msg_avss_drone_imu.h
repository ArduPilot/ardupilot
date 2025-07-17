#pragma once
// MESSAGE AVSS_DRONE_IMU PACKING

#define MAVLINK_MSG_ID_AVSS_DRONE_IMU 60052


typedef struct __mavlink_avss_drone_imu_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since FC boot).*/
 float q1; /*<  Quaternion component 1, w (1 in null-rotation)*/
 float q2; /*<  Quaternion component 2, x (0 in null-rotation)*/
 float q3; /*<  Quaternion component 3, y (0 in null-rotation)*/
 float q4; /*<  Quaternion component 4, z (0 in null-rotation)*/
 float xacc; /*< [m/s/s] X acceleration*/
 float yacc; /*< [m/s/s] Y acceleration*/
 float zacc; /*< [m/s/s] Z acceleration*/
 float xgyro; /*< [rad/s] Angular speed around X axis*/
 float ygyro; /*< [rad/s] Angular speed around Y axis*/
 float zgyro; /*< [rad/s] Angular speed around Z axis*/
} mavlink_avss_drone_imu_t;

#define MAVLINK_MSG_ID_AVSS_DRONE_IMU_LEN 44
#define MAVLINK_MSG_ID_AVSS_DRONE_IMU_MIN_LEN 44
#define MAVLINK_MSG_ID_60052_LEN 44
#define MAVLINK_MSG_ID_60052_MIN_LEN 44

#define MAVLINK_MSG_ID_AVSS_DRONE_IMU_CRC 101
#define MAVLINK_MSG_ID_60052_CRC 101



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AVSS_DRONE_IMU { \
    60052, \
    "AVSS_DRONE_IMU", \
    11, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_avss_drone_imu_t, time_boot_ms) }, \
         { "q1", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_avss_drone_imu_t, q1) }, \
         { "q2", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_avss_drone_imu_t, q2) }, \
         { "q3", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_avss_drone_imu_t, q3) }, \
         { "q4", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_avss_drone_imu_t, q4) }, \
         { "xacc", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_avss_drone_imu_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_avss_drone_imu_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_avss_drone_imu_t, zacc) }, \
         { "xgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_avss_drone_imu_t, xgyro) }, \
         { "ygyro", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_avss_drone_imu_t, ygyro) }, \
         { "zgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_avss_drone_imu_t, zgyro) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AVSS_DRONE_IMU { \
    "AVSS_DRONE_IMU", \
    11, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_avss_drone_imu_t, time_boot_ms) }, \
         { "q1", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_avss_drone_imu_t, q1) }, \
         { "q2", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_avss_drone_imu_t, q2) }, \
         { "q3", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_avss_drone_imu_t, q3) }, \
         { "q4", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_avss_drone_imu_t, q4) }, \
         { "xacc", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_avss_drone_imu_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_avss_drone_imu_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_avss_drone_imu_t, zacc) }, \
         { "xgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_avss_drone_imu_t, xgyro) }, \
         { "ygyro", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_avss_drone_imu_t, ygyro) }, \
         { "zgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_avss_drone_imu_t, zgyro) }, \
         } \
}
#endif

/**
 * @brief Pack a avss_drone_imu message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since FC boot).
 * @param q1  Quaternion component 1, w (1 in null-rotation)
 * @param q2  Quaternion component 2, x (0 in null-rotation)
 * @param q3  Quaternion component 3, y (0 in null-rotation)
 * @param q4  Quaternion component 4, z (0 in null-rotation)
 * @param xacc [m/s/s] X acceleration
 * @param yacc [m/s/s] Y acceleration
 * @param zacc [m/s/s] Z acceleration
 * @param xgyro [rad/s] Angular speed around X axis
 * @param ygyro [rad/s] Angular speed around Y axis
 * @param zgyro [rad/s] Angular speed around Z axis
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_avss_drone_imu_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, float q1, float q2, float q3, float q4, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AVSS_DRONE_IMU_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, q1);
    _mav_put_float(buf, 8, q2);
    _mav_put_float(buf, 12, q3);
    _mav_put_float(buf, 16, q4);
    _mav_put_float(buf, 20, xacc);
    _mav_put_float(buf, 24, yacc);
    _mav_put_float(buf, 28, zacc);
    _mav_put_float(buf, 32, xgyro);
    _mav_put_float(buf, 36, ygyro);
    _mav_put_float(buf, 40, zgyro);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AVSS_DRONE_IMU_LEN);
#else
    mavlink_avss_drone_imu_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.q1 = q1;
    packet.q2 = q2;
    packet.q3 = q3;
    packet.q4 = q4;
    packet.xacc = xacc;
    packet.yacc = yacc;
    packet.zacc = zacc;
    packet.xgyro = xgyro;
    packet.ygyro = ygyro;
    packet.zgyro = zgyro;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AVSS_DRONE_IMU_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AVSS_DRONE_IMU;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AVSS_DRONE_IMU_MIN_LEN, MAVLINK_MSG_ID_AVSS_DRONE_IMU_LEN, MAVLINK_MSG_ID_AVSS_DRONE_IMU_CRC);
}

/**
 * @brief Pack a avss_drone_imu message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since FC boot).
 * @param q1  Quaternion component 1, w (1 in null-rotation)
 * @param q2  Quaternion component 2, x (0 in null-rotation)
 * @param q3  Quaternion component 3, y (0 in null-rotation)
 * @param q4  Quaternion component 4, z (0 in null-rotation)
 * @param xacc [m/s/s] X acceleration
 * @param yacc [m/s/s] Y acceleration
 * @param zacc [m/s/s] Z acceleration
 * @param xgyro [rad/s] Angular speed around X axis
 * @param ygyro [rad/s] Angular speed around Y axis
 * @param zgyro [rad/s] Angular speed around Z axis
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_avss_drone_imu_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t time_boot_ms, float q1, float q2, float q3, float q4, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AVSS_DRONE_IMU_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, q1);
    _mav_put_float(buf, 8, q2);
    _mav_put_float(buf, 12, q3);
    _mav_put_float(buf, 16, q4);
    _mav_put_float(buf, 20, xacc);
    _mav_put_float(buf, 24, yacc);
    _mav_put_float(buf, 28, zacc);
    _mav_put_float(buf, 32, xgyro);
    _mav_put_float(buf, 36, ygyro);
    _mav_put_float(buf, 40, zgyro);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AVSS_DRONE_IMU_LEN);
#else
    mavlink_avss_drone_imu_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.q1 = q1;
    packet.q2 = q2;
    packet.q3 = q3;
    packet.q4 = q4;
    packet.xacc = xacc;
    packet.yacc = yacc;
    packet.zacc = zacc;
    packet.xgyro = xgyro;
    packet.ygyro = ygyro;
    packet.zgyro = zgyro;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AVSS_DRONE_IMU_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AVSS_DRONE_IMU;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AVSS_DRONE_IMU_MIN_LEN, MAVLINK_MSG_ID_AVSS_DRONE_IMU_LEN, MAVLINK_MSG_ID_AVSS_DRONE_IMU_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AVSS_DRONE_IMU_MIN_LEN, MAVLINK_MSG_ID_AVSS_DRONE_IMU_LEN);
#endif
}

/**
 * @brief Pack a avss_drone_imu message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since FC boot).
 * @param q1  Quaternion component 1, w (1 in null-rotation)
 * @param q2  Quaternion component 2, x (0 in null-rotation)
 * @param q3  Quaternion component 3, y (0 in null-rotation)
 * @param q4  Quaternion component 4, z (0 in null-rotation)
 * @param xacc [m/s/s] X acceleration
 * @param yacc [m/s/s] Y acceleration
 * @param zacc [m/s/s] Z acceleration
 * @param xgyro [rad/s] Angular speed around X axis
 * @param ygyro [rad/s] Angular speed around Y axis
 * @param zgyro [rad/s] Angular speed around Z axis
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_avss_drone_imu_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,float q1,float q2,float q3,float q4,float xacc,float yacc,float zacc,float xgyro,float ygyro,float zgyro)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AVSS_DRONE_IMU_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, q1);
    _mav_put_float(buf, 8, q2);
    _mav_put_float(buf, 12, q3);
    _mav_put_float(buf, 16, q4);
    _mav_put_float(buf, 20, xacc);
    _mav_put_float(buf, 24, yacc);
    _mav_put_float(buf, 28, zacc);
    _mav_put_float(buf, 32, xgyro);
    _mav_put_float(buf, 36, ygyro);
    _mav_put_float(buf, 40, zgyro);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AVSS_DRONE_IMU_LEN);
#else
    mavlink_avss_drone_imu_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.q1 = q1;
    packet.q2 = q2;
    packet.q3 = q3;
    packet.q4 = q4;
    packet.xacc = xacc;
    packet.yacc = yacc;
    packet.zacc = zacc;
    packet.xgyro = xgyro;
    packet.ygyro = ygyro;
    packet.zgyro = zgyro;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AVSS_DRONE_IMU_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AVSS_DRONE_IMU;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AVSS_DRONE_IMU_MIN_LEN, MAVLINK_MSG_ID_AVSS_DRONE_IMU_LEN, MAVLINK_MSG_ID_AVSS_DRONE_IMU_CRC);
}

/**
 * @brief Encode a avss_drone_imu struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param avss_drone_imu C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_avss_drone_imu_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_avss_drone_imu_t* avss_drone_imu)
{
    return mavlink_msg_avss_drone_imu_pack(system_id, component_id, msg, avss_drone_imu->time_boot_ms, avss_drone_imu->q1, avss_drone_imu->q2, avss_drone_imu->q3, avss_drone_imu->q4, avss_drone_imu->xacc, avss_drone_imu->yacc, avss_drone_imu->zacc, avss_drone_imu->xgyro, avss_drone_imu->ygyro, avss_drone_imu->zgyro);
}

/**
 * @brief Encode a avss_drone_imu struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param avss_drone_imu C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_avss_drone_imu_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_avss_drone_imu_t* avss_drone_imu)
{
    return mavlink_msg_avss_drone_imu_pack_chan(system_id, component_id, chan, msg, avss_drone_imu->time_boot_ms, avss_drone_imu->q1, avss_drone_imu->q2, avss_drone_imu->q3, avss_drone_imu->q4, avss_drone_imu->xacc, avss_drone_imu->yacc, avss_drone_imu->zacc, avss_drone_imu->xgyro, avss_drone_imu->ygyro, avss_drone_imu->zgyro);
}

/**
 * @brief Encode a avss_drone_imu struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param avss_drone_imu C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_avss_drone_imu_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_avss_drone_imu_t* avss_drone_imu)
{
    return mavlink_msg_avss_drone_imu_pack_status(system_id, component_id, _status, msg,  avss_drone_imu->time_boot_ms, avss_drone_imu->q1, avss_drone_imu->q2, avss_drone_imu->q3, avss_drone_imu->q4, avss_drone_imu->xacc, avss_drone_imu->yacc, avss_drone_imu->zacc, avss_drone_imu->xgyro, avss_drone_imu->ygyro, avss_drone_imu->zgyro);
}

/**
 * @brief Send a avss_drone_imu message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since FC boot).
 * @param q1  Quaternion component 1, w (1 in null-rotation)
 * @param q2  Quaternion component 2, x (0 in null-rotation)
 * @param q3  Quaternion component 3, y (0 in null-rotation)
 * @param q4  Quaternion component 4, z (0 in null-rotation)
 * @param xacc [m/s/s] X acceleration
 * @param yacc [m/s/s] Y acceleration
 * @param zacc [m/s/s] Z acceleration
 * @param xgyro [rad/s] Angular speed around X axis
 * @param ygyro [rad/s] Angular speed around Y axis
 * @param zgyro [rad/s] Angular speed around Z axis
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_avss_drone_imu_send(mavlink_channel_t chan, uint32_t time_boot_ms, float q1, float q2, float q3, float q4, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AVSS_DRONE_IMU_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, q1);
    _mav_put_float(buf, 8, q2);
    _mav_put_float(buf, 12, q3);
    _mav_put_float(buf, 16, q4);
    _mav_put_float(buf, 20, xacc);
    _mav_put_float(buf, 24, yacc);
    _mav_put_float(buf, 28, zacc);
    _mav_put_float(buf, 32, xgyro);
    _mav_put_float(buf, 36, ygyro);
    _mav_put_float(buf, 40, zgyro);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AVSS_DRONE_IMU, buf, MAVLINK_MSG_ID_AVSS_DRONE_IMU_MIN_LEN, MAVLINK_MSG_ID_AVSS_DRONE_IMU_LEN, MAVLINK_MSG_ID_AVSS_DRONE_IMU_CRC);
#else
    mavlink_avss_drone_imu_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.q1 = q1;
    packet.q2 = q2;
    packet.q3 = q3;
    packet.q4 = q4;
    packet.xacc = xacc;
    packet.yacc = yacc;
    packet.zacc = zacc;
    packet.xgyro = xgyro;
    packet.ygyro = ygyro;
    packet.zgyro = zgyro;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AVSS_DRONE_IMU, (const char *)&packet, MAVLINK_MSG_ID_AVSS_DRONE_IMU_MIN_LEN, MAVLINK_MSG_ID_AVSS_DRONE_IMU_LEN, MAVLINK_MSG_ID_AVSS_DRONE_IMU_CRC);
#endif
}

/**
 * @brief Send a avss_drone_imu message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_avss_drone_imu_send_struct(mavlink_channel_t chan, const mavlink_avss_drone_imu_t* avss_drone_imu)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_avss_drone_imu_send(chan, avss_drone_imu->time_boot_ms, avss_drone_imu->q1, avss_drone_imu->q2, avss_drone_imu->q3, avss_drone_imu->q4, avss_drone_imu->xacc, avss_drone_imu->yacc, avss_drone_imu->zacc, avss_drone_imu->xgyro, avss_drone_imu->ygyro, avss_drone_imu->zgyro);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AVSS_DRONE_IMU, (const char *)avss_drone_imu, MAVLINK_MSG_ID_AVSS_DRONE_IMU_MIN_LEN, MAVLINK_MSG_ID_AVSS_DRONE_IMU_LEN, MAVLINK_MSG_ID_AVSS_DRONE_IMU_CRC);
#endif
}

#if MAVLINK_MSG_ID_AVSS_DRONE_IMU_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_avss_drone_imu_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, float q1, float q2, float q3, float q4, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, q1);
    _mav_put_float(buf, 8, q2);
    _mav_put_float(buf, 12, q3);
    _mav_put_float(buf, 16, q4);
    _mav_put_float(buf, 20, xacc);
    _mav_put_float(buf, 24, yacc);
    _mav_put_float(buf, 28, zacc);
    _mav_put_float(buf, 32, xgyro);
    _mav_put_float(buf, 36, ygyro);
    _mav_put_float(buf, 40, zgyro);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AVSS_DRONE_IMU, buf, MAVLINK_MSG_ID_AVSS_DRONE_IMU_MIN_LEN, MAVLINK_MSG_ID_AVSS_DRONE_IMU_LEN, MAVLINK_MSG_ID_AVSS_DRONE_IMU_CRC);
#else
    mavlink_avss_drone_imu_t *packet = (mavlink_avss_drone_imu_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->q1 = q1;
    packet->q2 = q2;
    packet->q3 = q3;
    packet->q4 = q4;
    packet->xacc = xacc;
    packet->yacc = yacc;
    packet->zacc = zacc;
    packet->xgyro = xgyro;
    packet->ygyro = ygyro;
    packet->zgyro = zgyro;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AVSS_DRONE_IMU, (const char *)packet, MAVLINK_MSG_ID_AVSS_DRONE_IMU_MIN_LEN, MAVLINK_MSG_ID_AVSS_DRONE_IMU_LEN, MAVLINK_MSG_ID_AVSS_DRONE_IMU_CRC);
#endif
}
#endif

#endif

// MESSAGE AVSS_DRONE_IMU UNPACKING


/**
 * @brief Get field time_boot_ms from avss_drone_imu message
 *
 * @return [ms] Timestamp (time since FC boot).
 */
static inline uint32_t mavlink_msg_avss_drone_imu_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field q1 from avss_drone_imu message
 *
 * @return  Quaternion component 1, w (1 in null-rotation)
 */
static inline float mavlink_msg_avss_drone_imu_get_q1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field q2 from avss_drone_imu message
 *
 * @return  Quaternion component 2, x (0 in null-rotation)
 */
static inline float mavlink_msg_avss_drone_imu_get_q2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field q3 from avss_drone_imu message
 *
 * @return  Quaternion component 3, y (0 in null-rotation)
 */
static inline float mavlink_msg_avss_drone_imu_get_q3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field q4 from avss_drone_imu message
 *
 * @return  Quaternion component 4, z (0 in null-rotation)
 */
static inline float mavlink_msg_avss_drone_imu_get_q4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field xacc from avss_drone_imu message
 *
 * @return [m/s/s] X acceleration
 */
static inline float mavlink_msg_avss_drone_imu_get_xacc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field yacc from avss_drone_imu message
 *
 * @return [m/s/s] Y acceleration
 */
static inline float mavlink_msg_avss_drone_imu_get_yacc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field zacc from avss_drone_imu message
 *
 * @return [m/s/s] Z acceleration
 */
static inline float mavlink_msg_avss_drone_imu_get_zacc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field xgyro from avss_drone_imu message
 *
 * @return [rad/s] Angular speed around X axis
 */
static inline float mavlink_msg_avss_drone_imu_get_xgyro(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field ygyro from avss_drone_imu message
 *
 * @return [rad/s] Angular speed around Y axis
 */
static inline float mavlink_msg_avss_drone_imu_get_ygyro(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field zgyro from avss_drone_imu message
 *
 * @return [rad/s] Angular speed around Z axis
 */
static inline float mavlink_msg_avss_drone_imu_get_zgyro(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Decode a avss_drone_imu message into a struct
 *
 * @param msg The message to decode
 * @param avss_drone_imu C-struct to decode the message contents into
 */
static inline void mavlink_msg_avss_drone_imu_decode(const mavlink_message_t* msg, mavlink_avss_drone_imu_t* avss_drone_imu)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    avss_drone_imu->time_boot_ms = mavlink_msg_avss_drone_imu_get_time_boot_ms(msg);
    avss_drone_imu->q1 = mavlink_msg_avss_drone_imu_get_q1(msg);
    avss_drone_imu->q2 = mavlink_msg_avss_drone_imu_get_q2(msg);
    avss_drone_imu->q3 = mavlink_msg_avss_drone_imu_get_q3(msg);
    avss_drone_imu->q4 = mavlink_msg_avss_drone_imu_get_q4(msg);
    avss_drone_imu->xacc = mavlink_msg_avss_drone_imu_get_xacc(msg);
    avss_drone_imu->yacc = mavlink_msg_avss_drone_imu_get_yacc(msg);
    avss_drone_imu->zacc = mavlink_msg_avss_drone_imu_get_zacc(msg);
    avss_drone_imu->xgyro = mavlink_msg_avss_drone_imu_get_xgyro(msg);
    avss_drone_imu->ygyro = mavlink_msg_avss_drone_imu_get_ygyro(msg);
    avss_drone_imu->zgyro = mavlink_msg_avss_drone_imu_get_zgyro(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AVSS_DRONE_IMU_LEN? msg->len : MAVLINK_MSG_ID_AVSS_DRONE_IMU_LEN;
        memset(avss_drone_imu, 0, MAVLINK_MSG_ID_AVSS_DRONE_IMU_LEN);
    memcpy(avss_drone_imu, _MAV_PAYLOAD(msg), len);
#endif
}
