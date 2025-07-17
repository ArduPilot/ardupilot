#pragma once
// MESSAGE EKF_EXT PACKING

#define MAVLINK_MSG_ID_EKF_EXT 8007


typedef struct __mavlink_ekf_ext_t {
 uint64_t timestamp; /*< [us]  Time since system start*/
 float Windspeed; /*< [m/s]  Magnitude of wind velocity (in lateral inertial plane)*/
 float WindDir; /*< [rad]  Wind heading angle from North*/
 float WindZ; /*< [m/s]  Z (Down) component of inertial wind velocity*/
 float Airspeed; /*< [m/s]  Magnitude of air velocity*/
 float beta; /*< [rad]  Sideslip angle*/
 float alpha; /*< [rad]  Angle of attack*/
} mavlink_ekf_ext_t;

#define MAVLINK_MSG_ID_EKF_EXT_LEN 32
#define MAVLINK_MSG_ID_EKF_EXT_MIN_LEN 32
#define MAVLINK_MSG_ID_8007_LEN 32
#define MAVLINK_MSG_ID_8007_MIN_LEN 32

#define MAVLINK_MSG_ID_EKF_EXT_CRC 64
#define MAVLINK_MSG_ID_8007_CRC 64



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_EKF_EXT { \
    8007, \
    "EKF_EXT", \
    7, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_ekf_ext_t, timestamp) }, \
         { "Windspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ekf_ext_t, Windspeed) }, \
         { "WindDir", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ekf_ext_t, WindDir) }, \
         { "WindZ", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_ekf_ext_t, WindZ) }, \
         { "Airspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_ekf_ext_t, Airspeed) }, \
         { "beta", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_ekf_ext_t, beta) }, \
         { "alpha", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_ekf_ext_t, alpha) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_EKF_EXT { \
    "EKF_EXT", \
    7, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_ekf_ext_t, timestamp) }, \
         { "Windspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ekf_ext_t, Windspeed) }, \
         { "WindDir", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ekf_ext_t, WindDir) }, \
         { "WindZ", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_ekf_ext_t, WindZ) }, \
         { "Airspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_ekf_ext_t, Airspeed) }, \
         { "beta", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_ekf_ext_t, beta) }, \
         { "alpha", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_ekf_ext_t, alpha) }, \
         } \
}
#endif

/**
 * @brief Pack a ekf_ext message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us]  Time since system start
 * @param Windspeed [m/s]  Magnitude of wind velocity (in lateral inertial plane)
 * @param WindDir [rad]  Wind heading angle from North
 * @param WindZ [m/s]  Z (Down) component of inertial wind velocity
 * @param Airspeed [m/s]  Magnitude of air velocity
 * @param beta [rad]  Sideslip angle
 * @param alpha [rad]  Angle of attack
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ekf_ext_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float Windspeed, float WindDir, float WindZ, float Airspeed, float beta, float alpha)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_EKF_EXT_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, Windspeed);
    _mav_put_float(buf, 12, WindDir);
    _mav_put_float(buf, 16, WindZ);
    _mav_put_float(buf, 20, Airspeed);
    _mav_put_float(buf, 24, beta);
    _mav_put_float(buf, 28, alpha);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_EKF_EXT_LEN);
#else
    mavlink_ekf_ext_t packet;
    packet.timestamp = timestamp;
    packet.Windspeed = Windspeed;
    packet.WindDir = WindDir;
    packet.WindZ = WindZ;
    packet.Airspeed = Airspeed;
    packet.beta = beta;
    packet.alpha = alpha;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_EKF_EXT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_EKF_EXT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_EKF_EXT_MIN_LEN, MAVLINK_MSG_ID_EKF_EXT_LEN, MAVLINK_MSG_ID_EKF_EXT_CRC);
}

/**
 * @brief Pack a ekf_ext message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us]  Time since system start
 * @param Windspeed [m/s]  Magnitude of wind velocity (in lateral inertial plane)
 * @param WindDir [rad]  Wind heading angle from North
 * @param WindZ [m/s]  Z (Down) component of inertial wind velocity
 * @param Airspeed [m/s]  Magnitude of air velocity
 * @param beta [rad]  Sideslip angle
 * @param alpha [rad]  Angle of attack
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ekf_ext_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t timestamp, float Windspeed, float WindDir, float WindZ, float Airspeed, float beta, float alpha)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_EKF_EXT_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, Windspeed);
    _mav_put_float(buf, 12, WindDir);
    _mav_put_float(buf, 16, WindZ);
    _mav_put_float(buf, 20, Airspeed);
    _mav_put_float(buf, 24, beta);
    _mav_put_float(buf, 28, alpha);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_EKF_EXT_LEN);
#else
    mavlink_ekf_ext_t packet;
    packet.timestamp = timestamp;
    packet.Windspeed = Windspeed;
    packet.WindDir = WindDir;
    packet.WindZ = WindZ;
    packet.Airspeed = Airspeed;
    packet.beta = beta;
    packet.alpha = alpha;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_EKF_EXT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_EKF_EXT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_EKF_EXT_MIN_LEN, MAVLINK_MSG_ID_EKF_EXT_LEN, MAVLINK_MSG_ID_EKF_EXT_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_EKF_EXT_MIN_LEN, MAVLINK_MSG_ID_EKF_EXT_LEN);
#endif
}

/**
 * @brief Pack a ekf_ext message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us]  Time since system start
 * @param Windspeed [m/s]  Magnitude of wind velocity (in lateral inertial plane)
 * @param WindDir [rad]  Wind heading angle from North
 * @param WindZ [m/s]  Z (Down) component of inertial wind velocity
 * @param Airspeed [m/s]  Magnitude of air velocity
 * @param beta [rad]  Sideslip angle
 * @param alpha [rad]  Angle of attack
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ekf_ext_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float Windspeed,float WindDir,float WindZ,float Airspeed,float beta,float alpha)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_EKF_EXT_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, Windspeed);
    _mav_put_float(buf, 12, WindDir);
    _mav_put_float(buf, 16, WindZ);
    _mav_put_float(buf, 20, Airspeed);
    _mav_put_float(buf, 24, beta);
    _mav_put_float(buf, 28, alpha);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_EKF_EXT_LEN);
#else
    mavlink_ekf_ext_t packet;
    packet.timestamp = timestamp;
    packet.Windspeed = Windspeed;
    packet.WindDir = WindDir;
    packet.WindZ = WindZ;
    packet.Airspeed = Airspeed;
    packet.beta = beta;
    packet.alpha = alpha;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_EKF_EXT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_EKF_EXT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_EKF_EXT_MIN_LEN, MAVLINK_MSG_ID_EKF_EXT_LEN, MAVLINK_MSG_ID_EKF_EXT_CRC);
}

/**
 * @brief Encode a ekf_ext struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ekf_ext C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ekf_ext_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ekf_ext_t* ekf_ext)
{
    return mavlink_msg_ekf_ext_pack(system_id, component_id, msg, ekf_ext->timestamp, ekf_ext->Windspeed, ekf_ext->WindDir, ekf_ext->WindZ, ekf_ext->Airspeed, ekf_ext->beta, ekf_ext->alpha);
}

/**
 * @brief Encode a ekf_ext struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ekf_ext C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ekf_ext_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ekf_ext_t* ekf_ext)
{
    return mavlink_msg_ekf_ext_pack_chan(system_id, component_id, chan, msg, ekf_ext->timestamp, ekf_ext->Windspeed, ekf_ext->WindDir, ekf_ext->WindZ, ekf_ext->Airspeed, ekf_ext->beta, ekf_ext->alpha);
}

/**
 * @brief Encode a ekf_ext struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param ekf_ext C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ekf_ext_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_ekf_ext_t* ekf_ext)
{
    return mavlink_msg_ekf_ext_pack_status(system_id, component_id, _status, msg,  ekf_ext->timestamp, ekf_ext->Windspeed, ekf_ext->WindDir, ekf_ext->WindZ, ekf_ext->Airspeed, ekf_ext->beta, ekf_ext->alpha);
}

/**
 * @brief Send a ekf_ext message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us]  Time since system start
 * @param Windspeed [m/s]  Magnitude of wind velocity (in lateral inertial plane)
 * @param WindDir [rad]  Wind heading angle from North
 * @param WindZ [m/s]  Z (Down) component of inertial wind velocity
 * @param Airspeed [m/s]  Magnitude of air velocity
 * @param beta [rad]  Sideslip angle
 * @param alpha [rad]  Angle of attack
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ekf_ext_send(mavlink_channel_t chan, uint64_t timestamp, float Windspeed, float WindDir, float WindZ, float Airspeed, float beta, float alpha)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_EKF_EXT_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, Windspeed);
    _mav_put_float(buf, 12, WindDir);
    _mav_put_float(buf, 16, WindZ);
    _mav_put_float(buf, 20, Airspeed);
    _mav_put_float(buf, 24, beta);
    _mav_put_float(buf, 28, alpha);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EKF_EXT, buf, MAVLINK_MSG_ID_EKF_EXT_MIN_LEN, MAVLINK_MSG_ID_EKF_EXT_LEN, MAVLINK_MSG_ID_EKF_EXT_CRC);
#else
    mavlink_ekf_ext_t packet;
    packet.timestamp = timestamp;
    packet.Windspeed = Windspeed;
    packet.WindDir = WindDir;
    packet.WindZ = WindZ;
    packet.Airspeed = Airspeed;
    packet.beta = beta;
    packet.alpha = alpha;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EKF_EXT, (const char *)&packet, MAVLINK_MSG_ID_EKF_EXT_MIN_LEN, MAVLINK_MSG_ID_EKF_EXT_LEN, MAVLINK_MSG_ID_EKF_EXT_CRC);
#endif
}

/**
 * @brief Send a ekf_ext message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ekf_ext_send_struct(mavlink_channel_t chan, const mavlink_ekf_ext_t* ekf_ext)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ekf_ext_send(chan, ekf_ext->timestamp, ekf_ext->Windspeed, ekf_ext->WindDir, ekf_ext->WindZ, ekf_ext->Airspeed, ekf_ext->beta, ekf_ext->alpha);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EKF_EXT, (const char *)ekf_ext, MAVLINK_MSG_ID_EKF_EXT_MIN_LEN, MAVLINK_MSG_ID_EKF_EXT_LEN, MAVLINK_MSG_ID_EKF_EXT_CRC);
#endif
}

#if MAVLINK_MSG_ID_EKF_EXT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ekf_ext_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float Windspeed, float WindDir, float WindZ, float Airspeed, float beta, float alpha)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, Windspeed);
    _mav_put_float(buf, 12, WindDir);
    _mav_put_float(buf, 16, WindZ);
    _mav_put_float(buf, 20, Airspeed);
    _mav_put_float(buf, 24, beta);
    _mav_put_float(buf, 28, alpha);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EKF_EXT, buf, MAVLINK_MSG_ID_EKF_EXT_MIN_LEN, MAVLINK_MSG_ID_EKF_EXT_LEN, MAVLINK_MSG_ID_EKF_EXT_CRC);
#else
    mavlink_ekf_ext_t *packet = (mavlink_ekf_ext_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->Windspeed = Windspeed;
    packet->WindDir = WindDir;
    packet->WindZ = WindZ;
    packet->Airspeed = Airspeed;
    packet->beta = beta;
    packet->alpha = alpha;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EKF_EXT, (const char *)packet, MAVLINK_MSG_ID_EKF_EXT_MIN_LEN, MAVLINK_MSG_ID_EKF_EXT_LEN, MAVLINK_MSG_ID_EKF_EXT_CRC);
#endif
}
#endif

#endif

// MESSAGE EKF_EXT UNPACKING


/**
 * @brief Get field timestamp from ekf_ext message
 *
 * @return [us]  Time since system start
 */
static inline uint64_t mavlink_msg_ekf_ext_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field Windspeed from ekf_ext message
 *
 * @return [m/s]  Magnitude of wind velocity (in lateral inertial plane)
 */
static inline float mavlink_msg_ekf_ext_get_Windspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field WindDir from ekf_ext message
 *
 * @return [rad]  Wind heading angle from North
 */
static inline float mavlink_msg_ekf_ext_get_WindDir(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field WindZ from ekf_ext message
 *
 * @return [m/s]  Z (Down) component of inertial wind velocity
 */
static inline float mavlink_msg_ekf_ext_get_WindZ(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field Airspeed from ekf_ext message
 *
 * @return [m/s]  Magnitude of air velocity
 */
static inline float mavlink_msg_ekf_ext_get_Airspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field beta from ekf_ext message
 *
 * @return [rad]  Sideslip angle
 */
static inline float mavlink_msg_ekf_ext_get_beta(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field alpha from ekf_ext message
 *
 * @return [rad]  Angle of attack
 */
static inline float mavlink_msg_ekf_ext_get_alpha(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a ekf_ext message into a struct
 *
 * @param msg The message to decode
 * @param ekf_ext C-struct to decode the message contents into
 */
static inline void mavlink_msg_ekf_ext_decode(const mavlink_message_t* msg, mavlink_ekf_ext_t* ekf_ext)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    ekf_ext->timestamp = mavlink_msg_ekf_ext_get_timestamp(msg);
    ekf_ext->Windspeed = mavlink_msg_ekf_ext_get_Windspeed(msg);
    ekf_ext->WindDir = mavlink_msg_ekf_ext_get_WindDir(msg);
    ekf_ext->WindZ = mavlink_msg_ekf_ext_get_WindZ(msg);
    ekf_ext->Airspeed = mavlink_msg_ekf_ext_get_Airspeed(msg);
    ekf_ext->beta = mavlink_msg_ekf_ext_get_beta(msg);
    ekf_ext->alpha = mavlink_msg_ekf_ext_get_alpha(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_EKF_EXT_LEN? msg->len : MAVLINK_MSG_ID_EKF_EXT_LEN;
        memset(ekf_ext, 0, MAVLINK_MSG_ID_EKF_EXT_LEN);
    memcpy(ekf_ext, _MAV_PAYLOAD(msg), len);
#endif
}
