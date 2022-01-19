#pragma once
// MESSAGE AHRS PACKING

#define MAVLINK_MSG_ID_AHRS 163


typedef struct __mavlink_ahrs_t {
 float omegaIx; /*< [rad/s] X gyro drift estimate.*/
 float omegaIy; /*< [rad/s] Y gyro drift estimate.*/
 float omegaIz; /*< [rad/s] Z gyro drift estimate.*/
 float accel_weight; /*<  Average accel_weight.*/
 float renorm_val; /*<  Average renormalisation value.*/
 float error_rp; /*<  Average error_roll_pitch value.*/
 float error_yaw; /*<  Average error_yaw value.*/
} mavlink_ahrs_t;

#define MAVLINK_MSG_ID_AHRS_LEN 28
#define MAVLINK_MSG_ID_AHRS_MIN_LEN 28
#define MAVLINK_MSG_ID_163_LEN 28
#define MAVLINK_MSG_ID_163_MIN_LEN 28

#define MAVLINK_MSG_ID_AHRS_CRC 127
#define MAVLINK_MSG_ID_163_CRC 127



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AHRS { \
    163, \
    "AHRS", \
    7, \
    {  { "omegaIx", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_ahrs_t, omegaIx) }, \
         { "omegaIy", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_ahrs_t, omegaIy) }, \
         { "omegaIz", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ahrs_t, omegaIz) }, \
         { "accel_weight", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ahrs_t, accel_weight) }, \
         { "renorm_val", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_ahrs_t, renorm_val) }, \
         { "error_rp", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_ahrs_t, error_rp) }, \
         { "error_yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_ahrs_t, error_yaw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AHRS { \
    "AHRS", \
    7, \
    {  { "omegaIx", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_ahrs_t, omegaIx) }, \
         { "omegaIy", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_ahrs_t, omegaIy) }, \
         { "omegaIz", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ahrs_t, omegaIz) }, \
         { "accel_weight", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ahrs_t, accel_weight) }, \
         { "renorm_val", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_ahrs_t, renorm_val) }, \
         { "error_rp", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_ahrs_t, error_rp) }, \
         { "error_yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_ahrs_t, error_yaw) }, \
         } \
}
#endif

/**
 * @brief Pack a ahrs message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param omegaIx [rad/s] X gyro drift estimate.
 * @param omegaIy [rad/s] Y gyro drift estimate.
 * @param omegaIz [rad/s] Z gyro drift estimate.
 * @param accel_weight  Average accel_weight.
 * @param renorm_val  Average renormalisation value.
 * @param error_rp  Average error_roll_pitch value.
 * @param error_yaw  Average error_yaw value.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ahrs_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float omegaIx, float omegaIy, float omegaIz, float accel_weight, float renorm_val, float error_rp, float error_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AHRS_LEN];
    _mav_put_float(buf, 0, omegaIx);
    _mav_put_float(buf, 4, omegaIy);
    _mav_put_float(buf, 8, omegaIz);
    _mav_put_float(buf, 12, accel_weight);
    _mav_put_float(buf, 16, renorm_val);
    _mav_put_float(buf, 20, error_rp);
    _mav_put_float(buf, 24, error_yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AHRS_LEN);
#else
    mavlink_ahrs_t packet;
    packet.omegaIx = omegaIx;
    packet.omegaIy = omegaIy;
    packet.omegaIz = omegaIz;
    packet.accel_weight = accel_weight;
    packet.renorm_val = renorm_val;
    packet.error_rp = error_rp;
    packet.error_yaw = error_yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AHRS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AHRS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AHRS_MIN_LEN, MAVLINK_MSG_ID_AHRS_LEN, MAVLINK_MSG_ID_AHRS_CRC);
}

/**
 * @brief Pack a ahrs message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param omegaIx [rad/s] X gyro drift estimate.
 * @param omegaIy [rad/s] Y gyro drift estimate.
 * @param omegaIz [rad/s] Z gyro drift estimate.
 * @param accel_weight  Average accel_weight.
 * @param renorm_val  Average renormalisation value.
 * @param error_rp  Average error_roll_pitch value.
 * @param error_yaw  Average error_yaw value.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ahrs_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float omegaIx,float omegaIy,float omegaIz,float accel_weight,float renorm_val,float error_rp,float error_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AHRS_LEN];
    _mav_put_float(buf, 0, omegaIx);
    _mav_put_float(buf, 4, omegaIy);
    _mav_put_float(buf, 8, omegaIz);
    _mav_put_float(buf, 12, accel_weight);
    _mav_put_float(buf, 16, renorm_val);
    _mav_put_float(buf, 20, error_rp);
    _mav_put_float(buf, 24, error_yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AHRS_LEN);
#else
    mavlink_ahrs_t packet;
    packet.omegaIx = omegaIx;
    packet.omegaIy = omegaIy;
    packet.omegaIz = omegaIz;
    packet.accel_weight = accel_weight;
    packet.renorm_val = renorm_val;
    packet.error_rp = error_rp;
    packet.error_yaw = error_yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AHRS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AHRS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AHRS_MIN_LEN, MAVLINK_MSG_ID_AHRS_LEN, MAVLINK_MSG_ID_AHRS_CRC);
}

/**
 * @brief Encode a ahrs struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ahrs C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ahrs_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ahrs_t* ahrs)
{
    return mavlink_msg_ahrs_pack(system_id, component_id, msg, ahrs->omegaIx, ahrs->omegaIy, ahrs->omegaIz, ahrs->accel_weight, ahrs->renorm_val, ahrs->error_rp, ahrs->error_yaw);
}

/**
 * @brief Encode a ahrs struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ahrs C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ahrs_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ahrs_t* ahrs)
{
    return mavlink_msg_ahrs_pack_chan(system_id, component_id, chan, msg, ahrs->omegaIx, ahrs->omegaIy, ahrs->omegaIz, ahrs->accel_weight, ahrs->renorm_val, ahrs->error_rp, ahrs->error_yaw);
}

/**
 * @brief Send a ahrs message
 * @param chan MAVLink channel to send the message
 *
 * @param omegaIx [rad/s] X gyro drift estimate.
 * @param omegaIy [rad/s] Y gyro drift estimate.
 * @param omegaIz [rad/s] Z gyro drift estimate.
 * @param accel_weight  Average accel_weight.
 * @param renorm_val  Average renormalisation value.
 * @param error_rp  Average error_roll_pitch value.
 * @param error_yaw  Average error_yaw value.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ahrs_send(mavlink_channel_t chan, float omegaIx, float omegaIy, float omegaIz, float accel_weight, float renorm_val, float error_rp, float error_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AHRS_LEN];
    _mav_put_float(buf, 0, omegaIx);
    _mav_put_float(buf, 4, omegaIy);
    _mav_put_float(buf, 8, omegaIz);
    _mav_put_float(buf, 12, accel_weight);
    _mav_put_float(buf, 16, renorm_val);
    _mav_put_float(buf, 20, error_rp);
    _mav_put_float(buf, 24, error_yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AHRS, buf, MAVLINK_MSG_ID_AHRS_MIN_LEN, MAVLINK_MSG_ID_AHRS_LEN, MAVLINK_MSG_ID_AHRS_CRC);
#else
    mavlink_ahrs_t packet;
    packet.omegaIx = omegaIx;
    packet.omegaIy = omegaIy;
    packet.omegaIz = omegaIz;
    packet.accel_weight = accel_weight;
    packet.renorm_val = renorm_val;
    packet.error_rp = error_rp;
    packet.error_yaw = error_yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AHRS, (const char *)&packet, MAVLINK_MSG_ID_AHRS_MIN_LEN, MAVLINK_MSG_ID_AHRS_LEN, MAVLINK_MSG_ID_AHRS_CRC);
#endif
}

/**
 * @brief Send a ahrs message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ahrs_send_struct(mavlink_channel_t chan, const mavlink_ahrs_t* ahrs)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ahrs_send(chan, ahrs->omegaIx, ahrs->omegaIy, ahrs->omegaIz, ahrs->accel_weight, ahrs->renorm_val, ahrs->error_rp, ahrs->error_yaw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AHRS, (const char *)ahrs, MAVLINK_MSG_ID_AHRS_MIN_LEN, MAVLINK_MSG_ID_AHRS_LEN, MAVLINK_MSG_ID_AHRS_CRC);
#endif
}

#if MAVLINK_MSG_ID_AHRS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ahrs_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float omegaIx, float omegaIy, float omegaIz, float accel_weight, float renorm_val, float error_rp, float error_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, omegaIx);
    _mav_put_float(buf, 4, omegaIy);
    _mav_put_float(buf, 8, omegaIz);
    _mav_put_float(buf, 12, accel_weight);
    _mav_put_float(buf, 16, renorm_val);
    _mav_put_float(buf, 20, error_rp);
    _mav_put_float(buf, 24, error_yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AHRS, buf, MAVLINK_MSG_ID_AHRS_MIN_LEN, MAVLINK_MSG_ID_AHRS_LEN, MAVLINK_MSG_ID_AHRS_CRC);
#else
    mavlink_ahrs_t *packet = (mavlink_ahrs_t *)msgbuf;
    packet->omegaIx = omegaIx;
    packet->omegaIy = omegaIy;
    packet->omegaIz = omegaIz;
    packet->accel_weight = accel_weight;
    packet->renorm_val = renorm_val;
    packet->error_rp = error_rp;
    packet->error_yaw = error_yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AHRS, (const char *)packet, MAVLINK_MSG_ID_AHRS_MIN_LEN, MAVLINK_MSG_ID_AHRS_LEN, MAVLINK_MSG_ID_AHRS_CRC);
#endif
}
#endif

#endif

// MESSAGE AHRS UNPACKING


/**
 * @brief Get field omegaIx from ahrs message
 *
 * @return [rad/s] X gyro drift estimate.
 */
static inline float mavlink_msg_ahrs_get_omegaIx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field omegaIy from ahrs message
 *
 * @return [rad/s] Y gyro drift estimate.
 */
static inline float mavlink_msg_ahrs_get_omegaIy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field omegaIz from ahrs message
 *
 * @return [rad/s] Z gyro drift estimate.
 */
static inline float mavlink_msg_ahrs_get_omegaIz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field accel_weight from ahrs message
 *
 * @return  Average accel_weight.
 */
static inline float mavlink_msg_ahrs_get_accel_weight(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field renorm_val from ahrs message
 *
 * @return  Average renormalisation value.
 */
static inline float mavlink_msg_ahrs_get_renorm_val(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field error_rp from ahrs message
 *
 * @return  Average error_roll_pitch value.
 */
static inline float mavlink_msg_ahrs_get_error_rp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field error_yaw from ahrs message
 *
 * @return  Average error_yaw value.
 */
static inline float mavlink_msg_ahrs_get_error_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a ahrs message into a struct
 *
 * @param msg The message to decode
 * @param ahrs C-struct to decode the message contents into
 */
static inline void mavlink_msg_ahrs_decode(const mavlink_message_t* msg, mavlink_ahrs_t* ahrs)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    ahrs->omegaIx = mavlink_msg_ahrs_get_omegaIx(msg);
    ahrs->omegaIy = mavlink_msg_ahrs_get_omegaIy(msg);
    ahrs->omegaIz = mavlink_msg_ahrs_get_omegaIz(msg);
    ahrs->accel_weight = mavlink_msg_ahrs_get_accel_weight(msg);
    ahrs->renorm_val = mavlink_msg_ahrs_get_renorm_val(msg);
    ahrs->error_rp = mavlink_msg_ahrs_get_error_rp(msg);
    ahrs->error_yaw = mavlink_msg_ahrs_get_error_yaw(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AHRS_LEN? msg->len : MAVLINK_MSG_ID_AHRS_LEN;
        memset(ahrs, 0, MAVLINK_MSG_ID_AHRS_LEN);
    memcpy(ahrs, _MAV_PAYLOAD(msg), len);
#endif
}
