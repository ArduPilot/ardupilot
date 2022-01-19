#pragma once
// MESSAGE ESTIMATOR_STATUS PACKING

#define MAVLINK_MSG_ID_ESTIMATOR_STATUS 230


typedef struct __mavlink_estimator_status_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 float vel_ratio; /*<  Velocity innovation test ratio*/
 float pos_horiz_ratio; /*<  Horizontal position innovation test ratio*/
 float pos_vert_ratio; /*<  Vertical position innovation test ratio*/
 float mag_ratio; /*<  Magnetometer innovation test ratio*/
 float hagl_ratio; /*<  Height above terrain innovation test ratio*/
 float tas_ratio; /*<  True airspeed innovation test ratio*/
 float pos_horiz_accuracy; /*< [m] Horizontal position 1-STD accuracy relative to the EKF local origin*/
 float pos_vert_accuracy; /*< [m] Vertical position 1-STD accuracy relative to the EKF local origin*/
 uint16_t flags; /*<  Bitmap indicating which EKF outputs are valid.*/
} mavlink_estimator_status_t;

#define MAVLINK_MSG_ID_ESTIMATOR_STATUS_LEN 42
#define MAVLINK_MSG_ID_ESTIMATOR_STATUS_MIN_LEN 42
#define MAVLINK_MSG_ID_230_LEN 42
#define MAVLINK_MSG_ID_230_MIN_LEN 42

#define MAVLINK_MSG_ID_ESTIMATOR_STATUS_CRC 163
#define MAVLINK_MSG_ID_230_CRC 163



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ESTIMATOR_STATUS { \
    230, \
    "ESTIMATOR_STATUS", \
    10, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_estimator_status_t, time_usec) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 40, offsetof(mavlink_estimator_status_t, flags) }, \
         { "vel_ratio", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_estimator_status_t, vel_ratio) }, \
         { "pos_horiz_ratio", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_estimator_status_t, pos_horiz_ratio) }, \
         { "pos_vert_ratio", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_estimator_status_t, pos_vert_ratio) }, \
         { "mag_ratio", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_estimator_status_t, mag_ratio) }, \
         { "hagl_ratio", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_estimator_status_t, hagl_ratio) }, \
         { "tas_ratio", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_estimator_status_t, tas_ratio) }, \
         { "pos_horiz_accuracy", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_estimator_status_t, pos_horiz_accuracy) }, \
         { "pos_vert_accuracy", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_estimator_status_t, pos_vert_accuracy) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ESTIMATOR_STATUS { \
    "ESTIMATOR_STATUS", \
    10, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_estimator_status_t, time_usec) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 40, offsetof(mavlink_estimator_status_t, flags) }, \
         { "vel_ratio", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_estimator_status_t, vel_ratio) }, \
         { "pos_horiz_ratio", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_estimator_status_t, pos_horiz_ratio) }, \
         { "pos_vert_ratio", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_estimator_status_t, pos_vert_ratio) }, \
         { "mag_ratio", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_estimator_status_t, mag_ratio) }, \
         { "hagl_ratio", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_estimator_status_t, hagl_ratio) }, \
         { "tas_ratio", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_estimator_status_t, tas_ratio) }, \
         { "pos_horiz_accuracy", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_estimator_status_t, pos_horiz_accuracy) }, \
         { "pos_vert_accuracy", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_estimator_status_t, pos_vert_accuracy) }, \
         } \
}
#endif

/**
 * @brief Pack a estimator_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param flags  Bitmap indicating which EKF outputs are valid.
 * @param vel_ratio  Velocity innovation test ratio
 * @param pos_horiz_ratio  Horizontal position innovation test ratio
 * @param pos_vert_ratio  Vertical position innovation test ratio
 * @param mag_ratio  Magnetometer innovation test ratio
 * @param hagl_ratio  Height above terrain innovation test ratio
 * @param tas_ratio  True airspeed innovation test ratio
 * @param pos_horiz_accuracy [m] Horizontal position 1-STD accuracy relative to the EKF local origin
 * @param pos_vert_accuracy [m] Vertical position 1-STD accuracy relative to the EKF local origin
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_estimator_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, uint16_t flags, float vel_ratio, float pos_horiz_ratio, float pos_vert_ratio, float mag_ratio, float hagl_ratio, float tas_ratio, float pos_horiz_accuracy, float pos_vert_accuracy)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ESTIMATOR_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, vel_ratio);
    _mav_put_float(buf, 12, pos_horiz_ratio);
    _mav_put_float(buf, 16, pos_vert_ratio);
    _mav_put_float(buf, 20, mag_ratio);
    _mav_put_float(buf, 24, hagl_ratio);
    _mav_put_float(buf, 28, tas_ratio);
    _mav_put_float(buf, 32, pos_horiz_accuracy);
    _mav_put_float(buf, 36, pos_vert_accuracy);
    _mav_put_uint16_t(buf, 40, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ESTIMATOR_STATUS_LEN);
#else
    mavlink_estimator_status_t packet;
    packet.time_usec = time_usec;
    packet.vel_ratio = vel_ratio;
    packet.pos_horiz_ratio = pos_horiz_ratio;
    packet.pos_vert_ratio = pos_vert_ratio;
    packet.mag_ratio = mag_ratio;
    packet.hagl_ratio = hagl_ratio;
    packet.tas_ratio = tas_ratio;
    packet.pos_horiz_accuracy = pos_horiz_accuracy;
    packet.pos_vert_accuracy = pos_vert_accuracy;
    packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ESTIMATOR_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ESTIMATOR_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ESTIMATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ESTIMATOR_STATUS_LEN, MAVLINK_MSG_ID_ESTIMATOR_STATUS_CRC);
}

/**
 * @brief Pack a estimator_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param flags  Bitmap indicating which EKF outputs are valid.
 * @param vel_ratio  Velocity innovation test ratio
 * @param pos_horiz_ratio  Horizontal position innovation test ratio
 * @param pos_vert_ratio  Vertical position innovation test ratio
 * @param mag_ratio  Magnetometer innovation test ratio
 * @param hagl_ratio  Height above terrain innovation test ratio
 * @param tas_ratio  True airspeed innovation test ratio
 * @param pos_horiz_accuracy [m] Horizontal position 1-STD accuracy relative to the EKF local origin
 * @param pos_vert_accuracy [m] Vertical position 1-STD accuracy relative to the EKF local origin
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_estimator_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,uint16_t flags,float vel_ratio,float pos_horiz_ratio,float pos_vert_ratio,float mag_ratio,float hagl_ratio,float tas_ratio,float pos_horiz_accuracy,float pos_vert_accuracy)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ESTIMATOR_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, vel_ratio);
    _mav_put_float(buf, 12, pos_horiz_ratio);
    _mav_put_float(buf, 16, pos_vert_ratio);
    _mav_put_float(buf, 20, mag_ratio);
    _mav_put_float(buf, 24, hagl_ratio);
    _mav_put_float(buf, 28, tas_ratio);
    _mav_put_float(buf, 32, pos_horiz_accuracy);
    _mav_put_float(buf, 36, pos_vert_accuracy);
    _mav_put_uint16_t(buf, 40, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ESTIMATOR_STATUS_LEN);
#else
    mavlink_estimator_status_t packet;
    packet.time_usec = time_usec;
    packet.vel_ratio = vel_ratio;
    packet.pos_horiz_ratio = pos_horiz_ratio;
    packet.pos_vert_ratio = pos_vert_ratio;
    packet.mag_ratio = mag_ratio;
    packet.hagl_ratio = hagl_ratio;
    packet.tas_ratio = tas_ratio;
    packet.pos_horiz_accuracy = pos_horiz_accuracy;
    packet.pos_vert_accuracy = pos_vert_accuracy;
    packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ESTIMATOR_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ESTIMATOR_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ESTIMATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ESTIMATOR_STATUS_LEN, MAVLINK_MSG_ID_ESTIMATOR_STATUS_CRC);
}

/**
 * @brief Encode a estimator_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param estimator_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_estimator_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_estimator_status_t* estimator_status)
{
    return mavlink_msg_estimator_status_pack(system_id, component_id, msg, estimator_status->time_usec, estimator_status->flags, estimator_status->vel_ratio, estimator_status->pos_horiz_ratio, estimator_status->pos_vert_ratio, estimator_status->mag_ratio, estimator_status->hagl_ratio, estimator_status->tas_ratio, estimator_status->pos_horiz_accuracy, estimator_status->pos_vert_accuracy);
}

/**
 * @brief Encode a estimator_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param estimator_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_estimator_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_estimator_status_t* estimator_status)
{
    return mavlink_msg_estimator_status_pack_chan(system_id, component_id, chan, msg, estimator_status->time_usec, estimator_status->flags, estimator_status->vel_ratio, estimator_status->pos_horiz_ratio, estimator_status->pos_vert_ratio, estimator_status->mag_ratio, estimator_status->hagl_ratio, estimator_status->tas_ratio, estimator_status->pos_horiz_accuracy, estimator_status->pos_vert_accuracy);
}

/**
 * @brief Send a estimator_status message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param flags  Bitmap indicating which EKF outputs are valid.
 * @param vel_ratio  Velocity innovation test ratio
 * @param pos_horiz_ratio  Horizontal position innovation test ratio
 * @param pos_vert_ratio  Vertical position innovation test ratio
 * @param mag_ratio  Magnetometer innovation test ratio
 * @param hagl_ratio  Height above terrain innovation test ratio
 * @param tas_ratio  True airspeed innovation test ratio
 * @param pos_horiz_accuracy [m] Horizontal position 1-STD accuracy relative to the EKF local origin
 * @param pos_vert_accuracy [m] Vertical position 1-STD accuracy relative to the EKF local origin
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_estimator_status_send(mavlink_channel_t chan, uint64_t time_usec, uint16_t flags, float vel_ratio, float pos_horiz_ratio, float pos_vert_ratio, float mag_ratio, float hagl_ratio, float tas_ratio, float pos_horiz_accuracy, float pos_vert_accuracy)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ESTIMATOR_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, vel_ratio);
    _mav_put_float(buf, 12, pos_horiz_ratio);
    _mav_put_float(buf, 16, pos_vert_ratio);
    _mav_put_float(buf, 20, mag_ratio);
    _mav_put_float(buf, 24, hagl_ratio);
    _mav_put_float(buf, 28, tas_ratio);
    _mav_put_float(buf, 32, pos_horiz_accuracy);
    _mav_put_float(buf, 36, pos_vert_accuracy);
    _mav_put_uint16_t(buf, 40, flags);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESTIMATOR_STATUS, buf, MAVLINK_MSG_ID_ESTIMATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ESTIMATOR_STATUS_LEN, MAVLINK_MSG_ID_ESTIMATOR_STATUS_CRC);
#else
    mavlink_estimator_status_t packet;
    packet.time_usec = time_usec;
    packet.vel_ratio = vel_ratio;
    packet.pos_horiz_ratio = pos_horiz_ratio;
    packet.pos_vert_ratio = pos_vert_ratio;
    packet.mag_ratio = mag_ratio;
    packet.hagl_ratio = hagl_ratio;
    packet.tas_ratio = tas_ratio;
    packet.pos_horiz_accuracy = pos_horiz_accuracy;
    packet.pos_vert_accuracy = pos_vert_accuracy;
    packet.flags = flags;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESTIMATOR_STATUS, (const char *)&packet, MAVLINK_MSG_ID_ESTIMATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ESTIMATOR_STATUS_LEN, MAVLINK_MSG_ID_ESTIMATOR_STATUS_CRC);
#endif
}

/**
 * @brief Send a estimator_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_estimator_status_send_struct(mavlink_channel_t chan, const mavlink_estimator_status_t* estimator_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_estimator_status_send(chan, estimator_status->time_usec, estimator_status->flags, estimator_status->vel_ratio, estimator_status->pos_horiz_ratio, estimator_status->pos_vert_ratio, estimator_status->mag_ratio, estimator_status->hagl_ratio, estimator_status->tas_ratio, estimator_status->pos_horiz_accuracy, estimator_status->pos_vert_accuracy);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESTIMATOR_STATUS, (const char *)estimator_status, MAVLINK_MSG_ID_ESTIMATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ESTIMATOR_STATUS_LEN, MAVLINK_MSG_ID_ESTIMATOR_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_ESTIMATOR_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_estimator_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint16_t flags, float vel_ratio, float pos_horiz_ratio, float pos_vert_ratio, float mag_ratio, float hagl_ratio, float tas_ratio, float pos_horiz_accuracy, float pos_vert_accuracy)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, vel_ratio);
    _mav_put_float(buf, 12, pos_horiz_ratio);
    _mav_put_float(buf, 16, pos_vert_ratio);
    _mav_put_float(buf, 20, mag_ratio);
    _mav_put_float(buf, 24, hagl_ratio);
    _mav_put_float(buf, 28, tas_ratio);
    _mav_put_float(buf, 32, pos_horiz_accuracy);
    _mav_put_float(buf, 36, pos_vert_accuracy);
    _mav_put_uint16_t(buf, 40, flags);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESTIMATOR_STATUS, buf, MAVLINK_MSG_ID_ESTIMATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ESTIMATOR_STATUS_LEN, MAVLINK_MSG_ID_ESTIMATOR_STATUS_CRC);
#else
    mavlink_estimator_status_t *packet = (mavlink_estimator_status_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->vel_ratio = vel_ratio;
    packet->pos_horiz_ratio = pos_horiz_ratio;
    packet->pos_vert_ratio = pos_vert_ratio;
    packet->mag_ratio = mag_ratio;
    packet->hagl_ratio = hagl_ratio;
    packet->tas_ratio = tas_ratio;
    packet->pos_horiz_accuracy = pos_horiz_accuracy;
    packet->pos_vert_accuracy = pos_vert_accuracy;
    packet->flags = flags;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESTIMATOR_STATUS, (const char *)packet, MAVLINK_MSG_ID_ESTIMATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ESTIMATOR_STATUS_LEN, MAVLINK_MSG_ID_ESTIMATOR_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE ESTIMATOR_STATUS UNPACKING


/**
 * @brief Get field time_usec from estimator_status message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
static inline uint64_t mavlink_msg_estimator_status_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field flags from estimator_status message
 *
 * @return  Bitmap indicating which EKF outputs are valid.
 */
static inline uint16_t mavlink_msg_estimator_status_get_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  40);
}

/**
 * @brief Get field vel_ratio from estimator_status message
 *
 * @return  Velocity innovation test ratio
 */
static inline float mavlink_msg_estimator_status_get_vel_ratio(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field pos_horiz_ratio from estimator_status message
 *
 * @return  Horizontal position innovation test ratio
 */
static inline float mavlink_msg_estimator_status_get_pos_horiz_ratio(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field pos_vert_ratio from estimator_status message
 *
 * @return  Vertical position innovation test ratio
 */
static inline float mavlink_msg_estimator_status_get_pos_vert_ratio(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field mag_ratio from estimator_status message
 *
 * @return  Magnetometer innovation test ratio
 */
static inline float mavlink_msg_estimator_status_get_mag_ratio(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field hagl_ratio from estimator_status message
 *
 * @return  Height above terrain innovation test ratio
 */
static inline float mavlink_msg_estimator_status_get_hagl_ratio(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field tas_ratio from estimator_status message
 *
 * @return  True airspeed innovation test ratio
 */
static inline float mavlink_msg_estimator_status_get_tas_ratio(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field pos_horiz_accuracy from estimator_status message
 *
 * @return [m] Horizontal position 1-STD accuracy relative to the EKF local origin
 */
static inline float mavlink_msg_estimator_status_get_pos_horiz_accuracy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field pos_vert_accuracy from estimator_status message
 *
 * @return [m] Vertical position 1-STD accuracy relative to the EKF local origin
 */
static inline float mavlink_msg_estimator_status_get_pos_vert_accuracy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Decode a estimator_status message into a struct
 *
 * @param msg The message to decode
 * @param estimator_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_estimator_status_decode(const mavlink_message_t* msg, mavlink_estimator_status_t* estimator_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    estimator_status->time_usec = mavlink_msg_estimator_status_get_time_usec(msg);
    estimator_status->vel_ratio = mavlink_msg_estimator_status_get_vel_ratio(msg);
    estimator_status->pos_horiz_ratio = mavlink_msg_estimator_status_get_pos_horiz_ratio(msg);
    estimator_status->pos_vert_ratio = mavlink_msg_estimator_status_get_pos_vert_ratio(msg);
    estimator_status->mag_ratio = mavlink_msg_estimator_status_get_mag_ratio(msg);
    estimator_status->hagl_ratio = mavlink_msg_estimator_status_get_hagl_ratio(msg);
    estimator_status->tas_ratio = mavlink_msg_estimator_status_get_tas_ratio(msg);
    estimator_status->pos_horiz_accuracy = mavlink_msg_estimator_status_get_pos_horiz_accuracy(msg);
    estimator_status->pos_vert_accuracy = mavlink_msg_estimator_status_get_pos_vert_accuracy(msg);
    estimator_status->flags = mavlink_msg_estimator_status_get_flags(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ESTIMATOR_STATUS_LEN? msg->len : MAVLINK_MSG_ID_ESTIMATOR_STATUS_LEN;
        memset(estimator_status, 0, MAVLINK_MSG_ID_ESTIMATOR_STATUS_LEN);
    memcpy(estimator_status, _MAV_PAYLOAD(msg), len);
#endif
}
