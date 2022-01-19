#pragma once
// MESSAGE MAG_CAL_PROGRESS PACKING

#define MAVLINK_MSG_ID_MAG_CAL_PROGRESS 191


typedef struct __mavlink_mag_cal_progress_t {
 float direction_x; /*<  Body frame direction vector for display.*/
 float direction_y; /*<  Body frame direction vector for display.*/
 float direction_z; /*<  Body frame direction vector for display.*/
 uint8_t compass_id; /*<  Compass being calibrated.*/
 uint8_t cal_mask; /*<  Bitmask of compasses being calibrated.*/
 uint8_t cal_status; /*<  Calibration Status.*/
 uint8_t attempt; /*<  Attempt number.*/
 uint8_t completion_pct; /*< [%] Completion percentage.*/
 uint8_t completion_mask[10]; /*<  Bitmask of sphere sections (see http://en.wikipedia.org/wiki/Geodesic_grid).*/
} mavlink_mag_cal_progress_t;

#define MAVLINK_MSG_ID_MAG_CAL_PROGRESS_LEN 27
#define MAVLINK_MSG_ID_MAG_CAL_PROGRESS_MIN_LEN 27
#define MAVLINK_MSG_ID_191_LEN 27
#define MAVLINK_MSG_ID_191_MIN_LEN 27

#define MAVLINK_MSG_ID_MAG_CAL_PROGRESS_CRC 92
#define MAVLINK_MSG_ID_191_CRC 92

#define MAVLINK_MSG_MAG_CAL_PROGRESS_FIELD_COMPLETION_MASK_LEN 10

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MAG_CAL_PROGRESS { \
    191, \
    "MAG_CAL_PROGRESS", \
    9, \
    {  { "compass_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_mag_cal_progress_t, compass_id) }, \
         { "cal_mask", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_mag_cal_progress_t, cal_mask) }, \
         { "cal_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_mag_cal_progress_t, cal_status) }, \
         { "attempt", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_mag_cal_progress_t, attempt) }, \
         { "completion_pct", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_mag_cal_progress_t, completion_pct) }, \
         { "completion_mask", NULL, MAVLINK_TYPE_UINT8_T, 10, 17, offsetof(mavlink_mag_cal_progress_t, completion_mask) }, \
         { "direction_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_mag_cal_progress_t, direction_x) }, \
         { "direction_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_mag_cal_progress_t, direction_y) }, \
         { "direction_z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_mag_cal_progress_t, direction_z) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MAG_CAL_PROGRESS { \
    "MAG_CAL_PROGRESS", \
    9, \
    {  { "compass_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_mag_cal_progress_t, compass_id) }, \
         { "cal_mask", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_mag_cal_progress_t, cal_mask) }, \
         { "cal_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_mag_cal_progress_t, cal_status) }, \
         { "attempt", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_mag_cal_progress_t, attempt) }, \
         { "completion_pct", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_mag_cal_progress_t, completion_pct) }, \
         { "completion_mask", NULL, MAVLINK_TYPE_UINT8_T, 10, 17, offsetof(mavlink_mag_cal_progress_t, completion_mask) }, \
         { "direction_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_mag_cal_progress_t, direction_x) }, \
         { "direction_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_mag_cal_progress_t, direction_y) }, \
         { "direction_z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_mag_cal_progress_t, direction_z) }, \
         } \
}
#endif

/**
 * @brief Pack a mag_cal_progress message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param compass_id  Compass being calibrated.
 * @param cal_mask  Bitmask of compasses being calibrated.
 * @param cal_status  Calibration Status.
 * @param attempt  Attempt number.
 * @param completion_pct [%] Completion percentage.
 * @param completion_mask  Bitmask of sphere sections (see http://en.wikipedia.org/wiki/Geodesic_grid).
 * @param direction_x  Body frame direction vector for display.
 * @param direction_y  Body frame direction vector for display.
 * @param direction_z  Body frame direction vector for display.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mag_cal_progress_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t compass_id, uint8_t cal_mask, uint8_t cal_status, uint8_t attempt, uint8_t completion_pct, const uint8_t *completion_mask, float direction_x, float direction_y, float direction_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAG_CAL_PROGRESS_LEN];
    _mav_put_float(buf, 0, direction_x);
    _mav_put_float(buf, 4, direction_y);
    _mav_put_float(buf, 8, direction_z);
    _mav_put_uint8_t(buf, 12, compass_id);
    _mav_put_uint8_t(buf, 13, cal_mask);
    _mav_put_uint8_t(buf, 14, cal_status);
    _mav_put_uint8_t(buf, 15, attempt);
    _mav_put_uint8_t(buf, 16, completion_pct);
    _mav_put_uint8_t_array(buf, 17, completion_mask, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAG_CAL_PROGRESS_LEN);
#else
    mavlink_mag_cal_progress_t packet;
    packet.direction_x = direction_x;
    packet.direction_y = direction_y;
    packet.direction_z = direction_z;
    packet.compass_id = compass_id;
    packet.cal_mask = cal_mask;
    packet.cal_status = cal_status;
    packet.attempt = attempt;
    packet.completion_pct = completion_pct;
    mav_array_memcpy(packet.completion_mask, completion_mask, sizeof(uint8_t)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAG_CAL_PROGRESS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAG_CAL_PROGRESS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAG_CAL_PROGRESS_MIN_LEN, MAVLINK_MSG_ID_MAG_CAL_PROGRESS_LEN, MAVLINK_MSG_ID_MAG_CAL_PROGRESS_CRC);
}

/**
 * @brief Pack a mag_cal_progress message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param compass_id  Compass being calibrated.
 * @param cal_mask  Bitmask of compasses being calibrated.
 * @param cal_status  Calibration Status.
 * @param attempt  Attempt number.
 * @param completion_pct [%] Completion percentage.
 * @param completion_mask  Bitmask of sphere sections (see http://en.wikipedia.org/wiki/Geodesic_grid).
 * @param direction_x  Body frame direction vector for display.
 * @param direction_y  Body frame direction vector for display.
 * @param direction_z  Body frame direction vector for display.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mag_cal_progress_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t compass_id,uint8_t cal_mask,uint8_t cal_status,uint8_t attempt,uint8_t completion_pct,const uint8_t *completion_mask,float direction_x,float direction_y,float direction_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAG_CAL_PROGRESS_LEN];
    _mav_put_float(buf, 0, direction_x);
    _mav_put_float(buf, 4, direction_y);
    _mav_put_float(buf, 8, direction_z);
    _mav_put_uint8_t(buf, 12, compass_id);
    _mav_put_uint8_t(buf, 13, cal_mask);
    _mav_put_uint8_t(buf, 14, cal_status);
    _mav_put_uint8_t(buf, 15, attempt);
    _mav_put_uint8_t(buf, 16, completion_pct);
    _mav_put_uint8_t_array(buf, 17, completion_mask, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAG_CAL_PROGRESS_LEN);
#else
    mavlink_mag_cal_progress_t packet;
    packet.direction_x = direction_x;
    packet.direction_y = direction_y;
    packet.direction_z = direction_z;
    packet.compass_id = compass_id;
    packet.cal_mask = cal_mask;
    packet.cal_status = cal_status;
    packet.attempt = attempt;
    packet.completion_pct = completion_pct;
    mav_array_memcpy(packet.completion_mask, completion_mask, sizeof(uint8_t)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAG_CAL_PROGRESS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAG_CAL_PROGRESS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAG_CAL_PROGRESS_MIN_LEN, MAVLINK_MSG_ID_MAG_CAL_PROGRESS_LEN, MAVLINK_MSG_ID_MAG_CAL_PROGRESS_CRC);
}

/**
 * @brief Encode a mag_cal_progress struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mag_cal_progress C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mag_cal_progress_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mag_cal_progress_t* mag_cal_progress)
{
    return mavlink_msg_mag_cal_progress_pack(system_id, component_id, msg, mag_cal_progress->compass_id, mag_cal_progress->cal_mask, mag_cal_progress->cal_status, mag_cal_progress->attempt, mag_cal_progress->completion_pct, mag_cal_progress->completion_mask, mag_cal_progress->direction_x, mag_cal_progress->direction_y, mag_cal_progress->direction_z);
}

/**
 * @brief Encode a mag_cal_progress struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mag_cal_progress C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mag_cal_progress_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mag_cal_progress_t* mag_cal_progress)
{
    return mavlink_msg_mag_cal_progress_pack_chan(system_id, component_id, chan, msg, mag_cal_progress->compass_id, mag_cal_progress->cal_mask, mag_cal_progress->cal_status, mag_cal_progress->attempt, mag_cal_progress->completion_pct, mag_cal_progress->completion_mask, mag_cal_progress->direction_x, mag_cal_progress->direction_y, mag_cal_progress->direction_z);
}

/**
 * @brief Send a mag_cal_progress message
 * @param chan MAVLink channel to send the message
 *
 * @param compass_id  Compass being calibrated.
 * @param cal_mask  Bitmask of compasses being calibrated.
 * @param cal_status  Calibration Status.
 * @param attempt  Attempt number.
 * @param completion_pct [%] Completion percentage.
 * @param completion_mask  Bitmask of sphere sections (see http://en.wikipedia.org/wiki/Geodesic_grid).
 * @param direction_x  Body frame direction vector for display.
 * @param direction_y  Body frame direction vector for display.
 * @param direction_z  Body frame direction vector for display.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mag_cal_progress_send(mavlink_channel_t chan, uint8_t compass_id, uint8_t cal_mask, uint8_t cal_status, uint8_t attempt, uint8_t completion_pct, const uint8_t *completion_mask, float direction_x, float direction_y, float direction_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAG_CAL_PROGRESS_LEN];
    _mav_put_float(buf, 0, direction_x);
    _mav_put_float(buf, 4, direction_y);
    _mav_put_float(buf, 8, direction_z);
    _mav_put_uint8_t(buf, 12, compass_id);
    _mav_put_uint8_t(buf, 13, cal_mask);
    _mav_put_uint8_t(buf, 14, cal_status);
    _mav_put_uint8_t(buf, 15, attempt);
    _mav_put_uint8_t(buf, 16, completion_pct);
    _mav_put_uint8_t_array(buf, 17, completion_mask, 10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAG_CAL_PROGRESS, buf, MAVLINK_MSG_ID_MAG_CAL_PROGRESS_MIN_LEN, MAVLINK_MSG_ID_MAG_CAL_PROGRESS_LEN, MAVLINK_MSG_ID_MAG_CAL_PROGRESS_CRC);
#else
    mavlink_mag_cal_progress_t packet;
    packet.direction_x = direction_x;
    packet.direction_y = direction_y;
    packet.direction_z = direction_z;
    packet.compass_id = compass_id;
    packet.cal_mask = cal_mask;
    packet.cal_status = cal_status;
    packet.attempt = attempt;
    packet.completion_pct = completion_pct;
    mav_array_memcpy(packet.completion_mask, completion_mask, sizeof(uint8_t)*10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAG_CAL_PROGRESS, (const char *)&packet, MAVLINK_MSG_ID_MAG_CAL_PROGRESS_MIN_LEN, MAVLINK_MSG_ID_MAG_CAL_PROGRESS_LEN, MAVLINK_MSG_ID_MAG_CAL_PROGRESS_CRC);
#endif
}

/**
 * @brief Send a mag_cal_progress message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mag_cal_progress_send_struct(mavlink_channel_t chan, const mavlink_mag_cal_progress_t* mag_cal_progress)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mag_cal_progress_send(chan, mag_cal_progress->compass_id, mag_cal_progress->cal_mask, mag_cal_progress->cal_status, mag_cal_progress->attempt, mag_cal_progress->completion_pct, mag_cal_progress->completion_mask, mag_cal_progress->direction_x, mag_cal_progress->direction_y, mag_cal_progress->direction_z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAG_CAL_PROGRESS, (const char *)mag_cal_progress, MAVLINK_MSG_ID_MAG_CAL_PROGRESS_MIN_LEN, MAVLINK_MSG_ID_MAG_CAL_PROGRESS_LEN, MAVLINK_MSG_ID_MAG_CAL_PROGRESS_CRC);
#endif
}

#if MAVLINK_MSG_ID_MAG_CAL_PROGRESS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mag_cal_progress_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t compass_id, uint8_t cal_mask, uint8_t cal_status, uint8_t attempt, uint8_t completion_pct, const uint8_t *completion_mask, float direction_x, float direction_y, float direction_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, direction_x);
    _mav_put_float(buf, 4, direction_y);
    _mav_put_float(buf, 8, direction_z);
    _mav_put_uint8_t(buf, 12, compass_id);
    _mav_put_uint8_t(buf, 13, cal_mask);
    _mav_put_uint8_t(buf, 14, cal_status);
    _mav_put_uint8_t(buf, 15, attempt);
    _mav_put_uint8_t(buf, 16, completion_pct);
    _mav_put_uint8_t_array(buf, 17, completion_mask, 10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAG_CAL_PROGRESS, buf, MAVLINK_MSG_ID_MAG_CAL_PROGRESS_MIN_LEN, MAVLINK_MSG_ID_MAG_CAL_PROGRESS_LEN, MAVLINK_MSG_ID_MAG_CAL_PROGRESS_CRC);
#else
    mavlink_mag_cal_progress_t *packet = (mavlink_mag_cal_progress_t *)msgbuf;
    packet->direction_x = direction_x;
    packet->direction_y = direction_y;
    packet->direction_z = direction_z;
    packet->compass_id = compass_id;
    packet->cal_mask = cal_mask;
    packet->cal_status = cal_status;
    packet->attempt = attempt;
    packet->completion_pct = completion_pct;
    mav_array_memcpy(packet->completion_mask, completion_mask, sizeof(uint8_t)*10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAG_CAL_PROGRESS, (const char *)packet, MAVLINK_MSG_ID_MAG_CAL_PROGRESS_MIN_LEN, MAVLINK_MSG_ID_MAG_CAL_PROGRESS_LEN, MAVLINK_MSG_ID_MAG_CAL_PROGRESS_CRC);
#endif
}
#endif

#endif

// MESSAGE MAG_CAL_PROGRESS UNPACKING


/**
 * @brief Get field compass_id from mag_cal_progress message
 *
 * @return  Compass being calibrated.
 */
static inline uint8_t mavlink_msg_mag_cal_progress_get_compass_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field cal_mask from mag_cal_progress message
 *
 * @return  Bitmask of compasses being calibrated.
 */
static inline uint8_t mavlink_msg_mag_cal_progress_get_cal_mask(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field cal_status from mag_cal_progress message
 *
 * @return  Calibration Status.
 */
static inline uint8_t mavlink_msg_mag_cal_progress_get_cal_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field attempt from mag_cal_progress message
 *
 * @return  Attempt number.
 */
static inline uint8_t mavlink_msg_mag_cal_progress_get_attempt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  15);
}

/**
 * @brief Get field completion_pct from mag_cal_progress message
 *
 * @return [%] Completion percentage.
 */
static inline uint8_t mavlink_msg_mag_cal_progress_get_completion_pct(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field completion_mask from mag_cal_progress message
 *
 * @return  Bitmask of sphere sections (see http://en.wikipedia.org/wiki/Geodesic_grid).
 */
static inline uint16_t mavlink_msg_mag_cal_progress_get_completion_mask(const mavlink_message_t* msg, uint8_t *completion_mask)
{
    return _MAV_RETURN_uint8_t_array(msg, completion_mask, 10,  17);
}

/**
 * @brief Get field direction_x from mag_cal_progress message
 *
 * @return  Body frame direction vector for display.
 */
static inline float mavlink_msg_mag_cal_progress_get_direction_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field direction_y from mag_cal_progress message
 *
 * @return  Body frame direction vector for display.
 */
static inline float mavlink_msg_mag_cal_progress_get_direction_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field direction_z from mag_cal_progress message
 *
 * @return  Body frame direction vector for display.
 */
static inline float mavlink_msg_mag_cal_progress_get_direction_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a mag_cal_progress message into a struct
 *
 * @param msg The message to decode
 * @param mag_cal_progress C-struct to decode the message contents into
 */
static inline void mavlink_msg_mag_cal_progress_decode(const mavlink_message_t* msg, mavlink_mag_cal_progress_t* mag_cal_progress)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mag_cal_progress->direction_x = mavlink_msg_mag_cal_progress_get_direction_x(msg);
    mag_cal_progress->direction_y = mavlink_msg_mag_cal_progress_get_direction_y(msg);
    mag_cal_progress->direction_z = mavlink_msg_mag_cal_progress_get_direction_z(msg);
    mag_cal_progress->compass_id = mavlink_msg_mag_cal_progress_get_compass_id(msg);
    mag_cal_progress->cal_mask = mavlink_msg_mag_cal_progress_get_cal_mask(msg);
    mag_cal_progress->cal_status = mavlink_msg_mag_cal_progress_get_cal_status(msg);
    mag_cal_progress->attempt = mavlink_msg_mag_cal_progress_get_attempt(msg);
    mag_cal_progress->completion_pct = mavlink_msg_mag_cal_progress_get_completion_pct(msg);
    mavlink_msg_mag_cal_progress_get_completion_mask(msg, mag_cal_progress->completion_mask);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MAG_CAL_PROGRESS_LEN? msg->len : MAVLINK_MSG_ID_MAG_CAL_PROGRESS_LEN;
        memset(mag_cal_progress, 0, MAVLINK_MSG_ID_MAG_CAL_PROGRESS_LEN);
    memcpy(mag_cal_progress, _MAV_PAYLOAD(msg), len);
#endif
}
