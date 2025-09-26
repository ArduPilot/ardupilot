#pragma once
// MESSAGE SENS_MPPT PACKING

#define MAVLINK_MSG_ID_SENS_MPPT 8003


typedef struct __mavlink_sens_mppt_t {
 uint64_t mppt_timestamp; /*< [us]  MPPT last timestamp */
 float mppt1_volt; /*< [V]  MPPT1 voltage */
 float mppt1_amp; /*< [A]  MPPT1 current */
 float mppt2_volt; /*< [V]  MPPT2 voltage */
 float mppt2_amp; /*< [A]  MPPT2 current */
 float mppt3_volt; /*< [V] MPPT3 voltage */
 float mppt3_amp; /*< [A]  MPPT3 current */
 uint16_t mppt1_pwm; /*< [us]  MPPT1 pwm */
 uint16_t mppt2_pwm; /*< [us]  MPPT2 pwm */
 uint16_t mppt3_pwm; /*< [us]  MPPT3 pwm */
 uint8_t mppt1_status; /*<   MPPT1 status */
 uint8_t mppt2_status; /*<   MPPT2 status */
 uint8_t mppt3_status; /*<   MPPT3 status */
} mavlink_sens_mppt_t;

#define MAVLINK_MSG_ID_SENS_MPPT_LEN 41
#define MAVLINK_MSG_ID_SENS_MPPT_MIN_LEN 41
#define MAVLINK_MSG_ID_8003_LEN 41
#define MAVLINK_MSG_ID_8003_MIN_LEN 41

#define MAVLINK_MSG_ID_SENS_MPPT_CRC 231
#define MAVLINK_MSG_ID_8003_CRC 231



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SENS_MPPT { \
    8003, \
    "SENS_MPPT", \
    13, \
    {  { "mppt_timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sens_mppt_t, mppt_timestamp) }, \
         { "mppt1_volt", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_sens_mppt_t, mppt1_volt) }, \
         { "mppt1_amp", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_sens_mppt_t, mppt1_amp) }, \
         { "mppt1_pwm", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_sens_mppt_t, mppt1_pwm) }, \
         { "mppt1_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_sens_mppt_t, mppt1_status) }, \
         { "mppt2_volt", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_sens_mppt_t, mppt2_volt) }, \
         { "mppt2_amp", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_sens_mppt_t, mppt2_amp) }, \
         { "mppt2_pwm", NULL, MAVLINK_TYPE_UINT16_T, 0, 34, offsetof(mavlink_sens_mppt_t, mppt2_pwm) }, \
         { "mppt2_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_sens_mppt_t, mppt2_status) }, \
         { "mppt3_volt", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_sens_mppt_t, mppt3_volt) }, \
         { "mppt3_amp", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_sens_mppt_t, mppt3_amp) }, \
         { "mppt3_pwm", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_sens_mppt_t, mppt3_pwm) }, \
         { "mppt3_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_sens_mppt_t, mppt3_status) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SENS_MPPT { \
    "SENS_MPPT", \
    13, \
    {  { "mppt_timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sens_mppt_t, mppt_timestamp) }, \
         { "mppt1_volt", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_sens_mppt_t, mppt1_volt) }, \
         { "mppt1_amp", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_sens_mppt_t, mppt1_amp) }, \
         { "mppt1_pwm", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_sens_mppt_t, mppt1_pwm) }, \
         { "mppt1_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_sens_mppt_t, mppt1_status) }, \
         { "mppt2_volt", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_sens_mppt_t, mppt2_volt) }, \
         { "mppt2_amp", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_sens_mppt_t, mppt2_amp) }, \
         { "mppt2_pwm", NULL, MAVLINK_TYPE_UINT16_T, 0, 34, offsetof(mavlink_sens_mppt_t, mppt2_pwm) }, \
         { "mppt2_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_sens_mppt_t, mppt2_status) }, \
         { "mppt3_volt", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_sens_mppt_t, mppt3_volt) }, \
         { "mppt3_amp", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_sens_mppt_t, mppt3_amp) }, \
         { "mppt3_pwm", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_sens_mppt_t, mppt3_pwm) }, \
         { "mppt3_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_sens_mppt_t, mppt3_status) }, \
         } \
}
#endif

/**
 * @brief Pack a sens_mppt message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param mppt_timestamp [us]  MPPT last timestamp 
 * @param mppt1_volt [V]  MPPT1 voltage 
 * @param mppt1_amp [A]  MPPT1 current 
 * @param mppt1_pwm [us]  MPPT1 pwm 
 * @param mppt1_status   MPPT1 status 
 * @param mppt2_volt [V]  MPPT2 voltage 
 * @param mppt2_amp [A]  MPPT2 current 
 * @param mppt2_pwm [us]  MPPT2 pwm 
 * @param mppt2_status   MPPT2 status 
 * @param mppt3_volt [V] MPPT3 voltage 
 * @param mppt3_amp [A]  MPPT3 current 
 * @param mppt3_pwm [us]  MPPT3 pwm 
 * @param mppt3_status   MPPT3 status 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sens_mppt_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t mppt_timestamp, float mppt1_volt, float mppt1_amp, uint16_t mppt1_pwm, uint8_t mppt1_status, float mppt2_volt, float mppt2_amp, uint16_t mppt2_pwm, uint8_t mppt2_status, float mppt3_volt, float mppt3_amp, uint16_t mppt3_pwm, uint8_t mppt3_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENS_MPPT_LEN];
    _mav_put_uint64_t(buf, 0, mppt_timestamp);
    _mav_put_float(buf, 8, mppt1_volt);
    _mav_put_float(buf, 12, mppt1_amp);
    _mav_put_float(buf, 16, mppt2_volt);
    _mav_put_float(buf, 20, mppt2_amp);
    _mav_put_float(buf, 24, mppt3_volt);
    _mav_put_float(buf, 28, mppt3_amp);
    _mav_put_uint16_t(buf, 32, mppt1_pwm);
    _mav_put_uint16_t(buf, 34, mppt2_pwm);
    _mav_put_uint16_t(buf, 36, mppt3_pwm);
    _mav_put_uint8_t(buf, 38, mppt1_status);
    _mav_put_uint8_t(buf, 39, mppt2_status);
    _mav_put_uint8_t(buf, 40, mppt3_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENS_MPPT_LEN);
#else
    mavlink_sens_mppt_t packet;
    packet.mppt_timestamp = mppt_timestamp;
    packet.mppt1_volt = mppt1_volt;
    packet.mppt1_amp = mppt1_amp;
    packet.mppt2_volt = mppt2_volt;
    packet.mppt2_amp = mppt2_amp;
    packet.mppt3_volt = mppt3_volt;
    packet.mppt3_amp = mppt3_amp;
    packet.mppt1_pwm = mppt1_pwm;
    packet.mppt2_pwm = mppt2_pwm;
    packet.mppt3_pwm = mppt3_pwm;
    packet.mppt1_status = mppt1_status;
    packet.mppt2_status = mppt2_status;
    packet.mppt3_status = mppt3_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENS_MPPT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENS_MPPT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SENS_MPPT_MIN_LEN, MAVLINK_MSG_ID_SENS_MPPT_LEN, MAVLINK_MSG_ID_SENS_MPPT_CRC);
}

/**
 * @brief Pack a sens_mppt message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param mppt_timestamp [us]  MPPT last timestamp 
 * @param mppt1_volt [V]  MPPT1 voltage 
 * @param mppt1_amp [A]  MPPT1 current 
 * @param mppt1_pwm [us]  MPPT1 pwm 
 * @param mppt1_status   MPPT1 status 
 * @param mppt2_volt [V]  MPPT2 voltage 
 * @param mppt2_amp [A]  MPPT2 current 
 * @param mppt2_pwm [us]  MPPT2 pwm 
 * @param mppt2_status   MPPT2 status 
 * @param mppt3_volt [V] MPPT3 voltage 
 * @param mppt3_amp [A]  MPPT3 current 
 * @param mppt3_pwm [us]  MPPT3 pwm 
 * @param mppt3_status   MPPT3 status 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sens_mppt_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t mppt_timestamp, float mppt1_volt, float mppt1_amp, uint16_t mppt1_pwm, uint8_t mppt1_status, float mppt2_volt, float mppt2_amp, uint16_t mppt2_pwm, uint8_t mppt2_status, float mppt3_volt, float mppt3_amp, uint16_t mppt3_pwm, uint8_t mppt3_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENS_MPPT_LEN];
    _mav_put_uint64_t(buf, 0, mppt_timestamp);
    _mav_put_float(buf, 8, mppt1_volt);
    _mav_put_float(buf, 12, mppt1_amp);
    _mav_put_float(buf, 16, mppt2_volt);
    _mav_put_float(buf, 20, mppt2_amp);
    _mav_put_float(buf, 24, mppt3_volt);
    _mav_put_float(buf, 28, mppt3_amp);
    _mav_put_uint16_t(buf, 32, mppt1_pwm);
    _mav_put_uint16_t(buf, 34, mppt2_pwm);
    _mav_put_uint16_t(buf, 36, mppt3_pwm);
    _mav_put_uint8_t(buf, 38, mppt1_status);
    _mav_put_uint8_t(buf, 39, mppt2_status);
    _mav_put_uint8_t(buf, 40, mppt3_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENS_MPPT_LEN);
#else
    mavlink_sens_mppt_t packet;
    packet.mppt_timestamp = mppt_timestamp;
    packet.mppt1_volt = mppt1_volt;
    packet.mppt1_amp = mppt1_amp;
    packet.mppt2_volt = mppt2_volt;
    packet.mppt2_amp = mppt2_amp;
    packet.mppt3_volt = mppt3_volt;
    packet.mppt3_amp = mppt3_amp;
    packet.mppt1_pwm = mppt1_pwm;
    packet.mppt2_pwm = mppt2_pwm;
    packet.mppt3_pwm = mppt3_pwm;
    packet.mppt1_status = mppt1_status;
    packet.mppt2_status = mppt2_status;
    packet.mppt3_status = mppt3_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENS_MPPT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENS_MPPT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SENS_MPPT_MIN_LEN, MAVLINK_MSG_ID_SENS_MPPT_LEN, MAVLINK_MSG_ID_SENS_MPPT_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SENS_MPPT_MIN_LEN, MAVLINK_MSG_ID_SENS_MPPT_LEN);
#endif
}

/**
 * @brief Pack a sens_mppt message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mppt_timestamp [us]  MPPT last timestamp 
 * @param mppt1_volt [V]  MPPT1 voltage 
 * @param mppt1_amp [A]  MPPT1 current 
 * @param mppt1_pwm [us]  MPPT1 pwm 
 * @param mppt1_status   MPPT1 status 
 * @param mppt2_volt [V]  MPPT2 voltage 
 * @param mppt2_amp [A]  MPPT2 current 
 * @param mppt2_pwm [us]  MPPT2 pwm 
 * @param mppt2_status   MPPT2 status 
 * @param mppt3_volt [V] MPPT3 voltage 
 * @param mppt3_amp [A]  MPPT3 current 
 * @param mppt3_pwm [us]  MPPT3 pwm 
 * @param mppt3_status   MPPT3 status 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sens_mppt_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t mppt_timestamp,float mppt1_volt,float mppt1_amp,uint16_t mppt1_pwm,uint8_t mppt1_status,float mppt2_volt,float mppt2_amp,uint16_t mppt2_pwm,uint8_t mppt2_status,float mppt3_volt,float mppt3_amp,uint16_t mppt3_pwm,uint8_t mppt3_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENS_MPPT_LEN];
    _mav_put_uint64_t(buf, 0, mppt_timestamp);
    _mav_put_float(buf, 8, mppt1_volt);
    _mav_put_float(buf, 12, mppt1_amp);
    _mav_put_float(buf, 16, mppt2_volt);
    _mav_put_float(buf, 20, mppt2_amp);
    _mav_put_float(buf, 24, mppt3_volt);
    _mav_put_float(buf, 28, mppt3_amp);
    _mav_put_uint16_t(buf, 32, mppt1_pwm);
    _mav_put_uint16_t(buf, 34, mppt2_pwm);
    _mav_put_uint16_t(buf, 36, mppt3_pwm);
    _mav_put_uint8_t(buf, 38, mppt1_status);
    _mav_put_uint8_t(buf, 39, mppt2_status);
    _mav_put_uint8_t(buf, 40, mppt3_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENS_MPPT_LEN);
#else
    mavlink_sens_mppt_t packet;
    packet.mppt_timestamp = mppt_timestamp;
    packet.mppt1_volt = mppt1_volt;
    packet.mppt1_amp = mppt1_amp;
    packet.mppt2_volt = mppt2_volt;
    packet.mppt2_amp = mppt2_amp;
    packet.mppt3_volt = mppt3_volt;
    packet.mppt3_amp = mppt3_amp;
    packet.mppt1_pwm = mppt1_pwm;
    packet.mppt2_pwm = mppt2_pwm;
    packet.mppt3_pwm = mppt3_pwm;
    packet.mppt1_status = mppt1_status;
    packet.mppt2_status = mppt2_status;
    packet.mppt3_status = mppt3_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENS_MPPT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENS_MPPT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SENS_MPPT_MIN_LEN, MAVLINK_MSG_ID_SENS_MPPT_LEN, MAVLINK_MSG_ID_SENS_MPPT_CRC);
}

/**
 * @brief Encode a sens_mppt struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sens_mppt C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sens_mppt_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sens_mppt_t* sens_mppt)
{
    return mavlink_msg_sens_mppt_pack(system_id, component_id, msg, sens_mppt->mppt_timestamp, sens_mppt->mppt1_volt, sens_mppt->mppt1_amp, sens_mppt->mppt1_pwm, sens_mppt->mppt1_status, sens_mppt->mppt2_volt, sens_mppt->mppt2_amp, sens_mppt->mppt2_pwm, sens_mppt->mppt2_status, sens_mppt->mppt3_volt, sens_mppt->mppt3_amp, sens_mppt->mppt3_pwm, sens_mppt->mppt3_status);
}

/**
 * @brief Encode a sens_mppt struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sens_mppt C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sens_mppt_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sens_mppt_t* sens_mppt)
{
    return mavlink_msg_sens_mppt_pack_chan(system_id, component_id, chan, msg, sens_mppt->mppt_timestamp, sens_mppt->mppt1_volt, sens_mppt->mppt1_amp, sens_mppt->mppt1_pwm, sens_mppt->mppt1_status, sens_mppt->mppt2_volt, sens_mppt->mppt2_amp, sens_mppt->mppt2_pwm, sens_mppt->mppt2_status, sens_mppt->mppt3_volt, sens_mppt->mppt3_amp, sens_mppt->mppt3_pwm, sens_mppt->mppt3_status);
}

/**
 * @brief Encode a sens_mppt struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param sens_mppt C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sens_mppt_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_sens_mppt_t* sens_mppt)
{
    return mavlink_msg_sens_mppt_pack_status(system_id, component_id, _status, msg,  sens_mppt->mppt_timestamp, sens_mppt->mppt1_volt, sens_mppt->mppt1_amp, sens_mppt->mppt1_pwm, sens_mppt->mppt1_status, sens_mppt->mppt2_volt, sens_mppt->mppt2_amp, sens_mppt->mppt2_pwm, sens_mppt->mppt2_status, sens_mppt->mppt3_volt, sens_mppt->mppt3_amp, sens_mppt->mppt3_pwm, sens_mppt->mppt3_status);
}

/**
 * @brief Send a sens_mppt message
 * @param chan MAVLink channel to send the message
 *
 * @param mppt_timestamp [us]  MPPT last timestamp 
 * @param mppt1_volt [V]  MPPT1 voltage 
 * @param mppt1_amp [A]  MPPT1 current 
 * @param mppt1_pwm [us]  MPPT1 pwm 
 * @param mppt1_status   MPPT1 status 
 * @param mppt2_volt [V]  MPPT2 voltage 
 * @param mppt2_amp [A]  MPPT2 current 
 * @param mppt2_pwm [us]  MPPT2 pwm 
 * @param mppt2_status   MPPT2 status 
 * @param mppt3_volt [V] MPPT3 voltage 
 * @param mppt3_amp [A]  MPPT3 current 
 * @param mppt3_pwm [us]  MPPT3 pwm 
 * @param mppt3_status   MPPT3 status 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sens_mppt_send(mavlink_channel_t chan, uint64_t mppt_timestamp, float mppt1_volt, float mppt1_amp, uint16_t mppt1_pwm, uint8_t mppt1_status, float mppt2_volt, float mppt2_amp, uint16_t mppt2_pwm, uint8_t mppt2_status, float mppt3_volt, float mppt3_amp, uint16_t mppt3_pwm, uint8_t mppt3_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENS_MPPT_LEN];
    _mav_put_uint64_t(buf, 0, mppt_timestamp);
    _mav_put_float(buf, 8, mppt1_volt);
    _mav_put_float(buf, 12, mppt1_amp);
    _mav_put_float(buf, 16, mppt2_volt);
    _mav_put_float(buf, 20, mppt2_amp);
    _mav_put_float(buf, 24, mppt3_volt);
    _mav_put_float(buf, 28, mppt3_amp);
    _mav_put_uint16_t(buf, 32, mppt1_pwm);
    _mav_put_uint16_t(buf, 34, mppt2_pwm);
    _mav_put_uint16_t(buf, 36, mppt3_pwm);
    _mav_put_uint8_t(buf, 38, mppt1_status);
    _mav_put_uint8_t(buf, 39, mppt2_status);
    _mav_put_uint8_t(buf, 40, mppt3_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_MPPT, buf, MAVLINK_MSG_ID_SENS_MPPT_MIN_LEN, MAVLINK_MSG_ID_SENS_MPPT_LEN, MAVLINK_MSG_ID_SENS_MPPT_CRC);
#else
    mavlink_sens_mppt_t packet;
    packet.mppt_timestamp = mppt_timestamp;
    packet.mppt1_volt = mppt1_volt;
    packet.mppt1_amp = mppt1_amp;
    packet.mppt2_volt = mppt2_volt;
    packet.mppt2_amp = mppt2_amp;
    packet.mppt3_volt = mppt3_volt;
    packet.mppt3_amp = mppt3_amp;
    packet.mppt1_pwm = mppt1_pwm;
    packet.mppt2_pwm = mppt2_pwm;
    packet.mppt3_pwm = mppt3_pwm;
    packet.mppt1_status = mppt1_status;
    packet.mppt2_status = mppt2_status;
    packet.mppt3_status = mppt3_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_MPPT, (const char *)&packet, MAVLINK_MSG_ID_SENS_MPPT_MIN_LEN, MAVLINK_MSG_ID_SENS_MPPT_LEN, MAVLINK_MSG_ID_SENS_MPPT_CRC);
#endif
}

/**
 * @brief Send a sens_mppt message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sens_mppt_send_struct(mavlink_channel_t chan, const mavlink_sens_mppt_t* sens_mppt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sens_mppt_send(chan, sens_mppt->mppt_timestamp, sens_mppt->mppt1_volt, sens_mppt->mppt1_amp, sens_mppt->mppt1_pwm, sens_mppt->mppt1_status, sens_mppt->mppt2_volt, sens_mppt->mppt2_amp, sens_mppt->mppt2_pwm, sens_mppt->mppt2_status, sens_mppt->mppt3_volt, sens_mppt->mppt3_amp, sens_mppt->mppt3_pwm, sens_mppt->mppt3_status);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_MPPT, (const char *)sens_mppt, MAVLINK_MSG_ID_SENS_MPPT_MIN_LEN, MAVLINK_MSG_ID_SENS_MPPT_LEN, MAVLINK_MSG_ID_SENS_MPPT_CRC);
#endif
}

#if MAVLINK_MSG_ID_SENS_MPPT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sens_mppt_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t mppt_timestamp, float mppt1_volt, float mppt1_amp, uint16_t mppt1_pwm, uint8_t mppt1_status, float mppt2_volt, float mppt2_amp, uint16_t mppt2_pwm, uint8_t mppt2_status, float mppt3_volt, float mppt3_amp, uint16_t mppt3_pwm, uint8_t mppt3_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, mppt_timestamp);
    _mav_put_float(buf, 8, mppt1_volt);
    _mav_put_float(buf, 12, mppt1_amp);
    _mav_put_float(buf, 16, mppt2_volt);
    _mav_put_float(buf, 20, mppt2_amp);
    _mav_put_float(buf, 24, mppt3_volt);
    _mav_put_float(buf, 28, mppt3_amp);
    _mav_put_uint16_t(buf, 32, mppt1_pwm);
    _mav_put_uint16_t(buf, 34, mppt2_pwm);
    _mav_put_uint16_t(buf, 36, mppt3_pwm);
    _mav_put_uint8_t(buf, 38, mppt1_status);
    _mav_put_uint8_t(buf, 39, mppt2_status);
    _mav_put_uint8_t(buf, 40, mppt3_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_MPPT, buf, MAVLINK_MSG_ID_SENS_MPPT_MIN_LEN, MAVLINK_MSG_ID_SENS_MPPT_LEN, MAVLINK_MSG_ID_SENS_MPPT_CRC);
#else
    mavlink_sens_mppt_t *packet = (mavlink_sens_mppt_t *)msgbuf;
    packet->mppt_timestamp = mppt_timestamp;
    packet->mppt1_volt = mppt1_volt;
    packet->mppt1_amp = mppt1_amp;
    packet->mppt2_volt = mppt2_volt;
    packet->mppt2_amp = mppt2_amp;
    packet->mppt3_volt = mppt3_volt;
    packet->mppt3_amp = mppt3_amp;
    packet->mppt1_pwm = mppt1_pwm;
    packet->mppt2_pwm = mppt2_pwm;
    packet->mppt3_pwm = mppt3_pwm;
    packet->mppt1_status = mppt1_status;
    packet->mppt2_status = mppt2_status;
    packet->mppt3_status = mppt3_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_MPPT, (const char *)packet, MAVLINK_MSG_ID_SENS_MPPT_MIN_LEN, MAVLINK_MSG_ID_SENS_MPPT_LEN, MAVLINK_MSG_ID_SENS_MPPT_CRC);
#endif
}
#endif

#endif

// MESSAGE SENS_MPPT UNPACKING


/**
 * @brief Get field mppt_timestamp from sens_mppt message
 *
 * @return [us]  MPPT last timestamp 
 */
static inline uint64_t mavlink_msg_sens_mppt_get_mppt_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field mppt1_volt from sens_mppt message
 *
 * @return [V]  MPPT1 voltage 
 */
static inline float mavlink_msg_sens_mppt_get_mppt1_volt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field mppt1_amp from sens_mppt message
 *
 * @return [A]  MPPT1 current 
 */
static inline float mavlink_msg_sens_mppt_get_mppt1_amp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field mppt1_pwm from sens_mppt message
 *
 * @return [us]  MPPT1 pwm 
 */
static inline uint16_t mavlink_msg_sens_mppt_get_mppt1_pwm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  32);
}

/**
 * @brief Get field mppt1_status from sens_mppt message
 *
 * @return   MPPT1 status 
 */
static inline uint8_t mavlink_msg_sens_mppt_get_mppt1_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  38);
}

/**
 * @brief Get field mppt2_volt from sens_mppt message
 *
 * @return [V]  MPPT2 voltage 
 */
static inline float mavlink_msg_sens_mppt_get_mppt2_volt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field mppt2_amp from sens_mppt message
 *
 * @return [A]  MPPT2 current 
 */
static inline float mavlink_msg_sens_mppt_get_mppt2_amp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field mppt2_pwm from sens_mppt message
 *
 * @return [us]  MPPT2 pwm 
 */
static inline uint16_t mavlink_msg_sens_mppt_get_mppt2_pwm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  34);
}

/**
 * @brief Get field mppt2_status from sens_mppt message
 *
 * @return   MPPT2 status 
 */
static inline uint8_t mavlink_msg_sens_mppt_get_mppt2_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  39);
}

/**
 * @brief Get field mppt3_volt from sens_mppt message
 *
 * @return [V] MPPT3 voltage 
 */
static inline float mavlink_msg_sens_mppt_get_mppt3_volt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field mppt3_amp from sens_mppt message
 *
 * @return [A]  MPPT3 current 
 */
static inline float mavlink_msg_sens_mppt_get_mppt3_amp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field mppt3_pwm from sens_mppt message
 *
 * @return [us]  MPPT3 pwm 
 */
static inline uint16_t mavlink_msg_sens_mppt_get_mppt3_pwm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  36);
}

/**
 * @brief Get field mppt3_status from sens_mppt message
 *
 * @return   MPPT3 status 
 */
static inline uint8_t mavlink_msg_sens_mppt_get_mppt3_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  40);
}

/**
 * @brief Decode a sens_mppt message into a struct
 *
 * @param msg The message to decode
 * @param sens_mppt C-struct to decode the message contents into
 */
static inline void mavlink_msg_sens_mppt_decode(const mavlink_message_t* msg, mavlink_sens_mppt_t* sens_mppt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sens_mppt->mppt_timestamp = mavlink_msg_sens_mppt_get_mppt_timestamp(msg);
    sens_mppt->mppt1_volt = mavlink_msg_sens_mppt_get_mppt1_volt(msg);
    sens_mppt->mppt1_amp = mavlink_msg_sens_mppt_get_mppt1_amp(msg);
    sens_mppt->mppt2_volt = mavlink_msg_sens_mppt_get_mppt2_volt(msg);
    sens_mppt->mppt2_amp = mavlink_msg_sens_mppt_get_mppt2_amp(msg);
    sens_mppt->mppt3_volt = mavlink_msg_sens_mppt_get_mppt3_volt(msg);
    sens_mppt->mppt3_amp = mavlink_msg_sens_mppt_get_mppt3_amp(msg);
    sens_mppt->mppt1_pwm = mavlink_msg_sens_mppt_get_mppt1_pwm(msg);
    sens_mppt->mppt2_pwm = mavlink_msg_sens_mppt_get_mppt2_pwm(msg);
    sens_mppt->mppt3_pwm = mavlink_msg_sens_mppt_get_mppt3_pwm(msg);
    sens_mppt->mppt1_status = mavlink_msg_sens_mppt_get_mppt1_status(msg);
    sens_mppt->mppt2_status = mavlink_msg_sens_mppt_get_mppt2_status(msg);
    sens_mppt->mppt3_status = mavlink_msg_sens_mppt_get_mppt3_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SENS_MPPT_LEN? msg->len : MAVLINK_MSG_ID_SENS_MPPT_LEN;
        memset(sens_mppt, 0, MAVLINK_MSG_ID_SENS_MPPT_LEN);
    memcpy(sens_mppt, _MAV_PAYLOAD(msg), len);
#endif
}
